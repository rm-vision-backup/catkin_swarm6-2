#!/usr/bin/env python3
"""Train a ten-aircraft damage-attention GCN for the independent SITL experiment."""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch

from core import components, distance
from pipeline import DAG_ROOT, digest
from sim_multi import SimCase

DIRECTORY = Path(__file__).resolve().parent
# 场景可用 DAGL_SCENARIO 覆盖（随机布局演示：训练/推理/仿真必须用同一文件）。
SCENARIO = Path(os.environ.get("DAGL_SCENARIO", DIRECTORY / "scenarios/10.sitl.json"))
SCHEMA = "dagl10-residual-v1"
SCALE = 8.0


def configure_dag(case: SimCase):
    os.chdir(DAG_ROOT)
    sys.path.insert(0, str(DAG_ROOT))
    import Configurations as config

    config.config_num_of_agents = 10
    config.config_dimension = 2
    config.config_communication_range = case.radius
    config.config_width = case.upper[0] - case.lower[0]
    config.config_length = case.upper[1] - case.lower[1]
    config.config_space_range = np.array((config.config_width, config.config_length))
    config.config_central_point = config.config_space_range / 2
    return config


def build_network(hidden: int):
    from DAGL_Algorithm.Bipartite_GCN import Bipartite_GCN_structure

    network = Bipartite_GCN_structure(nfeat=2, nhid=hidden, nclass=2, dropout=0.0)
    torch.nn.init.zeros_(network.gc8.weight)
    torch.nn.init.zeros_(network.gc8.bias)
    return network


def lattice_targets(positions, member_ids, min_separation, max_spacing):
    """环阵编队（等价"格阵聚合"，但避免路径汇聚）：按方位角排序把成员分配到以
    质心为圆心的等距圆环上，相邻弦距 ∈ (min_separation, max_spacing) 且保持原方位
    次序——径向聚合路径不交叉，同时机动的最近接近距离 ≈ 相邻弦距，天然满足间距门禁。"""
    count = len(member_ids)
    chord = min(max_spacing, max(min_separation + 1.5, 5.5))
    if count <= 2:
        radius_ring = chord
    else:
        radius_ring = chord / (2.0 * math.sin(math.pi / count))
    center = (sum(positions[uav][0] for uav in member_ids) / count,
              sum(positions[uav][1] for uav in member_ids) / count)
    order = sorted(member_ids,
                   key=lambda uav: math.atan2(positions[uav][1] - center[1],
                                              positions[uav][0] - center[0]))
    targets = {}
    for index, uav in enumerate(order):
        angle = 2.0 * math.pi * index / count
        targets[uav] = (center[0] + radius_ring * math.cos(angle),
                        center[1] + radius_ring * math.sin(angle))
    return targets


def make_teacher(case: SimCase, positions: dict[str, tuple[float, float]]):
    if case.initial_scattered and case.fault_ids:
        # 受损用例的样本处于"编队完成后"状态，用该阶段的判据（散布用例用出生判据）。
        case.validate_damage_state(positions)
    else:
        case.validate_layout(positions)
    survivor_positions = {uav: positions[uav] for uav in case.survivors}
    if case.initial_scattered and not case.fault_ids:
        # 编队（0 故障）：按严格半径（radius-edge_margin=7m）分组——散布布局在
        # 9m"稳健"口径下可能已被桥链连通，而 8m 实际通信图是断开的，这正是编队
        # 阶段要解决的状态。
        groups = components(survivor_positions, case.radius - case.edge_margin)
    else:
        groups = components(survivor_positions, case.radius + case.edge_margin)
    if len(groups) < 2:
        raise ValueError("teacher requires at least two survivor groups")
    if case.initial_scattered:
        # 散布起步（编队/重组）：目标是"全体同时机动到一个紧凑连通格阵"——每机
        # 去自己的格位（对模型而言 = 每个孤立分量的一次平移）。格距 5.5 m：
        # >min_separation(4) 保证间距，<= radius-edge_margin-0.5 保证 6m 口径连通。
        member_ids = tuple(uav for uav in case.uav_ids if uav not in case.fault_ids)
        targets = lattice_targets(positions, member_ids, case.min_separation,
                                  case.radius - case.edge_margin - 0.5)
        if case.fault_ids:
            case.validate_targets({uav: positions[uav] for uav in member_ids}, targets,
                                  {uav: positions[uav] for uav in case.fault_ids},
                                  batches=(member_ids,))
        else:
            case.validate_assembly({uav: positions[uav] for uav in case.uav_ids}, targets)
        return targets, None, groups
    if len(groups) > 2:
        raise ValueError("teacher supports either two groups or a scattered layout")
    first, second = groups
    bridge = min(((left, right) for left in first for right in second),
                 key=lambda pair: (distance(positions[pair[0]], positions[pair[1]]), pair))
    start_distance = distance(positions[bridge[0]], positions[bridge[1]])
    direction = ((positions[bridge[1]][0] - positions[bridge[0]][0]) / start_distance,
                 (positions[bridge[1]][1] - positions[bridge[0]][1]) / start_distance)
    shift = (start_distance - (case.radius - case.edge_margin - 1.0)) / 2
    if shift <= 0:
        raise ValueError("teacher bridge was not broken")
    targets = {uav: (positions[uav][0] + shift * direction[0],
                     positions[uav][1] + shift * direction[1]) for uav in first}
    targets.update({uav: (positions[uav][0] - shift * direction[0],
                          positions[uav][1] - shift * direction[1]) for uav in second})
    case.validate_targets({uav: positions[uav] for uav in case.survivors}, targets,
                          {uav: positions[uav] for uav in case.fault_ids})
    return targets, bridge, groups


def perturb_layout(case: SimCase, generator: np.random.Generator, wide: bool):
    if case.initial_scattered and not case.fault_ids:
        # 编队用例：样本必须是"大范围随机散布"，否则模型学不会从散到连。
        while True:
            points = [(float(generator.uniform(case.lower[0] + 4.0, case.upper[0] - 4.0)),
                       float(generator.uniform(case.lower[1] + 4.0, case.upper[1] - 4.0)))
                      for _ in case.uav_ids]
            if all(math.dist(points[left], points[right]) >= case.min_separation + 0.5
                   for left in range(len(points)) for right in range(left + 1, len(points))):
                return {uav: point for uav, point in zip(case.uav_ids, points)}
    if case.initial_scattered:
        # 受损用例：损伤发生在"编队完成之后"（实际流程：散布->编队->损毁->重组），
        # 样本以编队目标形态（环阵）为基准 + 抖动。这样受损机与幸存机相邻，
        # 原仓库的 k-hop 损伤注意力才可达（全断开图会让它越界）。
        member_ids = tuple(case.uav_ids)
        base = lattice_targets({uav: case.layout[uav] for uav in case.uav_ids},
                               member_ids, case.min_separation,
                               case.radius - case.edge_margin - 0.5)
        angle = generator.uniform(0, 2 * math.pi)
        cos, sin = math.cos(angle), math.sin(angle)
        center = (sum(case.layout[uav][0] for uav in case.uav_ids) / len(case.uav_ids),
                  sum(case.layout[uav][1] for uav in case.uav_ids) / len(case.uav_ids))
        return {uav: (center[0] + (base[uav][0] - center[0]) * cos - (base[uav][1] - center[1]) * sin
                      + generator.normal(0, 0.4),
                      center[1] + (base[uav][0] - center[0]) * sin + (base[uav][1] - center[1]) * cos
                      + generator.normal(0, 0.4))
                for uav in case.uav_ids}
    angle = generator.uniform(-0.09 if wide else -0.07, 0.09 if wide else 0.07)
    scale = generator.uniform(0.96, 1.04)
    shift = generator.uniform((-1.3, -0.8), (1.3, 0.8))
    rotation = np.array(((math.cos(angle), -math.sin(angle)),
                         (math.sin(angle), math.cos(angle))))
    jitter = 0.22 if wide else 0.13
    return {uav: tuple((rotation @ (np.asarray(point) * scale)
                        + shift + generator.normal(0, jitter, size=2)).tolist())
            for uav, point in case.layout.items()}


@dataclass
class Sample:
    case: SimCase
    positions: dict[str, tuple[float, float]]
    targets: dict[str, tuple[float, float]]
    starts: torch.Tensor
    teacher: torch.Tensor
    features: torch.Tensor
    adjacency: torch.Tensor
    tree: tuple[tuple[int, int], ...]
    batches: tuple[tuple[int, ...], ...]


def forward_inputs(positions, identifiers, survivors, lower, upper, hops):
    from DAGL_Algorithm.Damage_Attention import dilated_DAM

    ordered = [identifiers.index(uav) for uav in survivors]
    locations = np.array([positions[uav] for uav in identifiers])
    offset = np.array(lower)
    starts, features, adjacency = dilated_DAM(locations - offset, ordered,
                                              khop=1, batch_size=hops)
    return (torch.tensor(starts + offset, dtype=torch.float32),
            torch.tensor((features - (np.array(upper) - offset) / 2) / SCALE,
                         dtype=torch.float32), adjacency.float())


def build_sample(case, positions, targets, bridge, groups, hops):
    survivors = case.survivors
    starts, features, adjacency = forward_inputs(
        positions, case.uav_ids, survivors, case.lower, case.upper, hops)
    # 损失里"必须收敛的边集"= 幸存机上的最小生成树：两分量场景等价于原实现
    # （桥边 + 各分量内树）；散布/多分量场景自然把所有孤立机串起来，正是编队/
    # 重组要拉近的边。
    tree = []
    connected = [survivors[0]]
    remaining = set(survivors[1:])
    while remaining:
        first, second = min(((uav, other) for uav in connected for other in remaining),
                            key=lambda pair: (distance(positions[pair[0]],
                                                       positions[pair[1]]), pair))
        tree.append((survivors.index(first), survivors.index(second)))
        connected.append(second)
        remaining.remove(second)
    # 分批口径直接用教师给的分组（= components(survivors, radius+edge_margin)），
    # 散布场景下 movement_batches 的"恰两块"约束不适用。
    batches = tuple(tuple(survivors.index(uav) for uav in group) for group in groups)
    return Sample(case, positions, targets, starts,
                  torch.tensor([targets[uav] for uav in survivors], dtype=torch.float32),
                  features, adjacency, tuple(tree), batches)


def move_to_device(sample, device):
    """把样本的全部张量搬到训练设备（positions/targets 保持 Python 原样）。"""
    return Sample(sample.case, sample.positions, sample.targets,
                  sample.starts.to(device), sample.teacher.to(device),
                  sample.features.to(device), sample.adjacency.to(device),
                  sample.tree, sample.batches)


def make_samples(cases, count, seed, hops, wide=False):
    generator = np.random.default_rng(seed)
    samples = []
    for case in cases:
        accepted = 0
        attempts = 0
        while accepted < count and attempts < count * 100:
            attempts += 1
            positions = perturb_layout(case, generator, wide)
            try:
                targets, bridge, groups = make_teacher(case, positions)
            except ValueError:
                continue
            samples.append(build_sample(case, positions, targets, bridge, groups, hops))
            accepted += 1
        if accepted != count:
            raise RuntimeError("could not generate enough safe training examples")
    generator.shuffle(samples)
    return samples


def candidate_goals(network, starts, features, adjacency, batches):
    candidates = []
    for output in network(features, adjacency, len(starts)):
        displacement = torch.zeros_like(output)
        for batch in batches:
            displacement[list(batch)] = output[list(batch)].mean(dim=0)
        candidates.append(starts + SCALE * displacement)
    return candidates


def predictions(network, sample):
    return candidate_goals(network, sample.starts, sample.features, sample.adjacency,
                           sample.batches)


def loss_for_goals(goals, sample):
    pairs = torch.triu_indices(len(goals), len(goals), offset=1)
    gaps = torch.linalg.vector_norm(goals[pairs[0]] - goals[pairs[1]], dim=1)
    tree_gaps = torch.stack([torch.linalg.vector_norm(goals[first] - goals[second])
                             for first, second in sample.tree])
    lower = torch.tensor(sample.case.lower, device=goals.device) + 3
    upper = torch.tensor(sample.case.upper, device=goals.device) - 3
    path_loss = goals.new_tensor(0.0)
    current = sample.starts
    for batch in sample.batches:
        finish = current.clone()
        finish[list(batch)] = goals[list(batch)]
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            point = current + fraction * (finish - current)
            path_gaps = torch.linalg.vector_norm(point[pairs[0]] - point[pairs[1]], dim=1)
            path_loss = path_loss + torch.relu((4.5 - path_gaps) / SCALE).square().mean()
        current = finish
    return (8 * ((goals - sample.teacher) / SCALE).square().sum(dim=1).mean()
            + 4 * torch.relu((tree_gaps - 6.0) / SCALE).square().mean()
            + 8 * torch.relu((4.5 - gaps) / SCALE).square().mean()
            + 4 * (torch.relu((lower - goals) / SCALE).square()
                   + torch.relu((goals - upper) / SCALE).square()).mean()
            + 4 * path_loss / (len(sample.batches) * 5)
            + 0.01 * torch.linalg.vector_norm((goals - sample.starts) / SCALE,
                                              dim=1).amax())


def evaluate(network, samples):
    network.eval()
    valid = 0
    errors = []
    with torch.no_grad():
        for sample in samples:
            # 候选整体搬一次 CPU：CUDA 上逐候选 .tolist() 会每项同步一次，
            # 验证阶段（300+ 样本 × 候选 × 门禁）会慢一个数量级。
            candidates = torch.stack(predictions(network, sample)).detach().cpu()
            teacher = sample.teacher.detach().cpu()
            errors.append(min(float(((goal - teacher) / SCALE).square().mean())
                              for goal in candidates))
            for goal in candidates:
                targets = {uav: tuple(value) for uav, value in
                           zip(sample.case.survivors, goal.tolist())}
                try:
                    sample.case.validate_targets(
                        {uav: sample.positions[uav] for uav in sample.case.survivors},
                        targets, {uav: sample.positions[uav] for uav in sample.case.fault_ids})
                except ValueError:
                    continue
                valid += 1
                break
    return {"passed": valid, "total": len(samples),
            "teacher_mse": sum(errors) / len(errors)}


def save_checkpoint(network, outdir, args, validation, testing, fixed):
    weights = outdir / "model.pt"
    # 保存前统一转 CPU：checkpoint 与训练设备解耦，CPU 推理 worker（dag_worker 显式
    # 屏蔽 CUDA）与无 GPU 机器都能直接加载。
    torch.save({name: value.detach().cpu() for name, value in network.state_dict().items()},
               weights)
    metadata = {"schema": SCHEMA, "status": "VALIDATED",
                "sha256": digest(weights), "scenario_sha256": digest(SCENARIO),
                "training_sha256": digest(Path(__file__)),
                "gcn_sha256": digest(DAG_ROOT / "DAGL_Algorithm/Bipartite_GCN.py"),
                "attention_sha256": digest(DAG_ROOT / "DAGL_Algorithm/Damage_Attention.py"),
                "hidden": args.hidden, "hops": args.hops, "scale_m": SCALE,
                "comm_radius_m": 8, "edge_margin_m": 1, "min_separation_m": 4,
                "validation": validation, "test": testing, "fixed": fixed,
                "seed": args.seed, "train_per_fault": args.train_samples,
                "validation_per_fault": args.validation_samples,
                "test_per_fault": args.test_samples,
                "epochs": args.epochs}
    (outdir / "model.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def load_checkpoint(path: Path):
    metadata_path = path.with_name("model.json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if (path.name != "model.pt" or metadata.get("schema") != SCHEMA
            or metadata.get("status") != "VALIDATED"
            or metadata.get("sha256") != digest(path)
            or metadata.get("scenario_sha256") != digest(SCENARIO)
            or metadata.get("training_sha256") != digest(Path(__file__))
            or metadata.get("gcn_sha256") != digest(DAG_ROOT / "DAGL_Algorithm/Bipartite_GCN.py")
            or metadata.get("attention_sha256")
            != digest(DAG_ROOT / "DAGL_Algorithm/Damage_Attention.py")
            or metadata.get("comm_radius_m") != 8
            or metadata.get("min_separation_m") != 4
            or metadata.get("edge_margin_m") != 1
            or metadata.get("scale_m") != SCALE
            or any(metadata.get(name, {}).get("passed") != metadata.get(name, {}).get("total")
                   for name in ("fixed", "validation", "test"))
            or any(metadata.get(name, {}).get("total", 0) < 200
                   for name in ("validation", "test"))
            or metadata.get("train_per_fault", 0) < 300):
        raise ValueError("unapproved or mismatched DAGL-10 checkpoint")
    network = build_network(int(metadata["hidden"]))
    network.load_state_dict(torch.load(path, map_location="cpu", weights_only=True))
    network.eval()
    return network, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=3407)
    parser.add_argument("--epochs", type=int, default=180)
    parser.add_argument("--train-samples", type=int, default=300, help="per fault set")
    parser.add_argument("--validation-samples", type=int, default=100, help="per fault set")
    parser.add_argument("--test-samples", type=int, default=100, help="per fault set")
    parser.add_argument("--hidden", type=int, default=64)
    parser.add_argument("--hops", type=int, default=3)
    parser.add_argument("--device", choices=("auto", "cpu", "cuda"), default="cpu",
                        help="训练设备；默认 cpu —— 本工作负载是逐样本（batch=1）小图，"
                             "2026-10-08 实测 GPU 反而慢约 6 倍（55 epochs：GPU ~55min vs "
                             "CPU ~9min，逐次同步/核开销占主导）；要用 GPU 需显式 "
                             "--device cuda，并建议先把训练改成批处理")
    parser.add_argument("--smoke", action="store_true", help="quick pipeline test; never approved")
    args = parser.parse_args()
    if (min(args.epochs, args.train_samples, args.validation_samples,
            args.test_samples, args.hidden, args.hops) <= 0 or args.hops > 10):
        parser.error("training sizes, epochs and hidden width must be positive; hops <= 10")
    if not args.smoke and (args.train_samples < 300 or args.validation_samples < 100
                           or args.test_samples < 100):
        parser.error("approval requires at least 300 train, 100 validation and 100 test cases per fault set")
    outdir = args.outdir.resolve()
    outdir.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    device = torch.device("cuda" if (args.device == "cuda" or
                                     (args.device == "auto" and torch.cuda.is_available()))
                          else "cpu")
    if device.type == "cuda":
        torch.cuda.manual_seed_all(args.seed)
        print("training device: cuda (%s)" % torch.cuda.get_device_name(0), flush=True)
    else:
        print("training device: cpu", flush=True)
    scenario_raw = json.loads(SCENARIO.read_text(encoding="utf-8"))
    # 散布场景额外含 "0"（编队阶段）；连通用例保持原样。
    counts = (0, 3, 4) if "0" in scenario_raw.get("fault_sets", {}) else (3, 4)
    cases = [SimCase.load(SCENARIO, count) for count in counts]
    configure_dag(cases[0])
    network = build_network(args.hidden).to(device)
    training = [move_to_device(sample, device)
                for sample in make_samples(cases, args.train_samples, args.seed + 1, args.hops)]
    validation = [move_to_device(sample, device) for sample in
                  make_samples(cases, args.validation_samples, args.seed + 2, args.hops,
                               wide=True)]
    testing = [move_to_device(sample, device) for sample in
               make_samples(cases, args.test_samples, args.seed + 3, args.hops, wide=True)]
    fixed = []
    for case in cases:
        # 固定布局验收："散布+受损"用例的参考状态是"编队完成后的环阵"（实际流程
        # 是 散布->编队->损毁->重组）；编队用例与连通场景用场景布局本身。
        if case.initial_scattered and case.fault_ids:
            members = tuple(case.uav_ids)
            reference = lattice_targets({uav: case.layout[uav] for uav in members}, members,
                                        case.min_separation,
                                        case.radius - case.edge_margin - 0.5)
        else:
            reference = case.layout
        targets, bridge, groups = make_teacher(case, reference)
        fixed.append(move_to_device(
            build_sample(case, reference, targets, bridge, groups, args.hops), device))
    optimizer = torch.optim.Adam(network.parameters(), lr=3e-4)
    best = None
    best_score = (-1, float("inf"))
    generator = np.random.default_rng(args.seed + 4)
    for epoch in range(args.epochs):
        network.train()
        total_loss = None
        for index in generator.permutation(len(training)):
            sample = training[index]
            objective = torch.stack([loss_for_goals(goals, sample)
                                     for goals in predictions(network, sample)]).mean()
            optimizer.zero_grad()
            objective.backward()
            torch.nn.utils.clip_grad_norm_(network.parameters(), 1.0)
            optimizer.step()
            # GPU 上按张量累加，每个 epoch 只同步一次：逐样本 float(...) 会强制
            # 每步一次设备同步（900 步/epoch），在 GPU 上反而比 CPU 慢一个数量级。
            total_loss = (objective.detach() if total_loss is None
                          else total_loss + objective.detach())
        epoch_loss = 0.0 if total_loss is None else float(total_loss) / len(training)
        if epoch % 5 == 4 or epoch == args.epochs - 1:
            result = evaluate(network, validation)
            fixed_result = evaluate(network, fixed)
            score = (result["passed"] + fixed_result["passed"],
                     result["teacher_mse"])
            if score[0] > best_score[0] or (score[0] == best_score[0]
                                             and score[1] < best_score[1]):
                best_score = score
                best = deepcopy(network.state_dict())
            print(f"epoch={epoch+1} loss={epoch_loss:.5f} "
                  f"validation={result['passed']}/{result['total']} "
                  f"fixed={fixed_result['passed']}/{fixed_result['total']}", flush=True)
    network.load_state_dict(best)
    validation_result = evaluate(network, validation)
    test_result = evaluate(network, testing)
    fixed_result = evaluate(network, fixed)
    summary = {"validation": validation_result, "test": test_result,
               "fixed": fixed_result, "smoke": args.smoke}
    (outdir / "training_summary.json").write_text(json.dumps(summary, indent=2),
                                                      encoding="utf-8")
    if (not args.smoke and all(report["passed"] == report["total"]
                              for report in (validation_result, test_result, fixed_result))):
        save_checkpoint(network, outdir, args, validation_result, test_result, fixed_result)
        print("VALIDATED checkpoint:", outdir / "model.pt")
    else:
        print("NO APPROVED CHECKPOINT: review training_summary.json and retrain")
        if not args.smoke:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
