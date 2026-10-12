#!/usr/bin/env python3
"""生成"随机散布"十机 SITL 场景（用于直观展示 DAGL 拓扑恢复功能）。

生成即验证：所有判据都按**工具自身的口径**执行，并且把"实测坐标 = 布局绕任务原点
旋转（冻结航向 ≈ ±0.05 rad）"这一系统误差显式建模——对旋转扫描后的坐标同样要求
全部判据成立，避免"布局达标、实测翻边"。

判据（与 core.validate_layout / sim_multi.movement_batches 一致）：
  1. 两两水平间距 ≥ 4.5 m（场景门限 4 m）；
  2. 初始十机图在 7 m（半径−余量）**稳健连通**；
  3. **只移除 A04**（首故障，桥）时在 9 m（半径+余量）恰好两块；
  4. 3/4 故障后按 9 m 分组恰好两组、每组 ≤4，且组内/组间各有 ≥0.5 m 余量；
  5. 布局先平移到"标定集（A01/A02/A07/A08）中位点 = (0,0)"（任务原点约定）；
  6. `train_dagl10.make_teacher` 对两种故障数都给出通过全部硬门禁的教师目标。

用法：
  python3 verification/dag_recovery/scenarios/make_random_scenario.py \
      --seed 20261011 --out verification/dag_recovery/scenarios/10.random.json \
      --fixture-out verification/dag_recovery/fixtures/ten_random_enu.json
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

DIRECTORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DIRECTORY))

from core import components, distance  # noqa: E402
from sim_multi import SimCase  # noqa: E402
from train_dagl10 import lattice_targets, make_teacher  # noqa: E402

UAV_IDS = ["A%02d" % index for index in range(1, 11)]
FAULT_SETS = {"3": ["A04", "A05", "A06"], "4": ["A04", "A05", "A06", "A10"]}
# 散布编队演示：额外训练/推理"0 故障"（编队）用例；仅在 scattered 模式写入场景。
ASSEMBLY_CASE = {"0": []}
CALIBRATION_UAVS = ("A01", "A02", "A07", "A08")   # sim_setup.py 的 SITL 标定集
# 残余旋转探针：sim_setup 已按 spawn_heading_rad 与冻结航向对齐（实测 = R(h_spawn−
# h_frozen)·布局），对齐后只剩标定航向噪声级残余（实测 ±0.005 rad 量级）。
ROTATION_PROBE_RAD = 0.01
SPAWN_HEADING_RAD = -0.048          # SITL 世界磁偏角（冻结航向实测值）；见 sim_setup.py


def sample_positions(rng: random.Random, half_x: float, half_y: float, min_sep: float):
    points = []
    attempts = 0
    while len(points) < len(UAV_IDS) and attempts < 100000:
        attempts += 1
        candidate = (round(rng.uniform(-half_x, half_x), 2), round(rng.uniform(-half_y, half_y), 2))
        if all(math.dist(candidate, other) >= min_sep for other in points):
            points.append(candidate)
    if len(points) < len(UAV_IDS):
        raise SystemExit("采样失败：场地太小或最小间距太大")
    return points


def anchor_to_calibration_median(layout):
    """平移布局，使标定集各坐标分量的中位数 = 0（= 任务原点，实飞同义）。"""
    xs = sorted(layout[uav][0] for uav in CALIBRATION_UAVS)
    ys = sorted(layout[uav][1] for uav in CALIBRATION_UAVS)
    middle = len(CALIBRATION_UAVS) // 2
    offset = ((xs[middle - 1] + xs[middle]) / 2.0, (ys[middle - 1] + ys[middle]) / 2.0)
    return {uav: (round(point[0] - offset[0], 2), round(point[1] - offset[1], 2))
            for uav, point in layout.items()}


def rotate_about_origin(layout, angle):
    """模拟实测坐标相对布局的系统旋转（绕任务原点）。"""
    cos, sin = math.cos(angle), math.sin(angle)
    return {uav: (point[0] * cos - point[1] * sin, point[0] * sin + point[1] * cos)
            for uav, point in layout.items()}


def block_gap(positions, faults, radius, edge_margin=1.0):
    """按（半径+余量）分组——movement_batches 口径：恰两组、每组 ≤4、各留 0.5 m 余量。"""
    survivors = {uav: positions[uav] for uav in UAV_IDS if uav not in faults}
    clusters = components(survivors, radius + edge_margin)
    if len(clusters) != 2 or any(len(cluster) > 4 for cluster in clusters):
        return None
    tightened = components(survivors, radius + edge_margin - 0.5)
    if sorted(map(sorted, tightened)) != sorted(map(sorted, clusters)):
        return None
    first, second = clusters
    return min(distance(survivors[a], survivors[b]) for a in first for b in second)


def criteria_ok(positions, radius, gap_min, scattered, half_x, half_y):
    """sim_multi.SimCase.validate_layout + movement_batches 的全部判据；不满足返回 None。"""
    if any(abs(point[0]) > half_x or abs(point[1]) > half_y for point in positions.values()):
        return None                                   # 场景边界内留余量
    if scattered:
        initial = components(positions, radius - 1.0)
        if len(initial) < 3 or any(len(group) > 4 for group in initial):
            return None                               # 散布起步：≥3 块且每块 ≤4（编队批次）
    else:
        if len(components(positions, radius - 1.0)) != 1:
            return None                               # 初始图 7 m 稳健连通
        bridge_only = {uav: point for uav, point in positions.items()
                       if uav != FAULT_SETS["3"][0]}
        if len(components(bridge_only, radius + 1.0)) != 2:
            return None                               # 只移除 A04 → 9 m 恰两块
    if scattered:
        survivors = {uav: positions[uav] for uav in UAV_IDS
                     if uav not in FAULT_SETS["3"]}
        if len(components(survivors, radius + 1.0)) < 2:
            return None                               # 故障退出后仍 ≥2 块
        return {"3": None, "4": None}                 # 散布模式无"两块间距"概念
    gaps = {}
    for count, faults in FAULT_SETS.items():
        gap = block_gap(positions, faults, radius)
        if gap is None or gap < gap_min:
            return None                               # 故障后恰 2 组、每组 ≤4、带余量
        gaps[count] = gap
    return gaps


def build_scattered_layout(rng: random.Random, half_x: float, half_y: float,
                           radius: float, min_sep: float):
    """散布布局（编队演示）：十机两两隔离（间距 > radius+edge_margin=9m），
    8 m 通信图全断开 = 最纯粹的"随机散布在大片区域"状态。编队阶段的目标是全体
    同时机动到一个紧凑连通格阵（train_dagl10.lattice_targets），即 DAG-GCBF 的
    "从散到连"语义；损毁后幸存机再同时聚合成新格阵。
    """
    isolation = radius + min_sep * 0.5      # 采样下限 10 m，给实测漂移留余量
    for _ in range(20000):
        points = [(round(rng.uniform(-half_x, half_x), 2),
                   round(rng.uniform(-half_y, half_y), 2)) for _ in UAV_IDS]
        if all(math.dist(points[left], points[right]) >= isolation
               for left in range(len(points)) for right in range(left + 1, len(points))):
            return dict(zip(UAV_IDS, points))
    raise ValueError("scattered sampling failed: enlarge the area or lower isolation")


def build_scenario(seed: int, half_x: float, half_y: float, min_sep: float,
                   gap_min: float, radius: float, scattered: bool = False):
    rng = random.Random(seed)
    # 场景边界：留出锚定平移与旋转的余量（散布模式下明显大于采样半径）。
    bound_x = 2.0 * half_x + 6.0
    bound_y = 2.0 * half_y + 6.0
    fault_sets = {**FAULT_SETS, **ASSEMBLY_CASE} if scattered else dict(FAULT_SETS)
    for _ in range(4000):
        if scattered:
            layout = anchor_to_calibration_median(
                build_scattered_layout(rng, half_x, half_y, radius, min_sep))
            candidates = [layout]
        else:
            points = sample_positions(rng, half_x, half_y, min_sep)
            labels = list(UAV_IDS)
            candidates = []
            for _ in range(400):
                rng.shuffle(labels)
                candidates.append(anchor_to_calibration_median(dict(zip(labels, points))))
        for layout in candidates:
            gaps = None
            ok = True
            for angle in (0.0, ROTATION_PROBE_RAD, -ROTATION_PROBE_RAD):
                gaps = criteria_ok(rotate_about_origin(layout, angle), radius, gap_min,
                                   scattered, bound_x - 2.0, bound_y - 2.0)
                if gaps is None:
                    ok = False
                    break
            if not ok:
                continue
            ring = None
            if scattered:
                members = tuple(UAV_IDS)
                ring = lattice_targets(
                    {uav: tuple(layout[uav]) for uav in members}, members, 4.0, radius - 1.5)
            for count, faults in fault_sets.items():
                case = SimCase(uav_ids=tuple(UAV_IDS), fault_ids=tuple(faults),
                               layout=layout, lower=(-bound_x, -bound_y),
                               upper=(bound_x, bound_y),
                               radius=radius, edge_margin=1, min_separation=4,
                               layout_tolerance=1.0, normal_height=15, fault_height=8,
                               altitude_tolerance=0.7, seed=seed,
                               initial_scattered=scattered)
                # 散布场景：受损用例的教师校验用"编队完成后"的环阵（实际流程是
                # 散布->编队->损毁->重组）；编队用例用出生散布态。
                probe = layout if (not scattered or not faults) else ring
                try:
                    make_teacher(case, probe)    # 教师目标 + 全部硬门禁
                except ValueError:
                    ok = False
                    break
            if not ok:
                continue
            raw = {
                "uav_ids": UAV_IDS,
                "fault_sets": {key: list(value) for key, value in fault_sets.items()},
                "layout_task_xy": {uav: [layout[uav][0], layout[uav][1]] for uav in UAV_IDS},
                "bounds_task_xy": [[-bound_x, -bound_y], [bound_x, bound_y]],
                "initial_graph": "scattered" if scattered else "connected",
                "spawn_heading_rad": SPAWN_HEADING_RAD,
                "comm_radius_m": radius,
                "edge_margin_m": 1,
                "min_separation_m": 4,
                "layout_tolerance_m": 1.0,
                "normal_height_m": 15,
                "fault_height_m": 8,
                "altitude_tolerance_m": 0.7,
                "seed": seed,
            }
            return raw, gaps
    raise SystemExit("在给定约束下未找到可行随机布局，请放宽 gap 范围或扩大场地")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=20261011)
    parser.add_argument("--scattered", action="store_true",
                        help="编队演示模式：大面积散布起步（初始图不连通，含 0 故障编队用例）")
    parser.add_argument("--half-x", type=float, default=None)
    parser.add_argument("--half-y", type=float, default=None)
    parser.add_argument("--min-sep", type=float, default=4.5)
    parser.add_argument("--gap-min", type=float, default=9.8,
                        help="两簇最近间距下限（须 > 半径+余量=9 m 并留实测余量）")
    parser.add_argument("--radius", type=float, default=8.0)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fixture-out", type=Path, default=None,
                        help="同时输出离线预检快照（heading=π/2 时 ENU 与任务系一致）")
    args = parser.parse_args()
    default_x, default_y = (30.0, 20.0) if args.scattered else (15.0, 9.0)
    half_x = default_x if args.half_x is None else args.half_x
    half_y = default_y if args.half_y is None else args.half_y
    raw, gaps = build_scenario(args.seed, half_x, half_y, args.min_sep,
                               args.gap_min, args.radius, scattered=args.scattered)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(raw, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if args.fixture_out is not None:
        # 散布场景的离线预检快照 = "编队完成后（受损前）"的环阵：实际流程是
        # 散布->编队->损毁->重组，受损推理的输入状态是编队态；用散布态会让原仓库
        # 的 k-hop 损伤注意力越界（受损机不可达）。
        if args.scattered:
            members = tuple(UAV_IDS)
            reference = lattice_targets(
                {uav: tuple(raw["layout_task_xy"][uav]) for uav in members}, members,
                float(raw["min_separation_m"]),
                float(raw["comm_radius_m"]) - float(raw["edge_margin_m"]) - 0.5)
            points = {uav: [reference[uav][0], reference[uav][1], raw["normal_height_m"]]
                      for uav in UAV_IDS}
        else:
            points = {uav: [point[0], point[1], raw["normal_height_m"]]
                      for uav, point in raw["layout_task_xy"].items()}
        fixture = {"heading_rad": math.pi / 2.0, "poses_enu": points}
        args.fixture_out.parent.mkdir(parents=True, exist_ok=True)
        args.fixture_out.write_text(json.dumps(fixture, ensure_ascii=False, indent=2) + "\n",
                                    encoding="utf-8")
        print("已生成离线预检快照:", args.fixture_out)
    print("已生成随机场景:", args.out)
    if gaps.get("3") is None:
        print("  散布模式：无两块间距（编队为格阵聚合）")
    else:
        print("  两两块间距: 3 故障 %.2f m | 4 故障 %.2f m" % (gaps["3"], gaps["4"]))
    for uav in UAV_IDS:
        print("   %s: (%.2f, %.2f)" % (uav, raw["layout_task_xy"][uav][0], raw["layout_task_xy"][uav][1]))


if __name__ == "__main__":
    main()
