#!/usr/bin/env python3
"""Train the independent 20-node DAGL topology-recovery model (offline only)."""

from __future__ import annotations

import argparse
import json
import random
from copy import deepcopy
from pathlib import Path

import numpy as np
import torch

from core import _pair, components
from pipeline import DAG_ROOT, digest
from sim_multi import SimCase
import train_dagl10 as training

DIRECTORY = Path(__file__).resolve().parent
SCENARIO = DIRECTORY / "scenarios/20.training.json"
SCHEMA = "dagl20-residual-v1"


class TwentyCase(SimCase):
    @classmethod
    def load(cls, path: Path, faults: int):
        raw = json.loads(path.read_text(encoding="utf-8"))
        if "max_displacement_m" in raw:
            raise ValueError("20-node training must not impose a displacement limit")
        if faults not in (3, 4):
            raise ValueError("select three or four logical exits")
        identifiers = tuple(raw["uav_ids"])
        fault_ids = tuple(raw["fault_sets"][str(faults)])
        bounds = raw["bounds_task_xy"]
        case = cls(
            uav_ids=identifiers, fault_ids=fault_ids,
            layout={uav: _pair(point) for uav, point in raw["layout_task_xy"].items()},
            lower=_pair(bounds[0]), upper=_pair(bounds[1]),
            radius=float(raw["comm_radius_m"]),
            edge_margin=float(raw["edge_margin_m"]),
            min_separation=float(raw["min_separation_m"]),
            layout_tolerance=float(raw["layout_tolerance_m"]),
            normal_height=float(raw["normal_height_m"]),
            fault_height=float(raw["fault_height_m"]),
            altitude_tolerance=float(raw["altitude_tolerance_m"]),
            seed=int(raw["seed"]),
        )
        if (len(identifiers) != 20 or len(set(identifiers)) != 20
                or set(case.layout) != set(identifiers)
                or len(fault_ids) != faults or len(set(fault_ids)) != faults
                or not set(fault_ids).issubset(identifiers)
                or (case.radius, case.edge_margin, case.min_separation,
                    case.normal_height, case.fault_height) != (8, 1, 4, 15, 8)
                or not 0 < case.layout_tolerance <= 1
                or not 0 < case.altitude_tolerance <= 0.7
                or case.seed < 0
                or any(case.lower[index] + 6 >= case.upper[index]
                       for index in range(2))):
            raise ValueError("invalid 20-node training configuration")
        case.validate_layout(case.layout)
        return case

    def movement_batches(self, positions):
        groups = components({uav: positions[uav] for uav in self.survivors},
                            self.radius + self.edge_margin)
        if len(groups) != 2:
            raise ValueError("20-node recovery requires two survivor groups")
        groups.sort(key=lambda group: sum(positions[uav][0] for uav in group) / len(group))
        batches = []
        for group, reverse in zip(groups, (True, False)):
            ordered = sorted(group, key=lambda uav: (positions[uav][0], uav),
                             reverse=reverse)
            batches.extend(tuple(ordered[start:start + 4])
                           for start in range(0, len(ordered), 4))
        return tuple(batches)


def configure_dag(case: TwentyCase):
    config = training.configure_dag(case)
    config.config_num_of_agents = len(case.uav_ids)
    return config


def save_checkpoint(network, outdir, args, validation, testing, fixed):
    weights = outdir / "model.pt"
    torch.save(network.state_dict(), weights)
    metadata = {"schema": SCHEMA, "status": "VALIDATED",
                "sha256": digest(weights), "scenario_sha256": digest(SCENARIO),
                "training_sha256": digest(Path(__file__)),
                "shared_training_sha256": digest(DIRECTORY / "train_dagl10.py"),
                "gcn_sha256": digest(DAG_ROOT / "DAGL_Algorithm/Bipartite_GCN.py"),
                "attention_sha256": digest(DAG_ROOT / "DAGL_Algorithm/Damage_Attention.py"),
                "num_agents": 20, "hidden": args.hidden, "hops": args.hops,
                "scale_m": training.SCALE, "comm_radius_m": 8,
                "edge_margin_m": 1, "min_separation_m": 4,
                "validation": validation, "test": testing, "fixed": fixed,
                "seed": args.seed, "train_per_fault": args.train_samples,
                "validation_per_fault": args.validation_samples,
                "test_per_fault": args.test_samples, "epochs": args.epochs}
    (outdir / "model.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def load_checkpoint(path: Path):
    metadata = json.loads(path.with_name("model.json").read_text(encoding="utf-8"))
    if (path.name != "model.pt" or metadata.get("schema") != SCHEMA
            or metadata.get("status") != "VALIDATED"
            or metadata.get("sha256") != digest(path)
            or metadata.get("scenario_sha256") != digest(SCENARIO)
            or metadata.get("training_sha256") != digest(Path(__file__))
            or metadata.get("shared_training_sha256") != digest(DIRECTORY / "train_dagl10.py")
            or metadata.get("gcn_sha256") != digest(DAG_ROOT / "DAGL_Algorithm/Bipartite_GCN.py")
            or metadata.get("attention_sha256")
            != digest(DAG_ROOT / "DAGL_Algorithm/Damage_Attention.py")
            or metadata.get("num_agents") != 20
            or (metadata.get("scale_m"), metadata.get("comm_radius_m"),
                metadata.get("edge_margin_m"), metadata.get("min_separation_m"))
            != (8, 8, 1, 4)
            or metadata.get("train_per_fault", 0) < 300
            or any(metadata.get(name, {}).get("passed") != metadata.get(name, {}).get("total")
                   for name in ("fixed", "validation", "test"))
            or any(metadata.get(name, {}).get("total", 0) < required
                   for name, required in (("fixed", 2), ("validation", 200), ("test", 200)))):
        raise ValueError("unapproved or mismatched DAGL-20 checkpoint")
    network = training.build_network(int(metadata["hidden"]))
    network.load_state_dict(torch.load(path, map_location="cpu", weights_only=True))
    network.eval()
    return network, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=3407)
    parser.add_argument("--epochs", type=int, default=55)
    parser.add_argument("--train-samples", type=int, default=300, help="per fault set")
    parser.add_argument("--validation-samples", type=int, default=100, help="per fault set")
    parser.add_argument("--test-samples", type=int, default=100, help="per fault set")
    parser.add_argument("--hidden", type=int, default=64)
    parser.add_argument("--hops", type=int, default=3)
    parser.add_argument("--smoke", action="store_true", help="pipeline test; no approved weights")
    args = parser.parse_args()
    if (min(args.epochs, args.train_samples, args.validation_samples,
            args.test_samples, args.hidden, args.hops) <= 0 or args.hops > 10):
        parser.error("training sizes, epochs and hidden width must be positive; hops <= 10")
    if not args.smoke and (args.train_samples < 300 or args.validation_samples < 100
                           or args.test_samples < 100):
        parser.error("approval requires 300 train, 100 validation and 100 test cases per fault set")
    outdir = args.outdir.resolve()
    outdir.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    cases = [TwentyCase.load(SCENARIO, count) for count in (3, 4)]
    configure_dag(cases[0])
    network = training.build_network(args.hidden)
    training_samples = training.make_samples(cases, args.train_samples, args.seed + 1,
                                             args.hops)
    validation = training.make_samples(cases, args.validation_samples, args.seed + 2,
                                       args.hops, wide=True)
    testing = training.make_samples(cases, args.test_samples, args.seed + 3,
                                    args.hops, wide=True)
    fixed = []
    for case in cases:
        targets, bridge, groups = training.make_teacher(case, case.layout)
        fixed.append(training.build_sample(case, case.layout, targets, bridge, groups,
                                           args.hops))
    optimizer = torch.optim.Adam(network.parameters(), lr=3e-4)
    generator = np.random.default_rng(args.seed + 4)
    best = None
    best_score = (-1, float("inf"))
    for epoch in range(args.epochs):
        network.train()
        total_loss = 0.0
        for index in generator.permutation(len(training_samples)):
            sample = training_samples[index]
            objective = torch.stack([training.loss_for_goals(goals, sample)
                                     for goals in training.predictions(network, sample)]).mean()
            optimizer.zero_grad()
            objective.backward()
            torch.nn.utils.clip_grad_norm_(network.parameters(), 1.0)
            optimizer.step()
            total_loss += float(objective.detach())
        if epoch % 5 == 4 or epoch == args.epochs - 1:
            result = training.evaluate(network, validation)
            fixed_result = training.evaluate(network, fixed)
            score = (result["passed"] + fixed_result["passed"], result["teacher_mse"])
            if score[0] > best_score[0] or (score[0] == best_score[0]
                                             and score[1] < best_score[1]):
                best_score = score
                best = deepcopy(network.state_dict())
            print(f"epoch={epoch+1} loss={total_loss/len(training_samples):.5f} "
                  f"validation={result['passed']}/{result['total']} "
                  f"fixed={fixed_result['passed']}/2", flush=True)
    network.load_state_dict(best)
    validation_result = training.evaluate(network, validation)
    testing_result = training.evaluate(network, testing)
    fixed_result = training.evaluate(network, fixed)
    summary = {"validation": validation_result, "test": testing_result,
               "fixed": fixed_result, "smoke": args.smoke}
    (outdir / "training_summary.json").write_text(json.dumps(summary, indent=2),
                                                      encoding="utf-8")
    if (not args.smoke and all(report["passed"] == report["total"]
                              for report in (validation_result, testing_result, fixed_result))):
        save_checkpoint(network, outdir, args, validation_result, testing_result,
                        fixed_result)
        print("VALIDATED checkpoint:", outdir / "model.pt")
    else:
        print("NO APPROVED CHECKPOINT: review training_summary.json and retrain")
        if not args.smoke:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
