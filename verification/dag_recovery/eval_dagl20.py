#!/usr/bin/env python3
"""Offline N20 learned-goal verification; never connects to ROS or PX4."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from core import _pair, components, distance
from pipeline import digest
from train_dagl10 import candidate_goals, forward_inputs
from train_dagl20 import SCENARIO, TwentyCase, configure_dag, load_checkpoint


def evaluate(case, positions, checkpoint):
    case.validate_layout(positions)
    if any(distance(positions[uav], case.layout[uav]) > case.layout_tolerance
           for uav in case.uav_ids):
        raise ValueError("positions exceed approved N20 layout tolerance")
    configure_dag(case)
    network, metadata = load_checkpoint(checkpoint)
    starts, features, adjacency = forward_inputs(
        positions, case.uav_ids, case.survivors, case.lower, case.upper,
        metadata["hops"])
    batches = tuple(tuple(case.survivors.index(uav) for uav in batch)
                    for batch in case.movement_batches(positions))
    with torch.no_grad():
        candidates = candidate_goals(network, starts, features, adjacency, batches)
    results = []
    for branch, candidate in enumerate(candidates):
        goals = {uav: _pair(point) for uav, point in
                 zip(case.survivors, candidate.tolist())}
        max_movement = max(distance(positions[uav], goals[uav])
                           for uav in case.survivors)
        try:
            case.validate_targets({uav: positions[uav] for uav in case.survivors},
                                  goals, {uav: positions[uav] for uav in case.fault_ids})
            error = None
        except ValueError as exc:
            error = str(exc)
        results.append({"branch": branch, "targets_task_xy": goals,
                        "max_target_displacement_m": max_movement, "failure": error})
    valid = [item for item in results if item["failure"] is None]
    selected = min(valid, key=lambda item: (item["max_target_displacement_m"],
                                            item["branch"])) if valid else None
    report = {"status": "OFFLINE_DAGL20_VALIDATED" if selected else "FAILED",
              "fault_ids": list(case.fault_ids), "checkpoint_sha256": metadata["sha256"],
              "pre_fault_components_at_7m": len(components(positions, 7)),
              "post_fault_components_at_9m": len(components(
                  {uav: positions[uav] for uav in case.survivors}, 9)),
              "branches": results, "selected": selected}
    if selected:
        goals = selected["targets_task_xy"]
        report["goal_components_at_7m"] = len(components(goals, 7))
        report["min_target_separation_m"] = min(
            distance(goals[first], goals[second])
            for index, first in enumerate(case.survivors)
            for second in case.survivors[index + 1:])
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--faults", type=int, required=True, choices=(3, 4))
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--positions-json", type=Path, help="optional 20 ID -> [right, forward] snapshot")
    args = parser.parse_args()
    case = TwentyCase.load(SCENARIO, args.faults)
    if args.positions_json:
        raw = json.loads(args.positions_json.read_text(encoding="utf-8"))
        positions = {uav: _pair(point) for uav, point in raw.items()}
    else:
        positions = case.layout
    args.outdir.mkdir(parents=True, exist_ok=False)
    try:
        report = evaluate(case, positions, args.checkpoint)
    except (ValueError, KeyError, RuntimeError, FileNotFoundError) as error:
        report = {"status": "FAILED", "reason": str(error)}
    report["scenario_sha256"] = digest(SCENARIO)
    (args.outdir / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False),
                                               encoding="utf-8")
    display = {key: value for key, value in report.items()
               if key not in ("branches", "selected")}
    if report.get("selected"):
        display["selected_branch"] = report["selected"]["branch"]
        display["max_target_displacement_m"] = report["selected"]["max_target_displacement_m"]
    print(json.dumps(display, indent=2, allow_nan=False))
    if report["status"] != "OFFLINE_DAGL20_VALIDATED":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
