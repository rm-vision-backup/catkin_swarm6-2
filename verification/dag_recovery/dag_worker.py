"""Standalone ML-DAGL worker; never imports ROS or GCBF+."""

import json
import os
import sys
from pathlib import Path


def main():
    if len(sys.argv) != 4:
        raise SystemExit("usage: dag_worker.py INPUT_JSON OUTPUT_JSON DAG_ROOT")
    input_path, output_path, dag_root = map(Path, sys.argv[1:])
    raw = json.loads(input_path.read_text(encoding="utf-8"))
    identifiers = raw["uav_ids"]
    positions = raw["positions_task_xy"]
    simulation = raw.get("simulation_only") is True
    fault_ids = raw["fault_ids"] if simulation else [raw["fault_id"]]
    lower, upper = raw["bounds_task_xy"]
    if len(set(identifiers)) != len(identifiers) or set(positions) != set(identifiers):
        raise ValueError("worker identity mismatch")
    if simulation:
        if (len(identifiers) != 10 or len(fault_ids) not in (0, 3, 4)
                or len(set(fault_ids)) != len(fault_ids)
                or not set(fault_ids).issubset(identifiers)):
            # 0 故障 = 散布编队阶段（全体十机同时机动连成一张图）。
            raise ValueError("simulation requires 10 UAVs and 0, 3 or 4 distinct faults")
    elif len(identifiers) not in (4, 11, 15) or fault_ids[0] not in identifiers:
        raise ValueError("flight worker invalid fault or swarm size")
    if not dag_root.is_dir() or not (dag_root / "DAGL_Algorithm" / "DAGL_Framework.py").is_file():
        raise ValueError("ML-DAGL source directory not found")

    os.environ["CUDA_VISIBLE_DEVICES"] = ""
    os.chdir(dag_root)
    sys.path.insert(0, str(dag_root))
    import numpy as np
    import torch
    import Configurations as cfg

    points = np.array([positions[uav] for uav in identifiers], dtype=np.float64)
    offset = np.asarray(lower, dtype=np.float64)
    extent = np.asarray(upper, dtype=np.float64) - offset
    if not np.isfinite(points).all() or not np.isfinite(extent).all() or np.any(extent <= 0):
        raise ValueError("non-finite DAGL input or invalid bounds")
    cfg.config_num_of_agents = len(identifiers)
    cfg.config_dimension = 2
    cfg.config_communication_range = float(raw["comm_radius_m"])
    cfg.config_width, cfg.config_length = map(float, extent)
    cfg.config_space_range = extent
    cfg.config_central_point = extent * 0.5
    cfg.config_initial_swarm_positions = points - offset

    remain = [index for index, uav in enumerate(identifiers) if uav not in fault_ids]
    if simulation:
        from sim_multi import SimCase
        from train_dagl10 import SCENARIO, candidate_goals, forward_inputs, load_checkpoint

        if not raw.get("checkpoint"):
            raise ValueError("ten-aircraft DAGL requires an approved checkpoint")
        case = SimCase.load(SCENARIO, len(fault_ids))
        if tuple(identifiers) != case.uav_ids or tuple(fault_ids) != case.fault_ids:
            raise ValueError("checkpoint fault set or UAV identity mismatch")
        network, metadata = load_checkpoint(Path(raw["checkpoint"]))
        starts, features, adjacency = forward_inputs(
            positions, identifiers, case.survivors, lower, upper, metadata["hops"])
        if case.initial_scattered:
            # 散布场景（编队/重组）：分批口径 = 严格半径（radius-edge_margin）下的
            # 连通分量，与训练一致；movement_batches 的"恰两块"约束不适用。
            from core import components
            survivor_positions = {uav: positions[uav] for uav in case.survivors}
            groups = components(survivor_positions, case.radius - case.edge_margin)
        else:
            groups = case.movement_batches(positions)
        batches = tuple(tuple(case.survivors.index(uav) for uav in group)
                        for group in groups)
        with torch.no_grad():
            options = candidate_goals(network, starts, features, adjacency, batches)
        candidates = []
        for index, option in enumerate(options):
            candidate = {uav: tuple(target) for uav, target in
                         zip(case.survivors, option.tolist())}
            try:
                case.validate_targets({uav: positions[uav] for uav in case.survivors},
                                      candidate, {uav: positions[uav] for uav in case.fault_ids})
            except ValueError:
                continue
            movement = float(torch.linalg.vector_norm(option - starts, dim=1).amax())
            candidates.append((movement, index, option))
        if candidates:
            estimated_time, selected, goals = min(candidates, key=lambda item: item[:2])
        else:
            selected = int(np.argmin([float(torch.linalg.vector_norm(option - starts,
                                                                      dim=1).amax())
                                      for option in options]))
            goals = options[selected]
            estimated_time = float(torch.linalg.vector_norm(goals - starts, dim=1).amax())
        result = goals.numpy() - offset
        solver = "DAGL-10 residual trained checkpoint"
        extra = {"checkpoint_sha256": metadata["sha256"], "branch": selected,
                 "safe_candidate_found": bool(candidates)}
    else:
        from DAGL_Algorithm.DAGL_Framework import DAGL

        torch.manual_seed(int(raw["seed"]))
        np.random.seed(int(raw["seed"]))
        _, estimated_time, result = DAGL(use_pretrained=False).dagl(
            cfg.config_initial_swarm_positions.copy(), remain
        )
        solver = "ML-DAGL untrained 50-step optimization"
        extra = {}
    targets = np.asarray(result, dtype=np.float64)
    if targets.shape != (len(remain), 2) or not np.isfinite(targets).all():
        raise ValueError("DAGL returned invalid targets")
    payload = {
        "targets_task_xy": {
            identifiers[index]: (targets[position] + offset).tolist()
            for position, index in enumerate(remain)
        },
        "dag_estimated_time": float(estimated_time),
        "solver": solver,
        **extra,
    }
    output_path.write_text(json.dumps(payload, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    main()
