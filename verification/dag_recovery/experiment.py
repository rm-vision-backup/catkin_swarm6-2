#!/usr/bin/env python3
"""Independent DAGL experiment driver; defaults to offline validation only."""

from __future__ import annotations

import argparse
import datetime
import json
import math
import re
import sys
import time
from pathlib import Path

from core import Scenario, components, distance, normalize_positions
from pipeline import Recorder, digest, provenance, run_solver, solver_input


def verify_approval(scenario: Scenario, scenario_path: Path, identity_path: Path,
                    deployment_path: Path, approval_path: Path) -> dict:
    if not scenario.approved:
        raise ValueError("flight scenario has not been reviewed")
    if any(re.fullmatch(r"A\d{2}", name) is None for name in scenario.uav_ids):
        raise ValueError("flight roster still contains unassigned template slots")
    deployment = json.loads(deployment_path.read_text(encoding="utf-8"))
    if (deployment["uav_ids"] != list(scenario.uav_ids)
            or Path(deployment["identity_file"]).resolve() != identity_path.resolve()
            or Path(deployment["scenario_file"]).resolve() != scenario_path.resolve()):
        raise ValueError("deployment manifest does not match selected scenario/identity")
    calibration = Path(deployment["calibration_file"])
    topology = Path(deployment["topology_file"])
    formation = Path(deployment["formation_file"])
    if not all(path.is_file() for path in (calibration, topology, formation)):
        raise ValueError("experiment-specific GCS and onboard configs missing")
    approval = json.loads(approval_path.read_text(encoding="utf-8"))
    required = ("approved_for_flight", "site_clearance_checked", "rc_takeover_checked",
                "rosbag_recording_verified", "airframes_checked", "task_load_checked",
                "single_setpoint_publisher_verified", "topology_distributed_to_all",
                "rtk_and_origin_checked", "bidirectional_mavlink_checked",
                "altitude_permission_checked", "flight_paths_signed_off")
    if any(approval.get(field) is not True for field in required):
        raise ValueError("flight approval checklist incomplete")
    if len(scenario.uav_ids) >= 11 and approval.get("four_aircraft_passed") is not True:
        raise ValueError("4-aircraft gate has not been passed")
    if len(scenario.uav_ids) == 15 and approval.get("eleven_aircraft_passed") is not True:
        raise ValueError("11-aircraft gate has not been passed")
    if (approval.get("flight_date") != datetime.date.today().isoformat()
            or not str(approval.get("operator", "")).strip()):
        raise ValueError("flight approval must identify an operator and today's date")
    hashes = {"scenario_sha256": digest(scenario_path),
              "identity_sha256": digest(identity_path),
              "topology_sha256": digest(topology),
              "calibration_sha256": digest(calibration),
              "formation_sha256": digest(formation)}
    if any(approval.get(key) != value for key, value in hashes.items()):
        raise ValueError("flight approval does not match immutable config hashes")
    import yaml
    selected = [item["exec_target"] for item in
                yaml.safe_load(identity_path.read_text(encoding="utf-8"))["uavs"]
                if item["uav_id"] in scenario.uav_ids and item.get("enabled")]
    if len(selected) != len(scenario.uav_ids) or set(selected) != set(deployment["exec_targets"]):
        raise ValueError("flight roster does not match enabled identity map")
    return {"approval": approval, "deployment": deployment}


def ensure_initial(scenario: Scenario, poses: dict, altitude: float):
    positions = {uav: point[:2] for uav, point in poses.items()}
    scenario.validate_fixture(positions)
    for uav in scenario.uav_ids:
        if abs(poses[uav][2] - altitude) > scenario.altitude_tolerance:
            raise ValueError("unexpected altitude for %s: %.2f" % (uav, poses[uav][2]))
    return positions


def targets_for(scenario: Scenario, initial: dict, recorder: Recorder,
                worker_python: str, health, target_fixture: Path | None = None):
    payload = solver_input(scenario, initial)
    if target_fixture is None:
        result = run_solver(payload, recorder, python=worker_python, healthy=health)
    else:
        result = json.loads(target_fixture.read_text(encoding="utf-8"))
        recorder.save("dag_input.json", payload)
        recorder.save("dag_output.json", result)
        recorder.event("fixture_only", file=str(target_fixture.resolve()))
    targets = result["targets_task_xy"]
    scenario.validate_targets({uav: initial[uav] for uav in scenario.uav_ids
                               if uav != scenario.fault_id}, targets,
                              initial[scenario.fault_id])
    recorder.event("plan_validated", targets=targets,
                   predicted_clusters=components(targets,
                                                  scenario.radius - scenario.edge_margin))
    return targets


def run_offline(scenario: Scenario, snapshot_path: Path, fixture: Path | None,
                recorder: Recorder, worker_python: str):
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    poses = normalize_positions(snapshot["poses_enu"], float(snapshot["heading_rad"]))
    positions = {uav: point[:2] for uav, point in poses.items()}
    scenario.validate_fixture(positions)
    if set(poses) != set(scenario.uav_ids):
        raise ValueError("offline snapshot must include the entire experiment roster")
    recorder.save("offline_snapshot.json", snapshot)
    targets = targets_for(scenario, positions, recorder, worker_python, lambda: None, fixture)
    summary = {"status": "OFFLINE_PLAN_ONLY", "uav_count": len(scenario.uav_ids),
               "initial_clusters": len(components(positions, scenario.radius)),
               "after_removal_clusters": len(components(
                   {uav: point for uav, point in positions.items() if uav != scenario.fault_id},
                   scenario.radius)),
               "predicted_final_clusters": len(components(targets, scenario.radius)),
               "targets_task_xy": targets}
    recorder.save("summary.json", summary)
    return summary


def run_flight(scenario: Scenario, identity_path: Path, recorder: Recorder,
               worker_python: str):
    from ros_adapter import RosFleet

    fleet = RosFleet(scenario, identity_path, recorder.run_dir.name, recorder)
    try:
        first = fleet.wait_ready()
        poses = first["poses_task"]
        ensure_initial(scenario, poses, altitude=5.0)
        recorder.save("takeoff_snapshot.json", first)
        fleet.activate()
        for start in range(0, len(scenario.uav_ids), 4):
            snapshot = fleet.snapshot()
            batch = scenario.uav_ids[start:start + 4]
            fleet.send_goal("MOVE_TO", {uav: (*snapshot["poses_task"][uav][:2],
                                              scenario.normal_height) for uav in batch})
        staged = fleet.snapshot()
        initial = ensure_initial(scenario, staged["poses_task"], scenario.normal_height)
        recorder.save("pre_fault_snapshot.json", staged)
        fleet.send_goal("FAULT_EXIT", {scenario.fault_id: (*initial[scenario.fault_id],
                                                         scenario.fault_height)})
        after_fault = fleet.snapshot()
        poses = after_fault["poses_task"]
        scenario.validate_fixture({uav: pose[:2] for uav, pose in poses.items()})
        for uav in scenario.uav_ids:
            if uav != scenario.fault_id and abs(poses[uav][2] - scenario.normal_height) > scenario.altitude_tolerance:
                raise ValueError("survivor drifted from 15 m layer: " + uav)
        if (abs(poses[scenario.fault_id][2] - scenario.fault_height)
                > scenario.altitude_tolerance
                or distance(initial[scenario.fault_id], poses[scenario.fault_id][:2])
                > scenario.layout_tolerance):
            raise ValueError("fault aircraft did not stabilize at the isolated 8 m layer")
        recorder.save("post_fault_snapshot.json", after_fault)
        fleet.remove_fault()
        remaining = {uav: poses[uav][:2] for uav in scenario.uav_ids
                     if uav != scenario.fault_id}
        targets = targets_for(scenario, initial, recorder,
                              worker_python, fleet.assert_healthy)
        movers = [uav for uav in scenario.uav_ids if uav != scenario.fault_id
                  and distance(remaining[uav], targets[uav]) > 0.2]
        for start in range(0, len(movers), 4):
            fleet.assert_healthy()
            current = fleet.snapshot()["poses_task"]
            scenario.validate_targets({uav: current[uav][:2] for uav in targets}, targets,
                                      current[scenario.fault_id][:2])
            batch = movers[start:start + 4]
            fleet.send_goal("MOVE_TO", {uav: (*targets[uav], scenario.normal_height)
                                        for uav in batch})
        fleet.wait_connected()
        final = fleet.snapshot()
        recorder.save("final_snapshot.json", final)
        summary = {"status": "REAL_LOGICAL_GRAPH_RECOVERED",
                   "uav_count": len(scenario.uav_ids),
                   "fault_time_monotonic": fleet.fault_time,
                   "min_3d_distance_m": fleet._last_min_distance,
                   "recovery_duration_s": time.monotonic() - fleet.fault_time,
                   "final_clusters": len(components(
                       {uav: pose[:2] for uav, pose in final["poses_task"].items()
                        if uav != scenario.fault_id}, scenario.radius)),
                   "targets_task_xy": targets,
                   "survivor_displacement_m": {uav: distance(initial[uav], final["poses_task"][uav][:2])
                                               for uav in scenario.uav_ids if uav != scenario.fault_id},
                   "survivor_goal_error_m": {uav: distance(targets[uav], final["poses_task"][uav][:2])
                                             for uav in targets}}
        recorder.save("summary.json", summary)
        return summary
    except BaseException as error:
        recorder.event("run_failed", error=str(error))
        fleet.hold_all()
        recorder.save("summary.json", {"status": "FAILED", "reason": str(error)})
        raise
    finally:
        fleet.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--worker-python", default=sys.executable)
    parser.add_argument("--snapshot", type=Path, help="offline measured ENU poses JSON")
    parser.add_argument("--targets-json", type=Path, help="offline fixture only, never flight")
    parser.add_argument("--execute", action="store_true", help="enable gated ROS real/SITL control")
    parser.add_argument("--identity", type=Path)
    parser.add_argument("--deployment", type=Path)
    parser.add_argument("--approval", type=Path)
    args = parser.parse_args()
    if args.execute:
        if (not all((args.identity, args.deployment, args.approval))
                or args.snapshot or args.targets_json):
            parser.error("flight requires identity/deployment/approval; no offline fixture allowed")
    elif not args.snapshot:
        parser.error("offline mode requires --snapshot; flight requires explicit --execute")
    scenario = Scenario.load(args.scenario)
    authorization = None
    if args.execute:
        authorization = verify_approval(scenario, args.scenario, args.identity,
                                        args.deployment, args.approval)
    recorder = Recorder(args.outdir)
    try:
        recorder.save("provenance.json", provenance(args.scenario, args.identity))
        frozen = {"scenario": args.scenario.read_text(encoding="utf-8")}
        if authorization:
            frozen["identity"] = args.identity.read_text(encoding="utf-8")
            for key in ("topology_file", "calibration_file", "formation_file"):
                frozen[key] = Path(authorization["deployment"][key]).read_text(encoding="utf-8")
            recorder.save("flight_authorization.json", authorization)
        recorder.save("frozen_configs.json", frozen)
        if authorization:
            summary = run_flight(scenario, args.identity, recorder, args.worker_python)
        else:
            summary = run_offline(scenario, args.snapshot, args.targets_json,
                                  recorder, args.worker_python)
        print(json.dumps(summary, indent=2))
    except BaseException as error:
        recorder.event("run_failed", error=str(error))
        recorder.save("summary.json", {"status": "FAILED", "reason": str(error)})
        raise
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
