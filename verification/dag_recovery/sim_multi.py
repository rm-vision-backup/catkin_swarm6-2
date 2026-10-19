#!/usr/bin/env python3
"""Ten-aircraft PX4 SITL experiment with three or four controlled logical exits."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

from core import _pair, components, distance, segment_distance
from pipeline import Recorder, digest, provenance, run_solver


@dataclass(frozen=True)
class SimCase:
    uav_ids: Tuple[str, ...]
    fault_ids: Tuple[str, ...]
    layout: Dict[str, Tuple[float, float]]
    lower: Tuple[float, float]
    upper: Tuple[float, float]
    radius: float
    edge_margin: float
    min_separation: float
    layout_tolerance: float
    normal_height: float
    fault_height: float
    altitude_tolerance: float
    seed: int
    # 散布起步（编队演示）：初始图允许多块；见 validate_layout 分支与 run_sitl 的
    # simultaneous 流程。默认 False = 原有"初始连通"语义，向后兼容。
    initial_scattered: bool = False

    @property
    def fault_id(self):
        return self.fault_ids[0]

    @property
    def survivors(self):
        return tuple(uav for uav in self.uav_ids if uav not in self.fault_ids)

    @classmethod
    def load(cls, path: Path, faults: int):
        raw = json.loads(path.read_text(encoding="utf-8"))
        if "max_displacement_m" in raw:
            raise ValueError("remove legacy max_displacement_m from the SITL scenario")
        lower, upper = raw["bounds_task_xy"]
        scenario = cls(
            uav_ids=tuple(raw["uav_ids"]), fault_ids=tuple(raw["fault_sets"][str(faults)]),
            layout={uav: _pair(pos) for uav, pos in raw["layout_task_xy"].items()},
            lower=_pair(lower), upper=_pair(upper),
            radius=float(raw["comm_radius_m"]), edge_margin=float(raw["edge_margin_m"]),
            min_separation=float(raw["min_separation_m"]),
            layout_tolerance=float(raw["layout_tolerance_m"]),
            normal_height=float(raw["normal_height_m"]),
            fault_height=float(raw["fault_height_m"]),
            altitude_tolerance=float(raw["altitude_tolerance_m"]), seed=int(raw["seed"]),
            initial_scattered=str(raw.get("initial_graph", "connected")).lower() == "scattered",
        )
        allowed_fault_counts = (0, 3, 4) if scenario.initial_scattered else (3, 4)
        if (len(scenario.uav_ids) != 10 or len(set(scenario.uav_ids)) != 10
                or len(scenario.fault_ids) not in allowed_fault_counts
                or len(set(scenario.fault_ids)) != len(scenario.fault_ids)
                or set(scenario.layout) != set(scenario.uav_ids)
                or not set(scenario.fault_ids).issubset(scenario.uav_ids)
                or (scenario.radius, scenario.edge_margin, scenario.min_separation,
                    scenario.normal_height, scenario.fault_height)
                != (8.0, 1.0, 4.0, 15.0, 8.0)
                or not 0 < scenario.layout_tolerance <= 1.0
                or not 0 < scenario.altitude_tolerance <= 0.7
                or scenario.seed < 0
                or any(scenario.lower[i] + 6 >= scenario.upper[i] for i in range(2))):
            raise ValueError("unsafe or malformed ten-aircraft SITL configuration")
        scenario.validate_layout(scenario.layout)
        return scenario

    def within_bounds(self, point):
        return all(self.lower[i] + 3 <= point[i] <= self.upper[i] - 3
                   for i in range(2))

    def validate_layout(self, positions):
        if set(positions) != set(self.uav_ids):
            raise ValueError("incomplete ten-aircraft layout")
        if not all(self.within_bounds(_pair(point)) for point in positions.values()):
            raise ValueError("UAV outside SITL inner bounds")
        for index, first in enumerate(self.uav_ids):
            for second in self.uav_ids[index + 1:]:
                if distance(positions[first], positions[second]) < self.min_separation:
                    raise ValueError("SITL start aircraft too close")
        if self.initial_scattered:
            # 出生态（散布）：必须"多块"，否则编队阶段没有意义。受损用例的
            # "编队后受损态"由 validate_damage_state 校验（阶段不同，状态不同）。
            if len(components(positions, self.radius - self.edge_margin)) < 3:
                raise ValueError("scattered initial graph needs at least three components")
            return
        if len(components(positions, self.radius - self.edge_margin)) != 1:
            raise ValueError("ten-aircraft graph not connected before damage")
        remaining = {uav: positions[uav] for uav in self.survivors}
        if len(components(remaining, self.radius + self.edge_margin)) != 2:
            raise ValueError("fault set does not split graph into exactly two components")

    def validate_damage_state(self, positions):
        """编队完成后的受损态（散布场景的训练样本）：全体在严格半径内连通，
        且每架受损机至少邻接一架幸存机——原仓库 k-hop 损伤注意力要求该可达性。"""
        if len(components(positions, self.radius - self.edge_margin)) != 1:
            raise ValueError("assembled formation is not robustly connected")
        for fault in self.fault_ids:
            nearest = min(distance(positions[fault], positions[uav])
                          for uav in self.uav_ids if uav not in self.fault_ids)
            if nearest > self.radius:
                raise ValueError("damaged aircraft unreachable for damage attention")

    def validate_fixture(self, positions):
        self.validate_layout(positions)
        for uav in self.uav_ids:
            if distance(positions[uav], self.layout[uav]) > self.layout_tolerance:
                raise ValueError("SITL spawn differs from approved ten-aircraft layout: " + uav)

    def validate_targets(self, starts, targets, fault_positions, batches=None):
        # batches：恢复执行中段复验时传入"故障后冻结的分组"。分组判据（恰两组、
        # 每组 ≤4）只对批次下发前的静止状态有意义；首块开始靠拢后按当前位置重算
        # 必然并组（这正是本实验要发生的事），故中段复验只做间距/边界检查。
        if batches is None:
            batches = self.movement_batches(starts)
        if (set(starts) != set(self.survivors) or set(targets) != set(self.survivors)
                or set(fault_positions) != set(self.fault_ids)):
            raise ValueError("DAGL targets do not match ten-aircraft failure set")
        goals = {uav: _pair(targets[uav]) for uav in self.survivors}
        for index, first in enumerate(self.survivors):
            for second in self.survivors[index + 1:]:
                gap = distance(goals[first], goals[second])
                if gap < self.min_separation:
                    raise ValueError("DAGL goals for %s/%s are only %.2f m apart (minimum %.2f m)"
                                     % (first, second, gap, self.min_separation))
        for uav in self.survivors:
            start, goal = _pair(starts[uav]), goals[uav]
            if not self.within_bounds(goal):
                raise ValueError("DAGL goal outside SITL inner bounds: " + uav)
            for fault in self.fault_ids:
                if math.hypot(segment_distance(start, goal, fault_positions[fault],
                                               fault_positions[fault]),
                              self.normal_height - self.fault_height) < self.min_separation:
                    raise ValueError("trajectory too close to physically present failed aircraft")
        if len(components(targets, self.radius - self.edge_margin)) != 1:
            raise ValueError("DAGL survivor goals do not reconnect with margin")
        if self.initial_scattered:
            # 同时机动模式：同上，直线同步接近模型不适用（安全性由 EGO +
            # 运行时三维间距监控兜底），规划层只查目标间距/连通/边界/故障机间距。
            return
        current = dict(starts)
        for batch in batches:
            for index, first in enumerate(self.survivors):
                first_start = current[first]
                first_end = targets[first] if first in batch else first_start
                for second in self.survivors[index + 1:]:
                    second_start = current[second]
                    second_end = targets[second] if second in batch else second_start
                    if synchronous_distance(first_start, first_end, second_start,
                                            second_end) < self.min_separation:
                        raise ValueError("SITL batched trajectories violate separation")
            current.update({uav: targets[uav] for uav in batch})

    def movement_batches(self, positions):
        groups = components({uav: positions[uav] for uav in self.survivors},
                            self.radius + self.edge_margin)
        if len(groups) != 2 or any(len(group) > 4 for group in groups):
            raise ValueError("SITL requires two survivor groups of at most four")
        return tuple(tuple(uav for uav in self.survivors if uav in group)
                     for group in groups)

    def validate_recovery_step(self, starts, targets, fault_positions):
        """全体幸存机同轮运动的几何门禁；中间点允许尚未连通。

        同步直线检查只是候选路径筛选，实际 EGO 曲线仍由全程 watchdog 监控。
        不调用 movement_batches：靠拢后分量可以合并，不再是执行批次。
        """
        if (set(starts) != set(self.survivors) or set(targets) != set(self.survivors)
                or set(fault_positions) != set(self.fault_ids)):
            raise ValueError("recovery step must include every survivor and all fault positions")
        starts = {uav: _pair(point) for uav, point in starts.items()}
        targets = {uav: _pair(point) for uav, point in targets.items()}
        faults = [_pair(point) for point in fault_positions.values()]
        for uav in self.survivors:
            if not self.within_bounds(starts[uav]) or not self.within_bounds(targets[uav]):
                raise ValueError("recovery step outside SITL inner bounds: " + uav)
            for fault in faults:
                if math.hypot(segment_distance(starts[uav], targets[uav], fault, fault),
                              self.normal_height - self.fault_height) < self.min_separation:
                    raise ValueError("recovery step too close to failed aircraft")
        for index, first in enumerate(self.survivors):
            for second in self.survivors[index + 1:]:
                if synchronous_distance(starts[first], targets[first], starts[second],
                                        targets[second]) < self.min_separation:
                    raise ValueError("simultaneous recovery trajectories violate separation")

    def assembly_input(self, positions):
        """编队阶段输入：fault_ids 为空，目标 = 全体十机同时机动连成一张图。"""
        payload = self.solver_input(positions)
        payload["fault_ids"] = []
        return payload

    def validate_assembly(self, starts, targets):
        """编队目标门禁：目标连通、间距/边界，且"全体同批"轨迹间距达标。"""
        if set(starts) != set(self.uav_ids) or set(targets) != set(self.uav_ids):
            raise ValueError("assembly targets must cover all ten aircraft")
        goals = {uav: _pair(targets[uav]) for uav in self.uav_ids}
        for index, first in enumerate(self.uav_ids):
            for second in self.uav_ids[index + 1:]:
                gap = distance(goals[first], goals[second])
                if gap < self.min_separation:
                    raise ValueError("assembly goals for %s/%s are only %.2f m apart"
                                     % (first, second, gap))
        for uav in self.uav_ids:
            if not self.within_bounds(goals[uav]):
                raise ValueError("assembly goal outside SITL inner bounds: " + uav)
        if len(components(goals, self.radius - self.edge_margin)) != 1:
            raise ValueError("assembly goals do not reconnect with margin")
        # 不再做"直线同步接近"检查：散布->聚合的直线路径必然穿越中心区域（实测
        # 最近对 0.8–1.9 m），该模型只适用于 A/B 的少量定时批次。同时编队的安全性
        # 由 EGO 机载避碰 + 运行时三维间距监控（越限即中止并 HOLD）兜底；
        # 规划层只保证目标间距/连通/边界。

    def solver_input(self, positions):
        if set(positions) != set(self.uav_ids):
            raise ValueError("missing DAGL initial aircraft position")
        return {"simulation_only": True, "uav_ids": list(self.uav_ids),
                "fault_ids": list(self.fault_ids),
                "positions_task_xy": {uav: list(_pair(positions[uav])) for uav in self.uav_ids},
                "bounds_task_xy": [list(self.lower), list(self.upper)],
                "comm_radius_m": self.radius, "seed": self.seed}


def synchronous_distance(first_start, first_end, second_start, second_end):
    relative_start = (first_start[0] - second_start[0],
                      first_start[1] - second_start[1])
    relative_velocity = (first_end[0] - first_start[0] - second_end[0] + second_start[0],
                         first_end[1] - first_start[1] - second_end[1] + second_start[1])
    speed_squared = sum(value * value for value in relative_velocity)
    instant = (max(0.0, min(1.0, -sum(relative_start[index] * relative_velocity[index]
                                      for index in range(2)) / speed_squared))
               if speed_squared else 0.0)
    return math.hypot(*(relative_start[index] + instant * relative_velocity[index]
                        for index in range(2)))


def plan(scenario, initial, recorder, python, health=lambda: None, fixture=None,
         checkpoint=None):
    payload = scenario.solver_input(initial)
    if fixture is None:
        if checkpoint is None:
            raise ValueError("ten-aircraft DAGL requires --checkpoint")
        payload["checkpoint"] = str(checkpoint.resolve())
        result = run_solver(payload, recorder, python=python, timeout_s=120.0, healthy=health)
    else:
        result = json.loads(fixture.read_text(encoding="utf-8"))
        recorder.save("dag_input.json", payload)
        recorder.save("dag_output.json", result)
        recorder.event("fixture_only", file=str(fixture.resolve()))
    targets = {uav: _pair(pos) for uav, pos in result["targets_task_xy"].items()}
    if set(targets) == set(scenario.survivors):
        separation = min(distance(targets[first], targets[second])
                         for index, first in enumerate(scenario.survivors)
                         for second in scenario.survivors[index + 1:])
        displacement = max(distance(initial[uav], targets[uav])
                           for uav in scenario.survivors)
        metrics = {"min_target_separation_m": separation,
                   "max_target_displacement_m": displacement,
                   "goal_components_with_margin": len(components(
                       targets, scenario.radius - scenario.edge_margin)),
                   "min_allowed_separation_m": scenario.min_separation}
        recorder.save("candidate_metrics.json", metrics)
        recorder.event("candidate_metrics", **metrics)
    batches = (tuple(scenario.survivors),) if scenario.initial_scattered else None
    scenario.validate_targets({uav: initial[uav] for uav in scenario.survivors}, targets,
                              {uav: initial[uav] for uav in scenario.fault_ids},
                              batches=batches)
    scenario.validate_recovery_step(
        {uav: initial[uav] for uav in scenario.survivors}, targets,
        {uav: initial[uav] for uav in scenario.fault_ids})
    recorder.event("plan_validated", targets=targets)
    return targets


def plan_assembly(scenario, initial, recorder, python, health=lambda: None, checkpoint=None):
    """编队阶段（0 故障）推理：DAGL 让全体十机同时机动到连通拓扑。"""
    if checkpoint is None:
        raise ValueError("assembly requires --checkpoint")
    payload = scenario.assembly_input(initial)
    payload["checkpoint"] = str(checkpoint.resolve())
    result = run_solver(payload, recorder, python=python, timeout_s=120.0, healthy=health)
    recorder.save("assembly_dag_input.json", payload)
    recorder.save("assembly_dag_output.json", result)
    targets = {uav: _pair(pos) for uav, pos in result["targets_task_xy"].items()}
    scenario.validate_assembly(initial, targets)
    recorder.event("assembly_planned",
                   targets={uav: list(point) for uav, point in targets.items()})
    return targets


def recover_together(scenario, fleet, targets, recorder, step_m=2.0, max_rounds=30):
    """一次 DAGL 终点求解，全体幸存机按最新实测位置分多轮短程靠近。

    使用共同插值比例以保持候选路径的同步几何关系；每轮只发一个全员任务，
    等完成并稳定后再读位置。没有运动中抢占，也不重新调用 DAGL。
    """
    if not math.isfinite(step_m) or step_m <= 1.0 or max_rounds < 1:
        raise ValueError("recovery step must exceed 1 m arrival tolerance; positive round limit required")
    if set(targets) != set(scenario.survivors):
        raise ValueError("recovery targets must include every survivor")
    targets = {uav: _pair(point) for uav, point in targets.items()}
    stalled = 0
    final_goal_sent = False
    for round_index in range(max_rounds + 1):
        fleet.assert_healthy()
        snapshot = fleet.snapshot()
        starts = {uav: snapshot["poses_task"][uav][:2] for uav in scenario.survivors}
        remaining = max(distance(starts[uav], targets[uav]) for uav in scenario.survivors)
        if final_goal_sent and remaining <= 1.0:
            # 到点与图恢复分开验收；不能因候选终点连通就报告实测恢复。
            fleet.wait_connected()
            fleet.assert_healthy()
            final = fleet.snapshot()["poses_task"]
            remaining = max(distance(final[uav][:2], targets[uav])
                            for uav in scenario.survivors)
            if remaining > 1.0:
                raise RuntimeError("survivors drifted from final recovery targets")
            recorder.event("recovery_finished", rounds=round_index,
                           max_target_error_m=remaining)
            return {"recovery_rounds": round_index, "recovery_step_m": step_m,
                    "max_target_error_m": remaining}
        if round_index == max_rounds:
            raise RuntimeError("simultaneous recovery exceeded round limit")
        fraction = step_m / remaining if remaining > step_m else 1.0
        waypoints = {uav: tuple(starts[uav][axis]
                               + fraction * (targets[uav][axis] - starts[uav][axis])
                               for axis in range(2)) for uav in scenario.survivors}
        scenario.validate_recovery_step(
            starts, waypoints,
            {uav: snapshot["poses_task"][uav][:2] for uav in scenario.fault_ids})
        recorder.event("recovery_round_started", round=round_index + 1,
                       starts_task_xy=starts, waypoints_task_xy=waypoints,
                       final_targets_task_xy=targets, max_target_error_m=remaining,
                       components=components(starts, scenario.radius))
        fleet.send_goal("MOVE_TO", {uav: (*waypoints[uav], scenario.normal_height)
                                    for uav in scenario.survivors}, timeout_s=60.0)
        final_goal_sent = fraction == 1.0
        actual = fleet.snapshot()["poses_task"]
        error = max(distance(actual[uav][:2], targets[uav]) for uav in scenario.survivors)
        recorder.event("recovery_round_completed", round=round_index + 1,
                       poses_task=actual, max_target_error_m=error,
                       components=components({uav: actual[uav][:2]
                                              for uav in scenario.survivors}, scenario.radius))
        stalled = stalled + 1 if remaining - error < 0.1 else 0
        if stalled >= 3:
            raise RuntimeError("simultaneous recovery stalled for three rounds")


def run_sitl(scenario, identity, recorder, python, checkpoint):
    from ros_adapter import RosFleet

    # 散布起步仍控制爬升/编队时序；恢复阶段统一使用全体幸存机短程同轮运动。
    # 排程已由实验用 gcs.launch 关闭，实际 START 存在服务分发时差。
    simultaneous = scenario.initial_scattered
    fleet = RosFleet(scenario, identity, recorder.run_dir.name, recorder,
                     max_batch=len(scenario.uav_ids))
    try:
        fleet.wait_ready(timeout_s=90)
        deadline = time.monotonic() + 120.0
        stable_since = None
        while time.monotonic() < deadline:
            first = fleet.snapshot()
            if all(abs(pose[2] - 5.0) <= scenario.altitude_tolerance
                   for pose in first["poses_task"].values()):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 2.0:
                    break
            else:
                stable_since = None
            time.sleep(0.2)
        else:
            raise RuntimeError("SITL soft takeoff did not stabilize all ten aircraft at 5 m")
        initial = {uav: pose[:2] for uav, pose in first["poses_task"].items()}
        scenario.validate_fixture(initial)
        recorder.save("takeoff_snapshot.json", first)
        fleet.activate()

        def climb_to_layer(uavs):
            poses = fleet.snapshot()["poses_task"]
            fleet.send_goal("MOVE_TO", {uav: (*poses[uav][:2], scenario.normal_height)
                                        for uav in uavs})

        if simultaneous:
            climb_to_layer(scenario.uav_ids)
        else:
            for start in range(0, 10, 4):
                climb_to_layer(scenario.uav_ids[start:start + 4])
        before = fleet.snapshot()
        scenario.validate_fixture({uav: pose[:2] for uav, pose in before["poses_task"].items()})
        if any(abs(pose[2] - scenario.normal_height) > scenario.altitude_tolerance
               for pose in before["poses_task"].values()):
            raise RuntimeError("not all SITL aircraft reached 15 m")
        recorder.save("pre_fault_snapshot.json", before)
        if simultaneous:
            # 编队阶段：初始图不连通 -> DAGL（0 故障）算目标 -> 全体同时机动成图。
            formation = plan_assembly(
                scenario, {uav: pose[:2] for uav, pose in before["poses_task"].items()},
                recorder, python, fleet.assert_healthy, checkpoint=checkpoint)
            fleet.send_goal("MOVE_TO", {uav: (*formation[uav], scenario.normal_height)
                                        for uav in scenario.uav_ids})
            fleet.wait_connected(timeout_s=150.0)
            before = fleet.snapshot()
            recorder.save("assembly_snapshot.json", before)
            fleet.send_goal("FAULT_EXIT", {uav: (*before["poses_task"][uav][:2],
                                                 scenario.fault_height)
                                           for uav in scenario.fault_ids})
        else:
            for start in range(0, len(scenario.fault_ids), 4):
                batch = scenario.fault_ids[start:start + 4]
                fleet.send_goal("FAULT_EXIT", {uav: (*before["poses_task"][uav][:2],
                                                       scenario.fault_height) for uav in batch})
        after = fleet.snapshot()
        if not simultaneous:
            # 散布模式下此刻机群位于编队目标处（≠初始布局），布局一致性检查仅对
            # "初始连通"流程有意义；编队/重组位置由 validate_assembly/validate_targets 把关。
            scenario.validate_fixture({uav: pose[:2] for uav, pose in after["poses_task"].items()})
        recorder.save("post_fault_snapshot.json", after)
        fleet.remove_fault()
        targets = plan(scenario, {uav: pose[:2] for uav, pose in after["poses_task"].items()},
                       recorder, python, fleet.assert_healthy, checkpoint=checkpoint)
        recovery = recover_together(scenario, fleet, targets, recorder)
        final = fleet.snapshot()
        recorder.save("final_snapshot.json", final)
        summary = {"status": "SITL_DAGL_EGO_GRAPH_RECOVERED", "fault_ids": scenario.fault_ids,
                   "recovery_duration_s": time.monotonic() - fleet.fault_time,
                   "min_3d_separation_m": fleet._last_min_distance,
                   "targets_task_xy": targets,
                   "recovery_mode": "all_survivors_short_steps",
                   "recovery_dagl_solves": 1, **recovery}
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
    parser.add_argument("--scenario", type=Path,
                        default=Path(__file__).with_name("scenarios") / "10.sitl.json")
    parser.add_argument("--faults", type=int, choices=(3, 4), required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--worker-python", default=sys.executable)
    parser.add_argument("--checkpoint", type=Path, help="approved DAGL-10 model.pt")
    parser.add_argument("--snapshot", type=Path, help="offline ENU snapshot")
    parser.add_argument("--targets-fixture", type=Path, help="offline test only; never SITL")
    parser.add_argument("--sitl", action="store_true", help="control PX4 SITL on localhost")
    parser.add_argument("--identity", type=Path)
    parser.add_argument("--deployment", type=Path)
    args = parser.parse_args()
    if args.sitl:
        if args.snapshot or args.targets_fixture or not args.identity or not args.deployment:
            parser.error("SITL requires identity/deployment; offline fixtures are prohibited")
        if os.environ.get("ROS_MASTER_URI") != "http://localhost:11310":
            parser.error("SITL must use the local GCS ROS master on port 11310")
        manifest = json.loads(args.deployment.read_text(encoding="utf-8"))
        if (manifest.get("simulation_only") is not True
                or Path(manifest["identity_file"]).resolve() != args.identity.resolve()
                or Path(manifest["scenario_file"]).resolve() != args.scenario.resolve()
                or not Path(manifest["topology_file"]).is_file()):
            parser.error("SITL-only deployment manifest does not match selected inputs")
    elif not args.snapshot:
        parser.error("offline mode requires --snapshot; SITL needs explicit --sitl")
    if not args.targets_fixture and (args.checkpoint is None or not args.checkpoint.is_file()):
        parser.error("DAGL requires --checkpoint pointing to an approved model.pt")
    scenario = SimCase.load(args.scenario, args.faults)
    recorder = Recorder(args.outdir)
    try:
        provenance_data = provenance(args.scenario, args.identity)
        for name in ("sim_multi.py", "sim_setup.py", "sim_onboard.launch", "run_sim10.sh"):
            path = Path(__file__).with_name(name)
            provenance_data["file_sha256"][str(path.resolve())] = digest(path)
        if args.checkpoint is not None:
            for path in (args.checkpoint, args.checkpoint.with_name("model.json"),
                         Path(__file__).with_name("train_dagl10.py")):
                provenance_data["file_sha256"][str(path.resolve())] = digest(path)
        recorder.save("provenance.json", provenance_data)
        recorder.save("simulation_config.json", json.loads(args.scenario.read_text(encoding="utf-8")))
        if args.sitl:
            result = run_sitl(scenario, args.identity, recorder, args.worker_python,
                              args.checkpoint)
        else:
            from core import normalize_positions
            snap = json.loads(args.snapshot.read_text(encoding="utf-8"))
            poses = normalize_positions(snap["poses_enu"], float(snap["heading_rad"]))
            scenario.validate_fixture({uav: pose[:2] for uav, pose in poses.items()})
            targets = plan(scenario, {uav: pose[:2] for uav, pose in poses.items()},
                           recorder, args.worker_python, fixture=args.targets_fixture,
                           checkpoint=args.checkpoint)
            result = {"status": "OFFLINE_ONLY", "fault_ids": scenario.fault_ids,
                      "survivor_clusters_after_fault": len(components(
                          {uav: poses[uav][:2] for uav in scenario.survivors}, scenario.radius)),
                      "planned_clusters": len(components(targets, scenario.radius)),
                      "targets_task_xy": targets}
            recorder.save("summary.json", result)
        print(json.dumps(result, indent=2))
    except BaseException as error:
        recorder.event("run_failed", error=str(error))
        recorder.save("summary.json", {"status": "FAILED", "reason": str(error)})
        raise
    finally:
        recorder.close()


if __name__ == "__main__":
    main()
