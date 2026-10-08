#!/usr/bin/env python3
"""三机同时交叉 SITL 编排（在现有 15 机 PX4 SITL 环境）。

场景：三机从三角形顶角向中心 (cx,cy) 同时交叉——
  UAV1 沿 x+（y=cy）、UAV2 沿 y+（x=cx）、UAV3 沿 45° 对角（x+ 且 y+），
  三线在中心 (cx,cy,z) 交汇（各起终点距中心 arm）。

流程：preflight → 串行 staging → 三线程 barrier START → 采集 → HOLD。
判定：三机 COMPLETED、三对最小距离 ≥ min_clearance、速度 ≤ max_speed、
  加速度 ≤ max_acceleration、横向振荡 ≤ max_sign_changes、intent 互见。

用法：ROS_MASTER_URI=http://localhost:11310 python3 verification/ego_smoke/triple_cross_15sitl.py \
  --execute --confirm-sitl I_UNDERSTAND_SITL
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from swarm_uav_interfaces.msg import Pose3DYaw, TaskAssignment, UavTaskState, UavTrajectoryIntent
from swarm_uav_interfaces.srv import (
    UavHold,
    UavHoldRequest,
    UavTask,
    UavTaskControl,
    UavTaskControlRequest,
    UavTaskRequest,
)

PROTOCOL_VERSION = "1.0"
GROUP_ID = "GroupA"
GCS_MASTER = "http://localhost:11310"
WS = Path(__file__).resolve().parents[2]  # verification/ego_smoke → 工作空间根
TERMINAL = {"COMPLETED", "FAILED", "HELD"}


@dataclass(frozen=True)
class Vehicle:
    name: str
    master_port: int

    @property
    def master_uri(self) -> str:
        return "http://localhost:%d" % self.master_port


@dataclass
class Sample:
    mono_s: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float


@dataclass
class VehicleData:
    state: Optional[State] = None
    state_mono_s: float = 0.0
    odom: Optional[Odometry] = None
    odom_mono_s: float = 0.0
    samples: List[Sample] = field(default_factory=list)
    task_states: Dict[str, UavTaskState] = field(default_factory=dict)
    intent_count: int = 0
    intent_ids: set = field(default_factory=set)
    intent_ages_s: List[float] = field(default_factory=list)


class TripleRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        names = [args.uav_a, args.uav_b, args.uav_c]
        indexes = [args.index_a, args.index_b, args.index_c]
        self.vehicles = [Vehicle(n, 11310 + i) for n, i in zip(names, indexes)]
        self.data = {v.name: VehicleData() for v in self.vehicles}
        self.lock = threading.RLock()
        self.mission_id = "triple-15sitl-%s" % uuid.uuid4().hex[:10]
        self.cross_started_mono_s: Optional[float] = None
        for vehicle in self.vehicles:
            rospy.Subscriber("/%s/mavros/state" % vehicle.name, State,
                             self._on_state, callback_args=vehicle.name, queue_size=1)
            rospy.Subscriber("/%s/mavros/local_position/odom" % vehicle.name, Odometry,
                             self._on_odom, callback_args=vehicle.name, queue_size=50)
            rospy.Subscriber("/%s/uav_task_state" % vehicle.name, UavTaskState,
                             self._on_task_state, callback_args=vehicle.name, queue_size=20)
            rospy.Subscriber("/%s/trajectory_intent" % vehicle.name, UavTrajectoryIntent,
                             self._on_intent, callback_args=vehicle.name, queue_size=50)

    def _on_state(self, msg, name):
        with self.lock:
            self.data[name].state = msg
            self.data[name].state_mono_s = time.monotonic()

    def _on_odom(self, msg, name):
        now = time.monotonic()
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            self.data[name].odom = msg
            self.data[name].odom_mono_s = now
            if self.cross_started_mono_s is not None:
                self.data[name].samples.append(Sample(now, p.x, p.y, p.z, v.x, v.y, v.z))

    def _on_task_state(self, msg, name):
        if msg.mission_id != self.mission_id:
            return
        with self.lock:
            self.data[name].task_states[msg.command_id] = msg

    def _on_intent(self, msg, name):
        now_ros = rospy.Time.now().to_sec()
        with self.lock:
            self.data[name].intent_count += 1
            self.data[name].intent_ids.add(int(msg.traj_id))
            if math.isfinite(msg.stamp):
                self.data[name].intent_ages_s.append(now_ros - msg.stamp)

    def scenario(self) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
        cx, cy, z, arm = (self.args.center_x, self.args.center_y,
                          self.args.altitude, self.args.arm)
        # 三顶角 → 中心 (cx,cy) 交叉：A 沿 x+、B 沿 y+、C 沿 45° 反向（(cx+arm,cy+arm)→
        # (cx-arm,cy-arm)），三线仍在中心交汇；顶角分散避免 staging 途中互相接近。
        return {
            self.vehicles[0].name: {"stage": (cx - arm, cy, z), "goal": (cx + arm, cy, z)},
            self.vehicles[1].name: {"stage": (cx, cy - arm, z), "goal": (cx, cy + arm, z)},
            self.vehicles[2].name: {"stage": (cx + arm, cy + arm, z),
                                    "goal": (cx - arm, cy - arm, z)},
        }

    def preflight(self) -> None:
        if os.environ.get("ROS_MASTER_URI", GCS_MASTER) != GCS_MASTER:
            raise RuntimeError("请在 GCS_A Master 运行：ROS_MASTER_URI=%s" % GCS_MASTER)
        if len(set(v.name for v in self.vehicles)) != 3:
            raise RuntimeError("三机必须互不相同")
        deadline = time.monotonic() + self.args.preflight_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = time.monotonic()
            with self.lock:
                ready = all(d.state is not None and d.odom is not None and
                            now - d.state_mono_s < 2.0 and now - d.odom_mono_s < 1.0
                            for d in self.data.values())
            if ready:
                break
            rate.sleep()
        else:
            raise RuntimeError("三机 MAVROS state/odometry 未在预检窗口内同时就绪")
        failures = []
        with self.lock:
            for vehicle in self.vehicles:
                state = self.data[vehicle.name].state
                if state is None or not state.connected:
                    failures.append("%s MAVROS disconnected" % vehicle.name)
                if state is None or not state.armed:
                    failures.append("%s not armed" % vehicle.name)
                if state is None or state.mode != "OFFBOARD":
                    failures.append("%s mode=%s" % (vehicle.name,
                                                    state.mode if state else "unknown"))
        if failures:
            raise RuntimeError("预检失败：" + "; ".join(failures))


    def _task_request(self, vehicle, command_id, command, target):
        req = UavTaskRequest()
        req.protocol_version = PROTOCOL_VERSION
        req.mission_id = self.mission_id
        req.group_id = GROUP_ID
        req.command_id = command_id
        req.uav_id = vehicle.name
        req.exec_target = vehicle.name
        req.command = command
        req.timeout_s = float(self.args.task_timeout)
        req.leader_id = ""
        assignment = TaskAssignment()
        assignment.uav_id = vehicle.name
        assignment.formation_follow = False
        assignment.target_id = ""
        assignment.target_pose = Pose3DYaw(x=target[0], y=target[1], z=target[2], yaw=0.0)
        if command == "FOLLOW_ROUTE":
            assignment.waypoints = [assignment.target_pose]
        req.assignment = assignment
        return req

    def prepare(self, vehicle, command_id, command, target):
        proxy = rospy.ServiceProxy("/%s/uav_task" % vehicle.name, UavTask)
        response = proxy(self._task_request(vehicle, command_id, command, target))
        if not response.accepted:
            raise RuntimeError("%s PREPARE rejected: %s %s" %
                               (vehicle.name, response.error_code, response.message))

    def start(self, vehicle, command_id):
        req = UavTaskControlRequest(
            protocol_version=PROTOCOL_VERSION, operation="START",
            mission_id=self.mission_id, command_id=command_id,
            uav_id=vehicle.name, exec_target=vehicle.name,
            reason="triple 15-SITL crossing test")
        response = rospy.ServiceProxy("/%s/uav_task_control" % vehicle.name,
                                      UavTaskControl)(req)
        if not response.accepted:
            raise RuntimeError("%s START rejected: %s %s" %
                               (vehicle.name, response.error_code, response.message))

    def wait_terminal(self, vehicle, command_id, timeout_s):
        deadline = time.monotonic() + timeout_s
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                state = self.data[vehicle.name].task_states.get(command_id)
            if state is not None and state.status in TERMINAL:
                return state
            rate.sleep()
        raise RuntimeError("%s command %s terminal timeout" % (vehicle.name, command_id))

    def stage(self):
        points = self.scenario()
        for vehicle in self.vehicles:
            command_id = "stage-%s" % vehicle.name.lower()
            self.prepare(vehicle, command_id, "FOLLOW_ROUTE", points[vehicle.name]["stage"])
            self.start(vehicle, command_id)
            state = self.wait_terminal(vehicle, command_id, self.args.task_timeout + 10.0)
            if state.status != "COMPLETED":
                raise RuntimeError("%s staging %s: %s %s" %
                                   (vehicle.name, state.status, state.error_code,
                                    state.message))

    def cross(self):
        points = self.scenario()
        command_ids = {}
        for vehicle in self.vehicles:
            command_id = "cross-%s" % vehicle.name.lower()
            command_ids[vehicle.name] = command_id
            self.prepare(vehicle, command_id, "FOLLOW_ROUTE", points[vehicle.name]["goal"])
        errors = []
        barrier = threading.Barrier(len(self.vehicles))

        def start_one(vehicle):
            try:
                barrier.wait(timeout=5.0)
                self.start(vehicle, command_ids[vehicle.name])
            except Exception as exc:
                errors.append(exc)

        self.cross_started_mono_s = time.monotonic()
        threads = [threading.Thread(target=start_one, args=(v,)) for v in self.vehicles]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        if errors:
            raise errors[0]
        results = {}
        for vehicle in self.vehicles:
            results[vehicle.name] = self.wait_terminal(
                vehicle, command_ids[vehicle.name], self.args.task_timeout + 10.0)
        return results


    def hold_all(self, reason):
        for vehicle in self.vehicles:
            try:
                req = UavHoldRequest(
                    protocol_version=PROTOCOL_VERSION, mission_id=self.mission_id,
                    command_id="safety-hold-%s" % vehicle.name.lower(),
                    uav_id=vehicle.name, exec_target=vehicle.name, reason=reason)
                rospy.ServiceProxy("/%s/uav_hold" % vehicle.name, UavHold)(req)
            except Exception as exc:
                rospy.logerr("%s HOLD failed: %s", vehicle.name, exc)

    def metrics(self, results):
        with self.lock:
            snapshot = {name: self.data[name] for name in self.data}
        names = [v.name for v in self.vehicles]
        pair_dist = {}
        for a_i, b_i in ((0, 1), (0, 2), (1, 2)):
            a_name, b_name = names[a_i], names[b_i]
            a_samples = list(snapshot[a_name].samples)
            b_samples = list(snapshot[b_name].samples)
            pairs = []
            j = 0
            for a in a_samples:
                while j + 1 < len(b_samples) and abs(b_samples[j + 1].mono_s - a.mono_s) < \
                        abs(b_samples[j].mono_s - a.mono_s):
                    j += 1
                if b_samples and abs(b_samples[j].mono_s - a.mono_s) <= 0.15:
                    pairs.append((a, b_samples[j]))
            pair_dist["%s-%s" % (a_name, b_name)] = (
                [math.sqrt((x.x - y.x) ** 2 + (x.y - y.y) ** 2 + (x.z - y.z) ** 2)
                 for x, y in pairs] if pairs else [])

        def lateral_of(name, sample):
            if name == self.vehicles[0].name:   # 沿 x → 横向 = y 相对 cy
                return sample.y - self.args.center_y
            if name == self.vehicles[1].name:   # 沿 y → 横向 = x 相对 cx
                return sample.x - self.args.center_x
            return (sample.x - sample.y)        # 45° 对角 → 法向 = x-y

        vehicles = {}
        for vehicle in self.vehicles:
            item = snapshot[vehicle.name]
            result = results.get(vehicle.name)
            samples = list(item.samples)
            speeds = [math.sqrt(s.vx ** 2 + s.vy ** 2 + s.vz ** 2) for s in samples]
            accelerations = []
            for prev, cur in zip(samples, samples[1:]):
                dt = cur.mono_s - prev.mono_s
                if dt >= 0.01:
                    accelerations.append(math.sqrt(
                        ((cur.vx - prev.vx) / dt) ** 2 +
                        ((cur.vy - prev.vy) / dt) ** 2 +
                        ((cur.vz - prev.vz) / dt) ** 2))
            lateral = [lateral_of(vehicle.name, s) for s in samples]
            vehicles[vehicle.name] = {
                "terminal_status": result.status if result else "MISSING",
                "error_code": result.error_code if result else "MISSING",
                "odom_samples": len(item.samples),
                "intent_messages": item.intent_count,
                "distinct_traj_ids": len(item.intent_ids),
                "max_abs_lateral_deviation_m": max(map(abs, lateral)) if lateral else None,
                "lateral_sign_changes": self._sign_changes(lateral),
                "max_speed_mps": max(speeds) if speeds else None,
                "max_acceleration_mps2": max(accelerations) if accelerations else None,
            }
        min_distances = {k: (min(v) if v else None) for k, v in pair_dist.items()}
        all_pairs_ok = all(v is not None and v >= self.args.min_clearance
                           for v in min_distances.values())
        passed = (len(next(iter(pair_dist.values()))) > 0 and all_pairs_ok and
                  all(result.status == "COMPLETED" for result in results.values()) and
                  all(item["intent_messages"] > 0 for item in vehicles.values()) and
                  all(item["max_speed_mps"] is not None and
                      item["max_speed_mps"] <= self.args.max_speed
                      for item in vehicles.values()) and
                  all(item["max_acceleration_mps2"] is not None and
                      item["max_acceleration_mps2"] <= self.args.max_acceleration
                      for item in vehicles.values()) and
                  all(item["lateral_sign_changes"] <= self.args.max_sign_changes
                      for item in vehicles.values()))
        return {
            "mission_id": self.mission_id,
            "passed": passed,
            "scenario_enu": self.scenario(),
            "min_pairwise_3d_distance_m": min_distances,
            "required_clearance_m": self.args.min_clearance,
            "max_allowed_speed_mps": self.args.max_speed,
            "max_allowed_acceleration_mps2": self.args.max_acceleration,
            "max_lateral_sign_changes": self.args.max_sign_changes,
            "vehicles": vehicles,
        }

    @staticmethod
    def _sign_changes(values, deadband=0.20):
        signs = [1 if value > deadband else -1 if value < -deadband else 0 for value in values]
        filtered = [value for value in signs if value]
        return sum(a != b for a, b in zip(filtered, filtered[1:]))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-sitl", default="")
    parser.add_argument("--uav-a", default="UAV1")
    parser.add_argument("--index-a", type=int, default=1)
    parser.add_argument("--uav-b", default="UAV2")
    parser.add_argument("--index-b", type=int, default=2)
    parser.add_argument("--uav-c", default="UAV3")
    parser.add_argument("--index-c", type=int, default=3)
    parser.add_argument("--center-x", type=float, default=10.0)
    parser.add_argument("--center-y", type=float, default=5.0)
    parser.add_argument("--altitude", type=float, default=12.0)
    parser.add_argument("--arm", type=float, default=8.0)
    parser.add_argument("--min-clearance", type=float, default=1.0)
    parser.add_argument("--max-sign-changes", type=int, default=2)
    parser.add_argument("--max-speed", type=float, default=2.5,
                        help="实测速度上限（与 ego 驱动 yield_max_velocity_mps=2.5 一致）")
    parser.add_argument("--max-acceleration", type=float, default=6.3)
    parser.add_argument("--task-timeout", type=float, default=90.0)
    parser.add_argument("--preflight-timeout", type=float, default=15.0)
    parser.add_argument("--output", default=str(WS / ".tmp/triple_cross_15sitl_result.json"))
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main():
    args = parse_args()
    rospy.init_node("triple_cross_15sitl_runner", anonymous=False)
    runner = TripleRunner(args)
    try:
        runner.preflight()
        print(json.dumps({"preflight": "PASS", "scenario_enu": runner.scenario()},
                         ensure_ascii=False, indent=2), flush=True)
        if not args.execute:
            print("DRY-RUN：未发送运动命令。执行需 --execute --confirm-sitl I_UNDERSTAND_SITL",
                  flush=True)
            return 0
        if args.confirm_sitl != "I_UNDERSTAND_SITL":
            raise RuntimeError("拒绝执行：--confirm-sitl 必须为 I_UNDERSTAND_SITL")
        runner.stage()
        results = runner.cross()
        metrics = runner.metrics(results)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n",
                          encoding="utf-8")
        print(json.dumps(metrics, ensure_ascii=False, indent=2), flush=True)
        print("result: %s (%s)" % ("PASS" if metrics["passed"] else "FAIL", output), flush=True)
        return 0 if metrics["passed"] else 2
    except Exception as exc:
        rospy.logerr("triple 15-SITL runner failed: %s", exc)
        return 1
    finally:
        if args.execute:
            runner.hold_all("triple SITL test finished")


if __name__ == "__main__":
    sys.exit(main())

