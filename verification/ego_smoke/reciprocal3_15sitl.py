#!/usr/bin/env python3
"""在现有 15 机 PX4 SITL 环境中编排三机同时交叉让路测试。

场景（ENU，中心 (cx, cy, z)）：
  UAV1 (STAND_ON, 低 ID)：stage (cx-arm, cy) -> goal (cx+arm, cy)   沿 x 东向
  UAV2 (YIELD)：stage (cx, cy-arm) -> goal (cx, cy+arm)             沿 y 北向
  UAV3 (YIELD)：stage (cx, cy+arm) -> goal (cx, cy-arm)             沿 y 南向（与 UAV2 对飞）

UAV2/UAV3 同走廊对飞、UAV1 横穿，形成多 peer 冲突：每个 YIELD 侧同时面对
两个 peer，检验 SafetyPredictor 最危险冲突选择、候选对全部邻机 clearance、
emergency 制动兜底。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
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
WS = Path("/home/yjq/catkin_swarm6-2")
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


class TriNodeRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.vehicles = [Vehicle(args.uav_a, 11310 + args.index_a),
                         Vehicle(args.uav_b, 11310 + args.index_b),
                         Vehicle(args.uav_c, 11310 + args.index_c)]
        self.data = {v.name: VehicleData() for v in self.vehicles}
        self.lock = threading.RLock()
        self.intent_probes: Dict[str, subprocess.Popen] = {}
        self.mission_id = "trinode-15sitl-%s" % uuid.uuid4().hex[:10]
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

    def _on_state(self, msg: State, name: str) -> None:
        with self.lock:
            self.data[name].state = msg
            self.data[name].state_mono_s = time.monotonic()

    def _on_odom(self, msg: Odometry, name: str) -> None:
        now = time.monotonic()
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        with self.lock:
            item = self.data[name]
            item.odom = msg
            item.odom_mono_s = now
            if self.cross_started_mono_s is not None:
                item.samples.append(Sample(now, p.x, p.y, p.z, v.x, v.y, v.z))

    def _on_task_state(self, msg: UavTaskState, name: str) -> None:
        if msg.mission_id != self.mission_id:
            return
        with self.lock:
            self.data[name].task_states[msg.command_id] = msg

    def _on_intent(self, msg: UavTrajectoryIntent, name: str) -> None:
        now_ros = rospy.Time.now().to_sec()
        with self.lock:
            item = self.data[name]
            item.intent_count += 1
            item.intent_ids.add(int(msg.traj_id))
            if math.isfinite(msg.stamp):
                item.intent_ages_s.append(now_ros - msg.stamp)

    def scenario(self) -> Dict[str, Tuple[float, float, float]]:
        cx, cy, z, arm = self.args.center_x, self.args.center_y, self.args.altitude, self.args.arm
        return {
            "%s_stage" % self.vehicles[0].name: (cx - arm, cy, z),
            "%s_goal" % self.vehicles[0].name: (cx + arm, cy, z),
            "%s_stage" % self.vehicles[1].name: (cx, cy - arm, z),
            "%s_goal" % self.vehicles[1].name: (cx, cy + arm, z),
            "%s_stage" % self.vehicles[2].name: (cx, cy + arm, z),
            "%s_goal" % self.vehicles[2].name: (cx, cy - arm, z),
        }

    def preflight(self) -> None:
        if os.environ.get("ROS_MASTER_URI", GCS_MASTER) != GCS_MASTER:
            raise RuntimeError("请在 GCS_A Master 运行：ROS_MASTER_URI=%s" % GCS_MASTER)
        names = [v.name for v in self.vehicles]
        if len(set(names)) != len(names):
            raise RuntimeError("三架测试机必须不同：%s" % names)

        deadline = time.monotonic() + self.args.preflight_timeout
        rate = rospy.Rate(10)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                ready = all(self.data[v.name].state and self.data[v.name].odom
                            for v in self.vehicles)
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

    def _onboard_env(self, vehicle: Vehicle) -> dict:
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = vehicle.master_uri
        env["ROS_HOSTNAME"] = "localhost"
        env["ROS_HOME"] = str(WS / ".ros_home")
        env["ROS_LOG_DIR"] = str(WS / ".ros_home/log")
        python_path = str(WS / "devel/lib/python3/dist-packages")
        env["PYTHONPATH"] = python_path + (":" + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
        return env

    def start_intent_probes(self) -> None:
        for vehicle in self.vehicles:
            process = subprocess.Popen(
                ["rostopic", "echo", "-n", "1", "/neighbor_intent"],
                env=self._onboard_env(vehicle), stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, start_new_session=True)
            self.intent_probes[vehicle.name] = process

    def neighbor_inputs_seen(self) -> Dict[str, bool]:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and any(
                process.poll() is None for process in self.intent_probes.values()):
            time.sleep(0.05)
        return {name: process.poll() == 0
                for name, process in self.intent_probes.items()}

    def stop_intent_probes(self) -> None:
        for process in self.intent_probes.values():
            if process.poll() is None:
                process.terminate()
        for process in self.intent_probes.values():
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        self.intent_probes = {}

    def _task_request(self, vehicle: Vehicle, command_id: str, command: str,
                      target: Tuple[float, float, float]) -> UavTaskRequest:
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

    def prepare(self, vehicle: Vehicle, command_id: str, command: str,
                target: Tuple[float, float, float]) -> None:
        proxy = rospy.ServiceProxy("/%s/uav_task" % vehicle.name, UavTask)
        response = proxy(self._task_request(vehicle, command_id, command, target))
        if not response.accepted:
            raise RuntimeError("%s PREPARE rejected: %s %s" %
                               (vehicle.name, response.error_code, response.message))

    def start(self, vehicle: Vehicle, command_id: str) -> None:
        req = UavTaskControlRequest(
            protocol_version=PROTOCOL_VERSION, operation="START",
            mission_id=self.mission_id, command_id=command_id,
            uav_id=vehicle.name, exec_target=vehicle.name,
            reason="15-SITL trinode crossing test")
        response = rospy.ServiceProxy("/%s/uav_task_control" % vehicle.name,
                                      UavTaskControl)(req)
        if not response.accepted:
            raise RuntimeError("%s START rejected: %s %s" %
                               (vehicle.name, response.error_code, response.message))

    def wait_terminal(self, vehicle: Vehicle, command_id: str, timeout_s: float) -> UavTaskState:
        deadline = time.monotonic() + timeout_s
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                state = self.data[vehicle.name].task_states.get(command_id)
            if state is not None and state.status in TERMINAL:
                return state
            rate.sleep()
        raise RuntimeError("%s command %s terminal timeout" % (vehicle.name, command_id))

    def stage(self) -> None:
        points = self.scenario()
        for vehicle in self.vehicles:
            command_id = "stage-%s" % vehicle.name.lower()
            self.prepare(vehicle, command_id, "FOLLOW_ROUTE",
                         points["%s_stage" % vehicle.name])
            self.start(vehicle, command_id)
            state = self.wait_terminal(vehicle, command_id, self.args.task_timeout + 10.0)
            if state.status != "COMPLETED":
                raise RuntimeError("%s staging %s: %s %s" %
                                   (vehicle.name, state.status, state.error_code, state.message))

    def cross(self) -> Dict[str, UavTaskState]:
        points = self.scenario()
        command_ids = {}
        for vehicle in self.vehicles:
            command_id = "cross-%s" % vehicle.name.lower()
            command_ids[vehicle.name] = command_id
            self.prepare(vehicle, command_id, "FOLLOW_ROUTE",
                         points["%s_goal" % vehicle.name])

        errors: List[Exception] = []
        barrier = threading.Barrier(len(self.vehicles))

        def start_one(vehicle: Vehicle) -> None:
            try:
                barrier.wait(timeout=5.0)
                self.start(vehicle, command_ids[vehicle.name])
            except Exception as exc:
                errors.append(exc)

        self.cross_started_mono_s = time.monotonic()
        threads = [threading.Thread(target=start_one, args=(v,)) for v in self.vehicles]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if errors:
            raise RuntimeError("concurrent START failed: %s" % errors)

        results = {}
        for vehicle in self.vehicles:
            results[vehicle.name] = self.wait_terminal(
                vehicle, command_ids[vehicle.name], self.args.task_timeout + 10.0)
        return results

    def hold_all(self, reason: str) -> None:
        for vehicle in self.vehicles:
            try:
                req = UavHoldRequest(
                    protocol_version=PROTOCOL_VERSION, mission_id=self.mission_id,
                    command_id="safety-hold-%s" % vehicle.name.lower(),
                    uav_id=vehicle.name, exec_target=vehicle.name, reason=reason)
                rospy.ServiceProxy("/%s/uav_hold" % vehicle.name, UavHold)(req)
            except Exception as exc:
                rospy.logerr("%s HOLD failed: %s", vehicle.name, exc)

    @staticmethod
    def _nearest_pair_samples(samples_a: List[Sample], samples_b: List[Sample]):
        pairs = []
        j = 0
        for a in samples_a:
            while j + 1 < len(samples_b) and abs(samples_b[j + 1].mono_s - a.mono_s) < \
                    abs(samples_b[j].mono_s - a.mono_s):
                j += 1
            if samples_b and abs(samples_b[j].mono_s - a.mono_s) <= 0.15:
                pairs.append((a, samples_b[j]))
        return pairs

    def metrics(self, results: Dict[str, UavTaskState],
                neighbor_inputs: Dict[str, bool]) -> Dict[str, object]:
        with self.lock:
            samples = {name: list(self.data[name].samples) for name in self.data}
            snapshot = {name: self.data[name] for name in self.data}

        all_horizontal = []
        all_spatial = []
        pair_dists = {}
        for i in range(len(self.vehicles)):
            for j in range(i + 1, len(self.vehicles)):
                a = self.vehicles[i].name
                b = self.vehicles[j].name
                pairs = self._nearest_pair_samples(samples[a], samples[b])
                horizontal = [math.hypot(x.x - y.x, x.y - y.y) for x, y in pairs]
                spatial = [math.sqrt((x.x - y.x) ** 2 + (x.y - y.y) ** 2 + (x.z - y.z) ** 2)
                           for x, y in pairs]
                pair_dists["%s-%s" % (a, b)] = {
                    "min_horizontal_m": min(horizontal) if horizontal else None,
                    "min_3d_m": min(spatial) if spatial else None,
                }
                all_horizontal.extend(horizontal)
                all_spatial.extend(spatial)

        vehicles = {}
        for vehicle in self.vehicles:
            item = snapshot[vehicle.name]
            result = results.get(vehicle.name)
            item_samples = list(item.samples)
            speeds = [math.sqrt(s.vx ** 2 + s.vy ** 2 + s.vz ** 2)
                      for s in item_samples]
            accelerations = []
            for previous, current in zip(item_samples, item_samples[1:]):
                dt = current.mono_s - previous.mono_s
                if dt >= 0.01:
                    accelerations.append(math.sqrt(
                        ((current.vx - previous.vx) / dt) ** 2 +
                        ((current.vy - previous.vy) / dt) ** 2 +
                        ((current.vz - previous.vz) / dt) ** 2))
            vehicles[vehicle.name] = {
                "terminal_status": result.status if result else "MISSING",
                "error_code": result.error_code if result else "MISSING",
                "odom_samples": len(item.samples),
                "intent_messages": item.intent_count,
                "neighbor_intent_input_seen": neighbor_inputs.get(vehicle.name, False),
                "distinct_traj_ids": len(item.intent_ids),
                "max_intent_age_s": max(item.intent_ages_s) if item.intent_ages_s else None,
                "max_speed_mps": max(speeds) if speeds else None,
                "max_acceleration_mps2": max(accelerations) if accelerations else None,
            }

        passed = (len(all_horizontal) > 0 and min(all_horizontal) >= self.args.min_clearance and
                  all(result.status == "COMPLETED" for result in results.values()) and
                  all(item["intent_messages"] > 0 for item in vehicles.values()) and
                  all(item["neighbor_intent_input_seen"] for item in vehicles.values()) and
                  all(item["max_speed_mps"] is not None and
                      item["max_speed_mps"] <= self.args.max_speed
                      for item in vehicles.values()) and
                  all(item["max_acceleration_mps2"] is not None and
                      item["max_acceleration_mps2"] <= self.args.max_acceleration
                      for item in vehicles.values()))
        return {
            "mission_id": self.mission_id,
            "passed": passed,
            "scenario_enu": self.scenario(),
            "pair_min_horizontal_m": pair_dists,
            "global_min_horizontal_m": min(all_horizontal) if all_horizontal else None,
            "global_min_3d_m": min(all_spatial) if all_spatial else None,
            "required_clearance_m": self.args.min_clearance,
            "vehicles": vehicles,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="执行运动；默认仅预检")
    parser.add_argument("--confirm-sitl", default="", help="执行时必须为 I_UNDERSTAND_SITL")
    parser.add_argument("--uav-a", default="UAV1")
    parser.add_argument("--index-a", type=int, default=1)
    parser.add_argument("--uav-b", default="UAV2")
    parser.add_argument("--index-b", type=int, default=2)
    parser.add_argument("--uav-c", default="UAV3")
    parser.add_argument("--index-c", type=int, default=3)
    parser.add_argument("--center-x", type=float, default=30.0)
    parser.add_argument("--center-y", type=float, default=15.0)
    parser.add_argument("--altitude", type=float, default=12.0)
    parser.add_argument("--arm", type=float, default=8.0)
    parser.add_argument("--min-clearance", type=float, default=1.0)
    parser.add_argument("--max-speed", type=float, default=1.5)
    parser.add_argument("--max-acceleration", type=float, default=6.3)
    parser.add_argument("--task-timeout", type=float, default=120.0)
    parser.add_argument("--preflight-timeout", type=float, default=15.0)
    parser.add_argument("--output", default=str(WS / ".tmp/trinode_15sitl_result.json"))
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main() -> int:
    args = parse_args()
    rospy.init_node("trinode_15sitl_runner", anonymous=False)
    runner = TriNodeRunner(args)
    try:
        runner.preflight()
        print(json.dumps({"preflight": "PASS", "scenario_enu": runner.scenario()},
                         ensure_ascii=False, indent=2), flush=True)
        if not args.execute:
            print("DRY-RUN：未发送运动命令。执行需增加 "
                  "--execute --confirm-sitl I_UNDERSTAND_SITL", flush=True)
            return 0
        if args.confirm_sitl != "I_UNDERSTAND_SITL":
            raise RuntimeError("拒绝执行：--confirm-sitl 必须为 I_UNDERSTAND_SITL")

        runner.stage()
        runner.start_intent_probes()
        results = runner.cross()
        metrics = runner.metrics(results, runner.neighbor_inputs_seen())
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n",
                          encoding="utf-8")
        print(json.dumps(metrics, ensure_ascii=False, indent=2), flush=True)
        print("result: %s (%s)" % ("PASS" if metrics["passed"] else "FAIL", output),
              flush=True)
        return 0 if metrics["passed"] else 2
    except Exception as exc:
        rospy.logerr("trinode 15-SITL runner failed: %s", exc)
        return 1
    finally:
        if args.execute:
            runner.hold_all("trinode SITL test finished")
        runner.stop_intent_probes()


if __name__ == "__main__":
    sys.exit(main())
