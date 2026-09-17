#!/usr/bin/env python3
"""在现有 15 机 PX4 SITL 环境中编排双机同高度正交互惠让路测试。

本脚本运行在 GCS_A ROS Master（默认 11310），直接调用两架测试机的
UavTask PREPARE/UavTaskControl START 服务，绕过 TCP/backend 批次聚合。因此脚本参数
使用已经统一后的本地 ENU 坐标，不再经过 task-frame right/forward -> ENU 转换。

测试步骤：
  1. 检查两机 MAVROS、odometry、任务服务与跨 Master trajectory_intent；
  2. 检查正式启动链已将邻机 intent 转发到本机 C++ /neighbor_intent；
  3. 串行 FOLLOW_ROUTE 到正交航线起点，避免 staging 本身形成并发冲突；
  4. 双机 FOLLOW_ROUTE 先 PREPARE，再用线程屏障近同时 START；
  5. 采集实际 odometry、任务状态与 intent，判定到达、最小间距和横向振荡；
  6. 无论结果如何，测试结束均向两架测试机发送 HOLD。

默认只做预检并打印场景。实际运动必须显式传入：
  --execute --confirm-sitl I_UNDERSTAND_SITL

示例：
  ROS_MASTER_URI=http://localhost:11310 python3 \
    verification/ego_smoke/reciprocal_15sitl.py
  ROS_MASTER_URI=http://localhost:11310 python3 \
    verification/ego_smoke/reciprocal_15sitl.py \
    --execute --confirm-sitl I_UNDERSTAND_SITL
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


class ReciprocalRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.vehicles = [Vehicle(args.uav_a, 11310 + args.index_a),
                         Vehicle(args.uav_b, 11310 + args.index_b)]
        self.data = {v.name: VehicleData() for v in self.vehicles}
        self.lock = threading.RLock()
        self.intent_probes: Dict[str, subprocess.Popen] = {}
        self.mission_id = "reciprocal-15sitl-%s" % uuid.uuid4().hex[:10]
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
        if self.args.swap:
            # 双机交换（同线对向）：A 沿 x+、B 沿 x-，中点 (cx,cy) 相遇，必须一方让路。
            return {
                "%s_stage" % self.vehicles[0].name: (cx - arm, cy, z),
                "%s_goal" % self.vehicles[0].name: (cx + arm, cy, z),
                "%s_stage" % self.vehicles[1].name: (cx + arm, cy, z),
                "%s_goal" % self.vehicles[1].name: (cx - arm, cy, z),
            }
        return {
            "%s_stage" % self.vehicles[0].name: (cx - arm, cy, z),
            "%s_goal" % self.vehicles[0].name: (cx + arm, cy, z),
            "%s_stage" % self.vehicles[1].name: (cx, cy - arm, z),
            "%s_goal" % self.vehicles[1].name: (cx, cy + arm, z),
        }

    def preflight(self) -> None:
        if os.environ.get("ROS_MASTER_URI", GCS_MASTER) != GCS_MASTER:
            raise RuntimeError("请在 GCS_A Master 运行：ROS_MASTER_URI=%s" % GCS_MASTER)
        if self.vehicles[0].name == self.vehicles[1].name:
            raise RuntimeError("两架测试机必须不同")
        for vehicle, index in zip(self.vehicles, (self.args.index_a, self.args.index_b)):
            if vehicle.name != "UAV%d" % index:
                raise RuntimeError("机名与索引不匹配：%s / %d" % (vehicle.name, index))
        if self.args.arm <= 0.0 or self.args.min_clearance <= 0.0:
            raise RuntimeError("--arm 与 --min-clearance 必须为正数")

        deadline = time.monotonic() + self.args.preflight_timeout
        service_names = []
        for vehicle in self.vehicles:
            service_names.extend(["/%s/uav_task" % vehicle.name,
                                  "/%s/uav_task_control" % vehicle.name,
                                  "/%s/uav_hold" % vehicle.name])
        for service in service_names:
            remaining = max(0.1, deadline - time.monotonic())
            rospy.wait_for_service(service, timeout=remaining)

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
            raise RuntimeError("两机 MAVROS state/odometry 未在预检窗口内同时就绪")

        failures = []
        with self.lock:
            for vehicle in self.vehicles:
                state = self.data[vehicle.name].state
                if state is None or not state.connected:
                    failures.append("%s MAVROS disconnected" % vehicle.name)
                if state is None or not state.armed:
                    failures.append("%s not armed" % vehicle.name)
                if state is None or state.mode != "OFFBOARD":
                    failures.append("%s mode=%s" %
                                    (vehicle.name, state.mode if state else "unknown"))
        if failures:
            raise RuntimeError("预检失败：" + "; ".join(failures))
        if self.args.require_rebound:
            disabled = []
            for vehicle in self.vehicles:
                try:
                    value = subprocess.check_output(
                        ["rosparam", "get", "/ego_planner_driver/enable_rebound"],
                        env=self._onboard_env(vehicle), text=True, timeout=3.0).strip().lower()
                except (OSError, subprocess.SubprocessError) as exc:
                    raise RuntimeError("无法读取 %s enable_rebound: %s" %
                                       (vehicle.name, exc)) from exc
                if value != "true":
                    disabled.append(vehicle.name)
            if disabled:
                raise RuntimeError("主动互惠测试要求测试机 enable_rebound=true：%s" %
                                   ",".join(disabled))

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
        """在两台机载 Master 各等待一条真正送入 C++ 的 /neighbor_intent。"""
        for vehicle in self.vehicles:
            process = subprocess.Popen(
                ["rostopic", "echo", "-n", "1", "/neighbor_intent"],
                env=self._onboard_env(vehicle), stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, start_new_session=True)
            self.intent_probes[vehicle.name] = process

    def neighbor_inputs_seen(self) -> Dict[str, bool]:
        deadline = time.monotonic() + 2.0
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
            reason="15-SITL reciprocal crossing test")
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
            except Exception as exc:  # propagated after both start threads join
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
    def _sign_changes(values: List[float], deadband: float = 0.20) -> int:
        signs = [1 if value > deadband else -1 if value < -deadband else 0 for value in values]
        filtered = [value for value in signs if value]
        return sum(a != b for a, b in zip(filtered, filtered[1:]))

    def metrics(self, results: Dict[str, UavTaskState],
                neighbor_inputs: Dict[str, bool]) -> Dict[str, object]:
        first, second = self.vehicles
        with self.lock:
            a_samples = list(self.data[first.name].samples)
            b_samples = list(self.data[second.name].samples)
            snapshot = {name: self.data[name] for name in self.data}

        # Nearest timestamp pairing prevents callback scheduling skew from becoming distance error.
        pairs = []
        j = 0
        for a in a_samples:
            while j + 1 < len(b_samples) and abs(b_samples[j + 1].mono_s - a.mono_s) < \
                    abs(b_samples[j].mono_s - a.mono_s):
                j += 1
            if b_samples and abs(b_samples[j].mono_s - a.mono_s) <= 0.15:
                pairs.append((a, b_samples[j]))
        horizontal = [math.hypot(a.x - b.x, a.y - b.y) for a, b in pairs]
        spatial = [math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
                   for a, b in pairs]
        a_lateral = [sample.y - self.args.center_y for sample in a_samples]
        b_lateral = [sample.x - self.args.center_x for sample in b_samples]

        vehicles = {}
        for vehicle, lateral in ((first, a_lateral), (second, b_lateral)):
            item = snapshot[vehicle.name]
            result = results.get(vehicle.name)
            samples = list(item.samples)
            speeds = [math.sqrt(sample.vx ** 2 + sample.vy ** 2 + sample.vz ** 2)
                      for sample in samples]
            accelerations = []
            for previous, current in zip(samples, samples[1:]):
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
                "max_abs_lateral_deviation_m": max(map(abs, lateral)) if lateral else None,
                "lateral_sign_changes": self._sign_changes(lateral),
                "max_speed_mps": max(speeds) if speeds else None,
                "max_acceleration_mps2": max(accelerations) if accelerations else None,
            }
        passed = (len(pairs) > 0 and min(horizontal) >= self.args.min_clearance and
                  all(result.status == "COMPLETED" for result in results.values()) and
                  all(item["intent_messages"] > 0 for item in vehicles.values()) and
                  all(item["neighbor_intent_input_seen"] for item in vehicles.values()) and
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
            "paired_samples": len(pairs),
            "min_horizontal_distance_m": min(horizontal) if horizontal else None,
            "min_3d_distance_m": min(spatial) if spatial else None,
            "required_clearance_m": self.args.min_clearance,
            "max_allowed_speed_mps": self.args.max_speed,
            "max_allowed_acceleration_mps2": self.args.max_acceleration,
            "max_lateral_sign_changes": self.args.max_sign_changes,
            "vehicles": vehicles,
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="执行运动；默认仅预检")
    parser.add_argument("--confirm-sitl", default="", help="执行时必须为 I_UNDERSTAND_SITL")
    parser.add_argument("--uav-a", default="UAV1")
    parser.add_argument("--index-a", type=int, default=1)
    parser.add_argument("--uav-b", default="UAV15")
    parser.add_argument("--index-b", type=int, default=15)
    parser.add_argument("--center-x", type=float, default=10.0)
    parser.add_argument("--center-y", type=float, default=5.0)
    parser.add_argument("--altitude", type=float, default=12.0)
    parser.add_argument("--arm", type=float, default=8.0, help="交叉中心到起终点距离")
    parser.add_argument("--swap", action="store_true",
                        help="双机交换模式：A/B 同线对向（中点相遇，必须一方让路）")
    parser.add_argument("--min-clearance", type=float, default=1.0)
    parser.add_argument("--max-sign-changes", type=int, default=2)
    parser.add_argument("--max-speed", type=float, default=2.5,
                        help="实测速度上限（26082621 迭代由 1.5 放宽：SLOW 1.2 加速档 + PX4 执行超调可达 1.5-1.55）")
    parser.add_argument("--max-acceleration", type=float, default=6.3)
    parser.add_argument("--no-require-rebound", dest="require_rebound", action="store_false",
                        help="仅诊断距离门禁时允许测试机 rebound 关闭")
    parser.set_defaults(require_rebound=True)
    parser.add_argument("--task-timeout", type=float, default=90.0)
    parser.add_argument("--preflight-timeout", type=float, default=15.0)
    parser.add_argument("--output", default=str(WS / ".tmp/reciprocal_15sitl_result.json"))
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main() -> int:
    args = parse_args()
    rospy.init_node("reciprocal_15sitl_runner", anonymous=False)
    runner = ReciprocalRunner(args)
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
        rospy.logerr("reciprocal 15-SITL runner failed: %s", exc)
        return 1
    finally:
        if args.execute:
            runner.hold_all("reciprocal SITL test finished")
        runner.stop_intent_probes()


if __name__ == "__main__":
    sys.exit(main())