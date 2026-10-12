"""ROS-only adapter: task batches, whole-session watchdog and verified HOLD."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Dict, Mapping

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool, Float64, String

from swarm_uav_interfaces.msg import Pose3DYaw, TaskAssignment, UavTaskState, UavTrajectoryIntent
from swarm_uav_interfaces.srv import UavHold, UavHoldRequest
from tcp_to_ros.identity import IdentityMap
from tcp_to_ros.msg import ExecuteGroupTaskAction, ExecuteGroupTaskGoal

from core import Scenario, components, distance, normalize_positions


class RosFleet:
    def __init__(self, scenario: Scenario, identity_path: Path, mission_id: str, recorder,
                 max_batch: int = 4):
        rospy.init_node("dag_recovery_experiment", disable_signals=True)
        self.scenario = scenario
        self.fault_ids = tuple(getattr(scenario, "fault_ids", (scenario.fault_id,)))
        self.mission_id = mission_id
        self.recorder = recorder
        # 单命令最大机数：A/B 任务默认 4；散布编队演示（排程已关）允许单命令全体同时下发。
        self._max_batch = int(max_batch)
        identities = IdentityMap.load(str(identity_path))
        self.targets = {}
        for uav in scenario.uav_ids:
            identity = identities.identity(uav)
            if not identity.enabled or not identity.exec_target:
                raise ValueError("disabled or unknown experiment identity: " + uav)
            self.targets[uav] = identity.exec_target
        if len(set(self.targets.values())) != len(self.targets):
            raise ValueError("duplicate execution target")
        self._lock = threading.RLock()
        self._poses = {}
        self._states = {}
        self._planner = {}
        self._planner_time = {}
        self._confirmed = {}
        self._ready = False
        self._heading = None
        self._watching = False
        self._fault_removed = False
        self._fault_xy = None
        self._failure = None
        self._last_min_distance = float("inf")
        self._hold_started = False
        self._hold_thread = None
        self._halt = threading.Event()
        self._samples = []
        self._last_sample_stamps = None
        self._last_unique_time = None
        self._goal_index = 0
        self._subscriptions = []
        self._client = actionlib.SimpleActionClient("/group_a/execute_task", ExecuteGroupTaskAction)
        self._subscriptions.append(rospy.Subscriber("/group_a/reference_heading", Float64,
                                  lambda msg: self._store_heading(msg.data), queue_size=1))
        self._subscriptions.append(rospy.Subscriber("/group_a/calibration_ready", Bool,
                                  lambda msg: self._store_ready(msg.data), queue_size=1))
        for uav, target in self.targets.items():
            for suffix, message_type, callback, size in (
                ("mavros/local_position/pose", PoseStamped, self._store_pose, 10),
                ("mavros/state", State, self._store_state, 10),
                ("exec_state", String, self._store_planner, 1),
                ("gp_origin_confirmed", Bool, self._store_confirmed, 1),
                ("uav_task_state", UavTaskState, self._store_task, 10),
                ("trajectory_intent", UavTrajectoryIntent, self._store_intent, 10),
            ):
                self._subscriptions.append(rospy.Subscriber(
                    "/%s/%s" % (target, suffix), message_type, callback,
                    callback_args=uav, queue_size=size))
        self._watcher = threading.Thread(target=self._watchdog, daemon=True)
        self._watcher.start()

    def _store_heading(self, value):
        with self._lock:
            if self._watching and self._heading is not None and abs(self._heading - float(value)) > 1e-4:
                self._abort("reference heading changed during mission")
            self._heading = float(value)

    def _store_ready(self, value):
        with self._lock:
            self._ready = bool(value)

    def _store_pose(self, msg, uav):
        point = msg.pose.position
        with self._lock:
            self._poses[uav] = ((point.x, point.y, point.z), time.monotonic(), msg.header.stamp)

    def _store_state(self, msg, uav):
        with self._lock:
            self._states[uav] = (bool(msg.connected), bool(msg.armed),
                                 str(msg.mode), time.monotonic())

    def _store_planner(self, msg, uav):
        with self._lock:
            self._planner[uav] = str(msg.data)
            self._planner_time[uav] = time.monotonic()

    def _store_confirmed(self, msg, uav):
        with self._lock:
            self._confirmed[uav] = bool(msg.data)

    def _store_task(self, msg, uav):
        if not self._halt.is_set() and msg.mission_id == self.mission_id:
            self.recorder.event("task_state", uav_id=uav, command_id=msg.command_id,
                                status=msg.status, stage=msg.detail_stage,
                                error_code=msg.error_code)

    def _store_intent(self, msg, uav):
        if self._halt.is_set():
            return
        if msg.uav_id != self.targets[uav] or msg.exec_target != self.targets[uav]:
            if self._watching:
                self._abort("bridge trajectory intent identity mismatch: " + uav)
            return
        points = [[pose.position.x, pose.position.y, pose.position.z]
                  for pose in msg.sampled_traj]
        if not all(math.isfinite(value) for point in points for value in point):
            if self._watching:
                self._abort("non-finite planned trajectory: " + uav)
            return
        self.recorder.event(
            "trajectory_intent", uav_id=uav, phase=int(msg.phase), traj_id=msg.traj_id,
            frame_id=msg.frame_id, source_stamp=msg.stamp,
            receive_age_s=rospy.Time.now().to_sec() - msg.stamp,
            sample_t=list(msg.t),
            sampled_enu=points)

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            if not self._ready or self._heading is None or not math.isfinite(self._heading):
                raise RuntimeError("calibration/heading not ready")
            poses = {}
            links = {}
            pose_stamps = {}
            problems = {}
            for uav in self.scenario.uav_ids:
                item = self._poses.get(uav)
                state = self._states.get(uav)
                source_age = ((rospy.Time.now() - item[2]).to_sec()
                              if item is not None else float("inf"))
                reasons = []
                if item is None:
                    reasons.append("pose_missing")
                else:
                    if now - item[1] > 0.6:
                        reasons.append("pose_rx_age=%.2f" % (now - item[1]))
                    if item[2].to_sec() <= 0.0:
                        reasons.append("pose_stamp_zero")
                    if not math.isfinite(source_age) or not -0.5 <= source_age <= 0.6:
                        reasons.append("pose_src_age=%.2f" % source_age)
                if state is None:
                    reasons.append("state_missing")
                else:
                    if now - state[3] > 2.0:
                        reasons.append("state_age=%.2f" % (now - state[3]))
                    if not state[0]:
                        reasons.append("not_connected")
                    if not state[1]:
                        reasons.append("not_armed")
                    if state[2] != "OFFBOARD":
                        reasons.append("mode=%s" % state[2])
                # gp_origin_confirmed / exec_state 在源头是"变了才发 + latched"，
                # 桥转发端不保留 latch（bridge_node.py 的非 latched Publisher），
                # 晚于标定/起飞接管（SITL 与实飞工具均为此时序）必然收不到首帧。
                # 两者不再作激活硬门禁：原点由 validate_fixture/ensure_initial 按
                # 实测队形把关，planner 由任务期状态、连通性检查与 HOLD 回执把关；
                # 是否见过这两个信号在下文 links 里留痕（wait_ready 会告警一次）。
                if reasons:
                    problems[uav] = reasons
                    continue
                poses[uav] = item[0]
                pose_stamps[uav] = item[2].to_sec()
                links[uav] = {"pose_age_s": now - item[1], "source_pose_age_s": source_age,
                              "state_age_s": now - state[3],
                              "connected": state[0], "armed": state[1], "mode": state[2],
                              "origin_confirmed_seen": bool(self._confirmed.get(uav)),
                              "planner_state_seen": uav in self._planner}
            if problems:
                detail = "; ".join("%s:%s" % (uav, ",".join(values))
                                   for uav, values in list(problems.items())[:5])
                raise RuntimeError("fleet not ready (%d/%d UAVs): %s"
                                   % (len(problems), len(self.scenario.uav_ids), detail))
            heading = self._heading
        positions = normalize_positions(poses, heading)
        if not all(all(math.isfinite(value) for value in point)
                   for point in positions.values()):
            raise RuntimeError("non-finite flight position")
        return {"poses_enu": poses, "poses_task": positions, "links": links,
                "pose_stamps_ros": pose_stamps,
                "heading_rad": heading, "monotonic_time": now}

    def wait_ready(self, timeout_s: float = 40.0) -> dict:
        deadline = time.monotonic() + timeout_s
        last_error = "not initialized"
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                snapshot = self.snapshot()
                if not self._client.wait_for_server(rospy.Duration(0.1)):
                    raise RuntimeError("ExecuteGroupTask backend not available")
                self._note_late_signals(snapshot)
                return snapshot
            except RuntimeError as exc:
                last_error = str(exc)
                time.sleep(0.2)
        raise RuntimeError("fleet readiness timed out: " + last_error)

    def _note_late_signals(self, snapshot: dict) -> None:
        """晚接管取证：gp_origin_confirmed / exec_state 不随桥补发，缺它们不阻塞
        激活，但必须在日志与 links 记录里可见（原点由实测队形校验、planner 由
        任务期状态与 HOLD 回执把关）。"""
        links = snapshot["links"]
        missing_origin = [uav for uav in self.scenario.uav_ids
                          if not links[uav]["origin_confirmed_seen"]]
        missing_planner = [uav for uav in self.scenario.uav_ids
                           if not links[uav]["planner_state_seen"]]
        if missing_origin or missing_planner:
            rospy.logwarn("late subscriber: gp_origin_confirmed unseen for %s; "
                          "exec_state unseen for %s; origin/planner covered by "
                          "measured-layout and mission checks instead",
                          missing_origin or "-", missing_planner or "-")

    def activate(self):
        self.snapshot()
        self._watching = True

    def remove_fault(self):
        current = self.snapshot()["poses_task"]
        self._fault_xy = {uav: current[uav][:2] for uav in self.fault_ids}
        self._fault_removed = True
        self.fault_time = time.monotonic()
        self.recorder.event("fault_removed", fault_ids=list(self.fault_ids))

    def _verify_snapshot(self, snapshot: dict) -> tuple[float, list[list[str]]]:
        poses = snapshot["poses_task"]
        for uav, pose in poses.items():
            if not (5.0 - self.scenario.altitude_tolerance
                    <= pose[2] <= self.scenario.normal_height + self.scenario.altitude_tolerance):
                raise RuntimeError("aircraft departed approved altitude envelope: " + uav)
            if (self._fault_removed and uav not in self.fault_ids
                    and abs(pose[2] - self.scenario.normal_height) > self.scenario.altitude_tolerance):
                raise RuntimeError("survivor departed 15 m recovery layer: " + uav)
        if self._fault_removed:
            for uav in self.fault_ids:
                fault_pose = poses[uav]
                if (distance(fault_pose[:2], self._fault_xy[uav]) > self.scenario.layout_tolerance
                        or abs(fault_pose[2] - self.scenario.fault_height) > self.scenario.altitude_tolerance):
                    raise RuntimeError("isolated fault aircraft drifted from its controlled layer: " + uav)
        with self._lock:
            statuses = dict(self._planner)
        for uav, status in statuses.items():
            if status in ("EGO_PLAN_FAILED", "EGO_EXEC_TIMEOUT", "BRAKE_HOLD"):
                raise RuntimeError("unexpected safety/failed planner state: %s %s" % (uav, status))
        for point in poses.values():
            if not self.scenario.within_bounds(point[:2]):
                raise RuntimeError("UAV outside approved experiment bounds")
        min_distance = float("inf")
        for index, first in enumerate(self.scenario.uav_ids):
            for second in self.scenario.uav_ids[index + 1:]:
                pos_a, pos_b = poses[first], poses[second]
                separation = math.hypot(distance(pos_a[:2], pos_b[:2]), pos_a[2] - pos_b[2])
                min_distance = min(min_distance, separation)
        if min_distance < self.scenario.min_separation:
            raise RuntimeError("whole-session 3D separation breached")
        healthy = {uav: poses[uav][:2] for uav in self.scenario.uav_ids
                   if not self._fault_removed or uav not in self.fault_ids}
        return min_distance, components(healthy, self.scenario.radius)

    def _watchdog(self):
        while not self._halt.wait(0.2) and not rospy.is_shutdown():
            if not self._watching:
                continue
            try:
                snapshot = self.snapshot()
                minimum, clusters = self._verify_snapshot(snapshot)
                self._last_min_distance = min(self._last_min_distance, minimum)
                stamps = snapshot["pose_stamps_ros"]
                unique = (self._last_sample_stamps is None or
                          all(stamps[uav] > self._last_sample_stamps[uav]
                              for uav in self.scenario.uav_ids))
                if unique:
                    self._last_sample_stamps = stamps
                    self._last_unique_time = snapshot["monotonic_time"]
                    self._samples.append((self._last_unique_time, len(clusters)))
                if snapshot["monotonic_time"] - self._last_unique_time > 0.4:
                    raise RuntimeError("full-swarm measured graph dropped below required 5 Hz")
                self.recorder.event("telemetry", poses_task=snapshot["poses_task"],
                                    poses_enu=snapshot["poses_enu"], links=snapshot["links"],
                                    pose_stamps_ros=snapshot["pose_stamps_ros"],
                                    new_positions=unique,
                                    heading_rad=snapshot["heading_rad"],
                                    components=clusters, min_separation_m=minimum,
                                    fault_removed=self._fault_removed)
            except Exception as exc:
                self._abort(str(exc))

    def _abort(self, reason: str):
        with self._lock:
            if self._failure is not None or self._halt.is_set():
                return
            self._failure = reason
        self.recorder.event("safety_abort", reason=reason)
        self._hold_thread = threading.Thread(target=self.hold_all, daemon=True)
        self._hold_thread.start()

    def assert_healthy(self):
        with self._lock:
            failure = self._failure
        if failure:
            raise RuntimeError("fleet watchdog aborted: " + failure)
        if rospy.is_shutdown():
            raise RuntimeError("ROS shutdown during experiment")
        snapshot = self.snapshot()
        self._verify_snapshot(snapshot)

    def send_goal(self, command: str, targets: Mapping[str, tuple[float, float, float]],
                  timeout_s: float = 180.0):
        if not 1 <= len(targets) <= self._max_batch or command not in ("MOVE_TO", "FAULT_EXIT"):
            raise ValueError("batch size exceeds %d for MOVE_TO/FAULT_EXIT" % self._max_batch)
        self.assert_healthy()
        self._goal_index += 1
        goal = ExecuteGroupTaskGoal()
        goal.mission_id = self.mission_id
        goal.group_id = "GroupA"
        goal.command_id = "%s-%03d" % (self.mission_id, self._goal_index)
        goal.command = command
        goal.timeout_s = timeout_s
        goal.leader_id = ""
        goal.assignments = []
        for uav in self.scenario.uav_ids:
            if uav not in targets:
                continue
            x, y, z = targets[uav]
            assignment = TaskAssignment()
            assignment.uav_id = uav
            assignment.formation_follow = False
            assignment.target_pose = Pose3DYaw(x=x, y=y, z=z, yaw=0.0)
            goal.assignments.append(assignment)
        self.recorder.event("command_sent", command=command, command_id=goal.command_id,
                            targets=targets)
        self._client.send_goal(goal)
        deadline = time.monotonic() + timeout_s + 15.0
        while not rospy.is_shutdown():
            self.assert_healthy()
            if self._client.wait_for_result(rospy.Duration(0.1)):
                result = self._client.get_result()
                completed = set(result.completed_uavs) if result else set()
                if (self._client.get_state() != GoalStatus.SUCCEEDED or result is None
                        or result.final_status != "COMPLETED" or completed != set(targets)):
                    raise RuntimeError("group task did not finish reliably: " + goal.command_id)
                self.recorder.event("command_completed", command_id=goal.command_id,
                                    completed_uavs=sorted(completed))
                self.wait_positions(targets)
                return
            if time.monotonic() > deadline:
                raise TimeoutError("group action exceeded deadline: " + goal.command_id)
        raise RuntimeError("ROS shutdown during group task")

    def wait_positions(self, targets: Mapping[str, tuple[float, float, float]],
                       timeout_s: float = 12.0):
        deadline = time.monotonic() + timeout_s
        stable_since = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.assert_healthy()
            poses = self.snapshot()["poses_task"]
            matched = all(distance(poses[uav][:2], target[:2]) <= 1.0
                          and abs(poses[uav][2] - target[2]) <= self.scenario.altitude_tolerance
                          for uav, target in targets.items())
            if matched:
                stable_since = stable_since or time.monotonic()
            else:
                stable_since = None
            if stable_since is not None and time.monotonic() - stable_since >= 2.0:
                return
            time.sleep(0.1)
        raise RuntimeError("action success not corroborated by stable physical positions")

    def wait_connected(self, stable_s: float = 10.0, timeout_s: float = 60.0):
        deadline = time.monotonic() + timeout_s
        stable_since = None
        previous_time = None
        next_sample = len(self._samples)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.assert_healthy()
            samples = self._samples[next_sample:]
            next_sample += len(samples)
            for timestamp, count in samples:
                if count != 1 or previous_time is not None and timestamp - previous_time > 0.4:
                    stable_since = None
                previous_time = timestamp
                if count == 1:
                    stable_since = stable_since or timestamp
                    if timestamp - stable_since >= stable_s:
                        self.recorder.event("connectivity_recovered", stable_s=stable_s,
                                            t0=getattr(self, "fault_time", None), end=timestamp,
                                            min_separation_m=self._last_min_distance)
                        return
            time.sleep(0.1)
        raise RuntimeError("recovery graph did not stay connected for required duration")

    def hold_all(self):
        with self._lock:
            if self._hold_started:
                return
            self._hold_started = True
        ids = list(self.scenario.uav_ids)
        request_time = time.monotonic()
        failures = []
        for start in range(0, len(ids), 4):
            batch = ids[start:start + 4]
            replies = {}
            threads = []

            def invoke(uav):
                target = self.targets[uav]
                request = UavHoldRequest()
                request.protocol_version = "1.0"
                request.mission_id = self.mission_id
                request.command_id = self.mission_id + "-safety-hold"
                request.uav_id = target
                request.exec_target = target
                request.reason = "GROUP_BRAKE_HOLD:dag_recovery"
                try:
                    service = rospy.ServiceProxy("/%s/uav_hold" % target, UavHold)
                    replies[uav] = bool(service(request).accepted)
                except Exception:
                    replies[uav] = False

            for uav in batch:
                thread = threading.Thread(target=invoke, args=(uav,), daemon=True)
                thread.start()
                threads.append(thread)
            batch_deadline = time.monotonic() + 1.5
            for thread in threads:
                thread.join(timeout=max(0.0, batch_deadline - time.monotonic()))
            for uav in batch:
                if replies.get(uav) is not True:
                    failures.append(uav)
        deadline = time.monotonic() + 3.0
        statuses = {}
        recent = {}
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._lock:
                statuses = dict(self._planner)
                planner_times = dict(self._planner_time)
                recent = {uav: time.monotonic() - self._poses[uav][1] < 1.0
                          for uav in self._poses}
            if all(statuses.get(uav) == "BRAKE_HOLD" and recent.get(uav)
                   and planner_times.get(uav, 0) >= request_time
                   for uav in ids if uav not in failures):
                break
            time.sleep(0.1)
        status_missing = sorted(uav for uav in ids
                                if statuses.get(uav) != "BRAKE_HOLD"
                                or planner_times.get(uav, 0) < request_time
                                or not recent.get(uav))
        unconfirmed = sorted(set(failures) | set(status_missing))
        self.recorder.event("hold_result", unconfirmed=unconfirmed,
                            rejected=sorted(failures), status_missing=status_missing)
        if unconfirmed:
            rospy.logerr("DAG experiment HOLD unconfirmed: %s (service_rejected=%s "
                         "status_missing=%s); RC intervention required",
                         unconfirmed, sorted(failures), status_missing)

    def close(self):
        self._halt.set()
        self._watching = False
        self._watcher.join(timeout=1.0)
        if self._hold_thread is not None and self._hold_thread.is_alive():
            self._hold_thread.join(timeout=12.0)
        for subscriber in self._subscriptions:
            subscriber.unregister()
