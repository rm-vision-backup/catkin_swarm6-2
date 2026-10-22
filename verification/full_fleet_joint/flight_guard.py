"""只读飞行观测；中止时使用现有逐机 UavHold，不改变机载安全阈值。

与六机版本的差异：参与机、订阅和逐阶段期望终点全部来自
generated/mission_spec.json，不再硬编码领机/±5m/备用机。
"""
from __future__ import annotations

import math
import threading
import time

# 与机载跟随环的领机 odom 超时对齐（ego_swarm._FOLLOW_POSE_TIMEOUT_S = 3.0）。
TARGET_POSE_MAX_AGE_S = 3.0


class RosFlightGuard:
    def __init__(self, package, spec):
        import rospy
        from geometry_msgs.msg import PointStamped
        from mavros_msgs.msg import State
        from std_msgs.msg import Bool, Float64
        from swarm_uav_interfaces.msg import UavTaskState

        rospy.init_node("full_fleet_joint_observer", anonymous=True, disable_signals=True)
        self.ros = rospy
        self.package = package
        self.spec = spec
        self.ids = list(spec["all_ids"])
        self.lock = threading.RLock()
        self.samples = {}
        self.task_failure = None
        self.subscribers = []
        for topic, kind in (
            ("/group_a/calibration_ready", Bool),
            ("/group_a/reference_heading", Float64),
        ):
            self.subscribers.append(
                rospy.Subscriber(topic, kind, self.receive, callback_args=topic, queue_size=1)
            )
        for uav in self.ids:
            number = int(uav[1:])
            for suffix, kind in (
                ("swarm_pose", PointStamped),
                ("mavros/state", State),
                ("uav_task_state", UavTaskState),
                ("gp_origin_confirmed", Bool),
            ):
                topic = "/UAV%d/%s" % (number, suffix)
                self.subscribers.append(
                    rospy.Subscriber(topic, kind, self.receive, callback_args=topic, queue_size=10)
                )

    def receive(self, message, topic):
        with self.lock:
            self.samples[topic] = (time.monotonic(), message)
            if topic.endswith("/uav_task_state"):
                if (
                    message.mission_id == self.package.context.mission_id
                    and message.status == "FAILED"
                ):
                    self.task_failure = "%s %s %s" % (
                        topic,
                        message.error_code,
                        message.message,
                    )

    def sample(self, topic, max_age=None):
        received, message = self.samples[topic]
        if max_age is not None and time.monotonic() - received > max_age:
            raise RuntimeError("消息过期 %s" % topic)
        return message

    def check(self):
        with self.lock:
            if self.ros.is_shutdown():
                raise RuntimeError("ROS 已关闭")
            if self.task_failure:
                raise RuntimeError(self.task_failure)
            for uav in self.ids:
                prefix = "/UAV%d/" % int(uav[1:])
                state = self.sample(prefix + "mavros/state")
                if not (state.connected and state.armed and state.mode == "OFFBOARD"):
                    raise RuntimeError("%s 未 connected/armed/OFFBOARD" % uav)

    def wait_ready(self):
        deadline = time.monotonic() + 10.0
        while True:
            try:
                if not self.sample("/group_a/calibration_ready").data:
                    raise RuntimeError("calibration_ready=False")
                heading = self.sample("/group_a/reference_heading").data
                if not math.isfinite(heading):
                    raise RuntimeError("参考航向无效")
                self.check()
                return
            except (KeyError, RuntimeError) as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError("观测未就绪：%s" % error)
                time.sleep(0.1)

    def check_pose(self, uav, pose):
        heading = self.sample("/group_a/reference_heading").data
        if not math.isfinite(heading):
            raise RuntimeError("参考航向无效")
        east = pose["x"] * math.sin(heading) + pose["y"] * math.cos(heading)
        north = -pose["x"] * math.cos(heading) + pose["y"] * math.sin(heading)
        point = self.sample(
            "/UAV%d/swarm_pose" % int(uav[1:]), TARGET_POSE_MAX_AGE_S
        ).point
        horizontal = math.hypot(point.x - east, point.y - north)
        vertical = abs(point.z - pose["z"])
        if horizontal > 2.0 or vertical > 1.0:
            raise RuntimeError(
                "%s 实测位置不符：水平误差%.2fm，高度误差%.2fm"
                % (uav, horizontal, vertical)
            )

    def check_expected(self, plan_id):
        expected = self.spec["expected_targets"][plan_id]
        with self.lock:
            for uav, pose in expected.items():
                self.check_pose(uav, pose)
            snapshot = {}
            for uav in expected:
                topic = "/UAV%d/swarm_pose" % int(uav[1:])
                received, msg = self.samples[topic]
                snapshot[uav] = {
                    "x": msg.point.x,
                    "y": msg.point.y,
                    "z": msg.point.z,
                    "receipt_age_s": time.monotonic() - received,
                }
            return snapshot

    def abort_all(self, mission_id, logger):
        from swarm_uav_interfaces.srv import UavHold, UavHoldRequest

        results = {}
        lock = threading.Lock()

        def hold_one(uav):
            target = "UAV%d" % int(uav[1:])
            try:
                service = "/%s/uav_hold" % target
                self.ros.wait_for_service(service, timeout=0.5)
                request = UavHoldRequest(
                    protocol_version="1.0",
                    mission_id=mission_id,
                    command_id="full-fleet-abort",
                    uav_id=target,
                    exec_target=target,
                    reason="RUNTIME_BRAKE_HOLD",
                )
                response = self.ros.ServiceProxy(service, UavHold)(request)
                value = {
                    "accepted": response.accepted,
                    "error_code": response.error_code,
                    "message": response.message,
                }
            except Exception as error:  # noqa: BLE001
                value = {"accepted": False, "message": str(error)}
            with lock:
                results[uav] = value

        for start in range(0, len(self.ids), 4):
            threads = [
                threading.Thread(target=hold_one, args=(uav,), daemon=True)
                for uav in self.ids[start:start + 4]
            ]
            for thread in threads:
                thread.start()
            deadline = time.monotonic() + 2.0
            for thread in threads:
                thread.join(max(0, deadline - time.monotonic()))
        with lock:
            snapshot = {
                uav: results.get(uav, {"accepted": False, "message": "service timeout"})
                for uav in self.ids
            }
        logger.write(
            {
                "event": "abort_hold_requests",
                "results": snapshot,
                "unconfirmed": [
                    uav for uav, result in snapshot.items() if not result["accepted"]
                ],
                "note": "accepted仅表示请求接收，飞手仍须确认实际悬停",
            }
        )
