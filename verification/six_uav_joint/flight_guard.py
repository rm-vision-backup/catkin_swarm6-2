"""只读飞行观测；中止时使用现有逐机 UavHold，不改变机载安全阈值。"""
import math
import threading
import time

from verification.six_uav_joint.driver import IDS

# 阶段终点核对时本机 swarm_pose 的最大年龄。与机载跟随环的领机 odom 超时对齐
# （ego_swarm._FOLLOW_POSE_TIMEOUT_S = 3.0 s，2026-10-10 依实测 1.1–5.2 s 空档放宽）：
# 地面站不再用更严的 1 s 门槛，避免已知量级的链路空档在终点核对瞬间误触发中止。
TARGET_POSE_MAX_AGE_S = 3.0


class RosFlightGuard:
    def __init__(self, package):
        import rospy
        from geometry_msgs.msg import PointStamped
        from mavros_msgs.msg import State
        from std_msgs.msg import Bool, Float64
        from swarm_uav_interfaces.msg import UavTaskState

        rospy.init_node("six_uav_joint_observer", anonymous=True, disable_signals=True)
        self.ros = rospy
        self.package = package
        self.lock = threading.RLock()
        self.samples = {}
        self.task_failure = None
        self.subscribers = []
        for topic, kind in (("/group_a/calibration_ready", Bool),
                            ("/group_a/reference_heading", Float64)):
            self.subscribers.append(rospy.Subscriber(topic, kind, self.receive,
                                                    callback_args=topic, queue_size=1))
        for uav in IDS:
            number = int(uav[1:])
            for suffix, kind in (("swarm_pose", PointStamped), ("mavros/state", State),
                                 ("uav_task_state", UavTaskState)):
                topic = "/UAV%d/%s" % (number, suffix)
                self.subscribers.append(rospy.Subscriber(topic, kind, self.receive,
                                                        callback_args=topic, queue_size=10))

    def receive(self, message, topic):
        with self.lock:
            self.samples[topic] = (time.monotonic(), message)
            if topic.endswith("/uav_task_state"):
                if message.mission_id == self.package.context.mission_id and message.status == "FAILED":
                    self.task_failure = "%s %s %s" % (topic, message.error_code, message.message)

    def sample(self, topic, max_age=None):
        received, message = self.samples[topic]
        if max_age is not None and time.monotonic() - received > max_age:
            raise RuntimeError("消息过期 %s" % topic)
        return message

    def check(self):
        """运行期只核对状态内容与任务失败锁存。

        calibration_ready/参考航向不在每轮循环检查（与三机流程一致）：
        启动时由 wait_ready 一次性核对，之后 backend 的 require_calibration_ready
        会在每批下发前拒单。mavros/state 只看最新消息内容、不看年龄；
        链路丢失由 backend 的 UavLinkMonitor（10 s suspect / 15 s lost）和
        机载执行器负责。连续的 swarm_pose 新鲜度与机间距离同样由机载执行器负责
        （领机 odom 超时 3.0 s -> LEADER_LOST；三维中心距 <1 m -> HOLD），
        地面站只在阶段结束用 check_targets 核对实测终点。
        """
        with self.lock:
            if self.ros.is_shutdown():
                raise RuntimeError("ROS 已关闭")
            if self.task_failure:
                raise RuntimeError(self.task_failure)
            for uav in IDS:
                prefix = "/UAV%d/" % int(uav[1:])
                state = self.sample(prefix + "mavros/state")
                if not (state.connected and state.armed and state.mode == "OFFBOARD"):
                    raise RuntimeError("%s 未 connected/armed/OFFBOARD" % uav)

    def wait_ready(self):
        """启动前一次性门禁：标定 ready、参考航向有效，然后六机状态持续就绪。"""
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
        # 与 tcp_to_ros.calibration.task_vector_to_enu 相同的右/前 -> ENU 变换。
        east = pose["x"] * math.sin(heading) + pose["y"] * math.cos(heading)
        north = -pose["x"] * math.cos(heading) + pose["y"] * math.sin(heading)
        point = self.sample("/UAV%d/swarm_pose" % int(uav[1:]),
                            TARGET_POSE_MAX_AGE_S).point
        horizontal = math.hypot(point.x-east, point.y-north)
        vertical = abs(point.z - pose["z"])
        if horizontal > 2.0 or vertical > 1.0:
            raise RuntimeError("%s 实测位置不符：水平误差%.2fm，高度误差%.2fm" % (uav, horizontal, vertical))

    def check_targets(self, plan_id):
        with self.lock:
            plan = self.package.context.plans[plan_id]
            leader_payload = plan.robot_assignments.get("A07")
            leader_end = None
            if leader_payload and leader_payload.command_type == "FOLLOW_ROUTE":
                leader_end = leader_payload.payload["waypoints"][-1]
            for uav, assignment in plan.robot_assignments.items():
                payload = assignment.payload
                if "target_pose" in payload:
                    target = payload["target_pose"]
                elif payload.get("waypoints"):
                    target = payload["waypoints"][-1]
                elif payload.get("formation_follow") and leader_end:
                    target = dict(leader_end)
                    target["x"] += -5.0 if uav in {"A06", "A11"} else 5.0
                else:
                    continue  # HOVER 保持当前位置，不把占位 target_pose 当作目标。
                self.check_pose(uav, target)
            if plan_id == "recovery-a":
                # 幸存机保持原位，只有备用机收到MOVE_TO；确认完整恢复点阵再继续覆盖。
                leader = self.package.context.plans["coverage-segment-2"].robot_assignments["A07"].payload["waypoints"][0]
                for uav, dx in (("A07", 0), ("A08", 5), ("A11", -5)):
                    self.check_pose(uav, dict(leader, x=leader["x"]+dx))
            return {uav: {"x": msg.point.x, "y": msg.point.y, "z": msg.point.z,
                          "receipt_age_s": time.monotonic()-received}
                    for uav in IDS
                    for received, msg in [self.samples["/UAV%d/swarm_pose" % int(uav[1:])]]}

    def abort_all(self, mission_id, logger):
        from swarm_uav_interfaces.srv import UavHold, UavHoldRequest

        results = {}
        lock = threading.Lock()

        def hold_one(uav):
            target = "UAV%d" % int(uav[1:])
            try:
                service = "/%s/uav_hold" % target
                self.ros.wait_for_service(service, timeout=1.0)
                request = UavHoldRequest(protocol_version="1.0", mission_id=mission_id,
                                         command_id="six-joint-abort", uav_id=target,
                                         exec_target=target, reason="RUNTIME_BRAKE_HOLD")
                response = self.ros.ServiceProxy(service, UavHold)(request)
                value = {"accepted": response.accepted, "error_code": response.error_code,
                         "message": response.message}
            except Exception as error:
                value = {"accepted": False, "message": str(error)}
            with lock:
                results[uav] = value

        # 每批3机，避免把6个服务请求同时压入电台；daemon防止服务阻塞退出。
        for start in range(0, len(IDS), 3):
            threads = [threading.Thread(target=hold_one, args=(uav,), daemon=True)
                       for uav in IDS[start:start+3]]
            for thread in threads:
                thread.start()
            deadline = time.monotonic() + 4.0
            for thread in threads:
                thread.join(max(0, deadline-time.monotonic()))
        with lock:
            snapshot = {uav: results.get(uav, {"accepted": False, "message": "service timeout"})
                        for uav in IDS}
        logger.write({"event": "abort_hold_requests", "results": snapshot,
                      "unconfirmed": [uav for uav, result in snapshot.items() if not result["accepted"]],
                      "note": "accepted仅表示请求接收，飞手仍须确认实际悬停"})
