"""只读飞行观测；中止时使用现有逐机 UavHold，不改变机载安全阈值。"""
import math
import threading
import time

from verification.six_uav_joint.driver import IDS


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
                                 ("gp_origin_confirmed", Bool), ("uav_task_state", UavTaskState)):
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
        with self.lock:
            if self.ros.is_shutdown():
                raise RuntimeError("ROS 已关闭")
            if self.task_failure:
                raise RuntimeError(self.task_failure)
            if not self.sample("/group_a/calibration_ready").data:
                raise RuntimeError("calibration_ready=False")
            heading = self.sample("/group_a/reference_heading").data
            if not math.isfinite(heading):
                raise RuntimeError("参考航向无效")
            points = []
            for uav in IDS:
                prefix = "/UAV%d/" % int(uav[1:])
                state = self.sample(prefix + "mavros/state", 3.0)
                if not (state.connected and state.armed and state.mode == "OFFBOARD"):
                    raise RuntimeError("%s 未 connected/armed/OFFBOARD" % uav)
                if not self.sample(prefix + "gp_origin_confirmed").data:
                    raise RuntimeError("%s 原点未确认" % uav)
                point = self.sample(prefix + "swarm_pose", 1.0).point
                if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                    raise RuntimeError("%s 位姿无效" % uav)
                points.append((uav, point))
            for i, (name, point) in enumerate(points):
                for other, q in points[i + 1:]:
                    distance = math.sqrt((point.x-q.x)**2 + (point.y-q.y)**2 + (point.z-q.z)**2)
                    if distance < 1.0:
                        raise RuntimeError("%s/%s 三维间距 %.2fm < 1m" % (name, other, distance))

    def wait_ready(self):
        deadline = time.monotonic() + 10.0
        while True:
            try:
                self.check()
                return
            except (KeyError, RuntimeError) as error:
                if time.monotonic() >= deadline:
                    raise RuntimeError("观测未就绪：%s" % error)
                time.sleep(0.1)

    def check_pose(self, uav, pose, initial=False):
        heading = self.sample("/group_a/reference_heading").data
        # 与 tcp_to_ros.calibration.task_vector_to_enu 相同的右/前 -> ENU 变换。
        east = pose["x"] * math.sin(heading) + pose["y"] * math.cos(heading)
        north = -pose["x"] * math.cos(heading) + pose["y"] * math.sin(heading)
        point = self.sample("/UAV%d/swarm_pose" % int(uav[1:]), 1.0).point
        horizontal = math.hypot(point.x-east, point.y-north)
        vertical = abs(point.z - pose["z"])
        if horizontal > (2.0 if initial else 1.0) or vertical > 1.0:
            raise RuntimeError("%s 实测位置不符：水平误差%.2fm，高度误差%.2fm" % (uav, horizontal, vertical))

    def check_initial_positions(self):
        with self.lock:
            for uav, assignment in self.package.context.plans["prepare-all"].robot_assignments.items():
                pose = dict(assignment.payload["target_pose"], z=5.0)
                self.check_pose(uav, pose, initial=True)

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
