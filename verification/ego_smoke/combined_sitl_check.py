#!/usr/bin/env python3
"""PX4 SITL/MAVROS + ego driver 组合验证 (safety-lease removed)."""
from __future__ import annotations

import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String
from swarm_uav_interfaces.msg import UavTaskState
from swarm_uav_interfaces.srv import (UavHold, UavHoldRequest, UavTask,
                                      UavTaskRequest)

# Ego node topics are prefix-free on this master (onboard premise).
NS = ""
# Executor business services keep the exec_target namespace (/UAV1/...).
EXEC_NS = "/UAV1"
MISSION = "sitl-ego-combined"


def main():
    rospy.init_node("ego_sitl_combined_check", anonymous=False)

    states = []
    setpoints = [0]
    latest_pose = [None]

    rospy.Subscriber(NS + "/exec_state", String, lambda m: states.append(m.data))
    rospy.Subscriber(NS + "/setpoint/ego", PositionTarget,
                     lambda m: (setpoints.__setitem__(0, setpoints[0] + 1),
                                latest_pose.__setitem__(0, m)))
    rospy.Subscriber(EXEC_NS + "/uav_task_state", UavTaskState, lambda m: None)

    pose_pub = rospy.Publisher(NS + "/local_pose", PoseStamped, queue_size=1)

    def pose_loop():
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            sp = latest_pose[0]
            if sp is not None:
                pose.pose.position.x = sp.position.x
                pose.pose.position.y = sp.position.y
                pose.pose.position.z = sp.position.z
                pose.pose.orientation.z = math.sin(sp.yaw / 2.0)
                pose.pose.orientation.w = math.cos(sp.yaw / 2.0)
            else:
                pose.pose.position.z = 15.0
                pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            rate.sleep()

    threading.Thread(target=pose_loop, daemon=True).start()
    time.sleep(1.0)

    rospy.wait_for_service(EXEC_NS + "/uav_task", timeout=10)
    task_srv = rospy.ServiceProxy(EXEC_NS + "/uav_task", UavTask)

    t = UavTaskRequest()
    t.protocol_version = "1.0"; t.mission_id = MISSION; t.group_id = "GroupA"
    t.command_id = "cmd-1"; t.uav_id = "A1"; t.exec_target = "UAV1"
    t.command = "MOVE_TO"; t.timeout_s = 30.0; t.leader_id = ""
    t.assignment.uav_id = "A1"; t.assignment.formation_follow = False
    t.assignment.target_id = ""
    t.assignment.target_pose.x = 5.0; t.assignment.target_pose.y = 0.0; t.assignment.target_pose.z = 15.0
    tr = task_srv(t)
    print("TASK MOVE_TO: accepted=%s status=%s error=%s" % (tr.accepted, tr.status, tr.error_code), flush=True)
    if not tr.accepted:
        return 1

    end = time.monotonic() + 20.0
    while time.monotonic() < end and "EXECUTING" not in states:
        time.sleep(0.05)
    print("ego states: %s" % states, flush=True)
    if "EXECUTING" not in states:
        print("FAIL: ego did not enter EXECUTING", flush=True)
        return 1
    time.sleep(3.0)
    print("setpoint count after EXECUTING: %d" % setpoints[0], flush=True)

    rospy.wait_for_service(EXEC_NS + "/uav_hold", timeout=10)
    hold_srv = rospy.ServiceProxy(EXEC_NS + "/uav_hold", UavHold)
    hr = UavHoldRequest()
    hr.protocol_version = "1.0"; hr.mission_id = MISSION; hr.command_id = "cmd-1"
    hr.uav_id = "A1"; hr.exec_target = "UAV1"; hr.reason = "combined check hold"
    hold_srv(hr)
    end = time.monotonic() + 10.0
    while time.monotonic() < end and "HOLD" not in states:
        time.sleep(0.05)

    print("COMBINED CHECK states=%s setpoints=%d" % (states, setpoints[0]), flush=True)
    ok = "EXECUTING" in states and "HOLD" in states and setpoints[0] > 0
    print("COMBINED %s" % ("PASS" if ok else "FAIL"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
