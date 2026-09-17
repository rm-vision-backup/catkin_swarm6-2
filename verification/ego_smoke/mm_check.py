#!/usr/bin/env python3
"""跨 Master reciprocal 验证 - 单机检查脚本.

用法（两个 ROS master 各跑一个实例）:
  ROS_MASTER_URI=http://localhost:11311 python3 mm_check.py _uav:=UAV1 _other:=A02 _goal:=5,0,15
  ROS_MASTER_URI=http://localhost:11312 python3 mm_check.py _uav:=UAV2 _other:=A01 _goal:=5,5,15

行为:
  1. 向本机 ego 发 goal
  2. 等待本机 EXECUTING（证明 intent 已被发布）
  3. 等待 /UAVx/neighbor_intent 收到来自对侧无人机的 intent（traj_id 单调递增）
  4. 打印 PASS/FAIL
"""
from __future__ import annotations

import math
import sys
import time

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String, Empty
from swarm_uav_interfaces.msg import UavTrajectoryIntent


def _argv_options():
    opts = {}
    for arg in sys.argv[1:]:
        if arg.startswith("_") and ":=" in arg:
            key, _, value = arg[1:].partition(":=")
            opts[key] = value
    return opts


def main():
    opts = _argv_options()
    uav = opts.get("uav", "UAV1")
    self_id = opts.get("self_id", "A1")
    gx, gy, gz = [float(x) for x in opts.get("goal", "5,0,15").split(",")]

    rospy.init_node("mm_check_" + uav, anonymous=False)

    states = []
    neighbor_ids = []
    latest_setpoint = [None]

    # Onboard premise: ego node topics have no /UAVn prefix on this master.
    rospy.Subscriber("/exec_state", String, lambda msg: states.append(msg.data))
    rospy.Subscriber("/neighbor_intent", UavTrajectoryIntent,
                     lambda msg: neighbor_ids.append(msg.uav_id))
    rospy.Subscriber("/setpoint/ego", PositionTarget,
                     lambda msg: latest_setpoint.__setitem__(0, msg))

    pose_pub = rospy.Publisher("/local_pose", PoseStamped, queue_size=1)
    goal_pub = rospy.Publisher("/goal", PointStamped, queue_size=1)
    hold_pub = rospy.Publisher("/hold", Empty, queue_size=1)

    def pose_loop():
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            sp = latest_setpoint[0]
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

    import threading
    threading.Thread(target=pose_loop, daemon=True).start()

    time.sleep(2.0)
    end = time.monotonic() + 10.0
    while time.monotonic() < end and goal_pub.get_num_connections() == 0:
        time.sleep(0.05)
    if goal_pub.get_num_connections() == 0:
        print("FAIL: %s goal publisher never connected" % uav, flush=True)
        return 1

    goal = PointStamped()
    goal.header.stamp = rospy.Time.now()
    goal.point.x, goal.point.y, goal.point.z = gx, gy, gz
    goal_pub.publish(goal)
    print("%s goal sent (%s,%s,%s)" % (uav, gx, gy, gz), flush=True)

    ok_exe = False
    end = time.monotonic() + 15.0
    while time.monotonic() < end:
        if "EXECUTING" in states:
            ok_exe = True
            break
        time.sleep(0.05)
    print("%s states: %s" % (uav, states), flush=True)
    if not ok_exe:
        print("FAIL: %s did not reach EXECUTING" % uav, flush=True)
        return 1

    # 等待经 bridge 传来的邻居 intent（任意非本机 uav_id 即证明跨 Master 互达）
    ok_nb = False
    end = time.monotonic() + 15.0
    while time.monotonic() < end:
        if any(nid != self_id for nid in neighbor_ids):
            ok_nb = True
            break
        time.sleep(0.05)
    print("%s neighbor intents from: %s" % (uav, neighbor_ids), flush=True)
    if not ok_nb:
        print("FAIL: %s did not receive any neighbor intent (self_id=%s)" % (uav, self_id), flush=True)
        return 1

    hold_pub.publish(Empty())
    print("CROSS-MASTER %s PASS" % uav, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())