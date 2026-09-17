#!/usr/bin/env python3
"""Single-Master ego_planner_driver smoke test v3.

Subscribes to the setpoint stream and feeds it back as local_pose to emulate
an ideal position controller, so the traversal can be checked arrival and
COMPLETED. Verifies the full lifecycle:
  IDLE -> EXECUTING -> COMPLETED -> (auto IDLE after 2s) -> HOLD -> (auto IDLE)
and that the setpoint stream keeps flowing in every state (no OFFBOARD loss).
"""
from __future__ import annotations
import sys, threading, time
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Empty, String
from swarm_uav_interfaces.msg import UavTrajectoryIntent

NS = ""
TIMEOUT = 30.0
TERMINAL_TO_IDLE_S = 2.0   # 对应 ego_planner_driver.launch terminal_to_idle_s
state_events = []
setpoint_count = [0]
latest_setpoint = [None]
lock = threading.Lock()


def wait_for(pred, what, timeout=TIMEOUT):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    print("FAIL: timeout waiting for %s" % what)
    return False


def last_index_of(seq, value):
    for i in range(len(seq) - 1, -1, -1):
        if seq[i] == value:
            return i
    return -1


def pose_loop(pub):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        with lock:
            sp = latest_setpoint[0]
        if sp is not None:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose = sp.pose
            pub.publish(pose)
        else:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.z = 15.0
            pose.pose.orientation.w = 1.0
            pub.publish(pose)
        rate.sleep()


def main():
    rospy.init_node("ego_smoke3", anonymous=True)
    rospy.Subscriber(NS + "/exec_state", String, lambda m: state_events.append(m.data))
    rospy.Subscriber(NS + "/setpoint", PoseStamped,
                     lambda m: (setpoint_count.__setitem__(0, setpoint_count[0] + 1),
                                latest_setpoint.__setitem__(0, m)))
    hold_pub = rospy.Publisher(NS + "/hold", Empty, queue_size=1)
    goal_pub = rospy.Publisher(NS + "/goal", PointStamped, queue_size=1)
    pose_pub = rospy.Publisher(NS + "/local_pose", PoseStamped, queue_size=1)
    th = threading.Thread(target=pose_loop, args=(pose_pub,), daemon=True)
    th.start()
    time.sleep(1.5)

    goal = PointStamped()
    goal.header.stamp = rospy.Time.now()
    goal.point.x, goal.point.y, goal.point.z = 5.0, 0.0, 15.0
    goal_pub.publish(goal)
    print("goal sent (5,0,15)")

    executing = wait_for(lambda: "EXECUTING" in state_events, "EXECUTING")
    completed = wait_for(lambda: "COMPLETED" in state_events, "COMPLETED")
    print("states: %s" % state_events)
    print("setpoints: %d" % setpoint_count[0])

    # COMPLETED 冻结 2s 后自动回 IDLE（第一次回退）
    reverted1 = wait_for(
        lambda: "COMPLETED" in state_events and
                last_index_of(state_events, "IDLE") > state_events.index("COMPLETED"),
        "COMPLETED -> IDLE auto revert", timeout=TERMINAL_TO_IDLE_S + 5)
    print("states after COMPLETED revert: %s" % state_events)

    # 回 IDLE 后 setpoint 流仍在（IDLE 也持续发布当前位姿）
    count_at_idle = setpoint_count[0]
    time.sleep(1.0)
    idle_streaming = setpoint_count[0] > count_at_idle
    print("setpoints during IDLE: +%d" % (setpoint_count[0] - count_at_idle))

    hold_pub.publish(Empty())
    held = wait_for(lambda: "HOLD" in state_events, "HOLD state")
    print("states after HOLD: %s" % state_events)

    # HOLD 冻结 2s 后自动回 IDLE（第二次回退）
    reverted2 = wait_for(
        lambda: "HOLD" in state_events and
                last_index_of(state_events, "IDLE") > state_events.index("HOLD"),
        "HOLD -> IDLE auto revert", timeout=TERMINAL_TO_IDLE_S + 5)
    print("states after HOLD revert: %s" % state_events)

    count_at_hold_idle = setpoint_count[0]
    time.sleep(1.0)
    idle_streaming_after_hold = setpoint_count[0] > count_at_hold_idle
    print("setpoints after HOLD revert: +%d" % (setpoint_count[0] - count_at_hold_idle))

    ok = (executing and completed and held and reverted1 and reverted2 and
          setpoint_count[0] > 0 and idle_streaming and idle_streaming_after_hold)
    print("SMOKE %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
