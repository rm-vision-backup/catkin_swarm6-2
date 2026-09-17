#!/usr/bin/env python3
"""Single-master reciprocal ego_planner_driver smoke test.

Two ego nodes (UAV1, UAV2) share one ROS master. Each node publishes its own
trajectory_intent and we wire each node's neighbor_intent to the other node's
trajectory_intent, mirroring how EgoSwarmDriver forwards bridge-received
neighbor intents into the C++ node.

Scenarios:
  A) cross: both UAVs fly in parallel at the same time (reciprocal mutual
     awareness, parallel routes keep the swarm cost from preventing arrival).
  B) stagger: UAV2 starts after UAV1 (neighbor intent stale/epoch handling).

Assertions:
  - each node receives the other's neighbor_intent (count > 0)
  - both nodes reach EXECUTING then COMPLETED (arrival is verifiable in the
    parallel scenario)
  - relative-axis conversion works: C++ replan receives msg.stamp and converts
    to local axis at replan time (verified indirectly by successful replan)
"""
from __future__ import annotations

import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Empty, String
from swarm_uav_interfaces.msg import UavTrajectoryIntent

TIMEOUT = 60.0
UAVS = ["UAV1", "UAV2"]
GOALS = {"UAV1": (5.0, 0.0, 15.0), "UAV2": (5.0, 5.0, 15.0)}  # parallel, 5m apart

state_events = {}
intent_counts = {}
latest_setpoint = {}
setpoint_counts = {}
lock = threading.Lock()

for uav in UAVS:
    state_events[uav] = []
    intent_counts[uav] = []
    latest_setpoint[uav] = [None]
    setpoint_counts[uav] = [0]


class IntentCounter(object):
    def __init__(self, uav):
        self.uav = uav

    def __call__(self, msg):
        with lock:
            intent_counts[self.uav].append(msg.uav_id)


def pose_loop(uav, pub):
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        with lock:
            sp = latest_setpoint[uav][0]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        if sp is not None:
            pose.pose.position.x = sp.position.x
            pose.pose.position.y = sp.position.y
            pose.pose.position.z = sp.position.z
            pose.pose.orientation.z = math.sin(sp.yaw / 2.0)
            pose.pose.orientation.w = math.cos(sp.yaw / 2.0)
        else:
            pose.pose.position.z = 15.0
            pose.pose.orientation.w = 1.0
        try:
            pub.publish(pose)
        except rospy.ROSException:
            return
        rate.sleep()


def wait_for(pred, what, timeout=TIMEOUT):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.05)
    print("FAIL: timeout waiting for %s" % what)
    return False


def main():
    rospy.init_node("ego_reciprocal_smoke", anonymous=True)

    pubs = {}
    for uav in UAVS:
        rospy.Subscriber(uav + "/exec_state", String,
                         lambda m, u=uav: state_events[u].append(m.data))
        rospy.Subscriber(uav + "/setpoint/ego", PositionTarget,
                         lambda m, u=uav: (setpoint_counts[u].__setitem__(0, setpoint_counts[u][0] + 1),
                                           latest_setpoint[u].__setitem__(0, m)))
        # neighbor_intent is the actual input to the C++ node's onNeighborIntent
        rospy.Subscriber(uav + "/neighbor_intent", UavTrajectoryIntent, IntentCounter(uav))
        pubs[uav] = rospy.Publisher(uav + "/local_pose", PoseStamped, queue_size=1)
        threading.Thread(target=pose_loop, args=(uav, pubs[uav]), daemon=True).start()

    npubs = {}
    for uav in UAVS:
        npubs[uav] = rospy.Publisher(uav + "/neighbor_intent", UavTrajectoryIntent, queue_size=10)
    for uav in UAVS:
        other = [x for x in UAVS if x != uav][0]
        rospy.Subscriber(other + "/trajectory_intent", UavTrajectoryIntent,
                         lambda m, u=uav: npubs[u].publish(m))

    # Pre-create goal/pose publishers and wait for ROS connections so the
    # first publish is not lost (a freshly created publisher has no TCP
    # connection yet and a non-latched publish on /goal would be dropped).
    goal_pubs = {}
    hold_pubs = {}
    for uav in UAVS:
        goal_pubs[uav] = rospy.Publisher(uav + "/goal", PointStamped, queue_size=1)
        hold_pubs[uav] = rospy.Publisher(uav + "/hold", Empty, queue_size=1)
    time.sleep(2.0)
    end = time.monotonic() + 10.0
    while time.monotonic() < end:
        if all(p.get_num_connections() > 0 for p in goal_pubs.values()):
            break
        time.sleep(0.05)
    else:
        print("FAIL: goal publishers never connected", flush=True)
        return 1

    def send_goal(uav):
        g = PointStamped()
        g.header.stamp = rospy.Time.now()
        x, y, z = GOALS[uav]
        g.point.x, g.point.y, g.point.z = x, y, z
        goal_pubs[uav].publish(g)
        print("goal sent to %s: (%s,%s,%s)" % (uav, x, y, z), flush=True)

    # Scenario A: parallel - both fly simultaneously
    print("=== Scenario A: parallel (simultaneous) ===")
    for uav in UAVS:
        send_goal(uav)

    ok_a = True
    for uav in UAVS:
        exec_ok = wait_for(lambda u=uav: "EXECUTING" in state_events[u], uav + " EXECUTING")
        done_ok = wait_for(lambda u=uav: "COMPLETED" in state_events[u], uav + " COMPLETED")
        print("%s states: %s" % (uav, state_events[uav]))
        ok_a = ok_a and exec_ok and done_ok

    for uav in UAVS:
        print("%s received neighbor intents from: %s (total=%d)"
              % (uav, sorted(set(intent_counts[uav])), len(intent_counts[uav])))
        ok_a = ok_a and len(intent_counts[uav]) > 0

    # HOLD both cleanly
    for uav in UAVS:
        hold_pubs[uav].publish(Empty())
    for uav in UAVS:
        held = wait_for(lambda u=uav: "HOLD" in state_events[u], uav + " HOLD")
        ok_a = ok_a and held
    print("states after HOLD: %s" % {u: state_events[u] for u in UAVS}, flush=True)

    # Scenario B: stagger - reset state, UAV2 starts 3s after UAV1
    print("=== Scenario B: stagger (UAV2 delayed) ===")
    for uav in UAVS:
        state_events[uav] = []
        latest_setpoint[uav][0] = None
    send_goal("UAV1")
    exec1 = wait_for(lambda: "EXECUTING" in state_events["UAV1"], "UAV1 EXECUTING (B)")
    time.sleep(3.0)
    send_goal("UAV2")
    exec2 = wait_for(lambda: "EXECUTING" in state_events["UAV2"], "UAV2 EXECUTING (B)")
    done1 = wait_for(lambda: "COMPLETED" in state_events["UAV1"], "UAV1 COMPLETED (B)")
    done2 = wait_for(lambda: "COMPLETED" in state_events["UAV2"], "UAV2 COMPLETED (B)")
    print("B UAV1 states: %s" % state_events["UAV1"])
    print("B UAV2 states: %s" % state_events["UAV2"])
    ok_b = exec1 and exec2 and done1 and done2

    ok = ok_a and ok_b
    print("RECIPROCAL SMOKE %s" % ("PASS" if ok else "FAIL"), flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())