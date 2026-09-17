#!/usr/bin/env python3
"""Single-master, no-MAVROS smoke test for the four full action commands.

Covers the new executor/driver semantics implemented from
implementation_plan_26081023.md:
- MOVE_TO/FAULT_EXIT: vertical-first transition, then horizontal goal.
- HOVER: freeze the current pose and republish it (no ego planning).
- FOLLOW_ROUTE follower: leader odom + formation offset PI tracking.

This script drives EgoSwarmDriver in isolation (only roscore, no live
ego_planner_driver node so the real /exec_state stream cannot race the
injected terminal states). Pose/leader state is fed through the driver
callbacks and the published goal/setpoint streams are captured by real ROS
subscribers.
"""
from __future__ import annotations
import sys, threading, time
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from swarm_uav_executor.drivers.ego_swarm import EgoSwarmDriver
from swarm_uav_executor.models import MotionGoal

NS = ""
results = []
goal_msgs = []
setpoint_msgs = []
lock = threading.Lock()


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print("%s %s %s" % ("PASS" if ok else "FAIL", name, detail))


def emit_state(driver, state):
    class Msg:
        def __init__(self, data):
            self.data = data
    driver._on_state(Msg(state))


def emit_pose(driver, x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    driver._on_pose(pose)


def emit_leader(driver, x, y, z):
    odom = Odometry()
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = z
    driver._on_leader_odom(odom)


def main():
    rospy.init_node("smoke_commands", anonymous=True)
    rospy.set_param("/uav_id", "A01")
    rospy.Subscriber(NS + "/goal", PointStamped,
                     lambda m: (lock.acquire(), goal_msgs.append(m), lock.release()))
    rospy.Subscriber(NS + "/setpoint/ego", PositionTarget,
                     lambda m: (lock.acquire(), setpoint_msgs.append(m), lock.release()))
    rospy.Subscriber(NS + "/setpoint/follower", PositionTarget,
                     lambda m: (lock.acquire(), setpoint_msgs.append(m), lock.release()))
    time.sleep(0.5)
    driver = EgoSwarmDriver(
        namespace=NS, state_timeout_s=5.0, steady_s=0.3,
        follower_p_gain=1.0, follower_i_gain=0.0,
        follower_limit_xy=2.0, follower_limit_z=1.0,
        formation_offsets={"A02": (1.0, 0.0, 0.0)},
    )
    time.sleep(0.3)
    if not driver.health().ready:
        print("FAIL: driver not ready")
        return 1

    # 1) HOVER: freeze the current pose and keep republishing it.
    emit_pose(driver, 2.0, 3.0, 15.0)
    hover = MotionGoal(0, 0, 15, 0, command="HOVER", layer_z=15.0)
    ev = threading.Event()
    holder = {}
    def hover_run():
        holder["r"] = driver.start_move_to(hover, ev, time.monotonic() + 4.0)
    th = threading.Thread(target=hover_run, daemon=True)
    th.start()
    time.sleep(0.4)
    emit_pose(driver, 50.0, 50.0, 15.0)   # vehicle drifts away
    time.sleep(0.4)
    ev.set(); th.join(2.0)
    with lock:
        sp = list(setpoint_msgs)
    hover_ok = len(sp) >= 2 and all(
        abs(m.position.x - 2.0) < 1e-6 and abs(m.position.y - 3.0) < 1e-6
        for m in sp[-2:])
    check("HOVER freezes pose", hover_ok,
          "setpoints=%d last=(%.2f,%.2f)" % (len(sp),
              sp[-1].position.x if sp else -1,
              sp[-1].position.y if sp else -1))

    # 2) FAULT_EXIT: vertical-first to 8 m, then horizontal.
    with lock:
        goal_msgs[:] = []
    emit_pose(driver, 1.0, 1.0, 15.0)
    fe = MotionGoal(4, 5, 8, 0, command="FAULT_EXIT", layer_z=8.0)
    holder2 = {}
    def fe_run():
        holder2["r"] = driver.start_move_to(fe, threading.Event(), time.monotonic() + 6.0)
    th2 = threading.Thread(target=fe_run, daemon=True)
    th2.start()
    time.sleep(0.2)
    emit_state(driver, "COMPLETED")
    time.sleep(0.5)
    emit_state(driver, "COMPLETED")
    th2.join(3.0)
    with lock:
        gl = list(goal_msgs)
    fe_ok = (len(gl) == 2 and abs(gl[0].point.z - 8.0) < 1e-6
             and abs(gl[1].point.x - 4.0) < 1e-6 and abs(gl[1].point.z - 8.0) < 1e-6
             and holder2.get("r") is not None and holder2["r"].success)
    check("FAULT_EXIT vertical-first then horizontal", fe_ok,
          "goals=%d r=%s" % (len(gl), holder2.get("r")))

    # 3) FOLLOW_ROUTE follower: leader odom + offset, PI setpoint published.
    with lock:
        setpoint_msgs[:] = []
    emit_pose(driver, 0.0, 0.0, 12.0)
    emit_leader(driver, 10.0, 0.0, 12.0)
    fl = MotionGoal(0, 0, 12, 0, command="FOLLOW_ROUTE", leader_id="A02",
                    formation_follow=True, layer_z=12.0,
                    formation_offset=(1.0, 0.0, 0.0))
    ev3 = threading.Event()
    holder3 = {}
    def fl_run():
        holder3["r"] = driver.start_move_to(fl, ev3, time.monotonic() + 3.0)
    th3 = threading.Thread(target=fl_run, daemon=True)
    th3.start()
    time.sleep(0.4)
    ev3.set(); th3.join(2.0)
    with lock:
        sp3 = list(setpoint_msgs)
    # target=(11,0,12), own=(0,0,12) -> error=(11,0,0); P=1 -> vel=(11,0,0);
    # clamp limit_xy=2.0 -> velocity=(2,0,0)，position=leader+offset=(11,0,12)
    fl_ok = len(sp3) >= 1 and abs(sp3[0].position.x - 11.0) < 0.02 \
        and abs(sp3[0].velocity.x - 2.0) < 0.02
    check("FOLLOW_ROUTE follower PI clamped with offset", fl_ok,
          "setpoints=%d first.x=%.3f vx=%.3f r=%s" % (len(sp3),
              sp3[0].position.x if sp3 else -1,
              sp3[0].velocity.x if sp3 else -1, holder3.get("r")))

    # 4) MOVE_TO vertical-first from low altitude to the 15 m layer.
    with lock:
        goal_msgs[:] = []
    emit_pose(driver, 2.0, 2.0, 5.0)
    mt = MotionGoal(10, 20, 15, 0, command="MOVE_TO")
    holder4 = {}
    def mt_run():
        holder4["r"] = driver.start_move_to(mt, threading.Event(), time.monotonic() + 6.0)
    th4 = threading.Thread(target=mt_run, daemon=True)
    th4.start()
    time.sleep(0.2)
    emit_state(driver, "COMPLETED")
    time.sleep(0.5)
    emit_state(driver, "COMPLETED")
    th4.join(3.0)
    with lock:
        g4 = list(goal_msgs)
    mt_ok = (len(g4) == 2 and abs(g4[0].point.z - 15.0) < 1e-6
             and abs(g4[1].point.x - 10.0) < 1e-6 and holder4.get("r") is not None
             and holder4["r"].success)
    check("MOVE_TO vertical-first then horizontal", mt_ok,
          "goals=%d r=%s" % (len(g4), holder4.get("r")))

    ok = all(ok for _, ok, _ in results)
    print("SMOKE_COMMANDS %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())