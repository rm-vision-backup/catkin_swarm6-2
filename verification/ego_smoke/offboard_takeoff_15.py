#!/usr/bin/env python3
"""15 机无头 SITL 批量起飞脚本（每机独立 ROS Master 11311..11325）。

职责（配合正确启动流程）：
  0. 先启动机载层（offboard 程序）：MAVROS + ego driver + executor + bridge，
     统一坐标系并待命；ego driver 启动后处于 TAKEOFF 状态，在 IDLE 下已持续
     30Hz 发布 /mavros/setpoint_position/local（经 setpoint_relay）。
  1. 本脚本对每架 UAV：
     a. arm 所有无人机
     b. 切 OFFBOARD（ego driver 在 TAKEOFF 状态检测到 arm+OFFBOARD 后
        自动执行软起飞：当前 XY 保持，Z 线性上升到 takeoff_height_m）
     c. 订阅 /exec_state 等待 TAKEOFF → IDLE 转换（表示起飞完成）

注意:
  - 无 RC 的 SITL 必须设 COM_RCL_EXCEPT=4，否则 RC 丢失触发 failsafe（RTL）
  - 起飞高度由 ego_planner_driver 的 takeoff_height_m 参数控制（默认 5.0m）
  - 本脚本不再调用 /mavros/cmd/takeoff（MAV_CMD_NAV_TAKEOFF），
    避免 PX4 preflight 在 set_gp_origin 修改 EKF2 origin 后拦截
"""
import argparse
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, ParamPush, ParamSet, SetMode
from std_msgs.msg import String as StringMsg

ROS_SETUP = "/opt/ros/noetic/setup.bash"
WS = "/home/yjq/catkin_swarm6-2"


class TakeoffUAV:
    def __init__(self, idx, timeout_s):
        self.idx = idx
        self.master_port = 11310 + idx
        self.timeout_s = timeout_s
        self.state = State()
        self.exec_state = None

    def on_state(self, msg):
        self.state = msg

    def on_exec_state(self, msg):
        self.exec_state = msg.data

    def wait_armed(self, timeout=8.0):
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < timeout:
            if self.state.armed:
                return True
            time.sleep(0.1)
        return False

    def wait_mode(self, mode, timeout=8.0):
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < timeout:
            if self.state.mode == mode:
                return True
            time.sleep(0.1)
        return False

    def wait_exec_state(self, state, timeout):
        """等待 exec_state 变为给定状态（TAKEOFF → IDLE 表示起飞完成）。"""
        t0 = time.time()
        while not rospy.is_shutdown() and time.time() - t0 < timeout:
            if self.exec_state == state:
                return True
            time.sleep(0.1)
        return False

    def run(self):
        os.environ["ROS_MASTER_URI"] = "http://localhost:%d" % self.master_port
        os.environ["ROS_HOSTNAME"] = "localhost"
        rospy.init_node("takeoff_uav%d" % self.idx, anonymous=True)
        rospy.loginfo("UAV%d: master=%d", self.idx, self.master_port)

        rospy.Subscriber("/mavros/state", State, self.on_state)
        rospy.Subscriber("/exec_state", StringMsg, self.on_exec_state)

        try:
            pose = rospy.wait_for_message(
                "/mavros/local_position/pose", PoseStamped, timeout=20)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: 无法获取 local_position/pose: %s", self.idx, exc)
            return 1

        rospy.loginfo("UAV%d: 当前位置(%.2f,%.2f,%.2f)",
                      self.idx, pose.pose.position.x, pose.pose.position.y,
                      pose.pose.position.z)

        # 0) 设置 PX4 参数（无 RC 的 SITL 必须）
        try:
            rospy.wait_for_service("/mavros/param/set", timeout=10)
            param_set = rospy.ServiceProxy("/mavros/param/set", ParamSet)
            for pid, ival in (("COM_RCL_EXCEPT", 4), ("NAV_RCL_ACT", 0)):
                pv = ParamValue()
                pv.integer = ival
                pv.real = 0.0
                resp = param_set(pid, pv)
                rospy.loginfo("UAV%d: param %s=%d success=%s",
                              self.idx, pid, ival, resp.success)
            rospy.wait_for_service("/mavros/param/push", timeout=10)
            push = rospy.ServiceProxy("/mavros/param/push", ParamPush)
            presp = push()
            rospy.loginfo("UAV%d: param push transfered=%s",
                          self.idx, presp.param_transfered)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("UAV%d: 参数设置失败（继续）: %s", self.idx, exc)

        # 1) arm（此时处于 HOLD / AUTO.LOITER 等模式）
        try:
            rospy.wait_for_service("/mavros/cmd/arming", timeout=10)
            arm = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: arm 服务不可用: %s", self.idx, exc)
            return 1

        ok = False
        for attempt in range(3):
            resp = arm(True)
            rospy.loginfo("UAV%d: 尝试 %d arm result=%s",
                          self.idx, attempt + 1, resp.success)
            if resp.success and self.wait_armed():
                ok = True
                break
            time.sleep(1.0)
        if not ok:
            rospy.logerr("UAV%d: arm 被拒绝（mode=%s armed=%s）",
                         self.idx, self.state.mode, self.state.armed)
            return 1
        rospy.loginfo("UAV%d: 已 armed（mode=%s）", self.idx, self.state.mode)

        # 2) 切 OFFBOARD（ego driver 在 TAKEOFF 状态自动执行软起飞）
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=10)
            set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        except rospy.ROSException as exc:
            rospy.logerr("UAV%d: set_mode 服务不可用: %s", self.idx, exc)
            return 1
        ok = False
        for attempt in range(3):
            resp = set_mode(0, "OFFBOARD")
            rospy.loginfo("UAV%d: 尝试 %d 切 OFFBOARD mode_sent=%s",
                          self.idx, attempt + 1, resp.mode_sent)
            if self.wait_mode("OFFBOARD"):
                ok = True
                break
            time.sleep(1.0)
        if not ok:
            rospy.logerr("UAV%d: 未能切 OFFBOARD（mode=%s armed=%s）",
                         self.idx, self.state.mode, self.state.armed)
            return 1
        rospy.loginfo("UAV%d: 已进入 OFFBOARD 且 armed，等待软起飞完成",
                      self.idx)

        # 3) 等待 ego driver 完成 TAKEOFF → IDLE（起飞完成）
        if not self.wait_exec_state("IDLE", self.timeout_s):
            rospy.logerr("UAV%d: 等待 TAKEOFF 完成超时（exec_state=%s）",
                         self.idx, self.exec_state)
            return 1
        rospy.loginfo("UAV%d: 软起飞完成，已进入 IDLE", self.idx)
        return 0


def main():
    parser = argparse.ArgumentParser(
        description="15 机批量 arm + OFFBOARD（ego driver 自动软起飞）")
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=15)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--individual", action="store_true",
                        help="逐个启动（不是并发），便于观察日志")
    args = parser.parse_args()

    results = {}
    idxs = list(range(args.start, args.end + 1))
    if args.individual:
        # 逐机串行：每个 UAV 在独立子进程内运行 --single，避免
        # rospy.init_node 同一进程重复调用的问题。
        import subprocess
        import subprocess as sp
        for idx in idxs:
            cmd = [
                "bash", "-c",
                "source %s && source %s/devel/setup.bash && "
                "export ROS_MASTER_URI=http://localhost:%d && "
                "exec python3 %s --single %d --timeout %.1f"
                % (ROS_SETUP, WS, 11310 + idx, __file__, idx, args.timeout),
            ]
            r = sp.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            results[idx] = "OK" if r.returncode == 0 else "FAIL(%d)" % r.returncode
    else:
        import subprocess
        procs = {}
        for idx in idxs:
            cmd = [
                "bash", "-c",
                "source %s && source %s/devel/setup.bash && "
                "export ROS_MASTER_URI=http://localhost:%d && "
                "exec python3 %s --single %d --timeout %.1f"
                % (ROS_SETUP, WS, 11310 + idx, __file__, idx, args.timeout),
            ]
            p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
            procs[idx] = p
        deadline = time.time() + args.timeout + 40
        for idx in idxs:
            p = procs[idx]
            while p.poll() is None and time.time() < deadline:
                time.sleep(1)
            if p.poll() is None:
                p.terminate()
                results[idx] = "TIMEOUT"
            else:
                results[idx] = "OK" if p.returncode == 0 else "FAIL(%d)" % p.returncode

    print("=== arm+OFFBOARD 汇总 ===")
    for idx in idxs:
        print("UAV%-2d (%d): %s" % (idx, 11310 + idx, results.get(idx, "?")))
    ok = [i for i, v in results.items() if v == "OK"]
    print("成功 %d/%d: %s" % (len(ok), len(idxs), sorted(ok)))
    return 0 if len(ok) == len(idxs) else 1


def _run_single(idx, args):
    return 0 if TakeoffUAV(idx, args.timeout).run() == 0 else 1


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        p = argparse.ArgumentParser()
        p.add_argument("--single", type=int)
        p.add_argument("--timeout", type=float, default=60.0)
        a, _ = p.parse_known_args()
        sys.exit(0 if TakeoffUAV(a.single, a.timeout).run() == 0 else 1)
    sys.exit(main())