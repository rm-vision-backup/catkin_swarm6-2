#!/usr/bin/env python3
"""SITL 批量 AUTO.LAND（每机独立 ROS Master 11311..11325）。

用法：
    python3 verification/two_teams_recon/sitl/land_15_sitl.py 1 2 3 4 5 6 7

逐机在独立子进程内切 AUTO.LAND 并等待 armed=False；全部落地返回 0。
"""
from __future__ import annotations

import argparse
import subprocess
import sys

ROS_SETUP = "/opt/ros/noetic/setup.bash"
WS = "/home/yjq/catkin_swarm6-2"
TIMEOUT_S = 180.0


def land_single(idx: int) -> int:
    import time

    import rospy
    from mavros_msgs.msg import State
    from mavros_msgs.srv import SetMode

    rospy.init_node("land_uav%d" % idx, anonymous=True, disable_signals=True)
    rospy.wait_for_service("/mavros/set_mode", timeout=10)
    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    rospy.wait_for_message("/mavros/state", State, timeout=10)
    response = set_mode(base_mode=0, custom_mode="AUTO.LAND")
    if not response.mode_sent:
        print("UAV%d: AUTO.LAND 被拒绝" % idx, flush=True)
        return 1
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            state = rospy.wait_for_message("/mavros/state", State, timeout=5)
        except Exception:
            continue
        if not state.armed:
            print("UAV%d: landed/disarmed" % idx, flush=True)
            return 0
    print("UAV%d: 等待落地超时" % idx, flush=True)
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("uavs", nargs="+", type=int)
    parser.add_argument("--single", type=int, default=None)
    args = parser.parse_args()
    idxs = [args.single] if args.single else args.uavs
    results = {}
    for idx in idxs:
        cmd = [
            "bash", "-c",
            "source %s && source %s/devel/setup.bash && "
            "export ROS_MASTER_URI=http://localhost:%d && "
            "exec python3 %s --single %d dummy"
            % (ROS_SETUP, WS, 11310 + idx, __file__, idx),
        ]
        results[idx] = "OK" if subprocess.run(cmd).returncode == 0 else "FAIL"
    print("=== AUTO.LAND 汇总 ===")
    for idx in idxs:
        print("UAV%-2d (%d): %s" % (idx, 11310 + idx, results[idx]))
    return 0 if all(value == "OK" for value in results.values()) else 1


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--single":
        parser = argparse.ArgumentParser()
        parser.add_argument("--single", type=int, required=True)
        parser.add_argument("dummy", nargs="?", default=None)
        parsed = parser.parse_args()
        sys.exit(land_single(parsed.single))
    sys.exit(main())
