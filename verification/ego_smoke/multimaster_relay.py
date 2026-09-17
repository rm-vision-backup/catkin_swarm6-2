#!/usr/bin/env python3
"""Per-master neighbor-intent decoupler for multi-master reciprocal verification.

Onboard premise (MAVROS-like): this machine's ego node publishes plain
/exec_state /setpoint /trajectory_intent etc. (no /UAVn prefix) on its own
ROS master.  The swarm_topology_bridge delivers neighbor intents under the
source prefix /<neighbor>/trajectory_intent.  This node therefore only:

  1. subscribes to /<neighbor>/trajectory_intent for each configured neighbor
     (topics produced by the bridge), and
  2. republishes them to /neighbor_intent (the input of the local C++ node's
     onNeighborIntent).

No re-publication of the local intent is needed: the ego node already emits
/trajectory_intent which the bridge collects directly.
"""
from __future__ import annotations

import sys
import time

import rospy
from swarm_uav_interfaces.msg import UavTrajectoryIntent


def _argv_options():
    opts = {}
    for arg in sys.argv[1:]:
        if arg.startswith("_") and ":=" in arg:
            key, _, value = arg[1:].partition(":=")
            opts[key] = value
    return opts


def main():
    rospy.init_node("ego_multimaster_relay", anonymous=False)
    argv_opts = _argv_options()
    neighbors = [x.strip() for x in
                 rospy.get_param("~neighbors", argv_opts.get("neighbors", "")).split(",")
                 if x.strip()]
    if not neighbors:
        raise ValueError("neighbors must be set via ~neighbors or _neighbors:=")

    local_pub = rospy.Publisher("/neighbor_intent", UavTrajectoryIntent, queue_size=10)
    for nb in neighbors:
        rospy.Subscriber(nb + "/trajectory_intent", UavTrajectoryIntent,
                         lambda msg, out_pub=local_pub: out_pub.publish(msg))
    time.sleep(0.5)
    rospy.loginfo("ego multimaster relay ready neighbors=%s", neighbors)
    rospy.spin()


if __name__ == "__main__":
    sys.exit(main())