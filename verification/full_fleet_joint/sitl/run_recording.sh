#!/usr/bin/env bash
# 仅本机 PX4/Gazebo；不连接实机，不调用全局 cleanup、不覆盖旧日志。
set -eo pipefail
cd "$(dirname "$0")/../../.."
source /opt/ros/noetic/setup.bash
source devel/setup.bash
export PX4_ROOT=${PX4_ROOT:-$HOME/PX4_Firmware}
source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"
export ROS_HOSTNAME=localhost ROS_MASTER_URI=http://localhost:11310
unset ROS_IP
export LIBGL_ALWAYS_SOFTWARE=1
exec python3 verification/full_fleet_joint/sitl/record_run.py "$@"
