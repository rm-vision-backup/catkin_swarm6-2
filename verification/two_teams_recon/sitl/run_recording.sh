#!/usr/bin/env bash
# 15 机同一环境、一起标定、两批次运行；飞行结束后默认保留环境供 QGC 截图。
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
exec python3 verification/two_teams_recon/sitl/run_two_batches.py "$@"
