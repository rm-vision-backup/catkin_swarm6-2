#!/usr/bin/env bash
# 单 Master 完整生命周期冒烟：IDLE -> EXECUTING -> COMPLETED -> IDLE -> HOLD -> IDLE
# 用法: bash verification/ego_smoke/run_smoke_lifecycle.sh
set -u
WS=/home/yjq/catkin_swarm6-2
PORT=11311
LOG=/tmp/ego_smoke_lifecycle.log

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export ROS_MASTER_URI="http://localhost:$PORT"
export ROS_HOSTNAME=localhost

for pat in "roscore -p $PORT" "rosmaster --core" "ego_planner_driver_node"; do
    for pid in $(pgrep -f "$pat"); do
        if [ -d "/proc/$pid" ] && [ "$pid" != "$$" ]; then
            kill "$pid" 2>/dev/null || true
        fi
    done
done
sleep 1

nohup roscore -p "$PORT" > "$LOG" 2>&1 &
RC_PID=$!
sleep 3
nohup roslaunch ego_planner_driver ego_planner_driver.launch >> "$LOG" 2>&1 &
NODE_PID=$!
sleep 5

python3 "$WS/verification/ego_smoke/smoke_single_master.py" 2>&1 | tee /tmp/ego_smoke_lifecycle_out.txt
RC=${PIPESTATUS[0]}

for pid in "$NODE_PID" "$RC_PID"; do
    kill "$pid" 2>/dev/null || true
done
sleep 1
for pat in "roscore -p $PORT" "rosmaster --core" "ego_planner_driver_node"; do
    for pid in $(pgrep -f "$pat"); do
        if [ -d "/proc/$pid" ] && [ "$pid" != "$$" ]; then
            kill "$pid" 2>/dev/null || true
        fi
    done
done

echo "LIFECYCLE_SMOKE_EXIT=$RC"
exit "$RC"
