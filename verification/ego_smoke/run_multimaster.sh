#!/bin/bash
# 双/三 Master reciprocal 验证启动脚本
# 用法:
#   bash verification/ego_smoke/run_multimaster.sh up dual    # UAV1:11311 + UAV2:11312
#   bash verification/ego_smoke/run_multimaster.sh up triple  # UAV1:11311 + UAV2:11312 + UAV3:11313
#   bash verification/ego_smoke/run_multimaster.sh down
set -u
WS=/home/yjq/catkin_swarm6-2
RELAY=$WS/verification/ego_smoke/multimaster_relay.py
BRIDGE_LAUNCH=$WS/verification/ego_smoke/multimaster_bridge.launch
PKG_SRC=$WS/src/swarm_topology_bridge/scripts/bridge_node.py

base_env() {
  source /opt/ros/noetic/setup.bash
  source "$WS/devel/setup.bash"
}

start_master() {
  local port=$1 name=$2 mode=$3
  export ROS_MASTER_URI="http://localhost:$port"
  export ROS_HOSTNAME=localhost
  roscore > /tmp/mm_${name}_roscore.log 2>&1 &
  echo $! > /tmp/mm_${name}_roscore.pid
  sleep 3

  base_env
  roslaunch ego_planner_driver ego_planner_driver.launch \
    uav_id:=A${name#UAV} exec_target:=$name \
    > /tmp/mm_${name}_ego.log 2>&1 &
  echo $! > /tmp/mm_${name}_ego.pid
  sleep 2

  # bridge 通过 launch 加载专用拓扑
  base_env
  roslaunch "$BRIDGE_LAUNCH" uav_name:=$name \
    > /tmp/mm_${name}_bridge.log 2>&1 &
  echo $! > /tmp/mm_${name}_bridge.pid
  sleep 1

  local nbrs=""
  if [ "$mode" = dual ]; then
    case $name in
      UAV1) nbrs=UAV2 ;;
      UAV2) nbrs=UAV1 ;;
    esac
  else
    case $name in
      UAV1) nbrs=UAV2,UAV3 ;;
      UAV2) nbrs=UAV1,UAV3 ;;
      UAV3) nbrs=UAV1,UAV2 ;;
    esac
  fi

  # relay 直接用 python3 运行（不在包里）
  base_env
  python3 "$RELAY" _neighbors:=$nbrs \
    > /tmp/mm_${name}_relay.log 2>&1 &
  echo $! > /tmp/mm_${name}_relay.pid
}

stop_all() {
  for n in UAV1 UAV2 UAV3; do
    for role in relay bridge ego roscore; do
      if [ -f /tmp/mm_${n}_${role}.pid ]; then
        kill "$(cat /tmp/mm_${n}_${role}.pid)" 2>/dev/null
        rm -f /tmp/mm_${n}_${role}.pid
      fi
    done
  done
  sleep 1
  # 兜底清理（精确匹配，避免误杀自身）
  ps aux | grep -E '[r]oscore|[b]ridge_node.py|[m]ultimaster_relay|[e]go_planner_driver_node' \
    | awk '{print $2}' | xargs -r kill 2>/dev/null || true
}

mode="${2:-dual}"
case "${1:-}" in
  up)
    stop_all
    sleep 1
    if [ "$mode" = triple ]; then
      start_master 11311 UAV1 triple
      start_master 11312 UAV2 triple
      start_master 11313 UAV3 triple
      echo "triple-master started: UAV1=11311 UAV2=11312 UAV3=11313"
    else
      start_master 11311 UAV1 dual
      start_master 11312 UAV2 dual
      echo "dual-master started: UAV1=11311 UAV2=11312"
    fi
    ;;
  down)
    stop_all
    echo "multi-master stopped"
    ;;
  *)
    echo "usage: $0 up|down [dual|triple]"
    exit 1
    ;;
esac