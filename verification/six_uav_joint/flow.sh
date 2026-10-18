#!/usr/bin/env bash
# 不隐式部署、起飞或启动任务。onboard-commands 仅打印逐机命令。
set -euo pipefail
WS=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS"
export AIR_FLOW_UAVS="6 7 8 9 10 11"
export AIR_FLOW_LEADER=7
export AIR_FLOW_DIR=six_uav_joint
command=${1:-help}
if [ "$#" -gt 0 ]; then shift; fi
case "$command" in
  odom|bag|stage1)
    exec bash verification/three_uav_recon/air_flow.sh "$command" "$@"
    ;;
  dry-run)
    exec python3 verification/six_uav_joint/driver.py --dry-run "$@"
    ;;
  run)
    exec python3 verification/six_uav_joint/driver.py --execute "$@"
    ;;
  onboard-commands)
    for n in 6 7 8 9 10 11; do
      intents=""; poses=""
      for peer in 6 7 8 9 10 11; do
        [ "$peer" = "$n" ] && continue
        intents+="${intents:+,}/UAV${peer}/trajectory_intent"
        poses+="${poses:+,}/UAV${peer}/swarm_pose"
      done
      printf 'bash src/tcp_to_ros/tools/onboard_ctl.sh start %s -- neighbor_intents:=%s neighbor_odom_topics:=%s uav_task_timeout_s:=180\n' \
        "$n" "$intents" "$poses"
    done
    ;;
  *)
    echo '用法：bash verification/six_uav_joint/flow.sh {onboard-commands|odom|bag [秒]|stage1 [秒]|dry-run|run --log 路径 [--until 阶段]}'
    [ "$command" = help ] || exit 2
    ;;
esac
