#!/usr/bin/env bash
# 14–16 架 A/B 联合任务入口。不隐式部署、起飞或启动任务。
set -euo pipefail
WS=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS"
DIR=verification/full_fleet_joint
SPEC=${FULL_FLEET_SPEC:-$DIR/generated/mission_spec.json}
command=${1:-help}
if [ "$#" -gt 0 ]; then shift; fi

case "$command" in
  generate)
    exec python3 "$DIR/generate_mission.py" "$@"
    ;;
  onboard-commands)
    if [ ! -f "$DIR/generated/onboard_commands.sh" ]; then
      echo "还没有生成任务包；先运行：bash $DIR/flow.sh generate --available ..." >&2
      exit 2
    fi
    cat "$DIR/generated/onboard_commands.sh"
    ;;
  odom|bag|stage1)
    if [ ! -f "$DIR/generated/flow_env.sh" ]; then
      echo "还没有生成任务包；先运行：bash $DIR/flow.sh generate --available ..." >&2
      exit 2
    fi
    # shellcheck disable=SC1090
    source "$DIR/generated/flow_env.sh"
    exec bash verification/three_uav_recon/air_flow.sh "$command" "$@"
    ;;
  dry-run)
    exec python3 "$DIR/driver.py" --dry-run --spec "$SPEC" "$@"
    ;;
  run)
    exec python3 "$DIR/driver.py" --execute --spec "$SPEC" "$@"
    ;;
  *)
    cat <<'EOM'
用法：
  bash verification/full_fleet_joint/flow.sh generate --available 1-16 [--leader UAV8]
  bash verification/full_fleet_joint/flow.sh onboard-commands
  bash verification/full_fleet_joint/flow.sh odom|bag [秒]|stage1 [秒]
  bash verification/full_fleet_joint/flow.sh dry-run [--until 阶段]
  bash verification/full_fleet_joint/flow.sh run [--until 阶段] [--log 路径]
EOM
    [ "$command" = help ] || exit 2
    ;;
esac
