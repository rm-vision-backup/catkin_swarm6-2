#!/usr/bin/env bash
# 两个编队侦查流程入口（每轮一个编队）。生成/打印/试运行不会启动仿真。
set -euo pipefail
WS=$(cd "$(dirname "$0")/../.." && pwd)
cd "$WS"
DIR=verification/two_teams_recon
command=${1:-help}
if [ "$#" -gt 0 ]; then shift; fi

team_arg() {
  local team=1
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --team) team=$2; shift 2 ;;
      *) shift ;;
    esac
  done
  echo "$team"
}

case "$command" in
  generate)
    exec python3 "$DIR/generate_mission.py" "$@"
    ;;
  onboard-commands)
    team=$(team_arg "$@")
    file="$DIR/generated/team${team}/onboard_commands.sh"
    if [ ! -f "$file" ]; then
      echo "还没有生成 team${team}；先运行：bash $DIR/flow.sh generate --team ${team}" >&2
      exit 2
    fi
    cat "$file"
    ;;
  dry-run)
    exec python3 "$DIR/driver.py" --dry-run "$@"
    ;;
  run)
    exec python3 "$DIR/driver.py" --execute "$@"
    ;;
  odom|bag|stage1)
    team=1
    rest=()
    while [ "$#" -gt 0 ]; do
      case "$1" in
        --team) team=$2; shift 2 ;;
        *) rest+=("$1"); shift ;;
      esac
    done
    env_file="$DIR/generated/team${team}/flow_env.sh"
    if [ ! -f "$env_file" ]; then
      echo "还没有生成 team${team}；先运行：bash $DIR/flow.sh generate --team ${team}" >&2
      exit 2
    fi
    # shellcheck disable=SC1090
    source "$env_file"
    exec bash verification/three_uav_recon/air_flow.sh "$command" ${rest[@]+"${rest[@]}"}
    ;;
  sitl)
    exec bash "$DIR/sitl/run_recording.sh" "$@"
    ;;
  *)
    cat <<'EOM'
用法：
  bash verification/two_teams_recon/flow.sh generate            # 一次生成两队
  bash verification/two_teams_recon/flow.sh onboard-commands --team 1|2
  bash verification/two_teams_recon/flow.sh dry-run --team 1|2 [--until 阶段]
  bash verification/two_teams_recon/flow.sh run --team 1|2 [--until 阶段] [--log 路径]
  bash verification/two_teams_recon/flow.sh odom|bag [秒]|stage1 --team 1|2
  bash verification/two_teams_recon/flow.sh sitl --outdir <新目录> [--headless] [--no-keep-alive]
EOM
    [ "$command" = help ] || exit 2
    ;;
esac
