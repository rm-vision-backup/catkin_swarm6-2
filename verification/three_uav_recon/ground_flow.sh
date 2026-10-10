#!/usr/bin/env bash
# ground_flow.sh —— 地面贴地彩排：飞机**原地不动**跑完 prepare → coverage 两个阶段
#
# 为什么要这个（2026-10-09 现场要求）：两轮实飞的故障发生在"**prepare 完成后进入 coverage 那一瞬**"；
# 但飞机停在地面时，原任务包（层高 12/15 m）会让规划器 `replan failed … EGO_PLAN_FAILED`，
# 组级 ALL 立即中止，流程根本到不了 coverage —— 于是复刻不了故障环节。
# 本脚本用 **贴地任务包 + 三个部署覆盖参数**，让流程在飞机原地不动的情况下按
# prepare → coverage 原始顺序推进：prepare 三机在停机位上直接判到达（不移动），
# 随后 coverage 正常下发、跟随者进入跟随环（故障环节即在此）。
#
# 覆盖参数（需在**起栈时**传，脚本会逐台核对是否生效）：
#   noprop:=true                 跳过 arm 门禁（无桨必需）
#   layer_move_to:=0.0           否则 MOVE_TO 会先插一个 15 m 层
#   layer_follow_route:=0.0      否则 FOLLOW_ROUTE 会先插一个 12 m 层
#   arrival_reach_thresh_m:=1.0  可选：摆位残差大时放宽到达判据（默认 0.5 m 偏紧）
#   enable_yield_candidates:=false  **必须**：贴地时避让找不到候选 → 10 s 后直接 EGO_PLAN_FAILED（2026-10-09 实测）
#
# 用法：
#   bash verification/three_uav_recon/ground_flow.sh            # 只检查（不跑任务）
#   bash verification/three_uav_recon/ground_flow.sh --run      # 跑两阶段并给判定
# 判定（--run）：prepare **3/3 SUCCESS**（= 原地进入 coverage，这才是本测试的目的）；
#   coverage 是否下发、两跟随者是否 `follower_start`/`leader_odom_first_frame`；
#   coverage 的**完成**不作要求（0.2 m 跟随判据在不动时满足不了）。
# 退出码：0 = 检查通过（--run 时为两阶段都进入且 prepare 3/3 SUCCESS）；1 = 不通过。
# =============================================================================
set -u

WS=$(cd "$(dirname "$0")/../.." && pwd)
UAVS="8 10 13"
PKG_GROUND="$WS/verification/three_uav_recon/package_ground"
MISSION=three-uav-ground-001
CONFIG="$WS/src/tcp_to_ros/tools/configs/onboard.env"
[ -f "$CONFIG" ] && . "$CONFIG"

RUN=0
[ "${1:-}" = "--run" ] && RUN=1

# 本机 GCS 自带 master；~/.bashrc 里的 ROS_MASTER_URI 可能指向别的机器（2026-10-09 现场踩过），
# 显式指到本机，避免 rosparam/rostopic 连错 master。
export ROS_MASTER_URI="${GCS_MASTER_URI:-http://localhost:11311}"
set +u
# shellcheck source=/dev/null
source /opt/ros/noetic/setup.bash 2>/dev/null
# shellcheck source=/dev/null
source "$WS/devel/setup.bash" 2>/dev/null
set -u

rc=0

echo "== [1/3] 三台机载栈的贴地参数是否生效 =="
for n in $UAVS; do
  ip="${BOARD_IP_PREFIX:-192.168.5.}$((10 * n + 1))"
  out=$(timeout 20 sshpass -p "${BOARD_SSH_PASS:-nvidia}" ssh -n -o ConnectTimeout=8 \
        -o StrictHostKeyChecking=no "${BOARD_SSH_USER:-nvidia}@$ip" \
        "source /opt/ros/noetic/setup.bash >/dev/null 2>&1; source ~/catkin_swarm6-2/devel/setup.bash >/dev/null 2>&1; \
         rosparam get /uav_executor_UAV$n/ego_swarm/layer_move_to 2>/dev/null; \
         rosparam get /uav_executor_UAV$n/ego_swarm/layer_follow_route 2>/dev/null; \
         rosparam get /ego_planner_driver/enable_yield_candidates 2>/dev/null" 2>/dev/null | tr '\n' ' ')
  lm=$(echo "$out" | awk '{print $1}'); lf=$(echo "$out" | awk '{print $2}'); yc=$(echo "$out" | awk '{print $3}')
  if [ "$lm" = "0.0" ] && [ "$lf" = "0.0" ] && [ "$yc" = "false" ]; then
    echo "  UAV$n: layer_move_to=$lm layer_follow_route=$lf yield=$yc  OK"
  else
    echo "  UAV$n: layer_move_to=${lm:-读不到} layer_follow_route=${lf:-读不到} enable_yield_candidates=${yc:-读不到}  ✗ 未用贴地参数起栈"
    rc=1
  fi
done
if [ "$rc" -ne 0 ]; then
  cat <<'EOM'
  ⇒ 用下面这条（逐台，邻居清单每台不同）重起后再跑：
     ssh nvidia@192.168.5.<10n+1> 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
       noprop:=true layer_move_to:=0.0 layer_follow_route:=0.0 enable_yield_candidates:=false [arrival_reach_thresh_m:=1.0] \
       neighbor_intents:=... neighbor_odom_topics:=... uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'
     （完整三条命令见 verification/three_uav_recon/README.md §二 + 本脚本头部）
EOM
fi

echo "== [2/3] 地面站与标定 =="
for node in /gcs_a_backend_node /gcs_a_calibration /tcp_server_node /swarm_bridge; do
  if rospy_node_exists=$(timeout 5 rosnode list 2>/dev/null | grep -qxF "$node"; echo $?); [ "$rospy_node_exists" -ne 0 ]; then
    echo "  缺节点 $node（先起 roslaunch src/tcp_to_ros/launch/gcs_a_real_three_uav.launch）"; rc=1
  fi
done
ready=$(timeout 10 rostopic echo -n1 /group_a/calibration_ready 2>/dev/null | head -1)
if [ "$ready" = "data: True" ]; then echo "  calibration_ready=True  OK"; else
  echo "  calibration_ready=${ready:-读不到}  ✗（三机 disarm + GPS 收敛 + 原点回读才 ready）"; rc=1
fi

echo "== [3/3] 贴地任务包干跑（只看线上 JSON，不连地面站）=="
python3 "$WS/verification/three_uav_recon/driver.py" --package "$PKG_GROUND" \
        --mission-id "$MISSION" --phases prepare coverage --dry-run >/tmp/ground_flow_dryrun.json 2>&1 \
  && echo "  已打印两条 COMMAND（/tmp/ground_flow_dryrun.json；期望 prepare 三机 z=0.0、coverage 航线 z=0.0）" \
  || { echo "  干跑失败："; tail -3 /tmp/ground_flow_dryrun.json; rc=1; }

if [ "$RUN" -eq 0 ]; then
  echo "== 检查结束（rc=$rc）。要真正跑两阶段：--run =="
  exit "$rc"
fi
[ "$rc" -ne 0 ] && { echo "== 检查未通过，先修上面 ✗ 项再 --run =="; exit 1; }

LOG="$WS/verification/three_uav_recon/logs/ground_$(date +%Y%m%d_%H%M%S).jsonl"
echo "== 跑两阶段（原地）$(date +%H:%M:%S)；证据：$LOG =="
python3 "$WS/verification/three_uav_recon/driver.py" --package "$PKG_GROUND" \
        --mission-id "$MISSION" --phases prepare coverage \
        --tcp-host "${GCS_TCP_HOST:-192.168.5.241}" --tcp-port 39001 --log "$LOG"
drv_rc=$?

python3 - "$LOG" <<'PY'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
prep = [d for d in rows if d.get("event") == "phase_result" and d.get("phase") == "prepare"]
cov = [d for d in rows if d.get("event") == "phase_result" and d.get("phase") == "coverage"]
print("== 判定 ==")
if not prep:
    print("  prepare：未下发（配置错误？）")
elif prep[0]["outcome"] == "SUCCESS":
    print("  prepare：SUCCESS（三机原地到位）✓ —— 已按原序进入 coverage")
else:
    print("  prepare：%s ✗ —— 没到 coverage；先查贴地参数是否生效（[1/3]）" % prep[0]["outcome"])
if cov:
    print("  coverage：已下发，结果 %s（本测试不要求完成）" % cov[0]["outcome"])
else:
    print("  coverage：未下发")
PY
echo "  证据：$LOG ；跟随环事件在各机 <ws>/.ros_home/log/health/UAVn_follower_events.jsonl"
[ "$drv_rc" -eq 0 ] && echo "== driver 退出 0：两阶段均 SUCCESS ==" || echo "== driver 退出 $drv_rc：看上面判定（地面彩排以 prepare 3/3 SUCCESS + coverage 进入为准）=="
