#!/bin/bash
# =============================================================================
# run_sim_single_uav.sh —— 单机预验证（UAV6/A06）仿真单轮：起飞 + 准备编队 + 侦查
#
# 目的：四机集群上真机前先跑一架，确认"链路 / 标定 / 任务下发 / 安全兜底"都对。
#       代码路径与四机完全一致（同一 driver、同一 prepare-a / coverage-segment-1 计划、
#       同一 GCS 后端），只是机数缩到 1：A06 自己当领机走完侦查航线（没有跟机）。
#
# 与 run_sim_four_uav.sh（四机版）的差异：
#   1. UAVS="6"、NUAV=1
#   2. 仿真层用 single_uav_spawn.launch（只 iris_5，出生点与四机版 UAV6 相同）
#   3. 地面站仍用四机版的 gcs_sim_four_uav.launch，但用 arg 覆盖成单机配置
#      （formation_offsets_single_uav.yaml / calibration_single_uav.yaml）
#   4. 任务阶段 driver 显式 --phases prepare coverage（单机包两个计划都有）
#   5. EGO_TASK_TIMEOUT_S=280 与四机版同值：侦查航线约 84 m，保留裕度、免改航点后踩线
#
# 阶段：clean -> .ros_home 隔离 -> sim(1 px4) -> onboard(1) -> gcs+标定 -> tcp(39001)
#        -> takeoff(1) -> task(prepare+coverage) -> RESULT -> 无条件 cleanup
#
# 用法：
#   bash verification/single_uav_recon/sitl/run_sim_single_uav.sh [--keep-ros-home-on-fail 1]
#
# 退出码：0 = 任务 SUCCESS；1 = 失败；2 = 用法错
# =============================================================================
set -u
# 工作空间根 = 脚本上三级（verification/single_uav_recon/sitl → 工作空间根）
WS=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$WS" || exit 2

UAVS="6"
NUAV=1
KEEP_ON_FAIL=1
MISSION_ID="single-uav-prepare-001"
SINGLE_CFG="$WS/src/tcp_to_ros/config"

# 机载任务执行超时（ego_swarm/task_timeout_s）：侦查航线约 84 m，canonical 默认 200 s
# 会在航程未走完时判 DRIVER_TIMEOUT。与四机版、实机启动命令取同一值。
export EGO_TASK_TIMEOUT_S=280

while [ "$#" -gt 0 ]; do
  case "$1" in
    --keep-ros-home-on-fail) KEEP_ON_FAIL="${2:-1}"; shift 2 ;;
    -h|--help) sed -n '1,26p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

log() { echo "[$(date '+%H:%M:%S')] $*"; }

# env.sh 提供 WS / PX4_ROOT / HLOG / ROS_HOME 以及 ROS 与 devel 的 source。
# 它内部会 source "$WS/devel/setup.bash"，所以本工作空间必须先 catkin build。
# shellcheck source=/dev/null
source "$WS/src/swarm_simulation/bashs/env.sh"

# 每轮从干净 .ros_home 开始
rm -rf "$WS/.ros_home" && mkdir -p "$WS/.ros_home/log"
mkdir -p "$HLOG"

# 注意：pid 文件名必须与四机版/15 机版一致（01_sim / 02_onboard / 04_gcs_backend /
# 06_takeoff / 07_task_*），cleanup.sh 按固定名收割进程，改名会收不干净。
cleanup() { bash "$WS/src/swarm_simulation/bashs/cleanup.sh" >/dev/null 2>&1 || true; }
trap cleanup EXIT

# --- [1] 预清场 ------------------------------------------------------------
log "===== [1] pre-clean ====="
bash "$WS/src/swarm_simulation/bashs/cleanup.sh" >/dev/null 2>&1 || true
sleep 2

# --- [2] 仿真层：1 机 PX4 + Gazebo ----------------------------------------
log "===== [2] sim ($NUAV px4: $UAVS, master 11300) ====="
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
setsid nohup roslaunch verification/single_uav_recon/sitl/single_uav_spawn.launch gui:=false \
  > "$HLOG/01_sim.log" 2>&1 < /dev/null &
echo $! > "$HLOG/01_sim.pid"

SIM_OK=0
for i in $(seq 1 60); do
  G=$(timeout 5 env ROS_MASTER_URI=http://localhost:11300 rosnode list 2>/dev/null | grep -c '/gazebo' || true)
  P=$(pgrep -x px4 | wc -l)
  log "sim gazebo=$G px4=$P/$NUAV (t=${i}x5s)"
  [ "$G" -ge 1 ] && [ "$P" -ge "$NUAV" ] && SIM_OK=1 && break
  sleep 5
done
[ "$SIM_OK" = 1 ] || { log "FAIL sim not ready"; exit 1; }

# --- [3] 机载层：只起 1 机（独立 Master 11316） ----------------------------
log "===== [3] onboard ($UAVS) ====="
setsid nohup bash src/swarm_uav_executor/scripts/startup_offboard_ego.sh $UAVS \
  > "$HLOG/02_onboard.log" 2>&1 < /dev/null &
echo $! > "$HLOG/02_onboard.pid"

ON_OK=0
for i in $(seq 1 45); do
  R=$(grep -c '^ready UAV' "$HLOG/02_onboard.log" 2>/dev/null || true)
  log "onboard ready=$R/$NUAV (t=${i}x5s)"
  [ "$R" -ge "$NUAV" ] && ON_OK=1 && break
  sleep 5
done
[ "$ON_OK" = 1 ] || { log "FAIL onboard not ready"; exit 1; }

# --- [4] 地面站 + 标定（一条 roslaunch；覆盖成单机配置） --------------------
log "===== [4] gcs + calibration (master 11310) ====="
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
setsid nohup roslaunch verification/four_uav_recon/sitl/gcs_sim_four_uav.launch \
  formation_offsets_file:="$SINGLE_CFG/formation_offsets_single_uav.yaml" \
  calibration_config_file:="$SINGLE_CFG/calibration_single_uav.yaml" \
  > "$HLOG/04_gcs.log" 2>&1 < /dev/null &
echo $! > "$HLOG/04_gcs_backend.pid"

CAL_OK=0
for i in $(seq 1 40); do
  C=$(timeout 3 env ROS_MASTER_URI=http://localhost:11310 \
      rosparam get /group_a/calibration_ready 2>/dev/null || echo false)
  log "calibration_ready=$C (t=${i}x5s)"
  [ "$C" = "true" ] && CAL_OK=1 && break
  sleep 5
done
[ "$CAL_OK" = 1 ] || { log "FAIL calibration not ready"; exit 1; }

# --- [5] TCP server --------------------------------------------------------
log "===== [5] tcp server (39001) ====="
TCP_OK=0
for i in $(seq 1 12); do
  L=$(ss -ltn 2>/dev/null | grep -c ':39001 ' || true)
  log "tcp listening=$L (t=${i}x3s)"
  [ "$L" -ge 1 ] && TCP_OK=1 && break
  sleep 3
done
[ "$TCP_OK" = 1 ] || { log "FAIL tcp not listening"; exit 1; }

# --- [5.5] 等 MAVROS 参数表就绪 --------------------------------------------
# 单机启动快（onboard → takeoff 只有 ~10 s），而 MAVROS 的 param/set 需要先把飞控的
# 参数表拉完；未拉完时 set 会失败（2026-09-21 实测：报 "param COM_RCL_EXCEPT=4 被拒绝"
# 或 service 异常）。四机/15 机版因为要等多机标定，天然多等 30~60 s 才撞不上。
#
# 判据必须看 success（只看 integer 会误判：失败时 mavros 返回 success: False + 默认 0）。
log "===== [5.5] wait mavros param table (master 11316) ====="
TBL_OK=0
for i in $(seq 1 45); do
  R=$(timeout 3 env ROS_MASTER_URI=http://localhost:11316 ROS_HOSTNAME=localhost \
      rosservice call /mavros/param/get COM_RCL_EXCEPT 2>/dev/null)
  S=$(printf '%s' "$R" | awk '/^success:/{print $2}')
  V=$(printf '%s' "$R" | awk '/integer:/{print $2}')
  log "param get: success=${S:-?} value=${V:-<none>} (t=${i}x2s)"
  [ "$S" = "True" ] && TBL_OK=1 && break
  sleep 2
done
[ "$TBL_OK" = 1 ] || log "WARN 参数表 90 s 内未就绪，仍继续（takeoff 会给出具体报错）"

# --- [6] 起飞：HOLD -> arm -> OFFBOARD，软起飞 5 m 由 ego TAKEOFF 完成 -------
log "===== [6] takeoff ($UAVS) ====="
cd "$WS"
setsid nohup python3 src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py $UAVS \
  > "$HLOG/06_takeoff.log" 2>&1 < /dev/null &
echo $! > "$HLOG/06_takeoff.pid"

TK_OK=0
for i in $(seq 1 40); do
  K=$(grep -c "成功 $NUAV/$NUAV" "$HLOG/06_takeoff.log" 2>/dev/null || true)
  log "takeoff ok=$K (t=${i}x5s)"
  [ "$K" -ge 1 ] && TK_OK=1 && break
  sleep 5
done
[ "$TK_OK" = 1 ] || { log "FAIL takeoff"; tail -20 "$HLOG/06_takeoff.log"; exit 1; }

# 起飞后静置，等价 15 机版的 wait 30s，等高度稳定
sleep 30

# --- [7] 任务：准备编队 + 侦查航线 -----------------------------------------
log "===== [7] task ($MISSION_ID, phases=prepare+coverage) ====="
cd "$WS"
python3 verification/four_uav_recon/driver.py \
  --package verification/single_uav_recon/package \
  --tcp-host 127.0.0.1 --tcp-port 39001 \
  --mission-id "$MISSION_ID" --phases prepare coverage \
  --log "$HLOG/07_task.jsonl" \
  2>&1 | tee "$HLOG/07_task.log"
RC=${PIPESTATUS[0]}

# --- RESULT ----------------------------------------------------------------
log "===== RESULT ====="
if [ "$RC" = "0" ]; then
  log "ROUND RESULT: SUCCESS"
  exit 0
fi
log "ROUND RESULT: FAILURE (driver exit=$RC)"

# --- [8] 收口（trap 也会兜底，这里显式跑一次以保留 .ros_home 归档顺序） ------
if [ "$KEEP_ON_FAIL" = 1 ]; then
  DEST="$WS/.ros_home_round_single_uav"
  rm -rf "$DEST"
  cp -a "$WS/.ros_home" "$DEST"
  # 机载各节点 stdout（driver/executor 的日志与异常栈）在 cleanup.sh:53 会被删掉，
  # 先归档，否则失败原因无法事后定位。
  mkdir -p "$DEST/logs"
  for idx in $UAVS; do
    cp -a "$WS/.tmp/logs/UAV${idx}_offboard_ego.log" "$DEST/logs/" 2>/dev/null || true
  done
  log "kept flight records -> $DEST (含 logs/UAV*_offboard_ego.log)"
fi
exit 1
