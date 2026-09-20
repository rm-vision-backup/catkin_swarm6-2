#!/bin/bash
# =============================================================================
# run_sim_four_uav.sh —— 4 机侦查测试（UAV6/UAV8/UAV10/UAV12）仿真单轮
#
# 与 15 机版（src/swarm_simulation/bashs/round_prepare_15sitl.sh）的差异只有四处：
#   1. 仿真层用 four_uav_spawn.launch，四机出生在 2x2、5 m（不是 15sim 的散列位置）
#   2. 机载层只起 6 8 10 12
#   3. 地面站用 gcs_sim_four_uav.launch（四机编队表 + 四机标定集合），且 bridge/backend/
#      calibration/tcp 合在一条 roslaunch 里 —— 消除了 15 机版 gcs_bridge_only →
#      gcs_backend_only 的 11310 master 竞态
#   4. 任务阶段跑 driver.py（本测试自己的编排），不是 task_test.py
#
# 阶段：clean -> .ros_home 隔离 -> sim(4 px4) -> onboard(4) -> gcs+标定 -> tcp(39001)
#        -> takeoff(4) -> task -> RESULT -> 无条件 cleanup
#
# 用法：
#   bash verification/four_uav_recon/sitl/run_sim_four_uav.sh [--keep-ros-home-on-fail 1]
#
# 退出码：0 = 任务 SUCCESS；1 = 失败；2 = 用法错
# =============================================================================
set -u
WS=/home/yjq/catkin_swarm6-2
cd "$WS" || exit 2

UAVS="6 8 10 12"
NUAV=4
KEEP_ON_FAIL=1
MISSION_ID="four-uav-recon-001"

# 机载任务执行超时（ego_swarm/task_timeout_s）：canonical 默认 200 s 短于侦查航线
# （领机约 129 m，预计 160~200 s），会误判 DRIVER_TIMEOUT。与实机启动命令取同一值。
export EGO_TASK_TIMEOUT_S=280

while [ "$#" -gt 0 ]; do
  case "$1" in
    --keep-ros-home-on-fail) KEEP_ON_FAIL="${2:-1}"; shift 2 ;;
    -h|--help) sed -n '1,25p' "$0"; exit 0 ;;
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

cleanup() { bash "$WS/src/swarm_simulation/bashs/cleanup.sh" >/dev/null 2>&1 || true; }
trap cleanup EXIT

# --- [1] 预清场 ------------------------------------------------------------
log "===== [1] pre-clean ====="
bash "$WS/src/swarm_simulation/bashs/cleanup.sh" >/dev/null 2>&1 || true
sleep 2

# --- [2] 仿真层：4 机 PX4 + Gazebo ----------------------------------------
log "===== [2] sim ($NUAV px4: $UAVS, master 11300) ====="
export ROS_MASTER_URI=http://localhost:11300
export ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
setsid nohup roslaunch verification/four_uav_recon/sitl/four_uav_spawn.launch gui:=false \
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

# --- [3] 机载层：只起 4 机（每机独立 Master） ------------------------------
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

# --- [4] 地面站 + 标定（一条 roslaunch） -----------------------------------
log "===== [4] gcs + calibration (master 11310) ====="
export ROS_MASTER_URI=http://localhost:11310
export ROS_HOSTNAME=localhost
setsid nohup roslaunch verification/four_uav_recon/sitl/gcs_sim_four_uav.launch \
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

# 起飞后静置，等价 15 机版的 wait 30s，等各机高度稳定
sleep 30

# --- [7] 任务：编队准备 + 侦查 ---------------------------------------------
log "===== [7] task ($MISSION_ID) ====="
cd "$WS"
python3 verification/four_uav_recon/driver.py \
  --package verification/four_uav_recon/package \
  --tcp-host 127.0.0.1 --tcp-port 39001 \
  --mission-id "$MISSION_ID" \
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
  DEST="$WS/.ros_home_round_four_uav"
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
