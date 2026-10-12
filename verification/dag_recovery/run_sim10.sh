#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DIRECTORY="$ROOT/verification/dag_recovery"
FAULTS=3
OUTDIR=""
WORKER_PYTHON=""
CHECKPOINT=""
EXECUTE=false
CHECK=false
ALGORITHM_ONLY=false

while (($#)); do
  case "$1" in
    --faults) FAULTS="${2:?missing fault count}"; shift 2 ;;
    --outdir) OUTDIR="${2:?missing output directory}"; shift 2 ;;
    --worker-python) WORKER_PYTHON="${2:?missing Python path}"; shift 2 ;;
    --checkpoint) CHECKPOINT="${2:?missing model.pt path}"; shift 2 ;;
    --check) CHECK=true; shift ;;
    --check-algorithm) ALGORITHM_ONLY=true; shift ;;
    --execute) EXECUTE=true; shift ;;
    --dry-run) shift ;;
    -h|--help)
      echo 'Check: bash verification/dag_recovery/run_sim10.sh --faults 3 --check'
      echo 'Offline DAGL: bash verification/dag_recovery/run_sim10.sh --faults 3 --checkpoint /path/to/model.pt --outdir /tmp/dag10_offline_1 --check-algorithm'
      echo 'Fly SITL: bash verification/dag_recovery/run_sim10.sh --faults 3 --checkpoint /path/to/model.pt --outdir /tmp/dag10_sitl_1 --execute'
      echo 'Optional: --worker-python /absolute/path/to/installed/python (auto-detected by default)'
      exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

[[ "$FAULTS" == 3 || "$FAULTS" == 4 ]] || { echo 'Use --faults 3 or 4' >&2; exit 2; }
# 场景与离线快照可用环境变量覆盖（随机布局演示：训练/预检/仿真必须用同一份场景；
# 快照需与该场景布局一致，可由 scenarios/make_random_scenario.py --fixture-out 生成）。
SCENARIO_FILE="${DAGL_SCENARIO:-$DIRECTORY/scenarios/10.sitl.json}"
SNAPSHOT_FILE="${DAGL_SNAPSHOT:-$DIRECTORY/fixtures/ten_enu.json}"
export DAGL_SCENARIO="$SCENARIO_FILE"
[[ -f "$SCENARIO_FILE" ]] || { echo "scenario not found: $SCENARIO_FILE" >&2; exit 2; }
[[ -f "$SNAPSHOT_FILE" ]] || { echo "snapshot not found: $SNAPSHOT_FILE" >&2; exit 2; }
echo "scenario: $SCENARIO_FILE"
echo "10 UAV PX4 SITL, $FAULTS logical exits; no runtime or TCP server"
echo 'Requires ROS Noetic, built workspace, PX4 Gazebo, and PyTorch DAGL dependencies.'
if { "$CHECK" && "$EXECUTE"; } || { "$CHECK" && "$ALGORITHM_ONLY"; } || {
  "$EXECUTE" && "$ALGORITHM_ONLY";
}; then
  echo 'Choose exactly one of --check, --check-algorithm, --execute' >&2
  exit 2
fi

resolve_worker() {
  local candidate message
  local -a candidates=()
  if [[ -n "$WORKER_PYTHON" ]]; then
    candidates=("$WORKER_PYTHON")
  else
    candidates=(python3 "$HOME/.conda/envs/gcbfplus/bin/python3")
  fi
  for candidate in "${candidates[@]}"; do
    if ! command -v "$candidate" >/dev/null 2>&1; then
      echo "$candidate: executable not found (replace example paths with an actual Python)" >&2
      continue
    fi
    if message="$("$candidate" -c 'import importlib.util, sys; modules=("torch","numpy","pandas","openpyxl","einops"); missing=[name for name in modules if importlib.util.find_spec(name) is None]; print("missing: " + ", ".join(missing) if missing else "ok"); sys.exit(bool(missing))' 2>&1)"; then
      if message="$("$candidate" -c 'import torch, numpy, pandas, openpyxl, einops; print("ready:", torch.__version__)' 2>&1)"; then
        WORKER_PYTHON="$(command -v "$candidate")"
        echo "DAGL worker: $WORKER_PYTHON ($message)"
        return 0
      fi
    fi
    echo "$candidate: $message" >&2
  done
  echo 'No suitable Python: install torch, numpy, pandas, openpyxl, einops in ONE Python environment.' >&2
  echo 'Example: the existing ~/.conda/envs/gcbfplus/bin/python3 has pandas/einops but still needs torch/openpyxl.' >&2
  return 1
}

if ! "$EXECUTE" && ! "$ALGORITHM_ONLY"; then
  if resolve_worker; then
    echo 'Dependencies ready; no simulator started (no --execute).'
    echo "After training: bash verification/dag_recovery/run_sim10.sh --faults $FAULTS --checkpoint /absolute/path/model.pt --outdir /tmp/dag10_sitl_1 --execute"
    exit 0
  fi
  echo 'No simulator started. Install missing Python packages, then rerun with --check.' >&2
  "$CHECK" && exit 1
  exit 0
fi
[[ -n "$OUTDIR" && ! -e "$OUTDIR" ]] || { echo 'Choose a NEW --outdir; existing data is never overwritten' >&2; exit 2; }
[[ -n "$CHECKPOINT" && -f "$CHECKPOINT" && -f "$(dirname "$CHECKPOINT")/model.json" ]] || {
  echo 'Provide --checkpoint with a validated DAGL-10 model.pt and model.json' >&2; exit 2;
}
CHECKPOINT="$(realpath "$CHECKPOINT")"
if ! "$ALGORITHM_ONLY"; then
  [[ -f "$ROOT/devel/setup.bash" && -f /opt/ros/noetic/setup.bash ]] || { echo 'Build catkin workspace first' >&2; exit 2; }
  PX4_ROOT="${PX4_ROOT:-/home/yjq/PX4_Firmware}"
  [[ -f "$PX4_ROOT/Tools/setup_gazebo.bash" ]] || { echo 'PX4 Gazebo installation missing' >&2; exit 2; }
  for command in roslaunch rosnode rosparam pgrep timeout; do
    command -v "$command" >/dev/null || { echo "Missing $command" >&2; exit 2; }
  done
  [[ "$(pgrep -xc px4 || true)" == 0 && "$(pgrep -xc gzserver || true)" == 0 ]] || {
    echo 'Other PX4/Gazebo processes exist; refusing to interfere' >&2; exit 2;
  }
fi
resolve_worker || exit 2

mkdir -p "$OUTDIR/logs"
echo "Checking actual ML-DAGL targets offline before launching PX4 (faults=$FAULTS)..."
if ! python3 "$DIRECTORY/sim_multi.py" --faults "$FAULTS" \
  --scenario "$SCENARIO_FILE" --snapshot "$SNAPSHOT_FILE" --worker-python "$WORKER_PYTHON" \
  --checkpoint "$CHECKPOINT" \
  --outdir "$OUTDIR/algorithm_preflight" > "$OUTDIR/logs/algorithm_preflight.log" 2>&1; then
  echo "DAGL preflight failed; PX4 was not started. See $OUTDIR/logs/algorithm_preflight.log" >&2
  echo "Worker details: $OUTDIR/algorithm_preflight/worker.log" >&2
  exit 1
fi
if "$ALGORITHM_ONLY"; then
  echo "Offline DAGL passed; no simulator started. Record: $OUTDIR/algorithm_preflight"
  exit 0
fi
echo 'DAGL preflight passed; now starting PX4/Gazebo (QGC can observe the simulation).'

set +u
source /opt/ros/noetic/setup.bash
source "$ROOT/devel/setup.bash"
source "$PX4_ROOT/Tools/setup_gazebo.bash" "$PX4_ROOT" "$PX4_ROOT/build/px4_sitl_default"
set -u
export ROS_HOME="$OUTDIR/ros_home"
export ROS_LOG_DIR="$ROS_HOME/log"
export ROS_HOSTNAME=localhost
export LIBGL_ALWAYS_SOFTWARE=1
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$PX4_ROOT:$PX4_ROOT/Tools/sitl_gazebo"
export GAZEBO_MASTER_URI=http://localhost:11345
mkdir -p "$ROS_LOG_DIR"
python3 "$DIRECTORY/sim_setup.py" --scenario "$SCENARIO_FILE" --outdir "$OUTDIR/config"

declare -a LAUNCH_PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${LAUNCH_PIDS[@]}"; do
    kill -- "-$pid" 2>/dev/null || true
  done
  for pid in "${LAUNCH_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

start_launch() {
  local master="$1" log="$2"
  shift 2
  ROS_MASTER_URI="http://localhost:$master" setsid roslaunch "$@" > "$OUTDIR/logs/$log" 2>&1 &
  LAUNCH_PIDS+=("$!")
}

wait_for() {
  local name="$1" deadline="$2"
  shift 2
  for ((attempt=0; attempt<deadline; attempt++)); do
    if "$@"; then return 0; fi
    sleep 3
  done
  echo "Timed out waiting for $name; logs: $OUTDIR/logs" >&2
  return 1
}

sim_ready() {
  local count
  count="$(pgrep -xc px4 || true)"
  [[ "$count" -ge 10 ]] && ROS_MASTER_URI=http://localhost:11300 timeout 4 rosnode list 2>/dev/null | grep -q /gazebo
}

onboard_ready() {
  local index="$1"
  ROS_MASTER_URI="http://localhost:$((11310 + index))" timeout 4 rosnode list 2>/dev/null |
    grep -q "/uav_executor_UAV$index"
}

gcs_ready() {
  [[ "$(ROS_MASTER_URI=http://localhost:11310 timeout 4 rosparam get /group_a/calibration_ready 2>/dev/null || true)" == true ]]
}

start_launch 11300 sim.log "$OUTDIR/config/spawn.launch" gui:=false
wait_for '10 PX4 instances and Gazebo' 120 sim_ready

for ((index=1; index<=10; index++)); do
  intents=""
  odometry=""
  for ((other_index=1; other_index<=10; other_index++)); do
    if ((other_index != index)); then
      intents+="/UAV${other_index}/trajectory_intent,"
      odometry+="/UAV${other_index}/mavros/local_position/odom,"
    fi
  done
  start_launch "$((11310 + index))" "onboard_$index.log" "$DIRECTORY/sim_onboard.launch" \
    "uav_name:=UAV$index" "tgt_system:=$index" \
    "topology_file:=$OUTDIR/config/topology.yaml" \
    "neighbor_intents:=${intents%,}" "neighbor_odom_topics:=${odometry%,}"
done
for ((index=1; index<=10; index++)); do
  wait_for "UAV$index executor" 50 onboard_ready "$index"
done

start_launch 11310 gcs.log "$DIRECTORY/gcs.launch" \
  "identity_file:=$ROOT/src/tcp_to_ros/config/uav_identity_map.yaml" \
  "topology_file:=$OUTDIR/config/topology.yaml" \
  "calibration_file:=$OUTDIR/config/calibration.yaml" \
  "formation_file:=$OUTDIR/config/formation.yaml"
wait_for 'GCS calibration' 80 gcs_ready

ROS_MASTER_URI=http://localhost:11310 python3 "$ROOT/src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py" \
  1 2 3 4 5 6 7 8 9 10 > "$OUTDIR/logs/takeoff.log" 2>&1 || {
    echo 'SITL soft takeoff failed; inspect takeoff.log' >&2; exit 1;
  }

ROS_MASTER_URI=http://localhost:11310 python3 "$DIRECTORY/sim_multi.py" \
  --sitl --faults "$FAULTS" --scenario "$SCENARIO_FILE" --outdir "$OUTDIR/experiment" \
  --identity "$ROOT/src/tcp_to_ros/config/uav_identity_map.yaml" \
  --deployment "$OUTDIR/config/deployment.json" --worker-python "$WORKER_PYTHON" \
  --checkpoint "$CHECKPOINT" \
  2>&1 | tee "$OUTDIR/logs/experiment.log"
