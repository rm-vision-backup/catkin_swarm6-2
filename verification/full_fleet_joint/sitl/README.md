# full_fleet_joint SITL 录制与回放

本目录提供：出生布局生成、**一键 PX4/Gazebo 录制**、仿真 GCS wrapper、以及从实测
`telemetry.jsonl` 渲染三维视频。只管理本轮启动的进程；不连接实机、不调用全局 cleanup。

## 1. 生成任务包

```bash
bash verification/full_fleet_joint/flow.sh generate --available 1-16
```

15/14 架备份任务包（B 自动从备用列底部补足 4 架）：

```bash
python3 verification/full_fleet_joint/generate_mission.py --available 1-15 \
  --out .tmp/full_fleet_joint/packages/fleet15
python3 verification/full_fleet_joint/generate_mission.py --available 1-14 \
  --out .tmp/full_fleet_joint/packages/fleet14
```

非默认任务包在录制时用 `--spec` 指定；例如：

```bash
bash verification/full_fleet_joint/sitl/run_recording.sh \
  --spec .tmp/full_fleet_joint/packages/fleet15/mission_spec.json \
  --outdir .tmp/full_fleet_joint/<新仿真轮次>
```

## 2. 一键录制（推荐）

先确认 `PX4_ROOT`（默认 `$HOME/PX4_Firmware`）和 11300/11310/11345/39061/各机 Master 未被占用。
`--outdir` 必须是新目录；脚本拒绝覆盖。

```bash
bash verification/full_fleet_joint/sitl/run_recording.sh \
  --outdir .tmp/full_fleet_joint/<新仿真轮次>
```

只打印计划、不启动 Gazebo/PX4：

```bash
bash verification/full_fleet_joint/sitl/run_recording.sh \
  --outdir .tmp/full_fleet_joint/<新仿真轮次>_dry --dry-run
```

脚本按本轮 `mission_spec.json`：

- 用 `sitl/generate_spawn.py` 生成 16/实际可用架的出生 launch；
- GCS 的 `formation_offsets_file` / `calibration_config_file` 自动指向 `--spec` 同目录的 `config/`，15/14 备份包不会误读默认 `generated/config`；
- 逐机起 `uav_offboard_ego.launch`，B 组带 `layer_move_to:=8.0`；
- 起 `sitl/gcs_sim.launch`（TCP 39061，避免与实机 39001 冲突）；
- 等待标定与逐机 `gp_origin_confirmed`，仿真起飞到 5m；
- 跑完整 driver，记录 `telemetry.jsonl`、`driver.jsonl`、`summary.json`；
- 结束时只收口本轮启动的进程。

## 3. 生成视频

```bash
python3 verification/full_fleet_joint/sitl/render_video.py \
  .tmp/full_fleet_joint/<新仿真轮次> \
  --out .tmp/full_fleet_joint/<新仿真轮次>/full_mission_3d.mp4 --speed 4
```

输出：MP4、首帧/末帧 PNG、同名 `.json`（来源、状态、时长、帧数、最小实测三维间距、
阶段结果、本轮可用机/领机/牺牲列/B 组）。**视频只用本轮 ROS 实测位置重建**；
`summary.json` 不是 SUCCESS 时不会生成“成功视频”。

## 4. 手动分阶段启动（排查用）

```bash
# 终端 1：Gazebo/PX4
export ROS_MASTER_URI=http://localhost:11300 ROS_HOSTNAME=localhost
export GAZEBO_MASTER_URI=http://localhost:11345
roslaunch .tmp/full_fleet_joint/<轮次>/spawn.launch gui:=false

# 终端 2：机载执行层（每机一个 Master 11311–11326）
EGO_TASK_TIMEOUT_S=300 bash src/swarm_uav_executor/scripts/startup_offboard_ego.sh 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16

# 终端 3：GCS
export ROS_MASTER_URI=http://localhost:11310 ROS_HOSTNAME=localhost
roslaunch verification/full_fleet_joint/sitl/gcs_sim.launch

# 终端 4：仿真起飞 + driver
python3 src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
python3 verification/full_fleet_joint/driver.py --execute \
  --tcp-host 127.0.0.1 --tcp-port 39061 \
  --log .tmp/full_fleet_joint/<轮次>/driver.jsonl
```

手动模式 B 组的 `layer_move_to:=8.0` 需要在该机 launch 参数中显式传入；一键脚本已处理。
`offboard_takeoff_15_sitl.py` 接受任意 UAV 编号列表，默认 1–16。
