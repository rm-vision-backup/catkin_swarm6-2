# 四机实机测试：准备编队 + 侦查（A06 / A08 / A10 / A12）

本目录是 `src/swarm_uav_interfaces/real_setup.md` §7.1 已批准的**四机工程测试**落地件：
A06/A08/A10/A12（执行层 UAV6/UAV8/UAV10/UAV12）按 **5 m 间距**只跑两个阶段 ——
**准备编队（MOVE_TO）+ 侦查（FOLLOW_ROUTE）**，与 12/15 机联合验收完全分离。

| 文件 | 作用 |
|---|---|
| `driver.py` | 任务下发脚本（实机用）：两阶段串行发 TCP 命令；整批超时/失败/Ctrl-C 自动发 HOLD；`--dry-run` 只打印线上 JSON；`--log` 写 JSONL 证据 |
| `package/` | 四机任务数据（robots / bootstrap / world / plans）；`prepare-a` + `coverage-segment-1`；leader = A06 |
| `sitl/` | 四机**仿真**彩排（`four_uav_spawn.launch` / `gcs_sim_four_uav.launch` / `run_sim_four_uav.sh`） |

地面站侧：`src/tcp_to_ros/launch/gcs_a_real_four_uav.launch`
（加载 `config/formation_offsets_four_uav.yaml` + `config/calibration_four_uav.yaml`；
身份映射 / topology / 端口 / **安全门禁**全部沿用共享配置，未做任何修改）。

---

## 一、现场摆位（决定 prepare 阶段飞多远）

标定冻结的原点 = 四机全球位置中位数 = **阵位几何中心**，所以 `prepare-a` 的目标就是
"以该中心为原点、按机头方向对齐的 5 m×5 m 方阵四角"。

1. 四机摆成 **5 m × 5 m 正方形**（相邻 5 m、对角 7.07 m）。
2. **四机机头大致同向**（航向差 ≤ 30°，否则标定不出 ready）。
3. 按机头方向定义前/右：**A06 左后角**、**A08 右后角（A06 正右 5 m）**、
   **A10 左前角（A06 正前 5 m）**、**A12 右前角（对角）**。
4. 摆好后不再挪动 → 标定 → 再 arm。

摆得不够方只会让 prepare 多飞几米（最多错峰约 10 s，远小于 150 s 阶段超时），不致命。

## 二、机载启动（每台机上各跑一条；参数必须逐台不同）

`neighbor_intents` **没有默认回退**（不传则规划层拿不到任何邻机意图，只剩执行层 1 m
门禁）；`uav_task_timeout_s` 不传就是 200 s，而侦查航线约 84 m（预计 100–140 s），
会在航程未走完时判 `DRIVER_TIMEOUT`。

```bash
# UAV6 / A06（领机）  邻居 = 8 10 12
ssh <user>@192.168.5.61 'bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV10/trajectory_intent,/UAV12/trajectory_intent \
  neighbor_odom_topics:=/UAV8/mavros/local_position/odom,/UAV10/mavros/local_position/odom,/UAV12/mavros/local_position/odom \
  uav_task_timeout_s:=280'

# UAV8 / A08   邻居 = 6 10 12   → 192.168.5.81   （参数同上，去掉自身、补上自己以外的三架）
# UAV10 / A10  邻居 = 6 8 12    → 192.168.5.101
# UAV12 / A12  邻居 = 6 8 10    → 192.168.5.121
```

- ssh 用户取 `src/tcp_to_ros/tools/` 共用参数文件里的 `BOARD_SSH_USER`（默认 ubuntu）。
- **不要**用 `tools/onboard_ctl.sh start 6 8 10 12 -- <参数>`：它把同一串参数发给所有目标，
  而邻居清单每台不同。
- `leader_odom_topic_prefix` / `gps_bias` / `uav_name` 都不用传（已有默认）。

## 三、地面站启动（GCS_A = 192.168.5.241，hostname 须为 GCS_A）

```bash
roslaunch src/tcp_to_ros/launch/gcs_a_real_four_uav.launch
```

## 四、跑一轮

1. 确认标定就绪（四机必须**全部 disarm**、pose 连续 2 s、航向差 ≤30°）：

```bash
rostopic echo -n1 /group_a/calibration_ready     # 期望 data: True
```

2. 遥控逐机 **arm + OFFBOARD**，确认 `/mavros/state` = `armed: True` / `mode: OFFBOARD`。
3. 下发任务（两阶段：准备编队 → 侦查）：

```bash
python3 verification/four_uav_recon/driver.py \
  --package verification/four_uav_recon/package \
  --tcp-host 192.168.5.241 --tcp-port 39001 \
  --mission-id four-uav-recon-001 \
  --log verification/four_uav_recon/logs/real_$(date +%Y%m%d_%H%M%S).jsonl
```

## 五、观测点与成功判据

- **成功判据**：driver 退出码 0；两条 `[prepare] SUCCESS` / `[coverage] SUCCESS`；
  GCS 侧 4/4 `COMPLETED`；全程 `MIN_DISTANCE_BREACH` 零次；跟机与领机保持 ≈5 m。
- 观测：driver 的 JSONL（`mission_start` / `phase_result` / `command_feedback`）；
  GCS `gcs_a_backend_node` 的 rosout（`GCS_A goal ...`、`GCS_A schedule ... tau {...}`）；
  `/UAVn/uav_task_state`、`/group_a/group_task_state`；
  机载 `~/.tmp/logs/UAVn_offboard_ego.log`（关注 `MIN_DISTANCE_BREACH`、`BRAKE_HOLD`、
  `EGO_PLAN_FAILED`）；整轮取证用 `src/tcp_to_ros/tools/log_fetch.sh <UAVn>`。

## 六、仿真彩排（改完配置先跑这个）

```bash
bash verification/four_uav_recon/sitl/run_sim_four_uav.sh
```

自动完成：4 机 SITL → 机载（6 8 10 12，自动补邻机话题 + `EGO_TASK_TIMEOUT_S=280`）→
四机 GCS + 标定 → takeoff → `driver.py`（`--tcp-host 127.0.0.1`）。
日志在 `.ros_home/log/headless_test/`（`04_gcs.log`、`06_takeoff.log`、`07_task.jsonl`），
退出码 0 = 两阶段 SUCCESS。

> 注意：仿真彩排由 `startup_offboard_ego.sh` **自动**填邻机话题，验证不到 §二 的手工参数；
> §二 的正确性只能靠实机首跑或逐机 `rosnode info` 复核。
