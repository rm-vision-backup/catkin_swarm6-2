# 2026-10-19 六机联合任务实机轮

状态：进行中（步骤 1 已通过；等待授权启动机载栈）

## 本轮固定信息

- 任务入口：`verification/six_uav_joint/`；GCS launch `gcs_real.launch`
- 机队：UAV6–11（任务身份 A06–A11）；领机 A07；UAV6 预设故障、UAV11 备用、A09/A10 模拟 B 组
- GCS：`/home/yjq/catkin_swarm6-2`，`ROS_IP=192.168.5.241`，`ROS_MASTER_URI=http://localhost:11311`
- GCS 系统时间：2026-10-19 10:17:45 +08:00（时区/日期正确）
- 通信测试：未选择（计划热测；机载栈稳定后、起 GCS 前执行，`flow.sh stage1 20` 二选一）
- 机载部署指纹：待步骤 5 用 rsync checksum 干跑核对（本轮未改动机载包）

## 代码基线（本轮开始前）

| 仓库 | HEAD | 未提交改动 |
|---|---|---|
| 主仓 `master` | `4a5f290` | `verification/six_uav_joint/{README,driver,flight_guard,test_joint,sitl/record_run}.py/md`、`real_uav_workflow/README.md`、`dag_recovery/*`、`claude.md` |
| `src/tcp_to_ros` | `1e316d5` | `config/backend_defaults.yaml`、`scripts/gcs_a_backend_node.py`（本轮改：`schedule_pose_timeout_s` 1→2 s） |
| `src/swarm_topology_bridge` | `73db782` | 多个文件未提交（清单见 `git -C src/swarm_topology_bridge status`） |
| `src/swarm_uav_executor` | `ccc5afe` | `launch/uav_offboard_ego_real.launch` |
| `references/runtime` | `dcf0e03` | 无 |

本轮 GCS 侧改动（未提交）：

- `verification/six_uav_joint/flight_guard.py`：运行期不再每轮查 `calibration_ready`/`reference_heading`（改为启动一次）；阶段终点 `swarm_pose` 年龄 1→3 s；
- `src/tcp_to_ros`：`schedule_pose_timeout_s` 1→2 s（YAML + 代码默认值）。

## 步骤状态

| 步骤 | 结果 | 说明 |
|---|---|---|
| 0 固定清单 / RUN_DIR | PASS | 本目录 |
| 1 只读检查（身份/router/板钟/残留） | **PASS** | UAV7/9 板钟 10:21 修正（rtc0 3 s、rtc1 1 s，PASS）；UAV11 10:19 上线并复查通过 |
| 2 session 建立 | PASS | 本文件 |
| 3 机载栈启动 | **PASS** | 10:23:16–10:23:38 逐台启动，六机 `STARTED=1`；日志 `onboard_start.log` |
| 4 热测（`stage1 20`） | **PASS** | 修正 UAV11 `MAV_2_MODE=2` 并重启飞控、六机并行重启后，10:41 热测六机全绿 |
| 5 GCS 起栈/标定 | **PASS** | GCS 10:42 起栈；配置/端口/bundle 核对通过；`calibration_ready=True`；六机原点确认 True |
| 6 起飞与空中快查 | 未开始 | |
| 7 driver 任务 | 未开始 | |
| 8 收尾/日志采集 | 未开始 | |

## 步骤 1 逐机结果

| UAV | ping | hostname | 栈进程 | router | 板钟 | 残留/端口 |
|---|---|---|---|---|---|---|
| UAV6 | UP | UAV6 | 0（DOWN） | enabled/active，单实例 PID 1022，参数仅 `127.0.0.1:15500`，NRestarts=0 | 正常 | 无；4200/14000/15500/11311 未监听 |
| UAV7 | UP | UAV7 | 0（DOWN） | enabled/active，单实例 PID 1021，参数同上，NRestarts=0 | 1970 → **10:21 已修正**（PASS，rtc0 +3 s / rtc1 +1 s） | 无；端口未监听 |
| UAV8 | UP | UAV8 | 0（DOWN） | enabled/active，单实例 PID 1040，参数同上，NRestarts=0 | 正常 | 无；端口未监听 |
| UAV9 | UP | UAV9 | 0（DOWN） | enabled/active，单实例 PID 1024，参数同上，NRestarts=0 | 1970 → **10:21 已修正**（PASS，rtc0 +3 s / rtc1 +1 s） | 无；端口未监听 |
| UAV10 | UP | UAV10 | 0（DOWN） | enabled/active，单实例 PID 1020，参数同上，NRestarts=0 | 正常 | 无；端口未监听 |
| UAV11 | UP（10:19 上线） | UAV11 | 0（DOWN） | enabled/active，单实例 PID 1011，参数同上，NRestarts=0 | 正常（10:21:37，uptime 105 s） | 无；端口未监听 |

说明：

- 六机机载栈均未运行（`PROCS=0`），符合上电后初始状态；未发现 `roslaunch`/`bridge_node`/`mavros`/`ego_planner`/`setpoint_relay`/`gp_origin`/`formation_follow`/`offb_hover` 残留。
- router 日志中的 `Could not open conf file`、`write serial port latency ... Invalid argument` 在全部机上一致出现，属既有已知告警；实际已打开 `/dev/ttyTHS0:921600` 与 `127.0.0.1:15500`。
- 端口 4200/14000/15500/11311 均未监听（栈停 + 遥测转发已移除），符合预期。

## 热测记录（步骤 3/4）

- 机载栈启动：UAV6–11 逐台 `onboard_ctl.sh start`，PROCS=1、STARTED=1、PASS；证据 `onboard_start.log`。
- 10:24 `stage1 20`：六机 remote `gt1000` +9…+19、`gap_max` 1.1–1.3 s；**UAV11 本机 tx 1.0/s、gt1000 +16**；其余五机本机 tx 10.0/s、gt1000=0。判 FAIL。证据 `stage1_20.log`。
- 10:30 重测：六机 SSH 全部超时，ARP 全 INCOMPLETE，GCS `enp5s0` 载波 UP 但 5 s 仅 RX +1 包；判无效。约 20 s 后恢复。
- 10:32 重测：UAV11 本机 tx 仍 1.0/s、gt1000 +17；其余五机 remote `gt1000` +11…+23，`gap_max` 累计 2.7–4.6 s（含 10:30 中断）。判 FAIL。证据 `stage1_20_retry2.log`。
- UAV11 本机实测（与 UAV8 对照）：`/mavros/local_position/pose` 1.0 Hz（UAV8 30 Hz）、`/mavros/global_position/global` 5 Hz（50 Hz）、`/mavros/attitude` 10 Hz（50 Hz）；MAVROS connected=True、armed=False、POSCTL。
- 飞控参数对照：UAV11 `MAV_2_CONFIG=102`（TELEM2）、`MAV_2_MODE=0`（Normal）；UAV8 伴飞链路对应实例为 `MAV_1_CONFIG=102`、`MAV_1_MODE=2`（Onboard）。两者 `MAV_0_*` 相同（101/Normal/1200）。
- 判断：UAV11 伴飞链路所在 MAVLink 实例仍是 Normal 模式，导致位姿流 1 Hz。**修改飞控参数需现场明确授权**，本轮尚未改动。

## 热测通过记录（10:41）

- UAV11 `MAV_2_MODE` 由 0 改为 2（Onboard），参数回读确认；通过 MAVLink 命令 246 重启飞控，MAVROS 重连后 `/mavros/local_position/pose` 恢复 30 Hz。
- 六机并行 stop/start 全部 PASS（`stop_*.log`、`start_*.log`）。
- 10:41:30 `stage1 20`：六机本机 `swarm_pose` 10.0/s、邻机源 5/5、`gt1000=0`、`stalls=0`、窗口内无 stall/link 事件；**PASS**。证据 `stage1_20_after_parallel_restart.log`。

## GCS 与标定记录（10:42）

- `gcs_real.launch` 起栈；TCP 39001 UP；`packing=bundle`。
- 实际加载：`formation_offsets_file=verification/six_uav_joint/config/formation_offsets.yaml`；`calibration_uavs=UAV6–10`；`all_uavs=UAV6–11`；`sample_window_s=8.0`；`schedule_pose_timeout_s=2.0`（本轮修改生效）。
- `/group_a/calibration_ready=True`；`reference_heading=-1.2679183899894295 rad`。
- 六机 `/gp_origin_confirmed=True`（含 UAV11）；`mavros/state` 均 connected=True、armed=False、POSCTL；`/mavros/setpoint_raw/local` 唯一发布者 `/setpoint_relay`。
- 下一步：飞手六机 arm/OFFBOARD 并稳定约 5 m → 起飞后 quick odom（可选）→ GCS driver。**目前尚未授权起飞/下发任务。**

## bag 录制与 GCS 重启（10:44）

- `flow.sh bag 1200`：10:44:44–10:44:57 六机启动，命令返回 PASS。
- 10:47 核对：UAV6/8/9/10/11 的 `.active` 文件已在增长（约 2.7–3.0 MB）；UAV7 启动命令已接受，但当时 SSH 超时未能回读。
- 上个会话结束时 GCS 后台进程被终止（ROS master 不在、39001 DOWN）。**GCS 改由现场手动启动**；重启后需重新确认 `calibration_ready` 与六机原点确认。
- bag 时长 1200 s（自 10:44 起，约 11:04 结束）；若起飞晚于该时间需重新启动录制。

## 取消 bag 与两机失联（10:50–10:54）

- 10:52 取消录制：UAV8/9/10/11 已 SIGINT 正常收尾，`.active` 消失并生成 `.bag`（约 8.9–9.4 MB）。
- **UAV7 机载电脑 10:49:51 自行重启**（当前 uptime 3 min），bag 进程随之消失，留下 `UAV7_20261019_104446_air.bag.active`（5.3 MB，未收尾）；机载栈 `uav_offboard_ego_real.launch` 未运行。上一 boot 的 journal 查不到错误/电源记录。
- **UAV6 自 10:52 起 ping 不通**，SSH 超时，无法停止其 bag。若进程仍在，将在 1200 s（约 11:04）自动收尾；若也重启，则 bag 同样是 `.active`。
- 结论：本轮无法继续飞行。需现场检查 UAV6/UAV7 供电与机载电脑状态；恢复后必须重启其机载栈，并重新做热测/标定。

## 11:00 状态复查（机载程序）

| UAV | boot | 机载栈 | bag |
|---|---|---|---|
| UAV6 | 09:57:34 | UP(2) | 已停并收尾，16.7 MB（10:59） |
| UAV7 | 10:49:51（重启） | **DOWN** | 未收尾 `.active` 5.3 MB |
| UAV8 | 09:57:52 | UP(2) | 已收尾 |
| UAV9 | 10:52 时仍在（栈/bag 正常） | **当前 ping 不通，无法复查** | 已收尾 |
| UAV10 | 09:58:12 | UP(2) | 已收尾 |
| UAV11 | 10:19:52 | UP(2) | 已收尾 |

- 10:52–11:00 之间出现 UAV6 失联后恢复、UAV7 重启、UAV9 失联，节点掉线呈随机、逐架发生。
- 在本轮故障排除前不应起飞；需现场确认是否存在电池/电源接头或人为断电操作。

## 阻塞项处理与下一步

1. **UAV11 已上线**（10:19 上电，uptime 105 s）：hostname/IP/router/板钟/残留全部复查通过，栈为停止状态。
2. **UAV7/UAV9 板钟已修**（10:21，`onboard_clock_fix.sh 7 9`）：系统钟与 GCS 偏差 0 s；两路 RTC 均写入并回读，rtc0 +3 s、rtc1 +1 s，工具判定 PASS。依据 `JETSON-RTC-TIME-RUNBOOK.md`，写入正确时间即打断“1970 写回 RTC”的闭环；未做 udev/ntp 整理性改动。
3. 下一步：**等待授权启动六机机载栈** → 机载稳定后做热测（`flow.sh stage1 20`）→ 起 GCS、标定。
