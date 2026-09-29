# 2026-09-23 四机编队侦查飞行 · 数据包（four-uav-recon-001）

- **机队**：A08（领机）/ A10 / A12 / A13 = **UAV8 / UAV10 / UAV12 / UAV13**（四机版首飞）
- **任务**：`four-uav-recon-001`，两阶段（prepare = MOVE_TO，coverage = FOLLOW_ROUTE）
- **运行时刻**：2026-09-23 **17:03:37** 起下发；driver 日志 `real_20260923_170337.jsonl`
- **结果**：**prepare 4/4 SUCCESS / coverage FAILURE**（A08、A12 SUCCESS；**A10、A13 FAILURE = `LEADER_LOST`**）
- **规程**：`verification/four_uav_recon/README.md` §四

## 一、结论速览（仅 driver 事件流，ulog 未分析）

| 阶段 | A08 | A10 | A12 | A13 |
|---|---|---|---|---|
| prepare（MOVE_TO） | SUCCESS | SUCCESS | SUCCESS | SUCCESS |
| coverage（FOLLOW_ROUTE） | **SUCCESS** | **FAILURE / LEADER_LOST** | **SUCCESS** | **FAILURE / LEADER_LOST** |

- 失败码 `LEADER_LOST` = 跟随者判领机 odom 陈旧（>1.0 s，`ego_swarm.py:65` `_FOLLOW_POSE_TIMEOUT_S`）。
- **注意与 09-22 的差异**：09-22 是 A13 报 LEADER_LOST 而 **A10 被误判 SUCCESS**；本轮 **A10 也报了 FAILURE**，
  且 A12 是 SUCCESS。**A10 这次判对了吗？A12 判对了吗？** 这需要用 ulog 核实际飞行（见 §四）。

## 二、目录结构与命名

```
2026-09-23_1703_four-uav-recon-001/
├── README.md                  ← 本文件
├── gcs/                       ← 地面站（本机）侧
│   ├── driver_four-uav-recon-001_20260923_170337.jsonl   ★ 逐事件记录（最快定位失败）
│   ├── GCS_a_gcs_a_backend_node.log    分组调度后端
│   ├── GCS_a_gcs_a_calibration.log     标定节点（冻结原点与航向）
│   ├── GCS_a_swarm_bridge.log          GCS 侧桥接
│   ├── GCS_a_tcp_server_node.log       下发通道
│   ├── rosout_all_nodes.log            全节点 rosout 合集
│   ├── ros_logs/                       原始 session 目录原样副本
│   └── prev_session_path.txt           上一轮 GCS session 路径（标定冻结那次的）
├── onboard/UAVn_2026-09-23_1707/
│   ├── fcu/ulog_list.txt       飞控日志清单（★ 本轮架次 = 表里最新那条，见 §五）
│   └── onboard/{ros_home, ego_planner, term}/   各节点日志 / planner 诊断 / roslaunch 转录
└── config_snapshot/            本轮生效的拓扑、标定集合、编队表、任务包 + 各子模块 commit
```

## 三、分析入口（按优先级）

1. **`onboard/UAVn_*/onboard/term/UAVn_offboard_ego.log`** —— 机载栈终端转录（含 **`mavros` 与
   `ego_planner_driver` 的输出** —— 这两个节点在本 session 里**没有独立的 rosout 日志文件**，
   只有终端转录，别以为采集漏了）。关注 `MIN_DISTANCE_BREACH` / `BRAKE_HOLD` / `EGO_PLAN_FAILED`。
   **注意时间覆盖范围**：机载节点日志常在任务中途静默，**不能拿尾部当事件终点**（09-22 踩过这个坑）。
2. **`gcs/driver_*.jsonl`** —— 事件流与失败码（`command_feedback.error_code`）。
3. **飞控 ulog（你自己下载，见 §五）** —— **权威时间基**。读法：`pyulog.ULog`，看
   `vehicle_local_position` / `vehicle_local_position_setpoint` / `vehicle_attitude` / `vehicle_status`；
   也可上传 FlightReview。注意四台 x/y 是**各自的 EKF 局部系**。
4. **`onboard/UAVn_*/onboard/ego_planner/`** —— planner 诊断（安全监督层快照）。
5. **`gcs/GCS_a_swarm_bridge.log`** + 各机 `ros_home/<session>/swarm_bridge-*.log` —— 桥接注册/连接情况。

## 四、待查（分析主线）

1. **A10/A13 的 `LEADER_LOST` 是真丢链还是判据问题？** 用 ulog 对时间轴：跟随者在报错时刻
   是否真的停止了跟随（setpoint 冻结）？领机当时在飞吗？
2. **A12 为什么 SUCCESS 而 A10 FAILURE？** 两者同为跟随者、同 offset 类型（A12 是 `(0,+5)` 正前，
   A10 是 `(-5,+5)` 左前 / A13 是 `(-5,0)` 正左）。**位置/朝向差异是否导致了不同的跟踪表现**？
3. **A08（领机）**：航线是否飞完？与 09-22 对比耗时。
4. **本轮是否再次出现"跟随者根本没跟随"**（09-22 的实际形态）——即 ulog 里跟随者水平位移远小于领机。
5. **机间桥接在任务态是否出现 1 s 级空档**（09-22 的根因）。本轮起飞前测过（120 s，双向零 ≥0.5 s 空档），
   **但任务态流量更大**（多了不限频的 `trajectory_intent`），需要从日志/ulog 里核。

## 五、飞控 ulog（**需你从 QGC 下载后归档到本目录 `onboard/UAVn_*/fcu/`**）

`fcu/ulog_list.txt` 是本轮各机飞控日志清单，**本轮架次 = 表里最新一条**：

| 机 | 日志 id | 大小 | 收尾时刻(utc+8) |
|---|---|---|---|
| UAV8 | **log_54** | 4,416,145 | 2026-09-23 17:05:12 |
| UAV10 | **log_38** | 3,914,035 | 2026-09-23 17:04:46 |
| UAV12 | **log_38** | 3,861,376 | 2026-09-23 17:04:48 |
| UAV13 | **log_57** | 3,879,668 | 2026-09-23 17:04:48 |

（下载方式：Type-C 连飞控 → QGC → 找对应 id → 存成 `log_NN_2026-9-23-HH-MM-SS.ulg` 放进各自 `fcu/`。）

## 六、采集状态

| 项 | 状态 |
|---|---|
| GCS 侧（driver JSONL / backend / calibration / bridge / tcp / rosout / session 副本） | **已完成** ✓ |
| 四台机载 ROS 日志 + planner 诊断 + 终端转录 | **已完成** ✓（rsync 快取） |
| 配置与版本快照（含各子模块 commit、板侧 topology md5） | **已完成** ✓ |
| 飞控 ulog ×4 | ⏳ **待你从 QGC 下载** |
| **rosbag** | ❌ **本轮仍未录**（09-22 报告 §4.3 已列为"下轮必做"，仍缺） |

## 七、本轮两个背景事实（分析时要带上）

1. **四机版首飞**：机队刚从 3 机（A08/A10/A13）恢复为 **A08/A10/A12/A13**，几何为 2×2、5 m 方阵
   （A13 占原 A06 角位）。摆位实测间距偏差最大 ~1 m、航向展度 26.3°（门槛 30°）。
2. **起飞前 UAV8 / UAV12 出现过"机载→飞控不通"**：判据是串口排空 0 KB/s、全程 `EAGAIN`、
   `param/get` 为 `False`、`gp_origin_confirmed` 为 `False`；**重启 `mavlink-routerd` 即修复**
   （排空恢复到 91 KB/s、`param/get` 变 True）。**这不是排线问题**（与 UAV11 那台不同）。
   修复后才做的标定与飞行。→ **分析时若要引用"飞机当时状态"，记得这段前置。**

## 八、备注

- 本目录是**分析数据**，不要提交进 git（建议 `src/uav_log/` 进 `.gitignore`）。
- 标定（本轮）：origin `(28.2198645, 112.86686415, 45.968)`、reference heading **−1.510 rad**（≈ −86.5°）。
