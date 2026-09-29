# 2026-09-22 三机编队侦查飞行 · 数据包（three-uav-recon-001）

- **机队**：A08（领机）/ A10 / A13 = UAV8 / UAV10 / UAV13
- **任务**：`three-uav-recon-001`，两阶段（prepare = MOVE_TO，coverage = FOLLOW_ROUTE）
- **运行时间**：2026-09-22 17:12:16 开始下发；driver 日志 `real_20260922_171216.jsonl`
- **结果**：**prepare SUCCESS / coverage FAILURE**（driver 退出码 1，已自动发 HOLD 兜底）
- **规程**：`verification/four_uav_recon/README.md` §四

## 一、结论速览

| 阶段 | A08（领机） | A10 | A13 |
|---|---|---|---|
| prepare（MOVE_TO） | SUCCESS | SUCCESS | SUCCESS |
| coverage（FOLLOW_ROUTE） | **SUCCESS** | **SUCCESS** | **FAILURE** |

失败码：**`A13 FAILURE / LEADER_LOST / "leader odom timed out"`**
—— 跟随者 A13 在航线段丢失领机 A08 的 odom（判据：>1.0 s 无新帧，
`ego_swarm.py:533-537`，`_FOLLOW_POSE_TIMEOUT_S=1.0`）。

**但更值得注意的事实（ulog 证据）**：本轮**编队跟随根本没发生** ——
领机 A08 飞完了 6 个航点（12 m 层 ~130 s、水平移动 ~30 m），
而 A10 / A13 只在 12 m 层定点悬停（水平移动各仅 ~7–8 m），
**A10 却仍被判 SUCCESS**（与数据矛盾）。
⇒ 分析主线不是"A13 单机丢链路"，而是"**跟随者为什么没跟随**"＋"**A10 为何误判成功**"。

> **注意**：本文件 §四 记录了一次已被推翻的早期推断（"A08↔A13 双向断链"），
> 引用请以 §四.1 的 ulog 事实为准。

## 二、目录结构与命名

```
2026-09-22_1712_three-uav-recon-001/
├── README.md                  ← 本文件（结论 + 索引）
├── gcs/                       ← 地面站（本机）侧
│   ├── driver_three-uav-recon-001_20260922_171216.jsonl   ★ 逐事件记录（最快定位失败）
│   ├── GCS_a_backend_node.log    分组调度后端（goal/schedule/ACCEPTED）
│   ├── GCS_a_calibration.log     标定节点（冻结原点与航向）
│   ├── Bridge_on_GCS.log         GCS 侧桥接（proxying 列表）
│   ├── TCP_server.log            下发通道
│   ├── rosout_all_nodes.log      全节点 rosout 合集
│   └── ros_logs/                 原始 session 目录原样副本
├── onboard/                   ← 三台机载（由 tools/log_fetch.sh 采集，含 manifest/md5）
│   ├── UAV8_2026-09-22_1718/     ├─ fcu/           PX4 ulog
│   ├── UAV10_2026-09-22_1718/    ├─ onboard/ros_home/<session>/  各节点日志
│   └── UAV13_2026-09-22_1718/    ├─ onboard/ego_planner/         规划器诊断
│                                 └─ onboard/term/                roslaunch 终端转录
└── config_snapshot/           ← 本轮生效的配置与代码版本（写报告时引用参数用）
```

命名约定：`<日期>_<开始时刻(HHMM)>_<mission-id>/`；机载目录沿用工具自带的
`UAVn_<日期>_<采集时刻>/`。多轮飞行各占一个同级目录，互不覆盖。

## 三、分析入口（按优先级）

> 先说结论：**"跟随者为什么没跟随"＋"A10 为何被判成功"是本轮两条主线**（见 §4.1-B / §4.3）。
> 机载 ROS 日志大多在 17:12:35 前后就静默，**时间序列一律以 ulog 为准**。

1. `onboard/UAV8|10|13_*/fcu/*.ulg` —— 三台真实轨迹/姿态/setpoint（**权威时间基**）。
   读法：`pyulog.ULog`，看 `vehicle_local_position` / `vehicle_local_position_setpoint` /
   `vehicle_attitude` / `vehicle_status`；也可上传 FlightReview。注意三台 x/y 是**各自 EKF 局部系**。
2. `onboard/UAV10_*/onboard/term/UAV10_offboard_ego.log`、`UAV13_*` 同名文件 ——
   跟随环是否在跑（`setpoint_relay` 候选/超时、`ego_swarm` 的 follow 目标日志）
3. `gcs/driver_*.jsonl` —— 事件流与失败码（`command_feedback.error_code`）
4. `onboard/UAV13_*/onboard/ego_planner/UAV13-ego-planner.log` —— 安全监督层快照
   （**只在日志未结束的时间段内有效**，见 §4.2）
5. `onboard/UAV13_*/onboard/ros_home/<session>/swarm_bridge-*.log` —— 桥接注册/连接情况
6. **对照 A10**（"成功"但实际悬停）：`onboard/UAV10_*/...` 同名文件

## 四、事实与待查（2026-09-22 采集后整理；**含一次已作废的推断**）

### 4.1 已确立的事实

**A. 失败码**：`A13 FAILURE / LEADER_LOST / "leader odom timed out"`
—— 是**陈旧**分支（曾收到过领机 odom，随后 >1.0 s 无新帧），
判据 `ego_swarm.py:533-537`，`_FOLLOW_POSE_TIMEOUT_S = 1.0`（:65）。

**B. 三台的实际飞行（以飞控 ulog 为准，权威时间基）**

| 机 | 12 m 层时长 | 水平移动总量 | 形态 | driver 判定 |
|---|---|---|---|---|
| A08（领机） | ~130 s | ~30 m（x 6→-21→1、y 0→-18） | **飞完 6 个航点** ✓ | SUCCESS ✓ |
| A10 | ~109 s | **~8 m**（y 5.4→-2.7 后定点悬停 ~80 s） | **未跟随** ✗ | SUCCESS **✗ 与数据矛盾** |
| A13 | ~110 s | **~7 m**（y -1→-7 后定点悬停 ~60 s） | **未跟随** ✗ | FAILURE（LEADER_LOST） |

⇒ 本轮 **编队跟随实际没有发生**：跟随者到位置后就定点悬停了，只有领机飞了完整航线。
这是比失败码本身更值得写进报告的现象。

**C. 桥接配置正常**：A13（17:08:56）与 A10（17:08:55）的 master 里，
`/UAV8/mavros/local_position/odom` 都注册了 `swarm_bridge` 发布者 + 本机 executor 订阅。

### 4.2 已被推翻的推断（**不要引用**）

采集初期曾根据 A13 的 `ego_planner` 日志推断"A08↔A13 在 17:12:33.8 双向断链导致立即失败"。
**该推断作废**：那份 planner 日志在 **17:12:35 就结束**（早于/正落在 coverage 开始 17:12:36.6），
其中的 `traj` 冻结与 `age` 增长只是**日志结束前的最后一次快照**，不能外推到航线段。
教训：拿机载节点日志的尾部当"事件终点"是错的，必须以 ulog 时间基对齐后再下结论。

### 4.5 追加分析：">1 s 延迟"到底出在哪一层（ulog + 日志 mtime 深挖）

**问题**：链路已限频（odom 20 Hz、pose 10 Hz、intent 不限频 5 Hz），为什么还会出现超过 1.0 s 的延迟？

**结论：延迟不来自速率，而来自"桥接传输层"的 stall。** 证据如下（都可复核）：

| # | 证据 | 数据来源 | 说明 |
|---|---|---|---|
| 1 | **伴飞链路（FCU↔机载）全程零中断**：`telemetry_status` 里 companion 链路（`mode=2`=Onboard）`rx_rate_avg` 中位 2525、**最小 2501**、**为 0 的样本 0/169**（三台一致） | 三台 `*.ulg` | 1 Hz 采样下任何 >1 s 断流都会露出 0 ⇒ **串口/UART/物理链路无辜** ✗ |
| 2 | 同一条链路上飞控**传给**机载的 `data_rate=46080`（≈46 KB/s）恒定 ⇒ 领机 mavros 的 odom 源数据从未断供 | 同上 | 源端也没问题 ✗ |
| 3 | **不限频的 5 Hz `trajectory_intent` 在起飞阶段就已经有 0.5–1.0 s 的 age**，17:12:26 越过 1 s | 领机 `UAV8-ego-planner.log` | 5 Hz 都这样 ⇒ **与限频数值无关**，是整条连接的 stall |
| 4 | 领机安全监督层：`level` 在 **17:12:26 由 `SAFE` 变 `INSUFFICIENT_DATA`**，此后整个航线段（~115 s）再没恢复 | 同上 | 邻机数据不可用 ⇒ CPA/间距评估失效（**安全相关**） |
| 5 | **17:12:34 起领机对 A10/A13 的 age 单调涨到 100 s 不恢复** | 同上 | 机间桥接**永久性**断掉（不是偶发丢包） |
| 6 | **三台 planner 都在 17:12:35 顿挫**：领机只停 1.24 s 后恢复；**两个跟随者从此再没输出**（日志 mtime 17:12:34/35） | 三台 planner 日志 + mtime | 与证据 5 是**同一事件**（跟随者 intent 停发 ⇒ 领机收不到） |
| 7 | 跟随者 setpoint 在 **17:13:20–25** 之后完全冻结（A10 停在 (-2.46,-2.73)、A13 停在 (-2.38,-6.91)），位置也定住不动 | 三台 `vehicle_local_position_setpoint` | 跟随环在此终结；领机却继续飞到 ~17:14:25 ✓ |

**机制解释（为什么限频挡不住）**：

- 限频只约束**吞吐**，不约束**连续性**。桥接是 **TCP/ZMQ 中继**，跑在同一个电台上：
  拥塞/丢包时 TCP 会因重传与 RTO 退避**整条连接停顿 1 s 以上**（这正是 `topology.yaml` 注释里
  记录的"几十条挤在一起 + 1~3 s 空档"，本轮限频就是为它加的缓解措施 —— 是缓解，不是根治），
  严重时**连接直接掉死**。
- 同一条电台链路上，**MAVLink 遥测走的是 UDP**：丢包即丢样本、流不中断，
  因此在 1 Hz 的 `telemetry_status` 上**看不出任何异常** —— 两条路径对同一拥塞的反应完全不同。
- 故障还是**按连接、且方向不对称**的：17:12:34 先死的是 "A10/A13 → A08"；"A08 → 跟随者"
  又撑了约 50 s（跟随者到 17:13:20-25 才冻结）⇒ 是**连接级**断裂，不是电台整体掉线。

**由此得到的两条报告级结论**：

1. 本轮失败的直接链条：**机间桥接先降级（0.5~1 s 空档）→ 17:12:26 安全层失去邻机数据（INSUFFICIENT_DATA）
   → 17:12:34 跟随者 planner 停摆、intent 断流 → 17:13:20-25 跟随环终结（A13 判 `LEADER_LOST`，
   A10 却记成 SUCCESS）→ 领机不受影响继续飞完航线**。
2. **安全层在整段航线上是"盲飞"状态**：邻机数据缺失时 CPA/间距门禁无法评估，
   除执行层 1 m 门禁外没有机间保护 —— 这条必须写进报告并在下轮前决定策略（告警？中止？）。

### 4.3 待查（按优先级，已按 §4.5 的结论收敛）

- [ ] **为什么两个跟随者的 planner 在 17:12:34-35 之后再没输出？**（领机同一时刻只顿挫 1.24 s 就恢复）
  ⇒ 疑似在"prepare 完成 / follow 启动"这条状态迁移上有阻塞或崩溃，需查代码：
  `swarm_uav_executor`（`ego_swarm.py` 的任务迁移）+ `ego_planner_driver`（planner 节点的线程/日志），
  并给 planner 加一条低频心跳日志以便下轮定位。
- [ ] **为什么 A10 跟随环终结被记成 SUCCESS？** 完成判据 `ego_swarm.py:581-585` 在该场景下给出错误结论，
  直接影响任务编排可信度，优先查。
- [ ] **桥接传输层加固**：给 bridge 加"每话题收包计数/断连重连日志"（本轮对断链**零记录**，是最大盲点）；
  评估把时延敏感话题（`mavros/local_position/odom`、`trajectory_intent`）改走 UDP 传输，
  或加"桥接陈旧 ⇒ 显式中止"的 watchdog，避免像本轮这样**带病继续飞**。
- [ ] **安全层策略**：邻机数据缺失（INSUFFICIENT_DATA）时是告警、降级还是中止任务？需要设计决策。
- [ ] 本轮机载 **ROS 节点日志没覆盖飞行段**（`rosout.log` / 各节点 log 的 mtime 停在 17:11:43–17:12:09）
  ⇒ 下轮前确认日志机制（或直接依赖 rosbag），否则又只能靠 ulog 反推。

### 4.4 建议下一轮必做

- [ ] **录 rosbag**：`/UAVn/mavros/local_position/odom`、`/UAVn/mavros/local_position/pose`、
      `/UAVn/trajectory_intent`、`/UAVn/uav_task_state`、`/UAVn/uav_task_control`
      （有了它，本轮这类问题 5 分钟就能定死；桥接话题已限频，录包压力可控，建议录在机上）
- [ ] 起栈后、下发前，**逐台确认跟随环在跑**：`setpoint_relay` 是否有候选（日志关键字见 4.3-③）

## 五、采集状态

| 项 | 状态 |
|---|---|
| GCS 侧（driver JSONL / backend / calibration / bridge / TCP / rosout） | **已完成** ✓ |
| 三台机载 ROS 日志（ros_home / ego_planner / term） | **已完成** ✓（小文件走 rsync 快取；UAV8 由 tools/log_fetch.sh 采集） |
| 配置与版本快照 | **已完成** ✓ |
| 三台飞控 ulog | **已完成** ✓（用户自行下载后归档） |

**ulog 清单与校验**

| 机 | 文件 | 字节 | 时长 | 备注 |
|---|---|---|---|---|
| UAV8 | `fcu/log_52_2026-9-22-17-14-54.ulg` | 7,039,562 | 169.0 s | 与飞控清单 `log_52` **逐字节一致** ✓（清单见 `fcu/ulog_list.txt`） |
| UAV10 | `fcu/log_32_2026-9-22-17-14-26.ulg` | 6,683,357 | 167.4 s | pyulog 解析通过 ✓（飞控侧清单未取到，不影响） |
| UAV13 | `fcu/log_54_2026-9-22-17-14-26.ulg` | 6,681,120 | 161.7 s | pyulog 解析通过 ✓ |

> 读取方式：`from pyulog import ULog`；关键数据集 `vehicle_local_position`（x/y/z，
> 注意是**各机自己的 EKF 局部系**，跨机比较需先对齐原点）、`vehicle_attitude`、
> `vehicle_local_position_setpoint`、`vehicle_status`。
> 用户下载时在工作区根留下了 `8/`、`10/`、`13/` 三个目录（本包内已有副本，可自行清理）。

## 六、备注

- **本轮未录 rosbag**：机载时间序列只能从 PX4 ulog 取。下次实飞建议补录
  `/UAVn/mavros/local_position/odom`、`/UAVn/uav_task_state`、`/UAVn/trajectory_intent`
  （桥接话题已在 topology 限频，录包对电台链路压力可控）。
- 本目录是**分析数据**，不要提交进 git（建议把 `src/uav_log/` 加进 .gitignore）。
- 本轮配置为 3 机方案（11 号机链路故障，见 `verification/four_uav_recon/README.md` §十）。
