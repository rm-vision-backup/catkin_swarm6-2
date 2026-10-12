# 五机工程测试（A08 领机 + A10/A12/A13/A16；通信分阶段 + 整体流程）

本目录是四机规程（`four_uav_recon/README.md`）的**五机版**：2026-10-10 决定在四机阵容
（A08+A10/A12/A13）基础上加入 **A16**（顶替 A06 参训）。任务与航线沿用历轮同一份
（`prepare-a` + `coverage-segment-1`，领机 6 航点逐字不变），只把队伍扩到 **5 架**。

与四机规程的差异只有五处，**其余条目照 `four_uav_recon/README.md` 执行**：

| 项 | 四机 | 五机（本目录） |
|---|---|---|
| 参加机 | A08 + A10/A12/A13 | **A08 + A10/A12/A13/A16** |
| 摆位 | 2×2 方阵、原点=方阵中心 | **5 m 十字、原点=A13 机位（中心机）**（§1.2） |
| 标定配置 | `config/calibration_four_uav.yaml` | `config/calibration_five_uav.yaml` |
| 编队/任务包 | `formation_offsets_four_uav` + 四机 package | `formation_offsets_five_uav` + 本目录 `package/` |
| 地面站入口 | `gcs_a_real_four_uav.launch` | `gcs_a_real_five_uav.launch` |

| 文件 | 作用 |
|---|---|
| `driver.py` | 任务下发入口（默认 package=本目录、mission-id=`five-uav-recon-001`，转调 four_uav 实现） |
| `package/` | 任务数据；leader = A08；5 机 prepare 目标见 §1.2 |
| `sitl/` | 五机仿真彩排（`five_uav_spawn.launch` / `run_sim_five_uav.sh`） |
| 分阶段流程 | **复用** `verification/three_uav_recon/air_flow.sh`（已机数参数化，见 §五） |

## 零、起飞前门禁（**全绿才装桨**）

除标注外，判据与出处同 `four_uav_recon/README.md` §零：

| # | 门禁 | 五机判据 |
|---|---|---|
| 0 | 空域净空 | 相对 A13 机位（原点）：**右 ≥20 m、前 ≥30 m、左 ≥15 m、后 ≥10 m、上 ≥15 m**；无人员/建筑/其它航空器 |
| 1 | 地面摆位 | 5 机 **5 m 十字**（§1.2），机头同向（差 ≤30°），摆好**不再挪动** |
| 2 | 身份 + 板钟 | 每台 `hostname -s` = UAVn（81/101/121/131/161）；`date +%s` ≈ 1.79e9 |
| 3 | 上位机预检 | `python3 verification/five_uav_recon/driver.py --dry-run`（§1.4） |
| 4 | 五机起栈 | 逐台 `onboard_ctl.sh status n`，五台都 UP |
| 5 | 邻机订阅 | 每台 `rosnode info /uav_executor_UAVn` 里有 **4 条** `/UAV…/trajectory_intent` + **4 条** `/UAV…/swarm_pose`（共 8 行；自己不该出现） |
| 6 | 地面站 | 五机 launch 无 `ValueError`；`formation_offsets_file` 指向五机那份（§三） |
| 7 | 标定 | 五机**全部 disarm**、GPS `fix_type ≥ 3`、`/group_a/calibration_ready` = True（**必须先于 arm**） |
| 8 | 现场项（非本工作区） | PX4 `COM_RCL_EXCEPT=4` / `NAV_RCL_ACT=0` 已按现场 RC 情况复核；遥控器在手 |
| 9 | 装桨后 | 逐机遥控 arm + OFFBOARD，`/mavros/state` = `armed: True` / `mode: OFFBOARD`；**飞机会自动垂直到 5 m**，人员已撤离 |
| 10 | 下发任务 | **五机都已 armed + OFFBOARD** 才算满足 `prepare()` 门禁；提前发会被 `VEHICLE_NOT_ARMED` 整批拒 |

## 一、现场准备

### 1.1 空域净空（五机版）

任务系 x=右、y=前（都按机头方向；原点 = **A13 机位**，见 §1.2）。
领机航线 x∈[-2.5, 17.5]、y∈[-2.5, 21.5]（与历轮同一份）；跟随者 offset
A10(-5,+5) / A13(-5,0) / A12(-10,0) / A16(-5,-5)（`formation_offsets_five_uav.yaml` 的
`formation_5uav`），全队包络（相对 A13 机位）：

| 方向 | 半径 | 怎么来的 |
|---|---|---|
| 右 | **20 m** | 队形最右 17.5（航线最右），领机右侧无跟随者 → 留余量 |
| 前 | **30 m** | 队形最前 26.5（航线最前 21.5 + A10 前偏 5） |
| 左 | **15 m** | 队形最左 −12.5（航线最左 −2.5 − A12 左偏 10）→ 留余量 |
| 后 | **10 m** | 航线最后 −2.5 − A16 后偏 5 = −7.5 → 留余量 |
| 上 | **15 m** | prepare 阶段升到 15 m 层（`layer_move_to=15.0`） |

### 1.2 地面摆位（决定 prepare 阶段飞多远）

标定冻结的原点 = **五机全球位置逐轴中位数** = 十字布局的**中心 = A13 机位**：

| 机 | 物理位（右/前，相对 A13） | 任务坐标 | prepare 目标（z=12） |
|---|---|---|---|
| A13 | (0, 0) 中心 | (0, 0) | (0, 0) |
| A08（领机） | (+5, 0) A13 右侧 5 m | (+5, 0) | (+5, 0) |
| A10 | (0, +5) A13 正前 5 m | (0, +5) | (0, +5) |
| A12 | (−5, 0) A13 左侧 5 m | (−5, 0) | (−5, 0) |
| A16 | (0, −5) A13 正后 5 m | (0, −5) | (0, −5) |

1. 五机按上表摆成十字、相邻 5 m；机头同向（航向差 ≤30°，否则标定不出 ready）；
2. 摆好后不再挪动 → 标定 → 再 arm；
3. 摆得不够准只会让 prepare 多飞几米（远小于 150 s 阶段超时），不致命。

### 1.3 上电、身份与板钟

```bash
for ip in 81 101 121 131 161; do
  ssh nvidia@192.168.5.$ip 'hostname -s; date +%s'
done
# 期望：UAV8/10/12/13/16 与 ~1.79e9；板钟停在 1970 就先修（必须在起栈之前）：
./src/tcp_to_ros/tools/onboard_clock_fix.sh 8 10 12 13 16
```

### 1.4 上位机预检（不连地面站，秒级）

```bash
cd "$HOME/catkin_swarm6-2"
python3 verification/five_uav_recon/driver.py --phases prepare coverage --dry-run
# 期望两条 COMMAND：
#  ① MOVE_TO：A08(+5,0) / A10(0,+5) / A12(−5,0) / A13(0,0) / A16(0,−5)，z=12
#  ② FOLLOW_ROUTE：A08 六个航点；A10/A12/A13/A16 为 formation_follow；leader=A08
```

## 二、机载启动（每台各一条；**参数逐台不同**）

**实飞**命令 = 下面五条（**不带** `noprop`；`uav_task_timeout_s:=280` 必传）。
无桨轮参照 `no_prop_link_test.md` 的做法在实飞命令上加 `noprop:=true`。

```bash
# ── UAV16 / A16 ── 邻居 = 8 10 12 13
ssh nvidia@192.168.5.161 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV10/trajectory_intent,/UAV12/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV8/swarm_pose,/UAV10/swarm_pose,/UAV12/swarm_pose,/UAV13/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV8 / A08（领机） 邻居 = 6 10 12 13 ──
ssh nvidia@192.168.5.81 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV16/trajectory_intent,/UAV10/trajectory_intent,/UAV12/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV16/swarm_pose,/UAV10/swarm_pose,/UAV12/swarm_pose,/UAV13/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV10 / A10 邻居 = 6 8 12 13 ──
ssh nvidia@192.168.5.101 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV16/trajectory_intent,/UAV8/trajectory_intent,/UAV12/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV16/swarm_pose,/UAV8/swarm_pose,/UAV12/swarm_pose,/UAV13/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV12 / A12 邻居 = 6 8 10 13 ──
ssh nvidia@192.168.5.121 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV16/trajectory_intent,/UAV8/trajectory_intent,/UAV10/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV16/swarm_pose,/UAV8/swarm_pose,/UAV10/swarm_pose,/UAV13/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV13 / A13 邻居 = 6 8 10 12 ──
ssh nvidia@192.168.5.131 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV16/trajectory_intent,/UAV8/trajectory_intent,/UAV10/trajectory_intent,/UAV12/trajectory_intent \
  neighbor_odom_topics:=/UAV16/swarm_pose,/UAV8/swarm_pose,/UAV10/swarm_pose,/UAV12/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'
```

起栈前先确认五台都没在跑（`onboard_ctl.sh status n` 逐台；有残留先 `stop n`）；
起栈后核对门禁 4/5（邻机订阅共 8 行）。
**不要**用 `onboard_ctl.sh start 8 10 12 13 16 -- <参数>` 扇出（邻居清单每台不同）。

## 三、地面站启动（本机 = GCS_A，192.168.5.241）

```bash
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'cd ~/catkin_swarm6-2 && source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
           roslaunch tcp_to_ros gcs_a_real_five_uav.launch'
```

- [ ] 启动日志无 `ValueError`；calibration 段打印 `calibration set=UAV16,UAV8,UAV10,UAV12,UAV13`
- [ ] 防"静默回退"：
  ```bash
  rosparam get /gcs_a_backend_node/formation_offsets_file   # 必须是 five_uav 那份
  ```
- [ ] 本机 TCP 服务在听：`nc -z 192.168.5.241 39001 && echo "tcp OK"`

## 四、跑一轮（整任务）

1. **标定**（硬顺序：先于 arm）：`rostopic echo -n1 /group_a/calibration_ready` 期望 True；
   冻结前逐台看 GPS 质量（`/mavros/gpsstatus/gps1/raw` 的 `fix_type ≥ 3`）。
2. **装桨 + 遥控 arm/OFFBOARD**（逐机）：核对 `armed: True` / `mode: "OFFBOARD"`；
   **自动垂直到 5 m 是设计行为**；遥控器始终在手。
3. **下发**（五机都 armed + OFFBOARD 之后）：
   ```bash
   python3 verification/five_uav_recon/driver.py \
     --tcp-host 192.168.5.241 --tcp-port 39001 \
     --log verification/five_uav_recon/logs/real_$(date +%Y%m%d_%H%M%S).jsonl
   ```
4. **预期飞行过程**：prepare（升 15 m 层 → 水平微调 → 降 12 m，各机位移≈0）→
   coverage（A08 绕 6 航点；A10/A12/A13/A16 按 `formation_5uav` offset 跟飞）。

## 五、分阶段通信测试（air_flow，五机）

`air_flow.sh` 已机数参数化（默认仍是三机），五机加两个环境变量即可：

```bash
export AIR_FLOW_UAVS="8 10 12 13 16"
export AIR_FLOW_DIR=five_uav_recon          # driver 用本目录包装

bash verification/three_uav_recon/air_flow.sh odom          # 秒级分层快查
bash verification/three_uav_recon/air_flow.sh bag 1800      # 五台同录 rosbag（-u 6,8,10,12,13）
bash verification/three_uav_recon/air_flow.sh stage1 60     # 空中静态链路窗 + 判定
bash verification/three_uav_recon/air_flow.sh stage2        # 只发 coverage（跟随环事件）
bash verification/three_uav_recon/air_flow.sh full          # prepare + coverage 全流程
```

判据/定层口径与三机 §九 完全相同；五机时每次窗口检查 5 台的 remote/local 与事件。

## 六、成功判据、观测与收尾

- 判据：driver 退出码 0；`[prepare] SUCCESS` + `[coverage] SUCCESS`；GCS 侧 **5/5 COMPLETED**；
  全程 `MIN_DISTANCE_BREACH` 0 次、`EGO_PLAN_FAILED` 0 次；各机保持其 offset（两两间距 ≥5 m）。
- 观测与异常处置同 `four_uav_recon/README.md` §五/§六（遥控接管第一优先）。
- 收尾：
  ```bash
  for n in 8 10 12 13 16; do ./src/tcp_to_ros/tools/onboard_ctl.sh stop $n; done   # 逐台
  ./src/tcp_to_ros/tools/log_fetch.sh 8 10 12 13 16        # 不含 ulog；bag 手工取（见三机 §9.4）
  ```
  `mavlink-routerd` 不要停（现场常驻件）。

## 七、彩排（SITL）

```bash
bash verification/five_uav_recon/sitl/run_sim_five_uav.sh
```

自动完成：5 机 SITL（十字布局，中心=iris_12=A13）→ 机载（5 台，各独立 master）→
五机 GCS + 标定 → takeoff → 静置 30 s → 五机 driver `prepare + coverage`。
失败轮归档到 `.ros_home_round_five_uav`（含机载日志）。退出码 0 = 两阶段都 SUCCESS。

## 八、已知风险与提醒

1. 四机 README §八 的 1–5 条**全部适用**（`onboard_ctl.sh` 多目标只处理第一个、
   跟随者完成判据死区、formation 静默回退、标定 ALL 语义、参考值）。
2. **五机特有**：`formation_5uav` 的 participants 必须精确等于 {A16,A08,A10,A12,A13}；
   增/减机必须换配置（三机轮用 three 那份、四机轮用 four 那份）。
3. **A16 顶替 A06 首次进入本阵容、A12 是 GPS 修复后首次参训**——两者都建议先在
   §七 SITL / 地面干跑里过一遍，再上五机实飞。
4. 五机邻居矩阵为 4×5=20 条机间流（此前三机是 6 条），链路负载明显上升——沿用
   当前减负配置（机间位姿流 `swarm_pose` 12 Hz、每流 ~45 B、router 去 242），
   并按 air_flow `stage1` 的判据观察。
