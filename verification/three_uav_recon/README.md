# 编队实机测试：准备编队 + 侦查（3 机：A08 领机 + A10/A13）

本目录是四机规程（`four_uav_recon/README.md`）的**三机版**：2026-10-09 现场只启用 3 台
（12 号机缺席），四机配置的标定集合与 formation 都含 A12（均为精确匹配/ALL 语义），
缺机时任务会被 `CALIBRATION_NOT_READY` / `INVALID_FORMATION_*` 拒 —— 故单开三机一套。
席位与 **2026-09-22 三机轮**同阵容（A08 领机 + A10/A13，见四机 README §十 席位变更史）。

与四机规程的差异只有四处，**其余条目照 `four_uav_recon/README.md` 执行**：

| 项 | 四机 | 三机（本目录） |
|---|---|---|
| 参加机 | A08 + A10/A12/A13 | **A08 + A10/A13** |
| 摆位 | 2×2 方阵、原点=方阵中心 | **5 m L 形、原点=A13 机位**（§1.2） |
| 标定配置 | `config/calibration_four_uav.yaml` | `config/calibration_three_uav.yaml` |
| 编队/任务包 | `formation_offsets_four_uav` + 四机 package | `formation_offsets_three_uav` + 本目录 `package/` |
| 地面站入口 | `gcs_a_real_four_uav.launch` | `gcs_a_real_three_uav.launch` |

| 文件 | 作用 |
|---|---|
| `driver.py` | 任务下发入口：补默认 package/mission-id 后换成 `four_uav_recon/driver.py`（同一实现，退出码/信号一致） |
| `package/` | 任务数据（robots / bootstrap / world / plans）；`prepare-a` + `coverage-segment-1`；leader = A08；mission_id = `three-uav-recon-002` |
| `no_prop_link_test.md` | **无桨**（不装桨、不 arm）链路 + 跟随环 dry-run 验证规程（当前现场轮次入口） |
| `package_ground/` + `ground_flow.sh` | **地面贴地彩排**：飞机原地不动跑完 prepare → coverage（见 §七） |

## 零、起飞前门禁（**全绿才装桨**）

除标注外，判据与出处同 `four_uav_recon/README.md` §零：

| # | 门禁 | 三机判据 |
|---|---|---|
| 0 | 空域净空 | 相对 A13 机位（原点）：**右 ≥20 m、前 ≥30 m、左 ≥10 m、后 ≥5 m、上 ≥15 m**；无人员/建筑/其它航空器 |
| 1 | 地面摆位 | 三机 **5 m L 形**（§1.2），机头同向（差 ≤30°），摆好**不再挪动** |
| 2 | 身份 + 板钟 | 每台 `hostname -s` = UAVn；`date +%s` ≈ 1.79e9 |
| 3 | 上位机预检 | `driver.py --dry-run`（§1.4） |
| 4 | 三机起栈 | 逐台 `onboard_ctl.sh status <n>`，三台都 UP |
| 5 | 邻机订阅 | 每台 `rosnode info /uav_executor_UAVn` 里有 **2 条** `/UAV…/trajectory_intent` + **2 条** `.../odom`（共 4 行；自己不该出现） |
| 6 | 地面站 | 三机 launch 无 `ValueError`；`formation_offsets_file` 指向三机那份（§三） |
| 7 | 标定 | 三机**全部 disarm**、GPS `fix_type ≥ 3`、`/group_a/calibration_ready` = True（**必须先于 arm**） |
| 8 | 现场项（非本工作区） | PX4 `COM_RCL_EXCEPT=4` / `NAV_RCL_ACT=0` 已按现场 RC 情况复核；遥控器在手，模式开关与应急开关已试过 |
| 9 | 装桨后 | 逐机遥控 arm + OFFBOARD，`/mavros/state` = `armed: True` / `mode: OFFBOARD`；**飞机会自动垂直到 5 m**，人员已撤离 |
| 10 | 下发任务 | **三机都已 armed + OFFBOARD** 才算满足 `prepare()` 门禁；提前发会被 `VEHICLE_NOT_ARMED` 整批拒 |

## 一、现场准备

### 1.1 空域净空（三机版）

任务系 x=右、y=前（都按机头方向；原点 = **A13 机位**，见 §1.2）。
领机航线 x∈[-2.5, 17.5]、y∈[-2.5, 21.5]（与四机轮同一份）；跟随者 offset
A10(-5,+5) / A13(-5,0)（`formation_offsets_three_uav.yaml` 的 `formation_3uav`），
全队包络（相对 A13 机位）：

| 方向 | 半径 | 怎么来的 |
|---|---|---|
| 右 | **20 m** | 队形最右 17.5（航线最右），领机右侧无跟随者 → 留余量 |
| 前 | **30 m** | 队形最前 26.5（航线最前 21.5 + A10 前偏 5） |
| 左 | **10 m** | 队形最左 −7.5（航线最左 −2.5 − A10/A13 左偏 5） |
| 后 | 5 m | 航线最后 −2.5 |
| 上 | **15 m** | prepare 阶段升到 15 m 层（`layer_move_to=15.0`） |

### 1.2 地面摆位（决定 prepare 阶段飞多远）

标定冻结的原点 = **三机全球位置逐轴中位数** = 方阵的**原点角 = A13 机位**
（三机的中位数落在角上，与四机落在中心不同——按本表摆，prepare 目标才与实位一致）：

| 机 | 物理位（右/前，相对 A13） | 任务坐标 | prepare 目标（z=12） |
|---|---|---|---|
| A13 | (0, 0) 原点角 | (0, 0) | (0, 0) |
| A08（领机） | (+5, 0) A13 右侧 5 m | (+5, 0) | (+5, 0) |
| A10 | (0, +5) A13 正前 5 m | (0, +5) | (0, +5) |

1. 三机按上表摆成 L 形、相邻 5 m；机头同向（航向差 ≤30°，否则标定不出 ready）。
2. 摆好后不再挪动 → 标定 → 再 arm。
3. 摆得不够准只会让 prepare 多飞几米（远小于 150 s 阶段超时），不致命。

### 1.3 上电、身份与板钟

```bash
for ip in 81 101 131; do
  ssh nvidia@192.168.5.$ip 'hostname -s; date +%s'
done
# 期望：UAV8/10/13 与 ~1.79e9；板钟停在 1970 就先修（必须在起栈之前）：
./src/tcp_to_ros/tools/onboard_clock_fix.sh 8 10 13
```

### 1.4 上位机预检（不连地面站，秒级）

```bash
cd "$HOME/catkin_swarm6-2"
python3 verification/three_uav_recon/driver.py --phases prepare coverage --dry-run
# 期望两条 COMMAND：
#  ① MOVE_TO：A08(+5,0) / A10(0,+5) / A13(0,0)，z=12
#  ② FOLLOW_ROUTE：A08 六个航点；A10/A13 为 formation_follow；leader=A08
```

## 二、机载启动（每台各一条；**参数逐台不同**）

起栈口径与四机模式一致（邻居清单改成三机的 2 台；`uav_task_timeout_s:=280` 必传）。
**实飞**命令 = 下面三条**去掉 `noprop:=true`**；**无桨轮**用 `no_prop_link_test.md` §二 那三条。

```bash
# ── UAV8 / A08（领机）  邻居 = 10 13 ──
ssh nvidia@192.168.5.81 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV10/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV10/mavros/local_position/odom,/UAV13/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV10 / A10  邻居 = 8 13 ──
ssh nvidia@192.168.5.101 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV8/mavros/local_position/odom,/UAV13/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV13 / A13  邻居 = 8 10 ──
ssh nvidia@192.168.5.131 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV10/trajectory_intent \
  neighbor_odom_topics:=/UAV8/mavros/local_position/odom,/UAV10/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'
```

起栈前先确认三台都没在跑（`onboard_ctl.sh status n` 逐台；有残留先 `stop n`）。
起栈后核对门禁 4/5 见 `no_prop_link_test.md` §二 的核对段（三机同样适用）。
不用传的参数（`formation_offsets` / `leader_odom_topic_prefix` / `uav_name`）与四机 README §二
相同；**不要**用 `onboard_ctl.sh start 8 10 13 -- <参数>` 扇出（邻居清单每台不同，
且该工具多目标只处理第一个）。

## 三、地面站启动（本机 = GCS_A，192.168.5.241）

```bash
# 本机 ~/.bashrc 的 ROS_HOSTNAME=yjq 经 /etc/hosts 解析到不存在的 192.168.1.100（且
# ROS_MASTER_URI 指向别机）→ 直接 roslaunch 会卡在自检（2026-10-09 实测）。必须显式覆盖：
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && \
           roslaunch src/tcp_to_ros/launch/gcs_a_real_three_uav.launch'
```

- [ ] 启动日志无 `ValueError`；calibration 段打印 `calibration set=UAV8,UAV10,UAV13`
- [ ] 防"静默回退"：
  ```bash
  rosparam get /gcs_a_backend_node/formation_offsets_file   # 必须是 three_uav 那份
  ```
- [ ] 本机 TCP 服务在听：`nc -z 192.168.5.241 39001 && echo "tcp OK"`

## 四、跑一轮

1. **标定**（硬顺序：先于 arm）：`rostopic echo -n1 /group_a/calibration_ready` 期望 True；
   冻结前逐台看 GPS 质量（`/mavros/gpsstatus/gps1/raw` 的 `fix_type ≥ 3`；不要用
   `global_position/global.status` 判 fix）。记录 `/group_a/gp_origin` 与 `reference_heading`。
2. **装桨 + 遥控 arm/OFFBOARD**（逐机）：核对 `armed: True` / `mode: "OFFBOARD"`；
   **看到自动垂直到 5 m 是设计行为**；从此刻起遥控器始终在手。
3. **下发**（三机都 armed + OFFBOARD 之后）：
   ```bash
   python3 verification/three_uav_recon/driver.py \
     --tcp-host 192.168.5.241 --tcp-port 39001 \
     --log verification/three_uav_recon/logs/real_$(date +%Y%m%d_%H%M%S).jsonl
   ```
   任务号口径：driver 强制 `bootstrap.mission_id == --mission-id`；重试**不需要**换号
   （每次运行的 command_id 含新时间戳，不会命中 1 h 缓存）。详见 `no_prop_link_test.md` §四.3。
4. **预期飞行过程**（与四机轮相同，只有队形不同）：

| # | 动作 | 高度 | 完成标志 |
|---|---|---|---|
| ① | 切 OFFBOARD 后自动起飞 | 0 → 5 m | 垂直上升约 5 s |
| ② | 准备编队（MOVE_TO） | 升到 15 m 层 → 水平微调 → 降到 12 m | `[prepare] SUCCESS`；三机水平位移≈0（按 §1.2 摆位） |
| ③ | 侦查（FOLLOW_ROUTE） | 保持 12 m | A08 绕 6 航点（20 m × 24 m 矩形往返，约 84 m）；A10 跟 leader+(-5,+5)、A13 跟 leader+(-5,0)；`[coverage] SUCCESS` |
| ④ | 结束 | 12 m 悬停 | driver 退出码 0 |

## 五、异常处置

逐条照 `four_uav_recon/README.md` §五 执行（遥控接管第一优先；driver Ctrl-C 自动 HOLD；
`MIN_DISTANCE_BREACH`/`EGO_PLAN_FAILED` 进安全终态；单台链路异常不要就地重启机载栈）。
唯一差异：报 `VEHICLE_NOT_ARMED` / `VEHICLE_NOT_OFFBOARD` 时确认是**哪台**没 arm/没切
OFFBOARD，换新任务号重发前先看 `no_prop_link_test.md` §四.3 的任务号口径。

## 六、观测点、收尾与成功判据

成功判据（三机版）：driver 退出码 0；`[prepare] SUCCESS` + `[coverage] SUCCESS`；
GCS 侧 **3/3 COMPLETED**；全程 `MIN_DISTANCE_BREACH` 0 次、`EGO_PLAN_FAILED` 0 次；
A10 与 A08 保持约 (−5,+5) m、A13 约 (−5,0) m（两两间距 ≥5 m）。

观测与收尾同四机 README §六：driver JSONL、GCS rosout、`/UAVn/uav_task_state`、
机载 `.ros_home/log/`（health/planner/终端转录），收尾
`for n in 8 10 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh stop $n; done` +
`./src/tcp_to_ros/tools/log_fetch.sh 8 10 13`（采集含 `onboard/health/*.jsonl`）。
`mavlink-routerd` 不要停（现场常驻件）。

## 七、彩排

### 7.1 地面彩排（原地跑完 prepare → coverage；无桨/未 arm 时用）

两轮实飞的故障发生在"**prepare 完成后进入 coverage 那一瞬**"；按原任务包（层高 12/15 m）
飞机停在地面时规划器会判 `EGO_PLAN_FAILED`、组级 ALL 立刻中止，流程**到不了 coverage**，
复刻不了故障环节。本彩排用**贴地任务包 + 三个部署覆盖参数**，让流程在**原地不动**的情况下
按 prepare → coverage 原序推进：

```bash
# 起栈：在 §二 的三条命令上加这三个参数（邻居清单照旧逐台）
#   noprop:=true layer_move_to:=0.0 layer_follow_route:=0.0 [arrival_reach_thresh_m:=1.0]
bash verification/three_uav_recon/ground_flow.sh          # 只检查（贴地参数是否生效/地面站/干跑）
bash verification/three_uav_recon/ground_flow.sh --run    # 跑两阶段并给判定
```

- 判定：**prepare 3/3 SUCCESS**（= 原地进入 coverage，这才是本彩排的目的）；coverage 是否下发、
  两跟随者是否 `follower_start`/`leader_odom_first_frame`；**coverage 的完成不作要求**
  （跟随判据要求"自机距 leader+offset ≤0.2 m"，不动时满足不了）。
- 三个参数的作用：`layer_move_to/layer_follow_route` 关掉 MOVE_TO/FOLLOW_ROUTE 的层高插入
  （否则原地必然失败）；`arrival_reach_thresh_m` 放宽规划器到达判据（吸收现场摆位残差，默认 0.5 m 偏紧）。
- 不覆盖：**空中链路条件**；任务态链路问题仍需实飞 + health + rosbag。

### 7.2 仿真彩排（SITL）

本目录暂无三机 SITL 脚本；改过配置先跑四机彩排
（`bash verification/four_uav_recon/sitl/run_sim_four_uav.sh`，验证的是共享机制），
三机专属文件（两个 config + package + launch）由 §1.4 预检与无桨干跑覆盖。

## 八、已知风险

1. 四机 README §八 的 1–5 条**全部适用**（`onboard_ctl.sh` 多目标只处理第一个、
   跟随者完成判据死区、formation 静默回退、标定 ALL 语义、参考值）。
2. **三机特有**：`formation_3uav` 的 participants 必须精确等于 {A08,A10,A13}；
   增/减机（例如 12 号机归队）必须换回四机配置，不能只改 package。
3. 三机摆位的中位数落在 **A13 机位**（角上），不是几何中心——若照四机 README §1.2
   只删一台机，prepare 目标会整体偏 ~1.7 m（无桨轮无影响，实飞会多飞一小段）。
   **2026-10-09 实测补记**：现场把 **A10 摆在了原点角**，于是任务包（按 A13 在角上写）让
   A13/A10 各要真飞 ~4.6/5.0 m。起飞前务必按 §1.2 核对"谁在角上"；不符就先摆位或先改包。

## 九、实飞分阶段测试（stage1 → stage2 → 全流程；含机载 rosbag）

目的：把 `no_prop_link_test.md` 的 Stage 1/2 从"地面/无桨"搬到**空中/实飞**，并让每一步都有
两层证据（rosbag 连续量 + health JSONL 分项），直接回答 9/22 的老问题——**跟随者到底有没有
拿到领机 odom、哪一段断了**。流程脚本：`verification/three_uav_recon/air_flow.sh`。

### 9.1 先决条件

- 起栈用**实飞命令**（README §二 去掉 `noprop:=true`，且**不带**任何贴地覆盖参数
  `layer_move_to/layer_follow_route/arrival_reach_thresh_m/enable_yield_candidates`——那些只属于
  §7.1 地面彩排；不传时行为与 canonical 完全一致，见四机 README §八）。
- 摆位按 §1.2 核对（见 §八.3 的实测补记）。
- 板上 rosbag 脚本要新版（含"链路哨兵"话题）：旧版先同步（bash 脚本不用 build）：
  ```bash
  for ip in 81 101 131; do
    rsync -rc -e "sshpass -p nvidia ssh -o StrictHostKeyChecking=no" \
      src/swarm_uav_executor/bashs/record_rosbag.sh \
      nvidia@192.168.5.$ip:~/catkin_swarm6-2/src/swarm_uav_executor/bashs/
  done
  ```
- GCS 起栈见 §三（含环境变量覆盖）；三机遥测可见（`rostopic hz /UAV8/mavros/local_position/odom`）。

### 9.2 步骤（`air_flow.sh` 子命令）

| # | 时机 | 命令 | 判据 / 用途 |
|---|---|---|---|
| 0 | 起飞前 | `bash .../air_flow.sh bag 1200` | 三台同录 bag（默认 600 s；**起飞前就起**，覆盖地面→空中全过程）；脚本会先核对板上脚本版本 |
| 1 | 空中 5 m 悬停 | `bash .../air_flow.sh stage1 60` | 空中静态链路：各机 remote odom 的 rx 增长、窗内 `gt1000=0`（无 ≥1 s 空档）、`stalls=0`、无 stall/link 事件；领机 local odom 同步增长 |
| 2 | stage1 通过后 | `bash .../air_flow.sh stage2` | 只发 coverage：领机起飞到 12 m 走航线、跟随者进跟随环。看 `follower_start` → `leader_odom_first_frame`（`wait_s ≲ 0.1 s`）且**无 LEADER_LOST**；coverage 完整完成是加分项不是门槛 |
| 3 | stage2 通过后 | `bash .../air_flow.sh full` | prepare + coverage 全流程（与 §四 等价）。建议**落地后重开一轮**再下发，状态更干净；同飞行续发也可（prepare 会把各机再拉回停机位正上方） |

任意时刻怀疑断链（尤其 stage2 里出现 9/22 型错误：`LEADER_LOST` / `leader odom timed out`）：

```bash
bash .../air_flow.sh odom     # 秒级分层：每台看"本机发出/收到邻机"的 age、stale_srcs、近期事件
```

### 9.3 出了 9/22 型错误时的定层口径

1. **第一步永远先跑 `odom`**：领机"本机发出"在涨、跟随者"收到邻机"停 → 断在中间段（桥/链路）；
   两边都正常但任务仍报 `LEADER_LOST` → 跟随环自身（订阅名/超时/线程）。
2. 记下时刻，保留 `health/*.jsonl`、`*_follower_events.jsonl`、driver JSONL 与三份 bag。
3. 收工后按 rosbag 三份对齐（同一时刻）：领机 bag 里 `/mavros/local_position/odom`（发送侧）
   ↔ 跟随者 bag 里 `/UAV8/mavros/local_position/odom`（接收侧）↔ 哨兵话题
   （`mavros/state`/`time_reference`/`rc/in`/`gps_bias`）——三类证据同停=整链路断，只有 odom 停=
   该话题发布/桥接端问题。bag 话题清单与理由见 `src/swarm_uav_executor/bashs/record_rosbag.sh` 头注释。

### 9.4 收尾

```bash
for n in 8 10 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh stop $n; done   # 逐台
./src/tcp_to_ros/tools/log_fetch.sh 8 10 13        # 含 onboard/health/*.jsonl；**不采 bag**
# bag 手工取（体积大）：
for ip in 81 101 131; do
  rsync -rc --include='*.bag' --exclude='*' -e "sshpass -p nvidia ssh -o StrictHostKeyChecking=no" \
    nvidia@192.168.5.$ip:'~/catkin_swarm6-2/.ros_home/log/rosbag/' \
    "src/uav_log/$(date +%Y-%m-%d)_airflow/raw_$ip/"
done
```

与既有文档的关系：`no_prop_link_test.md`（地面/无桨 Stage 1/2）保留作地面预演；
`ground_flow.sh`（贴地彩排）保留作不飞行时的流程验证；本节是**空中**版，三者共用同一套
health/事件/JSONL 判读口径。
