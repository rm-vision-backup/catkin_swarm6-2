# 编队实机测试：准备编队 + 侦查（4 机：A08 领机 + A10 / A12 / A13）

本目录是 `src/swarm_uav_interfaces/real_setup.md` §7.1 已批准的**编队工程测试**落地件。
当前参加机 = **A08/A10/A12/A13（执行层 UAV8/UAV10/UAV12/UAV13）**，按 **5 m 间隔 2×2 方阵**
只跑两个阶段 —— **准备编队（MOVE_TO）+ 侦查（FOLLOW_ROUTE）**，与 12/15 机联合验收完全分离。

> 席位变更史：原 4 机线为 **A06(领机)/A08/A10/A12**；6 号机炸机后由 **A13 顶替其角位**；
> 12 号机 GPS 一度故障（**RTK 模块坏，已修复**）期间曾用 3 机版（A08 领机 + A10/A13）；
> **2026-09-23 恢复 4 机：A08 领机 + A10/A12/A13**。
> 目录名/文件名仍沿用 `four_uav_*`（最小改动），语义以本文件为准。

| 文件 | 作用 |
|---|---|
| `driver.py` | 任务下发脚本（实机用）：两阶段串行发 TCP 命令；整批超时/失败/Ctrl-C 自动发 HOLD；`--dry-run` 只打印线上 JSON；`--log` 写 JSONL 证据 |
| `package/` | 任务数据（robots / bootstrap / world / plans）；`prepare-a` + `coverage-segment-1`；leader = A08 |
| `sitl/` | **仿真**彩排（`four_uav_spawn.launch` / `gcs_sim_four_uav.launch` / `run_sim_four_uav.sh`） |
| `no_prop_link_test.md` | **无桨**（不装桨、不 arm）链路 + 跟随环 dry-run 验证规程；门禁 0–8 照用，门禁 9/10 不执行 |

地面站侧：`src/tcp_to_ros/launch/gcs_a_real_four_uav.launch`
（加载 `config/formation_offsets_four_uav.yaml` + `config/calibration_four_uav.yaml`；
身份映射 / topology / 端口 / **安全门禁**全部沿用共享配置，未做任何修改）。

**现场怎么用**：先过 §零 门禁（逐条指向下面小节），再 §一 → §二 → §三 → §四 顺序执行；
出问题查 §五，收尾与取证见 §六。脚本侧两个阶段预算是 prepare 150 s / coverage 300 s
（`driver.py:75-76`），机载侧 `uav_task_timeout_s` 另算，勿混。

---

## 零、起飞前门禁（**全绿才装桨**）

| # | 门禁 | 判据 | 详见 |
|---|---|---|---|
| 0 | 空域净空 | 相对 A08 机位：**右 ≥20 m、前 ≥35 m、左 ≥10 m、后 ≥5 m、上 ≥15 m**；无人员/建筑/其它航空器 | §1.1 |
| 1 | 地面摆位 | 4 机 **2×2、5 m 间隔**（见 §1.2），机头同向（差 ≤30°），摆好**不再挪动** | §1.2 |
| 2 | 身份 + 板钟 | 每台 `hostname -s` = UAVn；`date +%s` ≈ 1.79e9（不是就先修钟） | §1.3 |
| 3 | 上位机预检 | `driver.py --phases prepare coverage --dry-run` 打印两条 COMMAND | §1.4 |
| 4 | 四机起栈 | 逐台 `onboard_ctl.sh status <n>`，四台都 UP | §二 |
| 5 | 邻机订阅 | 每台 `rosnode info /uav_executor_UAVn` 里有 **3 条** `/UAV…/trajectory_intent` | §二 |
| 6 | 地面站 | launch 无 `ValueError`；`formation_offsets_file` 指向本测试专用那份 | §三 |
| 7 | 标定 | 四机**全部 disarm**、GPS `fix_type ≥ 3`、`/group_a/calibration_ready` = True（**必须先于 arm**） | §四 |
| 8 | 现场项（非本工作区） | PX4 `COM_RCL_EXCEPT=4` / `NAV_RCL_ACT=0` 已按现场 RC 情况复核；遥控器在手，模式开关与应急开关已试过 | `real_setup.md` §2 |
| 9 | 装桨后 | 逐机遥控 arm + OFFBOARD，`/mavros/state` = `armed: True` / `mode: OFFBOARD`；**飞机会自动垂直到 5 m**，人员已撤离 | §4.2 |
| 10 | 下发任务 | **四机都已 armed + OFFBOARD** 才算满足 `prepare()` 门禁；提前发会被 `VEHICLE_NOT_ARMED` 整批拒 | §4.3 |

## 一、现场准备

### 1.1 空域净空（4 机版）

任务系 x=右、y=前（都按机头方向；原点 = 四机全球位置中位数 = **方阵中心**，见 §1.2）。
领机航线 x∈[-2.5, 17.5]、y∈[-2.5, 21.5]；跟随者 offset 为
A10(-5,+5) / A12(0,+5) / A13(-5,0)（`formation_offsets_four_uav.yaml` 的 `formation_4uav`），
所以**全队**包络比领机航线大一圈：

| 方向（相对 A08 机位） | 半径 | 怎么来的 |
|---|---|---|
| 右 | **20 m** | 队形最右 15（航线最右 17.5 − A08 位 2.5），领机右侧无跟随者 → 留 5 m 余量 |
| 前 | **35 m** | 队形最前 29（航线最前 21.5 + A10/A12 前偏 5 + A08 自身在原点前 2.5）→ 取 35 |
| 左 | **10 m** | 队形最左 −10（航线最左 −2.5 − A10/A13 左偏 5 − A08 位 2.5）→ 取 10 |
| 后 | 5 m | 航线最后 −2.5 + A08 自身在原点前 2.5 = 0，留余量 |
| 上 | **15 m** | prepare 阶段升到 15 m 层（`layer_move_to=15.0`，`ego_swarm.py:112`） |

单机操作单给的"右 17.5 / 前 21.5"是**单机航线包络**，编队不够用，别照抄。

### 1.2 地面摆位（决定 prepare 阶段飞多远）

标定冻结的原点 = **四机全球位置的中位数**（逐轴取中位）= 方阵的**几何中心**。

1. 四机按 **2×2、5 m 间隔**摆成方阵（相邻 5 m；对角 7.07 m）。
2. **四机机头大致同向**（航向差 ≤ 30°，否则标定不出 ready）。
3. 按机头方向定义前/右（物理方阵，以**原 A06 角位**为 (0,0) 参考）：
   - **A13 = (0,0)**（原 A06 的角位，方阵的"原点角"）
   - **A08 = (+5, 0)**（A13 右侧 5 m），**领机**；§1.1 的包络以它为准
   - **A10 = (0, +5)**（A13 正前 5 m）
   - **A12 = (+5, +5)**（A10 右侧 5 m / A08 正前 5 m）
4. 摆好后不再挪动 → 标定 → 再 arm。

摆好后任务坐标里四机就是
`A08(+2.5,-2.5) / A10(-2.5,+2.5) / A12(+2.5,+2.5) / A13(-2.5,-2.5)`，
与 `package/plans/plans.json` 的 prepare 目标**完全一致 —— 水平位移≈0**。
摆得不够准只会让 prepare 多飞几米（远小于 150 s 阶段超时），不致命。

### 1.3 上电、身份与板钟

```bash
for ip in 81 101 121 131; do
  ssh nvidia@192.168.5.$ip 'hostname -s; date +%s'
done
# 期望：UAV8/10/12/13 与 ~1.79e9
```

板钟停在 1970 就先修（**必须在起栈之前**，否则本轮 ROS 日志时间戳全废、只能靠
ulog 的 `LOG_ENTRY.time_utc` 锚定）：

```bash
./src/tcp_to_ros/tools/onboard_clock_fix.sh 8 10 12 13
```

### 1.4 上位机预检（不连地面站，秒级）

```bash
cd "$HOME/catkin_swarm6-2"
python3 verification/four_uav_recon/driver.py \
  --package verification/four_uav_recon/package \
  --mission-id four-uav-recon-001 --phases prepare coverage --dry-run
```

- [ ] 打印**两条** COMMAND：① `MOVE_TO`（`A08(+2.5,-2.5)` / `A10(-2.5,+2.5)` /
  `A12(+2.5,+2.5)` / `A13(-2.5,-2.5)`，z=12）
  ② `FOLLOW_ROUTE`（A08 六个航点；A10/A12/A13 为 `formation_follow`）
- [ ] 报错就别往下走（多半是包路径或 `--mission-id` 对不上）

## 二、机载启动（每台各一条；**参数逐台不同**）

两条硬要求：

- `neighbor_intents` **没有默认回退**（launch 默认空串，`ego_swarm.py:194` 不订阅任何
  邻机意图）→ 规划层拿不到邻机轨迹，只剩执行层 1 m 门禁。
- `uav_task_timeout_s` 不传就是 200 s，而侦查航线约 84 m（实测偏慢），
  会在航程未走完时判 `DRIVER_TIMEOUT` → 必须传 **280**。

ssh 用户是 **`nvidia`**（`src/tcp_to_ros/tools/configs/onboard.env` 的 `BOARD_SSH_USER`），
板 IP = `192.168.5.(10n+1)`。

**起之前先确认四台都没在跑**（分离式启动不会替你挡重复启动，起两套会抢 master）：

```bash
for n in 8 10 12 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh status $n | sed -n '3p'; done  # 期望四行 DOWN
for n in 8 10 12 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh stop $n; done                # 有残留就先停干净
```

下面四条**逐条复制**，只能改机号，不要合并成一条（邻居清单每台不同）：

```bash
# ── UAV8 / A08（领机）  邻居 = 10 12 13 ──
ssh nvidia@192.168.5.81 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV10/trajectory_intent,/UAV12/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV10/mavros/local_position/odom,/UAV12/mavros/local_position/odom,/UAV13/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV10 / A10  邻居 = 8 12 13 ──
ssh nvidia@192.168.5.101 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV12/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV8/mavros/local_position/odom,/UAV12/mavros/local_position/odom,/UAV13/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV12 / A12  邻居 = 8 10 13 ──
ssh nvidia@192.168.5.121 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV10/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV8/mavros/local_position/odom,/UAV10/mavros/local_position/odom,/UAV13/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV13 / A13  邻居 = 8 10 12 ──
ssh nvidia@192.168.5.131 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV10/trajectory_intent,/UAV12/trajectory_intent \
  neighbor_odom_topics:=/UAV8/mavros/local_position/odom,/UAV10/mavros/local_position/odom,/UAV12/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'
```

| 机 | 执行层 | IP | `neighbor_intents` 里的邻机 |
|---|---|---|---|
| A08（领机） | UAV8 | 192.168.5.81 | 10 / 12 / 13 |
| A10 | UAV10 | 192.168.5.101 | 8 / 12 / 13 |
| A12 | UAV12 | 192.168.5.121 | 8 / 10 / 13 |
| A13 | UAV13 | 192.168.5.131 | 8 / 10 / 12 |

上面用的是**分离式**启动（`setsid` + `&`，与 `onboard_ctl.sh start` 内部同一机制）：
ssh 立刻返回，断开终端不影响机载；终端转录仍逐字落在板上
`~/catkin_swarm6-2/.tmp/logs/UAVn_offboard_ego.log`。
想看实时输出用 `./src/tcp_to_ros/tools/onboard_ctl.sh logs <n>`（逐台；`-f` 跟读）。
若更习惯单机操作单那种**前台**写法，把 `setsid … </dev/null >/dev/null 2>&1 &` 去掉即可，
代价是那条 ssh 会一直占着（四台就要四个终端/tmux 窗口）。

**不用传的参数**（都有正确默认，传了反而多一处出错点）：

- `formation_offsets`：GCS 随任务逐跟随者下发（`gcs_a_backend_node.py:1237-1247`），
  只有 GCS 下发全零时机载才回退本机配置（`ego_swarm.py:498-504`）。
- `leader_odom_topic_prefix`：留空时按 `leader_id` 自动拼 `/UAV8/mavros/local_position/odom`
  （`ego_swarm.py:293-300`）。
- `uav_name`：取本机 `hostname`，写错反而启动失败。
- `neighbor_odom_topics` 其实也有默认（留空 = 自动订阅 UAV1–15 除自己，`ego_swarm.py:680-683`），
  上面显式传三台是为了少订阅、少踩带宽。

**不要**用 `onboard_ctl.sh start 8 10 12 13 -- <参数>` 扇出：它把同一串参数发给所有目标，
而邻居清单每台不同。另外该工具**多目标只处理第一个**（§八 风险 1），所以
`status` / `stop` / `logs` 也一律逐台调用。

### 起栈后核对（§零 门禁 4、5）

```bash
# 4) 四台都在跑（必须逐台调用，原因见 §八 风险 1）
for n in 8 10 12 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh status $n | sed -n '3p'; done
# 期望四行都是 UP(1)

# 5) 逐台查邻机订阅（把 8 换成各机号）
ssh nvidia@192.168.5.81 'source /opt/ros/noetic/setup.bash; source ~/catkin_swarm6-2/devel/setup.bash; \
  rosnode info /uav_executor_UAV8 | sed -n "/^Subscriptions/,/^Publishing/p" \
  | grep -E "trajectory_intent|local_position/odom"'
```

- 期望看到 **6 行**：`/UAV10` `/UAV12` `/UAV13` 各一条 `trajectory_intent` 和一条
  `mavros/local_position/odom`（`/UAV8` 自己不该出现）。
- **不要**用 `rosnode info … | grep -c` 计数：每个话题在 `Subscriptions` 与
  `Connections` 各出现一次，计数会翻倍，看着像"多订阅了/少订阅了"。
- `trajectory_intent` 一条都没有 = 没传邻居清单，必须停掉重起
  （逐台 `onboard_ctl.sh stop <n>`），别指望仿真彩排能替你测出来（见 §七）。

## 三、地面站启动（本机 = GCS_A，192.168.5.241，hostname 须为 GCS_A）

```bash
roslaunch src/tcp_to_ros/launch/gcs_a_real_four_uav.launch
```

- [ ] 启动日志无 `ValueError`（有的话多半是 `calibration_four_uav.yaml` 写了 `A08`
  而不是 `UAV8` —— 标定集合必须写执行层 `UAVn`）
- [ ] 该 launch 加载的是 `config/formation_offsets_four_uav.yaml` + `config/calibration_four_uav.yaml`，
  且 calibration 段打印的是四机集合（类似 `calibration set=UAV8,UAV10,UAV12,UAV13`）
- [ ] 防"静默回退"（见 §八 风险 3）：
  ```bash
  rosparam get /gcs_a_backend_node/formation_offsets_file   # 必须是 four_uav 那份
  ```
- [ ] 本机 TCP 服务在听：`nc -z 192.168.5.241 39001 && echo "tcp OK"`

## 四、跑一轮

### 4.1 标定（**硬顺序：先于 arm**）

```bash
rostopic echo -n1 /group_a/calibration_ready     # 期望 data: True
```

ready 是 **ALL 语义**（`gcs_a_calibration_node.py:113-117, 166-167`）：四架必须同时满足

1. `mavros/state` 新鲜且 **connected + disarm**（先 arm 就永远 ready 不了，任务会被
   `CALIBRATION_NOT_READY` 拒）；
2. `global_position/global` 与 `local_position/pose` 在 2 s 窗口内各有 ≥5 个样本
   （等效 ≥2.5 Hz，`min_samples_per_uav=5` / `sample_window_s=2.0`）；
3. 各机 yaw 与冻结参考航向差 ≤30°；
4. 四架都收到原点并回读 `gp_origin_confirmed`。

**冻结前逐台人工看一眼 GPS 质量**（重要，脚本不查这个）：

```bash
# 把 8 换成各机号；期望 fix_type ≥ 3（RTK fixed 是 6），satellites_visible 稳定
ssh nvidia@192.168.5.81 'source /opt/ros/noetic/setup.bash; source ~/catkin_swarm6-2/devel/setup.bash; \
  rostopic echo -n1 /mavros/gpsstatus/gps1/raw | grep -E "fix_type|satellites_visible"'
```

不要用 `/mavros/global_position/global` 的 `status` 字段判 fix —— mavros 1.20.1 实测在
`fix_type=6`、35 星时它仍报 0（NO_FIX）；GCS 侧 `on_global` 也只校验经纬高**有限**
（`gcs_a_calibration_node.py:80-85`），**没收敛但数值有限**的 GPS 照样入池，
会把冻结原点整体拉偏，四机中位最多偏约半个偏差量。

- [ ] 记录 `calibration_ready` 时刻；原点与航向：
  ```bash
  rostopic echo -n1 /group_a/gp_origin          # GeoPointStamped
  rostopic echo -n1 /group_a/reference_heading  # Float64（rad）
  ```
  地面站日志里也有 `calibration frozen from … origin=(…) heading=…`（单机参考值见 §八 风险 5）。

### 4.2 装桨 + 遥控 arm/OFFBOARD（逐机）

- [ ] 装桨前：人员撤到 §1.1 的净空之外、桨叶周围清空、电量充足、遥控器在手
- [ ] 遥控逐机 arm → 切 OFFBOARD，核对：
  ```bash
  ssh nvidia@192.168.5.81 'source ~/catkin_swarm6-2/devel/setup.bash && rostopic echo -n1 /mavros/state'
  # 期望 armed: True / mode: "OFFBOARD"
  ```
- [ ] **看到自动垂直爬升到 5 m**（`takeoff_height_m=5.0`，`src/ego_planner_driver/config/planner_defaults.yaml:41`）：
  约 5 s，只上升不水平移动 —— 这是设计行为，装桨前必须跟现场讲清
- [ ] 从此刻起遥控器始终在手：模式开关随时可切出 OFFBOARD 接管

### 4.3 下发任务（**四机都 armed + OFFBOARD 之后再发**）

```bash
python3 verification/four_uav_recon/driver.py \
  --package verification/four_uav_recon/package \
  --tcp-host 192.168.5.241 --tcp-port 39001 \
  --mission-id four-uav-recon-001 \
  --log verification/four_uav_recon/logs/real_$(date +%Y%m%d_%H%M%S).jsonl
```

**提前下发会被当场拒掉，不是排队等 arm**：机载 `prepare()` 明确要求"已 arm 且已在
OFFBOARD"（`ego_swarm.py:648-669`），未 arm 回 `VEHICLE_NOT_ARMED`、没切模式回
`VEHICLE_NOT_OFFBOARD`（MOVE_TO 还要求原点已确认）→ 该机 `FAILED` → 本任务
`completion_policy: ALL` → 整批 FAILURE、driver 退出码 1 并自动发 HOLD。飞机不会有任何
水平动作（不发任务相关 setpoint，规划器维持默认 HOLD 流）。
**重试必须换 `--mission-id`**：任务记录按 `(mission_id, command_id, uav_id)` 存 1 h
（`validation.py:222-223`、`executor_defaults.yaml:9`），同号同内容再发会命中缓存、
直接回上次那条 FAILED（`executor.py:24-30`），看起来像"第二次也秒失败"。

### 4.4 预期飞行过程（对照检查）

| # | 动作 | 高度 | 完成标志 |
|---|---|---|---|
| ① | 切 OFFBOARD 后自动起飞 | 0 → 5 m | 垂直上升约 5 s |
| ② | 准备编队（MOVE_TO） | 升到 15 m 层 → 水平微调 → 降到 12 m | `[prepare] SUCCESS`；四机就位于 2×2 方阵（位移≈0） |
| ③ | 侦查（FOLLOW_ROUTE） | 保持 12 m | A08 绕 6 航点（20 m × 24 m 矩形往返，约 84 m）；A10/A12/A13 跟踪领机 + offset（A10 左 5 前 5 / A12 前 5 / A13 左 5）；`[coverage] SUCCESS` |
| ④ | 结束 | 12 m 悬停 | driver 退出码 0 |

航线阶段开始后，跟随者才会建领机 odom 订阅（收到 `formation_follow` 目标才建，
`ego_swarm.py:295-305`）—— 这时可再核一次：

```bash
ssh nvidia@192.168.5.101 'source ~/catkin_swarm6-2/devel/setup.bash; \
  rosnode info /uav_executor_UAV10 | grep "UAV8/mavros/local_position/odom"'
```

## 五、异常处置（按优先级）

| 现象 | 动作 |
|---|---|
| 姿态/航迹异常，或靠近人员 | **遥控切模式接管**（第一优先，别等软件） |
| 任务卡住、想中止 | driver 终端 **Ctrl-C** → driver 自动发 HOLD（`driver.py:318-320`），飞机原地悬停 |
| 出现 `MIN_DISTANCE_BREACH` / `EGO_PLAN_FAILED` | 任务进安全终态（HOLD），按现场规程遥控接管 |
| driver 报整批超时（`TIMEOUT`）/ 整批失败 | 同上；driver 已自动发 HOLD（`:302` / `:336`），再用遥控收尾 |
| 某台 driver 报 `DRIVER_TIMEOUT` | 该台在 `uav_task_timeout_s:=280` 内没到终态 → 同上看是哪台 `uav_task_state` 没进终态 |
| 单台链路/图传异常 | 遥控接管后按现场规程处置；**不要**就地重启机载栈或只重启子集 |
| 标定一直不 ready | 见 §4.1 四条前提，逐条查（最常见：某台未 disarm 或 GPS 未收敛） |
| 下发后秒失败、`error_message` 为 `VEHICLE_NOT_ARMED` / `VEHICLE_NOT_OFFBOARD` | 有机器还没 arm 或没切 OFFBOARD（`prepare()` 门禁，`ego_swarm.py:664-669`）。**换新的 `--mission-id` 再发**——同号同内容会命中 1 h 缓存直接回 FAILED（§4.3） |

## 六、观测点、收尾与成功判据

**成功判据**

- [ ] driver 退出码 **0**；`[prepare] SUCCESS` + `[coverage] SUCCESS`
- [ ] GCS 侧 **4/4 COMPLETED**
- [ ] 全程 `MIN_DISTANCE_BREACH` **0 次**、`EGO_PLAN_FAILED` **0 次**
- [ ] 跟机与领机保持 ≈5 m（A10 左 5 前 5 / A12 前 5 / A13 左 5；两两间距 ≥5 m、对角 7.07）

**观测**

- driver 的 JSONL（`mission_start` / `phase_result` / `command_feedback`，含各机
  `ACCEPTED`/`RUNNING`/终态与 `error_message`）
- GCS `gcs_a_backend_node` 的 rosout（`GCS_A goal ...`、`GCS_A schedule ... tau {...}`）
- 话题：`/UAVn/uav_task_state`、`/group_a/group_task_state`
- 机载 `~/.tmp/logs/UAVn_offboard_ego.log`（关注 `MIN_DISTANCE_BREACH`、`BRAKE_HOLD`、
  `EGO_PLAN_FAILED`）

**收尾与取证**

```bash
for n in 8 10 12 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh stop $n; done    # 或现场规程遥控接管后逐台停
for n in 8 10 12 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh status $n | sed -n '3p'; done  # 期望四行 DOWN
./src/tcp_to_ros/tools/log_fetch.sh 8 10 12 13         # 机载日志 + 飞控 ulog
```

- [ ] 按现场规程**遥控接管**降落/返航（不要就地重启机载栈）
- [ ] 机载栈停干净、端口释放（对照 `single_uav_checklist_UAV6.md` §6）
- [ ] `mavlink-routerd` **不要停**（现场常驻件，起停我们栈都不碰它）

## 七、仿真彩排（改完配置先跑这个）

```bash
bash verification/four_uav_recon/sitl/run_sim_four_uav.sh
```

自动完成：4 机 SITL → 机载（8 10 12 13，自动补邻机话题 + `EGO_TASK_TIMEOUT_S=280`）→
四机 GCS + 标定 → takeoff → `driver.py`（`--tcp-host 127.0.0.1`）。
日志在 `.ros_home/log/headless_test/`（`04_gcs.log`、`06_takeoff.log`、`07_task.jsonl`），
退出码 0 = 两阶段 SUCCESS。

> 注意：仿真彩排由 `startup_offboard_ego.sh` **自动**填邻机话题，验证不到 §二 的手工参数。
> §二 那套命令的机制（分离式启动、逐台邻机订阅）已于 2026-09-22 在实机验证过
> （当时是四机、各 9 节点、邻机意图与 odom 各 3 条订阅都对）。
> 中间换成 3 机时三条命令与邻居清单都改过，**现已按四机恢复**；
> 正式首飞前按 §二 末尾那条逐台复核一次。

## 八、已知风险

1. **`onboard_ctl.sh` 多目标只处理第一个**（2026-09-22 实测）：`status 8 10 12 13` 只输出
   UAV8，`stop 8 10 12 13` 只会停 UAV8。根因是 `done <<<"$TARGET_LINES"` 循环里的 `ssh`
   吃掉了 here-string 的剩余行（`ssh_opts` 里没有 `-n`，`onboard_ctl.sh:283-295`）。
   本文件所有 `onboard_ctl.sh` 命令因此都写成**逐台调用**；`stop` 若漏了机，飞机上会
   留着 ROS 栈继续发 setpoint。
2. **跟随者完成判据死区**：跟随者要求自己距 `leader+offset` 在 0.2 m 内**且领机距其最终
   航点在 0.2 m 内**（`ego_swarm.py:581-585`），而领机自己的完成判据是
   `arrival_reach_thresh_m=0.5`（`src/ego_planner_driver/config/planner_defaults.yaml:32`）。
   存在最多 0.3 m 窗口：领机报 COMPLETED，跟随者永远满足不了 → 超时 → `ALL` 策略下整批
   FAILURE。若命中且飞行本身干净，属判据死区而非飞控问题。
3. **`_load_formation_offsets` 出错会静默回退 `leader_id="A01"`**
   （`gcs_a_backend_node.py:406-408`）→ 表现为 `INVALID_FORMATION_LEADER`。启动后按 §三 核对
   `formation_offsets_file`。
4. **标定是 ALL 语义**：任一架未 disarm / GPS 未收敛 / 未回读确认，全队都不 ready。
   2026-09-22 现场实例：12 号机当时 GPS `fix_type=0`（仅 3–10 星、`eph` 169 m）而另三台
   `fix_type=6`（RTK fixed、41–44 星）→ 一直不会 ready，光靠等没用。
   **该机已修复**（根因是 **RTK 模块坏**，不是 GPS 本身）；换机/换模块后同样先查那一台的
   天线/遮挡，并且**换机体时飞控侧 `MAV_SYS_ID` 必须等于该机号**，否则 mavros 一直
   `connected: False`。相关经过见 §十。
5. **实测参考（2026-09-21 单机 UAV6 实飞）**：标定在节点起来 2.4 s 后冻结
   （`origin=(28.219920800, 112.866877800, 53.630)`、`heading=-1.754 rad`）→ §4.1 那个
   2 s 窗口 ≥5 样本的门槛在 RTK 实机上能过。编队是同一门槛，但**逐机判定**。

## 九、机载 `mavlink-routerd`（现场常驻件）

**2026-10-09 起：router 由 systemd 管理（开机自启，不再依赖桌面/终端）。**

```bash
# 判活 / 查看（逐台；判活用 pgrep 或 systemctl，不要用 ss -lunp | grep 15500
# ——它的 UDP 是 connect 出去的 socket，不处于 LISTEN）
systemctl is-active mavlink-routerd      # 期望 active
pgrep -x mavlink-routerd                 # 期望 1 个进程
sudo journalctl -u mavlink-routerd -b    # 本次开机的启动记录（四条 Opened = UART + 3 个 UDP + TCP 5760）
# 手动起停（一般不需要；服务 enabled，开机自动起）
sudo systemctl restart mavlink-routerd
```

- unit：`tools/systemd/mavlink-routerd.service`（tcp_to_ros 仓）；实装/刷新：
  `bash src/tcp_to_ros/tools/router_service_install.sh 8 10 13`（从 GCS 逐台，幂等；
  前置要求机载栈没在跑）。
- 参数与旧脚本逐项一致：`-e 192.168.5.241:15500 -e 192.168.5.242:15500 -e 127.0.0.1:15500
  /dev/ttyTHS0:921600`；以 root 运行，**不再需要 `chmod 777 /dev/ttyTHS0`**。
- 旧的桌面自启（`~/.config/autostart/gnome-terminal.desktop` → `Mavlink-route.sh`）已**禁用**
  （`X-GNOME-Autostart-enabled=false` + `Hidden=true`）；脚本文件保留在板上备查。
- 2026-10-09 已在 8/10/13 上实装并**逐台重启验证**：三台均在 boot 时自动起来、端点全 Open、
  单实例、板钟（RTC）正常。

**仍然成立的两条（与启动方式无关）**

1. **顺序：router 必须先于机载栈**。若 mavros 先起（或 router 中途被重启），`mavros → 飞控`
   的**定向**消息通道会坏掉——现象极具迷惑性：遥测正常、`connected: True`、`mavros/state`
   看着全好，但**参数读不出、命令无 ACK、原点永远不 confirmed**（广播类如 TIMESYNC 正常）。
   判定：`rosservice call /mavros/param/get '{param_id: "MAV_SYS_ID"}'` 返回 `success: False`。
   2026-10-09 实测：**重启机载栈可能无效，重启 router（`systemctl restart mavlink-routerd`）
   才恢复**——即 router 自身也会进"遥测通、定向死"的状态。
2. **不要加 `-r`**（`--report_msg_statistics`，每 5 s 打印 10–20 KB）：stdout 没人读时会阻塞
   打印、**串口 TX 被饿死**（UAV6/8 旧实例）。当前 unit 未带 `-r`，保持即可。

**历史（为什么不再用桌面自启）**：旧方案脚本一退出，gnome-terminal 标签页关闭 →
systemd 的 `vte-spawn-*.scope` 被撤销 → cgroup 把 router 一起杀掉（`setsid` 只换会话、
换不掉 cgroup），表现为"开机后 router 根本没起来"；判据是
`journalctl -b | grep -E "vte-spawn|chmod 777"` 出现 `chmod 777` 后同秒 `scope: Succeeded`。

## 十、席位变更与 UAV11 本体故障记录（历史）

**6 号机炸机**后曾用 **UAV11 / A11** 顶替领机席位，代码部署（rsync + `catkin build`）、
板 hostname、飞控 `MAV_SYS_ID=11`、mavros `tgt_system=11`、`Mavlink-route.sh` 都已就绪，
但随后确认它**"机载 → 飞控"方向物理不通**，与软件无关。

**判据（都绕开 mavros / 我们的栈）**

| 测试 | 对照机 | UAV11 |
|---|---|---|
| 串口排空（O_NONBLOCK 灌 200 KB） | 81.2 KB/s 正常排空 ✓ | **81.2 KB/s 正常排空 ✓**（字节确实上线了） |
| 外部往返：GCS → 该机 router → 串口 → 飞控 → 回读 `PARAM_VALUE` | **有回复** ✓ | **无回复** ✗ |
| 直写串口改 `LOCAL_POSITION_NED` 流速（可观测效应） | 有效 ✓ | **无效** ✗ |
| 遥测流速（LOCAL_POSITION_NED） | 30 Hz（= mavros 请求值） | **1.0 Hz（飞控默认值 ⇒ 从未收到请求）** ✗ |
| termios / 设备树 / 端口持有者 | — | 与对照机 **逐位一致** ✓ |

**结论**：飞控 → 机载（RX）正常；机载 → 飞控方向的字节虽已由 UART 发出（排空正常），
但飞控侧毫无反应 ⇒ 断点在 **"机载 TX 引脚 → 飞控 RX"** 这一段。

**✅ 2026-09-23 已处理**：现场确认为**飞控与电脑之间的排线问题，已更换排线**（复验命令见下）。
**本轮四机阵容仍用 A08/A10/A12/A13**，A11 作为备机（复验通过后可再评估是否换回）。

```bash
# 复验（必须 success: True，值 11；同时 local_position/pose 应回到 ~30 Hz、
# time_reference 应 1 Hz）
ssh nvidia@192.168.5.111 "source ~/catkin_swarm6-2/devel/setup.bash; \
  rosservice call /mavros/param/get '{param_id: \"MAV_SYS_ID\"}'"
```

> 席位变更史：原 4 机线 **A06/A08/A10/A12** → 6 号炸机由 **A11** 顶替领机 →
> 11 号机本体故障 → 改 3 机（**A08 领机 + A10/A13**）→ 12 号机 GPS（RTK）修复 →
> **2026-09-23 恢复 4 机：A08 领机 + A10/A12/A13**（即本文件当前状态）。
