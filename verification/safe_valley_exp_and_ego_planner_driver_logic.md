# safe_valley_exp 与 ego_planner_driver 运行逻辑框图

> ⚠️ **已归档 / 过时（2026-08-27）**：本文档基于早期（min-snap 重构前）源码梳理，多处与当前实现不符
> （如 `max_vel=8.0`、`swarm_clearance=2.0`、`setpoint` 为 PoseStamped、0.5Hz replan、`IDLE`/`POSE_STALE`
> 状态、无监督/候选/周期碰撞检查）。仅作历史参考保留。
> 当前权威：`src/ego_planner_driver/README.md`（状态机 + replan 策略单一权威）、
> `src/tcp_to_ros/plans/README.md`（当前状态）、`src/swarm_uav_executor/README.md`（executor 语义）。
> safe_valley_exp 的 flock 算法为 legacy 参考实现，不用于 Group A 正式任务链路。

> 本文档基于源码梳理两个功能包的程序运行逻辑：
>
> - **safe_valley_exp**：Python 实现的 Reynolds 集群控制（凝聚 / 对齐 / 分离），面向 `config/flock.yaml` 中的 4 机集群（UAV6 为 Leader，UAV7/9/10 为 Follower）。
> - **ego_planner_driver**：C++ 实现的 ego-swarm 风格带时轨迹生成与互惠机间避碰驱动（B-spline + L-BFGS 优化）。

---

## 1. 总体架构一览

```text
┌─────────────────────────────────────────────────────────────┐
│ 仿真层（1 个 ROS Master, 11300）                             │
│                                                             │
│  multi_uav_sim.launch                                       │
│  ├── Gazebo empty_world.launch                              │
│  └── PX4 SITL ×4  iris_0(ID0) iris_1(ID1) iris_2(ID2) iris_3(ID3)
│       （MAVLink TCP 4560..4563）                             │
└─────────────────────────────────────────────────────────────┘
                        │ MAVLink / UDP（各自独立 fcu_url 端口）
                        ▼
┌─────────────────────────────────────────────────────────────┐
│ 机载层 UAV6（ROS Master 11311）  ←MAVROS 与算法同机          │
│                                                             │
│  MAVROS (px4.launch) ──ROS topics── safe_valley_exp         │
│                                      ├─ wait_mavros.py      │
│                                      │     └─execv→ safe_flock_main.py
│                                      └─ swarm_topology_bridge（bridge_node.py）
│                                          │ ZMQ PUB/SUB（4200+offset）
│                                          ▼
│  （同拓扑下其他 UAV 机载层相互对连）                          │
└─────────────────────────────────────────────────────────────┘
```

> 说明：仿真层只跑 Gazebo + PX4 SITL；机载层每机一个独立 ROS Master，跑 MAVROS + `swarm_topology_bridge` + 算法包。跨机数据经 ZMQ 桥传递，并以 `/源机名/*` 前缀在本地按源机重发布。

---

## 2. safe_valley_exp 运行逻辑

### 2.1 launch 启动链路

```text
multi_uav_sim.launch                 # 仿真层（复制到 px4/launch/ 使用）
├── include → gazebo_ros/empty_world.launch              # 加载 Gazebo
└── include → px4/single_vehicle_spawn_xtd.launch ×4    # 加载无人机实例
    ├── group ns=iris_0 (ID=0, mavlink_tcp_port=4560)
    ├── group ns=iris_1 (ID=1, mavlink_tcp_port=4561)
    ├── group ns=iris_2 (ID=2, mavlink_tcp_port=4562)
    └── group ns=iris_3 (ID=3, mavlink_tcp_port=4563)

uav_offboard_sim.launch              # 机载层，每机一个（示例 UAV6 / tgt_system=1）
├── include → mavros/px4.launch                          # 加载 MAVROS
│     └── fcu_url=udp://:24540@localhost:34580            # 按 ID/tgt_system 端口偏移
└── include → safe_flock_sim.launch
    ├── node safe_flock: wait_mavros.py                  # 等待 MAVROS connected
    │     └── os.execv 替换进程 → safe_flock_main.py    # 算法主程序
    ├── node submode_publisher: submode_publisher.py     # 仿真模拟 RC 三档开关
    ├── rospackage: swarm_topology_bridge
    │     └── node swarm_bridge: bridge_node.py          # 加载 topology_sim_swarm.yaml
    └── node rosbag_record: rosbag_record.py             # 可选 rosbag 记录

uav_offboard_real.launch             # 实机，每机一个（hostname 自动识别 UAV6/7/9/10）
├── include → mavros/px4.launch                          # 串口连接飞控
└── include → safe_flock_real.launch
    ├── node safe_flock: wait_mavros.py
    │     └── os.execv 替换进程 → safe_flock_main.py
    ├── rospackage: swarm_topology_bridge
    │     └── node swarm_bridge: bridge_node.py          # 加载 topology.yaml（实机 IP）
    └── node rosbag_record: rosbag_record.py             # 可选 rosbag 记录
```

### 2.2 核心节点内部分层

```text
safe_flock_main.py（主入口 / 状态机, 30Hz）
└── 初始化
    ├── ① FlockConfig()            —— flock_config.py（身份识别 + 参数中心）
    │     ├── own_name 解析：构造函数参数 → ~own_name → hostname(UAV*)
    │     └── 加载 flock.yaml / obstacles.yaml
    │           ├── control / leader / topics / topology / init_position
    │           └── 动态推导 r_in_floc / r_out_floc / e_max / r_align
    ├── ② FlockMethod(cfg)         —— flock_method.py（算法库）
    │     ├── cohe_control    : 向 Leader 垂直投影凝聚
    │     ├── flock_control   : 匹配 Leader 水平速度 + 高度控制
    │     ├── align_control   : 邻居相对速度制动（刹车距离公式）
    │     ├── sepa_control    : 动态椭圆势场（避邻居 / 避圆柱障碍，共线侧向逃逸）
    │     └── apply_limits    : v_max 限速
    └── ③ FlockComm(cfg).sync_origin()  —— flock_comm.py（通信 + 时空一致性）
          ├── 发布器：
          │     ├── /mavros/setpoint_velocity/cmd_vel
          │     ├── /mavros/setpoint_position/local
          │     ├── /mavros/global_position/set_gp_origin
          │     └── /leader_fix_origin（仅 Leader）
          ├── 订阅器：
          │     ├── /mavros/state、/mavros/local_position/odom、/mavros/time_reference
          │     ├── /offb_submode
          │     ├── Leader 的 /mavros/rc/in（submode_channel 通道）
          │     └── 邻居（经 bridge 的 /UAVx/ 前缀）odom / state / time_ref
          └── sync_origin()：
                ├── Leader  ：等 GPS fix≥min_gps_status → set_gp_origin + 广播 leader_fix_origin
                └── Follower：收 leader_fix_origin → 同步 set_gp_origin（统一 ENU 原点）
              GPS 时钟校正：T_own = T_other - Bias_other + Bias_own

主循环（等待 origin_set 就绪后进入）
└── check_state_change() 记录模式切换，切换时重置 submode_start_pose
```

### 2.3 主循环状态机（OFFBOARD 判定 + submode 分发）

```text
主循环（rate = control_rate, 30Hz）
│
├── origin_set == False ? ───────────────►  sleep，继续等待原点同步
│
├── check_state_change()：模式变化时重置 submode_start_pose / start_time
│
└── own_state.mode == OFFBOARD ?
    │
    ├── 否 → execute_hover()          锁定进入时位姿，持续发布该位姿悬停
    │
    └── 是 → 按 offb_submode 分发
        │
        ├── "form" → execute_formation()
        │     ├── Leader  ：目标 = 进入时 XY + leader_height 高度
        │     └── Follower：目标 = Leader 当前位置 + form_offset
        │                   统一移动：远距离 vel_form 匀速插值 / 近距离直接发位置
        │                   → /mavros/setpoint_position/local
        │
        ├── "navi" → execute_navigation()
        │     ├── Leader  ：圆形轨迹
        │     │     └── start + get_leader_circle_position(circle_speed/radius)
        │     │         → /mavros/setpoint_position/local（位置指令）
        │     └── Follower：集群速度合成
        │           ├── v_cohe  = cohe_control(own, leader)
        │           ├── v_align = align_control(own, neighbors)
        │           ├── v_sepa  = sepa_control(own, neighbors, obstacles)
        │           ├── v_flock = flock_control(own, leader, leader_vel)
        │           ├── v = v_cohe + v_align + v_sepa + v_flock
        │           ├── limited_v = apply_limits(v, own_vel)   # v_max 限幅
        │           └── → /mavros/setpoint_velocity/cmd_vel（速度指令）
        │
        └── 其他 → execute_hover()     安全回退：数据未就绪时就地悬停
```

### 2.4 submode 来源（RC 通道映射 / 仿真模拟）

```text
实机 / 仿真共用：
  Leader 的 /mavros/rc/in（submode_channel 通道, 1-based → 0-based）
  └── 通道值判定
      ├── val < 1300          → "form"
      ├── 1400 ≤ val ≤ 1600   → "hover"
      └── val ≥ 1700          → "navi"
              │
              ▼
         更新 /offb_submode 话题

仅仿真：
  stdin 输入 form/hover/navi ──► submode_publisher.py（10Hz）──► /offb_submode

safe_flock_main.py 订阅 /offb_submode 并按 submode 分发（见 2.3）
```

---

## 3. ego_planner_driver 运行逻辑

### 3.1 节点接口与启动

```text
ego_planner_driver.launch
├── 全局身份参数（/uav_id、/exec_target，供 EgoSwarmDriver / bridge 发现）
└── node ego_planner_driver_node
    └── 私有规划参数：
        ├── ts=0.4  max_vel=8.0  max_acc=6.0
        ├── swarm_clearance=2.0  planning_range=30.0
        ├── lambda_smooth / lambda_swarm / lambda_feasibility
        └── setpoint_rate_hz=30  local_pose_timeout_s=1.0 ...

节点接口（公开命名空间 <ns>/，如 /UAV1/）：
├── 发布：
│   ├── exec_state（String, latched）IDLE/EXECUTING/COMPLETED/HOLD/...
│   ├── setpoint（PoseStamped, 30Hz）
│   └── trajectory_intent（UavTrajectoryIntent）
└── 订阅：
    ├── goal / waypoints / goal_yaw
    ├── local_pose
    ├── neighbor_intent
    └── hold

线程结构：
├── replan_thread（0.5Hz）  —— 快照输入 → 规划 → 发布轨迹/意图
└── setpoint 定时器（30Hz） —— 按轨迹时间采样发布 setpoint 位置
```

### 3.2 节点状态机与数据流

```text
初始化 ───────────────► 状态 = IDLE

触发输入：
├── /goal 或 /waypoints ──► 置 pending_goal / waypoints，planning_requested=true
├── /neighbor_intent ────► 按 uav_id + traj_id 去重（单调递增）
│                          存入 inbox（5s 内视为新鲜），标记请求规划
└── /hold ───────────────► hold_requested=true，planning_requested=false

replanLoop（每 0.5s）：
│
├── hold_requested?
│   └── 是 → 状态=HOLD
│         ├── 有 fresh local_pose → steady_target=最新位姿，持续悬停
│         └── 无 fresh pose     → steady_target 清空，停止发布 setpoint（fail-safe）
│
├── planning_requested && pending_goal && !steady_target?
│   └── 否 → 跳过本轮
│   └── 是 → local_pose 新鲜?
│         ├── 否 → 状态=POSE_STALE，跳过
│         └── 是 → 快照 start / waypoints / neighbors
│                   （neighbor.start_time 转为相对 now 起点）
│                   │
│                   ▼
│               core_->planOnce(...)
│                 ├── 失败 → 状态=EGO_PLAN_FAILED
│                 └── 成功 → 优化耗时偏移校正（traj.t += offset）
│                             │
│                             ▼
│                         状态=EXECUTING
│                         current_=新轨迹，plan_start_=now
│                         publishIntent() 广播 trajectory_intent

执行与到达判定：
├── 30Hz setpoint 定时器：按 elapsed 采样当前轨迹 → 发布 setpoint
└── elapsed ≥ duration 时 checkArrival：
    ├── 到达误差 ≤ pos_tol_，稳定 ≥ arrival_stable_s
    │     └── 状态=COMPLETED，steady_target=终点（恒发终点，等效 HOLD）
    ├── 误差超限但 elapsed > duration + timeout_margin
    │     └── 状态=EGO_EXEC_TIMEOUT，steady_target=终点
    └── 否则继续采样发布
```

### 3.3 数值核心 `EgoPlannerCore::planOnce`（无 ROS，B-spline + L-BFGS）

```text
输入：start + 稀疏 waypoints + neighbor 轨迹
└── point_set = [start] + waypoints（不足 5 点则中点细分）
    └── parameterizeToBspline → (K+6) 个控制点
        └── 固定首尾 order_(=3) 个控制点，variable_num_ = 3×(K-4)
            └── L-BFGS 优化（max_iterations 次，mem_size 记忆）
                ├── 代价1 平滑：jerk 最小化（三阶差分）
                ├── 代价2 动态可行性：超 max_vel / max_acc 平方惩罚
                ├── 代价3 互惠机间避碰：按邻居带时轨迹同步采样，
                │         椭球间距（水平轴1.0m/垂直轴2.0m）< clearance*2 → 惩罚
                └── 代价4 终端：末段控制点逼近目标
                    └── 收敛 → 0.1s 步长采样 TimedTrajectory
                        └── 锚定起点 / 终点 → 输出带时轨迹（t / pts / yaw）
```

---

## 4. 两个功能包之间的关联

```text
ego_planner_driver（任一架位，C++ 节点）
├── trajectory_intent（本机 /ns/trajectory_intent）
│     └───► swarm_topology_bridge（ZMQ 桥）
│               └── 重发布为 /源机名/trajectory_intent
│                     └───► neighbor_intent（邻机带时轨迹进入避碰优化）
├── setpoint（30Hz PoseStamped）──► MAVROS / PX4
└── local_pose ◄── MAVROS / PX4

safe_valley_exp（Python 节点）
├── 经 bridge 订阅邻居：
│   ├── /UAVx/mavros/local_position/odom       （邻居位姿/速度）
│   ├── /UAVx/mavros/time_reference            （GPS 时钟 bias 校正）
│   ├── /UAVx/mavros/rc/in                     （Leader RC 通道，同步切模式）
│   └── /UAVx/leader_fix_origin                （Follower 同步原点）
├── 经 bridge 发布（Leader）：/leader_fix_origin ──► 其余 UAV
├── cmd_vel / setpoint_position ──► MAVROS / PX4
└── state / odom / global ◄── MAVROS / PX4

swarm_topology_bridge（bridge_node.py，ZMQ）
├── 话题桥（PUB/SUB）：保留来源前缀 /源机名/*
├── Service 代理（DEALER/ROUTER）：跨机透明转发 ROS Service
└── 每话题限频（max_freq，首条不丢弃）
```

| 维度 | safe_valley_exp | ego_planner_driver |
|---|---|---|
| 语言 / 运行形态 | Python 3, rospy 节点 | C++14, roscpp 节点 |
| 控制本质 | Reynolds 三力（凝聚/对齐/分离）+ 椭圆势场避障，速度指令 | B-spline 带时轨迹 + L-BFGS 优化，位置指令（含时间语义） |
| 与飞控接口 | `/mavros/setpoint_velocity/cmd_vel`、`/mavros/setpoint_position/local` | `setpoint`（PoseStamped, 30Hz OFFBOARD 兼容） |
| 坐标基准 | `set_gp_origin` 全局原点同步（Leader 广播 `leader_fix_origin`） | `frame_id=map`，local_pose 输入 |
| 时间一致性 | GPS TimeReference bias 校正他机时间戳 | `stamp`（绝对秒）转相对时间轴，优化耗时偏移校正 |
| 集群协同信息 | 邻居 odom / state / time_ref / rc_in（经 bridge） | 邻居 `trajectory_intent`（带时轨迹，经 bridge） |
| 互惠避碰 | 动态椭圆势场（速度相关离心率） | 椭球间距惩罚项（水平 1.0m / 垂直 2.0m） |
| 运行状态机 | hover / form / navi（`/offb_submode` 分发） | IDLE / EXECUTING / COMPLETED / HOLD / POSE_STALE / EGO_PLAN_FAILED / EGO_EXEC_TIMEOUT |
| 安全回退 | 邻居数据缺失 → 就地进行悬停 | 无 fresh pose → 停止发布 setpoint（fail-safe）；HOLD 锁最新位姿 |

---

## 5. 关键 Topic 清单

### safe_valley_exp 涉及

| Topic | 方向 | 消息 | 用途 |
|---|---|---|---|
| `/mavros/state` | 出 | `mavros_msgs/State` | OFFBOARD 模式判定 |
| `/mavros/local_position/odom` | 出/入 | `nav_msgs/Odometry` | 本机位姿/速度，邻居数据 |
| `/mavros/time_reference` | 出/入 | `sensor_msgs/TimeReference` | GPS 时钟 bias 校正 |
| `/mavros/rc/in` | 出（Leader） | `mavros_msgs/RCIn` | submode 三档开关 |
| `/mavros/global_position/set_gp_origin` | 入 | `geographic_msgs/GeoPointStamped` | 统一 ENU 原点 |
| `/leader_fix_origin` | 出（Leader）/入（Follower） | `sensor_msgs/NavSatFix` | 广播 GPS 基准 |
| `/mavros/setpoint_position/local` | 入 | `geometry_msgs/PoseStamped` | form/hover 位置指令 |
| `/mavros/setpoint_velocity/cmd_vel` | 入 | `geometry_msgs/TwistStamped` | navi 速度指令 |
| `/offb_submode` | 双向 | `std_msgs/String` | hover/form/navi 分发 |

### ego_planner_driver 涉及（位于 `<namespace>/`，如 `/UAV1/`）

| Topic | 方向 | 消息 | 用途 |
|---|---|---|---|
| `goal` | 入 | `geometry_msgs/PointStamped` | 单点目标 |
| `waypoints` | 入 | `geometry_msgs/PolygonStamped` | 航线（P1 leader FOLLOW_ROUTE） |
| `local_pose` | 入 | `geometry_msgs/PoseStamped` | 本机位姿 |
| `neighbor_intent` | 入 | `swarm_uav_interfaces/UavTrajectoryIntent` | 邻机带时轨迹（互惠避碰） |
| `hold` | 入 | `std_msgs/Empty` | HOLD 指令 |
| `setpoint` | 出 | `geometry_msgs/PoseStamped` | 30Hz OFFBOARD 位置指令 |
| `trajectory_intent` | 出 | `swarm_uav_interfaces/UavTrajectoryIntent` | 本机带时轨迹广播 |
| `exec_state` | 出 | `std_msgs/String`（latched） | 生命周期状态 |

---

## 6. 总结：两条并行控制链

```text
链路 A：safe_valley_exp 集群控制链（反应式）
  wait_mavros 就绪
    └── sync_origin 统一坐标系（set_gp_origin / leader_fix_origin）
          └── 状态机 hover/form/navi（/offb_submode）
                └── 方法库速度合成（cohe + align + sepa + flock）
                      └── cmd_vel / setpoint_position ──► PX4

链路 B：ego_planner_driver 轨迹执行链（主动式）
  goal / waypoints 输入
    └── B-spline 参数化
          └── L-BFGS 优化（平滑 + 动态可行 + 互惠避碰）
                ├── 带时轨迹采样 + trajectory_intent 广播 ──► 邻居（经 bridge）
                ├── neighbor_intent（邻机意图）──► 优化
                └── 30Hz setpoint 采样 ──► PX4
```

- **safe_valley_exp** 是“反应式”集群控制：不预先规划轨迹，每周期依据邻居状态合成速度指令。
- **ego_planner_driver** 是“主动式”轨迹规划：规划出带时轨迹并持续重规划（0.5Hz），通过 `trajectory_intent` 广播意图供邻居互惠避碰。
- 两者共用 `swarm_topology_bridge` 做跨 ROS Master 通信，并共用 MAVROS 与 PX4 交互；坐标系统一依赖 Leader 的 `set_gp_origin` / `leader_fix_origin` 同步。