# 真机 15 机 Group A 部署方案

## 总体目标

保留现有 SITL 文件不变，新增一套真机专用启动链路：

```text
每架无人机上电
  → systemd 启动本机 ROS/MAVROS/executor/ego/bridge
  → 保持 disarm、HOLD，不自动起飞

GCS_A
  → 启动 bridge/backend/TCP/标定
  → 通过 bridge 按 UAV 编号列表下发起飞请求

无人机本机
  → 收到起飞请求
  → 本机调用 MAVROS：HOLD → arm → OFFBOARD
  → ego_planner_driver 进入 TAKEOFF
  → 起飞完成后回到 HOLD

GCS_A
  → runtime 发送 MOVE_TO、FOLLOW_ROUTE、FAULT_EXIT、HOVER 等任务
```

部署假设：

- 分布式部署：每架无人机有自己的机载电脑；
- 每架机本地 ROS Master 使用 `11311`；
- GCS_A 使用 `192.168.5.10`；
- UAV1–UAV15 使用 `192.168.5.11`、`.21`、`.31` … `.151`；
- 每架飞控通过 `/dev/ttyTSH0:921600` 连接；
- 上电只启动节点，不自动 arm、不自动切 OFFBOARD；
- 起飞由 GCS_A 按编号列表经 bridge 下发；
- A01–A12 先起飞，A13–A15 在重编队前再起飞。

## 需要新增或修改的内容

### 1. 新增真机启动脚本

保留原有 SITL 文件，新增：

```text
.tmp/scripts/startup_ego_sitl_real.sh
src/safe_valley_exp/startup_offboard_ego_real.sh
```

`startup_offboard_ego_real.sh` 是每架无人机的通用机载启动脚本，通过环境变量区分设备：

```text
UAV_INDEX
UAV_NAME
UAV_ID
UAV_IP
ROS_MASTER_URI
FCU_URL=/dev/ttyTSH0:921600
```

它只启动本机节点，不执行仿真相关内容：

```text
roscore
MAVROS
uav_executor_ego
ego_planner_driver
pose_relay
setpoint_relay
topology bridge
gp_origin_receiver
gps_bias_node
takeoff agent
```

`startup_ego_sitl_real.sh` 作为 GCS_A 端启动器，负责：

```text
GCS_A ROS Master
GCS_A topology bridge
gcs_a_backend
TCP Server
标定节点
15 架无人机状态和服务就绪检查
```

它不得启动：

```text
Gazebo
PX4 SITL
multi_uav_ego_15sim.launch
offboard_takeoff_15.py
```

### 2. 新增真机 launch 和拓扑配置

新增：

```text
src/safe_valley_exp/launch/uav_offboard_ego_real.launch
src/tcp_to_ros/launch/topology_group_a_real.launch
src/tcp_to_ros/config/topology_group_a_real.yaml
```

真机 launch 应复用当前 `uav_executor_ego.launch` 的执行链路，但将 MAVROS 连接改为：

```xml
fcu_url:=/dev/ttyTSH0:921600
```

真机拓扑配置使用：

```text
GCS_A: 192.168.5.10
UAV1: 192.168.5.11
UAV2: 192.168.5.21
...
UAV15: 192.168.5.151
```

所有真机节点的：

```text
port_offset
```

统一设置为：

```text
0
```

不得继续使用 `127.0.0.1` 或 SITL 的端口偏移。

### 3. 增加桥接起飞服务

新增接口：

```text
src/swarm_uav_interfaces/srv/UavTakeoff.srv
```

并更新：

```text
src/swarm_uav_interfaces/CMakeLists.txt
```

每架无人机启动一个本地起飞服务节点，例如：

```text
/UAV1/uav_takeoff
/UAV2/uav_takeoff
...
/UAV15/uav_takeoff
```

该服务由 bridge 转发。服务请求只负责触发本机起飞流程，不能由 GCS 直接操作远程 `/mavros/set_mode` 或 `/mavros/cmd/arming`。

新增本机起飞 agent：

```text
src/swarm_uav_executor/scripts/uav_takeoff_agent.py
```

它在收到请求后执行：

```text
检查 MAVROS connected
检查当前为 disarm
发送 HOLD 或 AUTO.LOITER
arm
发送 OFFBOARD
等待 OFFBOARD 状态确认
```

起飞 agent 不得在上电时自动执行上述操作。

### 4. 改造 `offboard_takeoff_15.py`

保留现有默认 SITL 行为，增加明确的真机模式：

```bash
python3 offboard_takeoff_15.py --real 1 2 3
```

真机模式运行在 GCS_A，通过 bridge 调用：

```text
/UAV1/uav_takeoff
/UAV2/uav_takeoff
...
```

该模式不能设置以下 SITL 专用参数：

```text
COM_RCL_EXCEPT
NAV_RCL_ACT
```

起飞脚本必须支持分批列表：

```bash
python3 offboard_takeoff_15.py --real 1 2 3 4 5 6 7 8 9 10 11 12
```

之后：

```bash
python3 offboard_takeoff_15.py --real 13 14 15
```

GCS 端脚本通过 bridge 转发的 `/UAVn/mavros/state` 检查目标飞机是否已经：

```text
armed: True
mode: OFFBOARD
```

### 5. runtime 任务流程

第一阶段：

```bash
python3 offboard_takeoff_15.py --real 1 2 3 4 5 6 7 8 9 10 11 12
```

确认 A01–A12 起飞并进入 HOLD 后，运行：

```bash
python3 src/runtime/task_test.py \
  --backend tcp \
  --group A \
  --task prepare
```

该阶段只应产生 A01–A12 的 `MOVE_TO`。

在执行重编队任务前：

```bash
python3 offboard_takeoff_15.py --real 13 14 15
```

确认 A13–A15 已经完成：

```text
connected
armed
OFFBOARD
TAKEOFF
HOLD
```

然后继续执行 runtime 中的：

```text
recovery
coverage-segment-2
hold-end
```

其中需要明确验证：

- A01–A12 初始编队；
- `MOVE_TO`；
- `FOLLOW_ROUTE`；
- `FAULT_EXIT`；
- A13–A15 后续加入；
- 重编队；
- 最终 HOLD。

## 验证顺序

### 阶段一：不上桨验证

- 每架机上电后 systemd 正常启动；
- 所有节点处于运行状态；
- 所有飞机保持 `armed: False`；
- 所有飞机不自动进入 `OFFBOARD`；
- GCS_A 能看到 `/UAVn/mavros/state`；
- GCS_A 能看到 `/UAVn/uav_task`；
- bridge 能转发 `UavTakeoff` 服务；
- 发送起飞列表后，只有列表中的飞机动作。

### 阶段二：单机真机验证

只测试 UAV1：

```bash
python3 offboard_takeoff_15.py --real 1
```

确认：

```text
HOLD → arm → OFFBOARD → TAKEOFF → HOLD
```

然后只发送一个 `MOVE_TO`，确认：

```text
runtime
→ GCS backend
→ bridge
→ UAV1/uav_task
→ executor
→ ego planner
→ MAVROS
→ 飞控
```

### 阶段三：两机 MOVE_TO 验证

先起飞 UAV1、UAV2：

```bash
python3 offboard_takeoff_15.py --real 1 2
```

再执行：

```bash
python3 src/runtime/task_test.py \
  --backend tcp \
  --group A \
  --task prepare-two-uav
```

确认两机分别完成 `MOVE_TO`，且距离门禁、HOLD 和异常处理正常。

### 阶段四：12+3 分批验证

- A01–A12 起飞；
- 执行 `prepare`；
- A13–A15 起飞；
- 执行 `recovery`；
- 执行重编队和后续任务；
- 测试过程中不允许通过 systemd 自动重新 arm 或自动恢复 OFFBOARD。

## 成功标准

只有同时满足以下条件，才认为真机版本可用：

- 上电后节点自动启动，但飞机保持 disarm；
- GCS_A 可以按列表经 bridge 触发起飞；
- 未列入列表的飞机不 arm、不进入 OFFBOARD；
- `MOVE_TO` 能被 runtime 正确分发到目标 UAV；
- `ego_planner_driver` 能稳定发布 `/mavros/setpoint_raw/local`；
- 两机测试成功后，12+3 分批流程成功；
- A 组后续编队、重编队和 HOVER 流程成功；
- 真机模式不写入 SITL 专用 PX4 参数；
- 任意节点故障时不会自动重新 arm 或自动恢复飞行状态。
