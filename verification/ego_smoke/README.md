# EGO-Swarm P1 验证脚本

本文档下的脚本用于 P1 ego-swarm 开发阶段的运行时验证与回归。它们属于工程验证手段，
**不作为任何 ROS 功能包的交付物**，因此放在工作空间顶层 `verification/`（不属于任何
`src/*` git 仓库），避免改变各功能包 CMakeLists/构建范围，同时保证脚本可追溯、可复现。

## 脚本列表

| 脚本 | 用途 | 场景 |
|---|---|---|
| `smoke_single_master.py` | 单 Master ego 生命周期冒烟 | 单机 IDLE→EXECUTING→COMPLETED→HOLD |
| `smoke_reciprocal.py` | 单 Master 双 ego 节点 reciprocal 冒烟 | 双机同飞（cross）+ 错时起飞（stagger） |

## 运行方式

### 前置

- 已 `catkin build` 且 `source devel/setup.bash`；
- ROS Master、ego 节点由调用方另行启动（见各脚本注释中的拓扑假设）。

### 单 Master 冒烟

```bash
roscore &
roslaunch ego_planner_driver ego_planner_driver.launch uav_id:=A01 exec_target:=UAV1 &
python3 verification/ego_smoke/smoke_single_master.py
```

### 单 Master 双 ego reciprocal

```bash
roscore &
roslaunch ego_planner_driver ego_planner_driver.launch uav_id:=A01 exec_target:=UAV1 &
roslaunch ego_planner_driver ego_planner_driver.launch uav_id:=A02 exec_target:=UAV2 &
python3 verification/ego_smoke/smoke_reciprocal.py
```

## 验证结论

验证结果（含启动步骤、输出证据与清理）统一归档在
`src/tcp_to_ros/test_records/` 按日期命名的测试记录中。