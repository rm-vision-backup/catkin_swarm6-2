# 2026-10-19 UAV6/UAV11 bridge 部署与启动

## 范围

- 用户授权：检查 UAV6/11 连接；同步并启动两机机载栈（本轮主要是 bridge 变更）。
- 不涉及：UAV7–10（现场不可达）、GCS 栈启动、标定、飞行任务。

## 环境与连接

| 项目 | UAV6 (192.168.5.61) | UAV11 (192.168.5.111) |
|---|---|---|
| ping / SSH | PASS | PASS |
| hostname | UAV6 | UAV11 |
| 板钟（+0800） | 2026-10-19 14:40:49 | 2026-10-19 14:40:50 |
| mavlink-routerd | enabled/active，单实例，仅 127.0.0.1:15500 | 同左 |
| 启动前机载栈 | DOWN，4200/14000/11311 无监听 | DOWN，同左 |
| UAV7–10 | 192.168.5.71/81/91/101 均不可达 | — |

## 部署

- rsync checksum 干跑：bridge 差异为 `CMakeLists.txt`、`README.md`、
  `config/topology.yaml`、新增 `fleet_monitor.py`/`fleet_monitor.launch`/测试；
  executor 无差异。
- 同步后 checksum 干跑为空。
- `catkin build swarm_topology_bridge`：UAV6/UAV11 均成功（RC=0）。
- 部署后因发现 battery 类型不匹配，修正为 `sensor_msgs/BatteryState` 后再次
  rsync + build + 重启；日志见 `build_UAV6_retry.log`、`build_UAV11_retry.log`。

## 问题与修正

- 现象：state/GPSRAW 可跨机收到，`/mavros/battery` 远端无样本。
- 定位：机上 `/mavros/battery` 实际类型为 `sensor_msgs/BatteryState`，
  原配置为 `mavros_msgs/BatteryStatus`，ROS 两端类型不匹配，bridge 订阅未连通。
- 修正：bridge 配置与 fleet_monitor 订阅类型统一为 `sensor_msgs/BatteryState`。
  提交：bridge `9d46922`，主仓指针 `2f6dbec`。

## 启动与验证

启动参数（仅 UAV6/11 在线，互为唯一邻机）：

```text
UAV6  neighbor_intents:=/UAV11/trajectory_intent neighbor_odom_topics:=/UAV11/swarm_pose uav_task_timeout_s:=180
UAV11 neighbor_intents:=/UAV6/trajectory_intent  neighbor_odom_topics:=/UAV6/swarm_pose  uav_task_timeout_s:=180
```

| 检查项 | UAV6 收到 UAV11 | UAV11 收到 UAV6 |
|---|---|---|
| `/UAVn/mavros/battery` 类型 | sensor_msgs/BatteryState | sensor_msgs/BatteryState |
| battery 样本 | 23.466 V, 52% | 25.676 V, 100% |
| battery 转发频率 | 0.500 Hz（源 0.5 Hz） | 0.500 Hz |
| `/UAVn/mavros/state` | connected=True, armed=False, POSCTL | 同左 |
| `/UAVn/mavros/gpsstatus/gps1/raw` | fix_type=6 | fix_type=6 |
| 本机节点 | /mavros, /swarm_bridge, /uav_executor_UAV6 | /mavros, /swarm_bridge, /uav_executor_UAV11 |

结论：UAV6↔UAV11 三话题桥接 PASS；两机均 disarm、POSCTL。

## 未完成 / 下一步

- GCS 栈当前未运行；监视窗口只显示 `--` 属预期。
- 监视应用需重启以加载 `sensor_msgs/BatteryState` 修正。
- 启动 GCS 栈后才可从 GCS 查看 `/UAV6/...`、`/UAV11/...` 并进入标定流程。
- 若后续 UAV7–10 上线，UAV6/11 需用六机完整邻机列表重启。
