# 2026-10-19 UAV1/UAV3/UAV4 bridge 部署与监视功能验证

## 范围

- 用户授权：UAV1/3/4 上电后同步程序，并测试 battery/state/GPS fix 监视功能。
- GCS 栈已于 14:50 启动（上一轮记录）；UAV6/11 已在线。
- 不涉及飞行、标定集合变更。

## 连接与前置检查

| 项目 | UAV1 | UAV3 | UAV4 |
|---|---|---|---|
| ping / SSH | PASS | PASS | PASS |
| hostname | UAV1 | UAV3 | UAV4 |
| 板钟 | 正常 | **1970 → 已修复 PASS** | 正常 |
| mavlink-routerd | enabled/active，仅 127.0.0.1:15500 | 同左 | 同左 |
| 启动前机载栈 | DOWN | DOWN，且无 `~/catkin_swarm6-2` | DOWN |
| 厂家自启项 | `formation_follow.sh.desktop` 存在，当前无进程 | 同左 | 同左 |

## 部署

- UAV1/UAV4：bridge 差异同前（config/CMake/README + 3 个新文件）；
  `catkin build swarm_topology_bridge` 均成功（RC=0）。
- UAV3：无 `~/catkin_swarm6-2`，新建工作区并同步四个机载包：
  `swarm_uav_interfaces`、`swarm_topology_bridge`、`swarm_uav_executor`、
  `ego_planner_driver`。
- UAV3 首次全量构建：5 个包全部成功，耗时 62.3 s（`build_UAV3.log`）。
- 日志：`build_UAV1.log`、`build_UAV4.log`、`build_UAV3.log`、
  `catkin_init_UAV3.log`。

## 启动

在线集合 `{1,3,4,6,11}`，三机邻机参数为其余四机（intents + swarm_pose），
`uav_task_timeout_s:=180`。三机均 `STARTED=1 / PASS`。

## GCS 端验证（14:59）

| UAV | 电压 | 模式 | GPS fix |
|---|---|---|---|
| UAV1 | 25.14 V | AUTO.LOITER | 6 |
| UAV3 | 25.94 V | AUTO.LOITER | 6 |
| UAV4 | 25.69 V | AUTO.LOITER | 6 |
| UAV6 | 25.45 V | POSCTL | 6 |
| UAV11 | 23.32 V | POSCTL | 6 |

五机均为 `connected=True`、`armed=False`。

## Monitor 端到端输出

直接运行 `fleet_monitor.py` 4 s，截取最后一帧：

```text
Fleet Status   2026-10-19 14:59:53   stale > 3.0s: --
UAV     Voltage(V)   Mode            GPS Fix
UAV1    25.13        AUTO.LOITER     6
UAV2    --           --              --
UAV3    25.93        AUTO.LOITER     6
UAV4    25.69        AUTO.LOITER     6
UAV5    --           --              --
UAV6    25.45        POSCTL          6
UAV7    --           --              --
UAV8    --           --              --
UAV9    --           --              --
UAV10   --           --              --
UAV11   23.32        POSCTL          6
UAV12   --           --              --
UAV13   --           --              --
UAV14   --           --              --
UAV15   --           --              --
UAV16   --           --              --
```

结论：battery/state/GPS fix 三话题与 3 s 丢失显示功能 PASS。

## 备注

- UAV6/11 仍按上一轮“仅互为邻机”运行；bridge 数据不受影响。
  若需要完整规划邻机拓扑，后续用在线集合重启 6/11。
- GCS 标定节点配置仍为六机 UAV6–11；UAV1/3/4 不在其计算/确认集合。
- 三机均有 `formation_follow.sh.desktop` 自启项，当前未运行；建议现场确认是否禁用。
