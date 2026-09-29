# 项目背景与协作约定

> 供后续 AI 对话快速了解本工作区；状态截至 2026-09-29。这里是索引，不代替代码、现场规程或最新验证记录。文档与代码冲突时，先确认版本和实际运行配置，不凭早期方案推断已交付能力。

## 目标与边界

- 目标：在约 100 m × 150 m 场地开展不少于 30 架四旋翼的 A/B 两组联合任务。A 组编队覆盖侦查、模拟故障退出、备用机接替并继续覆盖；侦查结果触发 B 组模拟打击，随后两组返航。这里的“容错自愈”指**任务组织与重分配**，不指飞控/硬件自动修复。
- 本工作区实现 **Group A（GCS_A + 机载链路）**；B 组地面站及其打击执行不在此工作区的交付范围。`references/runtime/` 是独立行为树上层及 A/B 协议、示例任务包，**不是**本工作区的地面站实现。初版联合方案、31 节点示例包是目标与候选配置，不能等同于 30+ 架实机联合验收。
- 任务层负责批次、阶段、成败；飞控状态检查、轨迹执行和安全控制在下层。实机由现场人员遥控 arm/OFFBOARD，随后 planner 自动执行 5 m 软起飞；最终降落由人工接管。机载执行器不自行 arm、切换 OFFBOARD 或启动任务。

## 系统地图

`references/runtime/`（BT / TCP Client）→ `src/tcp_to_ros/`（GCS_A TCP Server、整批校验/分发/汇总与标定）→ `src/swarm_topology_bridge/`（多 ROS Master 的 Topic / Service 桥）→ `src/swarm_uav_executor/`（逐机 PREPARE/START/ABORT/HOLD、领机轨迹/跟机编队）→ `src/ego_planner_driver/`（EGO 规划候选）和 `setpoint_relay`（MAVROS setpoint 唯一出口）→ PX4。`src/swarm_uav_interfaces/` 定义共享 ROS 契约；`src/swarm_simulation/` 提供 PX4/Gazebo SITL。

- 任务 ID `Axx` 与机载执行身份 `UAVn` 分层；映射由 GCS_A 配置维护，换机须核对身份、飞控 `MAV_SYS_ID`、MAVROS 目标和任务包，不能仅凭机号假定映射。
- 上下层使用 TCP 长连接、UTF-8 JSON Lines（每条以 `\n` 结尾），默认端口 `39001`；一条 `COMMAND` 是包含 `assignments` 的**整组批次**。回复只用 `ACCEPTED` → `COMPLETED` / `FAILED`，不是行为树内部的 RUNNING/SUCCESS/FAILURE；5 秒内确认接收，按 `command_id` 幂等，`HOLD` 可抢占，断线或超时须进入安全悬停。具体字段、去重和断线语义以 `references/runtime/docs/实物无人机上下层TCP-IP接口对接方案.md` 及现有代码为准；`--auto-peer-events` 仅供单组联调，不代表正式 A/B 跨进程协同已打通。
- 实机标定须在 disarm 时完成，确认任务坐标原点/航向、高度基准、每架机的原点回读；任务坐标约定为右/前/上。起飞后确认 armed + OFFBOARD 和实际高度，再下发 PREPARE/START。EGO 链路中 `/mavros/setpoint_raw/local` 只能由 `setpoint_relay` 发布，其他发布者会造成控制争抢。

## 已有验证与未解决问题

- A 组主要链路已实现、部署，并做过自动测试与 SITL；例如 2026-09-11 的 **15 机 prepare SITL** 成功。这不代表 15 机完整覆盖、故障恢复或 A/B 联合实飞通过。
- **两轮编队实飞**：2026-09-22 三机、2026-09-23 四机均完成 `MOVE_TO/prepare`，但 `FOLLOW_ROUTE/coverage` 失败；两轮跟随者实际上未跟随领机。四机轮 A12 虽返回 SUCCESS，ulog 却显示基本未移动，不能以状态回执代替物理完成证据。
- 近期优先排查：任务态领机 odom 到跟机的订阅/传输（跟随机启动 3 秒无帧报 `LEADER_LOST`，随后 1 秒超时）、虚假 SUCCESS、机载关键段日志中断。起飞前限频测试正常，**不能**据此断言任务态正常；下一轮应录机载 rosbag 并用 PX4 ulog 对齐物理事实。不要沿用“跟机 planner 日志停写 = planner 卡死”的已推翻判断。
- 现场 `mavlink-routerd` 脚本修复以 **2026-09-23** 质量报告为准（移除 `-r`，保持前台运行）；2026-09-22 版后台化修法已证实有缺陷。参数不回传要区分 router、启动顺序与机载到飞控的物理链路，不要单凭遥测正常判定双向链路健康。

## AI 工作规范与资料入口

- 回答“已完成/已验证”时区分**代码实现、自动测试、SITL、实飞和完整联合任务**；优先核对当前代码与配置、`quality_reports/` 最新结论和 `verification/` 证据。历史 README/计划里的“待办”可能过时，尤其注意报告中的后续更正。
- 先查 `src/tcp_to_ros/plans/README.md` 的设计决策与计划状态，再查对应子包 README、`src/swarm_uav_interfaces/real_setup.md` 和具体测试记录；不要把废弃的 safety lease、旧编队席位或旧 router 修法当成当前方案。任务与协议分别查 `references/runtime/docs/容错自愈联合任务方案(初版).md`、`references/runtime/docs/实物无人机上下层TCP-IP接口对接方案.md`。
- 验证入口：`verification/ego_smoke/`（基础检查）、`verification/single_uav_recon/`、`verification/four_uav_recon/`（分阶段规程/仿真脚本）、`quality_reports/`（现场结论）、`src/uav_log/`（实飞原始数据及索引）。新修改先做针对性测试与无桨/SITL 验证；涉及实际飞行、网络部署或飞控参数的动作必须经过现场负责人确认。
- `src/*` 多为**独立 Git 子模块**；`references/runtime/` 也是独立子模块，原则上只读参考。不要覆盖已有未提交改动、批量同步机载代码、清理日志/构建产物或直接调整实机安全门禁；每次改动明确作用仓库、验证范围与未覆盖风险。构建使用 `catkin build`，不要编辑 `build/`、`devel/`、`logs/`。
