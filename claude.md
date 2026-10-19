# 项目背景与协作约定

> 供后续 AI 对话快速了解本工作区；当前入口与通信进度于 2026-10-19 复核，历史实测按各条日期解读。这里是索引，不代替代码、现场规程或最新验证记录。文档与代码冲突时，先确认版本和实际运行配置，不凭早期方案推断已交付能力。

## 目标与边界

- 目标：在约 100 m × 150 m 场地开展不少于 30 架四旋翼的 A/B 两组联合任务。A 组编队覆盖侦查、模拟故障退出、备用机接替并继续覆盖；侦查结果触发 B 组模拟打击，随后两组返航。这里的“容错自愈”指**任务组织与重分配**，不指飞控/硬件自动修复。
- 本工作区实现 **Group A（GCS_A + 机载链路）**；B 组地面站及其打击执行不在此工作区的交付范围。`references/runtime/` 是独立行为树上层及 A/B 协议、示例任务包，**不是**本工作区的地面站实现。初版联合方案、31 节点示例包是目标与候选配置，不能等同于 30+ 架实机联合验收。
- 任务层负责批次、阶段、成败；飞控状态检查、轨迹执行和安全控制在下层。实机由现场人员遥控 arm/OFFBOARD，随后 planner 自动执行 5 m 软起飞；最终降落由人工接管。机载执行器不自行 arm、切换 OFFBOARD 或启动任务。

## 系统地图

`references/runtime/`（BT / TCP Client）→ `src/tcp_to_ros/`（GCS_A TCP Server、整批校验/分发/汇总与标定）→ `src/swarm_topology_bridge/`（多 ROS Master 的 Topic / Service 桥；ZMQ over TCP，默认每节点一条跨话题 bundle 流、10 Hz，仍逐对端扇出；全连接随节点数平方增长，30 机容量未验收）→ `src/swarm_uav_executor/`（逐机 PREPARE/START/ABORT/HOLD、领机轨迹/跟机编队）→ `src/ego_planner_driver/`（EGO 规划候选）和 `setpoint_relay`（MAVROS setpoint 唯一出口）→ PX4。`src/swarm_uav_interfaces/` 定义共享 ROS 契约；`src/swarm_simulation/` 提供 PX4/Gazebo SITL。

- 任务 ID `Axx` 与机载执行身份 `UAVn` 分层；映射由 GCS_A 配置维护，换机须核对身份、飞控 `MAV_SYS_ID`、MAVROS 目标和任务包，不能仅凭机号假定映射。
- 上下层使用 TCP 长连接、UTF-8 JSON Lines（每条以 `\n` 结尾），默认端口 `39001`；一条 `COMMAND` 是包含 `assignments` 的**整组批次**。回复只用 `ACCEPTED` → `COMPLETED` / `FAILED`，不是行为树内部的 RUNNING/SUCCESS/FAILURE；5 秒内确认接收，按 `command_id` 幂等，`HOLD` 可抢占，断线或超时须进入安全悬停。具体字段、去重和断线语义以 `references/runtime/docs/实物无人机上下层TCP-IP接口对接方案.md` 及现有代码为准；`--auto-peer-events` 仅供单组联调，不代表正式 A/B 跨进程协同已打通。
- 实机标定须在 disarm 时完成，确认任务坐标原点/航向、高度基准、每架机的原点回读；任务坐标约定为右/前/上。标定节点只在启动时冻结一次；任何机载栈重启、换电或飞机移动后，必须重启 GCS 标定节点（或整个 GCS 栈）重新标定。桥中继不保留 ROS latch，`gp_origin_confirmed` 等一次性消息晚订阅收不到；`calibration_ready` 只覆盖计算集，确认集内的其他机（如六机测试的 UAV11）须人工逐机核对。起飞后确认 armed + OFFBOARD 和实际高度，再下发 PREPARE/START。EGO 链路中 `/mavros/setpoint_raw/local` 只能由 `setpoint_relay` 发布，其他发布者会造成控制争抢。

## 已有验证与未解决问题

- **当前进度（2026-10-18/19 六机地面调试）**：三机实飞已通过（历史）；2026-10-18 完成 UAV6–11 六机地面部署、bundle 桥接和标定；`flow.sh odom` 快查六机 `swarm_pose` 全互联（每机 5/5 源、无 stale），正式 20 s 热测窗口未完成（现场中止）。调整 router 转发后通信问题已基本解决。六机入口 `verification/six_uav_joint/`，GCS `gcs_real.launch`；标定计算集 UAV6–10、确认集六架。**六机完整任务实飞尚未执行/验收，30+ 架容量未验证。**
- A 组主要链路已实现、部署，并做过自动测试与 SITL（2026-09-11 的 15 机 prepare SITL、观测加固后四机彩排两轮均通过）；这不代表完整覆盖、故障恢复或 A/B 联合实飞通过。
- **2026-09-22/23 两轮实飞失败根因已定位**（`quality_reports/2026-10-08_*.md`）：`coverage` 开始后跟随者 0–3 s 收不到领机 odom，而领机正常飞行、订阅早已建好 ⇒ 断点在 bridge 经电台的中继段，且该层当时无观测、无自愈。两轮成败表里的 “SUCCESS” 是 runtime 合成的（已修），不能当飞行事实；四机轮 A12 更以 ulog 证伪（基本未移动）。不要沿用“跟机 planner 日志停写 = planner 卡死”的已推翻判断。
- **链路停摆是真实设备行为（历史，2026-10-09/10）**：2026-10-09 地面实测电台链路自发单向断（GCS→UAV10 ping 全丢、反向 MAVLink 仍通，断电重上才恢复）；2026-10-10 三机空中 5 m 悬停 6 分钟内 8 次整链路静默 **1.1–5.2 s**，全部发生在 UAV13 的机间链路（到 8、到 10 双向同时；GCS↔13 仅 2 次受波及），8↔10 零断口；数据不丢（TCP 积压后补发），定层在 **RF/电台链路余量**（`quality_reports/2026-10-10_*.md`）。2026-10-18 六机地面 odom 快查（router 转发调整、bundle）未复现断口，正式 20 s 热测窗口未完成；现场据此决定：地面 `flow.sh odom`/热测通过即作为通信门禁，起飞后 quick odom 可选，跳过须记 `SKIP（原因）`；新链路版本或出现异常时仍建议空中快查。
- 历史缓解（2026-10-10）：odom 桥接限频 **20→12 Hz**、`LEADER_LOST` 超时 **1→3 s**；后续已换轻量 `swarm_pose` 并对齐 bundle 10 Hz。UAV13 射频排查为历史遗留，不在当前 UAV6–11 六机测试范围；空中长时稳定性及 30+ 机分发容量仍须对应实测证据。
- router：2026-10-09 起为 **systemd 服务**（`tools/router_service_install.sh` 部署，取代桌面自启），2026-10-10 去掉 `.242:15500` 冗余转发，2026-10-18 去掉 `.241:15500` 的 GCS 遥测转发（GCS 无 MAVLink 消费者，转发只抬高电台包率；现只保留 `127.0.0.1:15500`）。2026-10-18 六机逐机复核 enabled/active/单实例/参数一致；**已出现的 UAV 已修改，新机需首次配置**，每轮仍须只读复核并登记。已知定向通道死（遥测正常但 `param/get` 或 `set_gp_origin` 命令超时）可通过重启 `mavlink-routerd` + 机载栈恢复；UAV7/9 本轮实测复现并恢复。定向通道异常要区分 router、启动顺序与机载→飞控物理链路，不能单凭遥测正常判定双向健康。

## AI 工作规范与资料入口

- 常规上机按 [`verification/real_uav_workflow/README.md`](verification/real_uav_workflow/README.md) 执行：首次接入登记 → 每轮残留/router/板钟检查 → 按部署指纹决定是否 rsync → 通信测试（冷/热二选一；飞行准备默认只做热测，冷测必须在起栈前）→ 起机载/GCS、标定、通信门禁 → 飞手起飞与 GCS 任务交接。GCS 启动/查询须覆盖 `ROS_HOSTNAME`：`env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311`。详细命令/历史排查仍查 `real_setup.md` 和任务专用规程；本流程不代表已完成合包实机验收。
- runtime 的 TCP 协议继续维护，联合任务初版仅作需求背景；历史状态机提案/快照不作为当前操作依据。
- 回答“已完成/已验证”时区分**代码实现、自动测试、SITL、实飞和完整联合任务**；优先核对当前代码与配置、`quality_reports/` 最新结论和 `verification/` 证据。历史 README/计划里的“待办”可能过时，尤其注意报告中的后续更正。
- 先查 `src/tcp_to_ros/plans/README.md` 的设计决策与计划状态，再查对应子包 README、`src/swarm_uav_interfaces/real_setup.md` 和具体测试记录；不要把废弃的 safety lease、旧编队席位或旧 router 桌面自启方案当成当前方案。任务与协议分别查 `references/runtime/docs/容错自愈联合任务方案(初版).md`、`references/runtime/docs/实物无人机上下层TCP-IP接口对接方案.md`。
- 验证入口：`verification/ego_smoke/`（基础检查）、`verification/six_uav_joint/`（**当前六机 UAV6–11 联合任务**，GCS `gcs_real.launch`）、`verification/three_uav_recon/`（历史标准形式；任务流程可不同，但下发任务前须完成地面通信检查——热测/`air_flow.sh odom` 通过；起飞后 quick odom 可选，跳过记 `SKIP`）、`verification/single_uav_recon/`、`verification/four_uav_recon/`、`verification/five_uav_recon/`（分阶段规程/仿真脚本）、`verification/airlink_test/`（电台包率/丢包实测）、`verification/dag_recovery/`（DAGL 十机拓扑恢复实验，先离线测试再 SITL）、`quality_reports/`（现场结论）、`src/uav_log/`（实飞原始数据及索引）。新修改先做针对性测试与无桨/SITL 验证；涉及实际飞行、网络部署或飞控参数的动作必须经过现场负责人确认。
- 六机 driver 当前口径（`verification/six_uav_joint/driver.py`）：自动生成 `src/uav_log/<时间>_six-uav-joint/driver.jsonl`，不需要 `CONTINUE` 或交互终端；不自动检查逐机 `gp_origin_confirmed`、起始位置/高度、连续 `swarm_pose` 新鲜度和 1m 间距（机载负责 3s `LEADER_LOST`/1m HOLD，backend 负责 10/15s 链路判定）；阶段终点核对水平≤2m、高度≤1m；UAV11 不参与启动全局位姿门槛。起始位置/高度和 5m 网格摆位由操作员起飞前人工保证。
- `src/*` 多为**独立 Git 子模块**；`references/runtime/`、`references/DAG-GCBF/` 也是独立子模块（同在 GitHub `rm-vision-backup` 组织下），原则上只读参考——DAG-GCBF 的说明性文档可改，但要在该仓自行提交并推送，再同步主仓指针。不要覆盖已有未提交改动、批量同步机载代码、清理日志/构建产物或直接调整实机安全门禁；每次改动明确作用仓库、验证范围与未覆盖风险。构建使用 `catkin build`，不要编辑 `build/`、`devel/`、`logs/`。

## 提交规则（多仓库工作区）

本工作区是"主仓 + 独立子模块"结构：**先在各子模块分别提交，最后在主仓提交一次指针同步**。

### 顺序与粒度

1. 一次改动涉及几个包就在几个子模块里各提交一次；**一个提交只做一件事**（常用拆分：功能实现 /
   该功能的文档 / 采集或工具适配 / 独立缺陷修复；不要混入"顺手改的"）。
2. 最后在主仓提交**指针同步**：`git add <子模块路径>`（**不要 `git add -A` / `git add .`**）。
3. 分支：`swarm_topology_bridge`=`services`，`swarm_uav_executor` / `tcp_to_ros` /
   `references/runtime`=`main`，主仓=`master`。**提交落在各仓当前分支**；不新建分支、不改写历史
   （amend / force-push）除非明确要求。

### 提交信息格式

```
<type>(<scope>): <一句话说做了什么>          ← 标题，中文，≤50 字
                                            ← 空行
为什么做 / 根因（1–3 句；能引报告、现象或前一轮结论）
- 改了什么（按文件/模块列点，关键处带 文件:行号）
- 验证：跑了什么、结果如何（测试数量、实测数字、SITL 轮次）；没验证就写"未验证 + 原因"

Co-Authored-By: Claude Code <noreply@anthropic.com>
```

- **type**：`feat` / `fix` / `docs` / `chore`（构建、配置、同步）/ `refactor`（不改行为）/ `test`；
  scope 用包名或子系统（如 `feat(tools):`），跨包时省略。
- 标题只写"做了什么"，**为什么必须进正文**；验证写实（"四机 SITL 彩排 ROUND RESULT: SUCCESS"、
  "102 passed"），涉及实飞行为的改动注明未覆盖风险（如"仅 SITL，电台链路未覆盖"）。
- 主仓同步提交要写清同步了哪几个子模块的哪件事。

### 不要提交

`runtime_logs/`、`.runtime_logs/`、`.ros_home/`、`quality_reports/`（均已 gitignore）。
`src/uav_log/` 自 2026-10-12 起按轮次整理入库：`rosbag/`、`*.bag`、`*_bridge_health*.jsonl`
与单文件 ≥5 MB 的由 .gitignore 排除（文件留在本地磁盘），其余文本证据随轮次提交。
