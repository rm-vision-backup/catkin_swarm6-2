# 实机上机测试工作流（AI 执行入口）

适用：GCS_A + 多架 UAV 的部署、地面通信快测、标定及实飞交接。详细部署查 [real_setup](../../src/swarm_uav_interfaces/real_setup.md)，通信测量查 [20 秒测试方法](communication.md)。**按编号执行；没有证据的步骤不得标为通过。**

用户已通过三机实飞；跨话题合包已实现；2026-10-19 六机 bundle 九阶段实飞通过（最终栈热测 SKIP，ulog 待 USB 核验），30+ 架容量未验收。历史记录 [10-12 报告](../../quality_reports/2026-10-12_airlink_scaling_test.md) 的 §8–§11 为本轮背景，预期 RTT/pps 不能填成实测结果。

## 执行约定

- 本轮操作范围由现场负责人确认；已有明确授权不重复询问。仅要求编写/审阅流程不构成连接、部署或飞行授权。
- AI 负责检查、已授权的部署/启动、采集和报告；发现未知控制程序时列出进程及自启来源，**请用户协助关闭，复查后继续**。不盲目 `pkill`。
- 地面准备与压力测试须**拆桨、全部 disarm、未处于 OFFBOARD**。AI 不代替飞手 arm、切模式或起飞。飞行中不改钟、不同步/构建、不重启 router/机载栈、不做 UDP 压测或批量下载。
- 不改飞控参数、安全门禁或任务机数来凑通过；失败保留证据，停在当前步骤。所有查询设超时（SSH 建连建议 8 s、单次 ROS 查询 10 s）；超时写“未通过”，不用无限重试。
- 每完成一步只报：`步骤 / 逐机结果 / 证据路径 / 下一步`；需要现场动作时才停下等待。步骤 6 是通信测试选择点，步骤 9 是飞行交接点。

## 0. 固定本轮清单

先从用户描述、既有记录与实际配置确定以下内容；缺项只询问缺项，不猜机队：

| 必填项 | 内容 |
|---|---|
| 本轮 ID / 机队 / 领机 | 时间、UAVn、IP、任务 Axx 映射；不能只按机数选配置 |
| 工作区 / SSH | GCS 与机载工作区路径；凭据用 `src/tcp_to_ros/tools/configs/onboard.env`，不输出密码 |
| 任务入口 | GCS launch、标定集合、formation、任务包、逐机邻居及 launch 参数 |
| 版本 / 测试范围 | 各源码包 commit + 未提交改动清单/文件校验；只测通信或还要飞行 |

本轮记录建在 `src/uav_log/<日期时间>_<测试名>/`，下文称 `RUN_DIR`。保存 `session.md`、同步/构建输出、配置快照及通信原始日志。`session.md` 逐步填 `PASS / FAIL / SKIP（原因）`，并记录用户的通信测试选择；不能只保留聊天记录。

已有匹配入口（参与集合必须完全一致）：

| 执行层机队 | GCS launch（`tcp_to_ros`） | 任务目录（`verification/`） |
|---|---|---|
| 8、10、13 | `gcs_a_real_three_uav.launch` | [three_uav_recon](../three_uav_recon/README.md) |
| 8、10、12、13 | `gcs_a_real_four_uav.launch` | [four_uav_recon](../four_uav_recon/README.md) |
| 8、10、12、13、16 | `gcs_a_real_five_uav.launch` | [five_uav_recon](../five_uav_recon/README.md) |

其它集合先建立对应配置并离线检查，不能套用上述入口。表中任务 prepare 可能升到 **15 m**，不能因初始悬停为 5 m 就认为后续任务也只飞 5 m。

## 1. 首次接入登记；每轮做残留检查

“已出现”依据持久记录判断，不依赖 AI 记忆。**UAV8/10/12/13/16** 在 10-12 轮已有上机记录；按本次用户说明，已出现机的 router 已修改，不重复安装。其它历史出现机可查 `real_setup` / `src/uav_log` 补登记；查不到记录的按首次接入处理。

- **新机**：核对 hostname、IP、飞控 `MAV_SYS_ID`、MAVROS 实际目标，检查进程、systemd/用户服务及 `~/.config/autostart/`。关注旧 `catkin_ws`、`offb_hover.py`、`formation_follow`、独立 MAVROS 等；发现后请用户关闭，并确认后续上电不会再次自动运行。
- **所有机每轮**：只读检查运行进程、`onboard_ctl.sh status <n>` 与关键端口占用；识别我方栈残留和其它控制程序。`status` 返回 0 不代表栈健康或已停止，须读逐机输出。已授权的我方栈收口仅在全部落地 disarm 后执行；未知程序仍交用户处理。
- 若仍有控制程序、未知端口占用或身份冲突，停在本步。router 是应保留的常驻服务。

## 2. 新机修改 router；老机跳过安装

仅在步骤 1 完成、机载栈停止时，对**新机编号**执行：

```bash
bash src/tcp_to_ros/tools/router_service_install.sh <新机编号>
```

脚本安装/启用 systemd 服务并禁用旧 router 桌面自启。现有 [unit 模板](../../src/tcp_to_ros/tools/systemd/mavlink-routerd.service) 已依次去掉 `192.168.5.242:15500`（2026-10-10）和 `192.168.5.241:15500`（2026-10-18；GCS 侧无 MAVLink 消费者，转发遥测只抬高电台包率），现只保留 `127.0.0.1:15500` 供本机 mavros 使用。模板绑定 `/home/nvidia/Mavlink_Routerd_Sh/mavlink-routerd`、`/dev/ttyTHS0:921600`；新机不同则先核实，不能硬套。

## 3. 每架机检查 router

逐机 SSH 执行只读查询，保存输出：

```bash
systemctl is-enabled mavlink-routerd
systemctl is-active mavlink-routerd
systemctl show mavlink-routerd -p MainPID -p ExecStart -p NRestarts
pgrep -ax mavlink-routerd
journalctl -u mavlink-routerd -b -n 30 --no-pager
```

通过：enabled、active、只有一个实例且 PID 对得上；实际参数只有 `127.0.0.1:15500`，无 `.241:15500` / `.242:15500`，日志无持续重启/串口错误。已登记机若不符合，记配置漂移并修复后复查，不以“以前改过”跳过。服务 active 不能代替步骤 7 的 MAVROS 双向健康检查。

## 4. 检查并同步板钟（在启动/构建前）

先核对 GCS 当前日期时间正确，再逐机读 `date -Ins; date +%s`。错钟/回 1970 的机用现有工具修正系统钟与各路 RTC：

```bash
bash src/tcp_to_ros/tools/onboard_clock_fix.sh <需修正的编号...>
```

核对输出机数与目标数一致、每机 PASS；正确的钟记录后直接复用。无 RTC 的板卡记入登记信息，每次重启复查。该工具默认容差 5 s，只供运行/粗略对时，**不能据此测毫秒级单向延迟，也不能保证精确日志对齐**。

## 5. rsync 同步及构建（同一版本只做一次）

这里的 sync 指 **`rsync`**。确认机载栈停止，先检查远端本地改动；有未知改动先保留并确认用途，不覆盖。只同步本轮需要的源码包：`swarm_uav_interfaces`、`swarm_topology_bridge`、`swarm_uav_executor`、`ego_planner_driver`；不推送整个工作区、凭据、日志或本机 `build/devel`。

逐机逐包模板（先替换实际登录目标和路径）：

```bash
rsync -rcn --itemize-changes --exclude='.git' --exclude='__pycache__/' --exclude='*.pyc' \
  src/<包名>/ <登录用户>@<机载IP>:<机载工作区>/src/<包名>/
# 核对清单后，同一命令去掉 -n 执行；默认不加 --delete。
```

若本次有删除/重命名，按差异清单处理对应旧文件；不能让旧文件继续参与构建。机载在其工作区执行 `source /opt/ros/noetic/setup.bash` 后用 `catkin build <受影响包及依赖>`，保存真实退出码及构建日志；新增 Python 模块也要按包安装规则构建。GCS 本地相关包同样须构建。构建后再次用 rsync checksum 干跑验证已同步文件无差异。

**合包本轮必须包含 GCS 和所有在线参与节点**：源码/依赖兼容、实际加载 topology 一致、`packing=bundle`；禁止混跑 legacy/bundle。不要仅凭源码 commit 认定机上已部署成功；记录实际同步文件校验和、构建结果、配置与逐机启动参数。

之后未改代码/配置且逐机部署证据仍一致，步骤 5 记 SKIP，重启无需重复同步；新增飞机、换工作区、代码/配置变化或发现漂移，仅重做受影响目标。重启仍须检查板钟、router、残留并重新标定。

## 6. 提醒用户选择通信测试

准备完成后告知：“部署检查完成。是否做 20 s 通信测试？冷测=栈停并每机模拟 UDP 1 Mbps、1200 B；热测=栈开、不注入负载，只测 ping RTT。” 等待选择；本轮已明确选择则直接沿用。**冷测和热测是二选一，不是两轮都做**：以飞行准备为目的时只做热测；冷测只在需要链路对比时做，且必须在起机载栈之前（栈停）完成。

- **需要**：按 [通信方法](communication.md) 做**热测**（机载程序启动稳定后，**不注入 UDP**；**以每架 UAV 的 `swarm_pose` 是否被全部在线邻机收到为主判据**，并测 ping RTT；**无需标定、不要求启动 GCS**）或**冷测**（我方机载/GCS 栈停止、router 保留，各机并发模拟 1 Mbps 上行），输出表格。全机同步参加每轮，不逐机串行压测。热测若在 GCS 启动后做，须记录 GCS 额外节点负载；冷测不得在起栈后做。
- 冷测/热测**不检查拆桨、disarm 或 OFFBOARD**：机载程序不会 arm、切 OFFBOARD 或启动任务（2026-10-18 现场口径）。测试不构成飞行授权。
- **不需要**：记录 SKIP，直接执行步骤 7；**启动、标定及实际消息连通性检查仍须完成**。你原设想的第 8 项按此理解。
- 新合包版本本轮尚无实机验证证据：可以只部署/启动观察；跳过测试不能写为链路验收通过，进入飞行前须补足对应无桨验证证据。

## 7. 启动机载与 GCS，完成标定（无论是否压测）

执行顺序：冷测（若选，必须在起栈前）→ 起机载栈 → 热测（若选，起 GCS 前）→ 起 GCS → 标定 → odom。热测不要放在标定之后；冷测不要放在起栈之后。

1. 按步骤 0 选定任务的摆位/航向/净空要求放好飞机，标定后不再移动。逐机准备邻居参数（**排除自己，包含全部本轮邻机**），实飞保留默认安全门禁，不带 `noprop:=true` 或贴地覆盖参数。**拆桨不等于需要 noprop；本步骤不下发任务。**
2. 使用 `start_onboard.sh`。推荐从 GCS 逐台调用包装工具，它最终执行该脚本并保留转录；每机参数不同，不能给多机套同一邻居列表：

   ```bash
   bash src/tcp_to_ros/tools/onboard_ctl.sh start <单机编号> -- \
     neighbor_intents:=<逗号分隔的邻机trajectory_intent路径> \
     neighbor_odom_topics:=<逗号分隔的邻机swarm_pose路径> \
     uav_task_timeout_s:=280
   ```

   `280` 是上表 recon 测试入口的部署参数，其它任务按各自配置核实。底层 [start_onboard.sh](../../src/swarm_uav_executor/bashs/start_onboard.sh) 不做前置检查；“进程在”不是就绪判据。
3. 若本轮选择热测，在起 GCS 前按 [通信方法](communication.md) 完成一次 20 s 热测并归档；热测不要求 GCS/标定。未选择则跳过。
4. GCS 选定 launch 在独立终端/受控会话启动并留日志（替换文件名）：

   ```bash
   env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
     bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
              roslaunch tcp_to_ros <本轮GCS入口.launch>'
   ```

5. 查询终端先 source 对应工作区环境，ROS Master 指向该机本地；GCS 查询同样清除 `ROS_HOSTNAME`、设置 `ROS_IP=192.168.5.241`、`ROS_MASTER_URI=http://localhost:11311`。每机核对：`/mavros/state` 为 connected=True、armed=False、非 OFFBOARD；MAVROS 实际目标正确；`/mavros/setpoint_raw/local` **唯一发布者是 `/setpoint_relay`**（用机载本地 `rostopic info`）。读取当前 launch 的 health_start/启动日志确认 GCS 与每机实际为 bundle，无 `proto_mismatch`/解码错误；不能只看 YAML。
6. GCS 核对实际 formation/标定集合与机队一致，`39001` 监听、逐机任务 service 可见；GPS `fix_type ≥3`、航向差 ≤30°、有效采样满足现有标定规则。**全部 disarm 时**完成标定，确认 `/group_a/calibration_ready=True`，并逐机确认 `/<UAVn>/gp_origin_confirmed=True`；保存本轮原点、参考航向及 ready 清单，不能沿用旧轮 latched True。**任何机载栈重启、换电或飞机移动后，必须重启 GCS 标定节点（或整个 GCS 栈）重新标定。**
7. 显式设置本轮 `AIR_FLOW_UAVS`、`AIR_FLOW_LEADER`、`AIR_FLOW_DIR` 后执行 `bash verification/three_uav_recon/air_flow.sh odom`，保存每机本地发出及**各邻机**接收新鲜度；缺机/缺流不能通过。这里只做 quick odom 快查（不是再次 20 s 热测）；若本轮选择热测，已在第 3 步完成，不要在此重复。`stage1 20` 仅在未做正式热测时作为替代观察窗。不运行会下发飞行任务的 `stage2/full`。

## 8. 输出通信表并判断是否可交接

按照 [表格模板与判读规则](communication.md#结果表与判读) 输出结果；选择跳过也明确写 SKIP。不能用 RTT/2 冒充实测单向延迟，不能用 UDP 测试代替 ROS 消息可达性，也不能用 GCS 网卡统计代替全网/机间包率。

任一身份/版本/单发布者/标定/消息流检查不通过，停止交接；有明显 500 ms 级排队回归、≥1 s 机间消息断口、持续断链或异常丢包时，先保留证据排查。20 s 无异常只能说明本窗口未复现，**不代表长时、空中或完整任务通过**。

## 9. 提醒飞手起飞，并交付 GCS 任务命令

先给出本轮机队、步骤状态、通信表/跳过原因、标定与单发布者结果；满足门禁后提醒现场负责人检查电量、场地、桨叶、遥控接管及既定 failsafe。飞手按现场规程装桨、逐机 arm/OFFBOARD，**会触发自动上升约 5 m**；人员先撤离。AI 不自动执行起飞或任务命令。

提供**已替换成本轮目录**的命令给 GCS 操作员：

```bash
# 先离线核对；不连接 GCS，不发送任务
python3 verification/<本轮任务目录>/driver.py --phases prepare coverage --dry-run
# 全机 armed + OFFBOARD、实际高度稳定，且起飞后再次 odom 快查正常后，操作员执行
python3 verification/<本轮任务目录>/driver.py --phases prepare coverage \
  --tcp-host 192.168.5.241 --tcp-port 39001 --log <本轮记录目录>/driver.jsonl
```

地面 odom/热测通过即可作为通信门禁；起飞后 odom 快查由现场负责人决定，默认建议做一次约 10 s 的 quick odom，但不再作为硬性门禁。若跳过，在 `session.md` 记 `SKIP（原因）`。地面检查不能覆盖空中姿态、天线朝向、多径和电机干扰的差异；新机数、新链路版本或曾出现空中断流时，仍建议起飞后快查。不向空中链路注入 UDP 压力流量；若本轮已做正式 20 s 热测，起飞后只做 quick odom，不重复 20 s 窗口。`air_flow.sh stage1 20` 仅在未做正式热测时作为替代观察窗。首次新机数/新链路版本按任务专用规程分阶段飞行，不自动进入 coverage。任务 dry-run 应在地面准备期间完成；这里交付的是最终命令及明确的执行时机。

## 10. 落地收尾与下轮复用

飞手确认全部落地 disarm 后，逐机 `onboard_ctl.sh stop <n>`、停止 GCS，保留 router。执行 `log_fetch.sh <本轮编号...>` 采集 ROS/health/终端日志；该工具**默认不取 ulog**，确需时落地后加 `--ulog` 串行获取；rosbag 单独归档。

在 `session.md` 补齐故障、最终实际部署指纹与逐机首次配置验收（时间、hostname/IP/sysid、旧自启处理、router 实际参数）；下轮据此跳过首次配置/无变化同步。通信测试与飞行任务分别记结果，不能把“起栈成功”“driver 返回成功”直接写为实飞成功。
