# 冷测 / 热测：20 秒通信延迟测试

由 [现场工作流](README.md#6-提醒用户选择通信测试) 调用。方法参考 [10-12 报告 §1、§8](../../quality_reports/2026-10-12_airlink_scaling_test.md)；复用已有 tx/rx 工具，不依赖历史 `/tmp/radio_sample.py`。

**称呼约定（2026-10-18 现场口径）**：

- **冷测**：我方机载 / GCS 栈停止（router 保持），由各机**人工模拟发布** UDP 1 Mbps、1200 B、20 s 上行负载，同时测 ping RTT。
- **热测**：机载程序已启动并稳定，**不注入任何人工负载**；核心检查是**每架 UAV 的 `swarm_pose` 能否被所有在线邻机收到**（逐源 rx / age / gap / stalls），并用 ping RTT 观察通信延迟。业务节点自身发布的流量就是负载。
- 两轮目的不同：冷测看“空栈 + 人工负载”下的链路表现；热测看生产栈自身流量的真实通信延迟。**冷/热同时改变了栈状态和人工负载，差值不能解释为单纯“启动机载栈”的影响。**

**安全与边界**：机载程序不会 arm、不会切 OFFBOARD、不会启动飞行任务，因此冷测 / 热测**不需要检查拆桨、disarm 或 OFFBOARD**。测试不构成飞行授权；测试期间不执行飞行任务、不改飞控参数、不重启 router。热测不要求标定，不等 `/group_a/calibration_ready` 或 `gp_origin_confirmed`，也不要求启动 GCS 栈；若同时启动了 GCS，须在 `session.md` 写明（负载多一个节点）。

## 固定条件

| 项目 | 冷测 | 热测 |
|---|---|---|
| 我方机载 / GCS 栈 | 全停（router 保留） | **机载栈启动并稳定**；不要求标定、不要求 GCS |
| 人工 UDP 负载 | 每机 1 Mbps，1200 B，20 s | **无** |
| 业务自身流量 | 无（栈停） | bridge / MAVROS / router 正常发布；**重点核对每机 `swarm_pose` 的全互联接收** |
| ping RTT | GCS → 每机，0.5 s 间隔，20 s | 同冷测 |
| 计数器 | GCS 及每台 UAV 实际电台网卡 | 同冷测 |
| UAV、摆位、网卡、电台设置 | 相同并记录 | 不变 |

冷测的 1 Mbps/机是人工附加负载（5 机 = 额外 5 Mbps），不是日常生产流量；扩到更多机时不机械叠加，先选定合理的每机或聚合负载并写入本轮记录，冷测各轮之间必须相同。热测不注入 UDP，因此**不得把热测 RTT 说成“有载 RTT”**；需要对比有载/无载时，另做一对明确标注的实验，不能事后换口径。

测量期间不 rsync / build / 下载日志，不调整 router / packing，不新启或重启节点。非参与但在线的 UAV 也可能占空口，须记录背景节点；AI 不擅自关其它设备。

## 准备一次

- 冷测需要 tx/rx 工具：用 rsync 把本机 `verification/airlink_test/airlink_tx.py` 同步到**每台**机载 `~/airlink_tx.py` 并校验文件；记录工具校验和。热测不需要发送工具。
- GCS 检查 UDP 5700 无旧接收器占用（冷测前）。逐机 `ip route get 192.168.5.241` 找到实际电台出口；GCS 用到机载 IP 的路由确定网卡，不假定一直是 `eth0/enp5s0`。
- 先建立 SSH 连接（可复用连接），准备并发调度。各机发送 / 探测窗口尽量同时，记录每项实际起止时间；不能串行每架测 20 s 再称为多机同时测试。
- 下文命令是组成部分，须由 AI 按并发顺序编排、重定向到本轮冷测 / 热测子目录；**占位符不能直接执行**。每个后台任务须等待完成并核对退出码，不能把 SSH 启动成功当作测量完成。

## 冷测动作

1. 建 `RUN_DIR/comm_cold/`，记录起止时间、机队、栈状态、负载参数；确认我方栈已停、router 仍 active。先起 GCS 接收器，再确认监听成功：

   ```bash
   python3 -u verification/airlink_test/airlink_rx.py \
     --proto udp --port 5700 --size 1200 --duration 30
   ```

   接收器多出的时间只供建连与收尾；所有发送端仍各测 20 s。全部启动必须落在其监听窗口内，否则本轮无效，不能按截断数据报丢包。
2. 并发执行各机发送、GCS 逐机 ping、GCS/各机计数器采样：

   ```bash
   # 每台机载，通过 SSH 运行；所有机并发
   python3 -u ~/airlink_tx.py --proto udp --host 192.168.5.241 \
     --port 5700 --rate 1M --size 1200 --duration 20
   # GCS，每个目标各一个进程，并发
   LC_ALL=C ping -n -D -i 0.5 -w 20 <机载IP>
   # GCS 与每台 UAV：在本地各运行一份，起止采样对应这轮窗口
   date -Ins
   cat /proc/net/dev /proc/net/snmp
   sleep 20
   date -Ins
   cat /proc/net/dev /proc/net/snmp
   ```

   ping 窗口须与 UDP 负载窗对齐；若 SSH 启动错位明显，保留原始结果并标注“混合空载/有载”，修正调度后重测，不能作为严格有载对照。实际测量时长写入记录。
3. 等 tx/rx/ping 全结束，核对目标数、每机 tx `[done]`、接收来源 IP、完整覆盖窗口；ping 无回复也要保留并写 FAIL。部分发送器未运行时，该机写“发送失败/无效”，不写 0% 丢包。

## 热测动作

1. 建 `RUN_DIR/comm_hot/`，记录起止时间、机队、机载栈启动时间与状态（bridge packing、关键节点进程）；**不启动接收器、不注入 UDP**。
2. 设定本轮在线机与领机，在 GCS 执行 `swarm_pose` 全互联快查并保存输出：

   ```bash
   AIR_FLOW_UAVS="<在线机号>" AIR_FLOW_LEADER=<领机号> AIR_FLOW_DIR=<任务目录> \
     bash verification/three_uav_recon/air_flow.sh odom
   ```

   逐机核对：本机 local `swarm_pose` 的 rx/tx 在增长；remote `swarm_pose` 的 `srcs` 必须等于**在线邻机数（不含自己）**，每个源 rx 持续增长。**非领机源**：`stale_srcs` 为空、`age_max` ≤1 s、`gt1000=0`、`stalls=0`；**领机源**（跟随环 `LEADER_LOST=3 s`）：`age_max` ≤3 s、`gt3000=0`，1–3 s 空档记 ⚠、不判 FAIL。缺源或无该话题条目 = FAIL。阈值可用 `AIR_FLOW_LEADER_TOL_S`（默认 3.0）/ `AIR_FLOW_PEER_TOL_S`（默认 1.0）覆盖，须在测试前写入记录。
3. 记录窗口起点后等待 20 s，再执行一次同样的 `odom` 快查（或直接对比窗口前后 health 样本）：每个 remote 源的 `swarm_pose` rx 应继续增长（约 20 s × 10 Hz，允许少量限频/调度偏差），本地 tx 同步增长；窗内非领机源不得出现该话题的 `stall_start` 或 ≥1 s 空档，领机源不得出现 ≥3 s 空档（1–3 s 记 ⚠）。
4. 并发逐机 ping 20 s 记录 RTT 与 ICMP 丢包（命令同冷测第 2 步，去掉 tx/rx 和接收器），作为通信延迟指标；同时做 GCS 与各机计数器采样。
5. 保存 bridge `health_start`、窗口前后 health 样本、各邻机 `swarm_pose` 收发与 stall / link / 协议错误。原始文件在各机 `~/catkin_swarm6-2/.ros_home/log/health/`，测量结束后再下载。
6. 核对 ping 与 `swarm_pose` 检查是否覆盖完整窗口；任一缺源、断口或协议错误写 FAIL。

## 统计口径

- **延迟主指标**：ping RTT min/avg/max/mdev（ms）与 ICMP 丢包率。它不依赖板钟，但 ICMP 往返与 ROS 应用消息延迟不是同一指标。工具输出的 owdelay 不用于绝对值，也不把 RTT/2 当成实测单向延迟。
- **冷测 UDP**：从每个 tx 最终包数和对应 rx 收到包数核对损失；rx 的 seq 区间丢包率漏计首尾全丢，不能只照抄。全窗完整时可用 `(tx包数-rx包数)/tx包数`；有重复/乱序等异常先核查（现工具不去重），不报负丢包率。来源缺失但 tx 证实发送完整时写 100% 丢包。
- **包率/吞吐**：实际电台网卡 `ΔRX_packets/Δt`、`ΔTX_packets/Δt`；Mbps 用 `8×Δbytes/Δt/10^6`。记录每台 UAV，不能只读 GCS——机间流量不经过 GCS。接口统计含探测、MAVLink、TCP ACK 等背景流量；不冒称“纯 bridge 包率”或全网空口 pps。
- **TCP 重传**：同窗 `/proc/net/snmp` 的 `Tcp RetransSegs/OutSegs` 增量，标为整机统计。bridge health 的帧/消息计数不是实际网卡包数，不可直接等同。
- **热测 `swarm_pose` 全互联（主判据）**：每台在线 UAV 的 remote `swarm_pose` 必须包含**全部在线邻机**作为 source，且窗前后各源 rx/tx 持续增长。**非领机源**（1 m 距离门禁，`pose_timeout_s=1 s`）：`age_max ≤1 s`、`gt1000=0`、`stalls=0`、无 `stale_srcs`。**领机源**（跟随环 `LEADER_LOST=3 s`）：`age_max ≤3 s`、`gt3000=0`；1–3 s 空档记 ⚠，不判失败，但出现 ≥3 s 断口或 `LEADER_LOST` 即 FAIL。缺源即该机热测 FAIL，不能用 ping 通替代。注意领机 1–3 s 空档期间该源不满足避碰门禁的 1 s 新鲜度，会暂时不参与 1 m 距离保护。阈值用 `AIR_FLOW_LEADER_TOL_S=3.0` / `AIR_FLOW_PEER_TOL_S=1.0` 覆盖，测试前写入记录。对比窗前后增量时，历史累计 gap 最大值不等于本轮新断口；限频丢弃/`latest` 替换中间样本不等同网络丢包。`mavros/state` 等低频话题的 1 s 级间隔是发布周期，不算链路 stall。板钟仅粗对齐时，跨机窗口筛选有边界误差，须同时保存各机本地起止采样。

## 结果表与判读

现场默认只输出以下延迟表、异常结论和原始证据路径；pps、吞吐、冷测 UDP 损失、`swarm_pose` 缺源明细等放进 `session.md` 支持排查。每台一行，失败/跳过写明原因，空白不用 0 填充：

| UAV | 冷测 RTT min/avg/max（ms） | 冷测 ping丢包 | 热测 RTT min/avg/max（ms） | 热测 ping丢包 | 热测 swarm_pose 接收（源数/应有） | 结果 |
|---|---|---|---|---|---|---|
| UAVn | 待测 | 待测 | 待测 | 待测 | 待测 | 待测 |

先判断本轮是否有效，再判断是否存在延迟回归。**没有项目已验收的统一 RTT 上限，不凭历史 60–90 ms 或预测 pps 自动盖 PASS。** 若本轮另有现场确认的阈值，测试前写入记录，不能看结果后改线。

热测中任一在线机的 `swarm_pose` 缺源、非领机源存在 `stale_srcs` 或 ≥1 s 断口、领机源 ≥3 s 断口或出现 `LEADER_LOST`，直接判 FAIL，停止飞行交接。发现 500 ms 级排队、异常丢包、非领机源 ≥1 s（或领机源 ≥3 s）的机间断口、或协议失配，标异常并停止飞行交接，保留证据定位；高 UDP 压力导致异常时，可停止压力流量补测热测，但须分别报告，不能覆盖原失败结果。无异常写“20 s 地面快测未发现异常，实飞未验证”。20 s 快测不证明 30+ 机容量或长时稳定性。

不自动进行 legacy 回滚或修改合包频率；需要专门对比时另建地面实验，确保全队及 GCS 一致切换并保留原配置。
