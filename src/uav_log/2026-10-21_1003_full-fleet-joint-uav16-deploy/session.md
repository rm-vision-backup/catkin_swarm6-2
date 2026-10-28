# 2026-10-21 15 架机载部署检查（UAV13 不参飞）

机队：UAV1–12、UAV14、UAV15、UAV16（15 架）。范围：板钟 / router / 残留 / 4 个机载包同步与构建。未启动机载栈，未标定，未飞行。

## 汇总

| 项目 | 结果 |
|---|---|
| 连通性 | 15/15 在线；UAV4/UAV15 上电初期有间歇丢包，重试后恢复 |
| 板钟 | **15/15 PASS**；UAV7/9/12/16 修前为 1970 并已步进，其余在容差内 |
| router | **15/15 PASS**：enabled、active、单实例、`-e 127.0.0.1:15500 /dev/ttyTHS0:921600`、NRestarts=0 |
| 残留 | 15/15 CLEAN（无 roslaunch/rosmaster/mavros/bridge，4200/11311 未监听） |
| 代码同步 | 15/15 完成；机载无额外本地改动（dry-run 无删除项） |
| 构建 | 15/15 全部 `All 4 packages succeeded`，`BUILD_EXIT=0` |
| 同步校验 | **15/15 checksum_diff=0** |

同步包：`swarm_uav_interfaces`、`swarm_topology_bridge`、`swarm_uav_executor`、`ego_planner_driver`。UAV16 先前已同步（本次 changed=0）；其余 14 台各更新 4–15 个文件，主要包含 `topology.yaml`、`fleet_monitor`、`ego_swarm.py`、`uav_offboard_ego.launch`。

## 逐机记录

- 时钟：`clock_all.log`
- router：`router_UAV<n>.log`
- 残留：`residual_UAV<n>.log`（UAV4/UAV15 见对应 router 日志）
- 同步：`sync_UAV<n>.log` / `dryrun_UAV<n>.log`
- 构建：`build_UAV<n>.log`
- 校验：`verify_all.log`

## 备注

- UAV7/9/12/16 上电后系统钟回到 1970，已修正并写 RTC；若再次复现，按 UAV7/UAV9 先例查 CR1220。
- router journal 的串口 latency 警告与 `/etc/mavlink-router/main.conf` 缺失为各机共有的历史噪声，无重启循环。
- UAV13 未上电、未检查、未同步。
- 下一步（未执行）：起机载栈、20 s 热测、GCS 标定、逐机 `gp_origin_confirmed` 核对。

## 2026-10-21 10:23 机载启动与热测

- 启动：15/15 PASS；就绪检查 15/15 `connected=True, armed=False`（UAV16 为 ALTCTL，其余 AUTO.LOITER）。
- 20 s 窗口（10:25:55–10:26:15）：15/15 无 stall/link 事件；本机 swarm_pose tx 10.0/s、rx 30/s；收到邻机 srcs=14、rx 127–140/s、age_max 85–165 ms；UAV10/14/15 的样例增量 gt1000+0、stalls+0、stale=0。
- **工具判定为 Stage 1 不通过，原因是 `stage1` 逐机串行读取把后段窗口拉长到 60–70 s**；实际首个链路事件发生在 10:26:49（窗口结束后 34 s），之后 1.5–3.4 s 的多机 stall 持续到约 10:27:34，主要涉及 UAV10/11/12/9，多个接收机同时受影响。
- 结论：**名义 20 s 窗口干净，但窗口后立即出现成片链路 stall，不能据此认为链路稳定**。与已知电台包率瓶颈一致（15 节点全互联 bundle 约 2400 pps，且 topology 仍把未上电的 UAV13 作为目的端）。
- 机载栈保持运行中。证据：`start_all.log`、`ready_all.log`、`hot_swarm_pose_20s.log`、`events_UAV*.log`。

## 2026-10-21 10:35 复测（并行窗口，主要看 swarm_pose）

- 方法：15 台并行取 T0 快照 → 20 s → 并行取 T1 快照；窗口取各机自身 `t0..t1`（实测约 28 s），只统计窗口内事件。预期 `srcs=14`（无 UAV13）。
- **结果：不通过。**
  - UAV11 SSH 两次失败；测试结束时 UAV11 ping/SSH 均不可达。
  - 14 台可读机 `srcs=14`，但窗口内 `gt1000` 和 `stalls` 普遍增加，出现大量 1.5–3.3 s stall。
  - 前 20 s 内就有：UAV1(+2s)、UAV14(+4s)、UAV16(+14s)、UAV2(+16s)、UAV3(+14s)、UAV10(+16s)、UAV12(+16s)、UAV4/UAV6/UAV7(+18s)；源端以 **UAV11** 为主，另有 UAV10/UAV12/UAV14。
- UAV12 ping 100% 丢包但 SSH 可用且机载栈在跑；该链路 ICMP 不能单独作为判据。
- 结论：本轮 `swarm_pose` 不满足任务前提，主因指向 UAV11 的机间链路/机载连通性，需现场检查 UAV11 后再测。
- 证据：`hot_retest/`（`t0_*`、`t1_*`、`ev_*`）。

## 2026-10-21 10:48 板钟复查与重启机载

- 只读板钟检查：15/15 正常（与 GCS 差 0–5 s，均为查询时延）。证据 `clock_readonly.log`。
- 启动前状态：15/15 `roslaunch=0, rosmaster=0`（DOWN）。证据 `procs_before_start.log`。
- 按生成命令启动：15/15 PASS（10:49:22–10:50:24）。证据 `start_all_1049.log`。
- 本轮未做 swarm_pose 复测。

## 2026-10-21 11:14 断电重启后 swarm_pose 复测

- 15/15 机载就绪（connected=True, armed=False）。
- 并行窗口（约 32 s）：**FAIL**。
  - UAV1、UAV12 `srcs=13`（缺 UAV4）；其余 `srcs=14`。
  - 窗口内多处 1.5–3.5 s stall：源端以 **UAV4** 为主，另有 UAV5/UAV10/UAV16/UAV2/UAV1。
  - UAV4 在测试中 SSH 失败；测试结束后 ping/SSH 均不可达。
  - 多数机 `gt1000/stalls` 在窗口内增加。
- 结论：复测不通过。断电重启后弱链从 UAV5 转移到 UAV4，呈现全网链路余量问题，而非单机软件问题。
- 证据：`hot_retest2/`（`t0_*`、`t1_*`、`ev_*`）。

## 2026-10-21 11:19 UAV4 重启后复测

- UAV4 时钟正常（RTC 保持），机载启动 PASS；窗口内 **UAV4 自身 PASS**（srcs=14、age_max 99 ms、无 stale）。
- 整网仍 **FAIL**：8 台 PASS（UAV4/5/6/7/9/11/15/16），7 台 FAIL（UAV1/2/3/8/10/12/14）。
- 失败呈成对链路特征：
  - UAV1 ↔ UAV12：双方 `NOT SEEN`（bridge 启动以来从未收到）；
  - UAV3 ↔ UAV8：曾收到约 2449/2412 帧，约 180 s 前双向同时中断；
  - UAV1 ↔ UAV14：2.5–2.8 s stall，age 4.5 s / 14.8 s；
  - UAV2 ↔ UAV10：反复约 2 s stall（stalls 30/9，gap_max 20.2 s）。
- 注意：`srcs=14` 包含 stale 源，不能单独作为通过判据；必须同时看 `stale_srcs`/age。
- 证据：`hot_retest3/`。
