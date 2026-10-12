# 2026-10-10 下午轮：换装后整任务飞行（三机 prepare + coverage 成功）——证据包

> 现场：UAV8/10/13；GCS_A 本机 `gcs_a_real_three_uav.launch`。
> **本轮结果：`air_flow.sh full` → prepare SUCCESS（A08/A10/A13）+ coverage SUCCESS（3/3）——整任务首次成功。**

## 时间线（下午轮）

| 时刻 | 事件 |
|---|---|
| 16:32 | P0 部署生效：router 去 242 重装、topology odom 12 Hz、executor `_FOLLOW_POSE_TIMEOUT_S=3.0` 上板 |
| 16:38–16:41 | 三机曾处于 ~1.5 m 簇状摆位（13 机头反向）→ 地面快测暴露 8↔13 遮挡级退化（断连 + 累计 34 s 空档，非飞行布局，作废） |
| ~16:41 | 现场重摆 → 重新标定冻结（原点 A） |
| 16:43 | 地面 `stage1 60` **通过**（新配置首次地面全绿） |
| 16:53–16:58 | **空中第一窗 60 s 通过**（gt1000=0/stalls=0）→ 电池告警降落 |
| 16:50–16:52 | UAV13 重启（对方）→ 重起 13 栈；GCS 重启，重标定（原点 B） |
| 16:58–17:06 | 换电池、整组挪位 ~23 m → 重起三栈 + GCS，重标定（原点 C＝当前机位中位数，已复核一致） |
| 17:07:46 / 17:08:2x | `odom` 快查 + `stage1 30` **通过** |
| **17:08:38** | 操作员运行 `air_flow.sh full` → **prepare SUCCESS + coverage SUCCESS（3/3）** |
| ~17:19 | 三台 bag 干净停止（SIGINT 收索引） |

## 结果

- **任务**：`driver/air_full_20261010_170838.jsonl` —— `prepare`（A08/A10/A13 = SUCCESS）+ `coverage`（3/3 SUCCESS）。
- **链路**：本轮全部 `stage1` 窗（空中 60 s、地面、快速 30 s）均 **通过**（三台 `gt1000=0`、`stalls=0`、无事件；收邻机 23.4/s、发 11.7/s）。
- 与今早基线对比：今早（旧配置）6 分钟 8 次 1.1–5.2 s 停摆；下午（去 242 + odom 12 Hz 后）观测窗内零停摆。**单轮样本仍小，结论以后续复测累计为准。**

## 内容

| 路径 | 说明 |
|---|---|
| `rosbag/raw_{81,101,131}/` | 今日下午全部包：`1638xx`（簇摆期，作废参考）/`1643xx`（重摆+重标定+地面窗）/`1653xx`（地面+空中第一窗 60 s）/`1706xx`（**整任务**，13.6 MB/台） |
| `driver/air_full_20261010_170838.jsonl` | `full` 的任务下发与阶段判定原始记录 |
| `gcs/` | `GCS_A_bridge_health.jsonl`（GCS 侧同刻视角）+ GCS roslaunch 会话 + 三次启动日志（165410/170318/170542） |
| `UAV8_2026-10-10_1734_usb/` `UAV10_2026-10-10_1733_usb/` `UAV13_2026-10-10_1736_usb/` | log_fetch 采集（onboard/{health,ros_home,ego_planner,term}；不含 ulog）。**经 USB 数据线直收**（比电台快 ~500×）。UAV13 的 health 有一处 SIZE-MISMATCH 属采集时文件正在追加的竞态（落地 13 546 131 B，md5 在 manifest 里），无碍 |

**包覆盖注意**：UAV13 缺 `1643xx` 包（当次 `record_rosbag.sh` 静默未起，16:43–16:53 无包；该时段由 UAV8/UAV10 包 + 三台 health 覆盖）。UAV13 的 `163819` 包为 16:43 干净停止（非断电截断，无需 reindex）。上午轮的包在 `2026-10-10_1515_three-uav-air-stage1/`。

**核对**：11 份 bag 与板端字节数逐一比对**全部一致**（4+4+3）；driver JSONL、GCS 三件套、三台日志目录均已落地（本轮归档共 134 MB）。

## 后续分析入口

- 跟随环事件：各机 `onboard/health/UAVn_follower_events.jsonl`（看 `follower_start` → `leader_odom_first_frame` 的 wait_s、有无 `LEADER_LOST`/`leader_odom_gap`）；
- 逐链路到达空档：按 `2026-10-10_1515_…/README.md` 的口径复算（bag 接收时间戳法）；
- P1（UDP 打流）与 P2（bridge UDP 化）见 `quality_reports/plans/synthetic-toasting-hellman.md`。
