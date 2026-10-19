# 2026-10-19 12:29 六机联合任务实机轮（全机断电重启后）

状态：进行中 — 时钟/router 已核对，机载栈启动中

## 背景

- 上午轮次记录：`src/uav_log/2026-10-19_1017_six-uav-joint/`。上午出现 UAV7 自行重启、UAV6/UAV9 随机失联，现场对**全部六机断电重启**后开始本轮。
- 机队：UAV6–11（A06–A11）；领机 A07；UAV6 故障机、UAV11 备用、A09/A10 模拟 B 组。
- 任务入口：`verification/six_uav_joint/`；GCS launch `gcs_real.launch`。
- 代码基线：主仓 `4a5f290` + 未提交改动；`tcp_to_ros` `1e316d5` + 本轮 `schedule_pose_timeout_s=2.0`；bridge `73db782`；executor `ccc5afe`。

## 上电后检查（12:29）

| UAV | 板钟（GCS 12:29:29） | router | 残留 |
|---|---|---|---|
| UAV6 | 正确（epoch 1792384169） | enabled/active，MainPID 1027，单实例，NRestarts=0，仅 `127.0.0.1:15500` | 0 |
| UAV7 | **1970 → 已修正**（12:29:42，rtc0 +3 s、rtc1 +1 s，PASS） | enabled/active，MainPID 1012，单实例，NRestarts=0 | 0 |
| UAV8 | 正确 | enabled/active，MainPID 1022，单实例，NRestarts=0 | 0 |
| UAV9 | **1970 → 已修正**（12:29:47，rtc0 +3 s、rtc1 +1 s，PASS） | enabled/active，MainPID 1026，单实例，NRestarts=0 | 0 |
| UAV10 | 正确 | enabled/active，MainPID 1036，单实例，NRestarts=0 | 0 |
| UAV11 | 正确 | enabled/active，MainPID 1011，单实例，NRestarts=0 | 0 |

- **重要发现**：UAV7/UAV9 在上午已写入正确 RTC（10:21），但整机断电后仍回到 1970 → 按 `JETSON-RTC-TIME-RUNBOOK.md` 属“RTC 电池未保住”，建议本轮结束后更换 CR1220。本轮已重新写入 RTC，当前时钟正确。
- 所有机载栈均未运行（residual=0），符合断电重启后状态。

## 标定核对（12:36–12:38）

- `calibration_ready=True`；计算集 5/5 确认：UAV6、UAV7、UAV8、UAV9、UAV10；`all_uavs` 含 UAV11。
- 冻结原点：`lat=38.4984469, lon=103.2062679, alt=1345.4257558`；参考航向 `-1.2417881489542273 rad`。
- 六机 `/mavros/global_position/gp_origin` 回读：lat/lon 完全一致，alt=1345.4249843（与冻结值差 **0.77 mm**）；六机 `gp_origin_confirmed=True`（含 UAV11）。
- 实际加载：`formation_offsets.yaml`、`calibration_uavs=UAV6–10`、`all_uavs=UAV6–11`、`schedule_pose_timeout_s=2.0`、`packing=bundle`、TCP 39001 UP。
- 六机当前 connected=True、armed=False、mode=AUTO.LOITER（未起飞）。
- 结论：本轮标定正确、可用。**若机载栈重启、换电或移动飞机，必须重新标定。**

## 步骤状态

| 步骤 | 结果 |
|---|---|
| 时钟/router | PASS（UAV7/9 已修正） |
| 机载栈启动 | **PASS**（12:30:17–12:30:19，六机 `STARTED=1`；日志 `start_*.log`） |
| 热测 | **SKIP**（现场负责人明确决定不做，理由：直接进入标定/飞行） |
| GCS/标定 | **PASS**（12:36:44 起栈；`calibration_ready=True`；六机原点回读一致） |
| 起飞/driver | 未开始 |
