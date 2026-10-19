# 2026-10-18 18:06 六机联合任务（UAV9/10 换电后重标）

- 原因：UAV9/UAV10 电量耗尽，更换电池后重新启动这两架机载栈。
- UAV9：在线，板钟相对 GCS +1 s，router enabled/active。
- UAV10：更换电池后短暂不可达（启动中），随后恢复；板钟 −1 s，router enabled/active。
- 两架机载栈分别于 18:05:02、18:05:19 启动；其余四架保持运行。

## 标定（GCS 重启后重新冻结）

- 六架全部 disarm，POSCTL；航向最大偏差 14.7°（门限 30°）。
- `gp_origin`：lat 38.4984778, lon 103.2062802, alt 1344.716692
- `reference_heading`：−1.403412 rad（−80.41°）
- `/group_a/calibration_ready` = true
- 六架 `gp_origin_confirmed=True`，EKF 原点回读一致（alt 1344.715974）

## 下一步

- 飞手 arm/OFFBOARD，升到约 5 m 稳定悬停。
- 运行 README §5.7 的 driver 命令（含 RUN_DIR 三行；自动建立本轮日志目录）。
- driver 已不再自动检查 `gp_origin_confirmed` 与起始位置；由操作员起飞前人工核对。
