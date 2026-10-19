# 2026-10-18 17:47 六机联合任务（换电后重标）

- 机队：UAV6–11；标定计算集 UAV6–10，确认集六架。
- 换电池后重新上电；GCS 栈重启以重新冻结原点。

## 检查

| 项目 | 结果 |
|---|---|
| 六机在线 | PASS |
| router（enabled/active/单实例/参数） | PASS：仅 `127.0.0.1:15500` |
| 板钟（同时刻比对） | PASS：偏差 −1…+1 s |
| 机载栈启动 | PASS：六架 17:45 启动 |
| disarm / GPS | PASS：六架 armed=False，fix_type=6 |
| 机头朝向（6–10） | PASS：−75.23…−66.48°，差 8.75°（门限 30°） |

## 标定（17:46:41 冻结）

- 计算集：UAV6、UAV7、UAV8、UAV9、UAV10
- `gp_origin`：lat 38.4984752, lon 103.2062572, alt 1344.752490
- `reference_heading`：−1.233864 rad（−70.69°）
- EKF 回读：六架一致（alt 1344.751906）
- `/group_a/calibration_ready` = true；六架 `gp_origin_confirmed=True`

## 下一步

- 飞手六机 arm/OFFBOARD、升到约 5 m 稳定悬停。
- 起飞后 `flow.sh odom` 快查。
- 运行 README §5.7 的 driver 命令（自动新建本轮日志目录）。
