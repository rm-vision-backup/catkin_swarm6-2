# 日志归档 UAV8_2026-10-10_1524

- 采集时刻：2026-10-10 15:28:46 CST（本机时钟；多目标共用同一 stamp）
- 目标：UAV8 / 192.168.5.81 / 飞控 MAVLink 8:1
- 机载：host=UAV8，板钟 epoch=1791617064（2026-10-10 15:24:24 CST）
- 命令：LOG_OUT=src/uav_log/2026-10-10_1515_three-uav-air-stage1 log_fetch.sh 8 10 13
- 结果：落地 22 项，FAIL 0 项，缺失/截断 1 项

## 内容

| 路径 | 来源 | 说明 |
|---|---|---|
| （本次未采飞控 ulog：默认只采我们程序记的日志；需要时加 `--ulog`，或自行接线读） | — | — |
| `onboard/ros_home/<session>/` | ssh | ROS 会话日志（rosout、各节点、master、roslaunch） |
| `onboard/ego_planner/*.log` | ssh | planner 诊断（机上超 32 MB 时只留尾部，见 manifest） |
| `onboard/term/*_offboard_ego.log` | ssh | 终端全量转录（way B），漏设 `UAV` 时文件名为 `_offboard_ego.log` |
| `onboard/health/*.jsonl` | ssh | 数据面观测：bridge per-topic 到达间隔/停顿/连接事件、执行器跟随环与 relay 事件（超 32 MB 只留尾部，首行可能半行） |
| `onboard/ssh.err` | ssh | 仅在有 stderr 输出时存在 |
| `manifest.txt` | — | 逐项状态/来源/字节/md5（`MISSING`/`FAIL` 都在这里） |
| `md5sums.txt` | — | `md5sum -c` 校验用 |

## 复现

```bash
cd "$(dirname "$0")" 2>/dev/null || cd .
md5sum -c md5sums.txt
less manifest.txt                # 缺什么、为什么缺
less manifest.txt                # 缺什么、为什么缺
```
