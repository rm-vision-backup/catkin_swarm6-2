# 日志归档 192.168.55.1_2026-10-10_1734

- 采集时刻：2026-10-10 17:34:49 CST（本机时钟；多目标共用同一 stamp）
- 目标：192.168.55.1 / 192.168.55.1 / 飞控 MAVLink -
- 机载：host=UAV8，板钟 epoch=1791624883（2026-10-10 17:34:43 CST）
- 命令：LOG_OUT=src/uav_log/2026-10-10_1706_three-uav-full log_fetch.sh nvidia@192.168.55.1
- 结果：落地 15 项，FAIL 0 项，缺失/截断 0 项

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
