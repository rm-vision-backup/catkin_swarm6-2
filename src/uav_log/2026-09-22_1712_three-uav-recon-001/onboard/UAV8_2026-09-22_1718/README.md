# 日志归档 UAV8_2026-09-22_1718

- 采集时刻：2026-09-22 17:24:32 CST（本机时钟；多目标共用同一 stamp）
- 目标：UAV8 / 192.168.5.81 / 飞控 MAVLink 8:1
- 机载：host=UAV8，板钟 epoch=1790068699（2026-09-22 17:18:19 CST）
- 命令：LOG_OUT=/home/yjq/catkin_swarm6-2/src/uav_log/2026-09-22_1712_three-uav-recon-001/onboard log_fetch.sh -n 1 8 10 13
- 结果：落地 16 项，FAIL 1 项，缺失/截断 0 项

## 内容

| 路径 | 来源 | 说明 |
|---|---|---|
| `fcu/ulog_list.txt` | MAVLink | PX4 `LOG_ENTRY` 全清单（含 GPS UTC 真时刻） |
| `fcu/log_*.ulg` | MAVLink | PX4 ulog（`-n N` 指定的最新 N 条；断点续传） |
| `fcu/ulog_fetch.log` | MAVLink | 下载过程日志（进度 / stall 重试痕迹） |
| `onboard/ros_home/<session>/` | ssh | ROS 会话日志（rosout、各节点、master、roslaunch） |
| `onboard/ego_planner/*.log` | ssh | planner 诊断（机上超 32 MB 时只留尾部，见 manifest） |
| `onboard/term/*_offboard_ego.log` | ssh | 终端全量转录（way B），漏设 `UAV` 时文件名为 `_offboard_ego.log` |
| `onboard/ssh.err` | ssh | 仅在有 stderr 输出时存在 |
| `manifest.txt` | — | 逐项状态/来源/字节/md5（`MISSING`/`FAIL` 都在这里） |
| `md5sums.txt` | — | `md5sum -c` 校验用 |

## 复现

```bash
cd "$(dirname "$0")" 2>/dev/null || cd .
md5sum -c md5sums.txt
ulog_info fcu/log_*.ulg          # duration / No Dropouts
less manifest.txt                # 缺什么、为什么缺
```
