# 14–16 架 A/B 联合任务：full_fleet_joint

2026-10-20 第五版：**边缘领机 + 初始 9 架 3×3 + 备用机原地待命/损毁后同批补位 + B 组 3 目标（1 架失败后目标附近 5m 悬停、第 4 架补打）+ 重构后四航带区域覆盖**。完成 16/15/14 三个规模的任务包、离线生成、协议回环、几何/观测器测试和 SITL 录制/渲染管线；实飞未执行。

> **2026-10-21 事故后 14 机配置**（UAV3/UAV7 不可用，UAV13 恢复）见 [14UAV_README.md](14UAV_README.md)。必须用 `--fleet verification/full_fleet_joint/fleet_14_custom.yaml` 生成；仅用 `--available` 会走本文件旧的自动 14 机逻辑（B=4、1 备机），不是本次任务。
>
> **2026-10-21 11 机 2×3 配置**见 [11UAV_README.md](11UAV_README.md)，必须用 `--fleet verification/full_fleet_joint/fleet_11_custom.yaml --available 2,6,8,9,10,11,12,13,14,15,16` 生成；该配置已完成 11 机 SITL（11/11 阶段 SUCCESS）。

## 1. 编组与地面块

| 实际可用总数 | B 组 | 初始 3×3 | 备用机 | 边缘损毁 | 补位后 |
|---:|---:|---:|---:|---:|---:|
| 16 | 4（UAV5/10/15/16） | 9（UAV1/2/3/6/7/8/11/12/13） | 3（UAV4/9/14） | 3（UAV3/8/13） | 9 |
| 15 | 4（UAV5/10/14/15） | 9 | 2（UAV4/9） | 2（UAV3/8） | 9 |
| 14 | 4（UAV5/9/10/14） | 9 | 1（UAV4） | 1（UAV3） | 9 |

- 上表是按 UAV1–15、UAV1–14 连续可用集自动选出的备份编组。B 先从备用列之外的剩余机中选；不足 4 架时从备用列底部往上补（先 UAV14、再 UAV9），靠前的备用机留给边缘列补位。
- 某架故障但另一架可用时，用 `--replace UAV13=UAV16` 让替换机顶替故障机的地面机位（替换机从自己的原机位移除）。本次 15 架实飞（UAV13 故障、UAV16 顶替）见 [15UAV_README.md](15UAV_README.md)。
- 若当天缺失的是备用列中的某架（如缺 UAV14），自动 B 仍为 UAV5/10/15/16，备用数按“可用总数 − 9 − 4”重算，损毁数同步减少。

- 初始 3×3 = 地面第 1–3 排、第 1–3 列；**领机选左边缘中排 UAV6**（远离中心、远离右侧牺牲列），编队只前移 1m 起飞，不做横向重排。
- 备用机 = 相邻第 4 列 UAV4/9/14，**起飞后原地 5m 不动**；边缘列损毁后才飞到 coverage1 终点的空位补位。
- 损毁机 = 3×3 右边缘列 UAV3/8/13，从 coverage1 终点向左退出 20m 到 8m（远离右侧 B 目标）。
- B 组 = 块和备用机之外的剩余机；默认 UAV5/10/15/16，可 `--b` 指定 4 架。**B 需要 4 架：3 架打击 + 1 架补打**；与地面块/备用机冲突时拒绝出包。
- 初始 9 架必须在地面完整可用；缺块内任意一架时生成器拒绝出包。

## 2. 坐标、覆盖航带与高度

任务坐标右 x / 前 y / 上 z；原点 = 标定计算排中心。默认左块（领机 UAV6）：

```text
第一排 y=+5 : UAV1  UAV2  UAV3  | UAV4  UAV5
第二排 y= 0 : UAV6  UAV7  UAV8  | UAV9  UAV10
第三排 y=-5 : UAV11 UAV12 UAV13 | UAV14 UAV15
第四排 y=-10: UAV16
              x=-10 -5 0 +5 +10
初始 3×3: x=-10/-5/0, y=-5/0/+5；领机 UAV6 (-10,0)
备用列: x=+5；B: x=+10（UAV5/10/15）+ UAV16
```

- **四航带 boustrophedon**（参考 references/runtime 的 Fields2Cover 思路，简单化）：
  - 航带 1（coverage1，初始 9 架）：领机 (-10,1)→(-10,61)。
  - 航带 2（coverage2，恢复 9 架）：横向 +12m 到 (2,61)，前飞 (2,121)。
  - 航带 3（coverage2）：横向到 (14,121)，后飞 (14,61)。
  - 航带 4（coverage2）：横向到 (26,61)，前飞 (26,121)。
  - 航带横向步距 12m（3×3 扫描带宽 18m，重叠 6m）；覆盖区域约 x=−14..+40、y=−4..126，面积约 54m×130m ≈ 7000m²；领机覆盖航程约 264m。
- 高度：A 覆盖 **12m**；A prepare MOVE_TO 转场 15m；B 转场 **8m**、打击 **5m**；损坏机退出 **8m**。
- 损坏机在 coverage1 终点（x=0，y=56/61/66）下降到 8m，向左退出到 x=−20；备用机从地面 5m 起飞（原地）后飞到 (0,56/61/66) 补位。
- B 目标在侦查航带内：`(-8,50)`（航带1）、`(10,80)`（航带2）、`(24,80)`（航带3）；地面 z=0、飞行 z=5。
- A 返航从 B 上方通过，垂直间隔 **7m**；B 转场 8m 对 A 覆盖 12m 有 4m 垂直间隔。
- 场界候选 x=−50..+50、y=−15..+140；**实飞前必须实测后写回 `fleet.yaml`**。

## 3. 流程（11 个逻辑阶段）

```text
prepare-a-1（3×3 左边缘列 3 架，前移 1m 到 12m）
→ prepare-a-2（3×3 中/右列 6 架）
→ prepare-spares（备用机原地 5m 观察，不动）
→ prepare-b-hold（B 组 4 架保持 5m）
→ coverage1（初始 9 架航带 1：60m）
→ fault（边缘列 3 架下降 8m，向左退出 20m）
→ recovery-spares（2–3 架备用机同批飞到航带1终点空位补位；最近约 2.7m）
→ recovery-hold（观察恢复后的 3×3）
→ coverage2（3×3 航带 2+3+4：60m 前飞 + 横向 12m + 60m 后飞 + 横向 12m + 60m 前飞）→ 本地 RECON_COMPLETE
→ strike（B 组 3 架到航带内目标上方 5m）
→ strike_fail（1 架模拟失败，在目标附近 5m 悬停）
→ strike_reinforce（第 4 架从起点补打失败目标）
→ strike_hold（3 架在目标上方 5m 保持 3s）→ 本地 STRIKE_COMPLETE
→ return_a（A 组 9 架返回 12m；从 B 上方 7m 通过）
→ hold（A 12m、B 3 架目标上方 5m、失败 B 目标附近 5m、损坏 A 8m）
→ 飞手逐机接管降落
```

- 不使用 HOVER；保持阶段只用 observe-only + 逐机实测位置核对。
- 任一阶段失败/超时/切出 OFFBOARD：停止后续，TCP HOLD + 逐机 `/UAVn/uav_hold`，不空中自动重试。

## 4. 标定

- 计算集取一排的最大可用对称子集（≥3 架），优先第 2 排 UAV6–10；默认原点 = UAV8 机位（UAV8 是右边缘损毁机，地面标定后不影响）。
- 偶数对称子集允许不含中心机；换排时所有任务几何按 `shift_y` 平移。
- `all_uavs` = 本轮全部参与机；`sample_window_s=8.0`；找不到 ≥3 架对称子集时拒绝出包。
- 起飞前人工逐机核对 `gp_origin_confirmed`，尤其备用机和 B 组。

## 5. 文件与生成

| 文件 | 作用 |
|---|---|
| `fleet.yaml` | 场界、地面块、备用列、四航带参数、B 目标、高度、几何阈值 |
| `generate_mission.py` | 按 `--available`/`--b` 生成任务包、编队/标定、`mission_spec.json`、逐机命令、manifest，并做几何/离线排程校验 |
| `driver.py` | 数据驱动 11 阶段；初始 9→损毁→备用同批补位→四航带；B 3+1 失败/补打 |
| `flight_guard.py` | 只读观测；期望终点来自 `mission_spec.json` |
| `flow.sh` | `generate/onboard-commands/odom/bag/stage1/dry-run/run` |
| `gcs_real.launch` | 替换生成编队/标定配置，UDP 合包 |
| `test_full_fleet.py` | 16/15/14（含 1–15、1–14 连续可用集）、备用缺失、B 指定、边缘领机、四航带、B 失败/补打、协议回环、观测器 |
| `sitl/` | 出生 launch、一键 PX4/Gazebo 录制、三维视频渲染，见 [sitl/README.md](sitl/README.md) |

```bash
bash verification/full_fleet_joint/flow.sh generate --available 1-16   # 16 机
bash verification/full_fleet_joint/flow.sh generate --available 1-15   # 15 机备份
bash verification/full_fleet_joint/flow.sh generate --available 1-14   # 14 机备份
# UAV13 故障、UAV16 顶替其机位：
bash verification/full_fleet_joint/flow.sh generate \
  --available 1-12,14,15,16 --replace UAV13=UAV16
# 指定 B（必须 4 架）：--b UAV5,UAV10,UAV15,UAV16
python3 -m unittest verification.full_fleet_joint.test_full_fleet -v
bash verification/full_fleet_joint/flow.sh dry-run
bash verification/full_fleet_joint/flow.sh onboard-commands
```

2026-10-20 第五版离线结果：**22 项测试通过**。几何检查：3×3 槽距 5m；B 三目标全部落在四条航带内；B 路径对 A coverage2 终点最小 3D 36.3m；A 返航对 B 垂直 7m；损坏退出点对 B 路径最小水平 13.4m（16 机）；损坏机对编队垂直 4m；prepare 路径最小 5m；MOVE_TO 最小位移 1m。离线排程（16 机）：prepare-a-1/a-2 3.4s、recovery-spares 89.4s、mock-b-strike 125.1s、mock-b-fail 10.6s、mock-b-reinforce 133.7s、fault-exit 30.6s，均低于阶段超时；15/14 机的排程量级相同。

## 6. 通信、部署与 GCS

当前按 UDP 部署；启动时 GCS 和每架机显式传 `bundle_transport:=udp`，逐机核对 `transport=udp`。A 组 9 架参与编队，备用 3 架在编队外待命/补位，B 组 4 架独立 MOVE_TO。

- 每轮 20s 热测；0% ping 丢包、`swarm_pose` 覆盖在线 peer；非领机源无 `gt1000`/stall/stale，领机源无 `gt3000`（1–3 s 空档记 ⚠）。
- full-fleet GCS 的 `confirm_wait_s=1.5`（组接纳仍 4s、TCP ACCEPT 5s 内），用于 B 打击期间 A 组服务空闲后的重连。
- `flow.sh onboard-commands` 只打印；A/备用默认层高，B 带 `layer_move_to:=8.0`，全机带 `bundle_transport:=udp`。
- GCS：

```bash
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
           roslaunch verification/full_fleet_joint/gcs_real.launch'
```

- 全部 disarm 时标定；任何机载栈重启、换电或飞机移动后，必须重启 GCS 标定节点重新标定。

## 7. 实飞步骤

1. 实测可用空域与净空，更新 `fleet.yaml`；按当天实际可用机重新 `generate`（必要时 `--b`）。
2. 核对身份、router、板钟、机载部署指纹和单 setpoint 发布者。
3. 按生成命令逐机起机载栈；核对 B 组层高 8m、其余默认层高、`transport=udp`。
4. 地面 20s 热测；未通过不下发任务。
5. 起 GCS、标定、核对 ready 与逐机原点。
6. 飞手让全部参与机人工进入 armed/OFFBOARD、约 5m 稳定悬停；初始 9 架在 3×3 地面块、备用机在相邻列原地、B 组在剩余机位。
7. 先 `flow.sh run --until prepare`，核对 3×3 起始线、备用机原地 5m、B 组 5m；再全流程：

```bash
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
           bash verification/full_fleet_joint/flow.sh run'
```

driver 会自动在 `src/uav_log/<时间>_full-fleet-joint/driver.jsonl` 落盘；需要固定证据目录时再加 `--log <路径>`。

8. `hold` 后飞手逐机接管；B 组按现场指定安全点/起降区降落。
9. 失败、超时、断线或切出 OFFBOARD 即中止并请求 HOLD。

## 8. 验收与已知风险

- 完整通过：11 个逻辑阶段成功；初始 9 架 3×3、3/2/1 架边缘损毁、备用机同批补位后仍 3×3；四条航带覆盖；`RECON_COMPLETE` 之后才出现 B 动作；B 3 架首次打击、1 架失败后在目标附近 5m 悬停、第 4 架补打，最终 3 架在目标上方 5m；A 返回 12m；全程无破距/规划/链路/模式异常。
- `mission_finished=SUCCESS` 只代表本轮驱动与观测条件通过，不等于 30+ 架或真实 B 组已验收。
- **已知最紧点**：备用机同批补位时平行航线最近约 2.7m（离线估计；SITL 实测 16/15/14 机分别为 4.43/2.97/3.01m，用户判断不会碰撞）；编队槽距 5m；损坏机在编队下方 4m 通过；A 返航从 B 上方 7m 通过。实飞前必须确认机载安全阈值与现场净空。
- B 目标是侦查航带内的模拟目标；A 返航会从 B 上方通过，这是本轮刻意设计的“侦查发现目标”场景。
- 初始 9 架必须在地面完整；B 固定 4 架，备用/损毁数 = 可用总数 − 9 − 4（16→3、15→2、14→1）；不足 4 架 B 时从备用列底部补足。
- 16 机 UDP 未实测（14 机已通过）；热测是硬门禁。四条航带使 coverage2 约 216m、返航约 125m，机载任务超时已改 300s。

## 9. 2026-10-20 三规模 SITL 结果

| 规模 | 运行目录 | 状态 | 任务时长 | recovery 最小 3D | coverage2 最小 3D | 视频 |
|---:|---|---|---:|---:|---:|---|
| 16 | `.tmp/full_fleet_joint/2026-10-20_124100_sitl16` | SUCCESS | 895s | 4.43m | 3.71m | `full_mission_3d.mp4`（114s，8×） |
| 15 | `.tmp/full_fleet_joint/2026-10-20_131408_sitl15` | SUCCESS | 880s | 2.97m | 3.76m | `full_mission_3d.mp4`（112s，8×） |
| 14 | `.tmp/full_fleet_joint/2026-10-20_133140_sitl14` | SUCCESS | 883s | 3.01m | 3.73m | `full_mission_3d.mp4`（112s，8×） |

- 三套均为 11 个逻辑阶段全部 SUCCESS；B 组 3 架打击、1 架失败目标附近 5m 悬停、第 4 架补打，最终 3 架在目标上方 5m。
- 15 机首跑曾因 `gcs_sim.launch` 固定读 `generated/config` 报 `FORMATION_PARTICIPANTS_MISMATCH`；现已按 `--spec` 同目录加载编队/标定配置并加入回归测试。
