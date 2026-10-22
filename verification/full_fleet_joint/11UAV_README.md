# 11 架 2×3 编队任务（2026-10-21）

## 1. 地面摆位

物理网格（右 x / 前 y，5m）：

```text
第 1 排 y=+5 : UAV6  UAV8  UAV9
第 2 排 y= 0 : UAV10 UAV11 UAV12
第 3 排 y=-5 : UAV13 UAV14 UAV15 UAV16 UAV2
              x=-10  -5    0    +5   +10
```

标定排中心为 UAV8（物理 `(-5,+5)`），任务坐标以它为原点：
`shift_x=+5`、`shift_y=-5`，即任务坐标 = 物理网格 − `(-5,+5)`。

```text
任务坐标（右 x / 前 y / 上 z）
第 1 排 y= 0 : A06 A08 A09
第 2 排 y=-5 : A10 A11 A12
第 3 排 y=-10: A13 A14 A15 A16 A02
              x=-5   0   5  10  15
```

不参飞：UAV1、UAV3、UAV4、UAV5、UAV7。

## 2. 角色

| 角色 | 飞机 | 说明 |
|---|---|---|
| 初始 2×3（6） | A06/A08/A09、A10/A11/A12 | 两排三列；领机 A06 |
| 模拟损毁（2） | A10/A11 | 降至 8m 后向左退出编队外侧（x=-35/-30） |
| 备用补位（2） | A13/A14 | 补 A10/A11 的槽位，恢复 2×3 |
| B 组（3） | A15/A16/A02 | A15、A16 首次打击；A15 失败；A02 补打 |
| 不参飞 | UAV1/3/4/5/7 | 不上电、不接入机载栈 |

- 打击目标（任务坐标）：目标1 `(10,65)`，目标2 `(0,65)`。
- A15 → 目标1，A16 → 目标2；A15 失败后悬停 `(4,65,5)`，A02 补打目标1 `(10,65,5)`。
- 标定计算集：第 1 排 UAV6/8/9（中心 UAV8）；`shift_x=+5`、`shift_y=-5`。profile 的 `b_targets` 中 x 直接按任务坐标配 `10/0`；y 配 `70`，减 `shift_y` 后任务坐标为 `65`。
- 覆盖：3 条航带（初始 1 条 + 重构后 2 条），沿用 60m 航段、12m 横向步距。

## 3. 生成与检查

```bash
bash verification/full_fleet_joint/flow.sh generate \
  --fleet verification/full_fleet_joint/fleet_11_custom.yaml \
  --available 2,6,8,9,10,11,12,13,14,15,16
python3 -m unittest verification.full_fleet_joint.test_full_fleet -v
bash verification/full_fleet_joint/flow.sh onboard-commands   # 11 条
bash verification/full_fleet_joint/flow.sh dry-run
```

- 初始编队：A06/A08/A09、A10/A11/A12；领机 A06。
- 损毁 A10/A11；补位 A13/A14；B 组 A15/A16/A02。
- `onboard_commands.sh` 11 条；B 组（15/16/2）带 `layer_move_to:=8.0`，其余不带。

## 4. GCS 与标定

- GCS 拓扑仍包含 UAV1–16；UAV1/3/4/5/7 作为静默 peer。
- 标定计算集：UAV6/UAV8/UAV9，中心 UAV8；`calibration_ready` 需 3/3。任务坐标原点即 UAV8 机位，生成器输出 `shift_x=+5`、`shift_y=-5`。
- 起飞前逐机核对 `gp_origin_confirmed`，重点 A13/A14（补位机）、A15/A16/A02（B 组）。
- 备用机 A13/A14 依靠 GCS 排程错峰（`schedule_geometric_distance_m=8.0`、`confirm_deadline_s=15.0`；driver/SITL accept 20s）。

GCS：

```bash
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
           roslaunch verification/full_fleet_joint/gcs_real.launch'
```

任务：

```bash
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
           bash verification/full_fleet_joint/flow.sh run --until prepare'
# 核对 2×3 起始线与 B 组后
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
           bash verification/full_fleet_joint/flow.sh run'
```

## 5. 验收

11 个逻辑阶段成功；初始 6 架 2×3；A10/A11 向左退出、A13/A14 补位后恢复 2×3；3 条航带覆盖；`RECON_COMPLETE` 后才出现 B 动作；A15 在 `(4,65,5)` 悬停、A02 补打 `(10,65,5)`；A16 在目标2 `(0,65,5)`；A 返回 12m。`mission_finished=SUCCESS` 只代表驱动与观测条件通过。

## 6. SITL 结果

2026-10-21 11 机 UAV16 版：

| 项目 | 结果 |
|---|---|
| 运行目录 | `.tmp/full_fleet_joint/20261021_175755_sitl11_uav16` |
| 状态 | SUCCESS，11/11 逻辑阶段 |
| 任务时长 | 约 747s（仿真实时） |
| 最小三维间距 | 3.80m（coverage2，A06–A13）；recovery 最小 4.82m（A06–A13） |
| B 组 | A15/A16 打击；A15 失败悬停；A02 补打；3 架最终 5m 保持 |
| 视频 | `.tmp/full_fleet_joint/20261021_175755_sitl11_uav16/full_mission_3d.mp4`（99s，8×，12fps） |

- 已验证出生点按 `ground_positions`（含 `shift_x/shift_y`）生成；此前备用机观测误差 4.9m 即缺少该平移。
- 回归测试 39 项全部通过（含 11 机 spawn 与 ground_positions 一致性检查）。
