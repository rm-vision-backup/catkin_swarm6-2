# 14 机事故后任务（2026-10-21）

UAV3、UAV7 不可用；UAV13 恢复。本配置替代 15UAV_README.md，任务阶段/高度与第五版一致；覆盖航带缩减为 3 条（初始 1 条 + 重构后 2 条）。

## 1. 地面摆位

任务坐标右 x / 前 y / 上 z；原点 = 第 2 排中心（UAV8 机位），5m 网格：

```text
第 1 排 y=+5 : UAV1  UAV2  UAV4  UAV5
第 2 排 y= 0 : UAV6  UAV16 UAV8  UAV9  UAV10
第 3 排 y=-5 : UAV11 UAV12 UAV13 UAV14 UAV15
              x=-10 -5    0     +5    +10
```

UAV3、UAV7 不参飞、不上电；UAV13 回到第 3 排第 3 列。

## 2. 角色

| 角色 | 飞机 | 说明 |
|---|---|---|
| 初始 3×3（9） | A01/A02/A04/A06/A16/A08/A11/A12/A13 | UAV4 在右边缘，UAV13 保留在编队内 |
| 领机 | A06 | 左边缘中排 |
| 模拟损毁（2） | A04/A08 | 右边缘上/中；降至 8m 后先沿 +y 离开编队占用区，再向左退出到 x=-30，退出航迹避开编队 x/y |
| 备用补位（2） | A05/A09 | 起飞后原地 5m；损毁后补到 A04/A08 的空位 |
| B 组（3） | A10/A14/A15 | A10、A14 首次打击；A14 失败；A15 补打 |
| 不参飞 | UAV3、UAV7 | 不上电、不接入机载栈 |

- 补位后 3×3：A01/A02/A05、A06/A16/A09、A11/A12/A13。
- B 目标 2 个：A10 → `(10,65)`，A14 → `(0,65)`（目标2 = 目标1 左边 10m）；两条打击航迹不交叉。A14 失败后在 `(-6,65,5)` 悬停，A15 补打 `(0,65,5)`。

## 3. 生成与检查

```bash
bash verification/full_fleet_joint/flow.sh generate \
  --fleet verification/full_fleet_joint/fleet_14_custom.yaml \
  --available 1,2,4,5,6,8,9,10,11,12,13,14,15,16
python3 -m unittest verification.full_fleet_joint.test_full_fleet -v
bash verification/full_fleet_joint/flow.sh onboard-commands   # 应为 14 条，含 start 13
bash verification/full_fleet_joint/flow.sh dry-run
```

`onboard_commands.sh` 应为 14 条；B 组（10/14/15）带 `layer_move_to:=8.0`；全部 `bundle_transport:=udp`。

## 4. GCS 与标定

- GCS 拓扑需要 UAV13 的 `/UAV13/...` 三组服务；UAV3/UAV7 保留为静默 peer（按现场要求不删除）。
- 标定计算集（`calibration.yaml`）：UAV6、UAV16、UAV8、UAV9、UAV10；原点 = UAV8。
- 起飞前逐机核对 `gp_origin_confirmed`，重点 UAV13（恢复机）、UAV16、UAV5、UAV9。
- 任何机载栈重启、换电或移动飞机后，重启 GCS 标定节点并重新标定。

GCS 启动：

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
# 核对起始 3×3、UAV13/UAV16 位置后
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
           bash verification/full_fleet_joint/flow.sh run'
```

## 5. 验收

11 个逻辑阶段成功；初始 9 架含 A13；A04/A08 损毁，A05/A09 补位后 3×3；三条航带覆盖；`RECON_COMPLETE` 后才出现 B 动作；A14 在 `(-6,65,5)` 悬停、A15 补打 `(0,65,5)`；A 返回 12m。`mission_finished=SUCCESS` 只代表驱动与观测条件通过。

## 6. SITL 结果（2026-10-21 16:20，3 航带 / 近目标）

- 运行目录：`.tmp/full_fleet_joint/20261021_162053_sitl14_3strips`
- 状态：**SUCCESS**，11 个逻辑阶段全部 SUCCESS，任务时长约 **682.5 s**
- 覆盖缩减为 3 条航带（初始 1 条 + 重构后 2 条）
- B 目标：A10 `(10,65)`、A14 `(0,65)`；A14 失败悬停 `(-6,65)`，A15 补打 `(0,65)`
- 备用机 A05/A09 通过 GCS 排程参数 `schedule_geometric_distance_m=8.0` 错峰补位
- 最小实测三维间距：待最终 SITL 更新
- 视频：同目录 `full_mission_3d.mp4`（4× 播放，约 172.7 s）
- 证据：`driver.jsonl`、`summary.json`、`telemetry.jsonl`、`full_mission_3d.json`

本次未覆盖：实机电台链路、真实 B 组、30+ 架容量。
