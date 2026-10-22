# 15 架实飞任务流程（UAV16 顶替故障 UAV13）

2026-10-21 实飞配置。**可用机 15 架：UAV1–12、UAV14、UAV15、UAV16；UAV13 故障不参飞，UAV16 摆到原 UAV13 的地面机位。** 其余流程与 `README.md` 第五版一致。

## 1. 地面摆位

任务坐标：右 x / 前 y / 上 z；原点 = 第 2 排中心（UAV8 机位）。四排五列、5m 网格。

```text
第一排 y=+5 : UAV1  UAV2  UAV3  | UAV4   UAV5
第二排 y= 0 : UAV6  UAV7  UAV8  | UAV9   UAV10
第三排 y=-5 : UAV11 UAV12 UAV16 | UAV14  UAV15
第四排 y=-10: （空；UAV16 已移到原 UAV13 机位）
              x=-10 -5    0      +5     +10
```

- **UAV16 必须摆在原 UAV13 的机位**：第 3 排第 3 列，任务坐标 `(0, -5)`，即 UAV8 正后方 5m；机头朝向与同排其他机一致。
- UAV13 断电/移出飞行区，不接入机载栈，不参与拓扑和标定。
- 地面摆位直接由 `fleet.yaml` + `--replace UAV13=UAV16` 生成，不要手改任务包。

## 2. 编组

| 角色 | 飞机 | 说明 |
|---|---|---|
| 初始 3×3（9） | A01/A02/A03/A06/A07/A08/A11/A12/A16 | A16 在 A13 的原槽位（右边缘中列） |
| 领机 | A06 | 左边缘中排，前移 1m 起飞 |
| 边缘损毁（2） | A03/A08 | 顶排/中排右边缘；损毁后降至 8m 向左退出 20m |
| 备用（2） | A04/A09 | 起飞后原地 5m；损毁后同批飞到 coverage1 终点空位 |
| B 组（4） | A05/A10/A14/A15 | A05/A10/A14 首次打击；A10 失败；A15 补打 |
| 不参飞 | UAV13 | 故障，地面移除 |

- 15 架 = 9 初始 + 2 备用 + 4 B。B 组第 4 架从备用列底部补入，因此 A14 归 B 组，备用只保留 A04/A09。
- 初始 3×3 的右边缘是 A03/A08/A16；本次只损毁 A03/A08，A16 保持在编队内，补位后仍是 3×3。

## 3. 高度与覆盖

- A 覆盖 **12m**；A prepare MOVE_TO 转场 15m；B 转场 **8m**、打击 **5m**；备用机待命 **5m**；损坏机退出 **8m**。
- 四航带 boustrophedon（coverage2 三条 + coverage1 一条）：
  - 航带 1（coverage1）：领机 `(-10,1) → (-10,61)`。
  - 航带 2：横向 +12m 到 `(2,61)`，前飞 `(2,121)`。
  - 航带 3：横向到 `(14,121)`，后飞 `(14,61)`。
  - 航带 4：横向到 `(26,61)`，前飞 `(26,121)`。
- 航带横向步距 12m；3×3 编队扫描带宽 18m，重叠 6m；覆盖区域约 x=−14..+40、y=−4..126，约 7000m²。

## 4. 阶段流程（11 个逻辑阶段）

```text
prepare（prepare-a-1 / prepare-a-2 / prepare-spares 观察 / prepare-b-hold 观察，240s）
→ coverage1（初始 9 架航带 1，60m，240s）
→ fault（A03/A08 下降 8m、向左退出 20m，180s）
→ recovery（A04/A09 同批 MOVE_TO 到 coverage1 终点空位，180s；随后观察恢复后的 3×3）
→ coverage2（3×3 航带 2+3+4，360s）→ 本地 RECON_COMPLETE
→ strike（B 组 A05/A10/A14 到目标上方 5m，180s）
→ strike_fail（A10 模拟失败，在目标附近 5m 悬停，180s）
→ strike_reinforce（A15 从起点补打 A10 的目标，180s）
→ strike_hold（3 架在目标上方 5m 保持 3s，30s）→ 本地 STRIKE_COMPLETE
→ return_a（A 组 9 架返回 12m，300s）
→ hold（观察 3s，30s）→ 飞手逐机接管
```

关键期望位置：

| 节点 | 期望 |
|---|---|
| prepare 起始 3×3 | A16 `(0,-4,12)`；领机 A06 `(-10,1,12)` |
| coverage1 终点 | A16 `(0,56,12)`；A03 `(0,66,12)`；A08 `(0,61,12)` |
| fault 退出点 | A03 `(-20,66,8)`；A08 `(-20,61,8)` |
| recovery 补位点 | A04 `(0,66,12)`；A09 `(0,61,12)` |
| coverage2 终点 | 领机 `(26,121,12)`；A16 `(36,116,12)` |
| B 目标 | A05 `(-8,50,5)`；A14 `(24,80,5)`；A10 失败悬停 `(4,80,5)`；A15 补打 `(10,80,5)` |
| 任务收尾 | A 组回到起始线 12m；B 3 架目标上方 5m；A10 在 `(4,80,5)`；A03/A08 在退出点 8m |

## 5. 生成与启动

```bash
# 生成 15 架任务包（UAV16 顶替 UAV13 机位）
bash verification/full_fleet_joint/flow.sh generate \
  --available 1-12,14,15,16 --replace UAV13=UAV16

# 核对编组、几何与逐机启动命令
python3 -m unittest verification.full_fleet_joint.test_full_fleet -v
bash verification/full_fleet_joint/flow.sh onboard-commands
bash verification/full_fleet_joint/flow.sh dry-run
```

- `onboard_commands.sh` 应为 15 条；UAV16 的命令不带 `layer_move_to:=8.0`（A 组），UAV5/10/14/15 带 8m。
- 机载栈：A 组与备用机默认层高，B 组 `layer_move_to:=8.0`，全部 `bundle_transport:=udp`。
- GCS 拓扑需包含 `/UAV16/...` 三组服务；UAV13 不需要。
- 标定计算集优先第 2 排 UAV6–10（本次 5 架全可用），原点 = UAV8 机位；UAV16 不改变标定原点。
- 起飞前逐机核对 `gp_origin_confirmed`，重点核对 UAV16（它是替换机）。

GCS 任务启动（`--log` 可省略，driver 自动写 `src/uav_log/<时间>_full-fleet-joint/driver.jsonl`）：

```bash
env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
  bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
           bash verification/full_fleet_joint/flow.sh run'
```

先 `--until prepare` 核对起始线与 UAV16 位置，再跑全流程。

## 6. 与 16 架版本的差异

| 项目 | 16 架 | 本次 15 架 |
|---|---|---|
| 可用机 | UAV1–16 | UAV1–12、14、15、16 |
| 初始 3×3 右边缘 | A03/A08/A13 | A03/A08/**A16**（A13 槽位） |
| 备用 | A04/A09/A14 | A04/A09 |
| B 组 | A05/A10/A15/A16 | A05/A10/**A14**/A15 |
| 损毁数 | 3（A03/A08/A13） | 2（A03/A08） |
| 补位后 | 3×3 | 3×3（A16 留在编队） |

其余高度、四航带、B 目标、失败/补打逻辑不变。

## 7. 中止与验收

- 任一阶段失败/超时/切出 OFFBOARD：停止后续，TCP HOLD + 逐机 `/UAVn/uav_hold`，飞手接管。
- 验收：11 个阶段成功；初始 9 架含 A16，A03/A08 损毁，A04/A09 补位后 3×3；四条航带覆盖；`RECON_COMPLETE` 后才有 B 动作；A10 失败后在 `(4,80,5)` 悬停，A15 补打，最终 3 架 B 在目标上方 5m；A 返回 12m。
- `mission_finished=SUCCESS` 只代表本轮驱动与观测条件通过；实飞前仍须完成 15 机 UDP 热测、场界实测和人工净空确认。

## 8. 2026-10-21 SITL 验证结果

| 项目 | 结果 |
|---|---|
| 运行目录 | `.tmp/full_fleet_joint/2026-10-21_094042_sitl15_uav16` |
| 状态 | **SUCCESS**，11 个逻辑阶段全部通过 |
| 任务时长 | 881s |
| 初始 9 架 | A01/A02/A03/A06/A07/A08/A11/A12/A16 |
| 损毁/备用 | A03/A08 退出；A04/A09 补位后 3×3 |
| B 组 | A05/A10/A14 打击，A10 失败悬停 `(4.1,80,5)`，A15 补打 `(10.1,80,5)` |
| 实测最小 3D | 全局 2.98m（recovery A09/A16）；coverage2 3.79m |
| UAV16 轨迹 | 起飞 `(0,-5)` → prepare `(0,-4,12)` → coverage1 终点 `(0,56,12)` → coverage2 终点 `(36,116,12)` → 返航 `(0,-4,12)` |
| 视频 | `full_mission_3d.mp4`，112s，8×，1346 帧 |

四航带覆盖与 15/16 架版本一致（x=−10/2/14/26，y=61..121）；A16 全程保持在编队右边缘列。
