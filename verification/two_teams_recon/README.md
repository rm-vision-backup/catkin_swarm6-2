# 两个编队侦查流程（two_teams_recon）

同一套 15 机仿真环境，**一次生成、一起标定、分两批运行**。环境固定使用
`src/swarm_simulation/launch/multi_uav_ego_15sim.launch`（不再生成独立 spawn launch）。

| 轮次 | 编队 | 领机 | 初始 6 机 | 损毁 | 替补 | 侦查区域（任务坐标） |
|---|---|---|---|---|---|---|
| 批次1 | UAV1–7 | UAV2 | UAV1–UAV6 | UAV5 | UAV7 → UAV5 槽位 | 原点向左 30 m、向前 50 m |
| 批次2 | UAV8–15 | UAV11 | UAV10–UAV15 | UAV10、UAV15 | UAV8→UAV10、UAV9→UAV15 | 原点向右 30 m、向前 50 m |

- 编队2的 UAV10 是损毁机，因此领机取前排中间的 **UAV11**；编队1领机为 **UAV2**。
- 批次1任务结束后 **AUTO.LAND** 自动降落（避免与批次2右侧区域在 12 m 重叠）；
  批次2结束后保持悬停，**环境不关闭**，供 QGC 截图。
- 损毁方式：原地下降到 8 m 悬停；替补机先在起飞点 5 m 待命，损毁后直接飞到空槽位（12 m）。

## 1. 环境与标定

- 15 机由 `multi_uav_ego_15sim.launch` 按 5 排 × 3 列出生（列向右 5 m、排向后 5 m，
  带 ±0.8 m 随机偏差），一次全部生成。
- 15 机机载栈全部启动后，用**同一标定节点**一起标定：
  `calibration_uavs = [UAV1, UAV2, UAV3]`（第一排），`all_uavs = UAV1..UAV15`。
- 标定原点 = **UAV2 机位**；参考航向 = 本轮出生朝向（当前 `REF_YAW_DEG=-65°`）。
  所有任务坐标都以该原点/航向解释。
- 任务坐标（右 x / 前 y / 上 z），以 UAV2 为 `(0,0)`：

```text
前排 y=  0 : A01(-5) A02(0) A03(+5)        # A02 = 标定原点/领机
      y= -5 : A04(-5) A05(0) A06(+5)
      y=-10 : A07(-5) A08(0) A09(+5)        # 备用
      y=-15 : A10(-5) A11(0) A12(+5)        # A11 = 编队2领机
      y=-20 : A13(-5) A14(0) A15(+5)
```

## 2. 区域与航带

- 区域从标定原点起算（30 m × 50 m）：
  - 编队1：`x ∈ [-30, 0]`，`y ∈ [0, 50]`（左 30 m、前 50 m）。
  - 编队2：`x ∈ [0, 30]`，`y ∈ [0, 50]`（右 30 m、前 50 m）。
- 每队 3 条平行航带、步距 10 m（编队扫描带宽 10 m），boustrophedon：

```text
编队1：(-25,0)→(-25,50) → (-15,50)→(-15,0) → (-5,0)→(-5,50)
编队2：(  5,0)→(  5,50) → ( 15,50)→(15,0) → (25,0)→(25,50)
```

- 高度：覆盖 12 m；损毁悬停 8 m；备用待命 5 m；MOVE_TO 转场走 15 m 层。
- prepare 前移 1 m，并分两批下发（先后排、再前排），避免 6 机同时前移的规划冲突。

## 3. 阶段与关键期望位置

```text
prepare（后排 MOVE_TO → 前排 MOVE_TO；备用 observe）→ coverage1（航带1）
→ fault（损毁机原地降至 8m）→ recovery（备用 MOVE_TO 空槽位 + observe）
→ coverage2（航带2+3，RECON_COMPLETE）→ return → hold
→ 批次1：AUTO.LAND；批次2：保持悬停，环境保留
```

| 节点 | 编队1 | 编队2 |
|---|---|---|
| coverage1 终点 | 领机 A02 `(-25,50)`；A01/A03 `(-30/ -20,50)`；A04/A05/A06 `(-30/ -25/ -20,45)` | 领机 A11 `(5,50)`；A10/A12 `(0/10,50)`；A13/A14/A15 `(0/5/10,45)` |
| 损毁点（8 m） | A05 `(-25,45)` | A10 `(0,50)`；A15 `(10,45)` |
| 补位点（12 m） | A07 `(-25,45)` | A08 `(0,50)`；A09 `(10,45)` |
| coverage2 终点 | 领机 A02 `(-5,50)` | 领机 A11 `(25,50)` |
| 返航终点 | 领机 A02 `(0,3)` | 领机 A11 `(0,-12)` |

观测器容差：水平 2 m、垂直 1 m。`prepare-spares` 只做 observe、不做位置判据
（备用机悬停点受出生随机偏差影响，由标定原点吸收）。

## 4. 生成与离线校验

```bash
bash verification/two_teams_recon/flow.sh generate      # 一次生成两队 + 共享 GCS 配置
python3 -m unittest verification.two_teams_recon.test_two_teams -v
bash verification/two_teams_recon/flow.sh dry-run --team 1|2
bash verification/two_teams_recon/flow.sh onboard-commands --team 1|2
```

输出：
- `generated/team1|2/`：`mission_spec.json`、`manifest.json`、`package/`、`config/formation_offsets.yaml`、`onboard_commands.sh`、`flow_env.sh`
- `generated/combined/`：共享 `calibration.yaml`、`formation_team1.yaml`、`formation_team2.yaml`、`manifest.json`

修改编组/区域/高度只改 `teams.yaml` 后重新生成。

## 5. 一键运行（15 机同环境、两批次）

先停止其它占用 **11300 / 11310 / 11345 / 39061 / 39062** 的仿真，再执行：

```bash
bash verification/two_teams_recon/sitl/run_recording.sh \
  --outdir .tmp/two_teams_recon/$(date +%Y%m%d_%H%M%S)_two_batches
```

- 启动 `multi_uav_ego_15sim.launch`（GUI 默认开启；`--headless` 关闭）。
- 启动 15 机机载栈（11311–11325）与**双 backend** GCS（39061 / 39062），共享一次标定。
- 等 15 机 `gp_origin_confirmed` 后：
  1. 批次1（UAV1–7）起飞 → 任务 → **AUTO.LAND** → 等待 disarm；
  2. 批次2（UAV8–15）起飞 → 任务 → 保持悬停。
- 默认运行结束后**保留环境**（Gazebo/PX4/机载栈/GCS）供 QGC 截图；脚本会打印
  `stop_environment.sh` 路径，用它统一停止。
- `--no-keep-alive`：结束后自动关闭环境；`--leave-running`：编排进程退出但保留环境
  （自动化验证用，停止仍用 `stop_environment.sh`）。

证据：`summary.json`、`driver_team1.jsonl`、`driver_team2.jsonl`、`telemetry.jsonl`、
各进程日志与 `stop_environment.sh`。

## 6. 双 backend 与关键实现

- `sitl/gcs_two_teams.launch`：一个 bridge + 一个标定节点 + 两个 backend
  （`/team1/*`、`/team2/*`）与两个 TCP server（39061/39062）。两个 backend 各自加载
  本队 formation 配置，因此每队可以有独立领机；批次串行，不会同时存在 active group。
- 两个 backend 都设 `fault_exit_schedule_enabled=false`：原地下降到 8 m 的水平位移为 0，
  时间偏移排程会以 `GEOMETRY_UNSUITABLE` 拒绝。
- `FAULT_EXIT` 的 TCP payload 必须是非空 `waypoints`（z=8），不能写 `target_pose`。
- coverage1 参与者匹配 `recon_initial`，coverage2/return 匹配 `recon_recovered`；
  领机分别为 A02 / A11。
- `sitl/land_15_sitl.py`：逐机在独立 ROS Master 上切 `AUTO.LAND` 并等待 disarm。

## 7. 已知限制

- 仅 SITL 验证；实飞前仍需现场净空、通信热测与标定核对。
- 出生 ±0.8 m 随机偏差由标定原点吸收；`prepare-spares` 不做位置判据。
- 批次1降落在返航点附近（原点右边界附近），批次2在其上方 12 m 飞行，垂直隔离。
- QGC 连接各机 PX4 的 GCS MAVLink 端口（实例 0–14 → UDP 18570–18584）。
