# 单机预验证（A06）：起飞 + 准备编队 + 侦查航线

> 本文是 A06 专用预验证，保留其任务语义；日常部署/通信/标定先按 [统一工作流](../real_uav_workflow/README.md)。历史四机席位后来已改为 A08/A10/A12/A13，不因本文存在就把 UAV6 加回当前机队。

历史上为四机集群（当时 A06/A08/A10/A13）上真机前定义这一轮：只让 **A06（执行层 UAV6）**走完
**起飞 → 准备编队（MOVE_TO）→ 侦查航线（FOLLOW_ROUTE）**，确认链路 / 标定 / 任务下发 /
安全兜底都对，再上四机。

- 流程与四机那条线**完全同源**：同一个下发脚本、同一份任务计划（`prepare-a` +
  `coverage-segment-1`，航点与四机逐字相同）、同一个 GCS 后端，只是机数缩到 1 ——
  **A06 自己当领机**，没有跟机。
- 授权依据：`src/swarm_uav_interfaces/real_setup.md` §7.2（§7.1 允许"单独定义、单独标注的小机数工程测试"）。

| 文件 | 作用 |
|---|---|
| `package/` | 单机任务数据（robots / bootstrap / world / plans），只含 A06，两个计划都齐 |
| `sitl/` | 单机仿真彩排（`single_uav_spawn.launch` / `run_sim_single_uav.sh`） |
| 下发脚本 | **复用** `verification/four_uav_recon/driver.py`（按任务包驱动，与机数无关） |
| 地面站 | `src/tcp_to_ros/launch/gcs_a_real_single_uav.launch`（薄 wrapper，只改两个配置指向单机版） |

---

## 一、现场摆位与空域

1. 平地放一架即可；**机头朝向随意但要清楚** —— 它自己的机头方向既定义任务坐标系的"前方"，
   也是冻结的参考航向（单机只有它自己，航向差天然为 0）。
2. **空域要求**（两个阶段都算）：
   - **准备编队**：目标 `(-2.5, -2.5, 12)` = 沿机头斜后 3.54 m、升到 12 m；
   - **侦查航线**：以该点为起点，飞一个约 **20 m × 24 m** 的矩形往返
     （6 个航点：(-2.5,-2.5) → (17.5,-2.5) → (17.5,9.5) → (-2.5,9.5) → (-2.5,21.5) → (17.5,21.5)，路径约 84 m）。
   - 也就是：**上方 15 m、起飞点往外 20 m × 24 m 的矩形范围**要清空（无建筑、无人员、无其它航空器）。
3. 摆好后不要再挪动 → 标定 → 再 arm。

## 二、机载启动（192.168.5.61）

```bash
ssh <user>@192.168.5.61 'bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  uav_task_timeout_s:=280'
```

- **不传** `neighbor_intents` / `neighbor_odom_topics`（单机没有邻机）。
- **建议传** `uav_task_timeout_s:=280`：侦查航线约 84 m（预计 100–140 s），canonical 默认 200 s
  虽大概率够，但保留 280 作裕度、与四机版取同一值，免得改场地/改航点后踩线。
- ssh 用户取 `src/tcp_to_ros/tools/` 共用参数文件里的 `BOARD_SSH_USER`。

## 三、地面站启动（GCS_A = 192.168.5.241，hostname 须为 GCS_A）

```bash
roslaunch src/tcp_to_ros/launch/gcs_a_real_single_uav.launch
```

（等价于 `gcs_a_real_four_uav.launch` + `formation_offsets_file:=config/formation_offsets_single_uav.yaml`
`calibration_config_file:=config/calibration_single_uav.yaml`。）

## 四、跑一轮

1. **标定必须先于 arm（硬顺序）**：UAV6 保持 **disarm**，等

```bash
rostopic echo -n1 /group_a/calibration_ready     # 期望 data: True
```

   （`gcs_a_calibration_node` 要求 not armed 才冻结原点；先 arm 就永远 ready 不了 → 任务会被 `CALIBRATION_NOT_READY` 拒。）

2. 遥控 arm + OFFBOARD，确认 `/mavros/state` = `armed: True` / `mode: OFFBOARD`。
   ⚠ **一进 OFFBOARD，机载就会自动垂直爬到 5 m**（`takeoff_height_m=5.0`、`takeoff_ascend_speed=1.0`）——
   现场必须预期；桨叶周围清空、人员撤到安全距离。遥控的模式开关随时可切出 OFFBOARD 接管
   （摇杆顶杆默认不覆盖 OFFBOARD，见 `real_setup.md` §7 的安全边界）。

3. 下发任务（两个阶段）：

```bash
python3 verification/four_uav_recon/driver.py \
  --package verification/single_uav_recon/package \
  --mission-id single-uav-prepare-001 --phases prepare coverage \
  --tcp-host 192.168.5.241 --tcp-port 39001 \
  --log verification/single_uav_recon/logs/real_$(date +%Y%m%d_%H%M%S).jsonl
```

   预期飞行过程：自动爬到 5 m → **准备编队**（升到 15 m 层高 → 移到目标点上方 → 降到 12 m）→
   **侦查航线**（按 6 个航点绕约 84 m，约 1.5–2 分钟）→ 悬停。

## 五、成功判据与观测

- **判据**：driver 退出码 0；出现 `[prepare] SUCCESS` 与 `[coverage] SUCCESS`；GCS 侧
  两个阶段都 1/1 `COMPLETED`；全程 `MIN_DISTANCE_BREACH` 零次；全程无 `EGO_PLAN_FAILED`。
- 观测：driver JSONL（`mission_start`/`phase_start`/`phase_result`/`command_feedback`）；
  GCS `gcs_a_backend_node` rosout（`GCS_A goal ...`、`GCS_A schedule ...`）；
  `/UAV6/uav_task_state`、`/group_a/group_task_state`；
  机载 `~/.tmp/logs/UAV6_offboard_ego.log`；整轮取证 `src/tcp_to_ros/tools/log_fetch.sh 6`。
- **收尾说明**：任务结束后按现场规程**遥控接管降落/返航**（`real_setup.md` §6.6）。
  仿真彩排里没有遥控接管，收尾是脚本 `cleanup` 停掉机载 offboard → PX4 的"offboard 丢失"
  failsafe 默认动作是 **Land**（`COM_OBL_ACT=0`），所以仿真末尾看到的是下降而不是悬停 ——
  这是正常的收尾现象，不是任务失败。

## 六、仿真彩排（先跑这个）

```bash
bash verification/single_uav_recon/sitl/run_sim_single_uav.sh
```

自动完成：1 机 SITL（出生点与四机版的 UAV6 相同）→ 机载（只起 6，`EGO_TASK_TIMEOUT_S=280`）→
单机 GCS + 标定 → takeoff → 静置 30 s → `driver.py --phases prepare coverage`。
日志在 `.ros_home/log/headless_test/`；失败轮归档到 `.ros_home_round_single_uav`（含机载日志）。
退出码 0 = 两个阶段都 SUCCESS。

## 与四机流程的差异提醒

| 项 | 单机 | 四机 |
|---|---|---|
| 机数/角色 | 1 架（A06 自己当领机，无跟机） | 4 架（A06 领机 + A08/A10/A13 跟随） |
| 机载启动参数 | 只传 `uav_task_timeout_s:=280` | 还要逐台传其余 3 架邻居清单 |
| 地面站 | `gcs_a_real_single_uav.launch` | `gcs_a_real_four_uav.launch` |
| 摆位 | 一架 + 机头前/左右各留空（见 §一） | 5 m×5 m 方阵、四机机头同向 |
| 判据 | 1/1、两阶段 SUCCESS | 4/4、跟机与领机保持 ≈5 m |
