# 无桨链路验证 + 跟随环 dry-run（4 机：A08 领 + A10/A12/A13，全程不 arm）

> 目的：在**不装桨、不 arm、不起飞**的前提下验证三件事——
> ① 机间 odom/pose 经电台的**双向**传递；② 2026-10-08 新增的健康/事件观测能在实机上落盘、可判读；
> ③ GCS→bridge→执行器的 PREPARE/START/FOLLOW_ROUTE 全链路能在真机体上跑起来。
>
> 背景：两轮 coverage 失败（跟随者 3 s 内收不到领机 odom）的定层依赖本轮的观测文件，见
> `quality_reports/2026-10-08_two_rounds_coverage_failure_root_cause.md`。
>
> **本规程不能替代实飞验收**：它保证"部署正确 + 观测在工作 + 静态链路没断"，
> 不覆盖 armed/OFFBOARD 门禁、软起飞、真实控制与飞行态负载。通过 ≠ 任务态干净。

## 零、与实飞规程的关系

- 复用 `README.md` §零 门禁 **0–8**（空域/摆位/身份板钟/预检/起栈/邻机订阅/地面站/标定）——
  标定本来就要求全机 **disarm**，与无桨天然兼容（`gcs_a_calibration_node.py:113-117`）。
- **不执行门禁 9/10**（装桨、arm+OFFBOARD、下发）。`prepare()` 的 armed/OFFBOARD 门由
  `noprop:=true` 绕过：它把 `ego_swarm/mavros_state_topic` 置空，驱动侧仅在该参数非空时做
  MAVROS 检查（`src/swarm_uav_executor/src/swarm_uav_executor/drivers/ego_swarm.py:710-721`）。
- **副作用**：置空同时跳过 `connected` 检查 ⇒ 起栈后**必须人工确认** `/mavros/state`（§1）。
- 开关定义在 `src/swarm_uav_executor/launch/uav_offboard_ego_real.launch`（`noprop` arg，
  默认 false；实飞不传即原行为）。命令行**无法**直接传空串（roslaunch 丢弃空值参数），故用开关。

## 前置：同步到板（每台机载板）

```bash
for r in swarm_topology_bridge swarm_uav_executor; do
  sshpass -p nvidia rsync -a --delete --exclude='.git/' \
    src/$r/ nvidia@192.168.5.NN:catkin_swarm6-2/src/$r/; done
ssh nvidia@192.168.5.NN 'cd ~/catkin_swarm6-2 && source /opt/ros/noetic/setup.bash && catkin build swarm_uav_executor'
```

launch 文件改动随 rsync 生效（rospack 解析到源码目录，无需 build）；Python 侧改动仍依赖上面的 `catkin build`。

## 一、摆位与上电（无桨）

- [ ] 四机**桨叶全部拆下**；2×2、5 m 间隔、机头同向（同 `README.md` §1.2，保持与实飞相近的电台近场条件）
- [ ] 开阔天空，GPS `fix_type ≥ 3`；摆好不再挪动
- [ ] 板钟（`tools/onboard_clock_fix.sh`，必须早于起栈）→ router 起（现场常驻件，前台运行）→
      人工确认 mavros 连通（**必做**，驱动内检查已被 noprop 关掉）：
  ```bash
  ssh nvidia@192.168.5.81 'source ~/catkin_swarm6-2/devel/setup.bash && rostopic echo -n1 /mavros/state'
  # 期望 connected: True（与是否 arm 无关；False 先查 router/排线，别怀疑硬件舵机）
  ```

## 二、起栈（逐台；在实飞命令上加 `noprop:=true`）

把 `README.md` §二 的四条 `start_onboard.sh` 命令**逐台**加一个参数 `noprop:=true`，其余不变
（邻居清单每台不同，照抄 §二；不要用 `onboard_ctl.sh start 8 10 12 13 -- ...` 扇出）。示例：

```bash
# ── UAV8 / A08 领机（邻居 = 10 12 13）──
ssh nvidia@192.168.5.81 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  noprop:=true \
  neighbor_intents:=/UAV10/trajectory_intent,/UAV12/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV10/mavros/local_position/odom,/UAV12/mavros/local_position/odom,/UAV13/mavros/local_position/odom \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'
```

起栈后核对：

```bash
# 1) 健康文件必须在 ~1 s 内出现（看不到就别继续）
ls -l ~/catkin_swarm6-2/.ros_home/log/health/          # 至少 <UAVn>_bridge_health.jsonl
head -1 ~/catkin_swarm6-2/.ros_home/log/health/UAV8_bridge_health.jsonl   # health_start 行
# 2) 门禁 4/5 照旧（四台 UP、邻机订阅 6 行），见 README.md §二
# 3) 事件文件（*_follower_events.jsonl / *_relay_events.jsonl）在对应事件发生时才出现，起栈时没有是正常的
```

## 三、Stage 1：链路层（≥10 min，纯读观测）

- [ ] 跟随机 `rostopic hz /UAV8/mavros/local_position/odom` ≈ **19.5–20 Hz**（拓扑限频值；领机本机源为 30 Hz）
- [ ] 读 health JSONL（**接收侧**，在跟随机上）：

```bash
python3 - <<'PY'
import json, glob, os
for f in glob.glob(os.path.expanduser("~/catkin_swarm6-2/.ros_home/log/health/UAV1*_bridge_health.jsonl")):
    for line in open(f):
        d = json.loads(line)
        if d.get("event") != "sample":  continue
        for r in d["remote"]:
            if r["topic"] == "mavros/local_position/odom":
                print(f, d["t"], "rx=%d srcs=%d age_max=%.1fms src=%s stale=%s gap_max=%.0fms gt1000=%d gt3000=%d stalls=%d"
                      % (r["rx"], r["srcs"], r["age_max_ms"], r["age_max_src"], r["stale_srcs"],
                         r["gap_max_ms"], r["gt"]["gt1000"], r["gt"]["gt3000"], r["stalls"]))
PY
```

- [ ] **发送侧**（在领机上，同上但读 `d["local"]`）：`mavros/local_position/odom` 的 `rx/tx` 持续增长、`tx` 频率≈限频值、`drop` 与限频丢弃一致
- [ ] **双向**都成立：每台机对每台邻机（`matrix` 行按 `src` 逐源列出）

判据：`rx` 持续增长、`age_max_ms ≲ 50`、`gt1000/gt3000 = 0`、`stale_srcs` 空、`stalls = 0`、
无 `stall_start` / `link` 事件。出现任一 `stall_start`/`link` 行 = **保留文件、记下时刻**，这正是要抓的证据。

> 注意：`mavros/state` 是 1 Hz 话题，它的 `gt200/gt500/gt1000` 天然非零——**不要**拿它做判据。
> 这条测试替代 2026-09-22 报告里的内联 heredoc 延迟测试（`quality_reports/2026-09-22_deploy_param_diag_three_uav_summary.md:173-200`），
> 证据从"看一眼"升级为持久 JSONL。

## 四、Stage 2：全栈跟随环 dry-run

1. [ ] 地面站起栈（同 `README.md` §三）：`roslaunch src/tcp_to_ros/launch/gcs_a_real_four_uav.launch`
2. [ ] 标定（同 §4.1，全机 disarm 即可）→ `/group_a/calibration_ready` = True
3. [ ] 下发（实飞 §4.3 原命令，**不需要**先 arm）：

```bash
python3 verification/four_uav_recon/driver.py \
  --package verification/four_uav_recon/package \
  --tcp-host 192.168.5.241 --tcp-port 39001 \
  --mission-id noprop-dryrun-$(date +%Y%m%d_%H%M%S) \
  --log /tmp/noprop_dryrun.jsonl
```

4. [ ] 预期现象（**看清"正常失败"，别误判为缺陷**）：

| 观察 | 期望 | 出处 |
|---|---|---|
| PREPARE | 四机 accepted（**机载侧**唯一门槛＝本机 pose 新鲜 + MOVE_TO 的原点已确认；GCS 侧仍有标定/编队/链路门） | `ego_swarm.py:710-721` |
| 跟随者事件 | `follower_start` → `leader_odom_first_frame`（`wait_s` ≲ 0.1 s；SITL 实测 0.033 s）→ 观察窗内**无** `LEADER_LOST` | `<UAVn>_follower_events.jsonl` |
| 领机 | **不会动**：未 arm ⇒ PX4 忽略 setpoint；规划器完成判据是**真实位置**到达末航点，地面不动 ⇒ 不会"虚完成"，任务最终超时（≤ `uav_task_timeout_s`=280 s）失败 → 组级 FAILED + HOLD 扇出。**领机/整组的终态不作为判据** | `ego_planner_driver_node.cpp:791-808` |
| 组级收口（顺带） | 各机 `/UAVn/uav_hold` 是否到达，看 GCS 侧 backend 日志/driver JSONL。**不要**用机载 `relay_events.hold_enter` 当证据：relay 的 HOLD 是"选中源过期"的本地回退（`setpoint_relay.py:193-213`），而 `ego_swarm.hold()` 对非 safety reason 不发布任何 setpoint（`ego_swarm.py:655-688`，10-08 报告 §6.2） | 同上 |

  观察窗 **≥60 s** 即可收尾（首帧证据在第 1 秒内）；想顺带看组级收口就等它跑到超时。

5. [ ] 收尾与采集（同实飞 §六）：

```bash
for n in 8 10 12 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh stop $n; done   # 逐台（多目标只处理第一个）
./src/tcp_to_ros/tools/log_fetch.sh 8 10 12 13        # 采集包含 onboard/health/*.jsonl
```

  GCS 侧 `<GCS 工作区>/.ros_home/log/health/*.jsonl` 手工归进采集包 `gcs/`。

## 五、判据表（定层口径）

各机 + GCS 的健康文件对齐时间轴，看 `src=UAV8, topic=mavros/local_position/odom`：

| 现象 | 断在哪一段 |
|---|---|
| 发送侧 `local` 行 `rx` 停 | mavros → bridge（上游节点/本地订阅） |
| 发送侧 `rx` 走、`tx` 停 | bridge 发送路径（`drop` 可区分限频丢弃） |
| 接收侧 `rx` 停、对端 `tx` 走 | ZMQ/电台链路（配合 `link` 事件看是否真断开重连） |
| 接收侧 `rx` 走、但执行器无 `leader_odom_first_frame`/`follower_exit` | 执行器内部 |
| `recv_loop_lag_s` 持续增大（>0.5 s） | 收包线程被饿死/卡住，与"链路没数据"是两回事。该标量由收包线程自己记录；`stuck_sockets` 字段已停用（2026-10-09 跨线程查 ZMQ_EVENTS 致桥堆损坏死掉，见 `quality_reports/2026-10-09_*`），恒为空 |

## 六、已知陷阱

1. `noprop` 跳过 `connected` 检查 ⇒ §1 的人工确认不能省。
2. 3 s 首帧宽限**只在"从未见过领机 odom"时生效**；同一进程内第二次 FOLLOW_ROUTE 直接从
   严格新鲜度分支（`_FOLLOW_POSE_TIMEOUT_S`，2026-10-10 由 1.0 s 放宽为 3.0 s）起跳
   （ego_swarm.py 跨任务不清零）⇒ 反复 dry-run 要么重启该机执行器节点，要么按严格分支口径判读。
3. **重试必须换 `--mission-id`**：任务记录按 `(mission_id, command_id, uav_id)` 缓存 1 h，
   同号重发会秒回上次结果（`README.md` §4.3）。
4. 领机 odom 话题由 `leader_id` 推导（`A02 → /UAV2/...`，`ego_swarm.py:301-306`），
   核对 GCS 注入的 leader_id 与实际机号一致。
5. 长跑前清空 health 目录，便于时间轴对齐（采集只留尾部）。
6. 地面无轨迹时 `trajectory_intent` 可能无数据流（订阅登记仍在，门禁 5 照旧可查），属预期。
7. 标定 `calibration_ready` 一直 False：先查 2026-09-23 报告 §3 末的"限频 vs 窗口跨度"修复
   是否已随 GCS 栈部署（`quality_reports/2026-09-23_router_autostart_fix_and_uav11_link_diag.md`）。

## 七、覆盖边界

- **覆盖**：bridge/电台双向 odom+pose 传递、健康/事件观测落盘与判读、GCS→bridge 服务代理
  （PREPARE/START/HOLD）、跟随环启动/首帧/空档判据。
- **不覆盖**：armed/OFFBOARD 门禁、5 m 软起飞、轨迹执行与真实控制、飞行态负载
  （领机在飞时的 trajectory_intent 等）。门禁 9/10 仍须按 `README.md` 在实飞前执行。
