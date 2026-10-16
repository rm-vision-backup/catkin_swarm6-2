# 无桨链路验证 + 跟随环 dry-run（3 机：A08 领 + A10/A13，全程不 arm）

> **专项任务链路 dry-run**，不是每轮必做的 20 s UDP/ping 快测。先完成 [统一上机工作流](../real_uav_workflow/README.md) 的准备步骤；下文 `noprop:=true` 会跳过部分飞行门禁，仅用于拆桨专项测试，后续实飞必须恢复正式参数并重新检查/标定。

> 目的：在**不装桨、不 arm、不起飞**的前提下验证三件事——
> ① 机间 odom/pose 经电台的**双向**传递；② 2026-10-08 新增的健康/事件观测能在实机上落盘、可判读；
> ③ GCS→bridge→执行器的 PREPARE/START/FOLLOW_ROUTE 全链路能在真机体上跑起来。
>
> 背景：两轮 coverage 失败（跟随者 3 s 内收不到领机 odom）的定层依赖本轮的观测文件，见
> `quality_reports/2026-10-08_two_rounds_coverage_failure_root_cause.md`。
>
> **本规程不能替代实飞验收**：它保证"部署正确 + 观测在工作 + 静态链路没断"，
> 不覆盖 armed/OFFBOARD 门禁、软起飞、真实控制与飞行态负载。通过 ≠ 任务态干净。
>
> **空中版（2026-10-10 起）**：Stage 1/2 的实飞版本在 `README.md §九`
> （`air_flow.sh`：stage1/stage2/full + 机载 rosbag + odom 快查），判据口径与本文件一致。
>
> 三机版说明（本文件 vs `four_uav_recon/no_prop_link_test.md`）：邻居清单从 3 台缩到 2 台、
> Stage 2 换三机 GCS 配置与三机 package，其余判据/陷阱完全一致。

## 前置 0：代码同步到板（**没做这步，健康文件不会出现**）

板上必须是含 10-08 观测的版本（bridge / executor 两个包）。判据：起栈后
`~/catkin_swarm6-2/.ros_home/log/health/UAVn_bridge_health.jsonl` 应在 1 s 内出现。
没出现 = 旧代码，先执行（每台）：

```bash
for r in swarm_topology_bridge swarm_uav_executor; do
  sshpass -p nvidia rsync -a --delete --exclude='.git/' \
    src/$r/ nvidia@192.168.5.NN:catkin_swarm6-2/src/$r/; done
ssh nvidia@192.168.5.NN 'cd ~/catkin_swarm6-2 && source /opt/ros/noetic/setup.bash && catkin build swarm_uav_executor'
```

同步前确认板上没有跑着机载栈（`pgrep -af roslaunch`），有就先停。launch 文件改动随 rsync
生效（rospack 解析源码目录，无需 build）；Python 侧改动依赖上面的 `catkin build`。

## 一、摆位与上电（无桨）

- [ ] 三机**桨叶全部拆下**；按下述 L 形摆位、5 m 间隔、机头同向（差 ≤30°）
- [ ] 开阔天空，GPS `fix_type ≥ 3`；摆好不再挪动
- [ ] 板钟（`tools/onboard_clock_fix.sh 8 10 13`，必须早于起栈）→ router 起（现场常驻件，
      systemd 服务）→ 人工确认 mavros 连通（**必做**，noprop 跳过驱动内 connected 检查）：
  ```bash
  ssh nvidia@192.168.5.81 'source ~/catkin_swarm6-2/devel/setup.bash && rostopic echo -n1 /mavros/state'
  # 期望 connected: True（与是否 arm 无关；False 先查 router/排线）
  ```

**L 形摆位**（与 `README.md` §1.2 同栅格，去掉右前角 A12；以 A13 机位为参考）：

| 机 | 物理位（右/前） | 任务坐标（z=12 时的 prepare 目标） |
|---|---|---|
| A13 | (0, 0) 原点角 | (0, 0) |
| A08（领机） | (+5, 0) A13 右侧 5 m | (+5, 0) |
| A10 | (0, +5) A13 正前 5 m | (0, +5) |

标定原点 = 三机全球位置逐轴中位数 = A13 机位；三机中位数的性质与四机不同，按上表摆
即可让 prepare 水平位移≈0。

## 二、起栈（逐台；邻居清单每台不同；**在实飞命令上加 `noprop:=true`**）

```bash
# ── UAV8 / A08（领机）  邻居 = 10 13 ──
ssh nvidia@192.168.5.81 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  noprop:=true \
  neighbor_intents:=/UAV10/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV10/swarm_pose,/UAV13/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV10 / A10  邻居 = 8 13 ──
ssh nvidia@192.168.5.101 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  noprop:=true \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV13/trajectory_intent \
  neighbor_odom_topics:=/UAV8/swarm_pose,/UAV13/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'

# ── UAV13 / A13  邻居 = 8 10 ──
ssh nvidia@192.168.5.131 'setsid bash ~/catkin_swarm6-2/src/swarm_uav_executor/bashs/start_onboard.sh \
  noprop:=true \
  neighbor_intents:=/UAV8/trajectory_intent,/UAV10/trajectory_intent \
  neighbor_odom_topics:=/UAV8/swarm_pose,/UAV10/swarm_pose \
  uav_task_timeout_s:=280 </dev/null >/dev/null 2>&1 &'
```

起栈后核对：

```bash
# 1) 健康文件必须在 ~1 s 内出现（看不到就别继续）
ls -l ~/catkin_swarm6-2/.ros_home/log/health/          # 至少 <UAVn>_bridge_health.jsonl
head -1 ~/catkin_swarm6-2/.ros_home/log/health/UAV8_bridge_health.jsonl   # health_start 行
# 2) 门禁：三台 UP（逐台调用，onboard_ctl.sh 多目标只处理第一个）
for n in 8 10 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh status $n | sed -n '3p'; done
# 3) 邻机订阅：每台应看到 2 个邻机 ×（intent+odom）= 4 行（把 8 换成各机号）
ssh nvidia@192.168.5.81 'source /opt/ros/noetic/setup.bash; source ~/catkin_swarm6-2/devel/setup.bash; \
  rosnode info /uav_executor_UAV8 | sed -n "/^Subscriptions/,/^Publishing/p" \
  | grep -E "trajectory_intent|swarm_pose"'
# 4) 事件文件（*_follower_events.jsonl / *_relay_events.jsonl）在对应事件发生时才出现，起栈时没有是正常的
```

## 三、Stage 1：链路层（现场定稿 **30 s** 基线窗；纯读观测）

准备：先把三台 health 目录清空，便于时间轴对齐（历史文件不影响运行，只影响判读）：

```bash
for ip in 81 101 131; do sshpass -p nvidia ssh nvidia@192.168.5.$ip \
  'mkdir -p ~/catkin_swarm6-2/.ros_home/log/health && rm -f ~/catkin_swarm6-2/.ros_home/log/health/*.jsonl'; done
```

- [ ] 跟随机上 `rostopic hz /UAV8/swarm_pose` ≈ **10 Hz**（当前 bundle tick 与 `swarm_pose` 桥上限频；领机本机 odom 源 ~30 Hz）
- [ ] 读 health JSONL（在**每台机**上读**本机**文件，接收侧看 `remote`）：

```bash
python3 - <<'PY'
import json, os, socket
f = os.path.expanduser("~/catkin_swarm6-2/.ros_home/log/health/%s_bridge_health.jsonl" % socket.gethostname())
for line in open(f):
    d = json.loads(line)
    if d.get("event") != "sample":
        continue
    for r in d["remote"]:
        if r["topic"] == "swarm_pose":
            print(d["t"], "rx=%d srcs=%d age_max=%sms src=%s stale=%s gap_max=%sms gt1000=%d gt3000=%d stalls=%d"
                  % (r["rx"], r["srcs"], r["age_max_ms"], r["age_max_src"], r["stale_srcs"],
                     r["gap_max_ms"], r["gt"]["gt1000"], r["gt"]["gt3000"], r["stalls"]))
PY
```

- [ ] **发送侧**（在领机上，同上但读 `d["local"]`）：`swarm_pose` 的 `rx/tx`
      持续增长、`tx` 频率≈限频值、`drop` 与限频丢弃一致
- [ ] **双向**都成立：每台机对每台邻机（`matrix` 行按 `src` 逐源列出）

窗口口径（2026-10-09 现场定稿）：**30 s 足够**做基线确认——判定性证据在 Stage 2 干跑（跟随环首帧）
与任务态链路；Stage 1 只回答"静态链路有没有明显缺口"。

判据：`rx` 持续增长、`age_max_ms ≲ 50`、`gt1000/gt3000 = 0`、`stale_srcs` 空、`stalls = 0`、
无 `stall_start` / `link` 事件。出现任一 `stall_start`/`link` 行 = **保留文件、记下时刻**，
这正是要抓的证据。

> 注意：`mavros/state` 是 1 Hz 话题，它的 `gt200/gt500/gt1000` 天然非零——**不要**拿它做判据。
> 这条测试替代 2026-09-22 报告里的内联 heredoc 延迟测试，证据从"看一眼"升级为持久 JSONL。

## 四、Stage 2：全栈跟随环 dry-run

1. [ ] 地面站起栈（三机入口，**必须先于 driver**）：
   ```bash
   roslaunch src/tcp_to_ros/launch/gcs_a_real_three_uav.launch
   ```
   启动日志核对：无 `ValueError`；`formation_offsets_file` 指向 `formation_offsets_three_uav.yaml`；
   `calibration set=UAV8,UAV10,UAV13`（防静默回退，见 README §八 风险 3）：
   ```bash
   rosparam get /gcs_a_backend_node/formation_offsets_file
   ```
2. [ ] 标定（同 README §4.1，全机 disarm 即可）→ `/group_a/calibration_ready` = True
3. [ ] 下发（**不需要**先 arm）：
   ```bash
   python3 verification/three_uav_recon/driver.py \
     --tcp-host 192.168.5.241 --tcp-port 39001 \
     --log /tmp/noprop_dryrun_$(date +%Y%m%d_%H%M%S).jsonl
   ```
   **任务号口径（与四机文档不同，已按代码核实）**：driver 强制 `bootstrap.mission_id == --mission-id`
   （`four_uav_recon/driver.py:189-192`），所以不要用随日期变的 `--mission-id`（会直接
   `ValueError`）；默认 `three-uav-recon-002` 即可。重试**不需要**换号——每次运行的
   `command_id` 含 `run_id`（epoch 秒，`:211`），不会命中执行器/GCS 的 1 h 任务缓存；
   只有"同 command_id 同内容重发"才会命中。若现场真遇到"秒回上次结果"，再换号
   （同时要改 `package/bootstrap.json`）。
4. [ ] 预期现象（**看清"正常失败"，别误判为缺陷**）：

| 观察 | 期望 | 出处 |
|---|---|---|
| PREPARE | 三机 accepted（机载侧唯一门槛＝本机 pose 新鲜 + MOVE_TO 的原点已确认；GCS 侧仍有标定/编队/链路门） | `ego_swarm.py:710-721` |
| 跟随者事件 | `follower_start` → `leader_odom_first_frame`（`wait_s` ≲ 0.1 s）→ 观察窗内**无** `LEADER_LOST` | `<UAVn>_follower_events.jsonl` |
| 领机 | **不会动**：未 arm ⇒ PX4 忽略 setpoint；规划器完成判据是**真实位置**到达末航点，地面不动 ⇒ 任务最终超时（≤ `uav_task_timeout_s`=280 s）失败 → 组级 FAILED + HOLD 扇出。**领机/整组的终态不作为判据** | `ego_planner_driver_node.cpp` |
| 组级收口（顺带） | 各机 `/UAVn/uav_hold` 是否到达，看 GCS 侧 backend 日志/driver JSONL。**不要**用机载 `relay_events.hold_enter` 当证据：relay 的 HOLD 是"选中源过期"的本地回退 | 同左 |

  观察窗 **≥60 s** 即可收尾（首帧证据在第 1 秒内）；想顺带看组级收口就等它跑到超时。
5. [ ] 收尾与采集（同 README §六）：
   ```bash
   for n in 8 10 13; do ./src/tcp_to_ros/tools/onboard_ctl.sh stop $n; done   # 逐台（多目标只处理第一个）
   ./src/tcp_to_ros/tools/log_fetch.sh 8 10 13        # 采集包含 onboard/health/*.jsonl
   ```
   GCS 侧 health 文件在本机 `<工作区>/.ros_home/log/health/`，手工归进采集包 `gcs/`。

## 五、判据表（定层口径）

各机 + GCS 的健康文件对齐时间轴，看 `src=UAV8, topic=swarm_pose`：

| 现象 | 断在哪一段 |
|---|---|
| 发送侧 `local` 行 `rx` 停 | mavros → bridge（上游节点/本地订阅） |
| 发送侧 `rx` 走、`tx` 停 | bridge 发送路径（`drop` 可区分限频丢弃） |
| 接收侧 `rx` 停、对端 `tx` 走 | ZMQ/电台链路（配合 `link` 事件看是否真断开重连） |
| 接收侧 `rx` 走、但执行器无 `leader_odom_first_frame`/`follower_exit` | 执行器内部 |
| `recv_loop_lag_s` 持续增大（>0.5 s） | 收包线程被饿死/卡住，与"链路没数据"是两回事。该标量由收包线程自己记录；`stuck_sockets` 字段已停用（2026-10-09 跨线程查 ZMQ_EVENTS 致桥堆损坏死掉，见 `quality_reports/2026-10-09_*`），恒为空 |

## 六、已知陷阱

1. `noprop` 跳过 `connected` 检查 ⇒ §一 的人工确认不能省。
2. 3 s 首帧宽限**只在"从未见过领机 odom"时生效**；同一进程内第二次 FOLLOW_ROUTE 直接从
   严格新鲜度分支（`_FOLLOW_POSE_TIMEOUT_S`，2026-10-10 由 1.0 s 放宽为 3.0 s）起跳
   （ego_swarm.py 跨任务不清零）⇒ 反复 dry-run 要么重启该机执行器节点，要么按严格分支口径判读。
3. 地面无轨迹时 `trajectory_intent` 可能无数据流（订阅登记仍在，门禁照旧可查），属预期。
4. 标定 `calibration_ready` 一直 False：先查 §三 部署是否完成、三机是否都 disarm、
   GPS 是否收敛；三机版配置已带 `sample_window_s: 8.0`（限频 vs 窗口跨度修复），
   若仍不 ready 按 `four_uav_recon/README.md` §八 风险 4 逐项查。
5. `onboard_ctl.sh` 多目标只处理第一个（2026-09-22 实测）⇒ 本文件全部逐台调用。

## 七、覆盖边界

- **覆盖**：bridge/电台双向 odom+pose 传递、健康/事件观测落盘与判读、GCS→bridge 服务代理
  （PREPARE/START/HOLD）、跟随环启动/首帧/空档判据。
- **不覆盖**：armed/OFFBOARD 门禁、5 m 软起飞、轨迹执行与真实控制、飞行态负载
  （领机在飞时的 trajectory_intent 等）。
