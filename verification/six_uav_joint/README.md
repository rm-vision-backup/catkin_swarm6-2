# 六机整体流程测试：UAV6–11（A组4架＋模拟B组2架）

2026-10-18，根据现场确认的四排五列、5 m 网格编写。包含任务代码、数据、离线检查及SITL录制工具；各轮仿真结果见文末与对应summary.json，实机尚未部署/验收。上机准备仍按 [统一工作流](../real_uav_workflow/README.md)，本目录替换其中的任务入口、标定、编队与逐机邻居参数。

## 1. 摆位、角色与坐标

面向机头方向，编号从左到右；第一排在最前方。全部机头保持当前飞行方向：

```text
                     机头 / 飞行前方（任务 +y） ↑

第一排               1      2      3      4      5
第二排               6      7      8      9     10
第三排              11     12     13     14     15
第四排              16

                     左 ←             → 右（任务 +x）
                     同排、相邻排间距均为 5 m
```

本轮仅给6–11号下发任务，其余飞机留在地面；其它在线节点的背景电台流量仍需记录。不自动停止其它节点。

| UAV / 任务身份 | 角色 | 起始任务坐标 x/y（m） | 后续行为 |
|---|---|---|---|
| UAV6 / A06 | 侦查成员、预设故障机 | −10 / 0 | 第一段覆盖后降至8m，向左退出到(−20,15)，持续悬停 |
| UAV7 / A07 | 固定领机 | −5 / 0 | 两段覆盖、返回 |
| UAV8 / A08 | 侦查成员 | 0 / 0 | 两段覆盖、返回 |
| UAV9 / A09 | 模拟B组成员1 | 5 / 0 | 到地面目标(5,15,0)上方5m完成模拟打击，持续悬停 |
| UAV10 / A10 | 模拟B组成员2 | 10 / 0 | 到地面目标(10,15,0)上方5m完成模拟打击，持续悬停 |
| UAV11 / A11 | A组备用机 | −10 / −5 | 空中待命，接替UAV6左侧席位，第二段覆盖、返回 |

**标定计算集为同排 UAV6–10 五架，原点位于 UAV8 的地面机位附近；确认集为全部六架。** 五架共线且间距相等，逐轴全球位置中位数对应中间的 UAV8，避免用“同排五架＋下一排一架”的偶数集合推断原点时受场地航向影响。实际原点以当轮冻结与回读为准，不能沿用前轮标定。

全部六架在 disarm 时完成标定，逐架确认 `gp_origin_confirmed=True`；之后由飞手让**全部六架**进入 armed/OFFBOARD、实际约5m稳定悬停。UAV11从一开始就在空中待命，本轮不做中途从地面自动起飞。

任务坐标为右/前/上；机头可以指向任意地理方向，GCS使用冻结航向旋转为ENU。两份编队配置均以 UAV7 为领机，左右偏置为 ±5m，不随航迹 yaw 旋转。

## 2. 完整流程与边界

```text
六机人工起飞、5m悬停
  → prepare：六机各自原位升到12m，备用机和模拟B组保持待命
  → coverage1：UAV6/7/8第一段覆盖
  → fault：UAV6下降至8m，向左离队并悬停
  → recovery：仅UAV11飞入空席位，UAV7/8保持原位，实测确认恢复三机编队
  → coverage2：重组后的三机继续第二段覆盖
  → 本地发布 RECON_COMPLETE
  → strike：UAV9/10到地面目标上方5m
  → strike_hold：在5m连续稳定悬停3s，完成模拟打击
  → 本地发布 STRIKE_COMPLETE；UAV9/10继续在目标上方5m悬停
  → return_a：UAV7/8/11返回各自起始点上方12m
  → hold：侦查组3架在返回点12m、模拟B组2架在目标上方5m确认悬停；UAV6在退出点8m悬停
  → 飞手分别接管六机最终降落
```

UAV6只是**任务名册中的模拟故障**，飞控仍正常、保持通信并参与邻机避碰。`FAULT_EXIT`不执行 LAND；全流程监测仍要求UAV6保持 armed/OFFBOARD，不在中途手动降落它，否则驱动中止。退出成功后才把A06从侦查名册移除；恢复成功后才把A11从reserve移入active。任何阶段失败都停止后续动作，不会用成功反馈替补失败机或跳过故障继续打击。

模拟B组在线上仍使用真实 `GroupA/A09/A10`，由同一个GCS_A串行调度，以 `MOVE_TO` 前往地面目标上方5m；本版不下发B组返回任务。目标高度为0，飞行高度为5，两个数分别保存于world目标和逐机飞行计划，不能将无人机飞行目标改成地面高度0。这里的5m按本轮平地任务原点计，不是地形跟随测距控制。目标停留、A组返回期间和最终收尾均连续核对B组保持点，不再发送同点MOVE_TO或HOVER占位目标；观测阶段没有伪造底层COMPLETED。物理GroupA名册包含模拟B组，侦查编队成员由每阶段计划明确限定。阶段事件仅在本进程内记录和检查；**本轮不验证真实B组接口、跨进程事件传输或正式A/B行为树联合执行**。后期接入真实B组时，这三项需要单独替换和验收。

本轮没有视觉识别、投放装置、自动起飞或自动降落。测试航线是缩小场地的流程验证，不宣称完成100×150m整场覆盖。

## 3. 航迹、净空与高度

领机均在12m飞行，第一段5个航点：

```text
(-5,0) → (-5,10) → (-10,10) → (-10,15) → (-5,15)
```

第二段从同一终点继续，不返回已完成的下半段：

```text
(-5,15) → (-5,25) → (-10,25) → (-10,30) → (-5,30)
```

每段领机折线长25m；跟机左右各偏5m，侦查水平包络为 x=−15..0、y=0..30。退出点为(−20,15,8)；UAV11恢复航路位于左侧 x=−10，模拟B组航路位于右侧 x=5/10。恢复、返回的关键直线几何已做离线间距检查；EGO实际曲线仍须SITL和现场核对。

**相对UAV8地面机位，现场至少预留左25m、右15m、后10m、前35m、上方15m，并另外核对实际障碍、人员及起降区域。** `world.json` 候选场界为 x=−30..20、y=−15..40，内缩5m后包住上述所需范围；它不是对现场场界的实测证明。若净空不足，先修改任务和重新验证，不能直接照飞。

`MOVE_TO`（含准备、恢复、模拟打击和A组返回）沿用生产动作：先到15m转场层，再水平移动，最后到指定目标高度（侦查12m、模拟打击5m）。因此即便起飞只到5m，后续仍需15m净空。退出机在8m水平离队，其他阶段不降低机载安全阈值。

## 4. 文件与离线核对

| 文件 | 用途 |
|---|---|
| `driver.py` | 九阶段编排；复用runtime TCP、批次汇总和名册接替逻辑 |
| `flight_guard.py` | GCS只读观测、终点回读；异常时额外逐机请求六机悬停 |
| `package/` | robots/bootstrap/world/plans；缩小场景，不是正式runtime审核包 |
| `config/` | 初始/恢复两套编队与五机计算、六机确认的标定配置 |
| `gcs_real.launch` | 实机GCS入口，复用现有四机入口并替换本轮配置 |
| `flow.sh` | 固定六机/领机的通信工具包装、逐机启动命令预览及任务入口 |
| `sitl/` | 六机出生布局、仿真GCS入口及手动彩排说明 |
| `test_joint.py` | 本地协议回环、失败分支、几何和观测器测试 |

从工作区根目录执行：

```bash
python3 verification/six_uav_joint/driver.py --dry-run
python3 -m unittest verification.six_uav_joint.test_joint -v
bash verification/six_uav_joint/flow.sh onboard-commands
```

默认运行driver也只做dry-run，不连接ROS/TCP。协议测试只监听127.0.0.1随机端口，不连接实机。现有runtime依赖（含Pydantic 2）和PyYAML须已安装。

本目录`package/`沿用recon的工程测试加载方式，不通过正式runtime `mission.json/tree.xml`审核加载器；文件改动需人工复核并重新离线验证，不能用dry-run成功代替航线审核。

## 5. 通信、部署与任务交接

1. 按统一工作流核对六机身份、router、时钟、机载部署指纹和单setpoint发布者。仅编写本测试不构成部署/启动授权。
2. `flow.sh onboard-commands` **只打印六条命令**；核对后按现场授权逐条执行。每架订阅其余5架的 `trajectory_intent` 与 `swarm_pose`，不含自己；保留已退出UAV6的订阅。机载任务超时为180s，与本驱动运动批次一致。
3. **地面热测（本轮默认只做热测）**：机载栈稳定后、起 GCS 前完成。冷测是另一类实验，必须在第2步之前（栈停）完成，不要和热测混在同一轮。按 [通信方法](../real_uav_workflow/communication.md) 做一次 20 s 热测（其中包含 odom 基线和 ping）；正式的 20 s 窗口与 `flow.sh stage1 20` 二选一，不要重复跑。热测不要求 GCS/标定；若 GCS 已在运行，记录额外节点负载。本轮是6–11号，实际在线背景仍按现场16架记录；六机栈间消息必须逐源、逐接收者核对，不凭GCS单侧收到消息判断全互通。

   ```bash
   # 快速基线 / 只做快查时：
   bash verification/six_uav_joint/flow.sh odom
   # 需要 20 s 判定且未做正式热测时（与上面的正式热测二选一）：
   bash verification/six_uav_joint/flow.sh stage1 20
   # 录包不在热测窗口内进行；地面标定完成后见第 5.1 步（可选）
   ```

4. 本轮GCS启动命令（在独立终端运行并保存输出）：

   ```bash
   env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
     bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
              roslaunch verification/six_uav_joint/gcs_real.launch'
   ```

5. 查询终端设置相同GCS环境并source工作区。核对 `/gcs_a_backend_node/formation_offsets_file` 和 `/gcs_a_calibration/calibration_uavs` 实际加载本目录配置。全部disarm时标定，确认六机原点回读与 `/group_a/calibration_ready=True`。**任何机载栈重启、换电或飞机移动后，必须重启 GCS 标定节点（或整个 GCS 栈）重新标定，不能沿用旧 ready。**

5.1 **可选：启动机载 bag 录制（实验数据留存）**。地面标定完成并核对通过、飞机仍 disarm 且未移动时，在 GCS 执行：

   ```bash
   bash verification/six_uav_joint/flow.sh bag 1200
   ```

   - 录制口径（`air_flow.sh bag` 自动按角色传 `-u`）：**跟机录本机完整话题 + 领机 UAV7 的转发话题**（`swarm_pose`、`mavros/state` 等）；**领机录本机完整话题 + 其余所有机的转发话题**，作为全组互联证据。GCS 的 `group_a/gp_origin`、`gps_bias` 始终保留。
   - 本机完整话题含轨迹/状态/控制/任务：`mavros/local_position/{odom,pose}`、`mavros/state`、`mavros/setpoint_raw/local`、`setpoint/ego|follower`、`exec_state`、`uav_task_state`、`trajectory_intent`、`rc/in`、`battery`、`rosout` 等；用于验收时还原飞行路线、控制输入与任务阶段。
   - `1200` 为默认时长（秒），按预计实验时长调整。正常实验建议保留此项，后续用 bag 支撑轨迹/链路结论。
   - 文件在各机 `~/catkin_swarm6-2/.ros_home/log/rosbag/<UAV>_<时间>_air.bag`；**落地 disarm 后再停止或取回，不在空中下载**。
   - bag/ulog 等大文件走 USB 或落地后归档；电台只取 health、terminal、driver 等小文件。
   - bag 只依赖机载栈，不依赖 GCS；若标定因重启/换电/移动而失效，需重新标定，bag 可继续录制。

6. 飞手让六机依次进入armed/OFFBOARD，确认各机实际约5m悬停。地面 `flow.sh odom`/热测通过即可作为通信门禁；起飞后是否再做一次 quick odom 由现场负责人决定（默认建议做一次约10s，但不作为硬性门禁）。若跳过，在 `session.md` 记 `SKIP（原因）`。地面检查不能覆盖空中姿态、天线朝向、多径和电机干扰差异；新链路版本或曾出现空中断流时仍建议快查。若本轮已做完整 20 s 热测，空中只做 quick odom，不重复 20 s 窗口。地面或空中核对完成后再下发任务。
7. 以下命令交给**GCS操作员**：在 `/home/yjq/catkin_swarm6-2` 直接复制执行。命令已包含 GCS ROS 环境覆盖；driver 会自动在 `src/uav_log/<时间>_six-uav-joint/driver.jsonl` 落盘（需要自定义时再加 `--log`）：

   ```bash
   env -u ROS_HOSTNAME ROS_IP=192.168.5.241 ROS_MASTER_URI=http://localhost:11311 \
     bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash &&
              python3 verification/six_uav_joint/driver.py --execute \
                --tcp-host 192.168.5.241 --tcp-port 39001'
   ```

   执行前必须已完成第6步（六机 armed/OFFBOARD、约5m稳定悬停、起飞后 odom 快查通过）。driver 启动后自动执行全部阶段，不再等待 `CONTINUE`；失败、超时、断线或飞手切出 OFFBOARD 时自动中止并请求 HOLD。通信测试证据和净空由现场确认，driver的GCS观测不能代替机间全互通检查。

8. 首轮可先 `--until prepare` 或 `--until coverage1`，只运行从准备开始的前缀；记录为 `PARTIAL_SUCCESS`。停止后保持当前悬停，最终人工收尾。**不支持跳过前序、从半途中续跑；下一轮须重新回到已标定起始机位上方约5m，移动地面飞机或重启后按工作流重做准备/标定。** 不能套用旧`air_flow.sh full/stage2`：本测试使用`flow.sh run`或本driver执行整体流程。

## 6. 监测、中止与验收口径

驱动只核对六机最新 MAVROS state 的内容（connected/armed/OFFBOARD）与标定 ready，不再设置状态年龄上限；链路丢失由 backend 的 `UavLinkMonitor`（10 s suspect / 15 s lost）和机载执行器负责。连续的 `swarm_pose` 新鲜度与机间距离由机载执行器负责（领机 odom 超时 3.0s → `LEADER_LOST`；三维中心距 <1m → HOLD），地面站不再重复设置更严的门槛。逐机 `gp_origin_confirmed=True` **不进入 driver 自动检查**：桥上中继不保留 latch，晚启动的 driver 收不到标定时一次性发布的 True；须由操作员起飞前人工核对六架，尤其 UAV11（不在 `calibration_ready` 门槛内）。driver 不再自动检查起始位置/高度；操作员必须在起飞前确认六机按预期摆位并稳定在约5m。每批成功后核对实测终点（水平≤2m、高度≤1m），覆盖跟机终点由本轮±5m偏置计算；目标停留3s期间也持续检查。以上是本测试的额外观测条件，不修改机载生产参数。

失败、超时、Ctrl-C或飞手切出OFFBOARD时，停止后续命令，先请求TCP HOLD，再经现有 `/UAVn/uav_hold` 向六机发送 `RUNTIME_BRAKE_HOLD`（每批3架、有界等待）。**TCP HOLD本身只覆盖当前活动批次；逐机服务的accepted也只表示接收请求。** JSONL逐机保留请求结果和未确认清单，飞手仍须确认实际悬停并接管；有失败或服务超时不宣称全组六机已安全悬停。安全锁存后不在空中自动复位或重试任务。

每次执行生成唯一mission_id，避免消费前轮缓存失败；JSONL包含实际mission_id、每条线上命令、逐机反馈（含error_message）、每阶段实测ENU位置、名册变化与本地阶段事件。超时、漏报COMPLETED成员、错误成功反馈都不能自动进入下一阶段。

完整通过要求：9阶段成功、A06退出后不再参加后续任务、A11加入后第二段覆盖成功、RECON_COMPLETE之后才出现模拟B动作、B组在目标上方5m完成模拟打击后A组才返回、A返回期间B持续保持5m，全程无破距/规划/链路/模式异常。最终现场确认侦查组3架返回点12m＋模拟B组2架目标上方5m＋UAV6退出点8m悬停。`mission_finished=SUCCESS`只表示本轮驱动与观测条件通过，仍须结合轨迹/机间链路/现场记录验收，不等于30+机或真实B组已通过。

本次离线验证：23项测试通过，覆盖正常TCP闭环、退出/恢复失败、漏报成员、断线、假完成、超时、操作员中止、名册切换、几何与观测条件，并检查地面目标/5m飞行高度区分、B组5m保持及A返回前B组漂移中止；完整任务dry-run通过。当前9阶段包含7个运动批次、19条逐机请求和2个观测保持阶段；实机及仿真GCS launch解析通过。六机实飞及实际电台链路尚未验证。

SITL历史：`.tmp/six_uav_joint/sitl_video_20261018_02` 在恢复批次超时并触发六机悬停收口，失败证据保留。该轮暴露原地幸存机重复收到同点MOVE_TO后没有新完成反馈；本测试改为只让备用机移动，随后实测核对完整恢复队形。当前下层HOVER占位处理也未在本测试中作为可靠冻结动作使用，保持阶段改为明确的连续位置观测；不通过伪造COMPLETED绕过失败，也未修改下层安全门禁。

历史旧版：2026-10-18 两组返回流程的完整PX4/Gazebo SITL通过1轮，证据在 `.tmp/six_uav_joint/sitl_video_20261018_03/`，当时10阶段全部SUCCESS，其中8个运动批次、21条逐机请求；5Hz最小采样三维间距4.020m。这是目标12m且B组返回的旧流程，不能代替当前地面目标/5m持续悬停版本的验收。

**当前地面目标版完整PX4/Gazebo SITL通过1轮（2026-10-18）**：本轮证据在 `.tmp/six_uav_joint/sitl_ground_targets_20261018_04/`，9阶段全部SUCCESS，7个运动批次、19条逐机请求。飞行阶段5Hz最小采样三维间距为 **3.938m**。UAV9/10在目标上方连续保持约 **56.86s**，该窗口实测高度 **4.894–5.099m**、最大水平误差0.089m；最终两架高度分别约5.044m、5.052m，A组3架返回约12m、UAV6保持退出点约8m。23项自动测试和生产TCP→ROS→executor适配校验通过。版本化轻量证据见 [验证摘要](sitl/results/2026-10-18_ground_targets.json)，其中包含任务/配置哈希；原始轨迹和视频保留于本地本轮目录。

最新本轮目录中的文件：

- `summary.json`、`driver.jsonl`：仿真总结果、逐阶段反馈与名册变化。
- `telemetry.jsonl`、`verification_report.json`：实测轨迹、逐阶段耗时、距离统计及限定范围。
- `full_mission_3d.mp4`：实际轨迹三维回放，1280×720、12fps、4倍速，包含起飞、全部任务阶段和悬停收尾。
- `full_mission_3d.json`：视频来源、阶段结果、播放倍率、帧数及距离口径。

本结论仅适用于本轮本机SITL；实机电台、现场部署、真实B组接口与跨地面站协同尚未验收。视频不包含最终人工降落。
