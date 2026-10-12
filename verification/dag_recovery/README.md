# DAGL + EGO：通信拓扑恢复实验

## 一句话看懂

**DAGL 决定幸存无人机去哪里；EGO 决定怎么安全飞过去。** 地面站从实测位置建立 8 m 距离阈值图，令若干飞机受控下降并从**逻辑图**退出，DAGL 算新目标，现有 A 组执行链路分批执行。退出机仍在空中、保持无线电连接并纳入避碰。**这不是物理通信断链恢复，也没有运行 GCBF+ 或 `runtime`。**

数据流：`PX4/MAVROS 位姿 → GCS bridge → DAGL 独立进程 → 目标门禁 → ExecuteGroupTask → EGO → setpoint_relay → MAVROS`。不用 TCP Server、行为树或 `FOLLOW_ROUTE`；实飞和 SITL 分开启动。

## 先验证十机仿真

十机布局见 `scenarios/10.sitl.json`：8 m 通信半径；选择 `--faults 3`（A04/A05/A06）或 `--faults 4`（再加 A10）。初始图连通，退出后两组，目标是在实测图上重新连成一组并持续 10 秒。退出机从 15 m 降到 8 m 悬停，其余飞机维持 15 m；从退出完成后计时。

所有示例输出目录统一放在工作区 `.tmp/dag_recovery/` 下（已 gitignore，重启不清空；
旧的 `/tmp` 写法会在重启后丢失）。

先跑**不用 ROS/不会启动无人机**的测试。固定目标文件只测试流程，**不是 DAGL 算法结果，更不会出现在 QGC 中**：

```bash
python3 -m unittest discover -s verification/dag_recovery -p 'test_*.py' -v
python3 verification/dag_recovery/sim_multi.py --faults 3 \
  --snapshot verification/dag_recovery/fixtures/ten_enu.json \
  --targets-fixture verification/dag_recovery/fixtures/ten_three_targets_fixture.json \
  --outdir .tmp/dag_recovery/dag10_offline_3
```

验证四故障：把 `3`、`three` 和输出目录分别改成 `4`、`four` 和一个新目录。固定目标仅用于检查流程，不是模型训练结果。

## 训练十机 DAGL

原始无预训练 DAGL 在此场景将目标挤到 0.08/0.11 m；现在**十机必须显式加载重新训练且通过验证的检查点**。训练仅使用 `references/DAG-GCBF` 的损伤注意力与二分图 GCN，不修改原仓库权重；每个受损后的幸存连通分量由 GCN 预测共同平移，保留内部队形。自动生成的几何可行目标只作训练标签和基线，**推理时不读取标签，也不拿固定 fixture 代替模型**。训练采用 8 m 输入尺度、可导桥接边损失、目标与轨迹间距损失；最终仍须通过 7 m 连通余量、4 m 最小间距、场界和航路硬门禁。最大位移只记录，不作为硬限制。

本机 DAGL 的依赖已安装在 conda 环境，不需要装在系统 Python。训练可较久且当前 `torch.cuda.is_available()` 为 `False`；从工作区根目录执行（每轮使用**全新**输出目录）：

```bash
$HOME/.conda/envs/gcbfplus/bin/python3 verification/dag_recovery/train_dagl10.py \
  --epochs 55 --outdir .tmp/dag_recovery/dagl10_train_1
```

输出 `VALIDATED checkpoint: .tmp/dag_recovery/dagl10_train_1/model.pt` 才算训练完成：至少每组故障 300 例训练、100 例验证和 100 例独立测试，固定 3/4 故障布局及验证/测试数据都须**全部通过硬门禁**。当前代码及种子下，`--epochs 55` 已在本机完成两组各 100 例验证、各 100 例独立测试和固定布局的离线验收；这**不是** SITL 验收。核对 `training_summary.json` 和 `model.json`；没有 `model.pt` 或进程返回非零，就是尚未训练成功，**禁止继续 SITL**。调整 `--epochs`、`--train-samples` 等参数重试须使用新目录。`--smoke` 只能测试训练管线，永远不会导出可用检查点。

## 离线复核（不启动 PX4）

确认训练成功后，运行**真正的模型权重**与安全门禁；两轮均成功后才考虑 SITL：

```bash
bash verification/dag_recovery/run_sim10.sh --faults 3 \
  --checkpoint .tmp/dag_recovery/dagl10_train_1/model.pt \
  --outdir .tmp/dag_recovery/dag10_preflight_3 --check-algorithm
bash verification/dag_recovery/run_sim10.sh --faults 4 \
  --checkpoint .tmp/dag_recovery/dagl10_train_1/model.pt \
  --outdir .tmp/dag_recovery/dag10_preflight_4 --check-algorithm
```

检查各自 `<输出目录>/algorithm_preflight/candidate_metrics.json`、`dag_output.json`、`summary.json` 及 `provenance.json` 的检查点哈希；失败轮必须保留。权重缺失、版本/源码哈希不匹配、目标过近均会拒绝，不会退回原随机模型。离线通过不等于 SITL 通过。

## SITL + QGC

先打开 QGC，确认无人机仿真场地安全后运行（每次新目录；先 3 故障，成功后再 4 故障）：

```bash
bash verification/dag_recovery/run_sim10.sh --faults 3 \
  --checkpoint .tmp/dag_recovery/dagl10_train_1/model.pt \
  --outdir .tmp/dag_recovery/dag10_sitl_run_1 --execute
```

四故障运行：将 `--faults 3` 改为 `--faults 4`，`--outdir` 换成新目录。每次先用同一检查点离线预检，不合格**不启动 PX4**；起飞后还会基于**仿真实测机位**重新推理和验证，再由 EGO 分批执行。QGC 可看到虚拟飞机；脚本退出只清理由它启动的进程。**不要同时运行 `round_prepare_15sitl.sh`**；两者使用相同的 PX4/Gazebo 端口。SITL 不接受固定目标文件，算法失败绝不切换为演示目标。`--execute` 才会启动仿真机。

## 怎么判断结果

查看 `<输出目录>/experiment/summary.json`（离线样例直接在输出目录下）：`OFFLINE_ONLY` 只代表示例校验通过；`SITL_DAGL_EGO_GRAPH_RECOVERED` 才表示真实 DAGL + EGO 的仿真闭环通过。`events.jsonl` 记录全队位姿和 5 Hz 逻辑图、安全事件；`dag_input.json` / `dag_output.json`、`worker.log`、坐标快照和 `provenance.json` 用于复查。**代码提供仿真入口，不等于已跑通十机 SITL；未运行前不得宣称验证成功。** EGO 实际曲线不等于 GCBF+ 的控制曲线；批次同步直线门禁也不证明实际规划轨迹的全时段间距，必须核对事件/轨迹/最小三维间距与失败轮。

## 二十机训练与离线验证

`scenarios/20.training.json` 是单独的 **20 节点二维训练/离线场景**：通信半径 8 m、最小目标和航路间距 4 m、无位移硬上限。损毁 3 架为 A09/A10/A11；损毁 4 架再加 A20。故障前连通，退出后恰好两块。教师目标由两块自动平移得到，并经过分批运动门禁；模型以最多四架为一批预测共同位移，**批与批的位移可以不同**。训练复用损伤注意力和 GCN，但**不会加载十机权重或 N20 原仓库权重**。

从仓库根目录训练（使用新目录；CPU 训练可能耗时）：

```bash
$HOME/.conda/envs/gcbfplus/bin/python3 verification/dag_recovery/train_dagl20.py \
  --epochs 55 --outdir .tmp/dag_recovery/dagl20_train_1
```

只有打印 `VALIDATED checkpoint` 且 `training_summary.json` 中两组固定布局、验证、测试全部通过，才可用生成的 `model.pt`。否则停止并查看失败数据，不能用 `--smoke` 生成正式权重。训练成功后，对**模型推理结果**分别做离线验收：

```bash
$HOME/.conda/envs/gcbfplus/bin/python3 verification/dag_recovery/eval_dagl20.py \
  --faults 3 --checkpoint .tmp/dag_recovery/dagl20_train_1/model.pt --outdir .tmp/dag_recovery/dagl20_offline_3
$HOME/.conda/envs/gcbfplus/bin/python3 verification/dag_recovery/eval_dagl20.py \
  --faults 4 --checkpoint .tmp/dag_recovery/dagl20_train_1/model.pt --outdir .tmp/dag_recovery/dagl20_offline_4
```

各目录的 `summary.json` 须为 `OFFLINE_DAGL20_VALIDATED`；包含所有候选分支、目标点、间距和哈希。可以用 `--positions-json` 指定与模板每机相差不超过 1 m 的实测任务坐标，再做独立离线复核。**目前 `run_sim10.sh` 和 QGC 入口只支持十机，不接受二十机权重；二十机 PX4/Gazebo SITL、机载/地面站部署与闭环实测尚未实现，不得把上述离线结果称为二十机飞行仿真。**

## 实物实验另有门禁

`experiment.py` 是**独立的单故障实飞工具**（4 机→11 机→15 机），默认离线；十机多故障 SITL **没有**解除任何实飞审批或变成实飞指令。真实起飞前还须完成当日机体/身份/共享原点/遥控/净空/法规许可检查，使用 `prepare_deployment.py` 配置全体机载和 GCS，并填写 `approval.template.json` 的签核及配置哈希；`--execute` 才允许接管**已经起飞到 5 m**的实机，程序不会为实机解锁起飞。先完成不上桨、任务态链路与全员 HOLD 确认，再做 1→2→4 机、最后 11 机（至少三轮）和另行审批的 15 机。若出现全体 HOLD 未确认，现场人员须立即遥控接管，保留失败数据，禁止放宽阈值。

完成独立现场审批后，单故障工具的调用形式为（`<...>` 由当日生成的配置替换，**不能用于十机多故障实验**）：

```bash
python3 verification/dag_recovery/experiment.py --execute \
  --scenario <签核的场景.json> --identity <签核的身份.yaml> \
  --deployment <生成目录>/deployment.json --approval <当日签核.json> \
  --worker-python <含PyTorch的Python> --outdir <全新实飞记录目录>
```
