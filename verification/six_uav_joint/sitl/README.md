# 六机PX4 SITL彩排入口

## 完整仿真与三维录制

从工作区根目录运行下面的仿真专用脚本，输出目录必须是新目录。脚本仅使用本机SITL，检查端口后启动自己的Gazebo、六机执行层和GCS；使用TCP 39061，自动执行仿真起飞及仿真阶段确认，实机driver的交互规则不变。记录5Hz实测三维轨迹、全部阶段反馈及最终结果，仅清理自己启动的进程。

```bash
bash verification/six_uav_joint/sitl/run_recording.sh \
  --outdir .tmp/six_uav_joint/<新仿真轮次>
MPLCONFIGDIR=/tmp/six_uav_video_mpl python3 verification/six_uav_joint/sitl/render_video.py \
  .tmp/six_uav_joint/<新仿真轮次> \
  --out .tmp/six_uav_joint/<新仿真轮次>/full_mission_3d.mp4 --speed 4
```

替换`<新仿真轮次>`后执行。渲染器使用该轮实测轨迹，不生成替代路径；MP4及同名JSON明确记录仿真结果、播放倍率、实测时长、阶段和最小实测三维间距。只有`summary.json`与driver九阶段全部成功，才能报告完整SITL通过。地面目标显示在0m，UAV9/10在目标上方5m完成模拟打击后持续悬停，不再返回；侦查组3架返回约12m、UAV6退出点约8m。视频不包含人工最终降落。

## 手动分阶段启动

此目录提供与实机同构的UAV6–11出生布局及GCS配置。未自动运行；避免与当前实机GCS或其它SITL实例争用39001、ROS Master及Gazebo端口。此流程不使用全局cleanup、不删除已有日志。仅在已授权的独立仿真会话中操作，结束时逐个终端Ctrl-C，并仅收口本轮启动的进程。

各终端从工作区根目录开始，source `/opt/ros/noetic/setup.bash` 和 `devel/setup.bash`。仿真环境沿用现有工程：加载 `$PX4_ROOT/Tools/setup_gazebo.bash`（PX4_ROOT取实际安装目录）并把PX4及`Tools/sitl_gazebo`加入ROS_PACKAGE_PATH；详见 [现有五机脚本](../../five_uav_recon/sitl/run_sim_five_uav.sh) 与 `src/swarm_simulation/bashs/env.sh`。

1. 仿真场地，单独终端：

   ```bash
   export ROS_MASTER_URI=http://localhost:11300 ROS_HOSTNAME=localhost
   export GAZEBO_MASTER_URI=http://localhost:11345
   roslaunch verification/six_uav_joint/sitl/six_uav_spawn.launch gui:=false
   ```

2. 六机执行层，单独终端（已有脚本按11310+编号创建各机Master；不调用其stop子命令）：

   ```bash
   EGO_TASK_TIMEOUT_S=180 bash src/swarm_uav_executor/scripts/startup_offboard_ego.sh 6 7 8 9 10 11
   ```

   这是仿真专用入口，沿用它默认的全目录邻机订阅；实机用上一级`flow.sh onboard-commands`生成精确六机订阅。确认6条`ready UAVn`，并保留本轮日志。所有机保持disarm完成下一步标定。

3. GCS，单独终端：

   ```bash
   export ROS_MASTER_URI=http://localhost:11310 ROS_HOSTNAME=localhost
   roslaunch verification/six_uav_joint/sitl/gcs_sim.launch
   ```

   同一Master查询`/group_a/calibration_ready`和六机`gp_origin_confirmed`。编队文件必须是本目录上一级`config/formation_offsets.yaml`。

4. 仿真机外部起飞（仅SITL脚本，禁止替换为实机起飞命令）：

   ```bash
   python3 src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py 6 7 8 9 10 11
   ```

   确认六机实际5m稳定悬停、armed/OFFBOARD和消息连续性后执行驱动。driver的观测与阶段确认保持启用：

   ```bash
   export ROS_MASTER_URI=http://localhost:11310 ROS_HOSTNAME=localhost
   python3 verification/six_uav_joint/driver.py --execute \
     --tcp-host 127.0.0.1 --log .tmp/six_uav_joint/<新轮次>/driver.jsonl
   ```

   替换`<新轮次>`，不要覆盖既有轮次。先跑`--until prepare`、落地收口后新轮再跑全流程；完整成功要求上一级README的全部条件。离线TCP测试不等于此处SITL已通过。
