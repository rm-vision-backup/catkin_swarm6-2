#!/usr/bin/env python3
"""从真实SITL的telemetry.jsonl渲染完整三维任务视频，不生成替代飞行轨迹。"""
import argparse
import bisect
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

matplotlib.rcParams.update({"font.sans-serif": ["Noto Sans CJK SC", "Noto Sans CJK JP", "DejaVu Sans"],
                          "axes.unicode_minus": False})
PHASES = [("takeoff", "人工起飞准备（SITL脚本）"), ("prepare", "六机准备 / 空中待命"),
          ("coverage1", "第一段覆盖：6 / 7 / 8"), ("fault", "UAV6 下降8m、离队"),
          ("recovery", "UAV11 接替、恢复编队"), ("coverage2", "第二段覆盖：7 / 8 / 11"),
          ("strike", "模拟B组到地面目标上方5m"), ("strike_hold", "模拟打击 / 5m悬停"),
          ("return_a", "侦查组返回 / B组保持5m"),
          ("hold", "任务收尾 / 悬停"), ("complete", "完整任务完成")]
COLORS = {"A06": "#ed8d36", "A07": "#2775dc", "A08": "#04a6bf",
          "A09": "#29975c", "A10": "#76a82c", "A11": "#965bc7"}
LABEL_DZ = {"A06": 0.6, "A07": 1.6, "A08": 0.6, "A09": 1.6, "A10": 0.6, "A11": 2.4}


def render(run, output, fps=12, speed=4):
    rows = [json.loads(line) for line in (run / "telemetry.jsonl").read_text().splitlines() if line]
    # 标定期间各机EKF原点尚未统一，不能把这些local坐标用于机间距离或飞行回放。
    # record_run仅在六机原点全部回读确认后才将phase切换为takeoff。
    raw_count = len(rows)
    rows = [row for row in rows if row["phase"] != "calibration"]
    summary = json.loads((run / "summary.json").read_text())
    driver_rows = [json.loads(line) for line in (run / "driver.jsonl").read_text().splitlines() if line]
    results = {r["phase"]: r["outcome"] for r in driver_rows if r.get("event") == "phase_result"}
    expected = [key for key, _ in PHASES if key not in {"takeoff", "complete"}]
    if summary["status"] == "SUCCESS" and (set(results) != set(expected) or any(v != "SUCCESS" for v in results.values())):
        raise ValueError("summary与完整阶段结果不一致，不能制作成功视频")
    if len(rows) < 2:
        raise ValueError("实测轨迹不足，不能制作视频")
    times = np.array([row["time"] for row in rows])
    ids = list(COLORS)
    paths = {u: np.array([r["poses"][u] for r in rows]) for u in ids}
    t0 = times[0]
    duration = times[-1]-t0
    frame_times = list(np.arange(times[0], times[-1], speed/fps)) + [times[-1]]*int(fps*2)
    min_distance = min(float(np.linalg.norm(paths[a]-paths[b], axis=1).min())
                       for i, a in enumerate(ids) for b in ids[i+1:])
    figure = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor="#f5f7fb")
    ax = figure.add_axes([0.0, 0.12, 0.74, 0.77], projection="3d")
    ax.set_facecolor("#f5f7fb")
    ax.view_init(elev=26, azim=-58)
    ax.set_xlim(-26, 16); ax.set_ylim(-12, 36); ax.set_zlim(0, 18)
    ax.set_xlabel("右 / x (m)", labelpad=9)
    ax.set_ylabel("前 / y (m)", labelpad=9)
    ax.set_zlabel("高度 / z (m)", labelpad=4)
    ax.grid(True, alpha=0.2)
    ax.tick_params(labelsize=8)
    for z, color, label in [(5, "#b5d7be", "5m 打击悬停层"),
                            (8, "#d8a278", "8m 退出层"), (12, "#9cb9d9", "12m 任务层"),
                            (15, "#b7accc", "15m 转场层")]:
        ax.plot([-24, 14, 14, -24, -24], [-10, -10, 34, 34, -10], [z]*5,
                color=color, alpha=0.45, linewidth=0.6)
        ax.text(-24, 34, z+0.2, label, fontsize=7, color="#727c8e")
    # 现场16架地面摆位作为背景；只绘制6–11的飞行实测轨迹。
    for number in range(1, 17):
        x = ((number-1)%5-2)*5
        y = (1-(number-1)//5)*5
        ax.scatter([x], [y], [0], s=13, color="#b9c1cf", alpha=0.7)
        ax.text(x, y, 0.3, str(number), fontsize=6, color="#8b95a6")
    ax.scatter([5, 10], [15, 15], [0, 0], marker="*", s=150,
               color="#519d61", alpha=0.8)
    for x, n in [(5, 9), (10, 10)]:
        ax.text(x, 15, 0.5, "地面目标%d" % n, fontsize=7, color="#427c4a")
        ax.plot([x, x], [15, 15], [0, 5], color="#519d61", linestyle="--", alpha=0.5)
    figure.text(0.04, 0.955, "六机联合任务 · 三维实测轨迹", fontsize=19, weight="bold", color="#20304b")
    figure.text(0.04, 0.91, "PX4 + Gazebo SITL  |  UAV6–11  |  A组4架 + 模拟B组2架", fontsize=10, color="#65738c")
    figure.text(0.76, 0.86, "任务进度", fontsize=13, weight="bold", color="#20304b")
    phase_texts = [figure.text(0.76, 0.81-i*0.044, "○  " + text, fontsize=9, color="#929bac")
                   for i, (_, text) in enumerate(PHASES)]
    clock = figure.text(0.04, 0.055, "", fontsize=12, color="#324867")
    note = figure.text(0.76, 0.19, "", fontsize=9, color="#5b6980", va="top")
    figure.text(0.04, 0.019, "画面由本轮ROS实测位置重建；模拟B组使用A组MOVE_TO及终点保持。最终降落由人工接管，不在任务中。",
                fontsize=8, color="#7d8798")
    progress, = figure.add_axes([0.04, 0.095, 0.91, 0.007]).plot([0, 0], [0, 0], color="#2775dc", linewidth=5)
    progress.axes.set_xlim(0, 1); progress.axes.set_ylim(-1, 1); progress.axes.axis("off")
    drone = {}
    for u, color in COLORS.items():
        arms = [ax.plot([], [], [], color=color, linewidth=2.0)[0] for _ in range(2)]
        trail, = ax.plot([], [], [], color=color, linewidth=1.15, alpha=0.55)
        rotors, = ax.plot([], [], [], "o", color=color, markersize=3)
        stem, = ax.plot([], [], [], color=color, linewidth=0.6, alpha=0.25)
        text = ax.text(0, 0, 0, "UAV"+str(int(u[1:])), fontsize=8, color=color, weight="bold")
        drone[u] = (arms, trail, rotors, stem, text)
    links, = ax.plot([], [], [], color="#3187cc", alpha=0.35, linewidth=1)
    writer = FFMpegWriter(fps=fps, codec="libx264", bitrate=3500,
                          extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
                          metadata={"title": "Six UAV full mission - measured PX4 SITL"})
    output.parent.mkdir(parents=True, exist_ok=True)
    with writer.saving(figure, str(output), dpi=100):
        for frame, stamp in enumerate(frame_times):
            index = min(max(bisect.bisect_right(times, stamp)-1, 0), len(rows)-1)
            after = min(index+1, len(rows)-1)
            fraction = 0 if index == after else (stamp-times[index])/(times[after]-times[index])
            poses = {u: paths[u][index]*(1-fraction)+paths[u][after]*fraction for u in ids}
            phase = rows[index]["phase"]
            number = next((i for i, (key, _) in enumerate(PHASES) if key == phase), -1)
            for i, text in enumerate(phase_texts):
                marker = "●" if i == number else "✓" if i < number else "○"
                text.set_text(marker+"  "+PHASES[i][1])
                text.set_color("#2775dc" if i == number else "#3c7b58" if i < number else "#929bac")
            for u, (arms, trail, rotors, stem, text) in drone.items():
                x, y, z = poses[u]
                for line, dx, dy in [(arms[0], 0.7, 0.7), (arms[1], 0.7, -0.7)]:
                    line.set_data([x-dx, x+dx], [y-dy, y+dy]); line.set_3d_properties([z, z])
                rotors.set_data([x-.7, x+.7, x-.7, x+.7], [y-.7, y+.7, y+.7, y-.7]); rotors.set_3d_properties([z]*4)
                stem.set_data([x, x], [y, y]); stem.set_3d_properties([0, z])
                trail.set_data(paths[u][:index+1:2, 0], paths[u][:index+1:2, 1]); trail.set_3d_properties(paths[u][:index+1:2, 2])
                text.set_position((x, y)); text.set_3d_properties(z+LABEL_DZ[u], zdir=None)
            members = ["A06", "A07", "A08"] if number <= 2 else ["A11", "A07", "A08"] if number >= 5 else []
            if members:
                points = [poses[u] for u in members]
                links.set_data([p[0] for p in points], [p[1] for p in points]); links.set_3d_properties([p[2] for p in points])
            else:
                links.set_data([], []); links.set_3d_properties([])
            elapsed = min(stamp-t0, duration)
            clock.set_text("任务时间 %05.1fs / %.1fs     |     %.0f× 播放     |     最小采样间距 %.2fm" %
                           (elapsed, duration, speed, min_distance))
            progress.set_data([0, elapsed/duration], [0, 0])
            if phase == "complete":
                note.set_text("本轮结果："+summary["status"]+"\n侦查组3架返回约12m\nUAV9/10目标上方5m悬停\nUAV6在退出点8m悬停")
                note.set_color("#32815a")
            elif phase == "failed":
                note.set_text("本轮失败 / 已停止推进\n请查本轮日志")
                note.set_color("#b54440")
            else:
                note.set_text("蓝色：领机7 / 侦查8\n橙色：退出机6\n紫色：备用机11\n绿色：模拟B组9 / 10")
            if frame == 0:
                figure.savefig(str(output.with_suffix(".png")), dpi=100)
            writer.grab_frame()
            if frame % 120 == 0:
                print("render %d/%d" % (frame, len(frame_times)), flush=True)
        figure.savefig(str(output.with_name(output.stem+"_end.png")), dpi=100)
    plt.close(figure)
    meta = {"source": str(run), "status": summary["status"], "simulation": summary["simulation_type"],
            "source_duration_s": duration, "video_duration_s": len(frame_times)/fps,
            "speed": speed, "fps": fps, "frames": len(frame_times), "min_distance_m": min_distance,
            "excluded_precalibration_samples": raw_count-len(rows),
            "phases": list(dict.fromkeys(row["phase"] for row in rows)), "phase_results": results}
    output.with_suffix(".json").write_text(json.dumps(meta, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(meta, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--speed", type=float, default=4)
    args = parser.parse_args()
    render(args.run, args.out, args.fps, args.speed)
