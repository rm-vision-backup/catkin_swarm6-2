#!/usr/bin/env python3
"""从本轮 SITL 的 telemetry.jsonl 渲染三维任务视频；不生成替代轨迹。

输入目录需含：mission_spec.json、fleet.yaml、telemetry.jsonl、summary.json、
driver.jsonl（record_run.py 自动生成）。
"""
from __future__ import annotations

import argparse
import bisect
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

matplotlib.rcParams.update({
    "font.sans-serif": ["Noto Sans CJK SC", "Noto Sans CJK JP", "DejaVu Sans"],
    "axes.unicode_minus": False,
})

PHASES = [
    ("takeoff", "SITL 起飞准备"),
    ("prepare", "初始 9 架 3×3 准备 / 备用机原地待命"),
    ("coverage1", "第一段覆盖（3×3）"),
    ("fault", "边缘列下降 8m、离队"),
    ("recovery", "备用机补位 / 恢复 3×3"),
    ("coverage2", "第二段覆盖（3×3）"),
    ("strike", "B 组到侦查区内目标上方 5m"),
    ("strike_fail", "B 组 1 架失败，目标附近悬停"),
    ("strike_reinforce", "第 4 架补打"),
    ("strike_hold", "模拟打击 / 5m 悬停"),
    ("return_a", "A 组返回 / B 组保持 5m"),
    ("hold", "任务收尾 / 悬停"),
    ("complete", "完整任务完成"),
]

CORE_COLORS = ["#04a6bf", "#2f7fbf", "#3b9e8f", "#5b8fd9", "#1f8a9e",
               "#6aa5c8", "#3f6fb5", "#7aa6a0", "#4f9bb5", "#2b6f8f"]
FAULT_COLORS = ["#ed8d36", "#d98b3a", "#c77a2e", "#e0a24a"]
SPARE_COLORS = ["#d4b106", "#b08a2e", "#c9a227", "#8f7a2a"]
B_COLORS = ["#29975c", "#76a82c", "#965bc7", "#b06ab3", "#4f9f4f"]


def load_fleet(run: Path, spec: dict) -> dict:
    path = run / "fleet.yaml"
    if not path.exists():
        path = Path(spec.get("fleet_file", ""))
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def grid_positions(fleet: dict, shift_y: float) -> dict:
    rows = fleet["grid"]["row_uavs"]
    ys = fleet["grid"]["row_y_m"]
    xs = fleet["grid"]["columns_m"]
    result = {}
    for row, y in zip(rows, ys):
        for index, uav in enumerate(row):
            result[str(uav)] = (float(xs[index]), float(y) + shift_y)
    return result


def assign_colors(spec: dict) -> tuple[dict, dict]:
    colors = {}
    leader = spec["leader_id"]
    colors[leader] = "#2775dc"
    faulted = set(spec["faulted_ids"])
    survivors = [
        u for u in spec["initial_ids"] if u != leader and u not in faulted
    ]
    for index, uav in enumerate(survivors):
        colors[uav] = CORE_COLORS[index % len(CORE_COLORS)]
    for index, uav in enumerate(spec["faulted_ids"]):
        colors[uav] = FAULT_COLORS[index % len(FAULT_COLORS)]
    for index, uav in enumerate(spec["spare_ids"]):
        colors[uav] = SPARE_COLORS[index % len(SPARE_COLORS)]
    for index, uav in enumerate(spec["b_strike_ids"]):
        colors[uav] = B_COLORS[index % len(B_COLORS)]
    colors[spec["b_failed_id"]] = "#d64545"
    colors[spec["b_reserve_id"]] = "#e0b000"
    for index, uav in enumerate(spec["all_ids"]):
        colors.setdefault(uav, CORE_COLORS[index % len(CORE_COLORS)])
    label_dz = {
        uav: 0.5 + (index % 3) * 0.65
        for index, uav in enumerate(spec["all_ids"])
    }
    return colors, label_dz


def render(run: Path, output: Path, fps: int = 12, speed: float = 4.0) -> None:
    run = Path(run)
    spec = json.loads((run / "mission_spec.json").read_text(encoding="utf-8"))
    fleet = load_fleet(run, spec)
    shift_y = float(spec["calibration"]["shift_y"])
    # 优先用 mission_spec 的实际地面位置（含 UAV13->UAV16 这类槽位替换）。
    if spec.get("ground_positions"):
        grid = {
            "UAV%d" % int(task_id[1:]): (item["x"], item["y"])
            for task_id, item in spec["ground_positions"].items()
        }
    else:
        grid = grid_positions(fleet, shift_y)
    colors, label_dz = assign_colors(spec)
    ids = [uav for uav in spec["all_ids"] if uav in colors]

    rows = [
        json.loads(line)
        for line in (run / "telemetry.jsonl").read_text().splitlines()
        if line
    ]
    raw_count = len(rows)
    rows = [row for row in rows if row["phase"] != "calibration"]
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    driver_rows = [
        json.loads(line)
        for line in (run / "driver.jsonl").read_text().splitlines()
        if line
    ]
    results = {
        row["phase"]: row["outcome"]
        for row in driver_rows
        if row.get("event") == "phase_result"
    }
    expected = [key for key, _ in PHASES if key not in {"takeoff", "complete"}]
    if summary["status"] == "SUCCESS" and (
        set(results) != set(expected)
        or any(value != "SUCCESS" for value in results.values())
    ):
        raise ValueError("summary 与完整阶段结果不一致，不能制作成功视频")
    if len(rows) < 2:
        raise ValueError("实测轨迹不足，不能制作视频")

    times = np.array([row["time"] for row in rows])
    paths = {uav: np.array([row["poses"][uav] for row in rows]) for uav in ids}
    t0 = times[0]
    duration = times[-1] - t0
    frame_times = list(np.arange(times[0], times[-1], speed / fps)) + [
        times[-1]
    ] * int(fps * 2)
    min_distance = min(
        float(np.linalg.norm(paths[a] - paths[b], axis=1).min())
        for index, a in enumerate(ids)
        for b in ids[index + 1:]
    )

    b_targets = spec["b_targets"]
    fault_exits = spec["expected_targets"]["fault-exit"]
    target_points = [(item["x"], item["y"]) for item in b_targets.values()]
    all_x = [p[0] for p in grid.values()] + [p[0] for p in target_points] + [
        item["x"] for item in fault_exits.values()
    ]
    all_y = [p[1] for p in grid.values()] + [p[1] for p in target_points] + [
        item["y"] for item in fault_exits.values()
    ]
    # 以实测轨迹的包络为准，避免覆盖段飞出坐标轴。
    for uav in ids:
        all_x.extend([float(paths[uav][:, 0].min()), float(paths[uav][:, 0].max())])
        all_y.extend([float(paths[uav][:, 1].min()), float(paths[uav][:, 1].max())])
    xlim = (min(all_x) - 8, max(all_x) + 8)
    ylim = (min(all_y) - 8, max(all_y) + 10)

    figure = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor="#f5f7fb")
    ax = figure.add_axes([0.0, 0.12, 0.74, 0.77], projection="3d")
    ax.set_facecolor("#f5f7fb")
    ax.view_init(elev=24, azim=-62)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(0, 18)
    ax.set_xlabel("右 / x (m)", labelpad=9)
    ax.set_ylabel("前 / y (m)", labelpad=9)
    ax.set_zlabel("高度 / z (m)", labelpad=4)
    ax.grid(True, alpha=0.2)
    ax.tick_params(labelsize=8)

    plane_x0, plane_x1 = xlim[0] + 1, xlim[1] - 1
    plane_y0, plane_y1 = ylim[0] + 1, ylim[1] - 1
    for z, color, label in [
        (5, "#b5d7be", "5m 打击悬停层"),
        (8, "#d8a278", "8m 退出 / B 转场层"),
        (12, "#9cb9d9", "12m 任务层"),
        (15, "#b7accc", "15m 转场层"),
    ]:
        ax.plot(
            [plane_x0, plane_x1, plane_x1, plane_x0, plane_x0],
            [plane_y0, plane_y0, plane_y1, plane_y1, plane_y0],
            [z] * 5,
            color=color,
            alpha=0.4,
            linewidth=0.6,
        )
        ax.text(plane_x0, plane_y1, z + 0.2, label, fontsize=7, color="#727c8e")

    # 现场 16 架地面摆位（按 fleet.yaml 网格 + 标定平移）。
    for number in range(1, 17):
        uav = "UAV%d" % number
        if uav not in grid:
            continue
        x, y = grid[uav]
        ax.scatter([x], [y], [0], s=13, color="#b9c1cf", alpha=0.7)
        ax.text(x, y, 0.3, str(number), fontsize=5.5, color="#8b95a6")

    for index, (task_id, item) in enumerate(b_targets.items(), start=1):
        ax.scatter([item["x"]], [item["y"]], [0], marker="*", s=150,
                   color="#519d61", alpha=0.85)
        ax.text(item["x"], item["y"], 0.5, "B目标%d" % index,
                fontsize=7, color="#427c4a")
        ax.plot([item["x"], item["x"]], [item["y"], item["y"]], [0, 5],
                color="#519d61", linestyle="--", alpha=0.5)
    for task_id, item in fault_exits.items():
        ax.scatter([item["x"]], [item["y"]], [item["z"]], marker="x", s=45,
                   color="#c77a2e", alpha=0.9)
        ax.text(item["x"], item["y"], item["z"] + 0.5, "退出 " + task_id,
                fontsize=6, color="#a45f20")
    for task_id, item in spec["expected_targets"].get("prepare-spares", {}).items():
        ax.scatter([item["x"]], [item["y"]], [item["z"]], marker="D", s=35,
                   color="#c9a227", alpha=0.9)
        ax.text(item["x"], item["y"], item["z"] + 0.4, "备用 " + task_id,
                fontsize=5.5, color="#8f7a2a")
    failed_hover = spec.get("b_failed_hover")
    if failed_hover:
        ax.scatter([failed_hover["x"]], [failed_hover["y"]], [failed_hover["z"]],
                   marker="x", s=55, color="#d64545", alpha=0.95)
        ax.text(failed_hover["x"], failed_hover["y"], failed_hover["z"] + 0.6,
                "失败悬停", fontsize=6, color="#a52a2a")

    figure.text(0.04, 0.955, "14–16 架 A/B 联合任务 · 三维实测轨迹",
                fontsize=19, weight="bold", color="#20304b")
    figure.text(
        0.04,
        0.91,
        "PX4 + Gazebo SITL  |  初始 %d（3×3）+ 备用 %d + B 组 %d"
        % (
            len(spec["initial_ids"]),
            len(spec["spare_ids"]),
            len(spec["b_ids"]),
        ),
        fontsize=10,
        color="#65738c",
    )
    figure.text(0.76, 0.86, "任务进度", fontsize=13, weight="bold", color="#20304b")
    phase_texts = [
        figure.text(0.76, 0.815 - index * 0.041, "○  " + text,
                    fontsize=8.5, color="#929bac")
        for index, (_, text) in enumerate(PHASES)
    ]
    clock = figure.text(0.04, 0.055, "", fontsize=12, color="#324867")
    note = figure.text(0.76, 0.20, "", fontsize=9, color="#5b6980", va="top")
    figure.text(
        0.04,
        0.019,
        "画面由本轮 ROS 实测位置重建；B 组为 GroupA 内模拟打击，"
        "最终降落由人工接管，不在任务中。",
        fontsize=8,
        color="#7d8798",
    )
    progress, = figure.add_axes([0.04, 0.095, 0.91, 0.007]).plot(
        [0, 0], [0, 0], color="#2775dc", linewidth=5
    )
    progress.axes.set_xlim(0, 1)
    progress.axes.set_ylim(-1, 1)
    progress.axes.axis("off")

    drone = {}
    for uav in ids:
        color = colors[uav]
        arms = [ax.plot([], [], [], color=color, linewidth=1.6)[0] for _ in range(2)]
        trail, = ax.plot([], [], [], color=color, linewidth=1.0, alpha=0.5)
        rotors, = ax.plot([], [], [], "o", color=color, markersize=2.4)
        stem, = ax.plot([], [], [], color=color, linewidth=0.5, alpha=0.22)
        text = ax.text(0, 0, 0, "UAV" + str(int(uav[1:])), fontsize=6.5,
                       color=color, weight="bold")
        drone[uav] = (arms, trail, rotors, stem, text)
    links, = ax.plot([], [], [], color="#3187cc", alpha=0.30, linewidth=0.9)

    writer = FFMpegWriter(
        fps=fps,
        codec="libx264",
        bitrate=3500,
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
        metadata={"title": "Full fleet A/B joint mission - measured PX4 SITL"},
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    core_ids = list(spec["core_ids"])
    initial_ids = list(spec["initial_ids"])
    with writer.saving(figure, str(output), dpi=100):
        for frame, stamp in enumerate(frame_times):
            index = min(max(bisect.bisect_right(times, stamp) - 1, 0), len(rows) - 1)
            after = min(index + 1, len(rows) - 1)
            fraction = (
                0
                if index == after
                else (stamp - times[index]) / (times[after] - times[index])
            )
            poses = {
                uav: paths[uav][index] * (1 - fraction) + paths[uav][after] * fraction
                for uav in ids
            }
            phase = rows[index]["phase"]
            number = next(
                (i for i, (key, _) in enumerate(PHASES) if key == phase), -1
            )
            for i, text in enumerate(phase_texts):
                marker = "●" if i == number else "✓" if i < number else "○"
                text.set_text(marker + "  " + PHASES[i][1])
                text.set_color(
                    "#2775dc" if i == number else "#3c7b58" if i < number else "#929bac"
                )
            for uav, (arms, trail, rotors, stem, text) in drone.items():
                x, y, z = poses[uav]
                for line, dx, dy in [(arms[0], 0.6, 0.6), (arms[1], 0.6, -0.6)]:
                    line.set_data([x - dx, x + dx], [y - dy, y + dy])
                    line.set_3d_properties([z, z])
                rotors.set_data([x - .6, x + .6, x - .6, x + .6],
                                [y - .6, y + .6, y + .6, y - .6])
                rotors.set_3d_properties([z] * 4)
                stem.set_data([x, x], [y, y])
                stem.set_3d_properties([0, z])
                trail.set_data(paths[uav][: index + 1:2, 0],
                               paths[uav][: index + 1:2, 1])
                trail.set_3d_properties(paths[uav][: index + 1:2, 2])
                text.set_position((x, y))
                text.set_3d_properties(z + label_dz[uav], zdir=None)
            if phase == "coverage1":
                link_ids = initial_ids
            elif phase in {"coverage2", "return_a"}:
                link_ids = core_ids
            else:
                link_ids = []
            if link_ids:
                points = [poses[uav] for uav in link_ids]
                links.set_data([p[0] for p in points], [p[1] for p in points])
                links.set_3d_properties([p[2] for p in points])
            else:
                links.set_data([], [])
                links.set_3d_properties([])
            elapsed = min(stamp - t0, duration)
            clock.set_text(
                "任务时间 %05.1fs / %.1fs     |     %.0f× 播放     |     最小采样间距 %.2fm"
                % (elapsed, duration, speed, min_distance)
            )
            progress.set_data([0, elapsed / duration], [0, 0])
            if phase == "complete":
                note.set_text(
                    "本轮结果：%s\nA 3×3 返回约 12m\nB 3 架目标上方 5m（侦查区内）\n失败机目标附近 5m / 第4架补打"
                    % summary["status"]
                )
                note.set_color("#32815a")
            elif phase == "failed":
                note.set_text("本轮失败 / 已停止推进\n请查本轮日志")
                note.set_color("#b54440")
            else:
                note.set_text(
                    "蓝：领机；青/蓝：初始编队\n橙：损毁边缘；黄：第4架补打\n红：失败悬停；绿/紫：B 打击"
                )
            if frame == 0:
                figure.savefig(str(output.with_suffix(".png")), dpi=100)
            writer.grab_frame()
            if frame % 120 == 0:
                print("render %d/%d" % (frame, len(frame_times)), flush=True)
        figure.savefig(str(output.with_name(output.stem + "_end.png")), dpi=100)
    plt.close(figure)

    meta = {
        "source": str(run),
        "status": summary["status"],
        "simulation": summary["simulation_type"],
        "source_duration_s": duration,
        "video_duration_s": len(frame_times) / fps,
        "speed": speed,
        "fps": fps,
        "frames": len(frame_times),
        "min_distance_m": min_distance,
        "excluded_precalibration_samples": raw_count - len(rows),
        "phases": list(dict.fromkeys(row["phase"] for row in rows)),
        "phase_results": results,
        "available": spec["available_uavs"],
        "initial": spec["initial_ids"],
        "core": spec["core_ids"],
        "spares": spec["spare_ids"],
        "faulted": spec["faulted_ids"],
        "b_ids": spec["b_ids"],
        "leader": spec["leader_id"],
    }
    output.with_suffix(".json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(meta, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--fps", type=int, default=12)
    parser.add_argument("--speed", type=float, default=4.0)
    args = parser.parse_args()
    render(args.run, args.out, args.fps, args.speed)
