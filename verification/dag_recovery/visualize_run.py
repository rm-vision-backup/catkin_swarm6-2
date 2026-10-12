#!/usr/bin/env python3
"""把 dag_recovery 运行记录渲染成三维视频：飞行轨迹 + 通信拓扑（8 m 水平距离图）。

用法（建议用 conda Python，自带较新 matplotlib）：
  ~/.conda/envs/gcbfplus/bin/python3 verification/dag_recovery/visualize_run.py \
      /tmp/dag10_sitl_run_9/experiment --out /tmp/dag10_run9.mp4

数据来源（全部在 run 的 experiment 目录）：
  events.jsonl          5 Hz telemetry：poses_task（任务系 x=右/y=前/z=上）、components、
                        fault_removed；command_sent/command_completed 事件
  simulation_config.json  comm_radius_m / 高度层 / 场地边界
  summary.json           最终判据（画在片尾）

拓扑口径与 core.py 一致：边 = 水平距离 ≤ comm_radius；fault_removed 后故障机退出逻辑图。
"""
from __future__ import annotations

import argparse
import bisect
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.family"] = "sans-serif"
matplotlib.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "WenQuanYi Zen Hei",
                                          "AR PL UMing CN", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
from matplotlib.animation import FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (注册 3d 投影)

FAULT_COLOR = "#d62728"
ALIVE_COLOR = "#1f77b4"
EDGE_IDLE = "#9aa0a6"
EDGE_OK = "#2e9e4f"

TRAIL_FRAMES = 30
DEFAULT_FPS = 12.0


def load_run(run_dir: Path):
    events = [json.loads(line) for line in
              (run_dir / "events.jsonl").read_text(encoding="utf-8").splitlines() if line.strip()]
    config = json.loads((run_dir / "simulation_config.json").read_text(encoding="utf-8"))
    summary_path = run_dir / "summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path.exists() else {}
    telemetry = sorted((row for row in events if row.get("type") == "telemetry"),
                       key=lambda row: row["wall_time"])
    if not telemetry:
        raise SystemExit("events.jsonl 中没有 telemetry 事件")
    commands = sorted(((row["wall_time"], row.get("command", ""))
                       for row in events if row.get("type") == "command_sent"))
    fault_times = [row["wall_time"] for row in events if row.get("type") == "fault_removed"]
    return telemetry, config, summary, commands, (fault_times[0] if fault_times else None)


def sample_at(telemetry, times, frame_time):
    """线性插值出 frame_time 时刻的全队位姿与故障状态。"""
    index = bisect.bisect_left(times, frame_time)
    if index <= 0:
        row = telemetry[0]
        return row["poses_task"], row["fault_removed"], telemetry[0]["wall_time"]
    if index >= len(telemetry):
        row = telemetry[-1]
        return row["poses_task"], row["fault_removed"], telemetry[-1]["wall_time"]
    before, after = telemetry[index - 1], telemetry[index]
    span = after["wall_time"] - before["wall_time"]
    ratio = 0.0 if span <= 0 else (frame_time - before["wall_time"]) / span
    poses = {}
    for uav, point in after["poses_task"].items():
        origin = before["poses_task"].get(uav, point)
        poses[uav] = [origin[axis] + (point[axis] - origin[axis]) * ratio for axis in range(3)]
    return poses, after["fault_removed"], after["wall_time"]


def graph_edges(poses, uav_ids, radius, fault_ids, fault_removed):
    """水平距离 ≤ radius 的边；故障退出后故障机不参与。"""
    active = [uav for uav in uav_ids if not (fault_removed and uav in fault_ids)]
    edges = []
    for index, first in enumerate(active):
        for second in active[index + 1:]:
            dx = poses[first][0] - poses[second][0]
            dy = poses[first][1] - poses[second][1]
            if (dx * dx + dy * dy) ** 0.5 <= radius:
                edges.append((first, second))
    components = 0
    seen = set()
    adjacency = {uav: set() for uav in active}
    for first, second in edges:
        adjacency[first].add(second)
        adjacency[second].add(first)
    for uav in active:
        if uav in seen:
            continue
        components += 1
        stack = [uav]
        while stack:
            current = stack.pop()
            if current in seen:
                continue
            seen.add(current)
            stack.extend(adjacency[current] - seen)
    return edges, components


def phase_text(frame_time, t0, fault_time, connected_once, summary):
    if fault_time is not None and frame_time >= fault_time:
        if connected_once:
            return "重组完成：单一连通组（保持中）"
        return "故障退出 → 幸存机重组机动"
    elapsed = frame_time - t0
    return "编队就位（爬升 15 m）" if elapsed < 45 else "巡航保持"


def render(run_dir: Path, out_path: Path, fps: float, stride: int, radius: float | None):
    telemetry, config, summary, commands, fault_time = load_run(run_dir)
    uav_ids = list(config["layout_task_xy"])
    fault_ids = list(summary.get("fault_ids") or [])
    radius = float(config.get("comm_radius_m", 8.0)) if radius is None else radius
    bounds = config.get("bounds_task_xy", [[-30, -15], [30, 15]])
    normal_h = float(config.get("normal_height_m", 15.0))
    fault_h = float(config.get("fault_height_m", 8.0))

    times = [row["wall_time"] for row in telemetry]
    t0, t_end = times[0], times[-1]
    frame_times = list(times[::stride])
    if frame_times[-1] != t_end:
        frame_times.append(t_end)

    figure = plt.figure(figsize=(13, 8), dpi=110)
    axes = figure.add_subplot(111, projection="3d")
    hud = figure.text(0.013, 0.965, "", va="top", ha="left", fontsize=11)
    verdict = figure.text(0.99, 0.02, "", va="bottom", ha="right", fontsize=13, weight="bold",
                          color="#2e9e4f")

    def paint(frame_time, poses, fault_removed, connected_once):
        axes.clear()
        axes.set_xlim(bounds[0][0], bounds[1][0])
        axes.set_ylim(bounds[0][1], bounds[1][1])
        axes.set_zlim(0.0, normal_h + 4.0)
        axes.set_xlabel("右 (m)")
        axes.set_ylabel("前 (m)")
        axes.set_zlabel("上 (m)")
        axes.set_title("DAGL 十机通信拓扑恢复 · 10-UAV SITL", pad=6)
        axes.grid(True, alpha=0.25)
        # 高度层参考面
        for height, color, label in ((normal_h, "#c8c8c8", "15 m 巡航层"),
                                     (fault_h, "#e8c8c8", "8 m 故障隔离层")):
            xs = [bounds[0][0], bounds[1][0], bounds[1][0], bounds[0][0], bounds[0][0]]
            ys = [bounds[0][1], bounds[0][1], bounds[1][1], bounds[1][1], bounds[0][1]]
            axes.plot(xs, ys, color=color, linewidth=0.8, alpha=0.6, zorder=1)
            axes.text(bounds[1][0], bounds[1][1], height, label, fontsize=8, color="#888888")
        edges, components = graph_edges(poses, uav_ids, radius, fault_ids, fault_removed)
        edge_color = EDGE_OK if components == 1 else EDGE_IDLE
        for first, second in edges:
            axes.plot([poses[first][0], poses[second][0]],
                      [poses[first][1], poses[second][1]],
                      [poses[first][2], poses[second][2]],
                      color=edge_color, linewidth=2.2 if components == 1 else 1.0,
                      alpha=0.85 if components == 1 else 0.55, zorder=2)
        for uav in uav_ids:
            x, y, z = poses[uav]
            faulted = fault_removed and uav in fault_ids
            color = FAULT_COLOR if faulted else ALIVE_COLOR
            axes.scatter([x], [y], [z], s=90, color=color, depthshade=False, zorder=4)
            axes.text(x, y, z + 0.9, uav, fontsize=9, color=color, zorder=5)
            axes.plot([x, x], [y, y], [0, z], color=color, linewidth=0.7, alpha=0.35, zorder=1)
        # 轨迹尾迹
        history = [entry for entry in trail if entry[0] >= frame_time - 6.0]
        for uav in uav_ids:
            points = [entry[1][uav] for entry in history if uav in entry[1]]
            if len(points) > 1:
                faulted = fault_removed and uav in fault_ids
                axes.plot([p[0] for p in points], [p[1] for p in points], [p[2] for p in points],
                          color=FAULT_COLOR if faulted else ALIVE_COLOR, linewidth=0.9, alpha=0.5)
        min_sep = float("inf")
        for index, first in enumerate(uav_ids):
            for second in uav_ids[index + 1:]:
                a, b = poses[first], poses[second]
                distance = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5
                min_sep = min(min_sep, distance)
        state = phase_text(frame_time, t0, fault_time, connected_once, summary)
        hud.set_text("t=%5.1fs  阶段：%s\n连通分量：%d  最小三维间距：%.2f m  故障机：%s"
                     % (frame_time - t0, state, components, min_sep,
                        ",".join(fault_ids) if fault_removed else "—"))
        status = summary.get("status", "")
        verdict.set_text("✓ " + status if status.startswith("SITL_") else status)
        verdict.set_color("#2e9e4f" if status.startswith("SITL_") else "#d62728")

    writer = FFMpegWriter(fps=fps, bitrate=4000, metadata={"title": "dag_recovery SITL"})
    connected_once = False
    with writer.saving(figure, str(out_path), dpi=110):
        trail = []
        for frame_time in frame_times:
            poses, fault_removed, stamp = sample_at(telemetry, times, frame_time)
            trail.append((stamp, poses))
            if len(trail) > TRAIL_FRAMES:
                trail.pop(0)
            if fault_time is not None and frame_time > fault_time:
                _, components = graph_edges(poses, uav_ids, radius, fault_ids, fault_removed)
                connected_once = connected_once or components == 1
            paint(frame_time, poses, fault_removed, connected_once)
            writer.grab_frame()
    plt.close(figure)
    return out_path, len(frame_times), summary.get("status", "")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path, help="run 的 experiment 目录（含 events.jsonl）")
    parser.add_argument("--out", type=Path, default=None, help="输出 mp4 路径")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS)
    parser.add_argument("--stride", type=int, default=2, help="每 N 个 5Hz 采样取一帧")
    parser.add_argument("--radius", type=float, default=None, help="覆盖通信半径（默认取 config）")
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    out_path = (args.out or (run_dir.parent / "dag_recovery_view.mp4")).resolve()
    path, frames, status = render(run_dir, out_path, args.fps, max(1, args.stride), args.radius)
    print("已输出 %s（%d 帧, ~%.0f s, 判据 %s）" % (path, frames, frames / args.fps, status or "N/A"))


if __name__ == "__main__":
    main()
