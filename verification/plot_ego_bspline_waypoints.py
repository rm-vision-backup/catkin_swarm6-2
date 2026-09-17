#!/usr/bin/env python3
"""Plot and check the waypoint expansion used by ego_planner_driver.

The route waypoints use z=15 m, with a 3 m vertical approach waypoint above
the initial position and a 3 m vertical descent waypoint below the final route
point.  The resulting sequence is uniformly densified so every
adjacent pair is at most 2 m apart.  ego_planner_driver then prepends the
current position once before parameterizing a cubic uniform B-spline.  This script ports the
parameterization and de Boor evaluation used by the C++ driver so runtime
plan batches can be inspected without ROS.

Important: the parameterization interpolates the supplied points, but the
current C++ LBFGS stage only hard-pins the first and last three control-point
columns.  Consequently, intermediate waypoints are not hard constraints after
optimization.  The report therefore checks the interpolation curve and labels
it separately from any sampled/optimized trajectory supplied by a caller.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any, Iterable

import matplotlib

if not os.environ.get("DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers the 3D projection)


DEFAULT_PLANS = (
    Path(__file__).resolve().parents[1]
    / "references/runtime/examples/joint_mission/group_a/plans/plans.json"
)


def _pose(value: Any) -> np.ndarray:
    if isinstance(value, dict):
        return np.array([value["x"], value["y"], value["z"]], dtype=float)
    if len(value) < 3:
        raise ValueError("waypoint must contain at least x, y, z")
    return np.array(value[:3], dtype=float)


def load_route(path: Path, plan_id: str, robot_id: str) -> list[np.ndarray]:
    data = json.loads(path.read_text(encoding="utf-8"))
    plans = data.get("plans", [])
    for plan in plans:
        if plan.get("plan_id") != plan_id:
            continue
        assignment = plan.get("robot_assignments", {}).get(robot_id, {})
        route = assignment.get("payload", {}).get("waypoints")
        if not route:
            raise ValueError(f"{plan_id}/{robot_id} has no payload.waypoints")
        return [_pose(item) for item in route]
    raise ValueError(f"plan_id {plan_id!r} and robot_id {robot_id!r} not found in {path}")


def prepare_route(route: Iterable[np.ndarray], start: np.ndarray) -> list[np.ndarray]:
    """Set route altitude and add vertical approach/descent waypoints."""
    route = [point.copy() for point in route]
    route_at_altitude = [np.array([point[0], point[1], 15.0]) for point in route]
    approach = start + np.array([0.0, 0.0, 3.0])
    descent = route_at_altitude[-1] - np.array([0.0, 0.0, 3.0])
    return [approach, *route_at_altitude, descent]


def densify_waypoints(points: Iterable[np.ndarray], max_spacing: float) -> list[np.ndarray]:
    """Insert uniformly spaced points so every adjacent distance is bounded."""
    if max_spacing <= 0:
        raise ValueError("max_spacing must be positive")
    points = [point.copy() for point in points]
    if len(points) < 2:
        return points

    dense = [points[0]]
    for left, right in zip(points, points[1:]):
        distance = float(np.linalg.norm(right - left))
        segments = max(1, int(np.ceil(distance / max_spacing)))
        dense.extend(left + (right - left) * (index / segments)
                     for index in range(1, segments + 1))
    return dense


def subdivide_until_five(points: list[np.ndarray]) -> list[np.ndarray]:
    points = list(points)
    while len(points) < 5:
        refined: list[np.ndarray] = []
        for left, right in zip(points, points[1:]):
            refined.extend((left, (left + right) * 0.5))
        refined.append(points[-1])
        points = refined
    return points


def parameterize(points: list[np.ndarray], ts: float) -> np.ndarray:
    """Port UniformBspline::parameterizeToBspline for zero boundary derivatives."""
    if ts <= 0 or len(points) <= 3:
        raise ValueError("need ts > 0 and at least four points")
    points = np.asarray(points, dtype=float)
    count = len(points)
    matrix = np.zeros((count + 4, count + 2))
    matrix[:count, :].flat[0:0]  # keep shape/type explicit for readability
    for index in range(count):
        matrix[index, index:index + 3] = np.array([1.0, 4.0, 1.0]) / 6.0
    velocity = np.array([-1.0, 0.0, 1.0]) / (2.0 * ts)
    acceleration = np.array([1.0, -2.0, 1.0]) / (ts * ts)
    matrix[count, 0:3] = velocity
    matrix[count + 1, count - 1:count + 2] = velocity
    matrix[count + 2, 0:3] = acceleration
    matrix[count + 3, count - 1:count + 2] = acceleration
    rhs = np.vstack((points, np.zeros((4, 3))))
    return np.linalg.lstsq(matrix, rhs, rcond=None)[0].T


def knots(control_count: int, degree: int, ts: float) -> np.ndarray:
    m = control_count - 1 + degree + 1
    return np.array([(-degree + index) * ts if index <= degree else
                     (index - degree) * ts for index in range(m + 1)], dtype=float)


def de_boor(control: np.ndarray, knot: np.ndarray, degree: int, u: float) -> np.ndarray:
    # C++ uses u_(m - p_) as the upper endpoint; with N control columns this
    # is knot[N - 1], not knot[N].
    lower, upper = knot[degree], knot[control.shape[1] - 1]
    ub = min(max(u, lower), upper)
    span = degree
    while span + 1 < len(knot) and knot[span + 1] < ub:
        span += 1
    work = [control[:, span - degree + i].copy() for i in range(degree + 1)]
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            denominator = knot[index + span - level + 1] - knot[index + span - degree]
            alpha = 0.0 if denominator == 0 else (ub - knot[index + span - degree]) / denominator
            work[index] = (1.0 - alpha) * work[index - 1] + alpha * work[index]
    return work[degree]


def evaluate(control: np.ndarray, ts: float, samples: int = 2000) -> np.ndarray:
    degree = 3
    knot = knots(control.shape[1], degree, ts)
    duration = knot[control.shape[1] - 1] - knot[degree]
    times = np.linspace(0.0, duration, samples)
    return np.array([de_boor(control, knot, degree, time + knot[degree]) for time in times])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plans", type=Path, default=DEFAULT_PLANS)
    parser.add_argument("--plan-id", default="coverage-segment-1")
    parser.add_argument("--robot-id", default="A01")
    parser.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        default=(0.0, 0.0, 12.0), help="current position prepended once")
    parser.add_argument("--ts", type=float, default=0.4)
    parser.add_argument("--max-spacing", type=float, default=2.0,
                        help="maximum distance between adjacent route points [m]")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).with_name("ego_bspline_waypoints.png"))
    parser.add_argument("--tolerance", type=float, default=0.05)
    args = parser.parse_args()

    source_route = load_route(args.plans, args.plan_id, args.robot_id)
    start = np.asarray(args.start, dtype=float)
    key_route = prepare_route(source_route, start)
    planner_points = densify_waypoints([start, *key_route], args.max_spacing)
    sent = planner_points[1:]
    points = subdivide_until_five(planner_points)
    control = parameterize(points, args.ts)
    curve = evaluate(control, args.ts)

    print(f"plan={args.plan_id} robot={args.robot_id} source={args.plans}")
    print(f"source route points: {len(source_route)}")
    print(f"key route points: {len(key_route)} (z=15 m with approach/descent points)")
    print(f"published waypoint messages: {len(sent)} (densified, max spacing {args.max_spacing:.2f} m)")
    print(f"planner point_set: {len(planner_points)} (current position prepended once)")
    print("published sequence:", " -> ".join("(%.2f, %.2f, %.2f)" % tuple(p) for p in sent))
    adjacent_distances = [float(np.linalg.norm(right - left))
                          for left, right in zip(sent, sent[1:])]
    print(f"max adjacent published-waypoint distance: {max(adjacent_distances, default=0.0):.6f} m")
    print("interpolation-curve nearest distances:")
    distances = []
    for index, point in enumerate(planner_points):
        distance = float(np.min(np.linalg.norm(curve - point, axis=1)))
        distances.append(distance)
        print(f"  point_set[{index}] {point.tolist()} : {distance:.6f} m")
    max_distance = max(distances)
    print(f"max parameterized-curve distance: {max_distance:.6f} m")
    if max_distance <= args.tolerance:
        print("CONCLUSION: parameterized curve passes all supplied points within tolerance.")
    else:
        print("CONCLUSION: parameterized cubic B-spline does NOT pass all supplied points within tolerance.")
    print("CONCLUSION: the current optimized C++ trajectory also does not guarantee intermediate points;")
    print("           only the first/last sampled positions are explicitly anchored after optimization.")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(curve[:, 0], curve[:, 1], curve[:, 2], label="parameterized cubic B-spline")
    ax.scatter(*np.asarray(planner_points).T, c="tab:red", s=12, label="densified planner point_set")
    ax.scatter(*np.asarray(key_route).T, c="tab:green", marker="x", s=55, label="key route")
    for index, point in enumerate(key_route):
        ax.text(*point, str(index), fontsize=8)
    plot_points = np.vstack((curve, planner_points, key_route))
    bounds_min = plot_points.min(axis=0)
    bounds_max = plot_points.max(axis=0)
    center = (bounds_min + bounds_max) * 0.5
    half_range = float(np.max(bounds_max - bounds_min)) * 0.5
    half_range = max(half_range, 1.0) * 1.05
    ax.set_xlim(center[0] - half_range, center[0] + half_range)
    ax.set_ylim(center[1] - half_range, center[1] + half_range)
    ax.set_zlim(center[2] - half_range, center[2] + half_range)
    # Older Matplotlib releases do not expose Axes3D.set_box_aspect(); equal
    # numeric limits still preserve the same data-unit scale on all axes.
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title(f"{args.plan_id}/{args.robot_id}: waypoint expansion and B-spline")
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output, dpi=150)
    print(f"plot: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())