"""Pure, fail-closed geometry and configuration for the DAGL flight experiment."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Mapping, Sequence, Tuple

Point = Tuple[float, float]
Pose = Tuple[float, float, float]


def _finite(value: object) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("non-finite number")
    return number


def _pair(value: Sequence[float]) -> Point:
    if len(value) != 2:
        raise ValueError("expected two coordinates")
    return _finite(value[0]), _finite(value[1])


def _pose(value: Sequence[float]) -> Pose:
    if len(value) != 3:
        raise ValueError("expected three coordinates")
    return _finite(value[0]), _finite(value[1]), _finite(value[2])


def distance(first: Point, second: Point) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def components(positions: Mapping[str, Point], radius: float) -> list[list[str]]:
    remaining = set(positions)
    clusters = []
    while remaining:
        root = min(remaining)
        remaining.remove(root)
        stack = [root]
        cluster = []
        while stack:
            current = stack.pop()
            cluster.append(current)
            neighbors = [other for other in sorted(remaining)
                         if distance(positions[current], positions[other]) <= radius]
            stack.extend(neighbors)
            remaining.difference_update(neighbors)
        clusters.append(sorted(cluster))
    return clusters


def _point_segment_distance(point: Point, first: Point, second: Point) -> float:
    delta = (second[0] - first[0], second[1] - first[1])
    length_squared = delta[0] ** 2 + delta[1] ** 2
    if length_squared == 0:
        return distance(point, first)
    fraction = max(0.0, min(1.0, ((point[0] - first[0]) * delta[0]
                                    + (point[1] - first[1]) * delta[1]) / length_squared))
    return distance(point, (first[0] + fraction * delta[0],
                            first[1] + fraction * delta[1]))


def _cross(first: Point, second: Point, third: Point) -> float:
    return ((second[0] - first[0]) * (third[1] - first[1])
            - (second[1] - first[1]) * (third[0] - first[0]))


def segment_distance(a: Point, b: Point, c: Point, d: Point) -> float:
    ab_c, ab_d = _cross(a, b, c), _cross(a, b, d)
    cd_a, cd_b = _cross(c, d, a), _cross(c, d, b)
    if (ab_c * ab_d <= 0 and cd_a * cd_b <= 0
            and max(min(a[0], b[0]), min(c[0], d[0]))
            <= min(max(a[0], b[0]), max(c[0], d[0]))
            and max(min(a[1], b[1]), min(c[1], d[1]))
            <= min(max(a[1], b[1]), max(c[1], d[1]))):
        return 0.0
    return min(_point_segment_distance(a, c, d), _point_segment_distance(b, c, d),
               _point_segment_distance(c, a, b), _point_segment_distance(d, a, b))


def enu_to_task(east: float, north: float, heading: float) -> Point:
    return (east * math.sin(heading) - north * math.cos(heading),
            east * math.cos(heading) + north * math.sin(heading))


def task_to_enu(right: float, forward: float, heading: float) -> Point:
    return (right * math.sin(heading) + forward * math.cos(heading),
            -right * math.cos(heading) + forward * math.sin(heading))


@dataclass(frozen=True)
class Scenario:
    uav_ids: Tuple[str, ...]
    fault_id: str
    layout: Dict[str, Point]
    lower: Point
    upper: Point
    radius: float
    edge_margin: float
    border_margin: float
    min_separation: float
    layout_tolerance: float
    normal_height: float
    fault_height: float
    altitude_tolerance: float
    seed: int
    approved: bool

    @classmethod
    def load(cls, path: Path) -> "Scenario":
        raw = json.loads(path.read_text(encoding="utf-8"))
        if "max_displacement_m" in raw:
            raise ValueError("remove legacy max_displacement_m and re-approve the scenario")
        identifiers = tuple(str(item) for item in raw["uav_ids"])
        layout = {str(key): _pair(value) for key, value in raw["layout_task_xy"].items()}
        bounds = raw["bounds_task_xy"]
        scenario = cls(
            uav_ids=identifiers,
            fault_id=str(raw["fault_id"]),
            layout=layout,
            lower=_pair(bounds[0]),
            upper=_pair(bounds[1]),
            radius=_finite(raw["comm_radius_m"]),
            edge_margin=_finite(raw["edge_margin_m"]),
            border_margin=_finite(raw["border_margin_m"]),
            min_separation=_finite(raw["min_separation_m"]),
            layout_tolerance=_finite(raw["layout_tolerance_m"]),
            normal_height=_finite(raw["normal_height_m"]),
            fault_height=_finite(raw["fault_height_m"]),
            altitude_tolerance=_finite(raw["altitude_tolerance_m"]),
            seed=int(raw["seed"]),
            approved=raw["approved"] is True,
        )
        if len(bounds) != 2 or len(identifiers) not in (4, 11, 15):
            raise ValueError("scenario must have 4, 11 or 15 UAVs and two bounds")
        if len(set(identifiers)) != len(identifiers) or set(layout) != set(identifiers):
            raise ValueError("layout and unique UAV identifiers must match")
        if scenario.fault_id not in layout or scenario.seed < 0:
            raise ValueError("invalid fault identity or seed")
        if any(value <= 0 for value in (scenario.radius, scenario.edge_margin,
                                        scenario.min_separation,
                                        scenario.layout_tolerance, scenario.altitude_tolerance)):
            raise ValueError("geometry limits must be positive")
        if (scenario.border_margin < 0 or scenario.radius <= scenario.edge_margin
                or scenario.normal_height - scenario.fault_height < scenario.min_separation
                or any(scenario.lower[index] + 2 * scenario.border_margin >= scenario.upper[index]
                       for index in range(2))):
            raise ValueError("unsafe bounds, radius or altitude separation")
        if (scenario.radius != 8.0 or scenario.edge_margin < 1.0
                or scenario.border_margin < 3.0 or scenario.min_separation < 4.0
                or scenario.layout_tolerance > 0.75
                or scenario.normal_height != 15.0 or scenario.fault_height != 8.0
                or scenario.altitude_tolerance > 0.7):
            raise ValueError("experiment safety constants cannot be relaxed")
        scenario.validate_layout(scenario.layout)
        return scenario

    def within_bounds(self, point: Point) -> bool:
        return all(self.lower[index] + self.border_margin <= point[index]
                   <= self.upper[index] - self.border_margin for index in range(2))

    def validate_layout(self, positions: Mapping[str, Point]) -> None:
        if set(positions) != set(self.uav_ids):
            raise ValueError("missing or unexpected UAV in measured layout")
        if not all(self.within_bounds(_pair(value)) for value in positions.values()):
            raise ValueError("initial position outside approved inner bounds")
        for index, first in enumerate(self.uav_ids):
            for second in self.uav_ids[index + 1:]:
                if distance(positions[first], positions[second]) < self.min_separation:
                    raise ValueError("initial aircraft too close")
        if len(components(positions, self.radius - self.edge_margin)) != 1:
            raise ValueError("initial graph is not robustly connected")
        survivors = {key: positions[key] for key in self.uav_ids if key != self.fault_id}
        if len(components(survivors, self.radius + self.edge_margin)) != 2:
            raise ValueError("removing bridge does not robustly split graph in two")

    def validate_fixture(self, positions: Mapping[str, Point]) -> None:
        self.validate_layout(positions)
        for uav in self.uav_ids:
            if distance(positions[uav], self.layout[uav]) > self.layout_tolerance:
                raise ValueError("measured site layout differs from approved fixture: " + uav)

    def validate_targets(self, starts: Mapping[str, Point], targets: Mapping[str, Point],
                         fault_position: Point) -> None:
        survivors = tuple(uav for uav in self.uav_ids if uav != self.fault_id)
        if set(targets) != set(survivors) or set(starts) != set(survivors):
            raise ValueError("targets and start points must match all survivors")
        for uav in survivors:
            start, target = _pair(starts[uav]), _pair(targets[uav])
            if not self.within_bounds(target):
                raise ValueError("DAGL target violates boundary")
            clearance = math.hypot(_point_segment_distance(fault_position, start, target),
                                   self.normal_height - self.fault_height)
            if clearance < self.min_separation:
                raise ValueError("recovery path too close to lower fault aircraft")
        if len(components(targets, self.radius - self.edge_margin)) != 1:
            raise ValueError("DAGL targets do not robustly reconnect the graph")
        for index, first in enumerate(survivors):
            for second in survivors[index + 1:]:
                if segment_distance(starts[first], targets[first], starts[second],
                                    targets[second]) < self.min_separation:
                    raise ValueError("swept paths too close, including stationary aircraft")


def normalize_positions(poses: Mapping[str, Sequence[float]], heading: float) -> Dict[str, Pose]:
    angle = _finite(heading)
    result = {}
    for uav, raw in poses.items():
        east, north, up = _pose(raw)
        right, forward = enu_to_task(east, north, angle)
        result[uav] = (right, forward, up)
    return result
