#!/usr/bin/env python3
"""Independently verify timed virtual connectivity from recorded 5 Hz positions."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from core import Scenario, components


def evaluate(scenario: Scenario, events: list[dict], stable_s: float = 10.0):
    if any(event["type"] in ("safety_abort", "run_failed", "manual_intervention")
           or event["type"] == "hold_result" and event.get("unconfirmed") for event in events):
        raise ValueError("unsafe or manually intervened trial cannot be reported as success")
    fault_events = [event for event in events if event["type"] == "fault_removed"]
    if len(fault_events) != 1:
        raise ValueError("expected exactly one confirmed fault removal")
    fault_time = fault_events[0]["monotonic_time"]
    telemetry = sorted((event for event in events if event["type"] == "telemetry"
                        and event["monotonic_time"] >= fault_time),
                       key=lambda event: event["monotonic_time"])
    ready_time = max((event["monotonic_time"] for event in events
                      if event["type"] == "command_completed"), default=float("inf"))
    first_connected = None
    previous_time = None
    recovered_at = None
    min_distance = float("inf")
    prior_stamps = None
    for event in telemetry:
        timestamp = event["monotonic_time"]
        positions = event["poses_task"]
        if set(positions) != set(scenario.uav_ids):
            raise ValueError("incomplete or unexpected replay roster")
        if (previous_time is not None and timestamp <= previous_time):
            raise ValueError("duplicate or unordered telemetry timestamp")
        previous_time = timestamp
        stamps = event.get("pose_stamps_ros", {})
        links = event.get("links", {})
        if (set(stamps) != set(scenario.uav_ids) or set(links) != set(scenario.uav_ids)
                or any(not math.isfinite(float(stamps[uav])) or float(stamps[uav]) <= 0
                       or not 0 <= float(links[uav]["source_pose_age_s"]) <= 0.6
                       for uav in scenario.uav_ids)):
            raise ValueError("unverified ROS pose timestamps in replay")
        measured = float("inf")
        for index, first in enumerate(scenario.uav_ids):
            for second in scenario.uav_ids[index + 1:]:
                first_pose, second_pose = positions[first], positions[second]
                if (len(first_pose) != 3 or len(second_pose) != 3
                        or not all(math.isfinite(float(value))
                                   for value in first_pose + second_pose)):
                    raise ValueError("non-finite or incomplete recorded 3D position")
                measured = min(measured, math.dist(first_pose, second_pose))
        if abs(float(event["min_separation_m"]) - measured) > 0.1:
            raise ValueError("replay separation disagrees with recorded positions")
        min_distance = min(min_distance, measured)
        if not math.isfinite(min_distance) or min_distance < scenario.min_separation:
            raise ValueError("unsafe or non-finite replay separation")
        survivors = {uav: positions[uav][:2] for uav in scenario.uav_ids
                     if uav != scenario.fault_id}
        connected = len(components(survivors, scenario.radius)) == 1
        new_positions = (prior_stamps is None or
                         all(float(stamps[uav]) > float(prior_stamps[uav])
                             for uav in scenario.uav_ids))
        if new_positions:
            prior_stamps = stamps
        if not connected or timestamp < ready_time or (
                first_connected is not None and timestamp - last_connected > 0.4):
            first_connected = None
            recovered_at = None
        if connected and timestamp >= ready_time and new_positions:
            if first_connected is None:
                first_connected = timestamp
            last_connected = timestamp
            if timestamp - first_connected >= stable_s and recovered_at is None:
                recovered_at = timestamp
    if not telemetry:
        raise ValueError("no telemetry after fault removal")
    return {"verified": recovered_at is not None,
            "recovery_duration_s": None if recovered_at is None else recovered_at - fault_time,
            "min_3d_separation_m": min_distance, "samples_after_fault": len(telemetry),
            "latest_sample_monotonic": previous_time}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--events", type=Path, required=True)
    args = parser.parse_args()
    events = [json.loads(line) for line in args.events.read_text(encoding="utf-8").splitlines()]
    print(json.dumps(evaluate(Scenario.load(args.scenario), events), indent=2))


if __name__ == "__main__":
    main()
