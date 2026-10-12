#!/usr/bin/env python3
"""Prepare isolated ten-aircraft Gazebo spawn and shared SITL bridge configuration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from xml.etree import ElementTree

import yaml

from core import task_to_enu
from sim_multi import SimCase

REPO_ROOT = Path(__file__).resolve().parents[2]


def prepare(scenario_path, identity_path, topology_path, output_dir):
    scenario = SimCase.load(scenario_path, 3)
    SimCase.load(scenario_path, 4)
    # 任务系与仿真世界系的对齐：实测任务坐标 = R(h_spawn − h_frozen)·布局，
    # h_frozen 是标定冻结航向（SITL 里 ≈ 世界磁偏角，实测 −0.048 rad）。场景可用
    # spawn_heading_rad 让 h_spawn = h_frozen，从而实测坐标与布局一致——大范围散布
    # （r > 20 m，旋转位移 0.048·r 超过 1 m 布局容差）必须对齐，否则 validate_fixture
    # 会把整片场地误判为"偏离批准布局"。
    spawn_heading = float(json.loads(scenario_path.read_text(encoding="utf-8"))
                          .get("spawn_heading_rad", 0.0))
    identities = yaml.safe_load(identity_path.read_text(encoding="utf-8"))["uavs"]
    mapped = {item["uav_id"]: item for item in identities}
    selected = []
    for index, uav in enumerate(scenario.uav_ids, 1):
        item = mapped.get(uav, {})
        if not item.get("enabled") or item.get("exec_target") != "UAV%d" % index:
            raise ValueError("SITL only accepts A01..A10 mapped to UAV1..UAV10")
        selected.append(item["exec_target"])
    topology = yaml.safe_load(topology_path.read_text(encoding="utf-8"))
    allowed = set(selected) | {"GCS_A"}
    topology["uavs"] = [node for node in topology["uavs"] if node["name"] in allowed]
    if set(node["name"] for node in topology["uavs"]) != allowed:
        raise ValueError("simulation bridge does not contain all selected nodes")
    if any(node["ip"] not in ("127.0.0.1", "localhost") for node in topology["uavs"]):
        raise ValueError("SITL bridge must never address real aircraft")
    topology["topology"] = {node: [peer for peer in topology["topology"][node]
                                    if peer in allowed] for node in allowed}
    topology["services"] = [item for item in topology["services"] if item["target"] in allowed]
    topics = [topic for topic in topology["topics"] if topic["name"] == "exec_state"]
    if topics and topics[0]["type"] != "std_msgs/String":
        raise ValueError("invalid simulation planner state topic")
    if not topics:
        topology["topics"].append({"name": "exec_state", "type": "std_msgs/String",
                                   "max_freq": 0})
    for target in selected:
        if not any(item["name"] == "/%s/uav_hold" % target
                   for item in topology["services"]):
            raise ValueError("missing SITL onboard HOLD service: " + target)
    root = ElementTree.Element("launch")
    for name, default in (("world", "$(find mavlink_sitl_gazebo)/worlds/empty.world"),
                          ("gui", "false")):
        ElementTree.SubElement(root, "arg", name=name, default=default)
    world = ElementTree.SubElement(root, "include", file="$(find gazebo_ros)/launch/empty_world.launch")
    for name, value in (("gui", "$(arg gui)"), ("world_name", "$(arg world)")):
        ElementTree.SubElement(world, "arg", name=name, value=value)
    for index, uav in enumerate(scenario.uav_ids):
        group = ElementTree.SubElement(root, "group", ns="iris_%d" % index)
        spawn = ElementTree.SubElement(
            group, "include", file="$(find px4)/launch/single_vehicle_spawn_xtd.launch")
        east, north = task_to_enu(*scenario.layout[uav], spawn_heading)
        values = {"x": east, "y": north, "z": 0.5, "R": 0, "P": 0, "Y": 0,
                  "vehicle": "iris", "sdf": "iris", "ID": index, "ID_in_group": index,
                  "mavlink_tcp_port": 4560 + index, "udp_gimbal_port": 13030 + index,
                  "interactive": "true"}
        for name, value in values.items():
            ElementTree.SubElement(spawn, "arg", name=name, value=str(value))
    calibration = {"calibration_uavs": ["UAV1", "UAV2", "UAV7", "UAV8"],
                   "all_uavs": selected, "sample_window_s": 8.0}
    formation = {"leader_id": scenario.uav_ids[0], "mission_anchor": [0.0, 0.0, 0.0],
                 "formations": {}}
    output_dir.mkdir(parents=True, exist_ok=False)
    for name, content in (("topology.yaml", topology), ("calibration.yaml", calibration),
                          ("formation.yaml", formation)):
        (output_dir / name).write_text(yaml.safe_dump(content, sort_keys=False), encoding="utf-8")
    (output_dir / "spawn.launch").write_text(ElementTree.tostring(root, encoding="unicode"),
                                               encoding="utf-8")
    manifest = {"simulation_only": True, "scenario_file": str(scenario_path.resolve()),
                "identity_file": str(identity_path.resolve()), "uav_ids": list(scenario.uav_ids),
                "exec_targets": selected, "topology_file": str((output_dir / "topology.yaml").resolve()),
                "calibration_file": str((output_dir / "calibration.yaml").resolve()),
                "formation_file": str((output_dir / "formation.yaml").resolve()),
                "spawn_file": str((output_dir / "spawn.launch").resolve())}
    (output_dir / "deployment.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, default=Path(__file__).with_name("scenarios")
                        / "10.sitl.json")
    parser.add_argument("--identity", type=Path, default=REPO_ROOT / "src/tcp_to_ros"
                        / "config/uav_identity_map.yaml")
    parser.add_argument("--topology", type=Path, default=REPO_ROOT / "src/swarm_topology_bridge"
                        / "config/topology_group_a_sim.yaml")
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.scenario, args.identity, args.topology, args.outdir), indent=2))


if __name__ == "__main__":
    main()
