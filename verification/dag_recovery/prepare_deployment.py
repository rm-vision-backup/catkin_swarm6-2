"""Generate experiment-only ROS configs without changing production topology."""

import argparse
import json
import re
from pathlib import Path

import yaml

from core import Scenario


def generate(scenario_path: Path, identity_path: Path, topology_path: Path,
             output_dir: Path) -> dict:
    scenario = Scenario.load(scenario_path)
    if any(re.fullmatch(r"A\d{2}", uav) is None for uav in scenario.uav_ids):
        raise ValueError("replace all unassigned template slots with verified Axx identities")
    identities = yaml.safe_load(identity_path.read_text(encoding="utf-8"))["uavs"]
    mapping = {item["uav_id"]: item for item in identities}
    selected = []
    for uav in scenario.uav_ids:
        identity = mapping.get(uav)
        if not identity or not identity.get("enabled") or not identity.get("exec_target"):
            raise ValueError("missing or disabled identity: " + uav)
        selected.append(identity["exec_target"])
    if len(set(selected)) != len(selected):
        raise ValueError("repeated onboard execution identity")
    topology = yaml.safe_load(topology_path.read_text(encoding="utf-8"))
    members = {item["name"]: item for item in topology["uavs"]}
    if not set(selected).issubset(members) or "GCS_A" not in members:
        raise ValueError("topology does not include all selected UAVs and GCS_A")
    for uav in scenario.uav_ids:
        identity = mapping[uav]
        if (identity.get("hostname") != identity["exec_target"]
                or identity.get("ip") != members[identity["exec_target"]]["ip"]):
            raise ValueError("identity hostname/IP differs from selected bridge mapping: " + uav)
    allowed = set(selected) | {"GCS_A"}
    topology["uavs"] = [item for item in topology["uavs"] if item["name"] in allowed]
    topology["topology"] = {node: [peer for peer in topology["topology"][node]
                                    if peer in allowed]
                            for node in allowed}
    topology["services"] = [item for item in topology["services"]
                            if item["target"] in allowed]
    if not all(any(item["name"] == "/%s/uav_hold" % target
                   for item in topology["services"]) for target in selected):
        raise ValueError("HOLD service missing for experiment airframe")
    topics = topology["topics"]
    state_topics = [item for item in topics if item["name"] == "exec_state"]
    if state_topics and state_topics[0]["type"] != "std_msgs/String":
        raise ValueError("exec_state topology topic has wrong type")
    if not state_topics:
        topics.append({"name": "exec_state", "type": "std_msgs/String", "max_freq": 0})
    left_count = scenario.uav_ids.index(scenario.fault_id)
    if left_count >= 2 and len(scenario.uav_ids) - left_count >= 3:
        anchors = [selected[0], selected[1], selected[left_count + 1], selected[left_count + 2]]
    else:
        anchors = selected
    calibration = {
        "calibration_uavs": anchors,
        "all_uavs": selected,
        "sample_window_s": 8.0,
    }
    formation = {"leader_id": scenario.uav_ids[0],
                 "mission_anchor": [0.0, 0.0, 0.0], "formations": {}}
    output_dir.mkdir(parents=True, exist_ok=False)
    for name, data in (("topology.yaml", topology), ("calibration.yaml", calibration),
                       ("formation.yaml", formation)):
        (output_dir / name).write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    manifest = {"identity_file": str(identity_path.resolve()),
                "scenario_file": str(scenario_path.resolve()),
                "uav_ids": list(scenario.uav_ids), "exec_targets": selected,
                "calibration_uavs": anchors,
                "topology_file": str((output_dir / "topology.yaml").resolve()),
                "calibration_file": str((output_dir / "calibration.yaml").resolve()),
                "formation_file": str((output_dir / "formation.yaml").resolve())}
    (output_dir / "deployment.json").write_text(
        json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path, required=True)
    parser.add_argument("--identity", type=Path, required=True)
    parser.add_argument("--topology", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(generate(args.scenario, args.identity, args.topology, args.outdir), indent=2))


if __name__ == "__main__":
    main()
