#!/usr/bin/env python3
"""生成 two_teams_recon 的两套任务包与共享 GCS 配置。

环境固定为 src/swarm_simulation/launch/multi_uav_ego_15sim.launch 的 15 机布局。
一次生成 15 机、一起标定（原点 = cluster.origin_uav 机位），分两批运行：
  team1 = UAV1–7，team2 = UAV8–15。
输出：
  generated/team1/、generated/team2/   各队 mission_spec/package/config(编队)/onboard
  generated/combined/                  共享 calibration.yaml + 两份编队 YAML + manifest

用法：
    python3 verification/two_teams_recon/generate_mission.py
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
RUNTIME_SRC = ROOT / "references" / "runtime" / "src"
for path in (ROOT, RUNTIME_SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import yaml  # noqa: E402

from verification.full_fleet_joint.generate_mission import (  # noqa: E402
    GenerationError,
    sha256,
    uav_name,
    uav_num,
    uid,
    write_json,
    write_yaml,
)

DEFAULT_TEAMS = HERE / "teams.yaml"
DEFAULT_OUT = HERE / "generated"
SCHEMA_VERSION = "1.0"
GROUP_ID = "GroupA"


def load_teams(path: Path = DEFAULT_TEAMS) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict) or str(data.get("schema_version")) not in ("1.0", "1"):
        raise GenerationError("teams.yaml schema_version 必须是 1.0")
    return data


def team_config(cfg: Dict[str, Any], team: str) -> Dict[str, Any]:
    key = "team%s" % str(team)
    teams = cfg.get("teams") or {}
    if key not in teams:
        raise GenerationError("未知编队 %r" % team)
    return teams[key]


def _sorted_uavs(uavs: Sequence[str]) -> List[str]:
    return sorted((str(u) for u in uavs), key=uav_num)


def cluster_layout(cfg: Dict[str, Any]) -> Dict[str, Tuple[float, float]]:
    """与 15sim 出生布局一致的标称任务坐标（以 UAV1 为 0,0，右 x / 前 y）。"""
    cluster = cfg["cluster"]
    columns = [float(v) for v in cluster["columns_m"]]
    rows = [float(v) for v in cluster["row_y_m"]]
    layout: Dict[str, Tuple[float, float]] = {}
    for row_index, row in enumerate(cluster["row_uavs"]):
        if len(row) != len(columns):
            raise GenerationError("cluster 每排数量必须等于 columns_m")
        for col_index, uav in enumerate(row):
            layout[str(uav)] = (columns[col_index], rows[row_index])
    return layout


def ground_task_positions(cfg: Dict[str, Any]) -> Dict[str, Tuple[float, float]]:
    """相对标定原点的地面位置：标称布局 − origin_uav 机位。"""
    layout = cluster_layout(cfg)
    origin_uav = str(cfg["cluster"]["origin_uav"])
    if origin_uav not in layout:
        raise GenerationError("origin_uav 不在集群布局中")
    ox, oy = layout[origin_uav]
    return {uav: (x - ox, y - oy) for uav, (x, y) in layout.items()}


def offsets_from_slots(
    uavs: Sequence[str], slots: Dict[str, Tuple[float, float]], leader: str
) -> Dict[str, List[float]]:
    leader_x, leader_y = slots[leader]
    return {
        uid(uav): [
            round(slots[uav][0] - leader_x, 6),
            round(slots[uav][1] - leader_y, 6),
            0.0,
        ]
        for uav in uavs
    }


def _pose(x: float, y: float, z: float) -> Dict[str, float]:
    return {"x": float(x), "y": float(y), "z": float(z), "yaw": 0.0}


def _plan(plan_id: str, assignments: Dict[str, Tuple[str, Dict[str, Any]]]) -> Dict[str, Any]:
    return {
        "plan_id": plan_id,
        "group_id": GROUP_ID,
        "completion_policy": "ALL",
        "robot_assignments": {
            robot_id: {
                "robot_id": robot_id,
                "command_type": command,
                "payload": payload,
            }
            for robot_id, (command, payload) in assignments.items()
        },
    }


def coverage_route(area: Dict[str, float], strips: int) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    """按矩形区域生成 boustrophedon 领机航点：coverage1 第一条，coverage2 其余。"""
    width = float(area["max_x"]) - float(area["min_x"])
    length = float(area["max_y"]) - float(area["min_y"])
    step = width / strips
    y_start, y_end = float(area["min_y"]), float(area["max_y"])
    centers = [float(area["min_x"]) + (index + 0.5) * step for index in range(strips)]
    cov1 = [(centers[0], y_start), (centers[0], y_end)]
    cov2: List[Tuple[float, float]] = []
    current_y = y_end
    for index in range(1, strips):
        cov2.append((centers[index], current_y))
        current_y = y_start if index % 2 == 1 else y_end
        cov2.append((centers[index], current_y))
    return cov1, cov2


def build_mission(cfg: Dict[str, Any], team: Dict[str, Any], key: str) -> Dict[str, Any]:
    defaults = cfg["defaults"]
    formation = [str(u) for u in team["formation"]]
    available = [str(u) for u in team["available"]]
    leader = str(team["leader"])
    faults = [str(u) for u in (team.get("faults") or [])]
    spares = [str(u) for u in (team.get("spares") or [])]
    replacements = {str(k): str(v) for k, v in (team.get("replacements") or {}).items()}

    if leader not in formation:
        raise GenerationError("leader 必须属于 formation")
    if leader in faults:
        raise GenerationError("leader 不能是损毁机")
    if set(faults) - set(formation):
        raise GenerationError("faults 必须属于 formation")
    if set(spares) & set(formation):
        raise GenerationError("spares 不得与 formation 重叠")
    if set(replacements) != set(faults):
        raise GenerationError("replacements 必须为每个损毁机指定替补机")
    if set(replacements.values()) != set(spares):
        raise GenerationError("replacements 的替补机必须与 spares 一致")
    participants = _sorted_uavs(formation + spares)
    if set(participants) != set(available):
        raise GenerationError("available 必须恰好等于 formation + spares")

    slots = ground_task_positions(cfg)
    for uav in participants:
        if uav not in slots:
            raise GenerationError("%s 不在集群布局中" % uav)
    initial_ids = [uid(uav) for uav in _sorted_uavs(formation)]
    faulted_ids = [uid(uav) for uav in _sorted_uavs(faults)]
    spare_ids = [uid(uav) for uav in _sorted_uavs(spares)]

    initial_offsets = offsets_from_slots(formation, slots, leader)
    survivors = [uav for uav in formation if uav not in set(faults)]
    recovered = _sorted_uavs(survivors + spares)
    recovered_offsets = offsets_from_slots(survivors, slots, leader)
    for fault, spare in replacements.items():
        recovered_offsets[uid(spare)] = list(initial_offsets[uid(fault)])

    ground = {
        uid(uav): {"x": slots[uav][0], "y": slots[uav][1], "z": 0.0}
        for uav in participants
    }

    alt = float(defaults["coverage_alt_m"])
    forward = float(defaults["prepare_forward_m"])
    strips = int(defaults["coverage_strips"])
    area = team["area"]
    cov1_wp, cov2_wp = coverage_route(area, strips)
    final_x, final_y = cov2_wp[-1]
    leader_ground_x, leader_ground_y = slots[leader]
    return_x, return_y = leader_ground_x, leader_ground_y + forward
    fleet_area = cfg["world"]["flight_area"]

    def route(waypoints: Sequence[Tuple[float, float]]) -> List[Dict[str, float]]:
        return [_pose(x, y, alt) for x, y in waypoints]

    def at_offsets(offsets: Dict[str, List[float]], x: float, y: float, z: float) -> Dict[str, Dict[str, float]]:
        return {
            robot_id: _pose(x + off[0], y + off[1], z)
            for robot_id, off in offsets.items()
        }

    strip1_x, strip1_y = cov1_wp[1]
    fault_targets = {
        uid(fault): _pose(
            strip1_x + initial_offsets[uid(fault)][0],
            strip1_y + initial_offsets[uid(fault)][1],
            float(defaults["fault_exit_alt_m"]),
        )
        for fault in faults
    }
    recovery_targets = {
        uid(fault): _pose(
            strip1_x + initial_offsets[uid(fault)][0],
            strip1_y + initial_offsets[uid(fault)][1],
            alt,
        )
        for fault in faults
    }
    recovery_hold_targets = at_offsets(recovered_offsets, strip1_x, strip1_y, alt)

    columns = len(cfg["cluster"]["columns_m"])
    front_row = formation[:columns]
    back_row = formation[columns:]
    prepare_back_targets = {
        uid(uav): _pose(slots[uav][0], slots[uav][1] + forward, alt)
        for uav in back_row
    }
    prepare_front_targets = {
        uid(uav): _pose(slots[uav][0], slots[uav][1] + forward, alt)
        for uav in front_row
    }
    prepare_targets = {**prepare_back_targets, **prepare_front_targets}
    spare_hold_targets = {
        uid(spare): _pose(
            slots[spare][0], slots[spare][1], float(defaults["spare_hold_alt_m"])
        )
        for spare in spares
    }

    plans = [
        # 分批 prepare：先后排、再前排，避免 6 机同时前移导致 EGO 规划冲突
        # （与 full_fleet_joint 的 prepare-a-1/a-2 拆分同理）。
        _plan(
            "prepare-formation-back",
            {
                task_id: ("MOVE_TO", {"target_pose": target})
                for task_id, target in prepare_back_targets.items()
            },
        ),
        _plan(
            "prepare-formation-front",
            {
                task_id: ("MOVE_TO", {"target_pose": target})
                for task_id, target in prepare_front_targets.items()
            },
        ),
        # 备用机只做 observe：出生随机偏差由标定原点吸收，不额外下发小位移命令。
        _plan(
            "prepare-spares",
            {
                task_id: ("MOVE_TO", {"target_pose": target})
                for task_id, target in spare_hold_targets.items()
            },
        ),
        _plan(
            "coverage-segment-1",
            {
                uid(uav): (
                    "FOLLOW_ROUTE",
                    {"waypoints": route(cov1_wp)}
                    if uav == leader
                    else {"formation_follow": True},
                )
                for uav in formation
            },
        ),
        _plan(
            "fault-exit",
            {
                task_id: ("FAULT_EXIT", {"waypoints": [target]})
                for task_id, target in fault_targets.items()
            },
        ),
        _plan(
            "recovery-spares",
            {
                uid(replacements[fault]): (
                    "MOVE_TO",
                    {"target_pose": recovery_targets[uid(fault)]},
                )
                for fault in faults
            },
        ),
        _plan(
            "recovery-hold",
            {
                task_id: ("MOVE_TO", {"target_pose": target})
                for task_id, target in recovery_hold_targets.items()
            },
        ),
        _plan(
            "coverage-segment-2",
            {
                uid(uav): (
                    "FOLLOW_ROUTE",
                    {"waypoints": route(cov2_wp)}
                    if uav == leader
                    else {"formation_follow": True},
                )
                for uav in recovered
            },
        ),
        _plan(
            "return-formation",
            {
                uid(uav): (
                    "FOLLOW_ROUTE",
                    {"waypoints": route([(return_x, return_y)])}
                    if uav == leader
                    else {"formation_follow": True},
                )
                for uav in recovered
            },
        ),
    ]
    hold_targets = {
        **at_offsets(recovered_offsets, return_x, return_y, alt),
        **fault_targets,
    }
    plans.append(
        _plan(
            "hold-end",
            {
                task_id: ("MOVE_TO", {"target_pose": target})
                for task_id, target in hold_targets.items()
            },
        )
    )

    plan_members = {
        plan["plan_id"]: sorted(plan["robot_assignments"], key=lambda i: int(i[1:]))
        for plan in plans
    }
    expected = {
        "prepare-formation-back": prepare_back_targets,
        "prepare-formation-front": prepare_front_targets,
        # prepare-spares 不做位置判据：备用机悬停在出生点，随机偏差由标定原点吸收。
        "coverage-segment-1": at_offsets(initial_offsets, strip1_x, strip1_y, alt),
        "fault-exit": fault_targets,
        "recovery-spares": {
            uid(replacements[fault]): recovery_targets[uid(fault)] for fault in faults
        },
        "recovery-hold": recovery_hold_targets,
        "coverage-segment-2": at_offsets(recovered_offsets, final_x, final_y, alt),
        "return-formation": at_offsets(recovered_offsets, return_x, return_y, alt),
        "hold-end": hold_targets,
    }

    timeouts = cfg["defaults"]["timeouts_s"]
    phases = [
        {
            "name": "prepare",
            "timeout_s": float(timeouts["prepare"]),
            "steps": [
                {"plan_id": "prepare-formation-back", "observe_only": False},
                {"plan_id": "prepare-formation-front", "observe_only": False},
                {"plan_id": "prepare-spares", "observe_only": True},
            ],
        },
        {
            "name": "coverage1",
            "timeout_s": float(timeouts["coverage1"]),
            "steps": [{"plan_id": "coverage-segment-1", "observe_only": False}],
        },
        {
            "name": "fault",
            "timeout_s": float(timeouts["fault"]),
            "steps": [{"plan_id": "fault-exit", "observe_only": False}],
        },
        {
            "name": "recovery",
            "timeout_s": float(timeouts["recovery"]),
            "steps": [
                {"plan_id": "recovery-spares", "observe_only": False},
                {"plan_id": "recovery-hold", "observe_only": True},
            ],
        },
        {
            "name": "coverage2",
            "timeout_s": float(timeouts["coverage2"]),
            "steps": [{"plan_id": "coverage-segment-2", "observe_only": False}],
            "publishes": "RECON_COMPLETE",
        },
        {
            "name": "return",
            "timeout_s": float(timeouts["return"]),
            "steps": [{"plan_id": "return-formation", "observe_only": False}],
        },
        {
            "name": "hold",
            "timeout_s": float(timeouts["hold"]),
            "dwell_s": 3.0,
            "steps": [{"plan_id": "hold-end", "observe_only": True}],
        },
    ]

    margin = float(fleet_area["safety_margin_m"])
    for plan_id, targets in expected.items():
        for task_id, target in targets.items():
            if not (
                fleet_area["min_x"] + margin <= target["x"] <= fleet_area["max_x"] - margin
                and fleet_area["min_y"] + margin <= target["y"] <= fleet_area["max_y"] - margin
            ):
                raise GenerationError("%s/%s 超出飞行区域" % (plan_id, task_id))
    for target in fault_targets.values():
        if abs(target["z"] - 8.0) > 1e-9:
            raise GenerationError("FAULT_EXIT z 必须为 8.0 m")

    return {
        "team_key": key,
        "name": str(team.get("name") or "team"),
        "available": available,
        "participants": participants,
        "formation": _sorted_uavs(formation),
        "initial_ids": initial_ids,
        "faulted_ids": faulted_ids,
        "spare_ids": spare_ids,
        "survivor_ids": [uid(uav) for uav in _sorted_uavs(survivors)],
        "recovered_ids": [uid(uav) for uav in recovered],
        "replacements": {uid(k): uid(v) for k, v in replacements.items()},
        "leader": leader,
        "leader_id": uid(leader),
        "ground": ground,
        "slots": {uid(u): [slots[u][0], slots[u][1]] for u in participants},
        "initial_offsets": initial_offsets,
        "recovered_offsets": recovered_offsets,
        "plans": plans,
        "plan_members": plan_members,
        "expected": expected,
        "phases": phases,
        "coverage": {
            "alt_m": alt,
            "strips": strips,
            "lateral_step_m": (float(area["max_x"]) - float(area["min_x"])) / strips,
            "segment_length_m": float(area["max_y"]) - float(area["min_y"]),
            "area": dict(area),
            "coverage1_waypoints": [list(item) for item in cov1_wp],
            "coverage2_waypoints": [list(item) for item in cov2_wp],
            "return_point": [return_x, return_y],
            "final_point": [final_x, final_y],
        },
        "calibration": {
            "calibration_uavs": [
                uav_name(uav_num(u)) for u in cfg["cluster"]["calibration_uavs"]
            ],
            "all_uavs": [
                uav_name(uav_num(uav))
                for uav in _sorted_uavs(
                    [u for row in cfg["cluster"]["row_uavs"] for u in row]
                )
            ],
            "sample_window_s": float(defaults["sample_window_s"]),
            "origin_uav": str(cfg["cluster"]["origin_uav"]),
        },
        "mission_id": "%s-team%s" % (cfg["world"]["mission_prefix"], key),
    }


def build_package(mission: Dict[str, Any], cfg: Dict[str, Any]) -> Dict[str, Any]:
    faults = set(mission["faulted_ids"])
    spares = set(mission["spare_ids"])
    leader_id = mission["leader_id"]
    robots = []
    for uav in mission["participants"]:
        task_id = uid(uav)
        if task_id in spares:
            capabilities = ["MOVE_TO", "FOLLOW_ROUTE"]
            role = "Reserve"
        else:
            capabilities = ["MOVE_TO", "FOLLOW_ROUTE"]
            role = "Recon"
            if task_id in faults:
                capabilities.append("FAULT_EXIT")
        robots.append(
            {
                "uav_id": task_id,
                "group_id": GROUP_ID,
                "role": role,
                "is_leader": task_id == leader_id,
                "is_reserve": task_id in spares,
                "capabilities": capabilities,
            }
        )
    bootstrap = {
        "schema_version": SCHEMA_VERSION,
        "mission_id": mission["mission_id"],
        "group_id": GROUP_ID,
        "roster": {
            "group_id": GROUP_ID,
            "active_ids": mission["initial_ids"],
            "reserve_ids": mission["spare_ids"],
            "failed_ids": [],
            "inactive_ids": [],
        },
        "initial_states": [
            {"uav_id": uid(uav), "online": True, "available": True}
            for uav in mission["participants"]
        ],
        "blackboard": {"mission_armed": True},
        "planned_fault_count": len(mission["faulted_ids"]),
        "planned_recovery_count": len(mission["spare_ids"]),
    }
    world = {
        "schema_version": SCHEMA_VERSION,
        "coordinate_frame": "mission_enu",
        "target_config_version": "two-teams-recon-v2",
        "flight_area": dict(cfg["world"]["flight_area"]),
        "targets": {"recon-area": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
    }
    formation_offsets = {
        "leader_id": leader_id,
        "mission_anchor": [0.0, 0.0, 0.0],
        "formations": {
            "recon_initial": {
                "participants": mission["initial_ids"],
                "offsets": mission["initial_offsets"],
            },
            "recon_recovered": {
                "participants": mission["recovered_ids"],
                "offsets": mission["recovered_offsets"],
            },
        },
    }
    return {
        "robots": {"schema_version": SCHEMA_VERSION, "uavs": robots},
        "bootstrap": bootstrap,
        "world": world,
        "plans": {"schema_version": SCHEMA_VERSION, "plans": mission["plans"]},
        "formation_offsets": formation_offsets,
    }


def build_spec(mission: Dict[str, Any], teams_path: Path) -> Dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "mission_id": mission["mission_id"],
        "team_key": mission["team_key"],
        "team_name": mission["name"],
        "teams_file": str(Path(teams_path).resolve()),
        "environment_launch": "src/swarm_simulation/launch/multi_uav_ego_15sim.launch",
        "available_uavs": mission["available"],
        "all_ids": [uid(uav) for uav in mission["participants"]],
        "initial_ids": mission["initial_ids"],
        "survivor_ids": mission["survivor_ids"],
        "faulted_ids": mission["faulted_ids"],
        "spare_ids": mission["spare_ids"],
        "recovered_ids": mission["recovered_ids"],
        "replacements": mission["replacements"],
        "leader_id": mission["leader_id"],
        "ground_positions": mission["ground"],
        "formation_offsets": {
            "recon_initial": mission["initial_offsets"],
            "recon_recovered": mission["recovered_offsets"],
        },
        "coverage": mission["coverage"],
        "phases": mission["phases"],
        "plan_members": mission["plan_members"],
        "expected_targets": mission["expected"],
        "calibration": mission["calibration"],
    }


def build_onboard_commands(mission: Dict[str, Any], cfg: Dict[str, Any]) -> str:
    numbers = [uav_num(uav) for uav in mission["participants"]]
    onboard = cfg["defaults"]["onboard"]
    lines = [
        "#!/usr/bin/env bash",
        "# 生成器输出；只打印本队逐机启动命令，不执行。",
        "# participants: %s" % " ".join(mission["participants"]),
    ]
    for number in numbers:
        peers = [n for n in numbers if n != number]
        args = [
            "neighbor_intents:=%s" % ",".join("/UAV%d/trajectory_intent" % n for n in peers),
            "neighbor_odom_topics:=%s" % ",".join("/UAV%d/swarm_pose" % n for n in peers),
            "uav_task_timeout_s:=%d" % int(onboard["uav_task_timeout_s"]),
            "bundle_transport:=%s" % onboard["bundle_transport"],
        ]
        lines.append(
            "bash src/tcp_to_ros/tools/onboard_ctl.sh start %d -- %s"
            % (number, " ".join(args))
        )
    return "\n".join(lines) + "\n"


def build_flow_env(mission: Dict[str, Any]) -> str:
    return (
        "#!/usr/bin/env bash\n"
        "# 生成器输出，供 verification/two_teams_recon/flow.sh source。\n"
        "export AIR_FLOW_UAVS=%r\n"
        "export AIR_FLOW_LEADER=%d\n"
        "export AIR_FLOW_DIR=two_teams_recon\n"
        % (
            " ".join(str(uav_num(uav)) for uav in mission["participants"]),
            uav_num(mission["leader"]),
        )
    )


def validate_package(out_dir: Path) -> None:
    from uav_bt_runtime.config import load_json_model, load_mission_context
    from uav_bt_runtime.package import WorldConfig

    package_dir = out_dir / "package"
    context = load_mission_context(
        package_dir / "robots.json",
        package_dir / "bootstrap.json",
        package_dir / "plans" / "plans.json",
    )
    load_json_model(package_dir / "world.json", WorldConfig)
    leaders = [spec.uav_id for spec in context.uav_specs.values() if spec.is_leader]
    if len(leaders) != 1:
        raise GenerationError("robots.json 必须恰好一个 leader")
    if set(context.roster.active_ids) | set(context.roster.reserve_ids) != set(context.uav_specs):
        raise GenerationError("active+reserve 必须覆盖全部参与机")


def generate(teams_path: Path, out_dir: Path) -> Dict[str, Any]:
    cfg = load_teams(teams_path)
    out = Path(out_dir)
    specs = {}
    manifests = {}
    for team in ("1", "2"):
        config = team_config(cfg, team)
        mission = build_mission(cfg, config, team)
        package = build_package(mission, cfg)
        spec = build_spec(mission, teams_path)
        specs[team] = spec
        team_dir = out / ("team%s" % team)
        package_dir = team_dir / "package"
        config_dir = team_dir / "config"
        write_json(package_dir / "robots.json", package["robots"])
        write_json(package_dir / "bootstrap.json", package["bootstrap"])
        write_json(package_dir / "world.json", package["world"])
        write_json(package_dir / "plans" / "plans.json", package["plans"])
        write_yaml(config_dir / "formation_offsets.yaml", package["formation_offsets"])
        write_json(team_dir / "mission_spec.json", spec)
        (team_dir / "onboard_commands.sh").write_text(
            build_onboard_commands(mission, cfg), encoding="utf-8"
        )
        (team_dir / "flow_env.sh").write_text(
            build_flow_env(mission), encoding="utf-8"
        )
        files = [
            package_dir / "robots.json",
            package_dir / "bootstrap.json",
            package_dir / "world.json",
            package_dir / "plans" / "plans.json",
            config_dir / "formation_offsets.yaml",
            team_dir / "mission_spec.json",
            team_dir / "onboard_commands.sh",
            team_dir / "flow_env.sh",
        ]
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "teams_file": str(Path(teams_path).resolve()),
            "teams_sha256": sha256(Path(teams_path)),
            "team": team,
            "team_name": mission["name"],
            "participants": mission["participants"],
            "initial": mission["initial_ids"],
            "faulted": mission["faulted_ids"],
            "spares": mission["spare_ids"],
            "recovered": mission["recovered_ids"],
            "replacements": mission["replacements"],
            "leader": mission["leader_id"],
            "coverage": mission["coverage"],
            "calibration": mission["calibration"],
            "file_sha256": {str(path.relative_to(team_dir)): sha256(path) for path in files},
        }
        manifests[team] = manifest
        write_json(team_dir / "manifest.json", manifest)
        validate_package(team_dir)

    # 共享标定配置 + 两队编队配置（供双 backend GCS 使用）
    combined = out / "combined"
    calibration = {
        "calibration_uavs": specs["1"]["calibration"]["calibration_uavs"],
        "all_uavs": specs["1"]["calibration"]["all_uavs"],
        "sample_window_s": specs["1"]["calibration"]["sample_window_s"],
    }
    write_yaml(combined / "calibration.yaml", calibration)
    for team in ("1", "2"):
        package = build_package(build_mission(cfg, team_config(cfg, team), team), cfg)
        write_yaml(combined / ("formation_team%s.yaml" % team), package["formation_offsets"])
    combined_manifest = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "environment_launch": "src/swarm_simulation/launch/multi_uav_ego_15sim.launch",
        "calibration": calibration,
        "teams": {team: manifests[team]["participants"] for team in ("1", "2")},
        "leaders": {team: specs[team]["leader_id"] for team in ("1", "2")},
        "areas": {team: specs[team]["coverage"]["area"] for team in ("1", "2")},
        "file_sha256": {
            str(path.relative_to(combined)): sha256(path)
            for path in (
                combined / "calibration.yaml",
                combined / "formation_team1.yaml",
                combined / "formation_team2.yaml",
            )
        },
    }
    write_json(combined / "manifest.json", combined_manifest)
    return {"specs": specs, "combined": combined_manifest}


def main(argv: Sequence[str] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teams", default=str(DEFAULT_TEAMS))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    args = parser.parse_args(argv)
    result = generate(Path(args.teams), Path(args.out))
    print("已生成 15 机两批任务包：%s" % Path(args.out))
    for team in ("1", "2"):
        spec = result["specs"][team]
        print(
            "  team%s leader=%s formation=%s faults=%s spares=%s area=%s"
            % (
                team,
                spec["leader_id"],
                ",".join(spec["initial_ids"]),
                ",".join(spec["faulted_ids"]),
                ",".join(spec["spare_ids"]),
                spec["coverage"]["area"],
            )
        )
    print("  combined: generated/combined/{calibration.yaml,formation_team1.yaml,formation_team2.yaml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
