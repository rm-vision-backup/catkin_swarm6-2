#!/usr/bin/env python3
"""按当天实际可用机生成 14–16 架 A/B 联合任务包（初始 9 架 + 备用补充）。

设计（2026-10-20 用户修订）：
- 初始 A 组 = 地面 3×3 块（默认第 1–3 排、第 1–3 列），垂直起飞后直接成编队；
- 备用机 = 3×3 块外侧相邻列的 1–3 架，在 coverage1 终点前方待命；
- 故障 = 3×3 块边缘列的 1–3 架，损毁后备用机补位回 3×3；
- B 组从“编队+备用机”之外的剩余机中选（也可 --b 显式指定）；
  B 组数量由 mission.b_count / b_strike_count 配置（默认 4 = 3 打击 + 1 补打），
  目标数等于打击机数；最后一架为补打机。
- B 目标放在侦查覆盖区内；A 返航时从 B 上方 7m 通过。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import sys
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT_FLEET = HERE / "fleet.yaml"
DEFAULT_OUT = HERE / "generated"
RUNTIME_SRC = ROOT / "references" / "runtime" / "src"
TCP_TO_ROS_SCRIPTS = ROOT / "src" / "tcp_to_ros" / "scripts"
for _path in (ROOT, RUNTIME_SRC, TCP_TO_ROS_SCRIPTS):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from tcp_to_ros.time_offset_scheduler import (  # noqa: E402
    ScheduleParams,
    UavSegment,
    compute_schedule,
)

SCHEMA_VERSION = "1.0"
MISSION_ID = "full-fleet-joint-001"
TARGET_CONFIG_VERSION = "full-fleet-ground-v2"
A_ID = "A%02d"


class GenerationError(RuntimeError):
    """Requested fleet cannot produce a safe, reviewable package."""


def uav_num(uav: str) -> int:
    return int(str(uav).upper().replace("UAV", ""))


def uav_name(number: int) -> str:
    return "UAV%d" % int(number)


def uid(uav: str) -> str:
    return A_ID % uav_num(uav)


def load_fleet(path: Path = DEFAULT_FLEET) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise GenerationError("fleet.yaml 必须是映射")
    if str(data.get("schema_version")) not in ("1.0", "1"):
        raise GenerationError("fleet.yaml schema_version 必须是 1.0")
    return data


def parse_uavs(values: Sequence[str]) -> List[str]:
    found: List[str] = []
    for raw in values:
        for token in str(raw).replace(";", ",").split(","):
            token = token.strip()
            if not token:
                continue
            upper = token.upper()
            if upper.startswith("UAV"):
                numbers = [int(upper[3:])]
            elif "-" in token:
                left, right = token.split("-", 1)
                numbers = list(range(int(left), int(right) + 1))
            else:
                numbers = [int(token)]
            for number in numbers:
                if not 1 <= number <= 16:
                    raise GenerationError("UAV 编号必须在 1–16：%r" % token)
                name = uav_name(number)
                if name not in found:
                    found.append(name)
    return sorted(found, key=uav_num)


def parse_replacements(values: Sequence[str]) -> Dict[str, str]:
    """Parse repeated --replace SLOT=ACTUAL options."""
    result: Dict[str, str] = {}
    for raw in values or ():
        text = str(raw).strip()
        if not text:
            continue
        if "=" not in text:
            raise GenerationError(
                "--replace 需要 SLOT=ACTUAL，例如 UAV13=UAV16"
            )
        slot, actual = text.split("=", 1)
        slot = uav_name(uav_num(slot))
        actual = uav_name(uav_num(actual))
        if slot == actual:
            raise GenerationError("槽位替换不能指向自身：%s" % text)
        if slot in result and result[slot] != actual:
            raise GenerationError("槽位 %s 有多个替换目标" % slot)
        result[slot] = actual
    if len(set(result.values())) != len(result):
        raise GenerationError("一个实际 UAV 不能顶替多个槽位：%s" % result)
    return result


def normalize_replacements(
    fleet: Dict[str, Any], replacements: Optional[Dict[str, str]] = None
) -> Dict[str, str]:
    """Merge fleet.yaml slot_replacements with CLI replacements, e.g. UAV13→UAV16."""
    combined: Dict[str, str] = {}
    sources = []
    configured = fleet.get("mission", {}).get("slot_replacements") or {}
    if configured:
        sources.append(configured)
    if replacements:
        sources.append(replacements)
    for source in sources:
        for slot, actual in source.items():
            slot = uav_name(uav_num(slot))
            actual = uav_name(uav_num(actual))
            if slot == actual:
                raise GenerationError("槽位替换不能指向自身：%s" % slot)
            if slot in combined and combined[slot] != actual:
                raise GenerationError("槽位 %s 有多个替换目标" % slot)
            combined[slot] = actual
    if len(set(combined.values())) != len(combined):
        raise GenerationError("一个实际 UAV 不能顶替多个槽位：%s" % combined)
    return combined


def effective_rows(
    fleet: Dict[str, Any], replacements: Optional[Dict[str, str]] = None
) -> List[List[Optional[str]]]:
    """Apply slot replacements: the actual UAV takes the failed UAV's grid cell."""
    replacements = normalize_replacements(fleet, replacements)
    rows: List[List[Optional[str]]] = [
        list(row) for row in fleet["grid"]["row_uavs"]
    ]
    actuals = set(replacements.values())
    for row in rows:
        for index, uav in enumerate(row):
            if str(uav) in actuals:
                row[index] = None
    for slot, actual in replacements.items():
        placed = False
        for row in rows:
            for index, uav in enumerate(row):
                if str(uav) == slot:
                    row[index] = actual
                    placed = True
        if not placed:
            raise GenerationError("替换槽位 %s 不在 grid.row_uavs 内" % slot)
    return rows


def grid_map(
    fleet: Dict[str, Any], replacements: Optional[Dict[str, str]] = None
) -> Dict[str, Tuple[float, float]]:
    replacements = normalize_replacements(fleet, replacements)
    rows = effective_rows(fleet, replacements)
    ys = fleet["grid"]["row_y_m"]
    xs = fleet["grid"]["columns_m"]
    absent = {
        uav_name(uav_num(u)) for u in fleet["grid"].get("absent_uavs", [])
    }
    result: Dict[str, Tuple[float, float]] = {}
    for row, y in zip(rows, ys):
        for index, uav in enumerate(row):
            if index >= len(xs):
                raise GenerationError("row_uavs 与 columns_m 长度不一致")
            if uav is None:
                continue
            result[str(uav)] = (float(xs[index]), float(y))
    still_present = sorted(absent & set(result), key=uav_num)
    if still_present:
        raise GenerationError(
            "grid.absent_uavs 仍出现在 row_uavs：%s" % ", ".join(still_present)
        )
    missing = [
        u for u in ("UAV%d" % n for n in range(1, 17))
        if u not in result and u not in replacements and u not in absent
    ]
    if missing:
        raise GenerationError("网格缺少 UAV：%s" % ", ".join(missing))
    return result


def block_spec(
    fleet: Dict[str, Any],
    block: str,
    replacements: Optional[Dict[str, str]] = None,
) -> Dict[str, Any]:
    """Return the ground 3×3 block, its edge column, adjacent spare column and offsets."""
    if block not in ("left", "right"):
        raise GenerationError("formation_block 必须是 left/right")
    rows = effective_rows(fleet, replacements)
    xs = [float(v) for v in fleet["grid"]["columns_m"]]
    ys = [float(v) for v in fleet["grid"]["row_y_m"]]
    row_idx = [0, 1, 2]
    col_idx = [0, 1, 2] if block == "left" else [2, 3, 4]
    spare_col = 3 if block == "left" else 1
    edge_col = 2  # block's column adjacent to the spare column

    def cell(row: int, col: int) -> str:
        uav = rows[row][col]
        if uav is None:
            raise GenerationError(
                "替换后地面网格为空：row=%d col=%d（检查 slot_replacements）"
                % (row, col)
            )
        return str(uav)

    formation = [cell(r, c) for r in row_idx for c in col_idx]
    # Leader on the edge opposite the damaged edge: left block uses its leftmost
    # column, right block uses its rightmost column. Avoids the center aircraft
    # leading the formation and keeps the leader away from the sacrificial edge.
    leader_col = col_idx[0] if block == "left" else col_idx[2]
    leader = cell(row_idx[1], leader_col)
    leader_x = xs[leader_col]
    leader_y = ys[row_idx[1]]
    offsets = {
        cell(r, c): (xs[c] - leader_x, ys[r] - leader_y)
        for r in row_idx
        for c in col_idx
    }
    spares_all = [cell(r, spare_col) for r in row_idx]
    edge_all = [cell(r, edge_col) for r in row_idx]
    spare_x = xs[spare_col]
    edge_x = xs[edge_col]
    edge_offsets = {
        cell(r, edge_col): (xs[edge_col] - leader_x, ys[r] - leader_y)
        for r in row_idx
    }
    return {
        "block": block,
        "formation": formation,
        "leader": leader,
        "leader_xy": (leader_x, leader_y),
        "offsets": offsets,
        "spares_all": spares_all,
        "edge_all": edge_all,
        "spare_x": spare_x,
        "edge_x": edge_x,
        "edge_offsets": edge_offsets,
        "rows": row_idx,
        "cols": col_idx,
    }


def explicit_block(
    fleet: Dict[str, Any],
    cfg: Dict[str, Any],
    replacements: Optional[Dict[str, str]] = None,
) -> Dict[str, Any]:
    """Build a block dict from explicit formation/fault/spare roles.

    Used by the 2x3 profile; the auto 3x3 path keeps using block_spec().
    """
    replacements = normalize_replacements(fleet, replacements)
    grid = grid_map(fleet, replacements)
    formation = [str(u) for u in (cfg.get("formation_uavs") or [])]
    leader = str(cfg.get("leader_uav") or "")
    faults = [str(u) for u in (cfg.get("faulted_uavs") or [])]
    spares = [str(u) for u in (cfg.get("spare_uavs") or [])]
    if not formation or leader not in formation:
        raise GenerationError("custom_roles: formation_uavs/leader_uav 配置无效")
    if len(faults) != len(spares):
        raise GenerationError(
            "custom_roles: faulted_uavs 与 spare_uavs 数量必须一致"
        )
    for uav in formation + faults + spares:
        if uav not in grid:
            raise GenerationError("custom_roles: %s 不在网格中" % uav)
    if not set(faults) <= set(formation):
        raise GenerationError("custom_roles: faulted_uavs 必须属于 formation_uavs")
    if set(spares) & set(formation):
        raise GenerationError("custom_roles: spare_uavs 不得与 formation_uavs 重叠")
    if leader in faults:
        raise GenerationError("custom_roles: leader_uav 不能是损毁机")
    side = str(cfg.get("fault_exit_side", "left")).lower()
    if side not in ("left", "right"):
        raise GenerationError("custom_roles: fault_exit_side 必须是 left/right")
    leader_x, leader_y = grid[leader]
    offsets = {
        u: (grid[u][0] - leader_x, grid[u][1] - leader_y) for u in formation
    }
    return {
        "block": "custom",
        "formation": formation,
        "leader": leader,
        "leader_xy": (leader_x, leader_y),
        "offsets": offsets,
        "spares_all": spares,
        "edge_all": faults,
        "spare_x": leader_x,
        "edge_x": leader_x,
        "edge_offsets": {u: offsets[u] for u in faults},
        "rows": [0, 1, 2],
        "cols": [],
        "fault_exit_sign": -1.0 if side == "left" else 1.0,
        "target_origin_x": leader_x,
    }


def find_calibration(
    fleet: Dict[str, Any],
    participating: Sequence[str],
    replacements: Optional[Dict[str, str]] = None,
) -> Dict[str, Any]:
    replacements = normalize_replacements(fleet, replacements)
    grid = grid_map(fleet, replacements)
    available_set = set(participating)
    min_count = int(fleet["calibration"]["min_compute_uavs"])
    for row_index, row in enumerate(fleet["calibration"]["preferred_rows"], start=1):
        row = [replacements.get(str(u), str(u)) for u in row]
        if len(row) < 3:
            continue
        center_index = len(row) // 2
        center = row[center_index]
        subset: List[str] = []
        if center in available_set:
            subset.append(center)
        for distance in range(1, center_index + 1):
            left = center_index - distance
            right = center_index + distance
            if left >= 0 and right < len(row):
                if row[left] in available_set and row[right] in available_set:
                    subset.extend([row[left], row[right]])
        subset = sorted(set(subset), key=uav_num)
        if len(subset) < min_count:
            continue
        center_x, center_y = grid[center]
        xs = sorted(grid[u][0] - center_x for u in subset)
        for left, right in zip(xs, reversed(xs)):
            if abs(left + right) > 1e-9:
                raise GenerationError("标定子集不对称：%s" % subset)
        if any(abs(grid[u][1] - center_y) > 1e-9 for u in subset):
            raise GenerationError("标定子集跨排：%s" % subset)
        return {
            "compute": subset,
            "all": sorted(participating, key=uav_num),
            "row_index": row_index,
            "row_center": center,
            "center_x": center_x,
            "origin_y_nominal": center_y,
            # 任务坐标以标定排中心为原点：网格坐标减中心。
            "shift_x": -center_x,
            "shift_y": -center_y,
            "sample_window_s": float(fleet["calibration"]["sample_window_s"]),
        }
    raise GenerationError(
        "找不到 ≥%d 架的可用对称标定子集；请补机或换排后再生成" % min_count
    )


def pose(x: float, y: float, z: float) -> Dict[str, float]:
    return {"x": float(x), "y": float(y), "z": float(z), "yaw": 0.0}


def assign_mission(
    fleet: Dict[str, Any],
    available: Sequence[str],
    leader_override: Optional[str],
    b_override: Optional[Sequence[str]],
    replacements: Optional[Dict[str, str]] = None,
) -> Dict[str, Any]:
    cfg = fleet["mission"]
    replacements = normalize_replacements(fleet, replacements)
    available_set = set(available)
    absent = {
        uav_name(uav_num(u)) for u in fleet["grid"].get("absent_uavs", [])
    }
    if available_set & absent:
        raise GenerationError(
            "--available 包含网格缺失机：%s" % ", ".join(sorted(available_set & absent))
        )
    if (set(replacements) | set(replacements.values())) & absent:
        raise GenerationError("槽位替换涉及网格缺失机")
    for slot, actual in replacements.items():
        if slot in available_set:
            raise GenerationError(
                "槽位 %s 仍在可用集内；请从 --available 移除后再用 %s 顶替"
                % (slot, actual)
            )
        if actual not in available_set:
            raise GenerationError("替换机 %s 不在可用集内" % actual)
    grid = grid_map(fleet, replacements)
    custom_roles = bool(cfg.get("custom_roles", False))
    block_name = str(cfg.get("formation_block", "custom" if custom_roles else "left"))
    if custom_roles:
        block = explicit_block(fleet, cfg, replacements)
    else:
        block = block_spec(fleet, block_name, replacements)
    formation = block["formation"]
    if not set(formation) <= available_set:
        raise GenerationError(
            "初始编队 %s 不完整（缺 %s）"
            % (block_name, sorted(set(formation) - available_set))
        )
    if leader_override:
        if leader_override not in formation:
            raise GenerationError("--leader 必须属于初始编队（%s）" % formation)
        leader = leader_override
    else:
        leader = block["leader"]
    b_count_target = int(cfg.get("b_count", 4))
    b_strike_count = int(cfg.get("b_strike_count", b_count_target - 1))
    if b_strike_count < 1 or b_count_target != b_strike_count + 1:
        raise GenerationError(
            "b_count 必须等于 b_strike_count + 1（当前 %d / %d）"
            % (b_count_target, b_strike_count)
        )

    if custom_roles:
        damaged = [str(u) for u in (cfg.get("faulted_uavs") or [])]
        spares = [str(u) for u in (cfg.get("spare_uavs") or [])]
        b_uavs = [str(u) for u in (cfg.get("b_uavs") or [])]
        strike_uavs = [str(u) for u in (cfg.get("b_strike_uavs") or [])]
        reserve_uav = str(cfg.get("b_reserve_uav") or "")
        failed_uav = str(cfg.get("b_failed_uav") or "")
        if len(damaged) != len(spares):
            raise GenerationError("faulted_uavs 与 spare_uavs 数量必须一致")
        if not strike_uavs or not reserve_uav or not failed_uav:
            raise GenerationError("必须配置 b_strike_uavs/b_reserve_uav/b_failed_uav")
        if len(b_uavs) != b_count_target or len(strike_uavs) != b_strike_count:
            raise GenerationError(
                "b_count/b_strike_count 与显式 B 列表不一致"
            )
        if set(strike_uavs) | {reserve_uav} != set(b_uavs):
            raise GenerationError(
                "b_strike_uavs + b_reserve_uav 必须等于 b_uavs"
            )
        if failed_uav not in strike_uavs:
            raise GenerationError("b_failed_uav 必须属于 b_strike_uavs")
        for uav in b_uavs:
            if uav not in available_set:
                raise GenerationError("B %s 不在可用集内" % uav)
            if uav in formation or uav in set(spares):
                raise GenerationError("B %s 与编队/备用机冲突" % uav)
        fault_count = len(damaged)
        if leader in damaged:
            raise GenerationError("领机不能是损毁机")
    else:
        spare_column = [u for u in block["spares_all"] if u in available_set]
        explicit_b = [str(u) for u in (b_override or cfg.get("b_uavs") or [])]
        if explicit_b:
            for uav in explicit_b:
                if uav not in available_set:
                    raise GenerationError("--b %s 不在可用集内" % uav)
                if uav in formation:
                    raise GenerationError(
                        "--b %s 与地面 3×3 编队冲突；B 组请从编队/备用机之外的剩余机中选择" % uav
                    )
            b_uavs = sorted(explicit_b, key=uav_num)[:b_count_target]
        else:
            # B 先从备用列之外的剩余机中选；机队不足 16 架时（如 1–15、1–14），
            # 从备用列底部往上补，保留靠前的备用机用于边缘列损毁补位。
            outside = [
                u for u in available if u not in formation and u not in spare_column
            ]
            b_uavs = list(outside[:b_count_target])
            for uav in reversed(spare_column):
                if len(b_uavs) >= b_count_target:
                    break
                b_uavs.append(uav)
            b_uavs = sorted(b_uavs[:b_count_target], key=uav_num)
        if len(b_uavs) != b_count_target:
            raise GenerationError(
                "B 组需要 %d 架（%d 架打击 + 1 架补打），当前 %d 架；可用集/显式 B 不足"
                % (b_count_target, b_strike_count, len(b_uavs))
            )
        a_total = len(available_set) - len(b_uavs)
        fault_count = a_total - len(formation)
        if not 1 <= fault_count <= 3:
            raise GenerationError(
                "备用机/故障数必须是 1–3（A=%d，B=%d）；请调整可用集或 B 组"
                % (a_total, len(b_uavs))
            )
        spare_candidates = [u for u in spare_column if u not in set(b_uavs)]
        if len(spare_candidates) < fault_count:
            raise GenerationError(
                "相邻列备用机不足：需要 %d 架，扣除 B 组后相邻列可用 %s"
                % (fault_count, spare_candidates)
            )
        spares = sorted(spare_candidates[:fault_count], key=uav_num)
        spare_rows = [
            row_index
            for row_index, uav in enumerate(block["spares_all"])
            if uav in spares
        ]
        damaged = [block["edge_all"][row] for row in spare_rows]
        if len(damaged) != fault_count:
            raise GenerationError("损毁机数量与备用机不一致")
        if leader in damaged:
            raise GenerationError("领机不能是损毁边缘列")

    def _check_expected(key: str, actual: Sequence[str], label: str) -> None:
        expected = [uav_name(uav_num(u)) for u in (cfg.get(key) or [])]
        if expected and sorted(expected, key=uav_num) != sorted(actual, key=uav_num):
            raise GenerationError(
                "%s 与布局推导不符：expected=%s actual=%s"
                % (label, sorted(expected, key=uav_num),
                   sorted(actual, key=uav_num))
            )

    _check_expected("expected_spare_uavs", spares, "备用机")
    _check_expected("expected_faulted_uavs", damaged, "损毁机")
    participants = formation + spares + b_uavs
    if len(set(participants)) != len(participants):
        raise GenerationError("编队/备用/B 存在重复飞机")
    unused = [u for u in available if u not in participants]
    calibration = find_calibration(fleet, participants, replacements)
    shift_x = float(calibration["shift_x"])
    shift_y = float(calibration["shift_y"])
    ground = {
        u: (grid[u][0] + shift_x, grid[u][1] + shift_y) for u in participants
    }
    leader_x, leader_ground_y = ground[leader]
    leader_start_y = leader_ground_y + float(cfg["prepare_forward_m"])
    segment = float(cfg["segment_length_m"])
    step = float(cfg["coverage_lateral_step_m"])
    strips = int(cfg["coverage_strips"])
    b_ids = sorted((uid(u) for u in b_uavs), key=lambda i: int(i[1:]))
    if custom_roles:
        b_strike_ids = [uid(u) for u in strike_uavs]
        b_reserve_id = uid(reserve_uav)
        b_failed_id = uid(failed_uav)
    else:
        b_strike_uavs = b_uavs[:b_strike_count]
        b_strike_ids = sorted((uid(u) for u in b_strike_uavs), key=lambda i: int(i[1:]))
        b_reserve_id = uid(b_uavs[b_strike_count])
        b_failed_index = int(cfg.get("b_failed_index", 1))
        if not 0 <= b_failed_index < len(b_strike_ids):
            raise GenerationError("b_failed_index 超出打击机范围")
        b_failed_id = b_strike_ids[b_failed_index]
    b_final_ids = sorted(
        [i for i in b_strike_ids if i != b_failed_id] + [b_reserve_id],
        key=lambda i: int(i[1:]),
    )
    b_targets_cfg = cfg["b_targets"][: len(b_strike_ids)]
    if len(b_targets_cfg) < len(b_strike_ids):
        raise GenerationError(
            "B 目标数量 %d 少于打击机数量 %d"
            % (len(b_targets_cfg), len(b_strike_ids))
        )
    if custom_roles:
        # 配置里的目标 x 就是任务坐标；target_origin_x 是网格坐标，
        # 需加上 shift_x 才能与 ground/leader 同框。
        left_leader_x = float(block["target_origin_x"]) + shift_x
    else:
        left_leader_x = (
            block_spec(fleet, "left", replacements)["leader_xy"][0] + shift_x
        )
    target_dx = [float(item["x"]) - left_leader_x for item in b_targets_cfg]
    b_targets = {
        task_id: pose(
            leader_x + dx,
            float(item["y"]) + shift_y,
            float(cfg["b_strike_alt_m"]),
        )
        for task_id, item, dx in zip(b_strike_ids, b_targets_cfg, target_dx)
    }
    b_starts = {
        uid(uav): pose(
            ground[uav][0], ground[uav][1], float(cfg["b_start_alt_m"])
        )
        for uav in b_uavs
    }
    b_failed_hover = pose(
        b_targets[b_failed_id]["x"] + float(cfg["b_failed_hover_dx_m"]),
        b_targets[b_failed_id]["y"] + float(cfg["b_failed_hover_dy_m"]),
        float(cfg["b_strike_alt_m"]),
    )
    return {
        "available": list(available),
        "participants": participants,
        "unused": unused,
        "block": block,
        "formation_uavs": formation,
        "initial_ids": sorted((uid(u) for u in formation), key=lambda i: int(i[1:])),
        "faulted_ids": sorted((uid(u) for u in damaged), key=lambda i: int(i[1:])),
        "spare_ids": sorted((uid(u) for u in spares), key=lambda i: int(i[1:])),
        "spares_uavs": spares,
        "damaged_uavs": damaged,
        "leader_uav": leader,
        "leader_id": uid(leader),
        "leader_x": leader_x,
        "leader_start_y": leader_start_y,
        "coverage_step_m": step,
        "coverage_strips": strips,
        "b_uavs": b_uavs,
        "b_ids": b_ids,
        "b_strike_ids": b_strike_ids,
        "b_reserve_id": b_reserve_id,
        "b_failed_id": b_failed_id,
        "b_final_ids": b_final_ids,
        "b_targets": b_targets,
        "b_starts": b_starts,
        "b_failed_hover": b_failed_hover,
        "fault_count": fault_count,
        "shift_x": shift_x,
        "shift_y": shift_y,
        "calibration": calibration,
        "ground": ground,
        "grid": grid,
        "slot_replacements": replacements,
        "segment_length_m": segment,
    }


def build_plans(
    fleet: Dict[str, Any], mission: Dict[str, Any]
) -> Tuple[List[Dict[str, Any]], Dict[str, List[str]], Dict[str, Dict[str, Dict[str, float]]]]:
    cfg = fleet["mission"]
    block = mission["block"]
    leader_id = mission["leader_id"]
    leader_x = mission["leader_x"]
    leader_start_y = mission["leader_start_y"]
    segment = mission["segment_length_m"]
    coverage_alt = float(cfg["coverage_alt_m"])
    fault_alt = float(cfg["fault_exit_alt_m"])
    # Exit away from the B targets: left block exits left, right block exits right.
    fault_sign = float(
        block.get("fault_exit_sign", -1.0 if block["block"] == "left" else 1.0)
    )
    fault_dx = float(cfg["fault_exit_dx_m"]) * fault_sign
    fault_dy = float(cfg["fault_exit_dy_m"])
    fault_clearance_y = float(cfg.get("fault_exit_clearance_y_m", 0.0))
    spare_hold_alt = float(cfg["spare_hold_alt_m"])
    coverage_step = mission["coverage_step_m"]
    strips = mission["coverage_strips"]
    shift = mission["shift_y"]
    initial_ids = mission["initial_ids"]
    faulted_ids = mission["faulted_ids"]
    spare_ids = mission["spare_ids"]
    b_ids = mission["b_ids"]
    b_strike_ids = mission["b_strike_ids"]
    b_reserve_id = mission["b_reserve_id"]
    b_failed_id = mission["b_failed_id"]
    b_final_ids = mission["b_final_ids"]
    survivor_ids = [i for i in initial_ids if i not in faulted_ids]
    core_ids = sorted(survivor_ids + spare_ids, key=lambda i: int(i[1:]))
    offsets = block["offsets"]
    edge_offsets = block["edge_offsets"]
    leader_pose = pose(leader_x, leader_start_y, coverage_alt)
    c1_end = pose(leader_x, leader_start_y + segment, coverage_alt)
    # Boustrophedon: strip 1 is coverage1; coverage2 covers strips 2..N.
    c2_waypoints: List[Dict[str, float]] = [c1_end]
    strip_x = leader_x
    current_y = c1_end["y"]
    for _strip_index in range(2, strips + 1):
        strip_x += coverage_step
        c2_waypoints.append(pose(strip_x, current_y, coverage_alt))
        current_y = (
            c1_end["y"] + segment if abs(current_y - c1_end["y"]) < 1e-9 else c1_end["y"]
        )
        c2_waypoints.append(pose(strip_x, current_y, coverage_alt))
    c2_end = c2_waypoints[-1]
    final = pose(leader_x, leader_start_y, coverage_alt)
    initial_slots = {
        task_id: pose(
            leader_x + offsets[mission_uav][0],
            leader_start_y + offsets[mission_uav][1],
            coverage_alt,
        )
        for task_id, mission_uav in ((uid(u), u) for u in mission["formation_uavs"])
    }
    core_c1 = {
        task_id: pose(
            c1_end["x"] + offsets[mission_uav][0],
            c1_end["y"] + offsets[mission_uav][1],
            coverage_alt,
        )
        for task_id, mission_uav in ((uid(u), u) for u in mission["formation_uavs"])
    }
    core_c2 = {
        task_id: pose(
            c2_end["x"] + offsets[mission_uav][0],
            c2_end["y"] + offsets[mission_uav][1],
            coverage_alt,
        )
        for task_id, mission_uav in ((uid(u), u) for u in mission["formation_uavs"])
    }
    core_final = {
        task_id: pose(
            final["x"] + offsets[mission_uav][0],
            final["y"] + offsets[mission_uav][1],
            coverage_alt,
        )
        for task_id, mission_uav in ((uid(u), u) for u in mission["formation_uavs"])
    }
    survivor_ids = [i for i in initial_ids if i not in set(faulted_ids)]
    recovered_ids = sorted(survivor_ids + spare_ids, key=lambda i: int(i[1:]))
    recovered_offsets: Dict[str, Tuple[float, float]] = {}
    for task_id in survivor_ids:
        uav = next(u for u in mission["formation_uavs"] if uid(u) == task_id)
        recovered_offsets[task_id] = offsets[uav]
    for task_id in spare_ids:
        uav = next(u for u in mission["spares_uavs"] if uid(u) == task_id)
        row_index = mission["block"]["spares_all"].index(uav)
        recovered_offsets[task_id] = edge_offsets[
            mission["block"]["edge_all"][row_index]
        ]
    c1_recovered = {
        task_id: pose(
            c1_end["x"] + recovered_offsets[task_id][0],
            c1_end["y"] + recovered_offsets[task_id][1],
            coverage_alt,
        )
        for task_id in recovered_ids
    }
    c2_recovered = {
        task_id: pose(
            c2_end["x"] + recovered_offsets[task_id][0],
            c2_end["y"] + recovered_offsets[task_id][1],
            coverage_alt,
        )
        for task_id in recovered_ids
    }
    final_recovered = {
        task_id: pose(
            final["x"] + recovered_offsets[task_id][0],
            final["y"] + recovered_offsets[task_id][1],
            coverage_alt,
        )
        for task_id in recovered_ids
    }
    # Spares take off and hold in place at 5m; they only fly after the fault.
    spare_starts: Dict[str, Dict[str, float]] = {}
    spare_slots: Dict[str, Dict[str, float]] = {}
    for uav in mission["spares_uavs"]:
        task_id = uid(uav)
        row_index = next(
            r for r, u in enumerate(block["spares_all"]) if u == uav
        )
        edge_uav = block["edge_all"][row_index]
        slot_offset = edge_offsets[edge_uav]
        spare_starts[task_id] = pose(
            mission["ground"][uav][0], mission["ground"][uav][1], spare_hold_alt
        )
        spare_slots[task_id] = pose(
            leader_x + slot_offset[0],
            leader_start_y + segment + slot_offset[1],
            coverage_alt,
        )
    fault_entries: Dict[str, Dict[str, float]] = {}
    fault_exits: Dict[str, Dict[str, float]] = {}
    fault_routes: Dict[str, List[Dict[str, float]]] = {}
    for uav in mission["damaged_uavs"]:
        task_id = uid(uav)
        row_index = next(r for r, u in enumerate(block["edge_all"]) if u == uav)
        slot_offset = edge_offsets[uav]
        entry = pose(
            leader_x + slot_offset[0],
            leader_start_y + segment + slot_offset[1],
            fault_alt,
        )
        fault_entries[task_id] = entry
        if fault_clearance_y > 0.0:
            # 先沿 +y 离开编队占用区，再横向退出，避免退出航迹穿过编队。
            mid = pose(entry["x"], entry["y"] + fault_clearance_y, fault_alt)
            exit_pose = pose(mid["x"] + fault_dx, mid["y"] + fault_dy, fault_alt)
            fault_routes[task_id] = [entry, mid, exit_pose]
        else:
            exit_pose = pose(
                entry["x"] + fault_dx, entry["y"] + fault_dy, fault_alt
            )
            fault_routes[task_id] = [entry, exit_pose]
        fault_exits[task_id] = exit_pose

    plans: List[Dict[str, Any]] = []
    plan_members: Dict[str, List[str]] = {}
    expected: Dict[str, Dict[str, Dict[str, float]]] = {}

    def add_plan(plan_id: str, assignments: Dict[str, Dict[str, Any]]) -> None:
        plans.append(
            {
                "plan_id": plan_id,
                "group_id": "GroupA",
                "completion_policy": "ALL",
                "robot_assignments": assignments,
            }
        )
        plan_members[plan_id] = sorted(assignments, key=lambda i: int(i[1:]))

    # Formation prepare: left/center columns first, right column second.
    left_ids = [
        uid(u)
        for u in mission["formation_uavs"]
        if block["offsets"][u][0] < 0
    ]
    center_ids = [
        uid(u)
        for u in mission["formation_uavs"]
        if block["offsets"][u][0] == 0
    ]
    right_ids = [
        uid(u)
        for u in mission["formation_uavs"]
        if block["offsets"][u][0] > 0
    ]
    prepare_1 = left_ids + center_ids
    prepare_2 = right_ids
    add_plan(
        "prepare-a-1",
        {
            task_id: {
                "robot_id": task_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": initial_slots[task_id]},
            }
            for task_id in prepare_1
        },
    )
    add_plan(
        "prepare-a-2",
        {
            task_id: {
                "robot_id": task_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": initial_slots[task_id]},
            }
            for task_id in prepare_2
        },
    )
    expected["prepare-a-1"] = {i: initial_slots[i] for i in prepare_1}
    expected["prepare-a-2"] = {i: initial_slots[i] for i in prepare_2}
    add_plan(
        "prepare-spares",
        {
            task_id: {
                "robot_id": task_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": spare_starts[task_id]},
            }
            for task_id in spare_ids
        },
    )
    expected["prepare-spares"] = {i: spare_starts[i] for i in spare_ids}
    add_plan(
        "prepare-b-hold",
        {
            task_id: {
                "robot_id": task_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": mission["b_starts"][task_id]},
            }
            for task_id in b_ids
        },
    )
    expected["prepare-b-hold"] = {i: mission["b_starts"][i] for i in b_ids}

    def coverage_plan(
        plan_id: str, members: Sequence[str], waypoints: Sequence[Dict[str, float]]
    ) -> None:
        assignments: Dict[str, Dict[str, Any]] = {
            leader_id: {
                "robot_id": leader_id,
                "command_type": "FOLLOW_ROUTE",
                "payload": {"waypoints": list(waypoints)},
            }
        }
        for task_id in members:
            if task_id == leader_id:
                continue
            assignments[task_id] = {
                "robot_id": task_id,
                "command_type": "FOLLOW_ROUTE",
                "payload": {"formation_follow": True},
            }
        add_plan(plan_id, assignments)

    coverage_plan("coverage-segment-1", initial_ids, [leader_pose, c1_end])
    expected["coverage-segment-1"] = {
        task_id: core_c1[task_id] for task_id in initial_ids
    }
    add_plan(
        "fault-exit",
        {
            task_id: {
                "robot_id": task_id,
                "command_type": "FAULT_EXIT",
                "payload": {"waypoints": fault_routes[task_id]},
            }
            for task_id in faulted_ids
        },
    )
    expected["fault-exit"] = {i: fault_exits[i] for i in faulted_ids}
    # All spares fly to their edge slots in one batch. The three parallel
    # transit paths stay about 2.7m apart at the closest point, which the
    # operator accepted for this short, same-direction move.
    add_plan(
        "recovery-spares",
        {
            task_id: {
                "robot_id": task_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": spare_slots[task_id]},
            }
            for task_id in spare_ids
        },
    )
    expected["recovery-spares"] = {i: spare_slots[i] for i in spare_ids}
    # Recovery observation: survivors hold their coverage1-end slots; spares occupy the edge slots.
    observed_recovered: Dict[str, Dict[str, Any]] = {}
    for task_id in survivor_ids:
        uav = next(u for u in mission["formation_uavs"] if uid(u) == task_id)
        observed_recovered[task_id] = {
            "robot_id": task_id,
            "command_type": "MOVE_TO",
            "payload": {"target_pose": core_c1[task_id]},
        }
    for task_id in spare_ids:
        observed_recovered[task_id] = {
            "robot_id": task_id,
            "command_type": "MOVE_TO",
            "payload": {"target_pose": spare_slots[task_id]},
        }
    add_plan("recovery-hold", observed_recovered)
    expected["recovery-hold"] = {
        **{i: core_c1[i] for i in survivor_ids},
        **{i: spare_slots[i] for i in spare_ids},
    }
    coverage_plan("coverage-segment-2", core_ids, c2_waypoints)
    expected["coverage-segment-2"] = {
        task_id: c2_recovered[task_id] for task_id in core_ids
    }
    add_plan(
        "mock-b-strike",
        {
            task_id: {
                "robot_id": task_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": mission["b_targets"][task_id]},
            }
            for task_id in b_strike_ids
        },
    )
    expected["mock-b-strike"] = {
        i: mission["b_targets"][i] for i in b_strike_ids
    }
    # One strike aircraft "fails" and returns to its start; the 4th takes its target.
    add_plan(
        "mock-b-fail",
        {
            b_failed_id: {
                "robot_id": b_failed_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": mission["b_failed_hover"]},
            }
        },
    )
    expected["mock-b-fail"] = {b_failed_id: mission["b_failed_hover"]}
    add_plan(
        "mock-b-reinforce",
        {
            b_reserve_id: {
                "robot_id": b_reserve_id,
                "command_type": "MOVE_TO",
                "payload": {"target_pose": mission["b_targets"][b_failed_id]},
            }
        },
    )
    expected["mock-b-reinforce"] = {
        b_reserve_id: mission["b_targets"][b_failed_id]
    }
    b_hold_assign: Dict[str, Dict[str, Any]] = {}
    for task_id in b_final_ids:
        target = (
            mission["b_targets"][task_id]
            if task_id in mission["b_targets"]
            else mission["b_targets"][b_failed_id]
        )
        b_hold_assign[task_id] = {
            "robot_id": task_id,
            "command_type": "MOVE_TO",
            "payload": {"target_pose": target},
        }
    b_hold_assign[b_failed_id] = {
        "robot_id": b_failed_id,
        "command_type": "MOVE_TO",
        "payload": {"target_pose": mission["b_failed_hover"]},
    }
    add_plan("mock-b-hold", b_hold_assign)
    expected["mock-b-hold"] = {
        task_id: assignment["payload"]["target_pose"]
        for task_id, assignment in b_hold_assign.items()
    }
    coverage_plan("return-a", core_ids, [c2_end, final])
    expected["return-a"] = {
        task_id: final_recovered[task_id] for task_id in core_ids
    }
    hold: Dict[str, Dict[str, Any]] = {}
    for task_id in core_ids:
        hold[task_id] = {
            "robot_id": task_id,
            "command_type": "MOVE_TO",
            "payload": {"target_pose": final_recovered[task_id]},
        }
    for task_id in faulted_ids:
        hold[task_id] = {
            "robot_id": task_id,
            "command_type": "MOVE_TO",
            "payload": {"target_pose": fault_exits[task_id]},
        }
    for task_id, assignment in b_hold_assign.items():
        hold[task_id] = {
            "robot_id": task_id,
            "command_type": "MOVE_TO",
            "payload": {"target_pose": assignment["payload"]["target_pose"]},
        }
    add_plan("hold-end", hold)
    expected["hold-end"] = {
        **{i: final_recovered[i] for i in core_ids},
        **{i: fault_exits[i] for i in faulted_ids},
        **{
            i: b_hold_assign[i]["payload"]["target_pose"]
            for i in b_hold_assign
        },
    }
    return plans, plan_members, expected


def build_package(
    fleet: Dict[str, Any], mission: Dict[str, Any]
) -> Dict[str, Any]:
    cfg = fleet["mission"]
    initial_ids = set(mission["initial_ids"])
    faulted_ids = set(mission["faulted_ids"])
    spare_ids = set(mission["spare_ids"])
    b_ids = set(mission["b_ids"])
    leader_id = mission["leader_id"]
    robots: List[Dict[str, Any]] = []
    for task_id in sorted(
        set(mission["initial_ids"]) | spare_ids | b_ids,
        key=lambda i: int(i[1:]),
    ):
        if task_id in b_ids:
            role, is_reserve = "MockStrike", False
            capabilities = ["MOVE_TO"]
            is_leader = False
        elif task_id in spare_ids:
            role, is_reserve = "ReconReserve", True
            capabilities = ["MOVE_TO", "FOLLOW_ROUTE"]
            is_leader = False
        else:
            role, is_reserve = "Recon", False
            capabilities = ["MOVE_TO", "FOLLOW_ROUTE"]
            if task_id in faulted_ids:
                capabilities.append("FAULT_EXIT")
            is_leader = task_id == leader_id
        robots.append(
            {
                "uav_id": task_id,
                "group_id": "GroupA",
                "role": role,
                "is_leader": is_leader,
                "is_reserve": is_reserve,
                "capabilities": capabilities,
            }
        )
    active_ids = sorted(initial_ids | b_ids, key=lambda i: int(i[1:]))
    reserve_ids = sorted(spare_ids, key=lambda i: int(i[1:]))
    all_ids = sorted(active_ids + reserve_ids, key=lambda i: int(i[1:]))
    bootstrap = {
        "schema_version": SCHEMA_VERSION,
        "mission_id": MISSION_ID,
        "group_id": "GroupA",
        "roster": {
            "group_id": "GroupA",
            "active_ids": active_ids,
            "reserve_ids": reserve_ids,
            "failed_ids": [],
            "inactive_ids": [],
        },
        "initial_states": [
            {"uav_id": task_id, "online": True, "available": True}
            for task_id in all_ids
        ],
        "blackboard": {"mission_armed": True},
        "planned_fault_count": len(mission["faulted_ids"]),
        "planned_recovery_count": len(mission["spare_ids"]),
    }
    targets = {
        "b-target-%d" % index: {
            "x": mission["b_targets"][task_id]["x"],
            "y": mission["b_targets"][task_id]["y"],
            "z": 0.0,
            "yaw": 0.0,
        }
        for index, task_id in enumerate(mission["b_strike_ids"], start=1)
    }
    world = {
        "schema_version": SCHEMA_VERSION,
        "coordinate_frame": "mission_enu",
        "target_config_version": TARGET_CONFIG_VERSION,
        "flight_area": {
            "min_x": float(fleet["field"]["min_x"]) + mission["shift_x"],
            "max_x": float(fleet["field"]["max_x"]) + mission["shift_x"],
            "min_y": float(fleet["field"]["min_y"]) + mission["shift_y"],
            "max_y": float(fleet["field"]["max_y"]) + mission["shift_y"],
            "safety_margin_m": float(fleet["field"]["safety_margin_m"]),
        },
        "targets": targets,
    }
    plans, plan_members, expected = build_plans(fleet, mission)
    offsets = mission["block"]["offsets"]
    edge_offsets = mission["block"]["edge_offsets"]
    block = mission["block"]
    spare_edge_offsets = {
        uid(uav): edge_offsets[
            block["edge_all"][block["spares_all"].index(uav)]
        ]
        for uav in mission["spares_uavs"]
    }
    formation_offsets = {
        "leader_id": leader_id,
        "mission_anchor": [0.0, 0.0, 0.0],
        "formations": {
            "recon_initial": {
                "participants": mission["initial_ids"],
                "offsets": {
                    task_id: [
                        float(offsets[uav][0]),
                        float(offsets[uav][1]),
                        0.0,
                    ]
                    for task_id, uav in (
                        (uid(u), u) for u in mission["formation_uavs"]
                    )
                },
            },
            "recon_recovered": {
                "participants": sorted(
                    [i for i in mission["initial_ids"] if i not in set(mission["faulted_ids"])]
                    + mission["spare_ids"],
                    key=lambda i: int(i[1:]),
                ),
                "offsets": {
                    **{
                        task_id: [
                            float(offsets[uav][0]),
                            float(offsets[uav][1]),
                            0.0,
                        ]
                        for task_id, uav in (
                            (uid(u), u)
                            for u in mission["formation_uavs"]
                            if uid(u) not in set(mission["faulted_ids"])
                        )
                    },
                    **{
                        task_id: [
                            float(spare_edge_offsets[task_id][0]),
                            float(spare_edge_offsets[task_id][1]),
                            0.0,
                        ]
                        for task_id in mission["spare_ids"]
                    },
                },
            },
        },
    }
    calibration = {
        "calibration_uavs": mission["calibration"]["compute"],
        "all_uavs": mission["calibration"]["all"],
        "sample_window_s": mission["calibration"]["sample_window_s"],
    }
    return {
        "robots": {"schema_version": SCHEMA_VERSION, "uavs": robots},
        "bootstrap": bootstrap,
        "world": world,
        "plans": {"schema_version": SCHEMA_VERSION, "plans": plans},
        "formation_offsets": formation_offsets,
        "calibration": calibration,
        "plan_members": plan_members,
        "expected_targets": expected,
    }


def build_phases(
    fleet: Dict[str, Any], package: Dict[str, Any]
) -> List[Dict[str, Any]]:
    return [
        {
            "name": "prepare",
            "timeout_s": 240.0,
            "requires": "",
            "publishes": "GROUPS_READY",
            "dwell_s": 0.0,
            "steps": [
                {"plan_id": "prepare-a-1", "observe_only": False},
                {"plan_id": "prepare-a-2", "observe_only": False},
                {"plan_id": "prepare-spares", "observe_only": True},
                {"plan_id": "prepare-b-hold", "observe_only": True},
            ],
        },
        {
            "name": "coverage1",
            "timeout_s": 240.0,
            "requires": "GROUPS_READY",
            "publishes": "",
            "dwell_s": 0.0,
            "steps": [{"plan_id": "coverage-segment-1", "observe_only": False}],
        },
        {
            "name": "fault",
            "timeout_s": 180.0,
            "requires": "",
            "publishes": "",
            "dwell_s": 0.0,
            "steps": [{"plan_id": "fault-exit", "observe_only": False}],
        },
        {
            "name": "recovery",
            "timeout_s": 180.0,
            "requires": "",
            "publishes": "",
            "dwell_s": 0.0,
            "steps": [
                {"plan_id": "recovery-spares", "observe_only": False},
                {"plan_id": "recovery-hold", "observe_only": True},
            ],
        },
        {
            "name": "coverage2",
            "timeout_s": 360.0,
            "requires": "",
            "publishes": "RECON_COMPLETE",
            "dwell_s": 0.0,
            "steps": [{"plan_id": "coverage-segment-2", "observe_only": False}],
        },
        {
            "name": "strike",
            "timeout_s": 180.0,
            "requires": "RECON_COMPLETE",
            "publishes": "",
            "dwell_s": 0.0,
            "steps": [{"plan_id": "mock-b-strike", "observe_only": False}],
        },
        {
            "name": "strike_fail",
            "timeout_s": 180.0,
            "requires": "",
            "publishes": "",
            "dwell_s": 0.0,
            "steps": [{"plan_id": "mock-b-fail", "observe_only": False}],
        },
        {
            "name": "strike_reinforce",
            "timeout_s": 180.0,
            "requires": "",
            "publishes": "",
            "dwell_s": 0.0,
            "steps": [{"plan_id": "mock-b-reinforce", "observe_only": False}],
        },
        {
            "name": "strike_hold",
            "timeout_s": 30.0,
            "requires": "",
            "publishes": "STRIKE_COMPLETE",
            "dwell_s": 3.0,
            "steps": [{"plan_id": "mock-b-hold", "observe_only": True}],
        },
        {
            "name": "return_a",
            "timeout_s": 300.0,
            "requires": "STRIKE_COMPLETE",
            "publishes": "",
            "dwell_s": 0.0,
            "steps": [{"plan_id": "return-a", "observe_only": False}],
        },
        {
            "name": "hold",
            "timeout_s": 30.0,
            "requires": "",
            "publishes": "",
            "dwell_s": 3.0,
            "steps": [{"plan_id": "hold-end", "observe_only": True}],
        },
    ]


def distance_2d(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sample_segment_2d(
    start: Tuple[float, float], end: Tuple[float, float], steps: int = 100
) -> List[Tuple[float, float]]:
    return [
        (
            start[0] + (end[0] - start[0]) * i / steps,
            start[1] + (end[1] - start[1]) * i / steps,
        )
        for i in range(steps + 1)
    ]


def validate_geometry(
    fleet: Dict[str, Any], mission: Dict[str, Any], package: Dict[str, Any]
) -> Dict[str, Any]:
    cfg = fleet["mission"]
    geo = fleet["geometry"]
    shift = mission["shift_y"]
    shift_x = mission["shift_x"]
    margin = float(fleet["field"]["safety_margin_m"])
    fx0 = float(fleet["field"]["min_x"]) + shift_x + margin
    fx1 = float(fleet["field"]["max_x"]) + shift_x - margin
    fy0 = float(fleet["field"]["min_y"]) + shift + margin
    fy1 = float(fleet["field"]["max_y"]) + shift - margin
    checks: Dict[str, Any] = {}

    def inside(name: str, x: float, y: float) -> None:
        if not (fx0 <= x <= fx1 and fy0 <= y <= fy1):
            raise GenerationError(
                "%s 越界：(%.2f, %.2f) 不在场界内缩 %.1fm 内 [%.1f..%.1f, %.1f..%.1f]"
                % (name, x, y, margin, fx0, fx1, fy0, fy1)
            )

    block = mission["block"]
    offsets = block["offsets"]
    initial_ids = mission["initial_ids"]
    faulted_ids = mission["faulted_ids"]
    spare_ids = mission["spare_ids"]
    b_ids = mission["b_ids"]
    formation = mission["formation_uavs"]
    slots = {
        uid(u): (
            mission["leader_x"] + offsets[u][0],
            mission["leader_start_y"] + offsets[u][1],
        )
        for u in formation
    }
    min_slot = min(
        distance_2d(a[1], b[1])
        for index, a in enumerate(slots.items())
        for b in list(slots.items())[index + 1:]
    )
    if min_slot < float(geo["min_pairwise_m"]) - 1e-9:
        raise GenerationError("编队槽位最小间距 %.2fm < %.2fm" % (min_slot, geo["min_pairwise_m"]))
    checks["min_slot_spacing_m"] = min_slot

    # Coverage envelope from the actual coverage1 + boustrophedon waypoints.
    c1_plan = next(
        p for p in package["plans"]["plans"] if p["plan_id"] == "coverage-segment-1"
    )
    c2_plan = next(
        p for p in package["plans"]["plans"] if p["plan_id"] == "coverage-segment-2"
    )
    c1_leader = c1_plan["robot_assignments"][mission["leader_id"]]["payload"]["waypoints"]
    c2_leader = c2_plan["robot_assignments"][mission["leader_id"]]["payload"]["waypoints"]
    survivor_ids = [i for i in initial_ids if i not in set(faulted_ids)]
    recovered_offsets: List[Tuple[float, float]] = []
    for task_id in survivor_ids:
        uav = next(u for u in formation if uid(u) == task_id)
        recovered_offsets.append(offsets[uav])
    for task_id in spare_ids:
        uav = next(u for u in mission["spares_uavs"] if uid(u) == task_id)
        row_index = block["spares_all"].index(uav)
        recovered_offsets.append(
            block["edge_offsets"][block["edge_all"][row_index]]
        )
    initial_offsets = [offsets[u] for u in formation]
    envelope: List[Tuple[float, float]] = []
    for wp in c1_leader:
        for dx, dy in initial_offsets:
            envelope.append((wp["x"] + dx, wp["y"] + dy))
    for wp in c2_leader:
        for dx, dy in recovered_offsets:
            envelope.append((wp["x"] + dx, wp["y"] + dy))
    for x, y in envelope:
        inside("覆盖包络", x, y)
    checks["coverage_envelope"] = {
        "min_x": min(p[0] for p in envelope),
        "max_x": max(p[0] for p in envelope),
        "min_y": min(p[1] for p in envelope),
        "max_y": max(p[1] for p in envelope),
    }
    scan = float(cfg["scan_band_m"]) / 2.0
    strip_rects: List[Tuple[float, float, float, float]] = []
    xs_initial = [dx for dx, _ in initial_offsets]
    x0 = c1_leader[0]["x"]
    y_start = min(wp["y"] for wp in c1_leader)
    y_end = max(wp["y"] for wp in c1_leader)
    strip_rects.append(
        (
            x0 + min(xs_initial) - scan,
            x0 + max(xs_initial) + scan,
            y_start - 5.0,
            y_end + 5.0,
        )
    )
    xs_rec = [dx for dx, _ in recovered_offsets]
    for index in range(1, len(c2_leader)):
        previous = c2_leader[index - 1]
        current = c2_leader[index]
        if abs(current["y"] - previous["y"]) < 1e-6:
            continue  # lateral move, not a scan strip
        strip_rects.append(
            (
                current["x"] + min(xs_rec) - scan,
                current["x"] + max(xs_rec) + scan,
                min(previous["y"], current["y"]) - 5.0,
                max(previous["y"], current["y"]) + 5.0,
            )
        )
    for task_id, slot in slots.items():
        inside("prepare %s" % task_id, slot[0], slot[1])
    for task_id, start in mission["b_starts"].items():
        inside("B start %s" % task_id, start["x"], start["y"])
    for task_id, target in mission["b_targets"].items():
        inside("B target %s" % task_id, target["x"], target["y"])
        if not any(
            rx0 <= target["x"] <= rx1 and ry0 <= target["y"] <= ry1
            for rx0, rx1, ry0, ry1 in strip_rects
        ):
            raise GenerationError(
                "B目标 %s (%.1f,%.1f) 不在任一侦查航带内（航带 %s）"
                % (task_id, target["x"], target["y"], strip_rects)
            )
    checks["b_targets_inside_coverage"] = True
    checks["coverage_scan_rects"] = strip_rects

    # B paths: 3 strike + failed aircraft return + reserve reinforcement.
    b_starts = mission["b_starts"]
    b_targets = mission["b_targets"]
    b_failed_id = mission["b_failed_id"]
    b_reserve_id = mission["b_reserve_id"]
    b_paths: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
    for task_id in mission["b_strike_ids"]:
        b_paths.append(
            (
                (b_starts[task_id]["x"], b_starts[task_id]["y"]),
                (b_targets[task_id]["x"], b_targets[task_id]["y"]),
            )
        )
    failed_hover = mission["b_failed_hover"]
    b_paths.append(
        (
            (b_targets[b_failed_id]["x"], b_targets[b_failed_id]["y"]),
            (failed_hover["x"], failed_hover["y"]),
        )
    )
    b_paths.append(
        (
            (b_starts[b_reserve_id]["x"], b_starts[b_reserve_id]["y"]),
            (b_targets[b_failed_id]["x"], b_targets[b_failed_id]["y"]),
        )
    )
    c2_end = c2_leader[-1]
    strike_positions = [
        (c2_end["x"] + dx, c2_end["y"] + dy) for dx, dy in recovered_offsets
    ]
    min_b_a_3d = math.inf
    b_transit_vertical = float(cfg["coverage_alt_m"]) - float(cfg["b_transit_alt_m"])
    for start, end in b_paths:
        for point in sample_segment_2d(start, end):
            for a_point in strike_positions:
                min_b_a_3d = min(
                    min_b_a_3d,
                    math.hypot(distance_2d(point, a_point), b_transit_vertical),
                )
    if min_b_a_3d < float(geo["min_b_a_strike_3d_m"]) - 1e-9:
        raise GenerationError(
            "B 路径对 A coverage2 终点最小三维距离 %.2fm < %.1fm"
            % (min_b_a_3d, geo["min_b_a_strike_3d_m"])
        )
    checks["min_b_a_strike_3d_m"] = min_b_a_3d
    vertical = float(cfg["coverage_alt_m"]) - float(cfg["b_strike_alt_m"])
    if vertical < float(geo["min_b_a_vertical_m"]) - 1e-9:
        raise GenerationError(
            "A 返航与 B 目标垂直间隔 %.2fm < %.1fm"
            % (vertical, geo["min_b_a_vertical_m"])
        )
    checks["b_a_vertical_m"] = vertical

    # Damaged exit vs all B paths.
    fault_exits = package["expected_targets"]["fault-exit"]
    min_fault_b = math.inf
    for start, end in b_paths:
        for point in sample_segment_2d(start, end):
            for exit_pose in fault_exits.values():
                min_fault_b = min(
                    min_fault_b,
                    distance_2d(point, (exit_pose["x"], exit_pose["y"])),
                )
    if min_fault_b < float(geo["min_fault_b_horizontal_m"]) - 1e-9:
        raise GenerationError(
            "故障退出点对 B 路径最小水平距离 %.2fm < %.1fm"
            % (min_fault_b, geo["min_fault_b_horizontal_m"])
        )
    checks["min_fault_b_horizontal_m"] = min_fault_b
    fault_vertical = float(cfg["coverage_alt_m"]) - float(cfg["fault_exit_alt_m"])
    if fault_vertical < float(geo["min_fault_formation_vertical_m"]) - 1e-9:
        raise GenerationError("故障机与编队垂直间隔 %.2fm 不足" % fault_vertical)
    checks["fault_formation_vertical_m"] = fault_vertical

    # Prepare second batch vs first batch slots; spare transit vs formation slots.
    plan_members = package["plan_members"]
    first_slots = [
        slots[task_id] for task_id in plan_members["prepare-a-1"]
    ]
    min_prepare = math.inf
    for task_id in plan_members["prepare-a-2"]:
        uav = next(u for u in formation if uid(u) == task_id)
        start = mission["ground"][uav]
        target = slots[task_id]
        for point in sample_segment_2d(start, target):
            for first in first_slots:
                min_prepare = min(min_prepare, distance_2d(point, first))
    if min_prepare < float(geo["min_prepare_batch_clearance_m"]) - 1e-9:
        raise GenerationError(
            "prepare 第二批/备用机路径距第一批机位 %.2fm < %.1fm"
            % (min_prepare, geo["min_prepare_batch_clearance_m"])
        )
    checks["min_prepare_batch_clearance_m"] = min_prepare

    # MOVE_TO displacement.
    min_move = math.inf
    for plan_id in (
        "prepare-a-1",
        "prepare-a-2",
        "recovery-spares",
        "mock-b-strike",
        "mock-b-fail",
        "mock-b-reinforce",
    ):
        for task_id in plan_members[plan_id]:
            if plan_id == "recovery-spares":
                uav = next(
                    u for u in mission["spares_uavs"] if uid(u) == task_id
                )
                start = mission["ground"][uav]
            elif plan_id == "mock-b-fail":
                failed_target = mission["b_targets"][task_id]
                start = (failed_target["x"], failed_target["y"])
            elif plan_id in ("mock-b-strike", "mock-b-reinforce"):
                start = (
                    mission["b_starts"][task_id]["x"],
                    mission["b_starts"][task_id]["y"],
                )
            else:
                uav = next(
                    u for u in mission["formation_uavs"] if uid(u) == task_id
                )
                start = mission["ground"][uav]
            target_pose = next(
                plan["robot_assignments"][task_id]["payload"]["target_pose"]
                for plan in package["plans"]["plans"]
                if plan["plan_id"] == plan_id
            )
            displacement = distance_2d(start, (target_pose["x"], target_pose["y"]))
            min_move = min(min_move, displacement)
            if displacement < float(geo["min_move_to_displacement_m"]) - 1e-9:
                raise GenerationError(
                    "%s/%s 水平位移 %.3fm < %.2fm"
                    % (plan_id, task_id, displacement, geo["min_move_to_displacement_m"])
                )
    checks["min_move_to_displacement_m"] = min_move

    # Formation participants match the plans exactly.
    formations = package["formation_offsets"]["formations"]
    for plan_id, name in (
        ("coverage-segment-1", "recon_initial"),
        ("coverage-segment-2", "recon_recovered"),
        ("return-a", "recon_recovered"),
    ):
        if set(plan_members[plan_id]) != set(formations[name]["participants"]):
            raise GenerationError("%s 与 %s participants 不一致" % (plan_id, name))
    if mission["leader_id"] not in formations["recon_initial"]["participants"]:
        raise GenerationError("leader 不在初始编队内")
    if mission["leader_id"] not in formations["recon_recovered"]["participants"]:
        raise GenerationError("leader 不在恢复编队内")
    if mission["leader_id"] in set(faulted_ids):
        raise GenerationError("leader 不能是故障机")
    if set(faulted_ids) & set(spare_ids):
        raise GenerationError("损毁机与备用机重叠")

    # Calibration symmetry.
    compute = mission["calibration"]["compute"]
    xs = [mission["grid"][u][0] for u in compute]
    ys = [mission["grid"][u][1] for u in compute]
    center_x = float(mission["calibration"].get("center_x", 0.0))
    if abs(statistics.median(xs) - center_x) > 1e-9:
        raise GenerationError(
            "标定计算集 x 中位数不是 %.1f：%s" % (center_x, xs)
        )
    if len(set(ys)) != 1:
        raise GenerationError("标定计算集跨排：%s" % ys)
    checks["calibration_median_x_m"] = center_x
    checks["calibration_row_y_m"] = ys[0]
    checks["shift_x_m"] = mission["shift_x"]
    checks["shift_y_m"] = shift
    return checks


def scheduler_check(
    fleet: Dict[str, Any], mission: Dict[str, Any], package: Dict[str, Any]
) -> List[Dict[str, Any]]:
    cfg = fleet["mission"]
    sched = fleet["scheduler"]
    params = ScheduleParams(
        horizontal_speed_mps=float(sched["horizontal_speed_mps"]),
        geometric_distance_m=float(sched["geometric_distance_m"]),
        safety_margin_m=float(sched["safety_margin_m"]),
        time_epsilon_s=float(sched["time_epsilon_s"]),
        max_start_delay_s=float(sched["max_start_delay_s"]),
        max_interval_combinations=int(sched["max_interval_combinations"]),
        max_compute_time_s=float(sched["max_compute_time_s"]),
    )
    params.validate()
    plan_members = package["plan_members"]
    expected = package["expected_targets"]
    batches: List[Tuple[str, List[UavSegment], float]] = []
    for plan_id, timeout in (
        ("prepare-a-1", 240.0),
        ("prepare-a-2", 240.0),
        ("recovery-spares", 180.0),
        ("mock-b-strike", 180.0),
        ("mock-b-fail", 180.0),
        ("mock-b-reinforce", 180.0),
    ):
        segments = []
        for task_id in plan_members[plan_id]:
            if plan_id == "recovery-spares":
                uav = next(
                    u for u in mission["spares_uavs"] if uid(u) == task_id
                )
                start = mission["ground"][uav]
            elif plan_id == "mock-b-fail":
                failed_target = expected["mock-b-strike"][task_id]
                start = (failed_target["x"], failed_target["y"])
            elif plan_id in ("mock-b-strike", "mock-b-reinforce"):
                start = (
                    mission["b_starts"][task_id]["x"],
                    mission["b_starts"][task_id]["y"],
                )
            else:
                uav = next(
                    u for u in mission["formation_uavs"] if uid(u) == task_id
                )
                start = mission["ground"][uav]
            target = expected[plan_id][task_id]
            segments.append(
                UavSegment(task_id, start, (target["x"], target["y"]))
            )
        batches.append((plan_id, segments, timeout))
    fault_segments = []
    fault_sign = float(
        mission["block"].get(
            "fault_exit_sign",
            -1.0 if mission["block"]["block"] == "left" else 1.0,
        )
    )
    fault_dx_signed = float(cfg["fault_exit_dx_m"]) * fault_sign
    for task_id in mission["faulted_ids"]:
        exit_pose = expected["fault-exit"][task_id]
        start = (
            exit_pose["x"] - fault_dx_signed,
            exit_pose["y"] - float(cfg["fault_exit_dy_m"]),
        )
        end = (exit_pose["x"], exit_pose["y"])
        fault_segments.append(UavSegment(task_id, start, end))
    batches.append(("fault-exit", fault_segments, 180.0))
    results: List[Dict[str, Any]] = []
    for plan_id, segments, timeout_s in batches:
        result = compute_schedule(segments, params)
        if not result.success:
            raise GenerationError(
                "%s 离线排程失败：%s %s（fail=%s）"
                % (plan_id, result.schedule_status, result.failure_reason, result.fail_ids)
            )
        total = (
            result.max_tau_s()
            + result.max_horizontal_time_s
            + float(sched["timeout_slack_s"])
        )
        if total > timeout_s:
            raise GenerationError(
                "%s 排程预算 %.1fs 超过阶段超时 %.1fs" % (plan_id, total, timeout_s)
            )
        results.append(
            {
                "plan_id": plan_id,
                "schedule_status": result.schedule_status,
                "max_tau_s": result.max_tau_s(),
                "max_horizontal_time_s": result.max_horizontal_time_s,
                "budget_s": total,
                "timeout_s": timeout_s,
                "non_global_optimal": result.non_global_optimal,
            }
        )
    return results


def build_onboard_commands(mission: Dict[str, Any]) -> str:
    participants = mission["participants"]
    lines = [
        "#!/usr/bin/env bash",
        "# 生成器输出；只打印逐机启动命令，不执行。",
        "# participants: %s" % " ".join(participants),
    ]
    b_ids = set(mission["b_ids"])
    for uav in participants:
        own = uav_num(uav)
        intents, poses = [], []
        for peer in participants:
            number = uav_num(peer)
            if number == own:
                continue
            intents.append("/UAV%d/trajectory_intent" % number)
            poses.append("/UAV%d/swarm_pose" % number)
        args = [
            "neighbor_intents:=%s" % ",".join(intents),
            "neighbor_odom_topics:=%s" % ",".join(poses),
            "uav_task_timeout_s:=300",
            "bundle_transport:=udp",
        ]
        if uid(uav) in b_ids:
            args.append("layer_move_to:=8.0")
        lines.append(
            "bash src/tcp_to_ros/tools/onboard_ctl.sh start %d -- %s"
            % (own, " ".join(args))
        )
    return "\n".join(lines) + "\n"


def build_flow_env(mission: Dict[str, Any]) -> str:
    numbers = " ".join(str(uav_num(u)) for u in mission["participants"])
    return (
        "#!/usr/bin/env bash\n"
        "# 生成器输出，供 verification/full_fleet_joint/flow.sh source。\n"
        "export AIR_FLOW_UAVS=%r\n"
        "export AIR_FLOW_LEADER=%d\n"
        "export AIR_FLOW_DIR=full_fleet_joint\n"
        % (numbers, uav_num(mission["leader_uav"]))
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_yaml(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")


def build_spec(
    fleet: Dict[str, Any],
    fleet_path: Path,
    mission: Dict[str, Any],
    package: Dict[str, Any],
    checks: Dict[str, Any],
    scheduler_results: List[Dict[str, Any]],
) -> Dict[str, Any]:
    phases = build_phases(fleet, package)
    initial_ids = mission["initial_ids"]
    faulted_ids = mission["faulted_ids"]
    spare_ids = mission["spare_ids"]
    core_ids = sorted(
        [i for i in initial_ids if i not in set(faulted_ids)] + spare_ids,
        key=lambda i: int(i[1:]),
    )
    for phase in phases:
        for step in phase["steps"]:
            if step["plan_id"] not in package["plan_members"]:
                raise GenerationError("阶段引用未知 plan：%s" % step["plan_id"])
    return {
        "schema_version": SCHEMA_VERSION,
        "mission_id": MISSION_ID,
        "fleet_file": str(Path(fleet_path).resolve()),
        "available_uavs": mission["participants"],
        "unused_uavs": mission["unused"],
        "formation_block": mission["block"]["block"],
        "slot_replacements": mission["slot_replacements"],
        "ground_positions": {
            uid(uav): pose(x, y, 0.0)
            for uav, (x, y) in mission["ground"].items()
        },
        "initial_ids": initial_ids,
        "core_ids": core_ids,
        "faulted_ids": faulted_ids,
        "spare_ids": spare_ids,
        "survivor_ids": sorted(
            [i for i in initial_ids if i not in set(faulted_ids)],
            key=lambda i: int(i[1:]),
        ),
        "leader_id": mission["leader_id"],
        "b_uavs": mission["b_uavs"],
        "b_ids": mission["b_ids"],
        "b_strike_ids": mission["b_strike_ids"],
        "b_reserve_id": mission["b_reserve_id"],
        "b_failed_id": mission["b_failed_id"],
        "b_final_ids": mission["b_final_ids"],
        "coverage_strips": mission["coverage_strips"],
        "coverage_step_m": mission["coverage_step_m"],
        "all_ids": sorted(
            set(initial_ids) | set(spare_ids) | set(mission["b_ids"]),
            key=lambda i: int(i[1:]),
        ),
        "fault_count": mission["fault_count"],
        "b_targets": mission["b_targets"],
        "b_failed_hover": mission["b_failed_hover"],
        "phases": phases,
        "plan_members": package["plan_members"],
        "expected_targets": package["expected_targets"],
        "calibration": mission["calibration"],
        "geometry_checks": checks,
        "scheduler": scheduler_results,
        "formation_offsets": package["formation_offsets"],
    }


def generate(
    fleet_path: Path = DEFAULT_FLEET,
    available: Sequence[str] = (),
    out_dir: Path = DEFAULT_OUT,
    leader: Optional[str] = None,
    b_uavs: Optional[Sequence[str]] = None,
    replacements: Optional[Dict[str, str]] = None,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    fleet = load_fleet(Path(fleet_path))
    available_uavs = parse_uavs(available)
    if not available_uavs:
        raise GenerationError("--available 不能为空")
    mission = assign_mission(
        fleet, available_uavs, leader, b_uavs, replacements
    )
    package = build_package(fleet, mission)
    checks = validate_geometry(fleet, mission, package)
    scheduler_results = scheduler_check(fleet, mission, package)
    spec = build_spec(
        fleet, Path(fleet_path), mission, package, checks, scheduler_results
    )
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    package_dir = out / "package"
    config_dir = out / "config"
    write_json(package_dir / "robots.json", package["robots"])
    write_json(package_dir / "bootstrap.json", package["bootstrap"])
    write_json(package_dir / "world.json", package["world"])
    write_json(package_dir / "plans" / "plans.json", package["plans"])
    write_yaml(config_dir / "formation_offsets.yaml", package["formation_offsets"])
    write_yaml(config_dir / "calibration.yaml", package["calibration"])
    write_json(out / "mission_spec.json", spec)
    (out / "onboard_commands.sh").write_text(
        build_onboard_commands(mission), encoding="utf-8"
    )
    (out / "flow_env.sh").write_text(build_flow_env(mission), encoding="utf-8")
    files = [
        package_dir / "robots.json",
        package_dir / "bootstrap.json",
        package_dir / "world.json",
        package_dir / "plans" / "plans.json",
        config_dir / "formation_offsets.yaml",
        config_dir / "calibration.yaml",
        out / "mission_spec.json",
        out / "onboard_commands.sh",
        out / "flow_env.sh",
    ]
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "fleet_file": str(Path(fleet_path).resolve()),
        "fleet_sha256": sha256(Path(fleet_path)),
        "available_input": mission["available"],
        "participants": mission["participants"],
        "unused": mission["unused"],
        "formation_block": mission["block"]["block"],
        "slot_replacements": mission["slot_replacements"],
        "initial": mission["initial_ids"],
        "spares": mission["spare_ids"],
        "faulted": mission["faulted_ids"],
        "b_uavs": mission["b_uavs"],
        "b_strike_ids": mission["b_strike_ids"],
        "b_reserve_id": mission["b_reserve_id"],
        "b_failed_id": mission["b_failed_id"],
        "b_final_ids": mission["b_final_ids"],
        "b_failed_hover": mission["b_failed_hover"],
        "coverage_strips": mission["coverage_strips"],
        "coverage_step_m": mission["coverage_step_m"],
        "leader": mission["leader_uav"],
        "calibration": mission["calibration"]["compute"],
        "shift_x": mission["shift_x"],
        "shift_y": mission["shift_y"],
        "geometry_checks": checks,
        "scheduler": scheduler_results,
        "file_sha256": {str(path.relative_to(out)): sha256(path) for path in files},
    }
    write_json(out / "manifest.json", manifest)
    validate_package_files(out)
    return spec, manifest


def validate_package_files(out_dir: Path) -> None:
    from uav_bt_runtime.config import load_json_model, load_mission_context
    from uav_bt_runtime.package import WorldConfig

    package_dir = Path(out_dir) / "package"
    context = load_mission_context(
        package_dir / "robots.json",
        package_dir / "bootstrap.json",
        package_dir / "plans" / "plans.json",
    )
    load_json_model(package_dir / "world.json", WorldConfig)
    if len([spec for spec in context.uav_specs.values() if spec.is_leader]) != 1:
        raise GenerationError("robots.json 必须恰好一个 leader")
    if len(context.roster.active_ids) + len(context.roster.reserve_ids) != len(
        context.uav_specs
    ):
        raise GenerationError("active+reserve 必须覆盖全部参与机")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--available", nargs="+", required=True)
    parser.add_argument("--fleet", default=str(DEFAULT_FLEET))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--leader", default=None)
    parser.add_argument("--b", default=None, help="显式指定 B 组：UAV5,UAV10,...")
    parser.add_argument(
        "--replace",
        action="append",
        default=[],
        metavar="SLOT=ACTUAL",
        help="地面槽位替换，如 UAV13=UAV16（UAV16 摆到 UAV13 的机位）",
    )
    args = parser.parse_args(argv)
    b_uavs = parse_uavs([args.b]) if args.b else None
    try:
        replacements = parse_replacements(args.replace)
        spec, manifest = generate(
            fleet_path=Path(args.fleet),
            available=args.available,
            out_dir=Path(args.out),
            leader=args.leader,
            b_uavs=b_uavs,
            replacements=replacements,
        )
    except GenerationError as error:
        print("生成失败：%s" % error, file=sys.stderr)
        return 2
    print("生成完成：%s" % Path(args.out).resolve())
    print("  participants: %s" % " ".join(spec["available_uavs"]))
    if spec["slot_replacements"]:
        print("  slot replacements: %s" % ", ".join(
            "%s->%s" % item for item in spec["slot_replacements"].items()
        ))
    print("  initial %d: %s" % (len(spec["initial_ids"]), " ".join(spec["initial_ids"])))
    print("  leader: %s" % spec["leader_id"])
    print("  faulted edge: %s" % " ".join(spec["faulted_ids"]))
    print("  spares: %s" % " ".join(spec["spare_ids"]))
    print("  B: %s" % " ".join(spec["b_ids"]))
    print(
        "  block: %s  shift_x: %.1fm  shift_y: %.1fm"
        % (spec["formation_block"], manifest["shift_x"], manifest["shift_y"])
    )
    print("  scheduler: %s" % ", ".join(
        "%s=%s t=%.2fs" % (row["plan_id"], row["schedule_status"], row["max_tau_s"])
        for row in spec["scheduler"]
    ))
    return 0


if __name__ == "__main__":
    sys.exit(main())
