#!/usr/bin/env python3
"""14–16 架 A/B 联合任务驱动。

任务成员、阶段、期望终点全部来自 generated/mission_spec.json；默认只打印计划，
--execute 才连接 GCS。不 arm、不切模式、不自动降落。

复用 four_uav_recon 的批次构造/JSONL，与 runtime 的 TCP、汇总和
SimulateDamageHandler；本版不使用 reserve/单机接替。
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional, Sequence, Tuple

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verification.four_uav_recon.driver import JsonlLogger, build_batch  # noqa: E402
from uav_bt_runtime.actions import (  # noqa: E402
    RecoverReconGroupHandler,
    SimulateDamageHandler,
)
from uav_bt_runtime.clock import SystemClock  # noqa: E402
from uav_bt_runtime.config import load_json_model, load_mission_context  # noqa: E402
from uav_bt_runtime.package import WorldConfig  # noqa: E402
from uav_bt_runtime.tcp_transport import TcpCommandTransport  # noqa: E402
from uav_bt_runtime.tracking import BatchResult, CommandBatchTracker  # noqa: E402


def default_spec_path() -> Path:
    return HERE / "generated" / "mission_spec.json"


def default_log_path() -> Path:
    stamp = time.strftime("%Y-%m-%d_%H%M%S")
    return ROOT / "src" / "uav_log" / ("%s_full-fleet-joint" % stamp) / "driver.jsonl"


@dataclass(frozen=True)
class Step:
    plan_id: str
    observe_only: bool


@dataclass(frozen=True)
class Phase:
    name: str
    timeout_s: float
    requires: str
    publishes: str
    dwell_s: float
    steps: Tuple[Step, ...]


def load_spec(path: Path) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        spec = json.load(handle)
    if spec.get("schema_version") != "1.0":
        raise ValueError("mission_spec schema_version 必须是 1.0")
    return spec


def build_phases(spec: Dict[str, Any]) -> Tuple[Phase, ...]:
    phases: List[Phase] = []
    for item in spec["phases"]:
        steps = tuple(
            Step(step["plan_id"], bool(step["observe_only"]))
            for step in item["steps"]
        )
        phases.append(
            Phase(
                name=item["name"],
                timeout_s=float(item["timeout_s"]),
                requires=str(item.get("requires", "")),
                publishes=str(item.get("publishes", "")),
                dwell_s=float(item.get("dwell_s", 0.0)),
                steps=steps,
            )
        )
    if not phases:
        raise ValueError("mission_spec 没有阶段")
    return tuple(phases)


def load_package(spec_path: Path) -> SimpleNamespace:
    spec_path = Path(spec_path)
    spec = load_spec(spec_path)
    package_dir = spec_path.parent / "package"
    context = load_mission_context(
        package_dir / "robots.json",
        package_dir / "bootstrap.json",
        package_dir / "plans" / "plans.json",
    )
    world = load_json_model(package_dir / "world.json", WorldConfig)
    expected_ids = set(spec["all_ids"])
    if set(context.uav_specs) != expected_ids:
        raise ValueError("robots.json 成员与 mission_spec 不一致")
    expected_active = set(spec["initial_ids"]) | set(spec["b_ids"])
    expected_reserve = set(spec["spare_ids"])
    if set(context.roster.active_ids) != expected_active:
        raise ValueError("roster active 必须是初始 9 架 + B 组")
    if set(context.roster.reserve_ids) != expected_reserve:
        raise ValueError("roster reserve 必须是备用机")
    if context.roster.failed_ids or context.roster.inactive_ids:
        raise ValueError("初始名册不允许 failed/inactive")
    leaders = [uav for uav, item in context.uav_specs.items() if item.is_leader]
    if leaders != [spec["leader_id"]]:
        raise ValueError("leader 必须是 %s" % spec["leader_id"])
    if context.mission_id != spec["mission_id"]:
        raise ValueError("bootstrap mission_id 与 mission_spec 不一致")
    b_ids = set(spec["b_ids"])
    strike_ids = set(spec["b_strike_ids"])
    if not strike_ids or not strike_ids < b_ids:
        raise ValueError("b_strike_ids 必须非空且为 b_ids 的真子集")
    if (spec["b_reserve_id"] not in b_ids
            or spec["b_reserve_id"] in strike_ids):
        raise ValueError("b_reserve_id 必须在 b_ids 且不在 b_strike_ids")
    if spec["b_failed_id"] not in strike_ids:
        raise ValueError("b_failed_id 必须在 b_strike_ids")
    if len(spec["b_targets"]) != len(strike_ids):
        raise ValueError("b_targets 数量必须等于 b_strike_ids 数量")
    expected_final = (strike_ids - {spec["b_failed_id"]}) | {spec["b_reserve_id"]}
    if set(spec["b_final_ids"]) != expected_final:
        raise ValueError("b_final_ids 与 strike/reserve 不一致")
    if b_ids & set(spec["spare_ids"]):
        raise ValueError("B 组与备用机不得重叠")
    hold_end = set(spec["expected_targets"].get("hold-end", {}))
    if hold_end != expected_ids:
        raise ValueError("hold-end expected_targets 必须覆盖全部参与机")
    for phase in build_phases(spec):
        for step in phase.steps:
            if step.plan_id not in context.plans:
                raise ValueError("缺少 plan %s" % step.plan_id)
            members = set(context.plans[step.plan_id].robot_assignments)
            if members != set(spec["plan_members"][step.plan_id]):
                raise ValueError("%s 成员与 mission_spec 不一致" % step.plan_id)
            if step.plan_id not in spec["expected_targets"]:
                raise ValueError("缺少 %s 的 expected_targets" % step.plan_id)
    return SimpleNamespace(context=context, world=world, spec=spec)


def update_roster(context, phase: Phase, spec: Dict[str, Any], logger: JsonlLogger) -> None:
    """Fault removes the edge column; recovery activates the adjacent spares."""
    services = SimpleNamespace(mission=context)
    if phase.name == "fault":
        handler = SimulateDamageHandler("fault", phase.steps[0].plan_id, phase.timeout_s)
        handler.selected_ids = list(spec["faulted_ids"])
        handler.on_success(services)
    elif phase.name == "recovery":
        handler = RecoverReconGroupHandler(
            "recovery", phase.steps[0].plan_id, phase.timeout_s
        )
        selected = handler.select_ids(
            services, context.plans[phase.steps[0].plan_id]
        )
        expected_active = set(spec["core_ids"]) | set(spec["b_ids"])
        if set(selected) != expected_active:
            raise RuntimeError(
                "接替前名册不符合预期：selected=%s expected=%s"
                % (sorted(selected), sorted(expected_active))
            )
        handler.on_success(services)
        if (
            set(context.roster.active_ids) != expected_active
            or context.roster.reserve_ids
            or set(context.roster.failed_ids) != set(spec["faulted_ids"])
        ):
            raise RuntimeError(
                "接替后名册不符合预期：active=%s reserve=%s failed=%s"
                % (
                    sorted(context.roster.active_ids),
                    context.roster.reserve_ids,
                    sorted(context.roster.failed_ids),
                )
            )


def _console(message: str) -> None:
    print("%s %s" % (time.strftime("%H:%M:%S"), message), flush=True)


def wait_phase(transport, batch, phase: Phase, plan_id: str, guard, logger) -> str:
    tracker = CommandBatchTracker(batch)
    deadline = time.monotonic() + phase.timeout_s
    reported = set()
    while True:
        if phase.name == "return_a":
            guard.check_expected("mock-b-hold")
        # 先落盘批次反馈，再查任务失败锁存；否则 GCS 的 CONFIRM_FAILED 详情会被
        # 机载 TASK_ABORTED 抢先打断，日志里只剩 “group acceptance failed”。
        for feedback in transport.poll_feedback():
            tracker.apply(feedback)
            logger.write(
                {
                    "event": "command_feedback",
                    "phase": phase.name,
                    "plan_id": plan_id,
                    "robot_id": feedback.robot_id,
                    "command_id": feedback.command_id,
                    "status": feedback.status.value,
                    "error_code": feedback.error_code,
                    "error_message": feedback.error_message,
                }
            )
            key = (feedback.robot_id, feedback.status.value)
            if key not in reported:
                reported.add(key)
                _console("[%s/%s] %s %s"
                         % (phase.name, plan_id, feedback.robot_id,
                            feedback.status.value))
        if tracker.result is BatchResult.FAILURE:
            _console("[%s/%s] FAILURE" % (phase.name, plan_id))
            return "FAILURE"
        if tracker.result is BatchResult.SUCCESS:
            observed = guard.check_expected(plan_id)
            logger.write(
                {
                    "event": "phase_observation",
                    "phase": phase.name,
                    "plan_id": plan_id,
                    "positions_enu": observed,
                    "time": time.time(),
                }
            )
            _console("[%s/%s] SUCCESS" % (phase.name, plan_id))
            return "SUCCESS"
        guard.check()
        if time.monotonic() >= deadline:
            _console("[%s/%s] TIMEOUT" % (phase.name, plan_id))
            return "TIMEOUT"
        time.sleep(0.05)


def confirm_prompt(phase: Phase, spec: Dict[str, Any]) -> Optional[str]:
    faulted = " ".join(spec["faulted_ids"])
    spares = " ".join(spec["spare_ids"])
    b_ids = " ".join(spec["b_ids"])
    if phase.name == "fault":
        return "确认第一段覆盖结束，边缘列 %s 下降至 8m 并向后方退出。" % faulted
    if phase.name == "recovery":
        return "确认备用机 %s 已到补充点，允许向左补位恢复 3×3。" % spares
    if phase.name == "strike":
        return "确认侦查完成，允许 B 组 %s 前往侦查区内目标上方 5m。" % b_ids
    return None


def run_sequence(
    package,
    transport,
    guard,
    logger: JsonlLogger,
    phases: Sequence[Phase],
    all_phase_count: int,
    confirm=None,
) -> int:
    context = package.context
    spec = package.spec
    clock = SystemClock()
    run_id = uuid.uuid4().hex
    events = set()
    logger.write(
        {
            "event": "mission_start",
            "mission_id": context.mission_id,
            "run_id": run_id,
            "participants": list(spec["all_ids"]),
            "initial_ids": list(spec["initial_ids"]),
            "b_ids": list(spec["b_ids"]),
            "b_strike_ids": list(spec["b_strike_ids"]),
            "b_reserve_id": spec["b_reserve_id"],
            "b_failed_id": spec["b_failed_id"],
            "b_final_ids": list(spec["b_final_ids"]),
            "core_ids": list(spec["core_ids"]),
            "faulted_ids": list(spec["faulted_ids"]),
            "spare_ids": list(spec["spare_ids"]),
            "survivor_ids": list(spec["survivor_ids"]),
            "leader_id": spec["leader_id"],
            "coordination": "local_mock_b",
            "phases": [phase.name for phase in phases],
        }
    )
    _console(
        "[driver] mission %s start: participants=%d initial=%s b=%s"
        % (context.mission_id, len(spec["all_ids"]),
           ",".join(spec["initial_ids"]), ",".join(spec["b_ids"]))
    )
    try:
        if confirm:
            confirm(
                "start",
                "确认通信检查通过、机位与净空已核对；全部参与机已人工起飞并稳定在约5m。",
            )
        guard.check()
        for phase in phases:
            if phase.requires and phase.requires not in events:
                raise RuntimeError("缺少阶段事件 %s" % phase.requires)
            prompt = confirm_prompt(phase, spec)
            if confirm and prompt:
                confirm(phase.name, prompt)
            guard.check()
            if phase.name == "return_a":
                # 下发返航前再核对 B 组仍在目标上方 5m；不能等返航批次发出后再发现漂移。
                guard.check_expected("mock-b-hold")
            _console("[%s] phase start: %s"
                     % (phase.name, ", ".join(step.plan_id for step in phase.steps)))
            for step in phase.steps:
                plan = context.plans[step.plan_id]
                logger.write(
                    {
                        "event": "phase_start",
                        "phase": phase.name,
                        "plan_id": step.plan_id,
                        "robot_ids": sorted(plan.robot_assignments),
                        "command": "OBSERVE_HOLD" if step.observe_only else None,
                        "time": time.time(),
                    }
                )
                if step.observe_only:
                    _console("[%s/%s] observe hold: %s"
                             % (phase.name, step.plan_id,
                                ",".join(sorted(plan.robot_assignments))))
                    guard.check_expected(step.plan_id)
                    result = "SUCCESS"
                    _console("[%s/%s] SUCCESS" % (phase.name, step.plan_id))
                else:
                    batch = build_batch(
                        context,
                        plan,
                        context.mission_id,
                        run_id,
                        phase.timeout_s,
                        clock,
                    )
                    _console("[%s/%s] sending %s -> %s"
                             % (phase.name, step.plan_id, batch[0].command_type,
                                ",".join(sorted(plan.robot_assignments))))
                    logger.write(
                        {
                            "event": "phase_start_command",
                            "phase": phase.name,
                            "plan_id": step.plan_id,
                            "command_type": batch[0].command_type,
                        }
                    )
                    transport.send_batch(batch)
                    sent = transport.history[-1].to_dict()
                    logger.write({"event": "command_sent", **sent})
                    _console("[%s/%s] sent command_id=%s"
                             % (phase.name, step.plan_id,
                                sent.get("command_id", "?")))
                    result = wait_phase(
                        transport, batch, phase, step.plan_id, guard, logger
                    )
                if result != "SUCCESS":
                    logger.write(
                        {
                            "event": "phase_result",
                            "phase": phase.name,
                            "plan_id": step.plan_id,
                            "outcome": result,
                        }
                    )
                    raise RuntimeError("%s/%s:%s" % (phase.name, step.plan_id, result))
                logger.write(
                    {
                        "event": "phase_result",
                        "phase": phase.name,
                        "plan_id": step.plan_id,
                        "outcome": result,
                        "observation_only": step.observe_only,
                        "time": time.time(),
                    }
                )
            deadline = time.monotonic() + phase.dwell_s
            last_plan = phase.steps[-1].plan_id
            while time.monotonic() < deadline:
                guard.check()
                guard.check_expected(last_plan)
                time.sleep(0.05)
            update_roster(context, phase, spec, logger)
            logger.write(
                {
                    "event": "roster",
                    "phase": phase.name,
                    "roster": context.roster.model_dump(mode="json"),
                }
            )
            if phase.publishes:
                events.add(phase.publishes)
                logger.write(
                    {
                        "event": "local_coordination",
                        "event_type": phase.publishes,
                        "scope": "single_process_mock_b",
                        "time": time.time(),
                    }
                )
            print("[%s] SUCCESS" % phase.name, flush=True)
        complete = len(phases) == all_phase_count
        logger.write(
            {
                "event": "mission_finished",
                "status": "SUCCESS" if complete else "PARTIAL_SUCCESS",
                "exited_hovering": [
                    uav for uav in spec["faulted_ids"] if uav in context.roster.failed_ids
                ],
            }
        )
        _console("[driver] mission %s (%d/%d phases)"
                 % ("complete" if complete else "partial",
                    len(phases), all_phase_count))
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        logger.write(
            {
                "event": "mission_finished",
                "status": "FAILURE",
                "reason": str(exc) or "interrupted",
            }
        )
        try:
            transport.emergency_hold(context.mission_id)
        except Exception as error:  # noqa: BLE001
            logger.write({"event": "tcp_hold_error", "reason": str(error)})
        try:
            guard.abort_all(context.mission_id, logger)
        except Exception as error:  # noqa: BLE001
            logger.write(
                {
                    "event": "hold_unconfirmed",
                    "reason": str(error),
                    "uavs": list(spec["all_ids"]),
                }
            )
        _console("[driver] ABORT: %s" % (str(exc) or "interrupted"))
        print("任务中止：%s；请飞手确认全部飞机状态并接管。" % (str(exc) or "interrupted"),
              file=sys.stderr)
        return 1


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="打印全部线上命令（默认）")
    mode.add_argument("--execute", action="store_true", help="现场操作员显式启动飞行任务")
    parser.add_argument("--spec", default=str(default_spec_path()))
    parser.add_argument("--tcp-host", default="192.168.5.241")
    parser.add_argument("--tcp-port", type=int, default=39001)
    parser.add_argument("--until", default=None, help="完成指定阶段后停止（含该阶段）")
    parser.add_argument("--log", help="JSONL证据路径；默认自动生成")
    args = parser.parse_args(argv)

    spec_path = Path(args.spec)
    if not spec_path.exists():
        print("找不到 %s；先运行 generate_mission.py --available ..." % spec_path,
              file=sys.stderr)
        return 2
    spec = load_spec(spec_path)
    all_phases = build_phases(spec)
    phase_names = [phase.name for phase in all_phases]
    if args.until is None:
        args.until = all_phases[-1].name
    if args.until not in phase_names:
        print("--until 必须是 %s 之一" % ", ".join(phase_names), file=sys.stderr)
        return 2
    phases = all_phases[: phase_names.index(args.until) + 1]

    log_path = Path(args.log) if args.log else None
    if args.execute and log_path is None:
        log_path = default_log_path()
        print("[driver] log: %s" % log_path, flush=True)
    if args.execute and log_path is not None:
        # 每轮把生成输入/哈希复制到证据目录，便于复盘“这轮到底飞了哪套配置”。
        log_dir = log_path.parent
        log_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(spec_path, log_dir / "mission_spec.json")
        manifest_path = spec_path.parent / "manifest.json"
        if manifest_path.exists():
            shutil.copy2(manifest_path, log_dir / "manifest.json")
        fleet_file = Path(spec.get("fleet_file", ""))
        if fleet_file.is_file():
            shutil.copy2(fleet_file, log_dir / "fleet.yaml")
    package = load_package(spec_path)
    if args.execute:
        package.context.mission_id += "-" + uuid.uuid4().hex[:12]

    clock = SystemClock()
    transport = TcpCommandTransport(
        package,
        clock,
        args.tcp_host,
        args.tcp_port,
        connect_timeout_s=10,
        # GCS 可能按排程错峰延迟 START（本任务最大约 9 s），ACCEPT 窗口需覆盖。
        accept_timeout_s=20,
    )
    try:
        if not args.execute:
            for phase in phases:
                for step in phase.steps:
                    logger_row = {
                        "phase": phase.name,
                        "plan_id": step.plan_id,
                        "observe_only": step.observe_only,
                    }
                    if step.observe_only:
                        print(json.dumps(dict(logger_row, wire=None), ensure_ascii=False, indent=2))
                        continue
                    batch = build_batch(
                        package.context,
                        package.context.plans[step.plan_id],
                        package.context.mission_id,
                        "dry-run",
                        phase.timeout_s,
                        clock,
                    )
                    transport._validate_batch(tuple(batch))
                    wire = transport._build_message(
                        tuple(batch), "dry-%s-%s" % (phase.name, step.plan_id)
                    )
                    print(
                        json.dumps(
                            dict(logger_row, wire=wire),
                            ensure_ascii=False,
                            indent=2,
                        )
                    )
                update_roster(package.context, phase, spec, JsonlLogger(None))
            return 0

        from verification.full_fleet_joint.flight_guard import RosFlightGuard

        guard = RosFlightGuard(package, spec)
        with JsonlLogger(log_path) as logger:
            _console("[driver] waiting for calibration_ready and all UAVs connected/armed/OFFBOARD")
            guard.wait_ready()
            _console("[driver] ready")
            transport.wait_until_ready()
            return run_sequence(
                package,
                transport,
                guard,
                logger,
                phases,
                len(all_phases),
            )
    finally:
        transport.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:  # noqa: BLE001
        print("配置/连接错误：%s" % exc, file=sys.stderr)
        sys.exit(2)
