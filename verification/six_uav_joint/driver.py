#!/usr/bin/env python3
"""UAV6–11 小规模联合流程；所有线上身份仍为 GroupA/Axx。

默认只打印计划。--execute 才连接 GCS；不 arm、不切模式、不自动降落。
复用 recon 的批次构造及 runtime 的 TCP、汇总与退出/接替名册逻辑。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))

from verification.four_uav_recon.driver import JsonlLogger, build_batch
from uav_bt_runtime.actions import RecoverReconGroupHandler, SimulateDamageHandler
from uav_bt_runtime.clock import SystemClock
from uav_bt_runtime.config import load_mission_context, load_json_model
from uav_bt_runtime.package import WorldConfig
from uav_bt_runtime.tcp_transport import TcpCommandTransport
from uav_bt_runtime.tracking import BatchResult, CommandBatchTracker

IDS = ("A06", "A07", "A08", "A09", "A10", "A11")
RECON_START = {"A06", "A07", "A08"}
RECON_RECOVERED = {"A07", "A08", "A11"}
MOCK_B = {"A09", "A10"}


@dataclass(frozen=True)
class Phase:
    name: str
    plan_id: str
    timeout_s: float = 180.0
    requires: str = ""
    publishes: str = ""
    dwell_s: float = 0.0
    observe_only: bool = False


PHASES = (
    Phase("prepare", "prepare-all", publishes="GROUPS_READY"),
    Phase("coverage1", "coverage-segment-1", requires="GROUPS_READY"),
    Phase("fault", "fault-exit"),
    Phase("recovery", "recovery-a"),
    Phase("coverage2", "coverage-segment-2", publishes="RECON_COMPLETE"),
    Phase("strike", "mock-b-strike", requires="RECON_COMPLETE"),
    Phase("strike_hold", "mock-b-hold", timeout_s=30, dwell_s=3,
          observe_only=True, publishes="STRIKE_COMPLETE"),
    Phase("return_a", "return-a", requires="STRIKE_COMPLETE"),
    Phase("hold", "hold-end", timeout_s=30, dwell_s=3, observe_only=True),
)


def load_package(path=HERE / "package"):
    path = Path(path)
    context = load_mission_context(path / "robots.json", path / "bootstrap.json",
                                   path / "plans/plans.json")
    package = SimpleNamespace(context=context,
                              world=load_json_model(path / "world.json", WorldConfig))
    if set(context.uav_specs) != set(IDS) or context.group_id != "GroupA":
        raise ValueError("本测试只支持 GroupA 的 UAV6–11")
    if context.roster.active_ids != ["A06", "A07", "A08", "A09", "A10"]:
        raise ValueError("物理 GroupA 初始 active 必须为 A06–A10；其中 A09/A10 模拟 B 组")
    if context.roster.reserve_ids != ["A11"]:
        raise ValueError("备用机必须为 A11")
    if context.roster.inactive_ids:
        raise ValueError("本轮六机均参与任务，不允许 inactive 成员")
    expected = (
        set(IDS), RECON_START, {"A06"}, {"A11"},
        RECON_RECOVERED, MOCK_B, MOCK_B,
        RECON_RECOVERED, RECON_RECOVERED | MOCK_B,
    )
    for phase, members in zip(PHASES, expected):
        plan = context.plans[phase.plan_id]
        if set(plan.robot_assignments) != members:
            raise ValueError("%s 成员错误" % phase.plan_id)
    return package


def update_roster(context, phase):
    """仅在整批真实成功后提交名册变化；模拟 B 不加入侦查编队。"""
    services = SimpleNamespace(mission=context)
    if phase.name == "fault":
        handler = SimulateDamageHandler("fault", phase.plan_id, phase.timeout_s)
        handler.selected_ids = ["A06"]
        handler.on_success(services)
    elif phase.name == "recovery":
        handler = RecoverReconGroupHandler("recovery", phase.plan_id, phase.timeout_s)
        selected = handler.select_ids(services, context.plans[phase.plan_id])
        if set(selected) - MOCK_B != RECON_RECOVERED:
            raise ValueError("接替前名册不符合预期")
        handler.on_success(services)


def wait_phase(transport, batch, phase, guard, logger):
    tracker = CommandBatchTracker(batch)
    deadline = time.monotonic() + phase.timeout_s
    while True:
        guard.check()
        if phase.name == "return_a":
            guard.check_targets("mock-b-hold")
        for feedback in transport.poll_feedback():
            tracker.apply(feedback)
            logger.write({"event": "command_feedback", "phase": phase.name,
                          "robot_id": feedback.robot_id, "command_id": feedback.command_id,
                          "status": feedback.status.value, "error_code": feedback.error_code,
                          "error_message": feedback.error_message})
        if tracker.result is BatchResult.FAILURE:
            return "FAILURE"
        if tracker.result is BatchResult.SUCCESS:
            # 独立检查实测终点，避免将合成 COMPLETED 当作飞行证据。
            observed = guard.check_targets(phase.plan_id)
            logger.write({"event": "phase_observation", "phase": phase.name,
                          "positions_enu": observed, "time": time.time()})
            return "SUCCESS"
        if time.monotonic() >= deadline:
            return "TIMEOUT"
        time.sleep(0.05)


def run_sequence(package, transport, guard, logger, phases=PHASES, confirm=None):
    """可注入本地假 GCS/观测器做协议测试；实机观测器没有旁路开关。"""
    context = package.context
    clock = SystemClock()
    run_id = uuid.uuid4().hex
    events = set()
    logger.write({"event": "mission_start", "mission_id": context.mission_id,
                  "run_id": run_id, "participants": list(IDS),
                  "coordination": "local_mock_b", "phases": [p.name for p in phases]})
    try:
        guard.wait_ready()
        if confirm:
            confirm("start", "确认本轮六机通信检查通过、机位与净空已核对；六机已人工起飞并稳定在约5m。")
        guard.check()
        guard.check_initial_positions()
        for phase in phases:
            if phase.requires and phase.requires not in events:
                raise RuntimeError("缺少阶段事件 %s" % phase.requires)
            if confirm and phase.name in {"fault", "recovery", "strike"}:
                prompts = {
                    "fault": "确认第一段覆盖结束，UAV6 下降至8m并向左离队的航路可用。",
                    "recovery": "确认 UAV6 已在退出点8m稳定悬停，允许 UAV11 加入；退出机继续保持空中悬停。",
                    "strike": "确认侦查完成，允许 UAV9/10 前往地面目标上方5m，并在完成后原地悬停。",
                }
                confirm(phase.name, prompts[phase.name])
            guard.check()
            if phase.name == "return_a":
                guard.check_targets("mock-b-hold")
            plan = context.plans[phase.plan_id]
            batch = build_batch(context, plan, context.mission_id, run_id,
                                phase.timeout_s, clock)
            logger.write({"event": "phase_start", "phase": phase.name,
                          "plan_id": phase.plan_id, "robot_ids": sorted(plan.robot_assignments),
                          "command": "OBSERVE_HOLD" if phase.observe_only else batch[0].command_type,
                          "time": time.time()})
            if phase.observe_only:
                # MOVE_TO完成后底层已经保持终点；仅观测，不再次发送同点目标或HOVER占位坐标。
                guard.check_targets(phase.plan_id)
                result = "SUCCESS"
            else:
                transport.send_batch(batch)
                # 发出后立即落盘；失败轮也保留线上命令，不等整轮成功。
                logger.write({"event": "command_sent", **transport.history[-1].to_dict()})
                result = wait_phase(transport, batch, phase, guard, logger)
            if result != "SUCCESS":
                logger.write({"event": "phase_result", "phase": phase.name, "outcome": result})
                raise RuntimeError("%s:%s" % (phase.name, result))
            deadline = time.monotonic() + phase.dwell_s
            while time.monotonic() < deadline:
                guard.check()
                guard.check_targets(phase.plan_id)
                time.sleep(0.05)
            logger.write({"event": "phase_result", "phase": phase.name,
                          "outcome": result, "observation_only": phase.observe_only,
                          "time": time.time()})
            print("[%s] %s" % (phase.name, result), flush=True)
            update_roster(context, phase)
            logger.write({"event": "roster", "phase": phase.name,
                          "roster": context.roster.model_dump(mode="json")})
            if phase.publishes:
                events.add(phase.publishes)
                logger.write({"event": "local_coordination", "event_type": phase.publishes,
                              "scope": "single_process_mock_b", "time": time.time()})
        complete = len(phases) == len(PHASES)
        logger.write({"event": "mission_finished",
                      "status": "SUCCESS" if complete else "PARTIAL_SUCCESS",
                      "exited_hovering": ["A06"] if "A06" in context.roster.failed_ids else []})
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        logger.write({"event": "mission_finished", "status": "FAILURE",
                      "reason": str(exc) or "interrupted"})
        try:
            transport.emergency_hold(context.mission_id)
        except Exception as error:
            logger.write({"event": "tcp_hold_error", "reason": str(error)})
        # TCP HOLD 仅覆盖当前活动批次；额外逐机请求六机悬停并明确记录未确认项。
        try:
            guard.abort_all(context.mission_id, logger)
        except Exception as error:
            logger.write({"event": "hold_unconfirmed", "reason": str(error),
                          "uavs": list(IDS)})
        print("任务中止：%s；请飞手确认六机状态并接管。" % (str(exc) or "interrupted"), file=sys.stderr)
        return 1


def operator_confirm(name, prompt):
    print("\n[%s] %s 输入 CONTINUE 继续；其它输入中止。" % (name, prompt), flush=True)
    # select 使等待确认期间仍持续检查机队；由 main 设置回调。
    import select
    while not select.select([sys.stdin], [], [], 0.1)[0]:
        operator_confirm.guard.check()
    if sys.stdin.readline().strip() != "CONTINUE":
        raise RuntimeError("操作员中止 %s" % name)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="打印全部线上命令，不连接ROS/TCP（默认）")
    mode.add_argument("--execute", action="store_true", help="现场操作员显式启动飞行任务")
    parser.add_argument("--tcp-host", default="192.168.5.241")
    parser.add_argument("--tcp-port", type=int, default=39001)
    parser.add_argument("--until", choices=[p.name for p in PHASES], default="hold",
                        help="从准备开始，完成指定阶段后停止；不能跳过前序阶段")
    parser.add_argument("--log", help="JSONL证据路径；实机执行必须指定")
    args = parser.parse_args(argv)
    if args.execute and (not args.log or not sys.stdin.isatty()):
        parser.error("--execute 必须指定 --log 并在交互终端中运行")
    package = load_package()
    if args.execute:
        # 同日重跑不消费上轮 latched FAILED；每轮日志记录实际 mission_id。
        package.context.mission_id += "-" + uuid.uuid4().hex[:12]
    phases = PHASES[:next(i for i, p in enumerate(PHASES) if p.name == args.until) + 1]
    clock = SystemClock()
    transport = TcpCommandTransport(package, clock, args.tcp_host, args.tcp_port,
                                    connect_timeout_s=10, accept_timeout_s=5)
    try:
        if not args.execute:
            for phase in phases:
                batch = build_batch(package.context, package.context.plans[phase.plan_id],
                                    package.context.mission_id, "dry-run", phase.timeout_s, clock)
                transport._validate_batch(tuple(batch))
                wire = transport._build_message(tuple(batch), "dry-" + phase.name)
                print(json.dumps({"phase": phase.name, "requires": phase.requires,
                                  "publishes": phase.publishes, "dwell_s": phase.dwell_s,
                                  "observation_only": phase.observe_only,
                                  "wire": None if phase.observe_only else wire}, ensure_ascii=False, indent=2))
                update_roster(package.context, phase)
            return 0
        from verification.six_uav_joint.flight_guard import RosFlightGuard
        guard = RosFlightGuard(package)
        operator_confirm.guard = guard
        with JsonlLogger(Path(args.log)) as logger:
            # 先检查观测就绪再连接，连接失败时没有发送任何飞行任务。
            guard.wait_ready()
            transport.wait_until_ready()
            return run_sequence(package, transport, guard, logger, phases, operator_confirm)
    finally:
        transport.close()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("配置/连接错误：%s" % exc, file=sys.stderr)
        sys.exit(2)
