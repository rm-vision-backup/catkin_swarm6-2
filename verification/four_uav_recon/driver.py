#!/usr/bin/env python3
"""A 组 4 机实机测试驱动：编队准备（MOVE_TO）+ 侦查（FOLLOW_ROUTE）。

为什么不用 runtime 的入口：``package.py`` 把 GroupA 行为树硬编码成固定 10 个动作
（prepare -> 发布就绪 -> 等 B 组 -> 侦查段1 -> 模拟损伤 -> 恢复 -> 侦查段2 -> ...），
且该校验器在 ``load_mission_package`` 里被无条件调用，没有旁路开关。本测试只有
"编队 + 侦查"两阶段，用行为树表达反而要伪造损伤机与备用机。

这里只复用 runtime 的传输层，跳过任务包校验：

- ``tcp_transport`` —— 线上消息构造、ACCEPTED/COMPLETED/FAILED 解析、HOLD 发送
- ``tracking``      —— 整批终态判定
- ``models`` / ``config`` —— 数据模型与 MissionContext

这样与 tcp_to_ros 之间的线上协议不可能漂移，因为用的是同一份代码。

安全兜底分工（已核实 ``tcp_transport.py:200-231``）：
- ACCEPTED 超时的 HOLD 由传输层在 ``poll_feedback()`` 里自动发，此处不重复实现；
- 本脚本负责：整批超时、整批失败、KeyboardInterrupt、自身异常 —— 均调用
  ``emergency_hold()``。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_SRC = REPO_ROOT / "src" / "runtime" / "src"
if str(RUNTIME_SRC) not in sys.path:
    sys.path.insert(0, str(RUNTIME_SRC))

from uav_bt_runtime.clock import SystemClock  # noqa: E402
from uav_bt_runtime.config import (  # noqa: E402
    PlanCatalog,
    MissionBootstrapConfig,
    UavCatalog,
    build_mission_context,
    load_json_model,
)
from uav_bt_runtime.models import CommandEnvelope  # noqa: E402
from uav_bt_runtime.package import WorldConfig  # noqa: E402
from uav_bt_runtime.tcp_transport import TcpCommandTransport  # noqa: E402
from uav_bt_runtime.tracking import BatchResult, CommandBatchTracker  # noqa: E402

# ACCEPTED 等待窗口。协议文档 §3.1 要求地面站 5 s 内回 ACCEPTED，该义务不变；
# 这里放宽到 10 s 是客户端单方面容差，不修改 tcp_transport 的默认值。
ACCEPT_TIMEOUT_S = 10.0

POLL_INTERVAL_S = 0.02

EXIT_SUCCESS = 0
EXIT_FAILURE = 1
EXIT_CONFIG_ERROR = 2


@dataclass(frozen=True)
class Phase:
    """一个阶段 == 一条整组 TCP 命令。"""

    name: str
    plan_id: str
    timeout_s: float


DEFAULT_PHASES = (
    Phase(name="prepare", plan_id="prepare-a", timeout_s=150.0),
    Phase(name="coverage", plan_id="coverage-segment-1", timeout_s=300.0),
)


class JsonlLogger:
    """把任务过程写成 JSON Lines，留下证据。"""

    def __init__(self, path: Optional[Path]) -> None:
        self.path = path
        self._handle = None

    def __enter__(self) -> "JsonlLogger":
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = self.path.open("a", encoding="utf-8")
        return self

    def __exit__(self, *exc_info) -> None:
        if self._handle is not None:
            self._handle.close()

    def write(self, record: Dict[str, object]) -> None:
        if self._handle is None:
            return
        self._handle.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
        self._handle.flush()


def build_batch(
    context,
    plan,
    mission_id: str,
    run_id: str,
    timeout_s: float,
    clock: SystemClock,
) -> List[CommandEnvelope]:
    """把一个 plan 摊平成一整批 per-UAV CommandEnvelope。

    同一批必须共享 mission / action_instance / group / command_type，
    否则传输层的 _validate_batch 会拒绝。
    """

    now = clock.time()
    action_instance_id = f"{plan.plan_id}-{run_id}"
    commands: List[CommandEnvelope] = []
    for sequence_number, robot_id in enumerate(sorted(plan.robot_assignments)):
        assignment = plan.robot_assignments[robot_id]
        commands.append(
            CommandEnvelope(
                mission_id=mission_id,
                action_instance_id=action_instance_id,
                command_id=f"{plan.plan_id}-{run_id}-{robot_id}",
                group_id=context.group_id,
                robot_id=robot_id,
                command_type=assignment.command_type,
                payload=dict(assignment.payload),
                timeout_s=timeout_s,
                issued_at=now,
                expires_at=now + timeout_s,
                sequence_number=sequence_number,
            )
        )
    return commands


def await_batch(
    transport: TcpCommandTransport,
    tracker: CommandBatchTracker,
    timeout_s: float,
    clock: SystemClock,
    logger: JsonlLogger,
) -> str:
    """轮询直到整批进入终态，或阶段超时。返回 SUCCESS / FAILURE / TIMEOUT。"""

    deadline = clock.monotonic() + timeout_s
    while True:
        for feedback in transport.poll_feedback():
            tracker.apply(feedback)
            logger.write(
                {
                    "event": "command_feedback",
                    "robot_id": feedback.robot_id,
                    "command_id": feedback.command_id,
                    "status": feedback.status.value,
                    "error_code": feedback.error_code,
                }
            )

        result = tracker.result
        if result is BatchResult.SUCCESS:
            return "SUCCESS"
        if result is BatchResult.FAILURE:
            return "FAILURE"
        if clock.monotonic() >= deadline:
            return "TIMEOUT"
        time.sleep(POLL_INTERVAL_S)


def run_mission(args: argparse.Namespace) -> int:
    package_dir = Path(args.package).resolve()

    catalog = load_json_model(package_dir / "robots.json", UavCatalog)
    bootstrap = load_json_model(package_dir / "bootstrap.json", MissionBootstrapConfig)
    plan_catalog = load_json_model(package_dir / "plans" / "plans.json", PlanCatalog)
    world = load_json_model(package_dir / "world.json", WorldConfig)
    context = build_mission_context(catalog, bootstrap, plan_catalog)

    if bootstrap.mission_id != args.mission_id:
        raise ValueError(
            f"mission_id mismatch: bootstrap has {bootstrap.mission_id!r}, "
            f"command line has {args.mission_id!r}"
        )

    phases = list(DEFAULT_PHASES)
    for phase in phases:
        if phase.plan_id not in context.plans:
            raise ValueError(
                f"plan {phase.plan_id!r} not present in the package; "
                f"available: {sorted(context.plans)}"
            )

    log_path = Path(args.log).resolve() if args.log else None
    clock = SystemClock()
    run_id = f"{int(clock.time())}"

    # 传输层只会用到这四处：context.group_id / roster.active_ids /
    # uav_specs[*].is_leader / world.targets（后者仅 B 组 ATTACK 使用）。
    shim = SimpleNamespace(context=context, world=world)

    with JsonlLogger(log_path) as logger:
        logger.write(
            {
                "event": "mission_start",
                "mission_id": args.mission_id,
                "group_id": context.group_id,
                "active_ids": list(context.roster.active_ids),
                "tcp_host": args.tcp_host,
                "tcp_port": args.tcp_port,
                "accept_timeout_s": ACCEPT_TIMEOUT_S,
                "phases": [{"name": p.name, "plan_id": p.plan_id} for p in phases],
            }
        )

        if args.dry_run:
            transport = TcpCommandTransport(
                shim, clock, "127.0.0.1", args.tcp_port,
                accept_timeout_s=ACCEPT_TIMEOUT_S,
            )
            for phase in phases:
                plan = context.plans[phase.plan_id]
                batch = build_batch(
                    context, plan, args.mission_id, run_id, phase.timeout_s, clock
                )
                # _build_message 是私有方法；dry-run 的目的就是看线上 JSON，
                # 所以这里直接调用，不另写一份构造逻辑（否则就失去核对意义）。
                wire = transport._build_message(tuple(batch), f"tcp-dryrun-{phase.name}")
                print(json.dumps(wire, ensure_ascii=False, indent=2))
            transport.close()
            return EXIT_SUCCESS

        transport = TcpCommandTransport(
            shim, clock, args.tcp_host, args.tcp_port,
            connect_timeout_s=args.connect_timeout_s,
            accept_timeout_s=ACCEPT_TIMEOUT_S,
        )
        transport.wait_until_ready()
        logger.write({"event": "tcp_connected", "host": args.tcp_host, "port": args.tcp_port})

        try:
            for phase in phases:
                plan = context.plans[phase.plan_id]
                batch = build_batch(
                    context, plan, args.mission_id, run_id, phase.timeout_s, clock
                )
                tracker = CommandBatchTracker(batch)

                logger.write(
                    {
                        "event": "phase_start",
                        "phase": phase.name,
                        "plan_id": phase.plan_id,
                        "command_type": batch[0].command_type,
                        "robot_ids": [item.robot_id for item in batch],
                        "timeout_s": phase.timeout_s,
                    }
                )

                transport.send_batch(batch)
                outcome = await_batch(
                    transport, tracker, phase.timeout_s, clock, logger
                )

                logger.write(
                    {
                        "event": "phase_result",
                        "phase": phase.name,
                        "plan_id": phase.plan_id,
                        "outcome": outcome,
                        "statuses": {
                            command.robot_id: tracker.statuses[command.command_id].value
                            if tracker.statuses[command.command_id] is not None
                            else None
                            for command in batch
                        },
                    }
                )
                print(
                    f"[{phase.name}] {outcome} "
                    f"({phase.plan_id}, {len(batch)} UAVs)"
                )

                if outcome != "SUCCESS":
                    # 整批超时 / 失败 —— 传输层不会替这个场景发 HOLD（它只管
                    # ACCEPTED 超时），必须由本脚本显式兜底。
                    transport.emergency_hold(args.mission_id)
                    logger.write(
                        {
                            "event": "mission_finished",
                            "status": "FAILURE",
                            "reason": f"{phase.name}:{outcome}",
                        }
                    )
                    print(f"[{phase.name}] 已发送 HOLD 安全兜底", file=sys.stderr)
                    return EXIT_FAILURE

            for record in transport.history:
                logger.write({"event": "command_sent", **record.to_dict()})
            logger.write({"event": "mission_finished", "status": "SUCCESS"})
            return EXIT_SUCCESS

        except KeyboardInterrupt:
            # 人工中止：协议文档 §7 要求此时发 HOLD 安全悬停。
            transport.emergency_hold(args.mission_id)
            logger.write(
                {"event": "mission_finished", "status": "FAILURE", "reason": "interrupted"}
            )
            print("\n已中断，已发送 HOLD 安全兜底", file=sys.stderr)
            return EXIT_FAILURE

        except Exception as exc:  # noqa: BLE001
            logger.write(
                {
                    "event": "mission_finished",
                    "status": "FAILURE",
                    "reason": f"driver_error:{type(exc).__name__}",
                }
            )
            try:
                transport.emergency_hold(args.mission_id)
            except Exception:  # noqa: BLE001
                pass
            raise

        finally:
            transport.close()


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="A 组 4 机实机测试驱动：编队准备 + 侦查"
    )
    parser.add_argument(
        "--package",
        default=str(Path(__file__).resolve().parent / "package"),
        help="任务数据目录（含 robots/bootstrap/world/plans）",
    )
    parser.add_argument("--mission-id", default="four-uav-recon-001")
    parser.add_argument("--tcp-host", default="192.168.5.241")
    parser.add_argument("--tcp-port", type=int, default=39001)
    parser.add_argument("--connect-timeout-s", type=float, default=10.0)
    parser.add_argument("--log", default=None, help="JSONL 日志路径（默认不落盘）")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印将要发送的线上 JSON，不连接地面站",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    try:
        return run_mission(args)
    except KeyboardInterrupt:
        return EXIT_FAILURE
    except Exception as exc:  # noqa: BLE001
        print(f"配置或程序错误: {type(exc).__name__}: {exc}", file=sys.stderr)
        return EXIT_CONFIG_ERROR


if __name__ == "__main__":
    sys.exit(main())
