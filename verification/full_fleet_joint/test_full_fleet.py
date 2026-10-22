"""离线生成/几何、真实TCP回环、初始9架+备用补充与观测器验证；不启动ROS/PX4。"""
from __future__ import annotations

import dataclasses
import io
import json
import math
import re
import socket
import statistics
import tempfile
import threading
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from types import SimpleNamespace as NS
from xml.etree import ElementTree as ET

from verification.full_fleet_joint import driver, generate_mission
from verification.full_fleet_joint.flight_guard import RosFlightGuard
from verification.full_fleet_joint.sitl import generate_spawn

CUSTOM_FLEET = Path(__file__).resolve().parent / "fleet_14_custom.yaml"
CUSTOM_AVAILABLE = ["1,2,4,5,6,8,9,10,11,12,13,14,15,16"]
CUSTOM_FLEET_11 = Path(__file__).resolve().parent / "fleet_11_custom.yaml"
CUSTOM_AVAILABLE_11 = ["2,6,8,9,10,11,12,13,14,15,16"]


class Logger:
    def __init__(self):
        self.rows = []

    def write(self, row):
        self.rows.append(row)


class Observer:
    """测试替身；不出现在命令行执行路径。"""

    def __init__(self, fail_plan=None):
        self.fail_plan = fail_plan
        self.checked = []
        self.aborted = False

    def check(self):
        pass

    def check_expected(self, plan_id):
        self.checked.append(plan_id)
        if plan_id == self.fail_plan:
            raise RuntimeError("实测未到点")

    def abort_all(self, mission_id, logger):
        self.aborted = True


class GroundStation:
    """本地 JSON Lines 协议对端：可模拟指定批次失败、漏报成员或断线。"""

    def __init__(self, fail_at=None, disconnect_at=None, incomplete_at=None):
        self.fail_at = fail_at
        self.disconnect_at = disconnect_at
        self.incomplete_at = incomplete_at
        self.received = []
        self.socket = socket.socket()
        self.socket.bind(("127.0.0.1", 0))
        self.socket.listen(1)
        self.port = self.socket.getsockname()[1]
        self.client = None
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def serve(self):
        try:
            self.client, _ = self.socket.accept()
            with self.client.makefile("rb") as stream:
                for line in stream:
                    message = json.loads(line)
                    self.received.append(message)
                    if message["command"] == "HOLD":
                        continue
                    index = sum(item["command"] != "HOLD" for item in self.received)
                    if index == self.disconnect_at:
                        self.client.shutdown(socket.SHUT_RDWR)
                        return
                    ids = [item["uav_id"] for item in message["assignments"]]
                    base = {
                        key: message[key]
                        for key in ("version", "mission_id", "group_id", "command_id")
                    }
                    base["type"] = "STATUS"
                    replies = [dict(base, status="ACCEPTED")]
                    if index == self.fail_at:
                        replies.append(
                            dict(
                                base,
                                status="FAILED",
                                failed_uavs=[ids[-1]],
                                error_code="TEST_FAILURE",
                                message="测试拒绝",
                            )
                        )
                    else:
                        replies.append(
                            dict(
                                base,
                                status="COMPLETED",
                                completed_uavs=(
                                    ids[:-1] if index == self.incomplete_at else ids
                                ),
                            )
                        )
                    self.client.sendall(
                        b"".join(
                            (json.dumps(row) + "\n").encode() for row in replies
                        )
                    )
        except OSError:
            pass

    def close(self):
        if self.client:
            try:
                self.client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.client.close()
        self.socket.close()
        self.thread.join(2)


class GeneratedFleetTests(unittest.TestCase):
    def make(self, available, leader=None, b_uavs=None, replacements=None):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        spec, manifest = generate_mission.generate(
            fleet_path=generate_mission.DEFAULT_FLEET,
            available=available,
            out_dir=Path(temp.name),
            leader=leader,
            b_uavs=b_uavs,
            replacements=replacements,
        )
        return Path(temp.name), spec, manifest

    def test_16_uav_initial9_spares3_b4(self):
        out, spec, manifest = self.make(["1-16"])
        self.assertEqual(len(spec["initial_ids"]), 9)
        self.assertEqual(len(spec["spare_ids"]), 3)
        self.assertEqual(len(spec["faulted_ids"]), 3)
        self.assertEqual(len(spec["core_ids"]), 9)
        self.assertEqual(len(spec["b_ids"]), 4)
        self.assertEqual(spec["leader_id"], "A06")  # edge leader, not the center
        self.assertEqual(spec["b_ids"], ["A05", "A10", "A15", "A16"])
        self.assertEqual(spec["b_strike_ids"], ["A05", "A10", "A15"])
        self.assertEqual(spec["b_reserve_id"], "A16")
        self.assertEqual(spec["b_failed_id"], "A10")
        self.assertEqual(spec["b_final_ids"], ["A05", "A15", "A16"])
        self.assertEqual(len(spec["b_targets"]), 3)
        self.assertEqual(spec["faulted_ids"], ["A03", "A08", "A13"])
        self.assertEqual(spec["spare_ids"], ["A04", "A09", "A14"])
        self.assertEqual(
            set(spec["core_ids"]),
            (set(spec["initial_ids"]) - set(spec["faulted_ids"]))
            | set(spec["spare_ids"]),
        )
        self.assertEqual(
            len(spec["initial_ids"]) + len(spec["spare_ids"]) + len(spec["b_ids"]),
            16,
        )
        checks = spec["geometry_checks"]
        self.assertEqual(checks["min_slot_spacing_m"], 5.0)
        self.assertTrue(checks["b_targets_inside_coverage"])
        self.assertEqual(len(checks["coverage_scan_rects"]), 4)
        self.assertGreaterEqual(checks["min_b_a_strike_3d_m"], 4.0)
        self.assertGreaterEqual(checks["b_a_vertical_m"], 5.0)
        self.assertGreaterEqual(checks["min_fault_b_horizontal_m"], 4.0)
        self.assertGreaterEqual(checks["fault_formation_vertical_m"], 3.0)
        self.assertGreater(checks["min_move_to_displacement_m"], 0.5)
        self.assertEqual(checks["calibration_median_x_m"], 0.0)
        formations = spec["formation_offsets"]["formations"]
        self.assertEqual(
            set(formations["recon_initial"]["participants"]), set(spec["initial_ids"])
        )
        self.assertEqual(
            set(formations["recon_recovered"]["participants"]), set(spec["core_ids"])
        )
        self.assertEqual(
            formations["recon_initial"]["offsets"][spec["leader_id"]],
            [0.0, 0.0, 0.0],
        )
        # Leader is on the left edge; the formation extends to its right.
        for task_id in spec["initial_ids"]:
            offset = formations["recon_initial"]["offsets"][task_id]
            self.assertIn(offset[0], (0.0, 5.0, 10.0))
            self.assertIn(offset[1], (-5.0, 0.0, 5.0))
        for task_id in spec["spare_ids"]:
            self.assertEqual(
                formations["recon_recovered"]["offsets"][task_id][0], 10.0
            )
        self.assertEqual(manifest["formation_block"], "left")

    def test_15_uav_with_one_spare_missing(self):
        available = ",".join(str(n) for n in range(1, 17) if n != 14)
        out, spec, manifest = self.make([available])
        self.assertEqual(len(spec["available_uavs"]), 15)
        self.assertEqual(len(spec["initial_ids"]), 9)
        self.assertEqual(len(spec["spare_ids"]), 2)
        self.assertEqual(len(spec["faulted_ids"]), 2)
        self.assertEqual(len(spec["b_ids"]), 4)
        self.assertEqual(spec["leader_id"], "A06")
        self.assertEqual(spec["spare_ids"], ["A04", "A09"])
        self.assertEqual(spec["faulted_ids"], ["A03", "A08"])
        self.assertEqual(spec["b_strike_ids"], ["A05", "A10", "A15"])
        self.assertEqual(spec["b_reserve_id"], "A16")
        self.assertEqual(spec["b_failed_id"], "A10")
        self.assertEqual(spec["b_final_ids"], ["A05", "A15", "A16"])

    def test_14_uav_with_one_spare(self):
        available = ",".join(str(n) for n in range(1, 17) if n not in (9, 14))
        out, spec, manifest = self.make([available])
        self.assertEqual(len(spec["available_uavs"]), 14)
        self.assertEqual(len(spec["initial_ids"]), 9)
        self.assertEqual(len(spec["spare_ids"]), 1)
        self.assertEqual(len(spec["faulted_ids"]), 1)
        self.assertEqual(len(spec["b_ids"]), 4)
        self.assertEqual(spec["leader_id"], "A06")
        self.assertEqual(spec["spare_ids"], ["A04"])
        self.assertEqual(spec["faulted_ids"], ["A03"])

    def test_15_uav_contiguous_fleet_uses_spare_column_for_b(self):
        # 15 架 = UAV1–15（UAV16 不可用）：B 需要 4 架，从备用列底部补 1 架。
        out, spec, manifest = self.make(["1-15"])
        self.assertEqual(len(spec["available_uavs"]), 15)
        self.assertEqual(len(spec["initial_ids"]), 9)
        self.assertEqual(len(spec["spare_ids"]), 2)
        self.assertEqual(len(spec["faulted_ids"]), 2)
        self.assertEqual(spec["spare_ids"], ["A04", "A09"])
        self.assertEqual(spec["faulted_ids"], ["A03", "A08"])
        self.assertEqual(spec["b_ids"], ["A05", "A10", "A14", "A15"])
        self.assertEqual(spec["b_strike_ids"], ["A05", "A10", "A14"])
        self.assertEqual(spec["b_reserve_id"], "A15")
        self.assertEqual(spec["b_failed_id"], "A10")
        self.assertEqual(len(spec["core_ids"]), 9)
        self.assertEqual(
            set(spec["plan_members"]["recovery-spares"]),
            set(spec["spare_ids"]),
        )

    def test_14_uav_contiguous_fleet_uses_spare_column_for_b(self):
        # 14 架 = UAV1–14：B 需要 4 架，从备用列底部补 2 架。
        out, spec, manifest = self.make(["1-14"])
        self.assertEqual(len(spec["available_uavs"]), 14)
        self.assertEqual(len(spec["initial_ids"]), 9)
        self.assertEqual(len(spec["spare_ids"]), 1)
        self.assertEqual(len(spec["faulted_ids"]), 1)
        self.assertEqual(spec["spare_ids"], ["A04"])
        self.assertEqual(spec["faulted_ids"], ["A03"])
        self.assertEqual(spec["b_ids"], ["A05", "A09", "A10", "A14"])
        self.assertEqual(spec["b_strike_ids"], ["A05", "A09", "A10"])
        self.assertEqual(spec["b_reserve_id"], "A14")
        self.assertEqual(spec["b_failed_id"], "A09")
        self.assertEqual(len(spec["core_ids"]), 9)
        self.assertEqual(
            set(spec["plan_members"]["recovery-spares"]),
            set(spec["spare_ids"]),
        )

    def test_15_uav_uav16_replaces_failed_uav13_slot(self):
        # 本次 15 架实飞：UAV13 故障，UAV16 顶替 UAV13 的地面机位。
        available = "1-12,14,15,16"
        with self.assertRaises(generate_mission.GenerationError):
            self.make([available])
        out, spec, manifest = self.make(
            [available], replacements={"UAV13": "UAV16"}
        )
        self.assertEqual(len(spec["available_uavs"]), 15)
        self.assertEqual(spec["slot_replacements"], {"UAV13": "UAV16"})
        self.assertEqual(len(spec["initial_ids"]), 9)
        self.assertIn("A16", spec["initial_ids"])
        self.assertNotIn("A13", spec["initial_ids"])
        self.assertEqual(
            spec["initial_ids"],
            ["A01", "A02", "A03", "A06", "A07", "A08", "A11", "A12", "A16"],
        )
        self.assertEqual(spec["spare_ids"], ["A04", "A09"])
        self.assertEqual(spec["faulted_ids"], ["A03", "A08"])
        self.assertEqual(spec["b_ids"], ["A05", "A10", "A14", "A15"])
        self.assertEqual(spec["b_strike_ids"], ["A05", "A10", "A14"])
        self.assertEqual(spec["b_reserve_id"], "A15")
        self.assertEqual(spec["b_failed_id"], "A10")
        self.assertEqual(len(spec["core_ids"]), 9)
        # UAV16 的地面位置 = 原 UAV13 机位（第 3 排第 3 列）。
        self.assertEqual(
            spec["ground_positions"]["A16"],
            {"x": 0.0, "y": -5.0, "z": 0.0, "yaw": 0.0},
        )
        self.assertEqual(
            spec["formation_offsets"]["formations"]["recon_initial"]["offsets"]["A16"],
            [10.0, -5.0, 0.0],
        )
        grid = generate_mission.grid_map(
            generate_mission.load_fleet(), {"UAV13": "UAV16"}
        )
        self.assertEqual(grid["UAV16"], (0.0, -5.0))
        self.assertNotIn("UAV13", grid)
        spawn = out / "spawn.launch"
        generate_spawn.generate(
            out / "mission_spec.json", spawn, generate_mission.DEFAULT_FLEET
        )
        groups = ET.parse(spawn).getroot().findall("group")
        self.assertEqual(len(groups), 15)
        by_number = {}
        for group in groups:
            args = {
                item.attrib["name"]: item.attrib["value"]
                for item in group.findall("include/arg")
            }
            by_number[int(args["ID"]) + 1] = (float(args["x"]), float(args["y"]))
        self.assertEqual(by_number[16], (100.0, 95.0))
        self.assertNotIn(13, by_number)

    def test_rejects_incomplete_ground_block(self):
        available = ",".join(str(n) for n in range(1, 17) if n != 8)
        with self.assertRaises(generate_mission.GenerationError):
            self.make([available])

    def test_explicit_b_override(self):
        available = ",".join(str(n) for n in range(1, 17) if n != 14)
        out, spec, manifest = self.make(
            [available], b_uavs=["UAV5", "UAV10", "UAV15", "UAV16"]
        )
        self.assertEqual(spec["b_ids"], ["A05", "A10", "A15", "A16"])
        self.assertEqual(spec["b_strike_ids"], ["A05", "A10", "A15"])
        self.assertEqual(spec["b_reserve_id"], "A16")
        self.assertEqual(spec["b_failed_id"], "A10")
        self.assertEqual(len(spec["b_targets"]), 3)
        self.assertEqual(len(spec["available_uavs"]), 15)

    def test_onboard_commands_have_neighbors_layers_and_udp(self):
        out, spec, manifest = self.make(["1-16"])
        text = (out / "onboard_commands.sh").read_text()
        lines = [line for line in text.splitlines() if line.startswith("bash ")]
        self.assertEqual(len(lines), 16)
        b_numbers = {int(task_id[1:]) for task_id in spec["b_ids"]}
        for line in lines:
            match = re.search(r"start (\d+) --", line)
            self.assertIsNotNone(match)
            number = int(match.group(1))
            self.assertIn("bundle_transport:=udp", line)
            intents = re.search(r"neighbor_intents:=([^ ]+)", line).group(1)
            poses = re.search(r"neighbor_odom_topics:=([^ ]+)", line).group(1)
            self.assertEqual(len(intents.split(",")), 15)
            self.assertEqual(len(poses.split(",")), 15)
            self.assertNotIn("/UAV%d/" % number, intents)
            if number in b_numbers:
                self.assertIn("layer_move_to:=8.0", line)
            else:
                self.assertNotIn("layer_move_to:=8.0", line)

    def test_flow_env_uses_generated_fleet_and_leader(self):
        out, spec, manifest = self.make(["1-16"])
        text = (out / "flow_env.sh").read_text()
        numbers = re.search(r"AIR_FLOW_UAVS='([^']*)'", text).group(1).split()
        self.assertEqual(set(int(n) for n in numbers), set(range(1, 17)))
        self.assertIn("AIR_FLOW_LEADER=6", text)

    def test_sitl_gcs_accepts_package_config_overrides(self):
        # record_run.py 按 --spec 所在任务包传入编队/标定配置；15/14 备份包
        # 不能回落到 generated/config，否则 coverage2 成员校验会失败。
        launch = generate_mission.HERE / "sitl" / "gcs_sim.launch"
        root = ET.parse(launch).getroot()
        args = {item.attrib["name"] for item in root.findall("arg")}
        self.assertIn("formation_offsets_file", args)
        self.assertIn("calibration_config_file", args)
        includes = root.findall("include")
        forwarded = set()
        for include in includes:
            forwarded |= {
                item.attrib["name"] for item in include.findall("arg")
            }
        self.assertIn("formation_offsets_file", forwarded)
        self.assertIn("calibration_config_file", forwarded)

    def test_sitl_spawn_matches_grid_and_ports(self):
        out, spec, _manifest = self.make(["1-16"])
        spawn = out / "spawn.launch"
        generate_spawn.generate(
            out / "mission_spec.json", spawn, generate_mission.DEFAULT_FLEET
        )
        root = ET.parse(spawn).getroot()
        groups = root.findall("group")
        self.assertEqual(len(groups), 16)
        # 出生点必须与任务包 ground_positions 一致（含 calibration.shift_y）。
        ground = spec["ground_positions"]
        for group in groups:
            args = {
                item.attrib["name"]: item.attrib["value"]
                for item in group.findall("include/arg")
            }
            identifier = int(args["ID"])
            pose = ground["A%02d" % (identifier + 1)]
            gx, gy = pose["x"], pose["y"]
            self.assertEqual(group.attrib["ns"], "iris_%d" % identifier)
            self.assertAlmostEqual(float(args["x"]), 100.0 + gx)
            self.assertAlmostEqual(float(args["y"]), 100.0 + gy)
            self.assertEqual(int(args["mavlink_tcp_port"]), 4560 + identifier)
            self.assertEqual(int(args["udp_gimbal_port"]), 13030 + identifier)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, _manifest = generate_mission.generate(
            fleet_path=generate_mission.DEFAULT_FLEET,
            available=["1-16"],
            out_dir=self.out,
        )
        self.spec_path = self.out / "mission_spec.json"

    def run_scenario(self, observer=None, until=None, **faults):
        server = GroundStation(**faults)
        self.addCleanup(server.close)
        package = driver.load_package(self.spec_path)
        transport = driver.TcpCommandTransport(
            package, driver.SystemClock(), "127.0.0.1", server.port
        )
        self.addCleanup(transport.close)
        transport.wait_until_ready()
        logger = Logger()
        observer = observer or Observer()
        phases = tuple(
            dataclasses.replace(p, dwell_s=0)
            for p in driver.build_phases(package.spec)
        )
        if until:
            phases = phases[:until]
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = driver.run_sequence(
                package,
                transport,
                observer,
                logger,
                phases,
                len(driver.build_phases(package.spec)),
            )
        transport.close()
        server.thread.join(1)
        return result, package.context, server.received, logger.rows, observer

    def test_full_mission_initial9_then_spares_join(self):
        rc, context, messages, rows, observer = self.run_scenario()
        self.assertEqual(rc, 0)
        non_hold = [message for message in messages if message["command"] != "HOLD"]
        self.assertEqual(len(non_hold), 10)
        self.assertEqual(
            [message["command"] for message in non_hold],
            [
                "MOVE_TO",
                "MOVE_TO",
                "FOLLOW_ROUTE",
                "FAULT_EXIT",
                "MOVE_TO",
                "FOLLOW_ROUTE",
                "MOVE_TO",
                "MOVE_TO",
                "MOVE_TO",
                "FOLLOW_ROUTE",
            ],
        )
        sets = [{item["uav_id"] for item in message["assignments"]} for message in non_hold]
        self.assertEqual(sets[0] | sets[1], set(self.spec["initial_ids"]))
        self.assertEqual(sets[2], set(self.spec["initial_ids"]))
        self.assertEqual(sets[3], set(self.spec["faulted_ids"]))
        # All spares move in one batch again (recovery-spares), not one by one.
        self.assertEqual(sets[4], set(self.spec["spare_ids"]))
        self.assertEqual(sets[5], set(self.spec["core_ids"]))
        self.assertEqual(sets[6], set(self.spec["b_strike_ids"]))
        self.assertEqual(sets[7], {self.spec["b_failed_id"]})
        self.assertEqual(sets[8], {self.spec["b_reserve_id"]})
        self.assertEqual(sets[9], set(self.spec["core_ids"]))
        self.assertTrue(all(item["z"] == 5 for item in non_hold[6]["assignments"]))
        self.assertTrue(all(item["z"] == 5 for item in non_hold[7]["assignments"]))
        self.assertTrue(all(item["z"] == 5 for item in non_hold[8]["assignments"]))
        self.assertTrue(
            all(
                all(waypoint["z"] == 8 for waypoint in item["waypoints"])
                for item in non_hold[3]["assignments"]
            )
        )
        self.assertEqual(set(context.roster.failed_ids), set(self.spec["faulted_ids"]))
        self.assertEqual(
            set(context.roster.active_ids),
            set(self.spec["core_ids"]) | set(self.spec["b_ids"]),
        )
        self.assertFalse(context.roster.reserve_ids)
        events = [row["event_type"] for row in rows if row["event"] == "local_coordination"]
        self.assertEqual(events, ["GROUPS_READY", "RECON_COMPLETE", "STRIKE_COMPLETE"])
        self.assertEqual(rows[-1]["status"], "SUCCESS")
        expected_plans = {
            step.plan_id
            for phase in driver.build_phases(self.spec)
            for step in phase.steps
        }
        self.assertTrue(expected_plans.issubset(set(observer.checked)))
        self.assertFalse(observer.aborted)

    def test_b_drift_after_strike_stops_a_return(self):
        class DriftingObserver(Observer):
            def check_expected(self, plan_id):
                super().check_expected(plan_id)
                if self.checked.count("mock-b-hold") >= 2:
                    raise RuntimeError("模拟B组未保持目标上方5m")

        rc, _, messages, rows, observer = self.run_scenario(DriftingObserver())
        self.assertEqual(rc, 1)
        self.assertTrue(observer.aborted)
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 9)
        self.assertFalse(
            any(
                row.get("phase") == "return_a" and row.get("outcome") == "SUCCESS"
                for row in rows
            )
        )

    def test_fault_batch_failure_keeps_initial_and_spares(self):
        rc, context, messages, rows, observer = self.run_scenario(fail_at=4)
        self.assertEqual(rc, 1)
        self.assertTrue(observer.aborted)
        self.assertFalse(context.roster.failed_ids)
        self.assertEqual(
            set(context.roster.active_ids),
            set(self.spec["initial_ids"]) | set(self.spec["b_ids"]),
        )
        self.assertEqual(set(context.roster.reserve_ids), set(self.spec["spare_ids"]))
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 4)
        self.assertTrue(any(m["command"] == "HOLD" for m in messages))
        self.assertEqual(rows[-1]["status"], "FAILURE")

    def test_coverage2_failure_keeps_recovered_membership(self):
        rc, context, messages, rows, observer = self.run_scenario(fail_at=6)
        self.assertEqual(rc, 1)
        self.assertEqual(set(context.roster.failed_ids), set(self.spec["faulted_ids"]))
        self.assertEqual(
            set(context.roster.active_ids),
            set(self.spec["core_ids"]) | set(self.spec["b_ids"]),
        )
        self.assertFalse(context.roster.reserve_ids)
        self.assertFalse(
            any(row.get("event_type") == "RECON_COMPLETE" for row in rows)
        )
        self.assertTrue(observer.aborted)

    def test_false_completed_is_rejected_by_position_observer(self):
        rc, _, messages, rows, observer = self.run_scenario(
            Observer("coverage-segment-1")
        )
        self.assertEqual(rc, 1)
        self.assertTrue(observer.aborted)
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 3)

    def test_disconnect_aborts_and_does_not_continue(self):
        rc, _, messages, _, observer = self.run_scenario(disconnect_at=2)
        self.assertEqual(rc, 1)
        self.assertTrue(observer.aborted)
        self.assertEqual(len(messages), 2)

    def test_incomplete_completed_member_list_fails(self):
        rc, _, messages, _, observer = self.run_scenario(incomplete_at=1)
        self.assertEqual(rc, 1)
        self.assertTrue(observer.aborted)
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 1)

    def test_partial_run_is_not_reported_as_full_mission(self):
        rc, _, messages, rows, _ = self.run_scenario(until=1)
        self.assertEqual(rc, 0)
        self.assertEqual(len(messages), 2)
        self.assertEqual(rows[-1]["status"], "PARTIAL_SUCCESS")


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, _manifest = generate_mission.generate(
            fleet_path=generate_mission.DEFAULT_FLEET,
            available=["1-16"],
            out_dir=self.out,
        )
        self.package = driver.load_package(self.out / "mission_spec.json")
        guard = object.__new__(RosFlightGuard)
        guard.lock = threading.RLock()
        guard.ros = NS(is_shutdown=lambda: False)
        guard.package = self.package
        guard.spec = self.spec
        guard.ids = list(self.spec["all_ids"])
        guard.task_failure = None
        guard.samples = {}
        guard.receive(NS(data=True), "/group_a/calibration_ready")
        guard.receive(NS(data=math.pi / 2), "/group_a/reference_heading")
        self.guard = guard
        for uav in guard.ids:
            guard.receive(
                NS(connected=True, armed=True, mode="OFFBOARD"),
                "/UAV%d/mavros/state" % int(uav[1:]),
            )

    def set_poses(self, plan_id):
        for uav, pose in self.spec["expected_targets"][plan_id].items():
            self.guard.receive(
                NS(point=NS(x=pose["x"], y=pose["y"], z=pose["z"])),
                "/UAV%d/swarm_pose" % int(uav[1:]),
            )

    def test_state_and_expected_positions_pass(self):
        self.set_poses("prepare-a-1")
        self.guard.check()
        self.guard.check_expected("prepare-a-1")

    def test_b_at_twelve_meters_does_not_satisfy_five_meter_target(self):
        self.set_poses("mock-b-hold")
        self.guard.check_expected("mock-b-hold")
        first_b = self.spec["b_ids"][0]
        pose = self.spec["expected_targets"]["mock-b-hold"][first_b]
        self.guard.receive(
            NS(point=NS(x=pose["x"], y=pose["y"], z=12.0)),
            "/UAV%d/swarm_pose" % int(first_b[1:]),
        )
        with self.assertRaisesRegex(RuntimeError, "高度误差"):
            self.guard.check_expected("mock-b-hold")

    def test_manual_takeover_stops_mission(self):
        self.guard.receive(
            NS(connected=True, armed=True, mode="POSCTL"), "/UAV9/mavros/state"
        )
        with self.assertRaisesRegex(RuntimeError, "OFFBOARD"):
            self.guard.check()

    def test_submitted_task_failure_is_latched_but_old_mission_ignored(self):
        msg = NS(mission_id="old", status="FAILED", error_code="TEST", message="failed")
        self.guard.receive(msg, "/UAV6/uav_task_state")
        self.assertIsNone(self.guard.task_failure)
        msg.mission_id = self.package.context.mission_id
        self.guard.receive(msg, "/UAV6/uav_task_state")
        with self.assertRaisesRegex(RuntimeError, "TEST"):
            self.guard.check()


class CustomFleet14Tests(unittest.TestCase):
    """2026-10-21 事故后 14 机任务：1/2/4 + 6/16/8 + 11/12/13，损毁 4/8，补位 5/9。"""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, self.manifest = generate_mission.generate(
            fleet_path=CUSTOM_FLEET,
            available=CUSTOM_AVAILABLE,
            out_dir=self.out,
        )

    def test_roles_faults_spares_and_b(self):
        spec = self.spec
        self.assertEqual(len(spec["available_uavs"]), 14)
        self.assertEqual(
            set(spec["initial_ids"]),
            {"A01", "A02", "A04", "A06", "A08", "A11", "A12", "A13", "A16"},
        )
        self.assertEqual(spec["leader_id"], "A06")
        self.assertEqual(set(spec["faulted_ids"]), {"A04", "A08"})
        self.assertEqual(set(spec["spare_ids"]), {"A05", "A09"})
        self.assertEqual(
            set(spec["core_ids"]),
            {"A01", "A02", "A05", "A06", "A09", "A11", "A12", "A13", "A16"},
        )
        self.assertEqual(spec["b_ids"], ["A10", "A14", "A15"])
        self.assertEqual(spec["b_strike_ids"], ["A10", "A14"])
        self.assertEqual(spec["b_reserve_id"], "A15")
        self.assertEqual(spec["b_failed_id"], "A14")
        self.assertEqual(spec["b_final_ids"], ["A10", "A15"])
        self.assertEqual(len(spec["b_targets"]), 2)
        self.assertEqual(spec["b_targets"]["A10"]["x"], 10.0)
        self.assertEqual(spec["b_targets"]["A10"]["y"], 65.0)
        self.assertEqual(spec["b_targets"]["A14"]["x"], 0.0)
        self.assertEqual(spec["b_targets"]["A14"]["y"], 65.0)
        self.assertEqual(spec["b_failed_hover"]["x"], -6.0)
        self.assertEqual(spec["b_failed_hover"]["y"], 65.0)
        self.assertEqual(spec["b_failed_hover"]["z"], 5.0)
        self.assertEqual(
            set(spec["calibration"]["compute"]),
            {"UAV6", "UAV8", "UAV9", "UAV10", "UAV16"},
        )
        self.assertEqual(spec["calibration"]["row_center"], "UAV8")
        self.assertAlmostEqual(spec["calibration"]["shift_y"], 0.0)

    def test_plans_for_two_targets_and_reinforce(self):
        spec = self.spec
        self.assertEqual(
            set(spec["plan_members"]["mock-b-strike"]), {"A10", "A14"}
        )
        self.assertEqual(set(spec["plan_members"]["mock-b-fail"]), {"A14"})
        self.assertEqual(
            set(spec["plan_members"]["mock-b-reinforce"]), {"A15"}
        )
        self.assertEqual(
            set(spec["plan_members"]["mock-b-hold"]), {"A10", "A14", "A15"}
        )
        hold = spec["expected_targets"]["mock-b-hold"]
        self.assertEqual(hold["A14"], spec["b_failed_hover"])
        self.assertEqual(hold["A15"], spec["b_targets"]["A14"])
        self.assertEqual(hold["A10"], spec["b_targets"]["A10"])
        self.assertEqual(
            spec["expected_targets"]["mock-b-reinforce"]["A15"],
            spec["b_targets"]["A14"],
        )
        self.assertEqual(
            set(spec["expected_targets"]["hold-end"]), set(spec["all_ids"])
        )

    def test_target2_is_10m_left_of_target1(self):
        target1 = self.spec["b_targets"]["A10"]
        target2 = self.spec["b_targets"]["A14"]
        self.assertAlmostEqual(target2["y"], target1["y"])
        self.assertAlmostEqual(target2["x"], target1["x"] - 10.0)

    def test_fault_exit_route_clears_formation(self):
        plans = json.loads(
            (self.out / "package/plans/plans.json").read_text(encoding="utf-8")
        )["plans"]
        plan = next(item for item in plans if item["plan_id"] == "fault-exit")
        for uav in ("A04", "A08"):
            waypoints = plan["robot_assignments"][uav]["payload"]["waypoints"]
            self.assertEqual(len(waypoints), 3)
            self.assertTrue(all(abs(wp["z"] - 8.0) < 1e-9 for wp in waypoints))
            self.assertAlmostEqual(waypoints[1]["y"], waypoints[0]["y"] + 10.0)
            self.assertAlmostEqual(waypoints[2]["x"], -30.0)
            self.assertAlmostEqual(waypoints[2]["y"], waypoints[1]["y"])
            # 横向退出段位于编队 y 范围（<=66）之外。
            self.assertGreaterEqual(waypoints[1]["y"], 71.0)

    def test_onboard_commands_and_flow_env(self):
        starts = [
            line for line in (self.out / "onboard_commands.sh").read_text().splitlines()
            if "onboard_ctl.sh start" in line
        ]
        self.assertEqual(len(starts), 14)
        numbers = set()
        for line in starts:
            match = re.search(r"start (\d+) ", line)
            self.assertIsNotNone(match)
            number = int(match.group(1))
            numbers.add(number)
            intents = line.split("neighbor_intents:=")[1].split(
                " neighbor_odom_topics:="
            )[0]
            poses = line.split("neighbor_odom_topics:=")[1].split(
                " uav_task_timeout_s:="
            )[0]
            self.assertEqual(len(intents.split(",")), 13)
            self.assertEqual(len(poses.split(",")), 13)
            self.assertIn("bundle_transport:=udp", line)
            if number in (10, 14, 15):
                self.assertIn("layer_move_to:=8.0", line)
            else:
                self.assertNotIn("layer_move_to", line)
        self.assertEqual(numbers, {1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16})
        env = (self.out / "flow_env.sh").read_text()
        match = re.search(r"AIR_FLOW_UAVS='([^']*)'", env)
        self.assertIsNotNone(match)
        self.assertEqual(
            set(int(item) for item in match.group(1).split()),
            {1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16},
        )
        self.assertIn("AIR_FLOW_LEADER=6", env)

    def test_absent_uavs_are_skipped_and_rejected_when_available(self):
        fleet = generate_mission.load_fleet(CUSTOM_FLEET)
        grid = generate_mission.grid_map(fleet)
        self.assertEqual(len(grid), 14)
        self.assertNotIn("UAV3", grid)
        self.assertNotIn("UAV7", grid)
        self.assertEqual(grid["UAV4"], (0.0, 5.0))
        self.assertEqual(grid["UAV5"], (5.0, 5.0))
        self.assertEqual(grid["UAV16"], (-5.0, 0.0))
        self.assertEqual(grid["UAV13"], (0.0, -5.0))
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(generate_mission.GenerationError, "缺失机"):
                generate_mission.generate(
                    fleet_path=CUSTOM_FLEET,
                    available=["1-16"],
                    out_dir=Path(temp),
                )


class CustomProfileProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, _manifest = generate_mission.generate(
            fleet_path=CUSTOM_FLEET,
            available=CUSTOM_AVAILABLE,
            out_dir=self.out,
        )
        self.spec_path = self.out / "mission_spec.json"

    def run_scenario(self, observer=None):
        server = GroundStation()
        self.addCleanup(server.close)
        package = driver.load_package(self.spec_path)
        transport = driver.TcpCommandTransport(
            package, driver.SystemClock(), "127.0.0.1", server.port
        )
        self.addCleanup(transport.close)
        transport.wait_until_ready()
        logger = Logger()
        observer = observer or Observer()
        phases = tuple(
            dataclasses.replace(p, dwell_s=0)
            for p in driver.build_phases(package.spec)
        )
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = driver.run_sequence(
                package,
                transport,
                observer,
                logger,
                phases,
                len(driver.build_phases(package.spec)),
            )
        transport.close()
        server.thread.join(1)
        return result, package.context, server.received, logger.rows, observer

    def test_full_mission_two_targets_and_failure(self):
        rc, context, messages, rows, observer = self.run_scenario()
        self.assertEqual(rc, 0)
        non_hold = [message for message in messages if message["command"] != "HOLD"]
        self.assertEqual(len(non_hold), 10)
        sets = [{item["uav_id"] for item in message["assignments"]}
                for message in non_hold]
        self.assertEqual(sets[0] | sets[1], set(self.spec["initial_ids"]))
        self.assertEqual(sets[2], set(self.spec["initial_ids"]))
        self.assertEqual(sets[3], {"A04", "A08"})
        self.assertEqual(sets[4], {"A05", "A09"})
        self.assertEqual(sets[5], set(self.spec["core_ids"]))
        self.assertEqual(sets[6], {"A10", "A14"})
        self.assertEqual(sets[7], {"A14"})
        self.assertEqual(sets[8], {"A15"})
        self.assertEqual(sets[9], set(self.spec["core_ids"]))
        for index in (6, 7, 8):
            self.assertTrue(all(item["z"] == 5 for item in non_hold[index]["assignments"]))
        self.assertEqual(set(context.roster.failed_ids), {"A04", "A08"})
        self.assertEqual(
            set(context.roster.active_ids),
            set(self.spec["core_ids"]) | set(self.spec["b_ids"]),
        )
        self.assertFalse(context.roster.reserve_ids)
        self.assertEqual(rows[-1]["status"], "SUCCESS")
        self.assertFalse(observer.aborted)


class CustomProfileGuardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, _manifest = generate_mission.generate(
            fleet_path=CUSTOM_FLEET,
            available=CUSTOM_AVAILABLE,
            out_dir=self.out,
        )
        self.package = driver.load_package(self.out / "mission_spec.json")
        guard = object.__new__(RosFlightGuard)
        guard.lock = threading.RLock()
        guard.ros = NS(is_shutdown=lambda: False)
        guard.package = self.package
        guard.spec = self.spec
        guard.ids = list(self.spec["all_ids"])
        guard.task_failure = None
        guard.samples = {}
        guard.receive(NS(data=True), "/group_a/calibration_ready")
        guard.receive(NS(data=math.pi / 2), "/group_a/reference_heading")
        self.guard = guard
        for uav in guard.ids:
            guard.receive(
                NS(connected=True, armed=True, mode="OFFBOARD"),
                "/UAV%d/mavros/state" % int(uav[1:]),
            )

    def test_two_target_hold_includes_failed_and_reinforce(self):
        for uav, pose in self.spec["expected_targets"]["mock-b-hold"].items():
            self.guard.receive(
                NS(point=NS(x=pose["x"], y=pose["y"], z=pose["z"])),
                "/UAV%d/swarm_pose" % int(uav[1:]),
            )
        self.guard.check()
        self.guard.check_expected("mock-b-hold")
        self.assertEqual(
            self.spec["expected_targets"]["mock-b-hold"]["A14"],
            self.spec["b_failed_hover"],
        )
        self.assertEqual(
            self.spec["expected_targets"]["mock-b-hold"]["A15"],
            self.spec["b_targets"]["A14"],
        )


class CustomFleet11Tests(unittest.TestCase):
    """11 机 2×3 编队：6/8/9 + 10/11/12，损毁 10/11，补位 13/14，B 15/1/2。"""

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, self.manifest = generate_mission.generate(
            fleet_path=CUSTOM_FLEET_11,
            available=CUSTOM_AVAILABLE_11,
            out_dir=self.out,
        )

    def test_roles_and_targets(self):
        spec = self.spec
        self.assertEqual(len(spec["available_uavs"]), 11)
        self.assertEqual(
            set(spec["initial_ids"]),
            {"A06", "A08", "A09", "A10", "A11", "A12"},
        )
        self.assertEqual(spec["leader_id"], "A06")
        self.assertEqual(set(spec["faulted_ids"]), {"A10", "A11"})
        self.assertEqual(set(spec["spare_ids"]), {"A13", "A14"})
        self.assertEqual(
            set(spec["core_ids"]),
            {"A06", "A08", "A09", "A12", "A13", "A14"},
        )
        self.assertEqual(spec["b_ids"], ["A02", "A15", "A16"])
        self.assertEqual(spec["b_strike_ids"], ["A15", "A16"])
        self.assertEqual(spec["b_reserve_id"], "A02")
        self.assertEqual(spec["b_failed_id"], "A15")
        self.assertEqual(spec["b_final_ids"], ["A02", "A16"])
        self.assertEqual(spec["b_targets"]["A15"]["x"], 10.0)
        self.assertEqual(spec["b_targets"]["A15"]["y"], 65.0)
        self.assertEqual(spec["b_targets"]["A16"]["x"], 0.0)
        self.assertEqual(spec["b_targets"]["A16"]["y"], 65.0)
        self.assertEqual(spec["b_failed_hover"]["x"], 4.0)
        self.assertEqual(spec["b_failed_hover"]["y"], 65.0)
        self.assertEqual(
            set(spec["calibration"]["compute"]),
            {"UAV6", "UAV8", "UAV9"},
        )
        self.assertEqual(spec["calibration"]["row_center"], "UAV8")
        self.assertEqual(spec["calibration"]["center_x"], -5.0)
        self.assertAlmostEqual(spec["calibration"]["shift_y"], -5.0)
        self.assertEqual(len(spec["geometry_checks"]["coverage_scan_rects"]), 3)

    def test_fault_exit_and_recovery_slots(self):
        # 任务坐标以标定排中心 UAV8 为原点（shift_x=+5, shift_y=-5）。
        self.assertEqual(
            self.spec["ground_positions"]["A13"],
            {"x": -5.0, "y": -10.0, "z": 0.0, "yaw": 0.0},
        )
        faults = self.spec["expected_targets"]["fault-exit"]
        self.assertEqual(faults["A10"], {"x": -35.0, "y": 56.0, "z": 8.0, "yaw": 0.0})
        self.assertEqual(faults["A11"], {"x": -30.0, "y": 56.0, "z": 8.0, "yaw": 0.0})
        recovery = self.spec["expected_targets"]["recovery-spares"]
        self.assertEqual(recovery["A13"], {"x": -5.0, "y": 56.0, "z": 12.0, "yaw": 0.0})
        self.assertEqual(recovery["A14"], {"x": 0.0, "y": 56.0, "z": 12.0, "yaw": 0.0})
        self.assertEqual(
            set(self.spec["plan_members"]["mock-b-strike"]), {"A15", "A16"}
        )
        self.assertEqual(set(self.spec["plan_members"]["mock-b-fail"]), {"A15"})
        self.assertEqual(
            set(self.spec["plan_members"]["mock-b-reinforce"]), {"A02"}
        )
        self.assertEqual(
            set(self.spec["plan_members"]["mock-b-hold"]), {"A02", "A15", "A16"}
        )

    def test_onboard_commands_and_flow_env(self):
        starts = [
            line for line in (self.out / "onboard_commands.sh").read_text().splitlines()
            if "onboard_ctl.sh start" in line
        ]
        self.assertEqual(len(starts), 11)
        numbers = set()
        for line in starts:
            match = re.search(r"start (\d+) ", line)
            self.assertIsNotNone(match)
            number = int(match.group(1))
            numbers.add(number)
            intents = line.split("neighbor_intents:=")[1].split(
                " neighbor_odom_topics:="
            )[0]
            poses = line.split("neighbor_odom_topics:=")[1].split(
                " uav_task_timeout_s:="
            )[0]
            self.assertEqual(len(intents.split(",")), 10)
            self.assertEqual(len(poses.split(",")), 10)
            if number in (15, 16, 2):
                self.assertIn("layer_move_to:=8.0", line)
            else:
                self.assertNotIn("layer_move_to", line)
        self.assertEqual(numbers, {2, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16})
        env = (self.out / "flow_env.sh").read_text()
        match = re.search(r"AIR_FLOW_UAVS='([^']*)'", env)
        self.assertIsNotNone(match)
        self.assertEqual(
            set(int(item) for item in match.group(1).split()),
            {2, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16},
        )
        self.assertIn("AIR_FLOW_LEADER=6", env)

    def test_grid_absent_uavs(self):
        fleet = generate_mission.load_fleet(CUSTOM_FLEET_11)
        grid = generate_mission.grid_map(fleet)
        self.assertEqual(len(grid), 11)
        for uav in ("UAV1", "UAV3", "UAV4", "UAV5", "UAV7"):
            self.assertNotIn(uav, grid)
        self.assertEqual(grid["UAV6"], (-10.0, 5.0))
        self.assertEqual(grid["UAV10"], (-10.0, 0.0))
        self.assertEqual(grid["UAV15"], (0.0, -5.0))
        self.assertEqual(grid["UAV16"], (5.0, -5.0))
        self.assertEqual(grid["UAV2"], (10.0, -5.0))
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(generate_mission.GenerationError, "缺失机"):
                generate_mission.generate(
                    fleet_path=CUSTOM_FLEET_11,
                    available=["1-16"],
                    out_dir=Path(temp),
                )

    def test_sitl_spawn_applies_shift_y(self):
        spawn = self.out / "spawn.launch"
        generate_spawn.generate(
            self.out / "mission_spec.json", spawn, CUSTOM_FLEET_11
        )
        ground = self.spec["ground_positions"]
        for group in ET.parse(spawn).getroot().findall("group"):
            args = {
                item.attrib["name"]: item.attrib["value"]
                for item in group.findall("include/arg")
            }
            pose = ground["A%02d" % (int(args["ID"]) + 1)]
            self.assertAlmostEqual(float(args["x"]), 100.0 + pose["x"])
            self.assertAlmostEqual(float(args["y"]), 100.0 + pose["y"])
        # 标定排在 y=+5，shift_y=-5：A13（第 3 排）出生 y 必须为 -10。
        self.assertEqual(ground["A13"]["y"], -10.0)


class CustomProfile11ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, _manifest = generate_mission.generate(
            fleet_path=CUSTOM_FLEET_11,
            available=CUSTOM_AVAILABLE_11,
            out_dir=self.out,
        )
        self.spec_path = self.out / "mission_spec.json"

    def run_scenario(self):
        server = GroundStation()
        self.addCleanup(server.close)
        package = driver.load_package(self.spec_path)
        transport = driver.TcpCommandTransport(
            package, driver.SystemClock(), "127.0.0.1", server.port
        )
        self.addCleanup(transport.close)
        transport.wait_until_ready()
        logger = Logger()
        observer = Observer()
        phases = tuple(
            dataclasses.replace(p, dwell_s=0)
            for p in driver.build_phases(package.spec)
        )
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = driver.run_sequence(
                package,
                transport,
                observer,
                logger,
                phases,
                len(driver.build_phases(package.spec)),
            )
        transport.close()
        server.thread.join(1)
        return result, package.context, server.received, logger.rows, observer

    def test_full_mission_2x3(self):
        rc, context, messages, rows, observer = self.run_scenario()
        self.assertEqual(rc, 0)
        non_hold = [message for message in messages if message["command"] != "HOLD"]
        self.assertEqual(len(non_hold), 10)
        sets = [{item["uav_id"] for item in message["assignments"]}
                for message in non_hold]
        self.assertEqual(sets[0] | sets[1], set(self.spec["initial_ids"]))
        self.assertEqual(sets[2], set(self.spec["initial_ids"]))
        self.assertEqual(sets[3], {"A10", "A11"})
        self.assertEqual(sets[4], {"A13", "A14"})
        self.assertEqual(sets[5], set(self.spec["core_ids"]))
        self.assertEqual(sets[6], {"A15", "A16"})
        self.assertEqual(sets[7], {"A15"})
        self.assertEqual(sets[8], {"A02"})
        self.assertEqual(sets[9], set(self.spec["core_ids"]))
        for index in (6, 7, 8):
            self.assertTrue(all(item["z"] == 5 for item in non_hold[index]["assignments"]))
        self.assertEqual(set(context.roster.failed_ids), {"A10", "A11"})
        self.assertEqual(
            set(context.roster.active_ids),
            set(self.spec["core_ids"]) | set(self.spec["b_ids"]),
        )
        self.assertFalse(context.roster.reserve_ids)
        self.assertEqual(rows[-1]["status"], "SUCCESS")
        self.assertFalse(observer.aborted)


class CustomProfile11GuardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.out = Path(self.temp.name)
        self.spec, _manifest = generate_mission.generate(
            fleet_path=CUSTOM_FLEET_11,
            available=CUSTOM_AVAILABLE_11,
            out_dir=self.out,
        )
        self.package = driver.load_package(self.out / "mission_spec.json")
        guard = object.__new__(RosFlightGuard)
        guard.lock = threading.RLock()
        guard.ros = NS(is_shutdown=lambda: False)
        guard.package = self.package
        guard.spec = self.spec
        guard.ids = list(self.spec["all_ids"])
        guard.task_failure = None
        guard.samples = {}
        guard.receive(NS(data=True), "/group_a/calibration_ready")
        guard.receive(NS(data=math.pi / 2), "/group_a/reference_heading")
        self.guard = guard
        for uav in guard.ids:
            guard.receive(
                NS(connected=True, armed=True, mode="OFFBOARD"),
                "/UAV%d/mavros/state" % int(uav[1:]),
            )

    def test_b_hold_positions(self):
        for uav, pose in self.spec["expected_targets"]["mock-b-hold"].items():
            self.guard.receive(
                NS(point=NS(x=pose["x"], y=pose["y"], z=pose["z"])),
                "/UAV%d/swarm_pose" % int(uav[1:]),
            )
        self.guard.check()
        self.guard.check_expected("mock-b-hold")
        hold = self.spec["expected_targets"]["mock-b-hold"]
        self.assertEqual(hold["A15"], self.spec["b_failed_hover"])
        self.assertEqual(hold["A02"], self.spec["b_targets"]["A15"])
        self.assertEqual(hold["A16"], self.spec["b_targets"]["A16"])


if __name__ == "__main__":
    unittest.main()
