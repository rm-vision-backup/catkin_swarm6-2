"""离线配置/几何、真实TCP回环、阶段中止与只读观测器验证；不启动ROS/PX4。"""
import dataclasses
import io
import json
import math
import socket
import threading
import time
import unittest
from contextlib import redirect_stdout, redirect_stderr
from types import SimpleNamespace as NS
from xml.etree import ElementTree as ET

import yaml

from verification.six_uav_joint import driver
from verification.six_uav_joint.flight_guard import RosFlightGuard


class Logger:
    def __init__(self):
        self.rows = []

    def write(self, row):
        self.rows.append(row)


class Observer:
    """测试替身；绝不出现在命令行执行路径中。"""
    def __init__(self, fail_target=None):
        self.fail_target = fail_target
        self.checked = []
        self.aborted = False

    def wait_ready(self):
        pass

    def check(self):
        pass

    def check_targets(self, plan_id):
        self.checked.append(plan_id)
        if plan_id == self.fail_target:
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
                    index = sum(m["command"] != "HOLD" for m in self.received)
                    if index == self.disconnect_at:
                        self.client.shutdown(socket.SHUT_RDWR)
                        return
                    ids = [item["uav_id"] for item in message["assignments"]]
                    base = {k: message[k] for k in ("version", "mission_id", "group_id", "command_id")}
                    base["type"] = "STATUS"
                    replies = [dict(base, status="ACCEPTED")]
                    if index == self.fail_at:
                        replies.append(dict(base, status="FAILED", failed_uavs=[ids[-1]],
                                            error_code="TEST_FAILURE", message="测试拒绝"))
                    else:
                        replies.append(dict(base, status="COMPLETED",
                                            completed_uavs=ids[:-1] if index == self.incomplete_at else ids))
                    # 一个write中发送两条完整行，覆盖连续反馈解析。
                    self.client.sendall(b"".join((json.dumps(row)+"\n").encode() for row in replies))
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


class ProtocolTests(unittest.TestCase):
    def run_scenario(self, observer=None, until=None, confirm=None, **faults):
        server = GroundStation(**faults)
        self.addCleanup(server.close)
        package = driver.load_package()
        transport = driver.TcpCommandTransport(package, driver.SystemClock(),
                                               "127.0.0.1", server.port)
        self.addCleanup(transport.close)
        transport.wait_until_ready()
        logger = Logger()
        observer = observer or Observer()
        phases = tuple(dataclasses.replace(p, dwell_s=0) for p in driver.PHASES)
        if until:
            phases = phases[:until]
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = driver.run_sequence(package, transport, observer, logger, phases, confirm)
        # 关闭写端确保服务端收齐最后发出的 HOLD 再断言。
        transport.close()
        server.thread.join(1)
        return result, package.context, server.received, logger.rows, observer

    def test_full_mission_switches_members_and_triggers_mock_b(self):
        rc, context, messages, rows, observer = self.run_scenario()
        self.assertEqual(rc, 0)
        self.assertEqual(len(messages), 7)
        self.assertEqual({m["group_id"] for m in messages}, {"GroupA"})
        self.assertEqual([m["command"] for m in messages],
                         ["MOVE_TO", "FOLLOW_ROUTE", "FAULT_EXIT", "MOVE_TO", "FOLLOW_ROUTE",
                          "MOVE_TO", "MOVE_TO"])
        sets = [{a["uav_id"] for a in m["assignments"]} for m in messages]
        self.assertEqual(sets[1], {"A06", "A07", "A08"})
        self.assertEqual(sets[4], {"A07", "A08", "A11"})
        self.assertEqual(sets[3], {"A11"})
        self.assertEqual(sets[5], {"A09", "A10"})
        self.assertEqual(sets[6], {"A07", "A08", "A11"})
        for assignment in messages[5]["assignments"]:
            self.assertEqual(assignment["z"], 5)
        self.assertTrue(all("A06" not in members for members in sets[3:]))
        self.assertEqual(context.roster.failed_ids, ["A06"])
        self.assertEqual(set(context.roster.active_ids), {"A07", "A08", "A09", "A10", "A11"})
        self.assertFalse(context.roster.reserve_ids)
        events = [r["event_type"] for r in rows if r["event"] == "local_coordination"]
        self.assertEqual(events, ["GROUPS_READY", "RECON_COMPLETE", "STRIKE_COMPLETE"])
        self.assertEqual(rows[-1]["status"], "SUCCESS")
        self.assertEqual(set(observer.checked), {phase.plan_id for phase in driver.PHASES})
        self.assertFalse(observer.aborted)

    def test_b_drift_after_strike_stops_a_return(self):
        class DriftingObserver(Observer):
            def check_targets(self, plan_id):
                super().check_targets(plan_id)
                if self.checked.count("mock-b-hold") >= 2:
                    raise RuntimeError("模拟B组未保持目标上方5m")
        rc, _, messages, rows, observer = self.run_scenario(DriftingObserver())
        self.assertEqual(rc, 1)
        self.assertTrue(observer.aborted)
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 6)
        self.assertFalse(any(r.get("phase") == "return_a" and r.get("outcome") == "SUCCESS" for r in rows))

    def test_exit_failure_does_not_activate_reserve_or_run_b(self):
        rc, context, messages, rows, observer = self.run_scenario(fail_at=3)
        self.assertEqual(rc, 1)
        self.assertFalse(context.roster.failed_ids)
        self.assertEqual(context.roster.reserve_ids, ["A11"])
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 3)
        self.assertTrue(any(m["command"] == "HOLD" for m in messages))
        self.assertTrue(observer.aborted)
        self.assertEqual(rows[-1]["status"], "FAILURE")

    def test_recovery_failure_keeps_failed_member_out(self):
        rc, context, messages, rows, _ = self.run_scenario(fail_at=4)
        self.assertEqual(rc, 1)
        self.assertEqual(context.roster.failed_ids, ["A06"])
        self.assertEqual(context.roster.reserve_ids, ["A11"])
        self.assertFalse(any(r.get("event_type") == "RECON_COMPLETE" for r in rows))

    def test_false_completed_is_rejected_by_position_observer(self):
        rc, _, messages, rows, observer = self.run_scenario(Observer("coverage-segment-1"))
        self.assertEqual(rc, 1)
        self.assertTrue(observer.aborted)
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 2)

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
        self.assertEqual(len(messages), 1)
        self.assertEqual(rows[-1]["status"], "PARTIAL_SUCCESS")

    def test_operator_abort_before_recovery_holds_all_and_keeps_reserve(self):
        def confirm(name, prompt):
            if name == "recovery":
                raise KeyboardInterrupt()
        rc, context, messages, rows, observer = self.run_scenario(confirm=confirm)
        self.assertEqual(rc, 1)
        self.assertEqual(context.roster.reserve_ids, ["A11"])
        self.assertTrue(observer.aborted)
        self.assertEqual(len([m for m in messages if m["command"] != "HOLD"]), 3)

    def test_timeout_stops_before_next_phase(self):
        package = driver.load_package()
        batch = driver.build_batch(package.context, package.context.plans["prepare-all"],
                                   package.context.mission_id, "timeout-test", 1, driver.SystemClock())
        phase = dataclasses.replace(driver.PHASES[0], timeout_s=0)
        transport = NS(poll_feedback=lambda: [])
        self.assertEqual(driver.wait_phase(transport, batch, phase, Observer(), Logger()), "TIMEOUT")


class GeometryTests(unittest.TestCase):
    def test_ground_targets_and_b_final_hover_are_separate_heights(self):
        package = driver.load_package()
        self.assertNotIn("mock-b-return", package.context.plans)
        for uav, name in (("A09", "mock-target-9"), ("A10", "mock-target-10")):
            target = package.world.targets[name]
            self.assertEqual(target["z"], 0)
            for plan_id in ("mock-b-strike", "mock-b-hold", "hold-end"):
                pose = package.context.plans[plan_id].robot_assignments[uav].payload["target_pose"]
                self.assertEqual((pose["x"], pose["y"], pose["z"]), (target["x"], target["y"], 5))

    def test_prepare_positions_match_user_grid(self):
        context = driver.load_package().context
        expected = {"A06": (-10, 0), "A07": (-5, 0), "A08": (0, 0),
                    "A09": (5, 0), "A10": (10, 0), "A11": (-10, -5)}
        for name, (x, y) in expected.items():
            pose = context.plans["prepare-all"].robot_assignments[name].payload["target_pose"]
            self.assertEqual((pose["x"], pose["y"], pose["z"]), (x, y, 12))

    def test_collinear_calibration_median_stays_at_uav8_for_any_heading(self):
        import statistics
        for degree in range(0, 360, 7):
            angle = math.radians(degree)
            east = [x*math.sin(angle) for x in (-10, -5, 0, 5, 10)]
            north = [-x*math.cos(angle) for x in (-10, -5, 0, 5, 10)]
            self.assertAlmostEqual(statistics.median(east), 0)
            self.assertAlmostEqual(statistics.median(north), 0)
        cfg = yaml.safe_load((driver.HERE / "config/calibration.yaml").read_text())
        self.assertEqual(cfg["calibration_uavs"], ["UAV6", "UAV7", "UAV8", "UAV9", "UAV10"])
        self.assertIn("UAV11", cfg["all_uavs"])

    def test_scan_segments_are_continuous_and_formations_match(self):
        context = driver.load_package().context
        formations = yaml.safe_load((driver.HERE / "config/formation_offsets.yaml").read_text())["formations"]
        for plan_id, formation_name in (("coverage-segment-1", "recon_initial"),
                                         ("coverage-segment-2", "recon_recovered")):
            plan = context.plans[plan_id]
            formation = formations[formation_name]
            self.assertEqual(set(plan.robot_assignments), set(formation["participants"]))
            for uav, offset in formation["offsets"].items():
                self.assertEqual(offset[1:], [0., 0.])
                self.assertEqual(offset[0], 0 if uav == "A07" else -5 if uav in {"A06", "A11"} else 5)
        first = context.plans["coverage-segment-1"].robot_assignments["A07"].payload["waypoints"][-1]
        second = context.plans["coverage-segment-2"].robot_assignments["A07"].payload["waypoints"][0]
        self.assertEqual(first, second)
        self.assertEqual(dict(first, x=first["x"]-5), context.plans["recovery-a"].robot_assignments["A11"].payload["target_pose"])

    def test_waypoints_and_follower_envelope_stay_inside_margin(self):
        package = driver.load_package()
        bounds = package.world.flight_area.model_dump()
        def inside(pose):
            self.assertGreaterEqual(pose["x"], bounds["min_x"] + bounds["safety_margin_m"])
            self.assertLessEqual(pose["x"], bounds["max_x"] - bounds["safety_margin_m"])
            self.assertGreaterEqual(pose["y"], bounds["min_y"] + bounds["safety_margin_m"])
            self.assertLessEqual(pose["y"], bounds["max_y"] - bounds["safety_margin_m"])
        for plan in package.context.plans.values():
            for assignment in plan.robot_assignments.values():
                for pose in assignment.payload.get("waypoints", []):
                    inside(pose)
                    if assignment.command_type == "FOLLOW_ROUTE":
                        for dx in (-5, 5):
                            inside(dict(pose, x=pose["x"]+dx))
                if "target_pose" in assignment.payload:
                    inside(assignment.payload["target_pose"])

    def test_recovery_and_return_linear_routes_have_five_meter_clearance(self):
        # 独立扫查关键转场的直线几何；不把它当成 EGO 实际曲线验收。
        context = driver.load_package().context
        for plan_id, starts in (
            ("recovery-a", {"A07": (-5, 15), "A08": (0, 15), "A11": (-10, -5)}),
            ("return-a", {"A07": (-5, 30), "A08": (0, 30), "A11": (-10, 30)}),
        ):
            for i in range(101):
                t = i/100
                points = []
                for uav, (x, y) in starts.items():
                    assignment = context.plans[plan_id].robot_assignments.get(uav)
                    end = assignment.payload["target_pose"] if assignment else {"x": x, "y": y}
                    points.append((x+(end["x"]-x)*t, y+(end["y"]-y)*t))
                for a in range(3):
                    for b in range(a+1, 3):
                        self.assertGreaterEqual(math.hypot(points[a][0]-points[b][0], points[a][1]-points[b][1]), 5)

    def test_sitl_spawn_matches_real_geometry_and_system_ids(self):
        root = ET.parse(driver.HERE / "sitl/six_uav_spawn.launch").getroot()
        self.assertEqual(len(root.findall("group")), 6)
        for group in root.findall("group"):
            args = {a.attrib["name"]: a.attrib["value"] for a in group.findall("include/arg")}
            n = int(args["ID"]) + 1
            self.assertEqual(group.attrib["ns"], "iris_%d" % (n-1))
            self.assertAlmostEqual(float(args["Y"]), math.pi/2)
            pose = driver.load_package().context.plans["prepare-all"].robot_assignments["A%02d" % n].payload["target_pose"]
            self.assertEqual((float(args["x"])-100, float(args["y"])-100), (pose["x"], pose["y"]))


class GuardTests(unittest.TestCase):
    def setUp(self):
        guard = object.__new__(RosFlightGuard)
        guard.lock = threading.RLock()
        guard.ros = NS(is_shutdown=lambda: False)
        guard.package = driver.load_package()
        guard.task_failure = None
        guard.samples = {}
        guard.receive(NS(data=True), "/group_a/calibration_ready")
        guard.receive(NS(data=math.pi/2), "/group_a/reference_heading")
        for uav, assignment in guard.package.context.plans["prepare-all"].robot_assignments.items():
            p = assignment.payload["target_pose"]
            prefix = "/UAV%d/" % int(uav[1:])
            guard.receive(NS(connected=True, armed=True, mode="OFFBOARD"), prefix + "mavros/state")
            guard.receive(NS(point=NS(x=p["x"], y=p["y"], z=5)), prefix + "swarm_pose")
        self.guard = guard

    def test_state_pass(self):
        self.guard.check()

    def test_manual_takeover_stops_mission(self):
        self.guard.receive(NS(connected=True, armed=True, mode="POSCTL"), "/UAV9/mavros/state")
        with self.assertRaisesRegex(RuntimeError, "OFFBOARD"):
            self.guard.check()

    def test_fake_completion_at_ground_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, "高度误差"):
            self.guard.check_targets("prepare-all")

    def test_b_hover_at_twelve_meters_does_not_satisfy_five_meter_target(self):
        for uav, a in self.guard.package.context.plans["mock-b-hold"].robot_assignments.items():
            p = a.payload["target_pose"]
            self.guard.receive(NS(point=NS(x=p["x"], y=p["y"], z=5)),
                               "/UAV%d/swarm_pose" % int(uav[1:]))
        self.guard.check_targets("mock-b-hold")
        self.guard.receive(NS(point=NS(x=5, y=15, z=12)), "/UAV9/swarm_pose")
        with self.assertRaisesRegex(RuntimeError, "高度误差"):
            self.guard.check_targets("mock-b-hold")

    def test_submitted_task_failure_is_latched_but_old_mission_ignored(self):
        msg = NS(mission_id="old", status="FAILED", error_code="TEST", message="failed")
        self.guard.receive(msg, "/UAV6/uav_task_state")
        self.assertIsNone(self.guard.task_failure)
        msg.mission_id = self.guard.package.context.mission_id
        self.guard.receive(msg, "/UAV6/uav_task_state")
        with self.assertRaisesRegex(RuntimeError, "TEST"):
            self.guard.check()


if __name__ == "__main__":
    unittest.main()
