"""Offline-only checks for the ten-aircraft topology experiment."""

import json
import subprocess
import sys
import tempfile
import unittest
import itertools
from contextlib import closing
from pathlib import Path
from types import SimpleNamespace
from xml.etree import ElementTree
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parent))

from core import components, distance, enu_to_task
from pipeline import Recorder
from sim_multi import SimCase, plan, recover_together, run_sitl, synchronous_distance
from sim_setup import prepare

DIRECTORY = Path(__file__).resolve().parent
ROOT = DIRECTORY.parents[1]
SCENARIO = DIRECTORY / "scenarios/10.sitl.json"
IDENTITY = ROOT / "src/tcp_to_ros/config/uav_identity_map.yaml"
TOPOLOGY = ROOT / "src/swarm_topology_bridge/config/topology_group_a_sim.yaml"


class RecoveryFleet:
    """Kinematic task fake: no ROS, control, collision avoidance or PX4 claims."""

    def __init__(self, scenario, undershoot=0.0, stuck=False):
        self.poses = {uav: (*point, scenario.fault_height if uav in scenario.fault_ids
                            else scenario.normal_height) for uav, point in scenario.layout.items()}
        self.commands = []
        self.health_checks = 0
        self.connected_checks = 0
        self.scenario = scenario
        self.undershoot = undershoot
        self.stuck = stuck

    def assert_healthy(self):
        self.health_checks += 1

    def snapshot(self):
        return {"poses_task": dict(self.poses)}

    def send_goal(self, command, targets, timeout_s=180.0):
        self.commands.append(dict(targets))
        if self.stuck:
            return
        for uav, target in targets.items():
            start = self.poses[uav]
            length = distance(start[:2], target[:2])
            fraction = max(0.0, 1 - self.undershoot / length) if length else 1.0
            self.poses[uav] = (*(start[i] + fraction * (target[i] - start[i])
                                 for i in range(2)), target[2])

    def wait_connected(self):
        self.connected_checks += 1
        if len(components({uav: self.poses[uav][:2] for uav in self.scenario.survivors},
                          self.scenario.radius)) != 1:
            raise RuntimeError("measured graph disconnected")


class TenAircraftSimulationTest(unittest.TestCase):
    def test_sitl_wires_one_solve_to_whole_fleet_recovery_and_holds_on_failure(self):
        for fail in (False, True):
            with self.subTest(fail=fail), tempfile.TemporaryDirectory() as temporary:
                scenario = SimCase.load(SCENARIO, 3)
                fleet = RecoveryFleet(scenario)
                fleet.poses = {uav: (*point[:2], 5.0) for uav, point in fleet.poses.items()}
                for method in ("wait_ready", "activate", "remove_fault", "close", "hold_all"):
                    setattr(fleet, method, Mock())
                fleet.fault_time = 0.0
                fleet._last_min_distance = 6.0
                constructor = Mock(return_value=fleet)
                output = json.loads((DIRECTORY / "fixtures/ten_three_targets_fixture.json")
                                    .read_text())
                output["dag_estimated_time"] = 7.0
                # Exercise SITL orchestration without importing ROS, sleeping or starting PX4.
                with closing(Recorder(Path(temporary) / "run")) as recorder, \
                        patch.dict(sys.modules, {"ros_adapter": SimpleNamespace(RosFleet=constructor)}), \
                        patch("sim_multi.time.monotonic", side_effect=itertools.count(1.0)), \
                        patch("sim_multi.time.sleep"), \
                        patch("sim_multi.run_solver", return_value=output) as solver, \
                        patch("sim_multi.recover_together", wraps=recover_together,
                              side_effect=RuntimeError("recovery failed") if fail else None):
                    if fail:
                        with self.assertRaisesRegex(RuntimeError, "recovery failed"):
                            run_sitl(scenario, IDENTITY, recorder, sys.executable, Path("model.pt"))
                        fleet.hold_all.assert_called_once()
                        self.assertEqual(json.loads((recorder.run_dir / "summary.json")
                                                    .read_text())["status"], "FAILED")
                    else:
                        result = run_sitl(scenario, IDENTITY, recorder, sys.executable, Path("model.pt"))
                        self.assertEqual(result["recovery_rounds"], 4)
                        self.assertTrue(all(set(command) == set(scenario.survivors)
                                            for command in fleet.commands[-4:]))
                        fleet.hold_all.assert_not_called()
                    solver.assert_called_once()
                    self.assertEqual(constructor.call_args.kwargs["max_batch"], 10)
                    fleet.close.assert_called_once()

    def test_one_plan_all_survivors_move_in_short_rounds(self):
        for count, label in ((3, "three"), (4, "four")):
            for undershoot in (0.0, 0.5):
                with self.subTest(faults=count, undershoot=undershoot), \
                        tempfile.TemporaryDirectory() as temporary:
                    scenario = SimCase.load(SCENARIO, count)
                    fixture = DIRECTORY / ("fixtures/ten_%s_targets_fixture.json" % label)
                    fleet = RecoveryFleet(scenario, undershoot=undershoot)
                    with closing(Recorder(Path(temporary) / "run")) as recorder:
                        # A single solve produces the final goals; recovery must never solve again.
                        output = json.loads(fixture.read_text())
                        output["dag_estimated_time"] = 7.0
                        with patch("sim_multi.run_solver", return_value=output) as solver:
                            targets = plan(scenario, scenario.layout, recorder, sys.executable,
                                           checkpoint=Path("unused-model.pt"))
                            result = recover_together(scenario, fleet, targets, recorder)
                            self.assertEqual(solver.call_count, 1)
                        events = [json.loads(line) for line in
                                  (recorder.run_dir / "events.jsonl").read_text().splitlines()]
                    rounds = [row for row in events if row["type"] == "recovery_round_started"]
                    self.assertGreaterEqual(len(rounds), 4)
                    self.assertEqual(len(rounds), result["recovery_rounds"])
                    for command, row in zip(fleet.commands, rounds):
                        self.assertEqual(set(command), set(scenario.survivors))
                        self.assertTrue(all(distance(row["starts_task_xy"][uav], target[:2])
                                            <= 2.0 + 1e-9 for uav, target in command.items()))
                    self.assertEqual(len(rounds[0]["components"]), 2)
                    self.assertEqual(fleet.connected_checks, 1)
                    self.assertLessEqual(result["max_target_error_m"], 1.0)
                    for fault in scenario.fault_ids:
                        self.assertEqual(fleet.poses[fault][:2], scenario.layout[fault])

    def test_simultaneous_step_allows_merge_but_rejects_crossing(self):
        scenario = SimCase.load(SCENARIO, 3)
        goals = json.loads((DIRECTORY / "fixtures/ten_three_targets_fixture.json")
                           .read_text())["targets_task_xy"]
        faults = {uav: scenario.layout[uav] for uav in scenario.fault_ids}
        starts = {uav: scenario.layout[uav] for uav in scenario.survivors}
        # A short intermediate step may still be disconnected.
        intermediate = {uav: (point[0] + (2 if point[0] < 0 else -2), point[1])
                        for uav, point in starts.items()}
        self.assertEqual(len(components(intermediate, 8)), 2)
        scenario.validate_recovery_step(starts, intermediate, faults)
        # The legacy two-group condition is inapplicable after merging.
        with self.assertRaises(ValueError):
            scenario.movement_batches(goals)
        scenario.validate_recovery_step(goals, goals, faults)
        crossing = {**starts, "A01": starts["A07"], "A07": starts["A01"]}
        for invalid in (crossing, {**goals, "A01": (100, 0)},
                        {**goals, "A01": (float("nan"), 0)},
                        {uav: point for uav, point in goals.items() if uav != "A01"}):
            with self.subTest(targets=invalid), self.assertRaises(ValueError):
                scenario.validate_recovery_step(starts, invalid, faults)

    def test_recovery_stall_limit_and_watchdog_failures_propagate(self):
        scenario = SimCase.load(SCENARIO, 3)
        goals = json.loads((DIRECTORY / "fixtures/ten_three_targets_fixture.json")
                           .read_text())["targets_task_xy"]
        for limit, message, commands in ((30, "stalled", 3), (2, "round limit", 2)):
            with self.subTest(limit=limit), tempfile.TemporaryDirectory() as temporary:
                fleet = RecoveryFleet(scenario, stuck=True)
                with closing(Recorder(Path(temporary) / "run")) as recorder:
                    with self.assertRaisesRegex(RuntimeError, message):
                        recover_together(scenario, fleet, goals, recorder, max_rounds=limit)
                self.assertEqual(len(fleet.commands), commands)
                self.assertEqual(fleet.connected_checks, 0)
        with tempfile.TemporaryDirectory() as temporary:
            fleet = RecoveryFleet(scenario)
            with closing(Recorder(Path(temporary) / "run")) as recorder:
                with patch.object(fleet, "assert_healthy", side_effect=RuntimeError("watchdog")):
                    with self.assertRaisesRegex(RuntimeError, "watchdog"):
                        recover_together(scenario, fleet, goals, recorder)
            self.assertFalse(fleet.commands)

    def test_target_arrival_does_not_replace_measured_connectivity(self):
        scenario = SimCase.load(SCENARIO, 3)
        # Artificial goals at the disconnected start: loop must still ask for graph verification.
        goals = {uav: scenario.layout[uav] for uav in scenario.survivors}
        fleet = RecoveryFleet(scenario)
        with tempfile.TemporaryDirectory() as temporary:
            with closing(Recorder(Path(temporary) / "run")) as recorder:
                with self.assertRaisesRegex(RuntimeError, "measured graph disconnected"):
                    recover_together(scenario, fleet, goals, recorder)
            self.assertEqual(fleet.connected_checks, 1)

    def test_target_drift_during_connectivity_wait_fails(self):
        scenario = SimCase.load(SCENARIO, 3)
        goals = json.loads((DIRECTORY / "fixtures/ten_three_targets_fixture.json")
                           .read_text())["targets_task_xy"]
        fleet = RecoveryFleet(scenario)

        def drift():
            fleet.poses["A01"] = (20.0, 0.0, scenario.normal_height)

        with tempfile.TemporaryDirectory() as temporary:
            with closing(Recorder(Path(temporary) / "run")) as recorder:
                with patch.object(fleet, "wait_connected", side_effect=drift):
                    with self.assertRaisesRegex(RuntimeError, "drifted"):
                        recover_together(scenario, fleet, goals, recorder)

    def test_three_and_four_fault_graphs_and_targets(self):
        for count, label in ((3, "three"), (4, "four")):
            with self.subTest(faults=count):
                scenario = SimCase.load(SCENARIO, count)
                targets = json.loads((DIRECTORY / ("fixtures/ten_%s_targets_fixture.json" % label))
                                     .read_text())["targets_task_xy"]
                scenario.validate_targets({uav: scenario.layout[uav] for uav in scenario.survivors},
                                          targets, {uav: scenario.layout[uav]
                                                    for uav in scenario.fault_ids})
                self.assertEqual(len(components(scenario.layout, 7)), 1)
                self.assertEqual(len(components({uav: scenario.layout[uav]
                                                 for uav in scenario.survivors}, 9)), 2)
                self.assertEqual(len(components(targets, 7)), 1)
                self.assertEqual(len(scenario.movement_batches(scenario.layout)), 2)
                self.assertEqual(scenario.solver_input(scenario.layout)["fault_ids"],
                                 list(scenario.fault_ids))

    def test_synchronized_sweep_and_rejection(self):
        self.assertEqual(synchronous_distance((-16, 0), (-9, 0), (-10, 0), (-3, 0)), 6)
        self.assertEqual(synchronous_distance((-6, 0), (6, 0), (6, 0), (-6, 0)), 0)
        scenario = SimCase.load(SCENARIO, 3)
        targets = json.loads((DIRECTORY / "fixtures/ten_three_targets_fixture.json")
                             .read_text())["targets_task_xy"]
        for invalid in ({**targets, "A07": (1, 0)},
                        {**targets, "A08": (-9, 0)},
                        {**targets, "A02": (-14, 0)},
                        {**targets, "A01": (float("nan"), 0)}):
            with self.subTest(targets=invalid), self.assertRaises(ValueError):
                scenario.validate_targets({uav: scenario.layout[uav]
                                           for uav in scenario.survivors}, invalid,
                                          {uav: scenario.layout[uav]
                                           for uav in scenario.fault_ids})

    def test_long_moves_still_require_clearance_and_connectivity(self):
        for count, label in ((3, "three"), (4, "four")):
            with self.subTest(faults=count):
                scenario = SimCase.load(SCENARIO, count)
                fixture = json.loads((DIRECTORY / ("fixtures/ten_%s_targets_fixture.json" % label))
                                     .read_text())["targets_task_xy"]
                targets = {uav: (point[0], point[1] + 5) for uav, point in fixture.items()}
                self.assertGreater(max(sum((targets[uav][index] - scenario.layout[uav][index]) ** 2
                                           for index in range(2)) ** 0.5
                                       for uav in scenario.survivors), 8.0)
                scenario.validate_targets({uav: scenario.layout[uav]
                                           for uav in scenario.survivors}, targets,
                                          {uav: scenario.layout[uav]
                                           for uav in scenario.fault_ids})

    def test_legacy_displacement_limit_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "legacy.json"
            config = json.loads(SCENARIO.read_text())
            config["max_displacement_m"] = 8
            path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError, "legacy max_displacement_m"):
                SimCase.load(path, 3)

    def test_sim_setup_localhost_and_unique_spawn(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "config"
            manifest = prepare(SCENARIO, IDENTITY, TOPOLOGY, output)
            self.assertTrue(manifest["simulation_only"])
            root = ElementTree.parse(output / "spawn.launch").getroot()
            instances = root.findall("./group/include")
            self.assertEqual(len(instances), 10)
            self.assertEqual([next(arg.attrib["value"] for arg in include.findall("arg")
                                   if arg.attrib["name"] == "ID") for include in instances],
                             [str(index) for index in range(10)])
            scenario = SimCase.load(SCENARIO, 3)
            for index, include in enumerate(instances):
                coords = {arg.attrib["name"]: arg.attrib["value"] for arg in include.findall("arg")}
                right, forward = enu_to_task(float(coords["x"]), float(coords["y"]), 0.0)
                self.assertAlmostEqual(right, scenario.layout[scenario.uav_ids[index]][0])
                self.assertAlmostEqual(forward, scenario.layout[scenario.uav_ids[index]][1])
            import yaml
            topology = yaml.safe_load((output / "topology.yaml").read_text())
            self.assertEqual(len(topology["uavs"]), 11)
            self.assertTrue(all(node["ip"] == "127.0.0.1" for node in topology["uavs"]))
            self.assertEqual(len([item for item in topology["topics"]
                                  if item["name"] == "exec_state"]), 1)

    def test_launcher_rejects_example_python_without_starting_px4(self):
        result = subprocess.run(["bash", str(DIRECTORY / "run_sim10.sh"),
                                 "--faults", "3", "--worker-python", "/path/to/python",
                                 "--check"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("executable not found", result.stderr)
        self.assertIn("No simulator started", result.stderr)

    def test_launcher_help_never_prints_fake_python_in_run_command(self):
        result = subprocess.run(["bash", str(DIRECTORY / "run_sim10.sh"), "--help"],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--check", result.stdout)
        self.assertIn("--check-algorithm", result.stdout)
        self.assertIn("--execute", result.stdout)

    def test_offline_cli_never_arms(self):
        for count, label in ((3, "three"), (4, "four")):
            with tempfile.TemporaryDirectory() as temporary:
                output = Path(temporary) / "trial"
                result = subprocess.run([sys.executable, str(DIRECTORY / "sim_multi.py"),
                                         "--faults", str(count), "--outdir", str(output),
                                         "--snapshot", str(DIRECTORY / "fixtures/ten_enu.json"),
                                         "--targets-fixture", str(DIRECTORY / (
                                             "fixtures/ten_%s_targets_fixture.json" % label))],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                summary = json.loads((output / "summary.json").read_text())
                self.assertEqual(summary["status"], "OFFLINE_ONLY")
                self.assertEqual(summary["survivor_clusters_after_fault"], 2)
                self.assertEqual(summary["planned_clusters"], 1)
                self.assertIn("MANUAL_TEST_FIXTURE_NOT_DAGL",
                              (output / "dag_output.json").read_text())
                metrics = json.loads((output / "candidate_metrics.json").read_text())
                self.assertEqual(metrics["min_allowed_separation_m"], 4.0)
                self.assertGreaterEqual(metrics["min_target_separation_m"], 4.0)
                self.assertIn("max_target_displacement_m", metrics)
                self.assertNotIn("max_allowed_displacement_m", metrics)

    def test_sitl_rejects_fixture(self):
        with tempfile.TemporaryDirectory() as temporary:
            result = subprocess.run([sys.executable, str(DIRECTORY / "sim_multi.py"),
                                     "--sitl", "--faults", "3", "--outdir", temporary,
                                     "--targets-fixture", str(DIRECTORY / "fixtures/ten_three_targets_fixture.json")],
                                    capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("offline fixtures are prohibited", result.stderr)


if __name__ == "__main__":
    unittest.main()
