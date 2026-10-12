"""Offline-only checks for the ten-aircraft topology experiment."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

sys.path.insert(0, str(Path(__file__).resolve().parent))

from core import components, enu_to_task
from sim_multi import SimCase, synchronous_distance
from sim_setup import prepare

DIRECTORY = Path(__file__).resolve().parent
ROOT = DIRECTORY.parents[1]
SCENARIO = DIRECTORY / "scenarios/10.sitl.json"
IDENTITY = ROOT / "src/tcp_to_ros/config/uav_identity_map.yaml"
TOPOLOGY = ROOT / "src/swarm_topology_bridge/config/topology_group_a_sim.yaml"


class TenAircraftSimulationTest(unittest.TestCase):
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
