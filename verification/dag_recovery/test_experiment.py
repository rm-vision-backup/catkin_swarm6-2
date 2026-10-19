"""Pure offline checks; never connects to ROS or arms an aircraft."""

import datetime
import json
import math
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

DIRECTORY = Path(__file__).resolve().parent
sys.path.insert(0, str(DIRECTORY))

from core import Scenario, components, enu_to_task, segment_distance, task_to_enu
from experiment import verify_approval
from pipeline import Recorder, digest, run_solver, solver_input
from prepare_deployment import generate
from replay import evaluate

IDENTITY = DIRECTORY.parents[1] / "src/tcp_to_ros/config/uav_identity_map.yaml"
TOPOLOGY = DIRECTORY.parents[1] / "src/swarm_topology_bridge/config/topology.yaml"
FOUR = DIRECTORY / "scenarios/4.template.json"
GOOD_TARGETS = {"A08": (-4, 0), "A12": (3, 0), "A13": (9, 0)}


class GeometryTest(unittest.TestCase):
    def test_layouts_and_graph_split(self):
        for count in (4, 11, 15):
            scenario = Scenario.load(DIRECTORY / ("scenarios/%s.template.json" % count))
            self.assertEqual(len(components(scenario.layout, scenario.radius - 1)), 1)
            remaining = {uav: pos for uav, pos in scenario.layout.items()
                         if uav != scenario.fault_id}
            self.assertEqual(len(components(remaining, scenario.radius + 1)), 2)

    def test_coordinate_round_trip(self):
        for angle in (0.0, math.pi / 2, 1.25):
            right, forward = enu_to_task(-7, 11, angle)
            east, north = task_to_enu(right, forward, angle)
            self.assertAlmostEqual(east, -7)
            self.assertAlmostEqual(north, 11)

    def test_segments_do_not_confuse_collinear_gap_and_overlap(self):
        self.assertEqual(segment_distance((0, 0), (2, 0), (4, 0), (6, 0)), 2)
        self.assertEqual(segment_distance((0, 0), (4, 4), (0, 4), (4, 0)), 0)

    def test_targets_gate(self):
        scenario = Scenario.load(FOUR)
        starts = {uav: point for uav, point in scenario.layout.items()
                  if uav != scenario.fault_id}
        scenario.validate_targets(starts, GOOD_TARGETS, scenario.layout[scenario.fault_id])
        scenario.validate_targets(starts, {"A08": (-3, 0), "A12": (1, 0), "A13": (7, 0)},
                                  scenario.layout[scenario.fault_id])
        long_move = {"A08": (-4, 10), "A12": (3, 10), "A13": (9, 10)}
        self.assertGreater(max(math.dist(starts[uav], target)
                               for uav, target in long_move.items()), 8.0)
        scenario.validate_targets(starts, long_move, scenario.layout[scenario.fault_id])
        for invalid in ({**GOOD_TARGETS, "A08": (-5, 0)},
                        {"A08": (-2.9, 0), "A12": (1, 0), "A13": (7, 0)},
                        {**GOOD_TARGETS, "A08": (100, 0)},
                        {**GOOD_TARGETS, "A08": (float("nan"), 0)},
                        {**GOOD_TARGETS, "A08": (3, 0)},
                        {"A08": (-4, 0)}):
            with self.subTest(targets=invalid), self.assertRaises(ValueError):
                scenario.validate_targets(starts, invalid, scenario.layout[scenario.fault_id])

    def test_unsafe_config_cannot_be_signed_off(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "unsafe.json"
            for field, value in (("min_separation_m", 3.9), ("edge_margin_m", 0.2),
                                 ("fault_height_m", 9), ("comm_radius_m", 100),
                                 ("max_displacement_m", 8)):
                config = json.loads(FOUR.read_text())
                config[field] = value
                path.write_text(json.dumps(config))
                with self.subTest(field=field), self.assertRaises(ValueError):
                    Scenario.load(path)


class PipelineTest(unittest.TestCase):
    def test_offline_fixture_cli(self):
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "trial"
            result = subprocess.run([sys.executable, str(DIRECTORY / "experiment.py"),
                                     "--scenario", str(FOUR), "--outdir", str(output),
                                     "--snapshot", str(DIRECTORY / "fixtures/four_enu.json"),
                                     "--targets-json", str(DIRECTORY / "fixtures/four_targets_fixture.json")],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads((output / "summary.json").read_text())["status"],
                             "OFFLINE_PLAN_ONLY")
            self.assertFalse((output / "flight_authorization.json").exists())

    def test_worker_output_and_timeout(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            fake = folder / "fake.py"
            fake.write_text("import json,sys,os\nos.chdir(sys.argv[3])\nprint('x'*100000)\n"
                            "json.dump({'targets_task_xy': {'A08': [-4,0], 'A12':[3,0],"
                            "'A13':[9,0]}, 'dag_estimated_time':3}, open(sys.argv[2],'w'))\n")
            recorder = Recorder(folder / "run")
            # Match documented relative --outdir and the real worker's cwd change.
            recorder.run_dir = Path(os.path.relpath(recorder.run_dir))
            try:
                result = run_solver(solver_input(Scenario.load(FOUR), Scenario.load(FOUR).layout),
                                    recorder, script=fake)
                self.assertEqual(result["targets_task_xy"], GOOD_TARGETS)
                self.assertGreater((recorder.run_dir / "worker.log").stat().st_size, 100000)
            finally:
                recorder.close()
            slow = folder / "slow.py"
            slow.write_text("import time\ntime.sleep(5)\n")
            recorder = Recorder(folder / "timeout")
            try:
                with self.assertRaises(TimeoutError):
                    run_solver({}, recorder, timeout_s=0.2, script=slow)
            finally:
                recorder.close()

    def test_deployment_and_approval_gate(self):
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            manifest = generate(FOUR, IDENTITY, TOPOLOGY, folder / "config")
            import yaml
            topology = yaml.safe_load(Path(manifest["topology_file"]).read_text())
            self.assertEqual(len(topology["uavs"]), 5)
            self.assertIn("exec_state", [topic["name"] for topic in topology["topics"]])
            self.assertEqual(len(topology["services"]), 12)
            with self.assertRaises(ValueError):
                generate(DIRECTORY / "scenarios/11.template.json", IDENTITY, TOPOLOGY,
                         folder / "invalid")
            approved = folder / "approved.json"
            content = json.loads(FOUR.read_text())
            content["approved"] = True
            approved.write_text(json.dumps(content))
            scenario = Scenario.load(approved)
            with self.assertRaises((ValueError, KeyError, FileNotFoundError)):
                verify_approval(scenario, approved, IDENTITY,
                                folder / "config/deployment.json", folder / "missing.json")
            signed = generate(approved, IDENTITY, TOPOLOGY, folder / "signed")
            approval = json.loads((DIRECTORY / "approval.template.json").read_text())
            approval["operator"] = "test-only"
            approval["flight_date"] = datetime.date.today().isoformat()
            for key, value in approval.items():
                if isinstance(value, bool) and key not in ("four_aircraft_passed", "eleven_aircraft_passed"):
                    approval[key] = True
            for key, path in (("scenario_sha256", approved), ("identity_sha256", IDENTITY),
                              ("topology_sha256", Path(signed["topology_file"])),
                              ("calibration_sha256", Path(signed["calibration_file"])),
                              ("formation_sha256", Path(signed["formation_file"]))):
                approval[key] = digest(path)
            approval_path = folder / "approval.json"
            approval_path.write_text(json.dumps(approval))
            self.assertEqual(verify_approval(scenario, approved, IDENTITY,
                                             folder / "signed/deployment.json",
                                             approval_path)["approval"]["operator"], "test-only")
            approval["task_load_checked"] = False
            approval_path.write_text(json.dumps(approval))
            with self.assertRaises(ValueError):
                verify_approval(scenario, approved, IDENTITY,
                                folder / "signed/deployment.json", approval_path)


class ReplayTest(unittest.TestCase):
    def test_stable_ten_seconds_and_missing_samples(self):
        scenario = Scenario.load(FOUR)
        events = [{"type": "fault_removed", "monotonic_time": 1}]
        events.append({"type": "command_completed", "monotonic_time": 2})
        positions = {uav: list(point) + [8 if uav == scenario.fault_id else 15]
                     for uav, point in scenario.layout.items()}
        positions["A08"][0] = -4
        for tick in range(65):
            events.append({"type": "telemetry", "monotonic_time": 2 + tick * 0.2,
                           "poses_task": positions, "min_separation_m": 6,
                           "pose_stamps_ros": {uav: 100 + tick * 0.2
                                               for uav in scenario.uav_ids},
                           "links": {uav: {"source_pose_age_s": 0.1}
                                     for uav in scenario.uav_ids}})
        self.assertTrue(evaluate(scenario, events)["verified"])
        self.assertFalse(evaluate(scenario, events[:35])["verified"])
        broken = events[:20] + events[32:]
        self.assertFalse(evaluate(scenario, broken)["verified"])
        with self.assertRaises(ValueError):
            evaluate(scenario, events + [{"type": "manual_intervention"}])
        stale = [dict(event) for event in events]
        stale[40] = {**stale[40], "pose_stamps_ros": stale[39]["pose_stamps_ros"]}
        self.assertFalse(evaluate(scenario, stale)["verified"])


if __name__ == "__main__":
    unittest.main()
