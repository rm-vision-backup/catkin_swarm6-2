"""two_teams_recon 离线测试：集群布局、区域、编组、FAULT_EXIT 与 dry-run。"""
from __future__ import annotations

import io
import json
import math
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verification.two_teams_recon import driver, generate_mission  # noqa: E402
from uav_bt_runtime.config import load_json_model, load_mission_context  # noqa: E402
from uav_bt_runtime.package import WorldConfig  # noqa: E402

TEAMS = Path(__file__).resolve().parent / "teams.yaml"


class TwoTeamsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.out = Path(cls.tmp.name)
        generate_mission.generate(TEAMS, cls.out)
        cls.specs = {}
        cls.packages = {}
        for team in ("1", "2"):
            spec_path = cls.out / ("team%s" % team) / "mission_spec.json"
            cls.specs[team] = driver.load_spec(spec_path)
            cls.packages[team] = driver.load_package(spec_path)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_package_loads(self):
        for team in ("1", "2"):
            base = self.out / ("team%s" % team) / "package"
            context = load_mission_context(
                base / "robots.json", base / "bootstrap.json", base / "plans" / "plans.json"
            )
            load_json_model(base / "world.json", WorldConfig)
            leaders = [uav.uav_id for uav in context.uav_specs.values() if uav.is_leader]
            self.assertEqual(leaders, [self.specs[team]["leader_id"]])

    def test_groups(self):
        one = self.specs["1"]
        self.assertEqual(one["initial_ids"], ["A01", "A02", "A03", "A04", "A05", "A06"])
        self.assertEqual(one["faulted_ids"], ["A05"])
        self.assertEqual(one["spare_ids"], ["A07"])
        two = self.specs["2"]
        self.assertEqual(two["initial_ids"], ["A10", "A11", "A12", "A13", "A14", "A15"])
        self.assertEqual(two["faulted_ids"], ["A10", "A15"])
        self.assertEqual(two["spare_ids"], ["A08", "A09"])
        self.assertNotIn(two["leader_id"], two["faulted_ids"])

    def test_ground_layout_uses_calibration_origin(self):
        # 原点 = UAV2（第一排中间）：任务坐标下 UAV2 = (0,0)
        expected = {
            "A01": (-5, 0), "A02": (0, 0), "A03": (5, 0),
            "A07": (-5, -10), "A08": (0, -10), "A09": (5, -10),
            "A10": (-5, -15), "A11": (0, -15), "A15": (5, -20),
        }
        ground = {**self.specs["1"]["ground_positions"], **self.specs["2"]["ground_positions"]}
        for task_id, (x, y) in expected.items():
            self.assertAlmostEqual(ground[task_id]["x"], x, places=6)
            self.assertAlmostEqual(ground[task_id]["y"], y, places=6)
        self.assertEqual(self.specs["1"]["calibration"]["calibration_uavs"],
                         ["UAV1", "UAV2", "UAV3"])
        self.assertEqual(len(self.specs["1"]["calibration"]["all_uavs"]), 15)

    def test_areas_and_routes(self):
        self.assertEqual(self.specs["1"]["coverage"]["area"],
                         {"min_x": -40.0, "max_x": 0.0, "min_y": 0.0, "max_y": 50.0})
        self.assertEqual(self.specs["2"]["coverage"]["area"],
                         {"min_x": 0.0, "max_x": 40.0, "min_y": 0.0, "max_y": 50.0})
        self.assertEqual(self.specs["1"]["coverage"]["strips"], 4)
        self.assertEqual(self.specs["1"]["coverage"]["lateral_step_m"], 10.0)
        self.assertEqual(self.specs["1"]["coverage"]["coverage1_waypoints"],
                         [[-35.0, 0.0], [-35.0, 50.0]])
        self.assertEqual(self.specs["2"]["coverage"]["coverage1_waypoints"],
                         [[5.0, 0.0], [5.0, 50.0]])
        self.assertEqual(self.specs["2"]["coverage"]["coverage2_waypoints"][-1],
                         [35.0, 0.0])

    def test_formation_offsets_and_pairings(self):
        for team in ("1", "2"):
            spec = self.specs[team]
            initial = spec["formation_offsets"]["recon_initial"]
            recovered = spec["formation_offsets"]["recon_recovered"]
            self.assertEqual(set(recovered), set(spec["recovered_ids"]))
            for fault, spare in spec["replacements"].items():
                self.assertEqual(recovered[spare], initial[fault])
            coords = [initial[uav][:2] for uav in initial]
            for left in range(len(coords)):
                for right in range(left + 1, len(coords)):
                    self.assertGreaterEqual(math.dist(coords[left], coords[right]), 4.999)

    def test_fault_exit_wire_format(self):
        expected = {
            "1": {"A05": (-35.0, 45.0)},
            "2": {"A10": (0.0, 50.0), "A15": (10.0, 45.0)},
        }
        for team in ("1", "2"):
            plan = self.packages[team].context.plans["fault-exit"]
            self.assertEqual(set(plan.robot_assignments), set(expected[team]))
            for task_id, assignment in plan.robot_assignments.items():
                self.assertEqual(assignment.command_type, "FAULT_EXIT")
                self.assertNotIn("target_pose", assignment.payload)
                waypoint = assignment.payload["waypoints"][0]
                self.assertEqual(waypoint["z"], 8.0)
                self.assertAlmostEqual(waypoint["x"], expected[team][task_id][0], places=6)
                self.assertAlmostEqual(waypoint["y"], expected[team][task_id][1], places=6)

    def test_recovery_displacement(self):
        for team in ("1", "2"):
            spec = self.specs[team]
            plan = self.packages[team].context.plans["recovery-spares"]
            for robot_id, assignment in plan.robot_assignments.items():
                target = assignment.payload["target_pose"]
                ground = spec["ground_positions"][robot_id]
                self.assertGreater(
                    math.hypot(target["x"] - ground["x"], target["y"] - ground["y"]), 0.5
                )

    def test_expected_targets_cover_all(self):
        for team in ("1", "2"):
            spec = self.specs[team]
            for phase in spec["phases"]:
                for step in phase["steps"]:
                    if not step["observe_only"]:
                        self.assertIn(step["plan_id"], spec["expected_targets"])
            self.assertEqual(set(spec["expected_targets"]["hold-end"]), set(spec["all_ids"]))
            self.assertNotIn("prepare-spares", spec["expected_targets"])

    def test_combined_configs(self):
        combined = self.out / "combined"
        calibration = json.loads(
            json.dumps(__import__("yaml").safe_load(
                (combined / "calibration.yaml").read_text(encoding="utf-8")))
        )
        self.assertEqual(calibration["calibration_uavs"], ["UAV1", "UAV2", "UAV3"])
        self.assertEqual(len(calibration["all_uavs"]), 15)
        import yaml
        leaders = {}
        for team in ("1", "2"):
            data = yaml.safe_load(
                (combined / ("formation_team%s.yaml" % team)).read_text(encoding="utf-8")
            )
            leaders[team] = data["leader_id"]
        self.assertEqual(leaders, {"1": "A02", "2": "A11"})

    def test_dry_run_batches(self):
        for team in ("1", "2"):
            spec_path = self.out / ("team%s" % team) / "mission_spec.json"
            buffer = io.StringIO()
            with redirect_stdout(buffer):
                self.assertEqual(driver.main(["--dry-run", "--spec", str(spec_path)]), 0)
            text = buffer.getvalue()
            self.assertTrue(text.strip())
            self.assertIn("FAULT_EXIT", text)
            self.assertIn("FOLLOW_ROUTE", text)

    def test_gcs_launch_has_two_backends(self):
        text = (Path(__file__).resolve().parent / "sitl" / "gcs_two_teams.launch").read_text(
            encoding="utf-8"
        )
        self.assertIn('ns="team1"', text)
        self.assertIn('ns="team2"', text)
        self.assertEqual(text.count('type="gcs_a_backend_node.py"'), 2)
        self.assertEqual(text.count('fault_exit_schedule_enabled" value="false"'), 2)


if __name__ == "__main__":
    unittest.main()
