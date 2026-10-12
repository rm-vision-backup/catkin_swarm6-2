"""Training and checkpoint gates; no ROS or simulator is started."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

DIRECTORY = Path(__file__).resolve().parent
SCENARIO = DIRECTORY / "scenarios/10.sitl.json"


class Dagl10TrainingTest(unittest.TestCase):
    def test_real_solver_requires_checkpoint(self):
        with tempfile.TemporaryDirectory() as folder:
            command = [sys.executable, str(DIRECTORY / "sim_multi.py"),
                       "--faults", "3", "--outdir", str(Path(folder) / "offline"),
                       "--snapshot", str(DIRECTORY / "fixtures/ten_enu.json")]
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("requires --checkpoint", result.stderr)
            self.assertFalse((Path(folder) / "offline").exists())

    def test_launcher_rejects_missing_checkpoint_before_px4(self):
        with tempfile.TemporaryDirectory() as folder:
            result = subprocess.run(["bash", str(DIRECTORY / "run_sim10.sh"),
                                     "--faults", "3", "--outdir", str(Path(folder) / "run"),
                                     "--check-algorithm"], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn("Provide --checkpoint", result.stderr)
            self.assertFalse((Path(folder) / "run").exists())

    def test_smoke_trains_without_exporting_approved_weights(self):
        executable = Path(os.environ.get("DAGL_WORKER_PYTHON",
                                         str(Path.home() / ".conda/envs/gcbfplus/bin/python3")))
        if not executable.is_file():
            self.skipTest("DAGL PyTorch environment not installed")
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "smoke"
            result = subprocess.run([str(executable), str(DIRECTORY / "train_dagl10.py"),
                                     "--outdir", str(output), "--smoke", "--epochs", "1",
                                     "--train-samples", "3", "--validation-samples", "2",
                                     "--test-samples", "2", "--hidden", "32", "--hops", "2"],
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(json.loads((output / "training_summary.json").read_text())["smoke"])
            self.assertFalse((output / "model.pt").exists())

    def test_dagl_inputs_keep_task_coordinates_for_displacement(self):
        executable = Path(os.environ.get("DAGL_WORKER_PYTHON",
                                         str(Path.home() / ".conda/envs/gcbfplus/bin/python3")))
        if not executable.is_file():
            self.skipTest("DAGL PyTorch environment not installed")
        code = ("import sys; sys.path.insert(0, %r); "
                "from train_dagl10 import SCENARIO,configure_dag,forward_inputs; "
                "from sim_multi import SimCase; "
                "case=SimCase.load(SCENARIO,3); configure_dag(case); "
                "starts,_,_=forward_inputs(case.layout,case.uav_ids,case.survivors,"
                "case.lower,case.upper,3); print(starts[0].tolist())" % str(DIRECTORY))
        result = subprocess.run([str(executable), "-c", code], capture_output=True,
                                text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), [-10.0, 0.0])


if __name__ == "__main__":
    unittest.main()
