"""Twenty-node DAGL training remains independent of ten-aircraft PX4 SITL."""

import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

DIRECTORY = Path(__file__).resolve().parent
PYTHON = Path(os.environ.get("DAGL_WORKER_PYTHON",
                            str(Path.home() / ".conda/envs/gcbfplus/bin/python3")))


class Dagl20TrainingTest(unittest.TestCase):
    def test_teacher_and_batches_for_both_fault_sets(self):
        if not PYTHON.is_file():
            self.skipTest("DAGL PyTorch environment not installed")
        code = ("import json,sys; sys.path.insert(0,%r); "
                "from train_dagl20 import TwentyCase,SCENARIO; "
                "from train_dagl10 import make_teacher; "
                "from core import components,distance; "
                "cases=[TwentyCase.load(SCENARIO,count) for count in (3,4)]; "
                "goals=[make_teacher(case,case.layout)[0] for case in cases]; "
                "print(json.dumps([[len(case.uav_ids),len(case.survivors),"
                "len(components(goal,7)),max(map(len,case.movement_batches(case.layout))),"
                "min(distance(goal[a],goal[b]) for i,a in enumerate(case.survivors) "
                "for b in case.survivors[i+1:])] for case,goal in zip(cases,goals)]))"
                % str(DIRECTORY))
        result = subprocess.run([str(PYTHON), "-c", code], capture_output=True,
                                text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout), [[20, 17, 1, 4, 6.0],
                                                       [20, 16, 1, 4, 6.0]])

    def test_smoke_has_no_deployable_checkpoint(self):
        if not PYTHON.is_file():
            self.skipTest("DAGL PyTorch environment not installed")
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "smoke"
            result = subprocess.run([str(PYTHON), str(DIRECTORY / "train_dagl20.py"),
                                     "--outdir", str(output), "--smoke", "--epochs", "1",
                                     "--train-samples", "3", "--validation-samples", "2",
                                     "--test-samples", "2", "--hidden", "32", "--hops", "2"],
                                    capture_output=True, text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertTrue(json.loads((output / "training_summary.json").read_text())["smoke"])
            self.assertFalse((output / "model.pt").exists())

    def test_unapproved_checkpoint_cannot_pass_offline_verification(self):
        if not PYTHON.is_file():
            self.skipTest("DAGL PyTorch environment not installed")
        with tempfile.TemporaryDirectory() as folder:
            checkpoint = Path(folder) / "model.pt"
            checkpoint.write_bytes(b"not a checkpoint")
            checkpoint.with_name("model.json").write_text(json.dumps({"status": "INVALID"}))
            output = Path(folder) / "results"
            result = subprocess.run([str(PYTHON), str(DIRECTORY / "eval_dagl20.py"),
                                     "--faults", "3", "--checkpoint", str(checkpoint),
                                     "--outdir", str(output)], capture_output=True,
                                    text=True, timeout=30)
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertEqual(json.loads((output / "summary.json").read_text())["status"],
                             "FAILED")


if __name__ == "__main__":
    unittest.main()
