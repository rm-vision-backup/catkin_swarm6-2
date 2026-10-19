"""Solver isolation, run provenance and lossless JSONL recording."""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Mapping

from core import Scenario, _pair

REPO_ROOT = Path(__file__).resolve().parents[2]
DAG_ROOT = REPO_ROOT / "references" / "DAG-GCBF"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Recorder:
    def __init__(self, run_dir: Path):
        run_dir.mkdir(parents=True, exist_ok=False)
        self.run_dir = run_dir
        self._stream = (run_dir / "events.jsonl").open("x", encoding="utf-8")
        self._lock = threading.Lock()

    def event(self, kind: str, **values):
        record = {"type": kind, "wall_time": time.time(),
                  "monotonic_time": time.monotonic(), **values}
        with self._lock:
            self._stream.write(json.dumps(record, allow_nan=False, sort_keys=True) + "\n")
            self._stream.flush()

    def save(self, name: str, value: object):
        (self.run_dir / name).write_text(
            json.dumps(value, allow_nan=False, sort_keys=True, indent=2), encoding="utf-8")

    def close(self):
        with self._lock:
            self._stream.close()


def solver_input(scenario: Scenario, positions: Mapping[str, tuple[float, float]]):
    if set(positions) != set(scenario.uav_ids):
        raise ValueError("DAGL input must include every initial UAV")
    return {"uav_ids": list(scenario.uav_ids), "fault_id": scenario.fault_id,
            "positions_task_xy": {uav: list(_pair(positions[uav])) for uav in scenario.uav_ids},
            "bounds_task_xy": [list(scenario.lower), list(scenario.upper)],
            "comm_radius_m": scenario.radius, "seed": scenario.seed}


def run_solver(payload: dict, recorder: Recorder, python: str = sys.executable,
               timeout_s: float = 60.0, healthy: Callable[[], None] = lambda: None,
               script: Path | None = None) -> dict:
    if timeout_s <= 0:
        raise ValueError("solver timeout must be positive")
    worker = script or Path(__file__).with_name("dag_worker.py")
    # DAG worker changes cwd to the reference repository before writing its result.
    input_path = (recorder.run_dir / "dag_input.json").resolve()
    output_path = (recorder.run_dir / "dag_output.json").resolve()
    recorder.save("dag_input.json", payload)
    begin = time.monotonic()
    with (recorder.run_dir / "worker.log").open("wb") as log:
        process = subprocess.Popen(
            [python, str(worker.resolve()), str(input_path), str(output_path), str(DAG_ROOT.resolve())],
            cwd=str(REPO_ROOT), stdout=log, stderr=subprocess.STDOUT,
        )
        try:
            while process.poll() is None:
                healthy()
                if time.monotonic() - begin > timeout_s:
                    raise TimeoutError("ML-DAGL worker exceeded compute deadline")
                time.sleep(0.1)
            if process.returncode or not output_path.is_file():
                raise RuntimeError("ML-DAGL worker failed; see worker.log")
            result = json.loads(output_path.read_text(encoding="utf-8"))
            targets = {uav: _pair(point) for uav, point in result["targets_task_xy"].items()}
            faults = (payload["fault_ids"] if payload.get("simulation_only") is True
                      else [payload["fault_id"]])
            if set(targets) != set(payload["uav_ids"]) - set(faults):
                raise ValueError("ML-DAGL returned missing or unexpected identities")
            result["targets_task_xy"] = targets
            if not math.isfinite(float(result["dag_estimated_time"])):
                raise ValueError("ML-DAGL estimated time is non-finite")
            recorder.event("solver_done", elapsed_s=time.monotonic() - begin)
            return result
        except BaseException:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            raise


def provenance(scenario_path: Path, identity_path: Path | None = None) -> dict:
    files = [scenario_path]
    if identity_path:
        files.append(identity_path)
    files.extend(Path(__file__).with_name(name) for name in (
        "core.py", "pipeline.py", "experiment.py", "dag_worker.py",
        "ros_adapter.py", "prepare_deployment.py", "gcs.launch"))
    files.extend(DAG_ROOT / name for name in (
        "Configurations.py", "Utils.py", "DAGL_Algorithm/DAGL_Framework.py",
        "DAGL_Algorithm/Damage_Attention.py", "DAGL_Algorithm/Bipartite_GCN.py"))
    result = {str(path.resolve()): digest(path) for path in files}
    revision = subprocess.run(["git", "-C", str(DAG_ROOT), "rev-parse", "HEAD"],
                              capture_output=True, text=True, check=False)
    return {"file_sha256": result, "dag_revision": revision.stdout.strip() or "unknown"}
