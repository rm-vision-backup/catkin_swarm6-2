#!/usr/bin/env python3
"""运行真实六机PX4 SITL并记录全部实测轨迹；只管理本轮启动的进程。"""
import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
import xmlrpc.client
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from verification.six_uav_joint import driver
from verification.six_uav_joint.flight_guard import RosFlightGuard


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    args = parser.parse_args()
    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_HOME"] = str(out / "ros_home")
    os.environ["ROS_LOG_DIR"] = str(out / "ros_home/log")
    processes = []
    files = []
    stop = threading.Event()
    recorder = None
    summary = {"simulation_type": "PX4_Gazebo_SITL", "status": "FAILED", "run_dir": str(out)}

    def launch(name, command, master):
        env = dict(os.environ, ROS_MASTER_URI="http://localhost:%d" % master,
                   GAZEBO_MASTER_URI="http://localhost:11345")
        log = (out / (name + ".log")).open("w")
        files.append(log)
        proc = subprocess.Popen(command, env=env, cwd=str(ROOT), stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, proc))
        print("START %s pid=%d" % (name, proc.pid), flush=True)
        return proc

    def wait_for(label, predicate, timeout=180):
        deadline = time.monotonic() + timeout
        reported = 0
        while time.monotonic() < deadline:
            try:
                if predicate():
                    print("READY " + label, flush=True)
                    return
            except Exception:
                pass
            for name, proc in processes:
                if proc.poll() is not None:
                    raise RuntimeError("%s exited=%s; see %s.log" % (name, proc.returncode, name))
            if time.monotonic() > reported:
                print("WAIT " + label, flush=True)
                reported = time.monotonic() + 20
            time.sleep(1)
        raise RuntimeError("timeout waiting " + label)

    try:
        # 拒绝复用已有Master/端口；检查发生在启动任何仿真进程之前。
        for port in (11300, 11310, 11345, 39061, *range(11316, 11322)):
            with socket.socket() as probe:
                if probe.connect_ex(("127.0.0.1", port)) == 0:
                    raise RuntimeError("端口%d已占用，未启动仿真" % port)
        package = driver.load_package()
        package.context.mission_id += "-sitl-" + uuid.uuid4().hex[:10]
        summary["mission_id"] = package.context.mission_id
        launch("gazebo", ["roslaunch", "verification/six_uav_joint/sitl/six_uav_spawn.launch",
                          "gui:=false"], 11300)
        def gazebo_ready():
            proxy = xmlrpc.client.ServerProxy("http://localhost:11300")
            code, _, nodes = proxy.getSystemState("/six_record")
            return code == 1 and any("/gazebo" in name for _, names in nodes[0] for name in names)
        wait_for("Gazebo", gazebo_ready)
        for n in range(6, 12):
            peers = [p for p in range(6, 12) if p != n]
            launch("uav%d" % n,
                   ["roslaunch", "swarm_uav_executor", "uav_offboard_ego.launch",
                    "uav_name:=UAV%d" % n, "uav_id:=UAV%d" % n, "tgt_system:=%d" % n,
                    "neighbor_intents:=" + ",".join("/UAV%d/trajectory_intent" % p for p in peers),
                    "neighbor_odom_topics:=" + ",".join("/UAV%d/swarm_pose" % p for p in peers),
                    "uav_task_timeout_s:=180", "interfaces_version:=six-joint-sitl",
                    "diagnostic_log_dir:=" + str(out / "planner"),
                    "runtime_log_dir:=" + str(out / "health")], 11310+n)
            time.sleep(2)
        launch("gcs", ["roslaunch", "verification/six_uav_joint/sitl/gcs_sim.launch",
                       "tcp_port:=39061"], 11310)
        wait_for("GCS master", lambda: xmlrpc.client.ServerProxy("http://localhost:11310").getPid("/six_record")[0] == 1)
        guard = RosFlightGuard(package)
        current = {"phase": "calibration"}

        def record():
            with (out / "telemetry.jsonl").open("w") as stream:
                while not stop.wait(0.2):
                    with guard.lock:
                        try:
                            if not guard.sample("/group_a/calibration_ready").data or not all(
                                guard.sample("/UAV%d/gp_origin_confirmed" % n).data for n in range(6, 12)):
                                continue
                            heading = guard.sample("/group_a/reference_heading").data
                            points = {u: guard.sample("/UAV%d/swarm_pose" % int(u[1:])).point
                                      for u in driver.IDS}
                            poses = {u: [p.x*math.sin(heading)-p.y*math.cos(heading),
                                         p.x*math.cos(heading)+p.y*math.sin(heading), p.z]
                                     for u, p in points.items()}
                            stream.write(json.dumps({"time": time.time(), "phase": current["phase"],
                                                     "poses": poses}) + "\n")
                            stream.flush()
                        except KeyError:
                            continue
        recorder = threading.Thread(target=record, daemon=True)
        recorder.start()

        def calibrated():
            with guard.lock:
                return guard.sample("/group_a/calibration_ready").data and all(
                    guard.sample("/UAV%d/gp_origin_confirmed" % n).data for n in range(6, 12))
        wait_for("six origins + calibration", calibrated)
        current["phase"] = "takeoff"
        takeoff = launch("takeoff", ["python3", "src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py",
                                     "6", "7", "8", "9", "10", "11"], 11310)
        deadline = time.monotonic() + 240
        while takeoff.poll() is None and time.monotonic() < deadline:
            time.sleep(1)
        if takeoff.poll() != 0:
            raise RuntimeError("SITL takeoff failed/timed out; see takeoff.log")
        print("TAKEOFF command complete, settling 15s", flush=True)
        time.sleep(15)
        guard.wait_ready()
        guard.check_initial_positions()
        transport = driver.TcpCommandTransport(package, driver.SystemClock(), "127.0.0.1", 39061)
        transport.wait_until_ready()
        class Log(driver.JsonlLogger):
            def write(self, row):
                if row.get("event") == "phase_start":
                    current["phase"] = row["phase"]
                super().write(row)
        def confirm(name, prompt):
            # 只在此仿真专用进程中自动确认；实机driver保留交互确认。
            guard.check()
            print("SITL checkpoint " + name, flush=True)
        try:
            with Log(out / "driver.jsonl") as log:
                result = driver.run_sequence(package, transport, guard, log, confirm=confirm)
            current["phase"] = "complete" if result == 0 else "failed"
            time.sleep(3)
            summary["status"] = "SUCCESS" if result == 0 else "FAILED"
        finally:
            transport.close()
    except Exception as exc:
        summary["error"] = str(exc)
        print("SITL ERROR: " + str(exc), flush=True)
    finally:
        stop.set()
        if recorder:
            recorder.join(2)
        (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
        # roslaunch进程组属于本轮；先SIGINT正常收尾，超时才终止同组。
        for name, proc in reversed(processes):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 15
        for name, proc in reversed(processes):
            try:
                proc.wait(timeout=max(0.1, deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for handle in files:
            handle.close()
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return 0 if summary["status"] == "SUCCESS" else 1


if __name__ == "__main__":
    sys.exit(main())
