#!/usr/bin/env python3
"""运行本轮 PX4/Gazebo SITL，记录全部实测轨迹并生成 summary/driver 证据。

只管理本轮启动的进程；端口/目录已被占用时拒绝启动。--dry-run 只打印计划，
不启动 Gazebo/PX4。视频由 render_video.py 从 telemetry.jsonl 重建。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
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
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verification.full_fleet_joint import driver
from verification.full_fleet_joint.flight_guard import RosFlightGuard
from verification.full_fleet_joint.sitl import generate_spawn


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument(
        "--spec",
        type=Path,
        default=ROOT / "verification/full_fleet_joint/generated/mission_spec.json",
    )
    parser.add_argument("--tcp-port", type=int, default=39061)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="只打印计划和 launch 命令")
    args = parser.parse_args()

    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_HOME"] = str(out / "ros_home")
    os.environ["ROS_LOG_DIR"] = str(out / "ros_home/log")

    spec_path = args.spec.resolve()
    spec = driver.load_spec(spec_path)
    package = driver.load_package(spec_path)
    package.context.mission_id += "-sitl-" + uuid.uuid4().hex[:10]
    available = list(spec["available_uavs"])
    numbers = [int(uav[3:]) for uav in available]
    b_ids = set(spec["b_ids"])
    b_numbers = {int(task_id[1:]) for task_id in b_ids}
    leader_number = int(spec["leader_id"][1:])
    fleet_file = Path(spec["fleet_file"])
    if not fleet_file.is_file():
        raise RuntimeError("fleet_file 不存在：%s" % fleet_file)
    # 编队/标定配置必须来自本轮 --spec 所在任务包；否则 15/14 备份包会
    # 加载到另一规模的 generated/config，coverage2 会报成员不匹配。
    config_dir = spec_path.parent / "config"
    formation_offsets_file = config_dir / "formation_offsets.yaml"
    calibration_config_file = config_dir / "calibration.yaml"
    for config_file in (formation_offsets_file, calibration_config_file):
        if not config_file.is_file():
            raise RuntimeError("任务包缺少配置：%s" % config_file)

    # 证据目录：spec/manifest/fleet 与 spawn 一起保存，render_video 只读本轮实测。
    shutil.copy2(spec_path, out / "mission_spec.json")
    manifest_path = spec_path.parent / "manifest.json"
    if manifest_path.exists():
        shutil.copy2(manifest_path, out / "manifest.json")
    shutil.copy2(fleet_file, out / "fleet.yaml")
    spawn_path = out / "spawn.launch"
    generate_spawn.generate(spec_path, spawn_path, fleet_file)

    processes = []
    files = []
    stop = threading.Event()
    recorder = None
    summary = {
        "simulation_type": "PX4_Gazebo_SITL",
        "status": "FAILED",
        "run_dir": str(out),
        "mission_id": package.context.mission_id,
        "available": available,
        "leader": spec["leader_id"],
        "faulted": list(spec["faulted_ids"]),
        "b_ids": list(spec["b_ids"]),
    }

    def launch(name, command, master):
        env = dict(
            os.environ,
            ROS_MASTER_URI="http://localhost:%d" % master,
            GAZEBO_MASTER_URI="http://localhost:11345",
        )
        log = (out / (name + ".log")).open("w")
        files.append(log)
        proc = subprocess.Popen(
            command,
            env=env,
            cwd=str(ROOT),
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
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
                    raise RuntimeError(
                        "%s exited=%s; see %s.log" % (name, proc.returncode, name)
                    )
            if time.monotonic() > reported:
                print("WAIT " + label, flush=True)
                reported = time.monotonic() + 20
            time.sleep(1)
        raise RuntimeError("timeout waiting " + label)

    def port_free(port):
        with socket.socket() as probe:
            return probe.connect_ex(("127.0.0.1", port)) != 0

    def onboard_command(number):
        peers = [n for n in numbers if n != number]
        command = [
            "roslaunch",
            "swarm_uav_executor",
            "uav_offboard_ego.launch",
            "uav_name:=UAV%d" % number,
            "uav_id:=UAV%d" % number,
            "tgt_system:=%d" % number,
            "neighbor_intents:="
            + ",".join("/UAV%d/trajectory_intent" % peer for peer in peers),
            "neighbor_odom_topics:="
            + ",".join("/UAV%d/swarm_pose" % peer for peer in peers),
            "uav_task_timeout_s:=300",
            "interfaces_version:=full-fleet-sitl",
            "diagnostic_log_dir:=" + str(out / "planner"),
            "runtime_log_dir:=" + str(out / "health"),
        ]
        if number in b_numbers:
            command.append("layer_move_to:=8.0")
        return command

    if args.dry_run:
        print(json.dumps({
            "outdir": str(out),
            "spec": str(spec_path),
            "spawn": str(spawn_path),
            "available": available,
            "b_numbers": sorted(b_numbers),
            "leader": leader_number,
            "tcp_port": args.tcp_port,
            "ports": sorted({11300, 11310, 11345, args.tcp_port,
                             *[11310 + n for n in numbers]}),
            "onboard": {"UAV%d" % n: onboard_command(n) for n in numbers},
        }, ensure_ascii=False, indent=2))
        return 0

    try:
        ports = [11300, 11310, 11345, args.tcp_port] + [11310 + n for n in numbers]
        busy = [port for port in ports if not port_free(port)]
        if busy:
            raise RuntimeError("端口已被占用，未启动仿真：%s" % busy)

        launch(
            "gazebo",
            ["roslaunch", str(spawn_path), "gui:=%s" % ("true" if args.gui else "false")],
            11300,
        )

        def gazebo_ready():
            proxy = xmlrpc.client.ServerProxy("http://localhost:11300")
            code, _, nodes = proxy.getSystemState("/full_fleet_record")
            return code == 1 and any(
                "/gazebo" in name for _, names in nodes[0] for name in names
            )

        wait_for("Gazebo", gazebo_ready)
        for number in numbers:
            launch("uav%d" % number, onboard_command(number), 11310 + number)
            time.sleep(1.5)
        launch(
            "gcs",
            ["roslaunch", "verification/full_fleet_joint/sitl/gcs_sim.launch",
             "tcp_port:=%d" % args.tcp_port,
             "formation_offsets_file:=%s" % formation_offsets_file,
             "calibration_config_file:=%s" % calibration_config_file],
            11310,
        )
        wait_for(
            "GCS master",
            lambda: xmlrpc.client.ServerProxy("http://localhost:11310")
            .getPid("/full_fleet_record")[0] == 1,
        )
        guard = RosFlightGuard(package, spec)
        current = {"phase": "calibration"}

        def origins_ready():
            with guard.lock:
                return (
                    guard.sample("/group_a/calibration_ready").data
                    and all(
                        guard.sample(
                            "/UAV%d/gp_origin_confirmed" % number
                        ).data
                        for number in numbers
                    )
                )

        def record():
            with (out / "telemetry.jsonl").open("w") as stream:
                while not stop.wait(0.2):
                    with guard.lock:
                        try:
                            if not origins_ready():
                                continue
                            heading = guard.sample("/group_a/reference_heading").data
                            points = {
                                task_id: guard.sample(
                                    "/UAV%d/swarm_pose" % int(task_id[1:])
                                ).point
                                for task_id in spec["all_ids"]
                            }
                            poses = {
                                task_id: [
                                    p.x * math.sin(heading) - p.y * math.cos(heading),
                                    p.x * math.cos(heading) + p.y * math.sin(heading),
                                    p.z,
                                ]
                                for task_id, p in points.items()
                            }
                            stream.write(json.dumps({
                                "time": time.time(),
                                "phase": current["phase"],
                                "poses": poses,
                            }) + "\n")
                            stream.flush()
                        except KeyError:
                            continue

        recorder = threading.Thread(target=record, daemon=True)
        recorder.start()
        wait_for("all origins + calibration", origins_ready, timeout=240)

        current["phase"] = "takeoff"
        takeoff = launch(
            "takeoff",
            ["python3", "src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py",
             *[str(n) for n in numbers]],
            11310,
        )
        deadline = time.monotonic() + 300
        while takeoff.poll() is None and time.monotonic() < deadline:
            time.sleep(1)
        if takeoff.poll() != 0:
            raise RuntimeError("SITL takeoff failed/timed out; see takeoff.log")
        print("TAKEOFF command complete, settling 15s", flush=True)
        time.sleep(15)
        guard.wait_ready()

        transport = driver.TcpCommandTransport(
            package, driver.SystemClock(), "127.0.0.1", args.tcp_port,
            # 与实机 driver 一致：覆盖 GCS 排程错峰造成的 START 延迟。
            accept_timeout_s=20,
        )
        transport.wait_until_ready()

        class Log(driver.JsonlLogger):
            def write(self, row):
                if row.get("event") == "phase_start":
                    current["phase"] = row["phase"]
                super().write(row)

        def confirm(name, prompt):
            guard.check()
            print("SITL checkpoint " + name, flush=True)

        try:
            phases = driver.build_phases(package.spec)
            with Log(out / "driver.jsonl") as log:
                result = driver.run_sequence(
                    package,
                    transport,
                    guard,
                    log,
                    phases,
                    len(phases),
                    confirm,
                )
            current["phase"] = "complete" if result == 0 else "failed"
            time.sleep(3)
            summary["status"] = "SUCCESS" if result == 0 else "FAILED"
        finally:
            transport.close()
    except Exception as exc:  # noqa: BLE001
        summary["error"] = str(exc)
        print("SITL ERROR: " + str(exc), flush=True)
    finally:
        stop.set()
        if recorder:
            recorder.join(2)
        (out / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        for _name, proc in reversed(processes):
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 15
        for _name, proc in reversed(processes):
            try:
                proc.wait(timeout=max(0.1, deadline - time.monotonic()))
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
