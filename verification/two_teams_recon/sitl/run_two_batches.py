#!/usr/bin/env python3
"""15 机同一环境、一起标定，分两批运行 two_teams_recon 任务。

环境固定使用 src/swarm_simulation/launch/multi_uav_ego_15sim.launch。
批次1（UAV1–7）飞完自动 AUTO.LAND；批次2（UAV8–15）飞完后默认保留环境，
供 QGC 截图。结束方式见输出目录的 stop_environment.sh。
"""
from __future__ import annotations

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
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from verification.two_teams_recon import driver  # noqa: E402

ENV_LAUNCH = "src/swarm_simulation/launch/multi_uav_ego_15sim.launch"
GCS_LAUNCH = "verification/two_teams_recon/sitl/gcs_two_teams.launch"
ALL_NUMBERS = list(range(1, 16))
TEAM_NUMBERS = {"1": list(range(1, 8)), "2": list(range(8, 16))}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--headless", action="store_true", help="不启动 Gazebo GUI")
    parser.add_argument("--no-keep-alive", action="store_true",
                        help="两批结束后关闭环境（默认保留）")
    parser.add_argument("--leave-running", action="store_true",
                        help="完成后本进程退出但保留环境（自动化验证用）")
    parser.add_argument("--attach-environment", action="store_true",
                        help="复用已在运行的 multi_uav_ego_15sim 环境，不再启动 Gazebo/PX4")
    parser.add_argument("--tcp-team1", type=int, default=11561)
    parser.add_argument("--tcp-team2", type=int, default=11562)
    args = parser.parse_args()

    out = args.outdir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_HOME"] = str(out / "ros_home")
    os.environ["ROS_LOG_DIR"] = str(out / "ros_home/log")

    specs = {}
    for team in ("1", "2"):
        spec_path = driver.spec_path_for_team(team)
        if not spec_path.is_file():
            raise SystemExit("缺少 %s；先运行 flow.sh generate" % spec_path)
        specs[team] = driver.load_spec(spec_path)
    combined_dir = ROOT / "verification/two_teams_recon/generated/combined"
    formation_files = {
        "1": combined_dir / "formation_team1.yaml",
        "2": combined_dir / "formation_team2.yaml",
    }
    calibration_file = combined_dir / "calibration.yaml"
    for path in (*formation_files.values(), calibration_file):
        if not path.is_file():
            raise SystemExit("缺少 %s；先运行 flow.sh generate" % path)

    processes = []
    files = []
    stop = threading.Event()
    recorder = None
    current_phase = {"name": "startup"}
    results = {"team1": None, "team2": None}
    summary = {
        "simulation_type": "PX4_Gazebo_SITL",
        "status": "FAILED",
        "run_dir": str(out),
        "environment_launch": ENV_LAUNCH,
        "batches": {"team1": TEAM_NUMBERS["1"], "team2": TEAM_NUMBERS["2"]},
        "leaders": {team: specs[team]["leader_id"] for team in ("1", "2")},
        "faulted": {team: specs[team]["faulted_ids"] for team in ("1", "2")},
        "spares": {team: specs[team]["spare_ids"] for team in ("1", "2")},
        "areas": {team: specs[team]["coverage"]["area"] for team in ("1", "2")},
        "kept_alive": False,
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

    def wait_for(label, predicate, timeout=240):
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
                code = proc.poll()
                # 0 = 正常结束（takeoff/land/driver 等），只把非零退出视为异常。
                if code is not None and code != 0:
                    raise RuntimeError(
                        "%s exited=%s; see %s.log" % (name, code, name)
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
        team = "1" if number in TEAM_NUMBERS["1"] else "2"
        peers = [n for n in TEAM_NUMBERS[team] if n != number]
        return [
            "roslaunch", "swarm_uav_executor", "uav_offboard_ego.launch",
            "uav_name:=UAV%d" % number,
            "uav_id:=UAV%d" % number,
            "tgt_system:=%d" % number,
            "neighbor_intents:=" + ",".join("/UAV%d/trajectory_intent" % p for p in peers),
            "neighbor_odom_topics:=" + ",".join("/UAV%d/swarm_pose" % p for p in peers),
            "uav_task_timeout_s:=480",
            "interfaces_version:=two-teams-recon-sitl",
            "diagnostic_log_dir:=" + str(out / "planner"),
            "runtime_log_dir:=" + str(out / "health"),
        ]

    def run_driver(team, tcp_port):
        current_phase["name"] = "team" + team
        spec_path = driver.spec_path_for_team(team)
        log_path = out / ("driver_team%s.jsonl" % team)
        cmd = [
            sys.executable, str(ROOT / "verification/two_teams_recon/driver.py"),
            "--execute", "--spec", str(spec_path),
            "--tcp-host", "127.0.0.1", "--tcp-port", str(tcp_port),
            "--log", str(log_path),
        ]
        proc = launch("driver_team%s" % team, cmd, 11310)
        proc.wait()
        results["team%s" % team] = "SUCCESS" if proc.returncode == 0 else "FAILED"
        return proc.returncode == 0

    try:
        ports = [11310, args.tcp_team1, args.tcp_team2] + [
            11310 + n for n in ALL_NUMBERS
        ]
        if not args.attach_environment:
            ports = [11300, 11345] + ports
        busy = [port for port in ports if not port_free(port)]
        if busy:
            raise RuntimeError("端口已被占用，未启动仿真：%s" % busy)

        def gazebo_ready():
            proxy = xmlrpc.client.ServerProxy("http://localhost:11300")
            code, _, nodes = proxy.getSystemState("/two_teams_two_batches")
            return code == 1 and any(
                "/gazebo" in name for _, names in nodes[0] for name in names
            )

        if args.attach_environment:
            wait_for("existing Gazebo/PX4 environment", gazebo_ready, timeout=30)
            print("ATTACH existing %s" % ENV_LAUNCH, flush=True)
        else:
            launch(
                "environment",
                ["roslaunch", ENV_LAUNCH,
                 "gui:=%s" % ("false" if args.headless else "true")],
                11300,
            )
            wait_for("Gazebo/PX4 environment", gazebo_ready)

            def env_counts():
                path = out / "environment.log"
                if not path.exists():
                    return (0, 0)
                text = path.read_text(errors="ignore")
                return (text.count("Successfully spawned entity"),
                        text.count("Simulator connected on TCP port"))

            wait_for("15 models spawned",
                     lambda: env_counts()[0] >= len(ALL_NUMBERS), timeout=240)
            wait_for("15 simulators connected",
                     lambda: env_counts()[1] >= len(ALL_NUMBERS), timeout=240)
        for number in ALL_NUMBERS:
            launch("uav%d" % number, onboard_command(number), 11310 + number)
            time.sleep(1.5)
        # 给 MAVROS/EKF 一点预热时间，避免标定节点在位置流尚未建立时启动。
        print("SETTLE 15s before GCS/calibration", flush=True)
        time.sleep(15)
        launch(
            "gcs",
            ["roslaunch", GCS_LAUNCH,
             "tcp_port_team1:=%d" % args.tcp_team1,
             "tcp_port_team2:=%d" % args.tcp_team2,
             "formation_team1:=%s" % formation_files["1"],
             "formation_team2:=%s" % formation_files["2"],
             "calibration_config_file:=%s" % calibration_file],
            11310,
        )
        wait_for(
            "GCS master",
            lambda: xmlrpc.client.ServerProxy("http://localhost:11310")
            .getPid("/two_teams_two_batches")[0] == 1,
        )

        import rospy
        from geometry_msgs.msg import PointStamped
        from std_msgs.msg import Bool, Float64

        rospy.init_node("two_teams_two_batches", anonymous=True, disable_signals=True)
        lock = threading.RLock()
        samples = {}

        def receive(message, topic):
            with lock:
                samples[topic] = (time.monotonic(), message)

        for topic in ("/group_a/calibration_ready", "/group_a/reference_heading"):
            rospy.Subscriber(topic, Bool if topic.endswith("ready") else Float64,
                             receive, callback_args=topic, queue_size=1)
        for number in ALL_NUMBERS:
            rospy.Subscriber("/UAV%d/gp_origin_confirmed" % number, Bool,
                             receive, callback_args="/UAV%d/gp_origin_confirmed" % number,
                             queue_size=1)
            rospy.Subscriber("/UAV%d/swarm_pose" % number, PointStamped,
                             receive, callback_args="/UAV%d/swarm_pose" % number,
                             queue_size=10)

        def origins_ready():
            with lock:
                if not samples.get("/group_a/calibration_ready", (0, None))[1].data:
                    return False
                return all(
                    samples.get("/UAV%d/gp_origin_confirmed" % n, (0, None))[1].data
                    for n in ALL_NUMBERS
                )

        def poses_fresh(numbers, max_age=1.5):
            with lock:
                now = time.monotonic()
                for number in numbers:
                    sample = samples.get("/UAV%d/swarm_pose" % number)
                    if sample is None or now - sample[0] > max_age:
                        return False
                    if not math.isfinite(sample[1].point.z):
                        return False
                return True

        def airborne(numbers, min_z=2.0):
            with lock:
                now = time.monotonic()
                for number in numbers:
                    sample = samples.get("/UAV%d/swarm_pose" % number)
                    if sample is None or now - sample[0] > 1.5:
                        return False
                    if sample[1].point.z < min_z:
                        return False
                return True

        def wait_poses(numbers, timeout=180, label="poses"):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if poses_fresh(numbers):
                    print("READY " + label, flush=True)
                    return
                time.sleep(1)
            with lock:
                now = time.monotonic()
                missing = []
                for number in numbers:
                    sample = samples.get("/UAV%d/swarm_pose" % number)
                    if sample is None:
                        missing.append("UAV%d:none" % number)
                    elif now - sample[0] > 1.5:
                        missing.append("UAV%d:stale" % number)
                    else:
                        missing.append("UAV%d:z=%.2f" % (number, sample[1].point.z))
            raise RuntimeError("%s 未就绪: %s" % (label, ",".join(missing)))

        def record():
            with (out / "telemetry.jsonl").open("w") as stream:
                while not stop.wait(0.2):
                    with lock:
                        try:
                            heading = samples["/group_a/reference_heading"][1].data
                            points = {
                                n: samples["/UAV%d/swarm_pose" % n][1].point
                                for n in ALL_NUMBERS
                            }
                        except KeyError:
                            continue
                    poses = {
                        "UAV%d" % n: [
                            p.x * math.sin(heading) - p.y * math.cos(heading),
                            p.x * math.cos(heading) + p.y * math.sin(heading),
                            p.z,
                        ]
                        for n, p in points.items()
                    }
                    stream.write(json.dumps({
                        "time": time.time(),
                        "phase": current_phase["name"],
                        "poses": poses,
                    }) + "\n")
                    stream.flush()

        recorder = threading.Thread(target=record, daemon=True)
        recorder.start()
        # 先确认 15 机位姿都经桥接到达，否则标定/原点确认会静默卡住。
        wait_poses(ALL_NUMBERS, timeout=180, label="all 15 swarm_pose")
        wait_for("calibration + 15 gp_origin_confirmed", origins_ready, timeout=300)

        TAKE_OFF_GROUPS = {
            "1": [[1, 2, 3], [4, 5, 6], [7]],
            "2": [[10, 11, 12], [13, 14, 15], [8, 9]],
        }

        def takeoff_team(team):
            current_phase["name"] = "team%s_takeoff" % team
            print("SETTLE 20s before team%s takeoff" % team, flush=True)
            time.sleep(20)
            for index, group in enumerate(TAKE_OFF_GROUPS[team]):
                name = "takeoff_team%s_%d" % (team, index)
                proc = launch(
                    name,
                    ["python3",
                     "src/swarm_uav_executor/scripts/offboard_takeoff_15_sitl.py",
                     *[str(n) for n in group]],
                    11310,
                )
                proc.wait()
                if proc.returncode != 0:
                    raise RuntimeError("%s failed; see %s.log" % (name, name))
                wait_for("team%s subgroup %s airborne (>2m)" % (team, group),
                         lambda group=group: airborne(group), timeout=90)
                time.sleep(5)
            wait_for("team%s all airborne (>2m)" % team,
                     lambda: airborne(TEAM_NUMBERS[team]), timeout=60)
            print("TEAM%s TAKEOFF complete, settling 10s" % team, flush=True)
            time.sleep(10)

        # ---- 批次1：起飞、任务、自动降落 ----
        takeoff_team("1")
        if not run_driver("1", args.tcp_team1):
            raise RuntimeError("team1 driver failed; see driver_team1.log")
        current_phase["name"] = "team1_landing"
        landing = launch(
            "land_team1",
            ["python3", "verification/two_teams_recon/sitl/land_15_sitl.py",
             *[str(n) for n in TEAM_NUMBERS["1"]]],
            11310,
        )
        landing.wait()
        if landing.returncode != 0:
            raise RuntimeError("team1 AUTO.LAND failed; see land_team1.log")
        print("TEAM1 LANDED", flush=True)

        # ---- 批次2：起飞、任务 ----
        takeoff_team("2")
        if not run_driver("2", args.tcp_team2):
            raise RuntimeError("team2 driver failed; see driver_team2.log")

        current_phase["name"] = "complete"
        summary["status"] = "SUCCESS"
    except Exception as exc:  # noqa: BLE001
        summary["error"] = str(exc)
        print("SITL ERROR: " + str(exc), flush=True)
    finally:
        stop.set()
        if recorder:
            recorder.join(2)
        summary["kept_alive"] = not args.no_keep_alive
        summary["orchestrator_exits"] = bool(args.leave_running)
        (out / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        stop_script = out / "stop_environment.sh"
        pids = [proc.pid for _name, proc in processes]
        stop_script.write_text(
            "#!/usr/bin/env bash\n"
            "# 停止本轮环境（两批飞行结束后保留环境时使用）。\n"
            "for pid in %s; do kill -INT -$pid 2>/dev/null; done\n"
            "sleep 3\n"
            "for pid in %s; do kill -9 -$pid 2>/dev/null; done\n"
            "pkill -9 -f '%s' 2>/dev/null\n"
            "kill -TERM %d 2>/dev/null\n"
            % (" ".join(map(str, pids)), " ".join(map(str, pids)),
               out / "ros_home", os.getpid())
        )
        stop_script.chmod(0o755)
        for handle in files:
            handle.close()
        if args.no_keep_alive:
            cleanup(processes, out)
            print(json.dumps(summary, ensure_ascii=False), flush=True)
            return 0 if summary["status"] == "SUCCESS" else 1
        if args.leave_running:
            print("ENVIRONMENT LEFT RUNNING (orchestrator exits): %s" % out, flush=True)
            print("停止环境：bash %s" % stop_script, flush=True)
            print(json.dumps(summary, ensure_ascii=False), flush=True)
            return 0 if summary["status"] == "SUCCESS" else 1
        print("ENVIRONMENT KEPT ALIVE: %s" % out, flush=True)
        print("停止环境：bash %s" % stop_script, flush=True)
        print(json.dumps(summary, ensure_ascii=False), flush=True)
        while True:
            time.sleep(5)


def cleanup(processes, out):
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
    try:
        output = subprocess.check_output(
            ["pgrep", "-f", str(Path(out) / "ros_home")], text=True
        )
    except subprocess.CalledProcessError:
        output = ""
    for pid_text in output.split():
        pid = int(pid_text)
        if pid in (os.getpid(), os.getppid()):
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


if __name__ == "__main__":
    sys.exit(main())
