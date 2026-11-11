"""Owned-process acceptance probe: real DDS, real HTTP, default Hold Position."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--domain", type=int, default=72)
    parser.add_argument("--launch", action="store_true", help="use the installed ROS launch file instead of direct Python nodes")
    args = parser.parse_args()
    if not 1 <= args.domain <= 101 or not 1 <= args.cycles <= 100:
        parser.error("domain must be 1..101 and cycles 1..100")
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(output / "ros"), PYTHONDONTWRITEBYTECODE="1")
    paths = [root, root / "vla_protocol/src", root / "vla_server/src", root / "cable_robot_vla"]
    os.environ["PYTHONPATH"] = os.pathsep.join([*(str(p) for p in paths), os.environ.get("PYTHONPATH", "")])
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    endpoint = f"http://127.0.0.1:{port}"
    processes, streams, events = [], [], []
    report = {"schema_version": 1, "simulation_only": True, "hardware_validated": False,
              "status": "FAIL", "endpoint": endpoint, "ros_domain_id": args.domain, "process_exit_codes": {}}
    node = executor = None
    try:
        def start(name, command, python=True):
            stream = (output / (name + ".log")).open("x")
            streams.append(stream)
            process = subprocess.Popen(([sys.executable, "-B"] if python else []) + command, cwd=root,
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append((name, process))
        start("server", ["-m", "vla_server.app", "--port", str(port)])
        import httpx
        end = time.monotonic() + 10
        while time.monotonic() < end:
            try:
                if httpx.get(endpoint + "/health", timeout=0.3, trust_env=False).status_code == 200: break
            except httpx.HTTPError:
                time.sleep(0.05)
        else:
            raise RuntimeError("HTTP server did not become ready")
        if args.launch:
            start("ros_launch", ["ros2", "launch", "cable_robot_vla", "local_mock.launch.py",
                "python_executable:=" + sys.executable, "server_url:=" + endpoint,
                "log_dir:=" + str(output / "loops"), "advance_simulation:=true",
                "allow_unvalidated_simulation:=true"], python=False)
        else:
            start("robot", ["-m", "cable_robot_vla.mock_robot_node", "--ros-args", "-p", "allow_unvalidated_simulation:=true"])
            start("pipeline", ["-m", "cable_robot_vla.vla_node", "--ros-args", "-p", "server_url:=" + endpoint,
                "-p", "log_dir:=" + str(output / "loops"), "-p", "advance_simulation:=true", "-p", "allow_unvalidated_simulation:=true"])
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("stage1_smoke_monitor", namespace="/idc_vla/mock")
        subscription = node.create_subscription(String, "/idc_vla/mock/execution_status", lambda msg: events.append(json.loads(msg.data)), 20)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        end = time.monotonic() + 20
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            completed = [e for e in events if e.get("status") == "SIMULATED_UNVALIDATED"]
            if len(completed) >= args.cycles: break
            if any(p.poll() is not None for _, p in processes):
                raise RuntimeError("a child process exited before acceptance")
        if len(completed) < args.cycles:
            raise RuntimeError("insufficient completed mock cycles")
        graph = dict(node.get_topic_names_and_types())
        if any("servo" in name or "kinematics" in name for name in graph):
            raise RuntimeError("unexpected motor-related topic in private graph")
        report.update(status="PASS", completed_cycles=len(completed), events=events, topics=graph,
                      final_positions=completed[-1]["positions"], validation_mode="NOT_CHECKED")
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if executor: executor.shutdown()
        if node:
            node.destroy_node()
            import rclpy
            rclpy.shutdown()
        # Stop edge nodes before the server, and only signal processes owned here.
        for name, process in reversed(processes):
            if process.poll() is None:
                if name == "ros_launch":
                    # Launch forwards SIGINT to its children. Signalling its group
                    # as well delivers two interrupts during node destruction.
                    process.send_signal(signal.SIGINT)
                else:
                    os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            report.setdefault("process_exit_codes", {})[name] = process.returncode
        for stream in streams: stream.close()
        report["clean_shutdown"] = all(p.returncode == 0 for _, p in processes)
        if args.launch:
            launch_log = (output / "ros_launch.log").read_text()
            report["launch_children_clean"] = ("process has died" not in launch_log
                and launch_log.count("process has finished cleanly") >= 2)
            report["clean_shutdown"] = report["clean_shutdown"] and report["launch_children_clean"]
        if not report["clean_shutdown"]: report["status"] = "FAIL"
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("status", "clean_shutdown", "process_exit_codes")}, indent=2))
    print(output / "report.json")
    raise SystemExit(0 if report["status"] == "PASS" else 1)


if __name__ == "__main__":
    main()
