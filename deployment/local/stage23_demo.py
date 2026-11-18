#!/usr/bin/env python3
"""Owned end-to-end Fortress/ROS/RViz/cloud acceptance. Never starts hardware drivers."""
import argparse
from collections import deque
from hashlib import sha256
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
from time import monotonic, sleep

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "vla_protocol/src"), str(ROOT / "cable_robot_vla")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-root", default=str(ROOT / "workspace_analysis"))
    parser.add_argument("--backend", required=True, help="existing native P2/FCL executable")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--server-url", default="http://127.0.0.1:18000")
    parser.add_argument("--domain", type=int, default=84)
    parser.add_argument("--enable-simulation", action="store_true", help="enable an independent P2-validated rule-generated scenario")
    parser.add_argument("--render-binary", help="existing RViz native snapshot helper; absence leaves visual render unverified")
    parser.add_argument("--plugin-dir", help="existing compiled stage23 initialization plugin; otherwise build locally")
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--fault-tests", action="store_true")
    parser.add_argument("--prepare-only", action="store_true", help="prepare P2-verified runtime/plugin for stage23.launch.py, without cloud access or simulation startup")
    args = parser.parse_args()
    from urllib.parse import urlparse
    parsed = urlparse(args.server_url)
    if parsed.hostname != "127.0.0.1" or parsed.scheme != "http" or parsed.username or parsed.password or parsed.query or parsed.fragment:
        parser.error("use the private localhost SSH tunnel")
    if not 1 <= args.domain <= 101:
        parser.error("require private ROS domain in 1..101")
    output = Path(args.output_dir).resolve()
    if output.is_relative_to(ROOT):
        parser.error("experiment outputs must be outside source")
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema_version": 1, "status": "FAIL", "test_kind": "genuine_cloud_simulation_integration",
        "physical_actuation": False, "native_actions_executable": False, "synthetic_control_enabled": args.enable_simulation,
        "source_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_dirty": bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT, text=True).strip()),
        "visual_render_verified": False, "process_exit_codes": {}}
    processes, streams = [], []
    monitor = None
    statuses, native, observations = [], [], []
    states = deque(maxlen=1000)
    counts = {"camera": 0, "markers": 0, "native_values": 0, "control": 0}
    simulator = bridge = pipeline = renderer = None
    image_saved = False
    subscriptions = []
    def start(name, command, env):
        stream = (output / (name + ".log")).open("x")
        streams.append(stream)
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))
        return process
    def stop(process):
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
    try:
        import httpx
        if not args.prepare_only:
            health = httpx.get(args.server_url + "/health", timeout=10, trust_env=False).json()
            assert health["model_type"] == "smolvla" and health["gpu_model_ready"] and health["policy_ready"]
            report["cloud_source_revision"] = health["source_revision"]
        from cable_robot_vla.integration.gazebo import prepare_runtime, PREFIX
        runtime_dir = output / "runtime"
        runtime = prepare_runtime(workspace_root=args.workspace_root, backend=args.backend, output=runtime_dir, domain=args.domain)
        report["runtime"] = runtime
        plugin_dir = Path(args.plugin_dir).resolve() if args.plugin_dir else output / "plugin-build"
        if not args.plugin_dir:
            with (output / "plugin-build.log").open("x") as log:
                subprocess.run(["cmake", "-S", str(ROOT / "cable_robot_vla/simulation"), "-B", str(plugin_dir), "-DCMAKE_BUILD_TYPE=Release"], stdout=log, stderr=subprocess.STDOUT, check=True)
                subprocess.run(["cmake", "--build", str(plugin_dir), "-j2"], stdout=log, stderr=subprocess.STDOUT, check=True)
        assert (plugin_dir / "libstage23_initialize.so").is_file()
        report["initialization_plugin_sha256"] = sha256((plugin_dir / "libstage23_initialize.so").read_bytes()).hexdigest()
        if args.prepare_only:
            report.update(status="PREPARED", test_kind="static_native_p2_preparation")
            return 0
        env = dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(args.domain), IGN_PARTITION=runtime["partition"],
            IGN_GAZEBO_SYSTEM_PLUGIN_PATH=str(plugin_dir), ROS_LOG_DIR=str(output / "ros"),
            PYTHONDONTWRITEBYTECODE="1", LIBGL_ALWAYS_SOFTWARE="1")
        env["PYTHONPATH"] = os.pathsep.join([str(ROOT), str(ROOT / "vla_protocol/src"), str(ROOT / "cable_robot_vla"), env.get("PYTHONPATH", "")])
        os.environ.update({name:env[name] for name in ("ROS_LOCALHOST_ONLY", "ROS_DOMAIN_ID", "ROS_LOG_DIR", "IGN_PARTITION")})
        simulator = start("gazebo", ["ign", "gazebo", "-s", "-r", "-v", "2", str(runtime_dir / "stage23.sdf")], env)
        (runtime_dir / "owner.json").write_text(json.dumps({"simulator_pid": simulator.pid}))
        bridge = start("bridge", ["/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge", "--ros-args", "-p", "config_file:=" + str(runtime_dir / "bridge.yaml")], env)
        import yaml
        rsp_params = output / "rsp.yaml"
        rsp_params.write_text(yaml.safe_dump({"/**": {"ros__parameters": {"robot_description": (runtime_dir / "source_robot.urdf").read_text(), "use_sim_time": True}}}))
        start("robot_state_publisher", ["/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher", "--ros-args",
            "--params-file", str(rsp_params), "-r", "__ns:=" + PREFIX, "-r", "joint_states:=" + PREFIX + "/joint_states",
            "-r", "/tf:=" + PREFIX + "/tf", "-r", "/tf_static:=" + PREFIX + "/tf_static", "-r", "/clock:=" + PREFIX + "/clock"], env)
        def pipeline_command(url, logdir):
            return [sys.executable, "-B", "-m", "cable_robot_vla.integration.node", "--ros-args",
                "-p", "use_sim_time:=true", "-p", "runtime_path:=" + str(runtime_dir / "runtime.json"),
                "-p", "log_dir:=" + str(logdir), "-p", "server_url:=" + url,
                "-p", "execution_backend:=" + ("gazebo" if args.enable_simulation else "inspection"),
                "-p", "enable_simulation:=" + ("true" if args.enable_simulation else "false"),
                "-r", "/clock:=" + PREFIX + "/clock"]
        pipeline = start("pipeline", pipeline_command(args.server_url, output / "loop"), env)
        import rclpy
        from sensor_msgs.msg import JointState, Image
        from std_msgs.msg import String, Float64, Float64MultiArray
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import qos_profile_sensor_data
        rclpy.init()
        monitor = rclpy.create_node("stage23_acceptance", namespace=PREFIX)
        def joints(msg):
            values = dict(zip(msg.name, msg.position))
            if all(f"joint{i}" in values for i in range(1,6)):
                states.append({"q": [float(values[f"joint{i}"]) for i in range(1,6)],
                    "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9})
        def image(msg):
            nonlocal image_saved
            counts["camera"] += 1
            if not image_saved and msg.encoding == "rgb8":
                from PIL import Image as PILImage
                data = bytes(msg.data)
                raw = b"".join(data[row*msg.step:row*msg.step+msg.width*3] for row in range(msg.height))
                PILImage.frombytes("RGB", (msg.width,msg.height), raw).save(output / "gazebo_camera.png")
                report["camera"] = {"frame_id": msg.header.frame_id, "width": msg.width, "height": msg.height,
                    "byte_range": [min(raw),max(raw)], "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec/1e9}
                image_saved = True
        def increment(kind):
            def callback(_): counts[kind] += 1
            return callback
        subscriptions = [monitor.create_subscription(JointState, PREFIX + "/joint_states", joints, qos_profile_sensor_data),
            monitor.create_subscription(Image, PREFIX + "/camera/image", image, qos_profile_sensor_data),
            monitor.create_subscription(String, PREFIX + "/status", lambda m:statuses.append(json.loads(m.data)), 100),
            monitor.create_subscription(String, PREFIX + "/native_actions", lambda m:native.append(json.loads(m.data)), 30),
            monitor.create_subscription(String, PREFIX + "/observation_state", lambda m:observations.append(json.loads(m.data)), 30),
            monitor.create_subscription(Float64MultiArray, PREFIX + "/native_values", increment("native_values"), 10),
            monitor.create_subscription(MarkerArray, PREFIX + "/markers", increment("markers"), 10),
            *[monitor.create_subscription(Float64, PREFIX + f"/control/joint{i}/position", increment("control"), 100) for i in range(1,6)]]
        deadline = monotonic() + args.timeout
        completed = False
        while monotonic() < deadline:
            rclpy.spin_once(monitor, timeout_sec=0.05)
            for name, proc in processes:
                if name != "rviz" and proc.poll() is not None:
                    raise RuntimeError(f"{name} exited early: {proc.returncode}")
            if args.render_binary and renderer is None and len(native) >= 1:
                renderer = start("rviz", [str(Path(args.render_binary).resolve()), str(ROOT / "cable_robot_vla/config/stage23.rviz"),
                    str(output / "rviz_actual.png"), "--ros-args", "-r", "__ns:=" + PREFIX,
                    "-r", "/tf:=" + PREFIX + "/tf", "-r", "/tf_static:=" + PREFIX + "/tf_static",
                    "-r", "/clock:=" + PREFIX + "/clock", "-p", "use_sim_time:=true"], env)
            completed = any(s.get("event") == "scenario_complete" for s in statuses)
            after = [o for o in observations if max(abs(a-b) for a,b in zip(o["positions"], runtime["scenario_target"])) < 0.02]
            ready = (completed and len(after) >= 3) if args.enable_simulation else len(native) >= 3
            render_done = not args.render_binary or (renderer is not None and renderer.poll() is not None)
            if ready and render_done: break
        assert len(native) >= 3, "genuine cloud action stream missing"
        assert all(a["shape"] == [50,6] and a["synthetic"] is False and a["policy"]["policy_type"] == "smolvla" for a in native)
        assert counts["markers"] > 0 and counts["native_values"] > 0 and image_saved and states
        assert report["camera"]["byte_range"][0] < report["camera"]["byte_range"][1]
        rejected = [s for s in statuses if s.get("event") == "native_rejected"]
        assert len(rejected) >= 3 and all(s["error"]["code"] == "UNSUPPORTED_ACTION_PROFILE" for s in rejected)
        if args.enable_simulation:
            assert completed and len(after) >= 3, "simulator movement/feedback loop incomplete"
            assert counts["control"] > 0
            assert max(abs(a-b) for a,b in zip(states[-1]["q"],runtime["initial_positions"])) > 0.05
            validations = [s["validated_trajectory"] for s in statuses if s.get("event") == "scenario_validation"]
            assert validations and all(v["status"] == "PASS_RESEARCH_ONLY" and v["provenance"]["workspace_checked"] for v in validations)
        else:
            assert counts["control"] == 0
            assert max(abs(a-b) for a,b in zip(states[-1]["q"],runtime["initial_positions"])) < 0.001
        if args.render_binary:
            assert renderer.returncode == 0 and (output / "rviz_actual.png").is_file()
            from PIL import Image as PILImage
            with PILImage.open(output / "rviz_actual.png") as img:
                assert img.width > 100 and img.height > 100 and any(a != b for a,b in img.convert("RGB").getextrema())
                report["visual_render_verified"] = True
                report["render_size"] = list(img.size)
        graph = dict(monitor.get_topic_names_and_types())
        assert not any("servo" in name or "cmd_vel" in name or "follow_joint_trajectory" in name or name.endswith("/execute") for name in graph)
        stop(pipeline)
        report.update(genuine_chunks=len(native), native_shape=[50,6], native_rejections=len(rejected), counts=dict(counts),
            final_positions=states[-1]["q"], simulated_feedback_in_later_observations=len(after) if args.enable_simulation else 0,
            source_timestamps_preserved=all(o["stamp"]["clock_domain"] == "ros_sim" for o in observations),
            control_source="rule_generated_simulation_only" if args.enable_simulation else "none", topics=graph)
        if args.fault_tests:
            with socket.socket() as probe:
                probe.bind(("127.0.0.1",0)); unused = probe.getsockname()[1]
            baseline_commands = counts["control"]
            fault = start("network_fault", pipeline_command(f"http://127.0.0.1:{unused}", output / "fault-loop"), env)
            end = monotonic()+15
            while monotonic()<end:
                rclpy.spin_once(monitor,timeout_sec=0.05)
                if any(s.get("error",{}).get("code") == "NETWORK_ERROR" for s in statuses): break
            assert any(s.get("error",{}).get("code") == "NETWORK_ERROR" for s in statuses)
            stop(bridge)
            end = monotonic()+5
            while monotonic()<end:
                rclpy.spin_once(monitor,timeout_sec=0.05)
                if any(s.get("error",{}).get("code") == "STALE_OBSERVATION" for s in statuses): break
            assert any(s.get("error",{}).get("code") == "STALE_OBSERVATION" for s in statuses)
            assert counts["control"] == baseline_commands, "faulted node sent simulation commands"
            stop(fault)
            report["network_failure_safe"] = report["missing_sensor_stream_safe"] = True
        report["status"] = "PASS"
    except Exception:
        import traceback
        report["error"] = traceback.format_exc()
    finally:
        if monitor is not None:
            monitor.destroy_node()
            import rclpy
            if rclpy.ok(): rclpy.shutdown()
        for name, process in reversed(processes):
            try: stop(process)
            except Exception as exc:
                report["status"] = "FAIL"; report.setdefault("cleanup_errors",[]).append(name+": "+str(exc))
            report["process_exit_codes"][name] = process.poll()
        for stream in streams: stream.close()
        (output / "statuses.json").write_text(json.dumps(statuses,indent=2,allow_nan=False)+"\n")
        (output / "observations.json").write_text(json.dumps(observations,indent=2,allow_nan=False)+"\n")
        (output / "report.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
        print(json.dumps({"status":report["status"],"report":str(output / "report.json"),"error":report.get("error")},indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__": raise SystemExit(main())
