"""Owned-process P3 preview acceptance, optionally capturing actual RViz rendering."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from vla_protocol import codec
from deployment.local.make_preview_fixture import synthetic_fixture

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--domain", type=int, default=77)
    parser.add_argument("--render-binary", help="Existing read-only RViz native snapshot helper; no synthetic image fallback")
    args = parser.parse_args()
    if not 1 <= args.domain <= 101: parser.error("private domain must be 1..101")
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    fixture = output / "synthetic_trajectory.json"
    fixture.write_bytes(codec.dumps(synthetic_fixture()))
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(output / "ros"), PYTHONDONTWRITEBYTECODE="1")
    stream = (output / "launch.log").open("x")
    process = subprocess.Popen(["ros2", "launch", "cable_robot_vla", "trajectory_preview.launch.py",
        "workspace_root:=" + str(Path(args.workspace_root).resolve()), "input_path:=" + str(fixture),
        "allow_synthetic:=true", "gui:=false", "loop:=true", "domain_id:=" + str(args.domain),
        "python_executable:=" + sys.executable], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    report = {"schema_version": 1, "status": "FAIL", "execution_authorized": False, "hardware_validated": False,
              "gpu_inference_verified": False, "visual_render_verified": False}
    renderer = render_log = node = executor = None
    statuses, states = [], []
    import rclpy
    try:
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("p3_preview_acceptance", namespace="/idc_vla/preview")
        subscriptions = [node.create_subscription(String, "/idc_vla/preview/status", lambda msg: statuses.append(json.loads(msg.data)), 50),
            node.create_subscription(JointState, "/idc_vla/preview/joint_states", lambda msg: states.append({"names": list(msg.name), "positions": list(msg.position)}), 50)]
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        end = time.monotonic() + 10
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if len(states) >= 10: break
            if process.poll() is not None: raise RuntimeError("preview launch exited early")
        if len(states) < 10: raise RuntimeError("no real JointState preview stream")
        if args.render_binary:
            binary = Path(args.render_binary).resolve(strict=True)
            render_log = (output / "rviz.log").open("x")
            env = dict(os.environ, QT_QPA_PLATFORM="xcb" if os.environ.get("DISPLAY") else "offscreen", LIBGL_ALWAYS_SOFTWARE="1")
            renderer = subprocess.Popen([str(binary), str(ROOT / "cable_robot_vla/config/trajectory_preview.rviz"), str(output / "rviz_actual.png"),
                "--ros-args", "-r", "__ns:=/idc_vla/preview", "-r", "/tf:=/idc_vla/preview/tf", "-r", "/tf_static:=/idc_vla/preview/tf_static"],
                env=env, stdout=render_log, stderr=subprocess.STDOUT, start_new_session=True)
            end = time.monotonic() + 30
            while renderer.poll() is None and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.05)
            if renderer.poll() != 0: raise RuntimeError("native RViz rendering failed or timed out")
            from PIL import Image
            with Image.open(output / "rviz_actual.png") as image:
                if image.width < 100 or image.height < 100 or all(low == high for low, high in image.convert("RGB").getextrema()):
                    raise RuntimeError("RViz render is empty or uniform")
                report.update(visual_render_verified=True, render_method="RViz VisualizationFrame / RenderWindow.captureScreenShot",
                              render_size=list(image.size))
        else:
            end = time.monotonic() + 3
            while time.monotonic() < end: executor.spin_once(timeout_sec=0.05)
        if not any(abs(row["positions"][0]) > 0.05 for row in states): raise RuntimeError("no changing synthetic trajectory observed")
        if any(row.get("execution_authorized") is not False or row.get("validation_status") != "NOT_CHECKED" for row in statuses):
            raise RuntimeError("preview status lost its read-only/unvalidated identity")
        graph = dict(node.get_topic_names_and_types())
        if any("servo" in name or "kinematics" in name or name.endswith("/execute") for name in graph):
            raise RuntimeError("unexpected command endpoint in private preview graph")
        report.update(status="PASS", observed_joint_states=len(states), observed_statuses=len(statuses), topics=graph,
            unique_arm_positions=len({tuple(row["positions"][:5]) for row in states}),
            loaded_model_sha256=statuses[-1]["loaded_model_sha256"], source_action_id=statuses[-1]["source_action_id"])
    except Exception as exc:
        report["error"] = type(exc).__name__ + ": " + str(exc)
    finally:
        if executor: executor.shutdown()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        if renderer and renderer.poll() is None:
            os.killpg(renderer.pid, signal.SIGINT)
            try: renderer.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(renderer.pid, signal.SIGKILL); renderer.wait(timeout=5)
        if process.poll() is None: process.send_signal(signal.SIGINT)
        try: process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
        stream.close()
        if render_log: render_log.close()
        launch_log = (output / "launch.log").read_text()
        report.update(launch_exit_code=process.returncode,
            clean_shutdown=process.returncode == 0 and "process has died" not in launch_log and launch_log.count("process has finished cleanly") >= 2)
        if not report["clean_shutdown"]: report["status"] = "FAIL"
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        (output / "status_samples.json").write_text(json.dumps(statuses, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["status"] == "PASS" else 1)


if __name__ == "__main__": main()
