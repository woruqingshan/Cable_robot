"""Real HTTP/DDS inspection rejection and official-model read-only preview."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import httpx
import pytest
from deployment.local.make_preview_fixture import synthetic_fixture
from vla_protocol import codec

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("semantics", ["absolute", "delta"])
def test_unmapped_fake_native_http_is_recorded_rejected_and_pipeline_continues(tmp_path, monkeypatch, semantics):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "75")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros"))
    from vla_server.config import ServerConfig, PolicyConfig
    from vla_protocol.profiles import RuntimeLimits
    import yaml
    config = ServerConfig(policy=PolicyConfig(type="fake", fake_semantics=semantics), limits=RuntimeLimits(max_horizon=64))
    path = tmp_path / "fake.yaml"
    path.write_text(yaml.safe_dump(config.model_dump()))
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    stream = (tmp_path / "server.log").open("w")
    process = subprocess.Popen([sys.executable, "-B", "-m", "vla_server.app", "--config", str(path), "--port", str(port)],
        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from cable_robot_vla.mock_robot_node import MockRobotNode
    from cable_robot_vla.vla_node import VLANode
    robot = pipeline = monitor = executor = None
    outcomes, commands = [], []
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end:
            try:
                if httpx.get(f"http://127.0.0.1:{port}/health", timeout=0.2, trust_env=False).status_code == 200: break
            except httpx.HTTPError: time.sleep(0.05)
        else: pytest.fail((tmp_path / "server.log").read_text())
        rclpy.init()
        robot = MockRobotNode(parameter_overrides=[Parameter("allow_unvalidated_simulation", value=True)])
        pipeline = VLANode(parameter_overrides=[Parameter("server_url", value=f"http://127.0.0.1:{port}"),
            Parameter("enable_native_protocol", value=True), Parameter("max_horizon", value=64),
            Parameter("advance_simulation", value=True), Parameter("allow_unvalidated_simulation", value=True),
            Parameter("log_dir", value=str(tmp_path / "loop"))])
        monitor = rclpy.create_node("native_inspection_monitor", namespace="/idc_vla/mock")
        subs = [monitor.create_subscription(String, "/idc_vla/mock/outcomes", lambda msg: outcomes.append(json.loads(msg.data)), 20),
                monitor.create_subscription(String, "/idc_vla/mock/execute", lambda msg: commands.append(msg.data), 20)]
        executor = SingleThreadedExecutor()
        for node in (robot, pipeline, monitor): executor.add_node(node)
        end = time.monotonic() + 12
        rejected = []
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            rejected = [v for v in outcomes if v.get("event") == "candidate_rejected"]
            if len(rejected) >= 3: break
        assert len(rejected) >= 3, outcomes
        candidates = [v for v in outcomes if v.get("event") == "candidate_received"]
        assert len(candidates) >= 3
        assert all(v["policy_action"]["shape"] == [50, 7] for v in candidates)
        assert all(v["policy_action"]["semantics"] == semantics for v in candidates)
        assert all(v["error"]["code"] == "UNSUPPORTED_ACTION_PROFILE" for v in rejected)
        assert len({v["request_id"] for v in rejected}) >= 3
        assert not commands and robot.state.q == [0.] * 5 and robot.state.state_revision == 0
        assert all(not p.topic_name.startswith("/servo") for p in pipeline.publishers)
        assert len(list(pipeline.clients)) == 0
    finally:
        if executor: executor.shutdown()
        if pipeline: pipeline.destroy_node()
        if robot: robot.destroy_node()
        if monitor: monitor.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8)
        stream.close()
    assert process.returncode == 0
    text = "\n".join(p.read_text() for p in (tmp_path / "loop").glob("*.jsonl"))
    assert "fake_model_native_v1" in text and "candidate_rejected" in text and "data_base64" not in text


def test_installed_read_only_preview_joint_order_timing_private_tf_and_live_rejection(tmp_path, monkeypatch):
    assert os.environ.get("IDC_P2_ROOT"), "explicit official model/P2 source root is required"
    assert os.environ.get("IDC_VLA_INSTALL"), "build and source the new cable_robot_vla install first"
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "76")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros"))
    fixture = synthetic_fixture()
    path = tmp_path / "fixture.json"
    path.write_bytes(codec.dumps(fixture))
    stream = (tmp_path / "preview.log").open("w")
    process = subprocess.Popen(["ros2", "launch", "cable_robot_vla", "trajectory_preview.launch.py",
        "workspace_root:=" + os.environ["IDC_P2_ROOT"], "input_path:=" + str(path),
        "allow_synthetic:=true", "gui:=false", "domain_id:=76", "python_executable:=" + sys.executable],
        stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    import tf2_ros
    node = executor = None
    statuses, states = [], []
    try:
        rclpy.init()
        node = rclpy.create_node("p3_preview_monitor", namespace="/idc_vla/preview",
            cli_args=["--ros-args", "-r", "/tf:=/idc_vla/preview/tf", "-r", "/tf_static:=/idc_vla/preview/tf_static"])
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, node, spin_thread=False)
        subs = [node.create_subscription(String, "/idc_vla/preview/status", lambda msg: statuses.append(json.loads(msg.data)), 20),
                node.create_subscription(JointState, "/idc_vla/preview/joint_states", lambda msg: states.append(msg), 20)]
        publisher = node.create_publisher(String, "/idc_vla/preview/candidate", 5)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        end = time.monotonic() + 15
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.03)
            if statuses and statuses[-1]["status"] == "COMPLETE": break
            assert process.poll() is None, (tmp_path / "preview.log").read_text()
        assert statuses and statuses[-1]["status"] == "COMPLETE", (tmp_path / "preview.log").read_text()
        assert len(states) >= 20 and any(abs(state.position[0]) > 0.05 for state in states)
        assert all(list(state.name[:5]) == ["joint1", "joint2", "joint3", "joint4", "joint5"] for state in states)
        assert all(v["execution_authorized"] is False and v["validation_status"] == "NOT_CHECKED" for v in statuses)
        from cable_robot_vla.visualization.playback import load_model, TrajectoryPlayback
        playback = TrajectoryPlayback(fixture, allow_synthetic=True)
        for value in statuses:
            assert value["q"] == pytest.approx(playback.at(value["elapsed_s"]))
            assert value["source_action_id"] == fixture.source_action_id
        model = load_model(os.environ["IDC_P2_ROOT"])
        from workspace_analysis.robot.fk import ForwardKinematics
        import numpy as np
        from scipy.spatial.transform import Rotation
        expected = ForwardKinematics(model)([0.] * 5)
        end = time.monotonic() + 4
        error = None
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            try:
                transform = buffer.lookup_transform("base_link", model.ee_frame, Time()).transform
                matrix = np.eye(4)
                matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
                rotation = transform.rotation
                matrix[:3, :3] = Rotation.from_quat([rotation.x, rotation.y, rotation.z, rotation.w]).as_matrix()
                error = float(np.max(np.abs(matrix - expected)))
                if error < 1e-9: break
            except Exception: pass
        assert error is not None and error < 1e-9
        publisher.publish(String(data='{"schema_version":1,"native_action_space":"unmapped_model"}'))
        end = time.monotonic() + 3
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            if statuses[-1]["error"] is not None: break
        assert statuses[-1]["error"] is not None
        assert statuses[-1]["source_action_id"] == fixture.source_action_id
        graph = dict(node.get_topic_names_and_types())
        assert "/idc_vla/preview/tf" in graph and "/idc_vla/preview/joint_states" in graph
        assert not any("servo" in name or "kinematics" in name or name == "/idc_vla/mock/execute" for name in graph)
    finally:
        if executor: executor.shutdown()
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        process.send_signal(signal.SIGINT)  # Launch alone forwards to its owned children.
        process.wait(timeout=8)
        stream.close()
    log = (tmp_path / "preview.log").read_text()
    assert process.returncode == 0 and "process has died" not in log
    assert log.count("process has finished cleanly") >= 2
