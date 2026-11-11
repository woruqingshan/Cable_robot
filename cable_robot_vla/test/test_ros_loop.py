"""Real DDS + localhost HTTP integration; also verifies the mock ROS graph."""
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


@pytest.mark.parametrize("policy", ["hold", "controlled_motion_fixture"])
def test_real_ros_http_repeated_loop_and_shutdown(tmp_path, monkeypatch, policy):
    # This test owns all launched components; it never starts a vendor overlay node.
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", os.environ.get("IDC_VLA_TEST_DOMAIN", "71"))
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_logs"))
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    log = (tmp_path / "server.log").open("w")
    command = [sys.executable, "-B", "-m", "vla_server.app", "--port", str(port)]
    if policy != "hold":
        command = [sys.executable, "-B", "-m", "uvicorn", "deployment.local.test_support:motion_fixture_app",
                   "--factory", "--host", "127.0.0.1", "--port", str(port)]
    server = subprocess.Popen(command,
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from cable_robot_vla.mock_robot_node import MockRobotNode
    from cable_robot_vla.vla_node import VLANode
    from sensor_msgs.msg import JointState, Image
    from std_msgs.msg import String
    robot = pipeline = monitor = executor = None
    results, states, images = [], [], []
    try:
        end = time.monotonic() + 10
        while time.monotonic() < end:
            try:
                if httpx.get(f"http://127.0.0.1:{port}/health", trust_env=False).status_code == 200: break
            except httpx.HTTPError:
                time.sleep(0.05)
        else:
            pytest.fail("server startup failed: " + (tmp_path / "server.log").read_text())
        rclpy.init()
        robot = MockRobotNode(parameter_overrides=[Parameter("allow_unvalidated_simulation", value=True)])
        pipeline = VLANode(parameter_overrides=[Parameter("server_url", value=f"http://127.0.0.1:{port}"),
            Parameter("advance_simulation", value=True), Parameter("allow_unvalidated_simulation", value=True),
            Parameter("log_dir", value=str(tmp_path / "loop"))])
        monitor = rclpy.create_node("stage1_test_monitor", namespace="/idc_vla/mock")
        subscriptions = [
            monitor.create_subscription(String, "/idc_vla/mock/outcomes", lambda msg: results.append(json.loads(msg.data)), 20),
            monitor.create_subscription(JointState, "/idc_vla/mock/joint_states", lambda msg: states.append(msg), 20),
            monitor.create_subscription(Image, "/idc_vla/mock/rgb", lambda msg: images.append(msg), 20)]
        executor = SingleThreadedExecutor()
        for node in (robot, pipeline, monitor): executor.add_node(node)
        end = time.monotonic() + 15
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
            done = [r for r in results if r.get("event") == "execution" and r.get("status") == "SIMULATE_UNVALIDATED"]
            if len(done) >= 3 and robot.state.state_revision >= 6: break
        assert len(done) >= 3, results
        assert images and states
        assert images[-1].encoding == "rgb8"
        assert list(states[-1].name) == ["joint1", "joint2", "joint3", "joint4", "joint5"]
        if policy == "hold":
            assert robot.state.q == [0.] * 5  # deterministic hold, with real execution revisions
        else:
            assert robot.state.q[0] >= 0.02 and robot.state.q[1:] == [0.] * 4
            assert max(s.position[0] for s in states) >= 0.02
        assert all(r["validated_trajectory"]["status"] == "NOT_CHECKED" for r in done)
        for node in (robot, pipeline):
            publishers = node.publishers
            assert all(p.topic_name.startswith("/idc_vla/mock/") or p.topic_name in ("/rosout", "/parameter_events") for p in publishers)
            assert len(list(node.clients)) == 0
        graph = dict(monitor.get_topic_names_and_types())
        assert not any("servo" in name or "kinematics" in name or "bus_servo" in name for name in graph)
        # Observe real task revision invalidation through the normal ROS path.
        task_pub = monitor.create_publisher(String, "/idc_vla/mock/task_instruction", 5)
        end = time.monotonic() + 3
        while task_pub.get_subscription_count() == 0 and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        task_pub.publish(String(data="Hold after task revision change"))
        end = time.monotonic() + 3
        while robot.state.task_revision < 1 and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        assert robot.state.task_revision == 1
    finally:
        if executor: executor.shutdown()
        if pipeline: pipeline.destroy_node()
        if robot: robot.destroy_node()
        if monitor: monitor.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        os.killpg(server.pid, signal.SIGINT)
        try: server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(server.pid, signal.SIGKILL)
            server.wait(timeout=5)
        log.close()
    assert server.returncode == 0
    rows = [json.loads(line) for path in (tmp_path / "loop").glob("*.jsonl") for line in path.read_text().splitlines()]
    assert rows[-1]["event"] == "shutdown" and rows[-1]["status"] == "CLEAN"
    assert len({r["request_id"] for r in rows if r["event"] == "observation"}) >= 3
    assert "data_base64" not in json.dumps(rows)


def test_unsafe_topic_remap_is_rejected_before_command_publisher(monkeypatch):
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "71")
    import rclpy
    from cable_robot_vla.ros_support import configure_topics, NAMESPACE
    from vla_protocol.errors import VLAError
    rclpy.init()
    node = rclpy.create_node("remap_probe", namespace=NAMESPACE,
                            cli_args=["--ros-args", "-r", "execute:=/servo_controller"])
    try:
        with pytest.raises(VLAError, match="outside"):
            configure_topics(node)
        assert not any(p.topic_name == "/servo_controller" for p in node.publishers)
    finally:
        node.destroy_node()
        rclpy.shutdown()
