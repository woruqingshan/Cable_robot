import ast
from pathlib import Path
import subprocess
import sys
import pytest

ROOT = Path(__file__).resolve().parents[2]


def test_no_hardware_or_gpu_imports_and_only_standard_ros_endpoints():
    forbidden = ("cable_robot_control", "servo_controller_msgs", "ros_robot_controller_msgs", "kinematics_msgs", "torch", "lerobot")
    for path in (ROOT / "cable_robot_vla/cable_robot_vla").rglob("*.py"):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(not alias.name.startswith(forbidden) for alias in node.names), path
            if isinstance(node, ast.ImportFrom):
                assert not (node.module or "").startswith(forbidden), path
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                assert node.func.attr != "create_client", path
    for package in ("vla_protocol", "vla_server"):
        script = f"import {package}; import sys; assert not any(n.startswith(('rclpy','torch','lerobot','workspace_analysis')) for n in sys.modules)"
        subprocess.run([sys.executable, "-B", "-c", script], check=True)


def test_isolation_environment_is_required(monkeypatch):
    from cable_robot_vla.ros_support import require_isolation
    from vla_protocol.errors import VLAError
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    monkeypatch.setenv("ROS_DOMAIN_ID", "0")
    with pytest.raises(VLAError): require_isolation()
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_DOMAIN_ID", "71")
    require_isolation()
