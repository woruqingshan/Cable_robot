"""Run a declared Stage 1 test group using the current Python environment."""
import argparse
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("group", choices=["unit", "ros", "p2", "stage15-unit", "stage15-ros"])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    roots = [root, root / "vla_protocol/src", root / "vla_server/src", root / "cable_robot_vla"]
    env["PYTHONPATH"] = os.pathsep.join([*(str(p) for p in roots), env.get("PYTHONPATH", "")])
    groups = {
        "unit": ["vla_protocol/tests/test_contract.py", "vla_server/tests/test_http_service.py", "cable_robot_vla/test/test_edge_core.py",
                 "cable_robot_vla/test/test_isolation.py", "cable_robot_vla/test/test_p2_adapter.py"],
        "ros": ["cable_robot_vla/test/test_ros_loop.py"],
        "p2": ["cable_robot_vla/test/test_p2_native.py"],
        "stage15-unit": ["vla_protocol/tests/test_stage15_compatibility.py", "vla_server/tests/test_stage15_model_boundary.py",
            "cable_robot_vla/test/test_stage15_adaptation_preview.py", "deployment/cloud/tests"],
        "stage15-ros": ["cable_robot_vla/test/test_stage15_ros.py"]}
    command = [sys.executable, "-B", "-m", "pytest", "-q", "-rs", "-p", "no:cacheprovider", *groups[args.group]]
    raise SystemExit(subprocess.call(command, cwd=root, env=env))


if __name__ == "__main__":
    main()
