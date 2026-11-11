"""Run a declared Stage 1 test group using the current Python environment."""
import argparse
import os
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("group", choices=["unit", "ros", "p2"])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    roots = [root, root / "vla_protocol/src", root / "vla_server/src", root / "cable_robot_vla"]
    env["PYTHONPATH"] = os.pathsep.join([*(str(p) for p in roots), env.get("PYTHONPATH", "")])
    groups = {
        "unit": ["vla_protocol/tests", "vla_server/tests", "cable_robot_vla/test/test_edge_core.py",
                 "cable_robot_vla/test/test_isolation.py", "cable_robot_vla/test/test_p2_adapter.py"],
        "ros": ["cable_robot_vla/test/test_ros_loop.py"],
        "p2": ["cable_robot_vla/test/test_p2_native.py"]}
    command = [sys.executable, "-B", "-m", "pytest", "-q", "-rs", "-p", "no:cacheprovider", *groups[args.group]]
    raise SystemExit(subprocess.call(command, cwd=root, env=env))


if __name__ == "__main__":
    main()
