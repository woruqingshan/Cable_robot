"""Standard-library-only bootstrap for the pinned HTTP server environment."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--venv", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    target = Path(args.venv).expanduser().resolve()
    try:
        if target == Path("/") or target == Path.home() or target.is_relative_to(ROOT) or ROOT.is_relative_to(target):
            raise ValueError("venv must be separate from the checkout, root and home directories")
        if target.exists() and not (target / "pyvenv.cfg").is_file():
            raise ValueError("existing directory is not a venv and will not be overwritten")
        commands = [] if target.exists() else [[sys.executable, "-m", "venv", str(target)]]
        commands += [[str(target / "bin/python"), "-m", "pip", "install", "--no-deps", "-r", str(ROOT / "deployment/cloud/requirements-server.lock")],
            [str(target / "bin/python"), "-m", "pip", "install", "--no-build-isolation", "--no-deps", str(ROOT / "vla_protocol"), str(ROOT / "vla_server")]]
        if args.execute:
            for command in commands: subprocess.run(command, check=True, stdout=sys.stderr)
        print(json.dumps({"operation": "install-server", "executed": args.execute, "commands": commands,
                          "gpu_inference_verified": False}, indent=2))
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(json.dumps({"schema_version": 1, "code": "SERVER_BOOTSTRAP_FAILED", "message": str(exc), "request_id": None}), file=sys.stderr)
        raise SystemExit(2)


if __name__ == "__main__": main()
