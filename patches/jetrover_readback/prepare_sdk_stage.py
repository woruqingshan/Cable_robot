"""Prepare a versioned SDK overlay only. No serial opens, RPCs or process control."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess

from patches.jetrover_readback.build_candidate_patch import (
    candidate_sources, HERE, SDK_SHA, NODE_SHA)


def prepare(output, vendor, revision, reference=""):
    output, vendor = Path(output), Path(vendor)
    if len(revision) != 40 or any(c not in "0123456789abcdef" for c in revision):
        raise ValueError("full lowercase Git SHA required")
    if not isinstance(reference, str) or len(reference) > 512 or (reference and not reference.strip()):
        raise ValueError("invalid diagnostic reference")
    sdk = (vendor/"ros_robot_controller_sdk.py").read_text()
    node = (vendor/"ros_robot_controller_node.py").read_text()
    sdk, node = candidate_sources(sdk, node)
    # All validation precedes mkdir; refuse overwrite, including symlinks.
    output.mkdir(mode=0o700, parents=False, exist_ok=False)
    package = output/"ros_robot_controller"
    package.mkdir()
    files = {
        "ros_robot_controller/__init__.py": "",
        "ros_robot_controller/ros_robot_controller_sdk.py": sdk,
        "ros_robot_controller/ros_robot_controller_node.py": node,
        "ros_robot_controller/guarded_readback.py": (HERE/"guarded_readback.py").read_text(),
        "maintenance.json": json.dumps({"ros_robot_controller": {"ros__parameters": {
            "readback_maintenance": True, "readback_probe_reference": reference}}}, indent=2)+"\n",
    }
    launcher = '''"""Explicit maintenance entry point. Merely preparing this file does not run it."""
from hashlib import sha256
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
manifest = json.loads((root/"manifest.json").read_text())
for relative, expected in manifest["file_sha256"].items():
    path = root/relative
    if path.is_symlink() or sha256(path.read_bytes()).hexdigest() != expected:
        raise RuntimeError("staged file hash mismatch: "+relative)
sys.path.insert(0, str(root))
sys.argv = [__file__, "--ros-args", "--params-file", str(root/"maintenance.json")]
from ros_robot_controller import ros_robot_controller_sdk as sdk
from ros_robot_controller import ros_robot_controller_node as node
for module in (sdk, node):
    if not Path(module.__file__).resolve().is_relative_to(root):
        raise RuntimeError("unexpected vendor module source")
node.main()
'''
    files["run_maintenance.py"] = launcher
    for relative, text in files.items():
        (output/relative).write_text(text)
    manifest = {
        "schema_version": 1, "source_sha": revision,
        "vendor_sdk_sha256": SDK_SHA, "vendor_node_sha256": NODE_SHA,
        "default_gate": "DISARMED", "diagnostic_id": 1 if reference else None,
        "reference": reference, "continuous_readback_supported": False,
        "hardware_validated": False, "prepared_only": True,
        "file_sha256": {name: sha256((output/name).read_bytes()).hexdigest() for name in files},
        "approval_required": ["stop original driver", "reopen serial/start overlay",
                              "first ID1 RPC"],
        "rollback": "stop overlay only; original files unchanged; do not auto-restart original startup",
    }
    (output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vendor-directory", type=Path, required=True)
    parser.add_argument("--reference", default="")
    args = parser.parse_args()
    repository = HERE.parents[1]
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=repository, text=True).strip()
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=repository, text=True).strip():
        raise RuntimeError("commit and review sources before generating a deployment package")
    manifest = prepare(args.output, args.vendor_directory, revision, args.reference)
    print(json.dumps({"prepared_only": True, "output": str(args.output),
                      "source_sha": manifest["source_sha"]}))


if __name__ == "__main__":
    main()
