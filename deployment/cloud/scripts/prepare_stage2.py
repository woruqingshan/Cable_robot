#!/usr/bin/env python3
"""Record existing assets/venv and produce an inspection-only Stage 2 deployment."""
import argparse
from datetime import datetime, timezone
from importlib.metadata import version
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "vla_protocol/src"), str(ROOT / "vla_server/src")]
from vla_server.artifacts import inventory, inventory_hash, file_hash
from vla_server.config import ServerConfig
from vla_server.deployment import DeploymentManifest
from verify_smolvla_inference import LEROBOT_REV, CHECKPOINT_REV, BACKBONE_REV, git


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", default="/root/autodl-tmp/idc-vla")
    parser.add_argument("--inference-report", required=True)
    parser.add_argument("--output-dir", required=True, help="new directory outside source/model/cache directories")
    args = parser.parse_args()
    base, output = Path(args.base).resolve(), Path(args.output_dir).resolve()
    for protected in (ROOT, base / "models", base / "cache", base / "venvs", base / "third_party"):
        if output.is_relative_to(protected) or protected.is_relative_to(output):
            parser.error("output directory must be separate from source and model/environment assets")
    if output.exists():
        parser.error("choose a new output directory; existing experiment evidence is preserved")
    report = json.loads(Path(args.inference_report).read_text())
    if not (report.get("status") == "PASS" and report.get("genuine_forward_pass_verified")
            and report.get("requested_dtype") == "checkpoint" and report.get("checkpoint_revision") == CHECKPOINT_REV
            and report.get("lerobot_revision") == LEROBOT_REV and report.get("backbone_revision") == BACKBONE_REV):
        parser.error("require passing genuine inference evidence at original checkpoint precision and exact revisions")
    if git(ROOT, "status", "--porcelain", "--untracked-files=no"):
        parser.error("commit tracked source changes before recording deployment provenance")
    import torch
    if sys.prefix == sys.base_prefix or os.environ.get("ROS_DISTRO"):
        parser.error("run in the independent audited cloud venv without ROS")
    model = base / "models/smolvla_base"
    files = inventory(model)
    observed = report["checkpoint_files"]
    if any(files.get(name) != value for name, value in observed.items()) or any(
            name not in observed for name in files if name != "checkpoint_receipt.json"):
        parser.error("model assets differ from the genuine inference evidence")
    packages = {p: version(p) for p in report["packages"]}
    if packages != report["packages"] or git(base / "third_party/lerobot", "rev-parse", "HEAD") != LEROBOT_REV:
        parser.error("environment differs from passing inference evidence")
    output.mkdir(parents=True)
    receipt = output / "checkpoint_receipt.json"
    receipt.write_text(json.dumps({"repository": "lerobot/smolvla_base", "revision": CHECKPOINT_REV,
        "files": files, "revision_evidence": str(base / "models/smolvla_base_revision.txt")}, indent=2) + "\n")
    freeze = output / "installed-freeze.txt"
    freeze.write_text(subprocess.check_output([sys.executable, "-m", "pip", "freeze", "--all"], text=True))
    revision = git(ROOT, "rev-parse", "HEAD")
    cfg = ServerConfig.model_validate({"source_revision": revision, "policy": {
        "type": "smolvla", "lerobot_checkout": str(base / "third_party/lerobot"), "lerobot_revision": LEROBOT_REV,
        "checkpoint_path": str(model), "checkpoint_revision": CHECKPOINT_REV,
        "checkpoint_inventory_sha256": inventory_hash(files), "checkpoint_receipt_path": str(receipt),
        "hf_home": str(base / "cache/huggingface"), "backbone_revision": BACKBONE_REV,
        "feature_spec": {"model_id": "smolvla_base_synthetic_inspection_v1", "processor_id": "smolvla_official_processors_v1",
            "state_channels": ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"],
            "state_units": ["rad"] * 6, "state_dimension": 6,
            "source_profiles": ["synthetic_uncalibrated_v1"],
            "cameras": [{"camera_id": "mock_rgb" if i == 1 else f"camera{i}",
                "feature_name": f"observation.images.camera{i}", "shape_chw": [3, 256, 256],
                "shape_policy": "official_processor", "required": i == 1} for i in range(1, 4)]},
        "native_spec": {"space_id": "smolvla_base_native_6d_unmapped_v1", "channels": [f"native_{i}" for i in range(6)],
            "units": ["model_native_unknown"] * 6, "semantics": "unknown", "step_s": 0.02,
            "processor_id": "smolvla_official_processors_v1", "normalization_id": "smolvla_checkpoint_mean_std_v1",
            "semantics_source": "Inspection only: native continuous output; channel units/semantics and JetRover mapping unknown. 0.02 s offsets are a declared synthetic inspection schedule, not calibrated robot timing."}},
        "limits": {"max_horizon": 64, "request_timeout_s": 30.0},
        "timing": {"inference_timeout_s": 20.0}})
    (output / "server.yaml").write_text(json.dumps(cfg.model_dump(), indent=2) + "\n")
    manifest = DeploymentManifest.model_validate({"cable_robot_revision": revision,
        "lerobot": {"revision": LEROBOT_REV, "checkout": str(base / "third_party/lerobot")},
        "checkpoint": {"revision": CHECKPOINT_REV, "directory": str(model),
            "inventory_sha256": inventory_hash(files), "receipt_path": str(receipt)},
        "environment": {"kind": "existing_venv", "requirements_format": "installed_freeze",
            "python_version": ".".join(map(str, sys.version_info[:3])), "torch_version": torch.__version__,
            "cuda_runtime": torch.version.cuda, "requirements_lock": str(freeze), "requirements_sha256": file_hash(freeze)},
        "server_config": "server.yaml"})
    manifest.require_resolved()
    (output / "manifest.yaml").write_text(json.dumps(manifest.model_dump(), indent=2) + "\n")
    (output / "provenance.json").write_text(json.dumps({"created_utc": datetime.now(timezone.utc).isoformat(),
        "source_revision": revision, "inference_report": str(Path(args.inference_report).resolve()),
        "inference_report_sha256": file_hash(args.inference_report), "packages": packages,
        "backbone_revision": BACKBONE_REV, "precision": "original mixed BF16/FP32; no conversion",
        "execution": "native inspection only; no registered robot mapping", "asset_revision_evidence": "operator-uploaded revision record plus recorded file inventory; not an independent Hub attestation"}, indent=2) + "\n")
    print(json.dumps({"status": "PREPARED", "directory": str(output), "source_revision": revision}))


if __name__ == "__main__":
    main()
