"""Prepare a hash-pinned button-only review package. Never installs or restarts."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import re
import subprocess

from patches.jetrover_readback import wifi_button_adapter as adapter

UNIT_PATH = "/etc/systemd/system/button_scan.service"
UNIT_SHA256 = "45972d2cc0f5b995cb7232f9aae8d464c732b62581e70389b8a779a4d8dca635"
DROPIN_PATH = "/etc/systemd/system/button_scan.service.d/90-idc-serial-owner.conf"


def _hash(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def prepare_stage(output, *, backup_manifest, backup_sha256, revision,
                  button=adapter.BUTTON_PATH, unit=UNIT_PATH):
    if not re.fullmatch(r"[0-9a-f]{40}", revision):
        raise ValueError("EXACT_COMMIT_REQUIRED")
    output = Path(output).absolute()
    if output.exists() or not re.fullmatch(r"/[A-Za-z0-9_./-]+", str(output)):
        raise ValueError("NEW_SAFE_STAGE_PATH_REQUIRED")
    adapter.inspect_vendor(button)  # AST and hash checks only; never GPIO/ROS.
    if _hash(unit) != UNIT_SHA256:
        raise ValueError("BUTTON_UNIT_HASH_MISMATCH")
    backup_manifest = Path(backup_manifest)
    if _hash(backup_manifest) != backup_sha256:
        raise ValueError("BACKUP_MANIFEST_HASH_MISMATCH")
    backup = json.loads(backup_manifest.read_text())
    backup_root = Path(backup["backup_root"])
    if backup_root.resolve() != backup_manifest.parent.resolve() or not backup["backup_verified"]:
        raise ValueError("INVALID_BACKUP_ROOT")
    expected = {adapter.BUTTON_PATH: adapter.BUTTON_SHA256, UNIT_PATH: UNIT_SHA256}
    for name, digest in expected.items():
        record = next((r for r in backup["files"] if r["source"] == name), None)
        if record is None or record["sha256"] != digest:
            raise ValueError("REQUIRED_BACKUP_MISSING")
        relative = Path(record["backup"])
        if relative.is_absolute() or ".." in relative.parts or _hash(backup_root/relative) != digest:
            raise ValueError("BACKUP_COPY_HASH_MISMATCH")
    script_path = output/"wifi_button_adapter.py"
    if not re.fullmatch(r"/[A-Za-z0-9_./-]+", str(Path(button))):
        raise ValueError("SAFE_BUTTON_PATH_REQUIRED")
    original_start = next(line for line in Path(unit).read_text().splitlines()
                          if line.startswith("ExecStart="))
    old = "python3 /home/ubuntu/wifi_manager/button_scan.py;"
    if original_start.count(old) != 1:
        raise ValueError("BUTTON_STARTUP_CONTRACT_CHANGED")
    new = "exec /usr/bin/python3 " + str(script_path) + " --run --script " + str(button) + ";"
    # Preserve the original shell environment, restart policy and logging settings.
    override = "[Service]\nExecStart=\n" + original_start.replace(old, new, 1) + "\n"
    module_bytes = Path(adapter.__file__).read_bytes()
    output.mkdir(parents=True, exist_ok=False)
    output.chmod(0o700)
    script_path.write_bytes(module_bytes)
    override_path = output/"button_scan.override.conf"
    override_path.write_text(override)
    manifest = {
        "schema_version": 1, "source_commit": revision,
        "source_module_sha256": sha256(module_bytes).hexdigest(),
        "button_source_sha256": adapter.BUTTON_SHA256, "button_unit_sha256": UNIT_SHA256,
        "backup_manifest": str(backup_manifest), "backup_manifest_sha256": backup_sha256,
        "files": {str(script_path.name): _hash(script_path),
                  str(override_path.name): _hash(override_path)},
        "proposed_dropin": DROPIN_PATH,
        "runtime_files_modified": False, "services_restarted": [],
        "vendor_button_modified": False, "hardware_rpc_sent": 0,
        "separate_approval_required": True,
        "apply_scope": ["install only this button_scan drop-in", "systemd daemon-reload",
                        "restart button_scan.service only", "passive FD/DDS verification only"],
        "excluded_operations": ["restart start_app_node.service", "modify SDK", "bus readback",
                                "trigger buttons", "reset Wi-Fi", "halt", "motion", "torque"],
        "rollback": ["verify installed drop-in hash matches this package",
                     "remove only that drop-in", "daemon-reload",
                     "restart button_scan.service only", "verify original unit/script hashes"],
    }
    (output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    if _hash(unit) != UNIT_SHA256 or _hash(button) != adapter.BUTTON_SHA256:
        raise ValueError("LIVE_BASELINE_CHANGED_DURING_PREPARATION")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--backup-manifest", required=True)
    parser.add_argument("--backup-sha256", required=True)
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[2]
    if subprocess.check_output(["git", "status", "--porcelain"], cwd=root, text=True).strip():
        raise ValueError("COMMIT_AND_TEST_REVIEWED_WORK_FIRST")
    revision = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip()
    manifest = prepare_stage(args.output, backup_manifest=args.backup_manifest,
        backup_sha256=args.backup_sha256, revision=revision)
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
