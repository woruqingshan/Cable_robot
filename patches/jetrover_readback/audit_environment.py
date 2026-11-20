"""Read-only metadata inventory. Never opens a device or imports the vendor SDK."""
import argparse
from datetime import datetime, timezone
from hashlib import sha256
import json
import os
from pathlib import Path


def serial_openers(device, proc_root=Path("/proc")):
    """Report visible open FDs, not proof of reader activity or exclusive access."""
    target = Path(device).resolve()
    owners = []
    denied = 0
    for process in proc_root.iterdir():
        if not process.name.isdigit():
            continue
        try:
            descriptors = list((process/"fd").iterdir())
        except PermissionError:
            denied += 1
            continue
        except FileNotFoundError:
            continue
        matching = []
        for fd in descriptors:
            try:
                if fd.resolve(strict=True) == target:
                    matching.append(fd.name)
            except (OSError, RuntimeError):
                continue
        if matching:
            try:
                name = (process/"comm").read_text().strip()
                # Never emit full command lines or environment variables.
                args = (process/"cmdline").read_bytes().split(b"\0")
                scripts = [a.decode(errors="replace") for a in args
                           if a.startswith(b"/") and a.endswith(b".py")]
            except (OSError, UnicodeError):
                name, scripts = "unknown", []
            owners.append({"pid": int(process.name), "process_name": name,
                           "script_paths": scripts, "fds": sorted(matching)})
    return {
        "device_path": str(device), "resolved_path": str(target),
        "visible_openers": sorted(owners, key=lambda owner: owner["pid"]),
        "inaccessible_process_fd_directories": denied,
        "ownership_assessment": "MULTIPLE_OPENERS" if len(owners) > 1 else "NOT_PROVEN_EXCLUSIVE",
        "visibility_complete": False,
        "receive_or_write_activity_proven": False,
        "safe_to_read": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/rrc")
    parser.add_argument("--vendor-directory", required=True)
    args = parser.parse_args()
    vendor = Path(args.vendor_directory)
    sources = {}
    for name in ("ros_robot_controller_sdk.py", "ros_robot_controller_node.py"):
        path = vendor/name
        sources[name] = {"path": str(path), "sha256": sha256(path.read_bytes()).hexdigest()}
    print(json.dumps({
        "schema_version": 1,
        "host_reported_time_utc": datetime.now(timezone.utc).isoformat(),
        "vendor_sources": sources,
        "serial_ownership": serial_openers(args.device),
        "hardware_rpc_requests_sent": 0,
        "device_open_calls": 0,
        "vendor_modified": False,
        "scope": "filesystem_and_proc_metadata_only",
    }, indent=2))


if __name__ == "__main__":
    main()
