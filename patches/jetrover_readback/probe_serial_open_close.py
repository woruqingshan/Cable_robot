"""One supervised open/close verification. Default mode is inspection only."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

from patches.jetrover_readback.guarded_readback import (
    initialize_existing_serial, close_existing_serial_once)

HERE = Path(__file__).resolve().parent
REPOSITORY = HERE.parents[1]


def inventory():
    raw = subprocess.check_output([
        "sudo","-n",sys.executable,str(HERE/"audit_environment.py"),
        "--vendor-directory","/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller"],
        text=True)
    return json.loads(raw)


def cached_identity():
    base = Path("/sys/class/tty/ttyACM0/device").resolve()
    driver = str((base/"driver").resolve())
    for parent in [base,*base.parents]:
        if (parent/"idVendor").is_file() and (parent/"idProduct").is_file():
            return {"interface_path":str(base), "driver":driver,
                    "vid":(parent/"idVendor").read_text().strip(),
                    "pid":(parent/"idProduct").read_text().strip()}
    raise RuntimeError("USB_IDENTITY_UNAVAILABLE")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--execute-once",action="store_true")
    parser.add_argument("--expected-source-sha",default="")
    args = parser.parse_args(argv)
    revision = subprocess.check_output(["git","rev-parse","HEAD"],cwd=REPOSITORY,text=True).strip()
    if args.execute_once:
        if revision != args.expected_source_sha or len(revision)!=40:
            raise ValueError("PINNED_SOURCE_REQUIRED")
        if subprocess.check_output(["git","status","--porcelain"],cwd=REPOSITORY,text=True).strip():
            raise ValueError("CLEAN_REVIEWED_SOURCE_REQUIRED")
    record = dict(source_sha=revision, mode="execute_once" if args.execute_once else "inspect_only",
        physical_open_attempts=0, serial_writes=0, servo_queries=0,
        usb_reset=False, retry=False, clean_protocol_epoch_proven=False)
    with args.output.open("x") as destination:
        destination.write(json.dumps(record)+"\n");destination.flush()
        try:
            record["preflight"] = inventory()
            owners = record["preflight"]["serial_ownership"]
            if owners["visible_openers"] or owners["inaccessible_process_fd_directories"]:
                raise RuntimeError("EXCLUSIVE_COORDINATION_NOT_VERIFIED")
            if owners["resolved_path"] != "/dev/ttyACM0":
                raise RuntimeError("DEVICE_MAPPING_CHANGED")
            record["usb_identity"] = cached_identity()
            identity = record["usb_identity"]
            if (identity["vid"],identity["pid"]) != ("1a86","55d4") or not identity["driver"].endswith("/cdc_acm"):
                raise RuntimeError("USB_IDENTITY_CHANGED")
            if args.execute_once:
                import serial
                port = serial.Serial(None,1000000,timeout=5,write_timeout=.05)
                record["physical_open_attempts"] = 1
                record["open_report"] = initialize_existing_serial(port,"/dev/rrc")
                started = time.monotonic_ns()
                record["close_report"] = close_existing_serial_once(port)
                record["close_elapsed_ns"] = time.monotonic_ns()-started
                if not record["close_report"]["port_closed"]:
                    raise RuntimeError("CLOSE_NOT_CONFIRMED_NO_RETRY")
                record["status"] = "OPEN_CLOSE_PASS_NO_POSITION_QUERIES"
            else:
                record["status"] = "INSPECTED_NO_OPEN"
        except BaseException as error:
            record.update(status="STOP_NO_RETRY",exception_type=type(error).__name__,
                errno=getattr(error,"errno",None),error=str(error),
                initialization_report=getattr(error,"serial_initialization_report",None))
        finally:
            try:
                record["postflight"] = inventory()
                post = record["postflight"]["serial_ownership"]
                if post["visible_openers"] or post["inaccessible_process_fd_directories"]:
                    record["status"] = "STOP_POSTFLIGHT_NOT_CLEAR"
            except Exception as post_error:
                record["postflight_error"] = str(post_error)
                record["status"] = "STOP_POSTFLIGHT_CHECK_FAILED"
            destination.seek(0);destination.truncate()
            destination.write(json.dumps(record,indent=2)+"\n")
    print(json.dumps(record,indent=2))
    return 0 if record["status"] in ("OPEN_CLOSE_PASS_NO_POSITION_QUERIES","INSPECTED_NO_OPEN") else 2


if __name__ == "__main__":
    raise SystemExit(main())
