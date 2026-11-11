#!/usr/bin/env python3
"""User-run G1-B ROS2 TF validation. Never starts drivers or robot control."""

import argparse
from datetime import datetime, timezone
from hashlib import sha256
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import signal
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/workspace.yaml")
    parser.add_argument("--model", type=Path, help="Optional .urdf/.xacro override")
    parser.add_argument("--output", type=Path, default=ROOT / "results/g1b_tf_v3.json")
    parser.add_argument("--domain-id", type=int, help="Unused local test domain in 0..101; default 81 (82 if caller uses 81)")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds per discovery/parameter/TF wait (default 10)")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    report = {"stage": "G1-B", "report_version": 3, "status": "RUNNING", "hardware_checked": False,
              "stage_acceptance": "pending_user_review", "tf_checked": False,
              "endpoint_identity_policy": "strict_wait_no_fallback",
              "run_id": uuid.uuid4().hex[:16], "cases": [],
              "started_utc": datetime.now(timezone.utc).isoformat(),
              "environment": {"python": platform.python_version(), "executable": sys.executable,
                              **{key: os.environ.get(key) for key in
                                 ("ROS_DISTRO", "RMW_IMPLEMENTATION", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}}}
    # Reserve report before starting ROS; never truncate an existing report.
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        stream = args.output.open("x", encoding="utf-8")
    except OSError as exc:
        print(f"Cannot create report (no ROS nodes started): {exc}", file=sys.stderr)
        return 1
    interrupted = False
    previous_term = signal.getsignal(signal.SIGTERM)

    def terminate(signum, frame):
        raise KeyboardInterrupt(f"received signal {signum}")

    signal.signal(signal.SIGTERM, terminate)
    try:
        json.dump(report, stream, indent=2)
        stream.flush()
        from workspace_analysis.robot.model import JOINT_ORDER, load_model
        from workspace_analysis.robot.fk import ForwardKinematics
        from workspace_analysis.experiments.tf_validation import (
            CASES, POSITION_TOLERANCE, ROTATION_TOLERANCE, TFValidationError,
            choose_domain, finalize_report, run_case,
        )
        domain = choose_domain(args.domain_id, os.environ.get("ROS_DOMAIN_ID"))
        model = load_model(args.config, model_path=args.model)  # exactly once
        if (model.base_frame != "base_link" or model.ee_frame != "end_effector_link"
                or model.joint_order != JOINT_ORDER):
            raise TFValidationError("G1-B requires base_link -> end_effector_link and joint1..joint5")
        for _, q in CASES:
            model.joint_positions(q)  # validate all inputs before any ROS child
        fk = ForwardKinematics(model)
        report.update(model=model.metadata(), config_path=str(args.config.expanduser().resolve()),
                      config_sha256=sha256(args.config.expanduser().read_bytes()).hexdigest(),
                      domain_id=domain, effective_ros_localhost_only=True, timeout_per_wait_s=args.timeout,
                      synchronization_strategy="fresh_rsp_and_filtered_buffer_per_case_exact_stamp_and_endpoint_graph",
                      thresholds={"position_m": POSITION_TOLERANCE, "rotation_rad": ROTATION_TOLERANCE})
        report["versions"] = {}
        for package in ("rclpy", "tf2_ros_py", "numpy", "scipy", "xacro", "urdfdom_py"):
            try:
                report["versions"][package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                report["versions"][package] = "unknown"
        print(f"G1-B domain={domain}; localhost only; private topics; output={args.output}", flush=True)
        for index, case in enumerate(CASES, 1):
            name, q = case
            row = {"name": name, "q_rad": list(q), "passed": False, "status": "RUNNING"}
            report["cases"].append(row)
            namespace = f"/g1b_{report['run_id']}/case_{index}"
            print(f"[{name}] waiting for a fresh RSP/TF context...", flush=True)
            run_case(model, fk, case, row, domain_id=domain, timeout=args.timeout, namespace=namespace)
            print(json.dumps(row, indent=2, ensure_ascii=False, allow_nan=False), flush=True)
            if not row["passed"]:
                raise TFValidationError(f"{name}: comparison, identity, RSP exit or cleanup failed; see case report")
        finalize_report(report)
    except BaseException as exc:
        interrupted = isinstance(exc, KeyboardInterrupt)
        report.update(error=f"{type(exc).__name__}: {exc}", status="FAIL")
        report["error_code"] = getattr(exc, "code", type(exc).__name__)
        if report["cases"] and report["cases"][-1].get("status") == "RUNNING":
            report["cases"][-1].update(status="ERROR", error=report["error"])
        print(f"G1-B failed: {report['error']}", file=sys.stderr)
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        # Also enforce failure when initialization/imports did not reach helpers.
        report["stage_acceptance"] = "pending_user_review"
        report["hardware_checked"] = False
        rows = report["cases"]
        report["strict_endpoint_identity_verified"] = (len(rows) == 3 and all(
            r.get("strict_endpoint_identity_verified", False) for r in rows))
        report["isolated_topic_source_checked"] = (len(rows) == 3 and all(
            r.get("isolated_topic_source_checked", False) for r in rows))
        if report.get("error"):
            report["status"] = "FAIL"
        try:
            stream.seek(0)
            stream.truncate()
            json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
        except Exception as exc:
            report["status"] = "FAIL"
            print(f"Report write failed: {exc}", file=sys.stderr)
        finally:
            stream.close()
    print(f"{report['status']}; stage_acceptance=pending_user_review; report={args.output}", flush=True)
    return 130 if interrupted else (0 if report["status"] == "PASS" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
