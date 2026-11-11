#!/usr/bin/env python3
"""User-run G1-A checks; no ROS node, controller, TF launch or sampling."""

import argparse
import importlib.metadata
import json
from pathlib import Path
import platform
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

CASES = {
    "Zero": [0.0, 0.0, 0.0, 0.0, 0.0],
    "Nonzero": [0.35, -0.60, 0.80, -0.40, 0.20],
    "Near-limit": [2.08, -0.40, 0.50, -0.30, 0.25],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/workspace.yaml")
    parser.add_argument("--model", type=Path, help="Override model path (.urdf or .xacro)")
    parser.add_argument("--reference", action="store_true", help="Compare all 3 poses with test-only NumPy/SciPy FK")
    parser.add_argument("--output", type=Path, help="Also save JSON; refuses to overwrite an existing file")
    args = parser.parse_args()
    try:
        import numpy as np
        import PyKDL
        from workspace_analysis.robot.model import JOINT_ORDER, load_model
        from workspace_analysis.robot.fk import ForwardKinematics

        model = load_model(args.config, model_path=args.model)
        if model.joint_order != JOINT_ORDER:
            raise ValueError("check_fk.py's three approved poses require joint1..joint5 in order")
        fk = ForwardKinematics(model)
        versions = {"python": platform.python_version(), "python_executable": sys.executable,
                    "PyKDL": getattr(PyKDL, "__version__", "unknown")}
        for package in ["numpy", "scipy", "urdfdom_py", "xacro", "PyYAML"]:
            try:
                versions[package] = importlib.metadata.version(package)
            except importlib.metadata.PackageNotFoundError:
                versions[package] = "not found in this interpreter"
        if args.reference:
            sys.path.insert(0, str(ROOT / "tests"))
            from reference_fk import pose_errors, reference_fk

        records = []
        for name, q in CASES.items():
            pose = fk.pose(q)
            matrix, rotation = pose.transform, pose.rotation
            valid = (matrix.shape == (4, 4) and np.isfinite(matrix).all()
                     and np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
                     and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0)
                     and abs(np.linalg.det(rotation) - 1) <= 1e-12)
            if not valid:
                raise RuntimeError(f"{name}: invalid homogeneous transform/rotation")
            if not np.array_equal(matrix, fk(q)):
                raise RuntimeError(f"{name}: repeated FK is not deterministic")
            row = {"name": name, "q_rad": q, "position_m": pose.position.tolist(),
                   "rotation": rotation.tolist(), "T_base_to_ee": matrix.tolist(),
                   "valid_transform": True, "deterministic": True}
            if args.reference:
                expected = reference_fk(model.urdf_xml, q, base_frame=model.base_frame,
                                        ee_frame=model.ee_frame, joint_order=model.joint_order)
                position_error, rotation_error = pose_errors(matrix, expected)
                if position_error > 1e-5 or rotation_error > 1e-4:
                    raise RuntimeError(f"{name}: reference mismatch: {position_error} m, {rotation_error} rad")
                row.update(reference_T=expected.tolist(), position_error_m=position_error,
                           rotation_error_rad=rotation_error, reference_agrees=True)
            records.append(row)
        report = {"stage": "G1-A", "status": "SOFTWARE_CHECKS_PASSED",
                  "stage_acceptance": "pending_user_review", "tf_checked": False,
                  "hardware_checked": False, "reference_checked": args.reference,
                  "thresholds": {"position_m": 1e-5, "rotation_rad": 1e-4},
                  "model": model.metadata(), "versions": versions, "poses": records}
        text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("x", encoding="utf-8") as stream:
                stream.write(text)
        sys.stdout.write(text)
        return 0
    except Exception as exc:
        print(f"G1-A check failed: {type(exc).__name__}: {exc}\n"
              "Use /usr/bin/python3 and source ROS Humble + jetrover_sim_ws overlay. "
              "No dependencies are installed automatically.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
