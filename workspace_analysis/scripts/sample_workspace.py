#!/usr/bin/env python3
"""G2 CPU joint sampling. No ROS node, control, collision test or visualization."""

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/workspace.yaml")
    parser.add_argument("--samples", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output-prefix", type=Path)
    args = parser.parse_args()
    try:
        from workspace_analysis.model import JOINT_ORDER, load_model
        from workspace_analysis.sampling import sample_workspace, validate_integer
        from workspace_analysis.results import (check_output_available, file_sha256, load_results, save_results)
        import numpy as np

        validate_integer(args.samples, "samples", minimum=1)
        validate_integer(args.seed, "seed", minimum=0)
        prefix = args.output_prefix or ROOT / "results" / f"workspace_n{args.samples}_seed{args.seed}"
        paths = check_output_available(prefix)
        config = args.config.expanduser().resolve()
        config_hash = file_sha256(config)
        model = load_model(config)
        if config_hash != file_sha256(config):
            raise ValueError("Configuration changed while the model was loading")
        if (model.base_frame != "base_link" or model.ee_frame != "end_effector_link"
                or model.joint_order != JOINT_ORDER):
            raise ValueError("G2 CLI requires base_link -> end_effector_link and joint1..joint5")
        print(f"Model: {model.source_path}", flush=True)
        print(f"Base={model.base_frame}, EE={model.ee_frame}, joint_order={list(model.joint_order)}", flush=True)
        print(f"N={args.samples}, seed={args.seed}; independent uniform joint sampling, CPU", flush=True)
        def progress(completed, total):
            if completed == total or completed % 1000 == 0:
                print(f"FK completed: {completed}/{total}", flush=True)
        run = sample_workspace(model, args.samples, args.seed, progress=progress)
        metadata = save_results(prefix, run, model, config_path=config, config_sha256=config_hash)
        arrays, reloaded = load_results(prefix, model=model)
        if any(not np.array_equal(value, arrays[key]) for key, value in run.arrays().items()) or reloaded != metadata:
            raise ValueError("Final pair readback differs from the generated dataset")
        stats = metadata["workspace_statistics"]
        for index, axis in enumerate("XYZ"):
            print(f"{axis} range [m]: {stats['min_m'][index]:.12g} .. {stats['max_m'][index]:.12g}")
        print(f"FK loop elapsed [s]: {run.fk_elapsed_s:.6f}")
        print(f"NPZ: {paths[0]}\nJSON: {paths[1]}")
        print("status: PASS; stage_acceptance: pending_user_review; hardware_checked: false")
        return 0
    except KeyboardInterrupt:
        print("status: ERROR; interrupted; no stage acceptance", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"status: ERROR; {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
