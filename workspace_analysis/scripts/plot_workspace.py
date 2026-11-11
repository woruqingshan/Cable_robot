#!/usr/bin/env python3
"""Read an existing G2 pair and export two offline G3 HTML views."""

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/workspace.yaml")
    parser.add_argument("--input-prefix", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path)
    parser.add_argument("--max-points", type=int, help="Optional deterministic display limit; default shows every row")
    args = parser.parse_args()
    try:
        from workspace_analysis.viz.workspace import check_html_available, export_workspace, load_workspace
        output = args.output_prefix or args.input_prefix
        check_html_available(output)
        view = load_workspace(args.input_prefix, args.config, max_points=args.max_points)
        model = view.metadata["robot_model"]
        print(f"Input: {view.source_prefix}", flush=True)
        print(f"Model: {view.model_name}; Base={model['base_frame']}; EE={model['ee_frame']}", flush=True)
        print(f"Samples: {len(view.position)} displayed / {view.total_samples} total; seed={view.metadata['experiment']['seed']}", flush=True)
        for index, axis in enumerate("XYZ"):
            print(f"{axis} sampled range [m]: {view.full_min[index]:.12g} .. {view.full_max[index]:.12g}", flush=True)
        paths = export_workspace(view, output)
        for path in paths:
            print(f"HTML: {path}")
        print("status: PASS; stage_acceptance: pending_user_review; hardware_checked: false")
        return 0
    except KeyboardInterrupt:
        print("status: ERROR; visualization interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"status: ERROR; {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
