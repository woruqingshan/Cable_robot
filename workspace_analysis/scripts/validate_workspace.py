#!/usr/bin/env python3
"""G4 offline experiments. Explicit new output directory required; no ROS nodes."""

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/workspace.yaml')
    parser.add_argument('--output-dir', type=Path, required=True, help='Must not already exist')
    parser.add_argument('--reuse-prefix', action='append', type=Path, default=[], help='Existing validated G2 prefix; repeatable')
    parser.add_argument('--report-path', type=Path, help='Optional new Markdown path, default inside output directory')
    parser.add_argument('--sample-counts', nargs='+', type=int, default=[100, 1000, 10000])
    parser.add_argument('--seeds', nargs='+', type=int, default=[42, 43, 44])
    parser.add_argument('--repro-samples', type=int, default=1000)
    parser.add_argument('--repro-seed', type=int, default=42)
    parser.add_argument('--voxel-size', type=float, default=0.02, help='Analysis resolution in metres, not robot accuracy')
    parser.add_argument('--grid-origin', nargs=3, type=float, default=[-0.6, -0.6, -0.4], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--grid-upper', nargs=3, type=float, default=[0.6, 0.6, 0.8], metavar=('X', 'Y', 'Z'))
    parser.add_argument('--g1a-report', type=Path, default=ROOT / 'results/g1a_fk.json')
    parser.add_argument('--g1b-report', type=Path, default=ROOT / 'results/g1b_tf_v3.json')
    parser.add_argument('--visualization-prefix', type=Path, default=ROOT / 'results/workspace_n10000_seed42',
                        help='Inventory existing G3 HTML; does not regenerate or revalidate visualization')
    args = parser.parse_args()
    try:
        from workspace_analysis.experiments.workspace_validation import VoxelGrid, run_validation
        report, paths = run_validation(
            args.config, args.output_dir, reuse_prefixes=args.reuse_prefix, sample_counts=args.sample_counts,
            seeds=args.seeds, repro_samples=args.repro_samples, repro_seed=args.repro_seed,
            grid=VoxelGrid(tuple(args.grid_origin), tuple(args.grid_upper), args.voxel_size),
            report_path=args.report_path, g1a_report=args.g1a_report, g1b_report=args.g1b_report,
            visualization_prefix=args.visualization_prefix)
        for name, path in paths.items():
            print(f'{name}: {path}')
        print(f"profile: {report['experiment_profile']}; reproducibility: {report['reproducibility']['status']}")
        print(f"status: {report['status']}; stage_acceptance: pending_user_review; hardware_checked: false")
        return 0 if report['status'] == 'PASS' else 1
    except KeyboardInterrupt:
        print('status: ERROR; interrupted; no stage acceptance', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'status: ERROR; {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
