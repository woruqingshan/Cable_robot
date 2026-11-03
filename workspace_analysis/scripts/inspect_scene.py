#!/usr/bin/env python3
"""B2 Scene Only or verified G2 workspace overlay; one offline HTML, no ROS nodes."""

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def main() -> int:
    """Validate inputs and publish HTML; errors return nonzero with no false PASS."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=ROOT / 'config/workspace.yaml')
    parser.add_argument('--input-prefix', type=Path)
    parser.add_argument('--max-points', type=int)
    parser.add_argument('--axis-length', type=float, default=.05, help='Frame axis length in metres')
    parser.add_argument('--cylinder-segments', type=int, default=48)
    args = parser.parse_args()
    try:
        from workspace_analysis.viz.scene import check_html_available, load_scene_figure, export_scene_html
        check_html_available(args.output)
        figure = load_scene_figure(args.scene, input_prefix=args.input_prefix, config_path=args.config,
                                  max_points=args.max_points, axis_length=args.axis_length,
                                  cylinder_segments=args.cylinder_segments)
        meta = figure.layout.meta
        print(f"Scene: {meta['scene_id']}; Base={meta['reference_frame']}; objects={meta['geometry_object_count']}", flush=True)
        print(f"Scene input: {meta['scene_path']}\nRaw YAML SHA256: {meta['raw_yaml_sha256']}\nCanonical SHA256: {meta['canonical_scene_sha256']}", flush=True)
        label = 'Synthetic Scene' if meta['scene_provenance']['kind'] == 'synthetic' else 'Scene'
        print(f'{label} / Not Hardware Verified; geometry display, no collision classification', flush=True)
        view = meta['workspace']
        if view is None:
            print('Mode: Scene Only', flush=True)
        else:
            print(f"Workspace: {view['source_prefix']}; model={view['model_name']}; EE={view['ee_frame']}", flush=True)
            print(f"Samples: {view['displayed_samples']} displayed / {view['total_samples']} total; seed={view['seed']}", flush=True)
            print(f"NPZ SHA256: {view['npz_sha256']}; selection={view['selection']}", flush=True)
        path = export_scene_html(figure, args.output)
        print(f'HTML: {path}')
        print('status: PASS; stage_acceptance: pending_user_visualization_review; hardware_checked: false; collision_checked: false')
        return 0
    except KeyboardInterrupt:
        print('status: ERROR; interrupted; no visualization acceptance', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'status: ERROR; {type(exc).__name__}: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
