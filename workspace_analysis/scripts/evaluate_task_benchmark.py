#!/usr/bin/env python3
"""T5.1 generic RESEARCH_ONLY task benchmark; all outputs are new-only."""
import argparse
import json
from pathlib import Path
import sys
import uuid
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend', type=Path)
    p.add_argument('--allow-candidate-baseline', action='store_true')
    p.add_argument('--scene', type=Path, default=ROOT/'config/scenes/idc_prototype_v0.yaml')
    p.add_argument('--task-template', type=Path, default=ROOT/'config/tasks/p22_research_v0.yaml')
    p.add_argument('--seed-prefix', type=Path)
    p.add_argument('--ports', nargs='+', default=['P11','P12','P13','P21','P22','P23','P31','P32','P33'])
    p.add_argument('--viewer-ports', nargs='*', default=['P22'])
    p.add_argument('--output-dir', type=Path)
    p.add_argument('--verify-run', type=Path)
    p.add_argument('--max-time-s', type=float, default=20.)
    p.add_argument('--max-iterations', type=int, default=1500)
    p.add_argument('--random-seed', type=int, help='Explicitly derive task/IK/path/approach seed and task identity')
    p.add_argument('--summary-viewer', action='store_true')
    args = p.parse_args()
    from workspace_analysis.experiments.task_benchmark_validation import verify_benchmark
    from workspace_analysis.robot.model import load_model
    if args.verify_run:
        print(json.dumps(verify_benchmark(args.verify_run, load_model(ROOT/'config/workspace.yaml')))); return
    if args.backend is None or args.seed_prefix is None or not args.allow_candidate_baseline:
        p.error('--backend, --seed-prefix and --allow-candidate-baseline required')
    from workspace_analysis.experiments.task_benchmark import run_benchmark
    from workspace_analysis.experiments.assembly_semantics import _write, _hash
    out = args.output_dir or ROOT/'results/task_benchmark'/('t5_1_s1_'+uuid.uuid4().hex[:10])
    summary = run_benchmark(ROOT, args.backend, args.scene, args.task_template, args.seed_prefix, out,
        port_ids=args.ports, allow_candidate=True, viewer_ports=tuple(args.viewer_ports),
        random_seed=args.random_seed,
        summary_viewer=args.summary_viewer,
        planning={'max_time_s': args.max_time_s, 'max_iterations': args.max_iterations})
    report = verify_benchmark(out, load_model(ROOT/'config/workspace.yaml'))
    _write(out/'independent_validation.json', report)
    _write(out/'validation_manifest.json', {'manifest_sha256': _hash(out/'manifest.json'),
        'report_sha256': _hash(out/'independent_validation.json')})
    print(json.dumps({'output_dir': str(out), 'successful_ports': summary['successful_ports'], 'verification': report}))
    if any(r['status'] in ('ERROR', 'BLOCKED') for r in summary['rows']): return 3
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except FileNotFoundError as exc:
        print(json.dumps({'status':'BLOCKED','reason':'MISSING_INPUT_OR_DEPENDENCY','error':str(exc)}),file=sys.stderr)
        sys.exit(3)
    except Exception as exc:
        print(json.dumps({'status':'ERROR','error_type':type(exc).__name__,'error':str(exc)}),file=sys.stderr)
        sys.exit(2)
