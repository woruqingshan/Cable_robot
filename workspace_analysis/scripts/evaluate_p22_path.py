#!/usr/bin/env python3
"""T3: research-only P22 joint-space linear / RRT-Connect discrete paths."""
import argparse
import json
from pathlib import Path
import sys
import uuid
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend', type=Path)
    p.add_argument('--allow-candidate-baseline', action='store_true')
    p.add_argument('--ik-run', type=Path, default=ROOT/'results/task_reachability/p22_20261104_02')
    p.add_argument('--seed-prefix', type=Path, default=ROOT/'results/g4_run01/experiments/workspace_n1000_seed42')
    p.add_argument('--output-dir', type=Path)
    p.add_argument('--verify-run', type=Path)
    p.add_argument('--edge-resolution-rad', type=float, default=.02)
    p.add_argument('--verification-resolution-rad', type=float, default=.01)
    p.add_argument('--step-rad', type=float, default=.25)
    p.add_argument('--random-seed', type=int, default=20261104)
    p.add_argument('--max-iterations', type=int, default=1500)
    p.add_argument('--max-time-s', type=float, default=20.)
    p.add_argument('--goal-bias', type=float, default=.05)
    p.add_argument('--rrt-mode', choices=['on-failure', 'always'], default='on-failure', help='always also measures RRT when linear passes')
    p.add_argument('--self-clearance-m', type=float, default=0.)
    p.add_argument('--environment-clearance-m', type=float, default=0.)
    a = p.parse_args()
    from workspace_analysis.experiments.p22_path_planning import run_p22_path, verify_p22_path, settings_checked
    from workspace_analysis.planning.joint_path import PathError
    if a.verify_run:
        from workspace_analysis.robot.model import load_model
        print(json.dumps(verify_p22_path(a.verify_run, load_model(ROOT/'config/workspace.yaml')))); return
    if a.backend is None or not a.allow_candidate_baseline: p.error('--backend and --allow-candidate-baseline required')
    keys = ['edge_resolution_rad', 'verification_resolution_rad', 'step_rad', 'random_seed', 'max_iterations', 'max_time_s', 'goal_bias', 'rrt_mode', 'self_clearance_m', 'environment_clearance_m']
    try: settings = settings_checked({k: getattr(a, k) for k in keys})
    except PathError as exc: p.error(str(exc))
    output = a.output_dir or ROOT/'results/task_planning'/('p22_'+uuid.uuid4().hex[:10])
    summary = run_p22_path(ROOT, a.backend, a.ik_run, a.seed_prefix, output, allow_candidate=True, settings=settings)
    print(json.dumps({'output_dir': str(output), 'status': summary['status'], 'start_sample_index': summary['start_sample_index'],
                      'comparison': summary['comparison'], 'usage': 'RESEARCH_ONLY'}))


if __name__ == '__main__': main()
