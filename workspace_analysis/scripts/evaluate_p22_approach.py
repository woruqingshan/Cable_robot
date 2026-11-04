#!/usr/bin/env python3
"""T4 research-only Cartesian approach, preserving T2 starts and task tolerances."""
import argparse
import json
from pathlib import Path
import sys
import uuid
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path);p.add_argument('--allow-candidate-baseline',action='store_true')
    p.add_argument('--ik-run',type=Path,default=ROOT/'results/task_reachability/p22_20261104_02')
    p.add_argument('--path-run',type=Path,default=ROOT/'results/task_planning/p22_20261104_02')
    p.add_argument('--output-dir',type=Path);p.add_argument('--verify-run',type=Path)
    p.add_argument('--end-distance-mm',type=float,default=10.);p.add_argument('--cartesian-step-mm',type=float,default=10.)
    p.add_argument('--edge-resolution-rad',type=float,default=.01);p.add_argument('--max-joint-delta-rad',type=float,default=.35)
    p.add_argument('--backup-seeds',type=int,default=2);p.add_argument('--backup-radius-rad',type=float,default=.03)
    p.add_argument('--random-seed',type=int,default=20261104)
    p.add_argument('--self-clearance-m',type=float,default=0.);p.add_argument('--environment-clearance-m',type=float,default=0.)
    args=p.parse_args()
    from workspace_analysis.planning.cartesian_approach import approach_settings
    from workspace_analysis.planning.joint_path import PathError
    from workspace_analysis.experiments.p22_approach import run_p22_approach,verify_approach
    if args.verify_run:
        from workspace_analysis.robot.model import load_model
        print(json.dumps(verify_approach(args.verify_run,load_model(ROOT/'config/workspace.yaml'))));return
    if args.backend is None or not args.allow_candidate_baseline:p.error('--backend and --allow-candidate-baseline required')
    try:
        settings=approach_settings({'end_distance_m':args.end_distance_mm/1000,'cartesian_step_m':args.cartesian_step_mm/1000,
            **{key:getattr(args,key) for key in ['edge_resolution_rad','max_joint_delta_rad','backup_seeds','backup_radius_rad','random_seed','self_clearance_m','environment_clearance_m']}})
        if settings['end_distance_m']<.01:raise PathError('T4 free-space experiment stops at >=10 mm')
    except PathError as exc:p.error(str(exc))
    output=args.output_dir or ROOT/'results/task_approach'/('p22_'+uuid.uuid4().hex[:10])
    summary=run_p22_approach(ROOT,args.backend,args.ik_run,args.path_run,output,allow_candidate=True,settings=settings)
    print(json.dumps({'output_dir':str(output),'comparison':summary['comparison'],'usage':'RESEARCH_ONLY'}))


if __name__=='__main__':main()
