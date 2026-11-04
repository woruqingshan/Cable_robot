#!/usr/bin/env python3
"""T1/T2 P22 synthetic-TCP, bounded multi-start IK and semantic collision checks."""
import argparse,json,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path)
    p.add_argument('--allow-candidate-baseline',action='store_true')
    p.add_argument('--task-config',type=Path,default=ROOT/'config/tasks/p22_research_v0.yaml')
    p.add_argument('--seed-prefix',type=Path,default=ROOT/'results/g4_run01/experiments/workspace_n1000_seed42')
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--verify-run',type=Path);a=p.parse_args()
    from workspace_analysis.experiments.p22_reachability import run_p22,verify_p22
    if a.verify_run:
        from workspace_analysis.robot.model import load_model
        print(json.dumps(verify_p22(a.verify_run,load_model(ROOT/'config/workspace.yaml'))));return
    if a.backend is None or not a.allow_candidate_baseline:p.error('--backend and --allow-candidate-baseline required for RESEARCH_ONLY experiment')
    out=a.output_dir or ROOT/'results/task_reachability'/('p22_'+uuid.uuid4().hex[:10])
    s=run_p22(ROOT,a.backend,a.task_config,a.seed_prefix,out,allow_candidate=True)
    print(json.dumps({'output_dir':str(out),'final_status':s['final_status'],'unique_task_pose_solutions':s['unique_task_pose_solutions'],'collision_comparison':s['collision_comparison']}))
if __name__=='__main__':main()
