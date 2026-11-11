#!/usr/bin/env python3
"""C3 SRDF audit, reference positives, diagnostic workspace comparisons and viewers."""
import argparse,json,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path);p.add_argument('--output-dir',type=Path)
    p.add_argument('--input-prefix',action='append',type=Path,default=[])
    p.add_argument('--gripper',type=float,default=.3);p.add_argument('--candidate-trials',type=int,default=0)
    p.add_argument('--verify-run',type=Path);a=p.parse_args()
    from workspace_analysis.experiments.semantic_workspace import run_semantic_study,verify_run
    if a.verify_run:
        from workspace_analysis.robot.model import load_model
        print(json.dumps(verify_run(a.verify_run,load_model(ROOT/'config/workspace.yaml'))));return
    if a.backend is None:p.error('--backend required for generation')
    out=a.output_dir or ROOT/'results/c3'/('semantic_'+uuid.uuid4().hex[:10])
    manifest=run_semantic_study(ROOT,a.backend,out,a.input_prefix,gripper=a.gripper,candidate_trials=a.candidate_trials)
    print(json.dumps({'output_dir':str(out),'status':manifest['completion_status'],'normal_reference_status':manifest['normal_reference_status']}))
if __name__=='__main__':main()
