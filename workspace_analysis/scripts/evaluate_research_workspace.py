#!/usr/bin/env python3
"""RESEARCH_ONLY strict/candidate workspace evaluation; no new joint sampling."""
import argparse,json,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path)
    p.add_argument('--candidate-srdf',type=Path,default=ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf')
    p.add_argument('--allow-candidate-baseline',action='store_true')
    p.add_argument('--input-prefix',type=Path,action='append',default=[])
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--system-source',type=Path,default=ROOT.parents[1]/'system_source')
    p.add_argument('--gripper',type=float,default=.3)
    p.add_argument('--self-clearance',type=float,default=0.)
    p.add_argument('--environment-clearance',type=float,default=0.)
    p.add_argument('--batch-size',type=int,default=64)
    p.add_argument('--verify-run',type=Path);a=p.parse_args()
    from workspace_analysis.experiments.research_workspace import run_research_workspace,verify_research_workspace
    if a.verify_run:
        from workspace_analysis.robot.model import load_model
        print(json.dumps(verify_research_workspace(a.verify_run,load_model(ROOT/'config/workspace.yaml'))));return
    if a.backend is None or not a.allow_candidate_baseline:p.error('--backend and --allow-candidate-baseline required for RESEARCH_ONLY evaluation')
    prefixes=a.input_prefix or [ROOT/'results/workspace_n100_seed42',ROOT/'results/g4_run01/experiments/workspace_n1000_seed42']
    out=a.output_dir or ROOT/'results/constrained_workspace'/('research_'+uuid.uuid4().hex[:10])
    studies=run_research_workspace(ROOT,a.backend,out,prefixes,source=a.system_source,candidate_srdf=a.candidate_srdf,
        allow_candidate=True,gripper=a.gripper,clearance={'self_m':a.self_clearance,'environment_m':a.environment_clearance},batch_size=a.batch_size)
    print(json.dumps({'output_dir':str(out),'usage':'RESEARCH_ONLY','datasets':list(studies)}))
if __name__=='__main__':main()
