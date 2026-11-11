#!/usr/bin/env python3
"""D2 official full geometry and task scope comparison; no new q sampling."""
import argparse,json,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',type=Path,required=True);p.add_argument('--input-prefix',type=Path,required=True);p.add_argument('--output-dir',type=Path);p.add_argument('--gripper',type=float,default=.3);p.add_argument('--batch-size',type=int,default=64);p.add_argument('--self-clearance',type=float,default=0);p.add_argument('--environment-clearance',type=float,default=0);a=p.parse_args()
    from workspace_analysis.experiments.task_scoped_workspace import run_task_pilot
    out=a.output_dir or ROOT/'results/constrained_workspace'/('d2_'+uuid.uuid4().hex)
    s=run_task_pilot(ROOT,a.backend,a.input_prefix,out,gripper=a.gripper,batch_size=a.batch_size,clearance={'self_m':a.self_clearance,'environment_m':a.environment_clearance})
    print(json.dumps({'output':str(out),'task_summary':s['B_task_scope']}))
if __name__=='__main__':main()
