#!/usr/bin/env python3
"""Run a model-only constrained workspace pilot over an explicit existing G2 pair."""
import argparse,json,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',type=Path,required=True)
    p.add_argument('--input-prefix',type=Path,required=True);p.add_argument('--output-dir',type=Path)
    p.add_argument('--gripper',type=float,default=.3);p.add_argument('--batch-size',type=int,default=64)
    p.add_argument('--environment-clearance',type=float,default=0);p.add_argument('--self-clearance',type=float,default=0)
    a=p.parse_args()
    from workspace_analysis.experiments.constrained_workspace import run_pilot
    output=a.output_dir or ROOT/'results/constrained_workspace'/uuid.uuid4().hex
    result=run_pilot(ROOT,a.backend,a.input_prefix,output,gripper=a.gripper,batch_size=a.batch_size,clearance={'environment_m':a.environment_clearance,'self_m':a.self_clearance})
    print(json.dumps({'output':str(output),'summaries':result},indent=2));return 1 if any(x['decisions']['ERROR'] for x in result.values()) else 0

if __name__=='__main__':raise SystemExit(main())
