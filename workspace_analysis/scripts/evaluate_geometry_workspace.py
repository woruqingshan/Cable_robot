#!/usr/bin/env python3
"""D1 official/refined geometry comparison with fixed ACM v1 and original G2 q."""
import argparse,json,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',type=Path,required=True);p.add_argument('--input-prefix',type=Path,required=True);p.add_argument('--output-dir',type=Path);p.add_argument('--gripper',type=float,default=.3);p.add_argument('--batch-size',type=int,default=64);p.add_argument('--skip-robot-only',action='store_true',help='IDC comparison still includes all 38 obstacles')
    a=p.parse_args()
    from workspace_analysis.experiments.geometry_workspace import run_comparison
    out=a.output_dir or ROOT/'results/constrained_workspace'/('d1_'+uuid.uuid4().hex)
    s=run_comparison(ROOT,a.backend,a.input_prefix,out,gripper=a.gripper,batch_size=a.batch_size,include_robot_only=not a.skip_robot_only)
    print(json.dumps({'output':str(out),'summary':{k:{n:v.get('model_scope_collision_free') for n,v in value.items() if n!='paired'} for k,value in s.items()}}))
if __name__=='__main__':main()
