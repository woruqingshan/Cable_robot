#!/usr/bin/env python3
"""User-run C1 single-state or existing 100-row offline collision diagnostic."""
import argparse
import json
from pathlib import Path
import sys
from datetime import datetime, timezone
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'config/workspace.yaml')
    parser.add_argument('--scene',type=Path,default=ROOT/'config/scenes/idc_prototype_v0.yaml')
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--timeout',type=float,default=15)
    parser.add_argument('--gripper',type=float,default=0)
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--q',type=float,nargs=5)
    group.add_argument('--input-prefix',type=Path)
    args=parser.parse_args()
    run_id=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
    output=args.output_dir or ROOT/'results/c1/collision_probes'/run_id
    try:
        from workspace_analysis.collision.diagnostics import run_diagnostic
        prefix=args.input_prefix if args.q is not None else args.input_prefix or ROOT/'results/workspace_n100_seed42'
        summary=run_diagnostic(args.config,args.scene,args.backend,output,input_prefix=prefix,q=args.q,
                               gripper=args.gripper,timeout=args.timeout)
        print(json.dumps({'output_dir':str(output),**summary},indent=2))
        return 0 if summary['status']=='PASS' else 1
    except Exception as exc:
        print(f'C1 ERROR: {type(exc).__name__}: {exc}',file=sys.stderr);return 1


if __name__=='__main__':raise SystemExit(main())
