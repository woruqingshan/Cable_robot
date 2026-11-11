#!/usr/bin/env python3
"""Explicit historical P22 equivalence without rerunning IK/FCL or modifying history."""
import argparse
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['benchmark','ik-run','path-run','approach-run']:
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--json-output',type=Path)
    args=parser.parse_args()
    from workspace_analysis.experiments.task_benchmark_regression import compare_p22
    from workspace_analysis.robot.model import load_model
    report=compare_p22(args.benchmark,args.ik_run,args.path_run,args.approach_run,load_model(ROOT/'config/workspace.yaml'))
    if args.json_output:
        with args.json_output.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps(report));return 0 if report['status']=='PASS' else 3


if __name__=='__main__':sys.exit(main())
