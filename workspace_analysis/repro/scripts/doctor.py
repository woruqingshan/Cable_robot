#!/usr/bin/env python3
"""Read-only default; --json-output explicitly creates one new JSON report."""
import sys
sys.dont_write_bytecode=True
import argparse
import json
import os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src')]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=['core','full'],default='core');p.add_argument('--backend',type=Path,default=os.environ.get('C1_BACKEND'))
    p.add_argument('--model-path',type=Path);p.add_argument('--assets-file',type=Path);p.add_argument('--json',action='store_true');p.add_argument('--json-output',type=Path)
    a=p.parse_args()
    from repro.doctor import run_doctor
    report=run_doctor(ROOT,a.mode,backend=a.backend,model_path=a.model_path,assets_file=a.assets_file)
    if a.json_output:
        try:
            with a.json_output.open('x') as f:json.dump(report,f,indent=2)
        except OSError as exc:p.error('refusing report overwrite or inaccessible output: '+str(exc))
    if a.json:print(json.dumps(report,indent=2))
    else:
        print('R2 Environment Doctor')
        for row in report['checks']:print(f"[{row['status']}] {row['id']}: {row['summary']}")
        print('Status: '+report['status'])
    return report['exit_code']


if __name__=='__main__':raise SystemExit(main())
