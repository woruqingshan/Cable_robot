#!/usr/bin/env python3
"""Plot only hash-bound recorded Pilot rows; no IK/FCL queries or resampling."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--expected-manifest-sha256',required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=a.run.resolve()
    raw=(run/'manifest.json').read_bytes()
    if sha256(raw).hexdigest()!=a.expected_manifest_sha256:raise ValueError('manifest trust anchor differs')
    manifest=json.loads(raw)
    for name,h in manifest['files'].items():
        path=(run/name).resolve()
        if not path.is_relative_to(run) or sha256(path.read_bytes()).hexdigest()!=h:raise ValueError('artifact hash/path differs: '+name)
    from workspace_analysis.viz.base_pose import export_base_pose_summary
    export_base_pose_summary(json.loads((run/'summary.json').read_text()),a.output)


if __name__=='__main__':main()
