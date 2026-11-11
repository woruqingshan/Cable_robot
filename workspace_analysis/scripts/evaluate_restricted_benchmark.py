#!/usr/bin/env python3
"""T5.3-A fixed saved-goal restricted benchmark, no multi-start IK."""
import argparse
from pathlib import Path
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run',type=Path,required=True)
    parser.add_argument('--backend',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--allow-candidate-baseline',action='store_true')
    parser.add_argument('--expected-source-manifest-sha256',required=True)
    args=parser.parse_args()
    from workspace_analysis.experiments.restricted_benchmark import run_restricted
    run_restricted(ROOT,args.source_run,args.backend,args.output_dir,allow_candidate=args.allow_candidate_baseline,
        expected_source_manifest_sha256=args.expected_source_manifest_sha256)


if __name__=='__main__':
    main()
