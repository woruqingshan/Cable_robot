#!/usr/bin/env python3
"""T5.3-C read-only Home audit and reference S0 -> saved Ready motion validation."""
import argparse
from pathlib import Path
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-run',type=Path,required=True);p.add_argument('--expected-source-manifest-sha256',required=True)
    p.add_argument('--backend',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--system-source',type=Path,required=True);p.add_argument('--allow-candidate-baseline',action='store_true')
    a=p.parse_args()
    from workspace_analysis.experiments.preparation_benchmark import run_preparation
    run_preparation(ROOT,a.source_run,a.backend,a.output_dir,expected_source_manifest_sha256=a.expected_source_manifest_sha256,
        system_source=a.system_source,allow_candidate=a.allow_candidate_baseline)


if __name__=='__main__':main()
