#!/usr/bin/env python3
"""T5.3-C standard postprocess of accepted, hash-bound RRT paths."""
import argparse
from pathlib import Path
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--search-run',type=Path,required=True);p.add_argument('--expected-search-manifest-sha256',required=True)
    p.add_argument('--backend',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--allow-candidate-baseline',action='store_true')
    p.add_argument('--shortcut-config',type=Path,help='Explicit uniform config; default preserves the original 100-attempt profile')
    p.add_argument('--workers',type=int,default=1,help='Independent Native/model process workers, 1..8; timings record concurrency')
    a=p.parse_args()
    from workspace_analysis.experiments.shortcut_benchmark import run_shortcuts
    run_shortcuts(ROOT,a.search_run,a.backend,a.output_dir,expected_search_manifest_sha256=a.expected_search_manifest_sha256,
        allow_candidate=a.allow_candidate_baseline,shortcut_config=a.shortcut_config,workers=a.workers)


if __name__=='__main__':main()
