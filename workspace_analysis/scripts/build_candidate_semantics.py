#!/usr/bin/env python3
"""Build an explicitly unverified SRDF using ONLY frozen MoveIt Adjacent/Default."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate-matrix',type=Path,required=True)
    p.add_argument('--strict-srdf',type=Path,default=ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf')
    p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.collision.candidate_baseline import build_candidate_baseline
    config=build_candidate_baseline(load_model(ROOT/'config/workspace.yaml'),a.strict_srdf,a.candidate_matrix,a.output_dir)
    print(json.dumps({'output_dir':str(a.output_dir),'disabled_pairs':len(config.wire()['semantic']['disabled_pairs']),'review_status':config.review_status,'usage':'research_only'}))
if __name__=='__main__':main()
