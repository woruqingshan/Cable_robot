#!/usr/bin/env python3
"""Four-case strict vs explicit experimental MoveIt SRDF and system-source audit."""
import argparse,json,sys,uuid
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path,required=True)
    p.add_argument('--candidate-srdf',type=Path,required=True)
    p.add_argument('--allow-candidate-baseline',action='store_true',help='explicit opt-in to physically unverified research semantics')
    p.add_argument('--system-source',type=Path,default=ROOT.parents[1]/'system_source')
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--gripper',type=float,default=.3);a=p.parse_args()
    from workspace_analysis.experiments.assembly_semantics import compare_assembly_semantics
    out=a.output_dir or ROOT/'results/c3'/('assembly_'+uuid.uuid4().hex[:10])
    summary=compare_assembly_semantics(ROOT,a.backend,out,candidate_srdf=a.candidate_srdf,source=a.system_source,allow_candidate=a.allow_candidate_baseline,gripper=a.gripper)
    print(json.dumps({'output_dir':str(out),'cases':len(summary['comparisons']),'formal_srdf_approval':summary['formal_srdf_approval'],'physical_review':summary['whole_pair_physical_review']}))
if __name__=='__main__':main()
