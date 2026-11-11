#!/usr/bin/env python3
"""Read-only T5.3-B statistics, failure diagnosis and checked TCP projections."""
import argparse
import csv
from collections import Counter
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run.resolve();out=args.output_dir.absolute()
    from workspace_analysis.experiments.multistart_benchmark import bound_files,open_ledger
    from workspace_analysis.experiments.assembly_semantics import _hash,_write
    parent=json.loads((run/'manifest.json').read_text());bound_files(run,parent)
    out.mkdir(parents=True,exist_ok=False)
    rows=json.loads((run/'summary.json').read_text())['rows'];failures=[];stats=[]
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    for name in parent['conditions']:
        condition=run/name;data=json.loads((condition/'result.json').read_text());ledger=open_ledger(condition)
        for goal in data['goals']:
            fig,axes=plt.subplots(1,2,figsize=(10,4))
            for trial in goal['paths']:
                lo,hi=trial.get('query_range',[0,0]);queried=ledger[lo:hi]
                counts=Counter(reason for s in queried for reason in s['reasons'])
                stats.append({'condition':name,'goal':goal['solution_id'],'planner':trial['method'],
                    'status':trial['status'],'query_reason_occurrences':dict(counts),'quality':trial['quality']})
                check=trial.get('validation');state=trial.get('first_failure_state')
                if not trial['success']:
                    failures.append({'condition':name,'start':data['start_id'],'mode':data['workspace_mode'],
                        'goal':goal['solution_id'],'planner':trial['method'],'status':trial['status'],
                        'first_failed_sample':None if check is None else check['first_failure'],
                        'state':state,'query_reason_occurrences':dict(counts),
                        'note':'rejected search states describe exploration, not proof of infeasibility'})
                if check is not None:
                    points=[ledger[i]['workspace']['world_tcp_xyz_m'] for i in check['state_query_ids']]
                    for ax,vertical,label in [(axes[0],1,'Y'),(axes[1],2,'Z')]:
                        ax.plot([p[0]*1000 for p in points],[p[vertical]*1000 for p in points],
                            '-' if trial['success'] else '--',label=trial['method']+' '+trial['status'])
                        ax.set_xlabel('World X [mm]');ax.set_ylabel('World '+label+' [mm]')
                for ax in axes:
                    ax.axvline(250,color='magenta',linewidth=.8);ax.grid(True);ax.set_aspect('equal',adjustable='datalim')
            for ax in axes:
                if ax.lines:ax.legend(fontsize=6)
            fig.suptitle(name+' / Goal '+str(goal['solution_id'])+' / RESEARCH_ONLY',fontsize=9)
            fig.tight_layout();fig.savefig(out/(name+'_goal'+str(goal['solution_id'])+'.png'),dpi=130);plt.close(fig)
    counts=[]
    for start in ('S0','S1','S2'):
        for mode in ('conditional','local_only'):
            for method in ('linear','rrt_connect'):
                subset=[r for r in rows if r['start']==start and r['mode']==mode and r['planner']==method]
                if subset:counts.append({'start':start,'mode':mode,'planner':method,'scheduled':len(subset),
                    'executed':sum(r['status']!='NOT_RUN' for r in subset),'accepted':sum(r['success'] for r in subset)})
    _write(out/'analysis_summary.json',{'usage':'RESEARCH_ONLY','source_manifest_sha256':_hash(run/'manifest.json'),
        'counts':counts,'planner_trials':stats,'scope':'one fixed seed; no home-to-ready motion verified'})
    _write(out/'failure_diagnosis.json',{'usage':'RESEARCH_ONLY','failures':failures})
    _write(out/'manifest.json',{'usage':'RESEARCH_ONLY','source_manifest_sha256':_hash(run/'manifest.json'),
        'files':{p.name:_hash(p) for p in out.iterdir() if p.is_file()}})
    print(json.dumps(counts,indent=2))


if __name__=='__main__':main()
