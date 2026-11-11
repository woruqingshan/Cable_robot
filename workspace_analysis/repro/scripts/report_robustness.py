#!/usr/bin/env python3
"""Read-only T5.3-C multi-seed/shortcut/preparation paired report and projections."""
import argparse
import csv
import json
from pathlib import Path
import sys
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--search-run',type=Path,required=True);p.add_argument('--shortcut-run',type=Path,required=True)
    p.add_argument('--preparation-run',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True)
    a=p.parse_args()
    from workspace_analysis.experiments.multistart_benchmark import bound_files
    from workspace_analysis.experiments.assembly_semantics import _hash,_write
    from workspace_analysis.experiments.robustness_reporting import robustness_groups,paired_optimization,distribution
    for folder in (a.search_run,a.shortcut_run,a.preparation_run):bound_files(folder,json.loads((folder/'manifest.json').read_text()))
    raw=json.loads((a.search_run/'summary.json').read_text())['rows']
    optimized=json.loads((a.shortcut_run/'summary.json').read_text())['rows']
    prep=json.loads((a.preparation_run/'summary.json').read_text())['rows']
    index=json.loads((a.shortcut_run/'optimization_index.json').read_text())['rows']
    pairs=paired_optimization(raw,optimized,index);groups=robustness_groups(raw)
    out=a.output_dir.absolute();out.mkdir(parents=True,exist_ok=False)
    with (out/'paired_comparison.csv').open('x',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(pairs[0]));w.writeheader();w.writerows(pairs)
    _write(out/'robustness_summary.json',{'usage':'RESEARCH_ONLY','main_denominator':sum(r['experiment']=='main' for r in raw),
        'local_denominator':sum(r['experiment']=='local' for r in raw),'groups':groups})
    _write(out/'shortcut_comparison.json',{'usage':'RESEARCH_ONLY','pairs':pairs,
        'joint_reduction_fraction':distribution([r['joint_reduction_fraction'] for r in pairs]),
        'tcp_reduction_fraction':distribution([r['tcp_reduction_fraction'] for r in pairs]),
        'tcp_length_increased':sum(r['tcp_reduction_fraction'] is not None and r['tcp_reduction_fraction']<0 for r in pairs),
        'environment_clearance_decreased':sum(r['environment_clearance_change_m'] is not None and r['environment_clearance_change_m']<0 for r in pairs)})
    _write(out/'budget_sensitivity.json',json.loads((a.search_run/'budget_comparison.json').read_text()))
    _write(out/'preparation_summary.json',{'usage':'RESEARCH_ONLY','label':'REFERENCE_START_TO_READY','rows':prep,
        'home_audit':json.loads((a.preparation_run/'home_source_audit.json').read_text())})
    from collections import Counter
    from workspace_analysis.experiments.multistart_benchmark import open_ledger
    failures=[]
    for r in raw+prep:
        if r['success']:continue
        data=json.loads((Path(r['trial'])/'result.json').read_text());trial=data['trial']
        queried=open_ledger(Path(r['trial']))
        codes=Counter(c for q in queried for c in q['reasons'])
        env=Counter((v['first'],v['second']) for q in queried for v in q.get('environment_pairs',[]))
        failures.append({'query_reason_occurrences':dict(codes),
            'environment_pair_occurrences':[{'first':a,'second':b,'queries':n} for (a,b),n in env.items()],
            'query_scope':'real Native query records, not cache-hit request counts; exploratory rejections are not a no-path proof',
            'row':r,'first_failure':(trial.get('validation') or {}).get('first_failure'),
            'raw_search_status':(trial.get('search') or {}).get('status'),
            'iterations':(trial.get('search') or {}).get('iterations'),
            'note':'search budget exhaustion is not proof of geometric infeasibility'})
    _write(out/'failure_diagnosis.json',{'failures':failures})
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    import numpy as np
    fig,axes=plt.subplots(1,3,figsize=(12,4))
    for ax,case in zip(axes,('C1','C2','C3')):
        selected=[g for g in groups if g['experiment']=='main' and g['case']==case]
        ax.bar([g['start'] for g in selected],[g['accepted'] for g in selected]);ax.set_ylim(0,10)
        ax.set_title(case+' accepted / 10 seeds');ax.set_ylabel('Accepted discrete paths');ax.grid(axis='y')
    fig.tight_layout();fig.savefig(out/'multi_seed_success_counts.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(12,7))
    for col,case in enumerate(('C1','C2','C3')):
        for line,field,label in [(0,'joint_length_rad','Joint length [rad]'),(1,'tcp_length_m','TCP length [m]')]:
            values=[[r[field] for r in raw if r['experiment']=='main' and r['case']==case and r['start']==sid and r['success']] for sid in ('S0','S1','S2')]
            axes[line,col].boxplot(values,labels=['S0','S1','S2']);axes[line,col].set_title(case+' successful paths')
            axes[line,col].set_ylabel(label);axes[line,col].grid(axis='y')
    fig.tight_layout();fig.savefig(out/'accepted_path_length_distributions.png',dpi=140);plt.close(fig)
    eligible=[r for r in pairs if r['optimized_joint_length_rad'] is not None]
    fig,axes=plt.subplots(1,3,figsize=(12,4))
    for ax,xkey,ykey,label in [(axes[0],'raw_joint_length_rad','optimized_joint_length_rad','Joint length [rad]'),
            (axes[1],'raw_tcp_length_m','optimized_tcp_length_m','TCP length [m]'),
            (axes[2],'raw_environment_clearance_m','optimized_environment_clearance_m','Environment separation [m]')]:
        x=[r[xkey] for r in eligible];y=[r[ykey] for r in eligible]
        ax.scatter(x,y,s=12);lo=min(x+y);hi=max(x+y);ax.plot([lo,hi],[lo,hi],'k--',linewidth=.8)
        ax.set_xlabel('Raw '+label);ax.set_ylabel('Shortcut '+label);ax.grid(True)
    fig.tight_layout();fig.savefig(out/'raw_shortcut_paired_metrics.png',dpi=140);plt.close(fig)
    # Fixed representative seed chosen before seeing optimization gains.
    representatives=[r for r in pairs if r['seed']==20261104 and r['case'] in ('C1','LOCAL') and r['optimized_trial'] is not None]
    for row in representatives:
        fig,axes=plt.subplots(1,2,figsize=(10,4))
        for label,path in [('Raw',row['raw_trial']),('Shortcut',row['optimized_trial'])]:
            folder=Path(path);data=json.loads((folder/'result.json').read_text());inp=json.loads((folder/'inputs.json').read_text())
            world=np.array(json.loads((folder/'base_pose_context.json').read_text())['world_from_base'])
            with np.load(folder/data['trial']['trajectory_file'],allow_pickle=False) as arrays:
                pts=(world@arrays['T_base_tcp'])[:,:3,3]*1000
            for ax,dim,name in [(axes[0],1,'Y'),(axes[1],2,'Z')]:
                ax.plot(pts[:,0],pts[:,dim],label=label);ax.set_xlabel('World X [mm]');ax.set_ylabel('World '+name+' [mm]')
                ax.axvline(250,color='magenta',linewidth=.7);ax.grid(True);ax.set_aspect('equal',adjustable='datalim')
        for ax in axes:ax.legend()
        fig.suptitle(row['case']+' / '+row['start']+' / seed 20261104 / RESEARCH_ONLY')
        fig.tight_layout();fig.savefig(out/(row['case']+'_'+row['start']+'_raw_shortcut.png'),dpi=130);plt.close(fig)
    _write(out/'manifest.json',{'usage':'RESEARCH_ONLY','source_manifests':{str(f):_hash(f/'manifest.json') for f in (a.search_run,a.shortcut_run,a.preparation_run)},
        'files':{f.name:_hash(f) for f in out.iterdir() if f.is_file()}})
    print(json.dumps({'groups':[(g['case'],g['start'],g['accepted'],g['scheduled']) for g in groups],
        'shortcut_pairs':len(pairs),'preparation_trials':len(prep)},indent=2))


if __name__=='__main__':main()
