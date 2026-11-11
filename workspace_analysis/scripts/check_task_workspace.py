#!/usr/bin/env python3
"""Read-only D2 audit: input identities, raw facts, task semantics, CSV/NPZ/HTML."""
import argparse,csv,json,sys
from pathlib import Path
from hashlib import sha256
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from workspace_analysis.workspace.results import load_results,file_sha256
from workspace_analysis.experiments.task_scoped_workspace import task_summary
from workspace_analysis.experiments.constrained_workspace import summarize

def check(path):
    path=Path(path)
    if (path/'.incomplete').exists():raise ValueError('incomplete D2')
    m=json.loads((path/'manifest.json').read_text());s=json.loads((path/'summary.json').read_text());r=json.loads((path/'evaluations.json').read_text());arrays,_=load_results(m['input_prefix']);n=len(arrays['q'])
    for suffix,digest in m['input_sha256'].items():assert file_sha256(str(m['input_prefix'])+suffix)==digest
    text=json.dumps(m['scope'],sort_keys=True,separators=(',',':'),ensure_ascii=False)
    assert sha256(text.encode()).hexdigest()==m['identities']['B_task_scope']['task_scope_sha256']
    a=r['A_official_full'];b=r['B_task_scope'];assert len(a)==len(b)==n
    for label,rows in r.items():
        assert [v['sample_index'] for v in rows]==list(range(n))
        np.testing.assert_array_equal([v['q'] for v in rows],arrays['q']);np.testing.assert_allclose([v['ee_position'] for v in rows],arrays['position'],atol=1e-5,rtol=0);np.testing.assert_allclose([v['ee_orientation'] for v in rows],arrays['rotation'],atol=1e-5,rtol=0)
        assert all(v['identity']==m['identities'][label] for v in rows)
    for x,y in zip(a,b):
        for key in ['raw_self_collision','effective_self_collision','environment_collision','remaining_collision_pairs','allowed_pairs']:assert x[key]==y[key]
        assert y['official_effective_self_collision']==x['effective_self_collision']
        native=y['task_scope_diagnostics'];assert native['candidate_computed_from_fresh_boolean_queries'] and native['scope_sha256']==m['identities']['B_task_scope']['task_scope_sha256']
        assert native['gripper_internal_verification']=='UNVERIFIED_MECHANISM' and y['hardware_safe'] is None
        assert not y['task_scope_candidate'] or (not y['task_scope_blocking_self_collision'] and not y['environment_collision'] and all(y['task_scope_clearance']['threshold_met'].values()))
    assert summarize(a)==s['A_official_full']
    for key,value in task_summary(b).items():assert s['B_task_scope'][key]==value
    assert sum(s['B_task_scope'][k] for k in ['candidates','rejected','unknown','errors'])==n
    counters={k:0 for k in r}
    with (path/'per_sample.csv').open(newline='') as f:
        for row in csv.DictReader(f):
            label=row['condition'];i=counters[label];original=r[label][i]
            assert int(row['sample_index'])==i and json.loads(row['q'])==original['q'] and json.loads(row['identity'])==original['identity']
            if label=='B_task_scope':assert row['task_scope_status']==original['task_scope_status'] and json.loads(row['task_scope_diagnostics'])==original['task_scope_diagnostics']
            counters[label]+=1
    assert all(v==n for v in counters.values())
    with (path/'comparison.csv').open(newline='') as f:
        rows=list(csv.DictReader(f));assert len(rows)==n
        declared={tuple(p['links']) for p in m['scope']['internal_diagnostic_pairs']}
        for i,row in enumerate(rows):
            assert int(row['sample_index'])==i and json.loads(row['q'])==arrays['q'][i].tolist()
            assert {tuple(p) for p in json.loads(row['removed_from_blocking'])}<=declared
    masks={'full_rejected':np.array([not v['model_scope_collision_free'] for v in a]),'task_rejected':np.array([v['task_scope_status']=='REJECTED' for v in b]),'task_candidates':np.array([v['task_scope_candidate'] is True for v in b]),'environment_rejected':np.array([v['environment_collision'] for v in b]),'unknown':np.array([v['task_scope_status'] in ['UNKNOWN','ERROR'] for v in b])}
    with np.load(path/'workspace_results.npz',allow_pickle=False) as npz:
        for key,values in {**arrays,**masks}.items():np.testing.assert_array_equal(npz[key],values)
    html=(path/'figures/workspace_comparison.html').read_text();start=html.rindex('Plotly.newPlot(');traces,_=json.JSONDecoder().raw_decode(html[html.index('[',start):]);assert len(traces)==7
    for trace,mask in zip(traces,[np.ones(n,dtype=bool),None,*masks.values()]):
        if mask is not None:np.testing.assert_array_equal(np.column_stack([trace[k] for k in ['x','y','z']]),arrays['position'][mask])
    return {'run':str(path),'configurations':n,'passed':True,'input_arrays_and_indices_preserved':True,'independent_native_scope_boolean':True,'html_trace_counts':{t['name']:len(t['x']) for t in traces},'physical_hardware_validated':False}
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True);print(json.dumps(check(p.parse_args().run_dir),indent=2))
