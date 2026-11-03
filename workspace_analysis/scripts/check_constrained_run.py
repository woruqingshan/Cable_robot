#!/usr/bin/env python3
"""Read-only audit of a D0 run against its original G2 input and Plotly traces."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def check_run(directory):
    from workspace_analysis.workspace.results import load_results,file_sha256
    from workspace_analysis.experiments.constrained_workspace import summarize
    directory=Path(directory)
    if (directory/'.incomplete').exists():raise ValueError('run is incomplete')
    manifest=json.loads((directory/'manifest.json').read_text())
    prefix=Path(manifest['input_prefix'])
    if not prefix.is_absolute():prefix=ROOT/prefix
    arrays,_=load_results(prefix)
    for suffix,digest in manifest['input_sha256'].items():
        if file_sha256(str(prefix)+suffix)!=digest:raise ValueError('input hash mismatch')
    rows_by_policy=json.loads((directory/'evaluations.json').read_text())
    summaries=json.loads((directory/'summary.json').read_text());checks={}
    counts={p:0 for p in rows_by_policy}
    with (directory/'per_sample.csv').open(newline='') as f:
        for record in csv.DictReader(f):
            policy=record['policy'];i=counts[policy];row=rows_by_policy[policy][i]
            assert int(record['sample_index'])==i==row['sample_index']
            assert json.loads(record['q'])==row['q']
            assert json.loads(record['identity'])==row['identity']
            assert record['final_decision']==row['final_decision']
            for key in ['ee_position','ee_orientation','clearance','allowed_pairs','remaining_collision_pairs']:
                assert json.loads(record[key])==row[key]
            counts[policy]+=1
    assert all(counts[p]==len(rows) for p,rows in rows_by_policy.items())
    for policy,rows in rows_by_policy.items():
        assert [r['sample_index'] for r in rows]==list(range(len(arrays['q'])))
        np.testing.assert_array_equal([r['q'] for r in rows],arrays['q'])
        for i,row in enumerate(rows):
            assert row['identity']==manifest['identities'][policy]
            if row['ee_position'] is not None:
                np.testing.assert_allclose(row['ee_position'],arrays['position'][i],atol=1e-5,rtol=0)
                np.testing.assert_allclose(row['ee_orientation'],arrays['rotation'][i],atol=1e-5,rtol=0)
        masks={'model_scope_free':np.array([r['model_scope_collision_free'] is True for r in rows]),
               'self_rejected':np.array([r['effective_self_collision'] is True for r in rows]),
               'environment_rejected':np.array([r['environment_collision'] is True for r in rows]),
               'unknown_unverified':np.array([r['final_decision'] in ['UNVERIFIED','ERROR'] for r in rows])}
        with np.load(directory/f'{policy}_workspace_results.npz',allow_pickle=False) as result:
            for key,value in {**arrays,**masks}.items():np.testing.assert_array_equal(result[key],value)
        calculated=summarize(rows)
        for key,value in calculated.items():assert summaries[policy][key]==value,(policy,key)
        assert not np.any(masks['model_scope_free'] & (masks['self_rejected']|masks['environment_rejected']))
        html=(directory/'figures'/f'{policy}.html').read_text()
        assert '</html>' in html.lower()
        pos=html.rindex('Plotly.newPlot(')
        traces,end=json.JSONDecoder().raw_decode(html[html.index('[',pos):])
        expected=[np.ones(len(rows),dtype=bool),None,*masks.values()]
        assert len(traces)==len(expected)
        for trace,mask in zip(traces,expected):
            if mask is None:continue  # base origin marker
            actual=np.column_stack([trace[key] for key in ['x','y','z']])
            np.testing.assert_array_equal(actual,arrays['position'][mask])
        checks[policy]={'configurations':len(rows),'trace_counts':{t['name']:len(t['x']) for t in traces},'input_npz_hash':manifest['input_sha256']['.npz'],'passed':True}
    return {'run':str(directory),'policies':checks,'browser_visual_acceptance':'not assessed by this data audit'}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    print(json.dumps(check_run(parser.parse_args().run_dir),indent=2))
