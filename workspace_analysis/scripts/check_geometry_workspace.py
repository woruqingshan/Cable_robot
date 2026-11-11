#!/usr/bin/env python3
"""Read-only D1 paired artifact / identity / exact q and HTML coordinate audit."""
import argparse,csv,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'scripts'))
from check_constrained_run import check_run
from workspace_analysis.workspace.results import load_results,file_sha256
from workspace_analysis.collision.geometry_profile import canonical
from hashlib import sha256


def check(directory):
    directory=Path(directory)
    if (directory/'.incomplete').exists():raise ValueError('D1 incomplete')
    m=json.loads((directory/'manifest.json').read_text());s=json.loads((directory/'summary.json').read_text());arrays,_=load_results(m['input_prefix']);n=len(arrays['q'])
    for suffix,digest in m['input_sha256'].items():assert file_sha256(str(m['input_prefix'])+suffix)==digest
    results={};records={};paired=[]
    for context in m['identities']:
        results[context]={};records[context]={}
        for variant in ['official','refined']:
            child=directory/context/variant;results[context][variant]=check_run(child)
            rows=json.loads((child/'evaluations.json').read_text())['assembly_policy_v1'];records[context][variant]=rows
            identity=m['identities'][context][variant]
            assert identity['geometry_profile_sha256']==sha256(canonical(m['profiles'][variant]).encode()).hexdigest()
            assert all(r['identity']==identity for r in rows)
        a=m['identities'][context]['official'];b=m['identities'][context]['refined']
        assert a['urdf_sha256']==b['urdf_sha256'] and a['policy_sha256']==b['policy_sha256'] and a['scene_sha256']==b['scene_sha256']
        assert a['robot_geometry_sha256']!=b['robot_geometry_sha256'] and b['geometry_review_status']=='diagnostic_only'
        for i,(a,b) in enumerate(zip(records[context]['official'],records[context]['refined'])):
            assert a['q']==b['q']==arrays['q'][i].tolist()
            assert a['environment_collision']==b['environment_collision']
            x={(p['kind'],*sorted(p['links'])) for p in a['remaining_collision_pairs']};y={(p['kind'],*sorted(p['links'])) for p in b['remaining_collision_pairs']}
            paired.append((context,i,sorted(x-y),sorted(y-x)))
    with (directory/'comparison.csv').open(newline='') as f:
        rows=list(csv.DictReader(f));assert len(rows)==len(paired)
        for row,(context,i,removed,added) in zip(rows,paired):
            assert row['scenario']==context and int(row['sample_index'])==i and json.loads(row['q'])==arrays['q'][i].tolist()
            assert json.loads(row['removed_pairs'])==[list(x) for x in removed] and json.loads(row['added_pairs'])==[list(x) for x in added]
    with np.load(directory/'workspace_results.npz',allow_pickle=False) as data:
        for k,v in arrays.items():np.testing.assert_array_equal(data[k],v)
        a=np.array([r['model_scope_collision_free'] is True for r in records['idc']['official']]);b=np.array([r['model_scope_collision_free'] is True for r in records['idc']['refined']])
        env=np.array([r['environment_collision'] for r in records['idc']['refined']]);unknown=np.array([r['final_decision']=='UNVERIFIED' for r in records['idc']['refined']])
        for k,v in {'official_candidate':a,'refined_candidate':b,'official_rejected':~a,'refined_rejected':~b,'environment_collision':env,'unknown_unverified':unknown}.items():np.testing.assert_array_equal(data[k],v)
        p=s['idc']['paired'];assert len(p['recovered_indices'])+len(p['new_rejected_indices'])+p['both_rejected_count']+p['both_candidate_count']==n
        html=(directory/'figures/profile_comparison.html').read_text();start=html.rindex('Plotly.newPlot(');traces,_=json.JSONDecoder().raw_decode(html[html.index('[',start):])
        for trace,mask in zip(traces,[np.ones(n,dtype=bool),None,~a,~b,b,env,unknown]):
            if mask is not None:np.testing.assert_array_equal(np.column_stack([trace[x] for x in ['x','y','z']]),arrays['position'][mask])
    return {'run':str(directory),'configurations':n,'paired_records':len(paired),'passed':True,'child_checks':results,'physical_safety_verified':False,'browser_visual_acceptance':'pending user review'}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True);print(json.dumps(check(p.parse_args().run_dir),indent=2))
