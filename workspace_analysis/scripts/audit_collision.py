#!/usr/bin/env python3
"""C2 topology audit and explicit policy comparison over the existing 100 G2 rows."""
import argparse
from html import escape
from collections import Counter
from datetime import datetime,timezone
from hashlib import sha256
import csv
import json
from pathlib import Path
import sys
import uuid
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path,required=True)
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--view-sample',type=int,default=8)
    p.add_argument('--c1-report',type=Path,default=ROOT/'results/c1/collision_probes/20261103T101535Z_n100_2cec62ce/diagnostics.json')
    args=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.workspace.results import load_results
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.policy import load_policy
    from workspace_analysis.collision.coverage import coverage_report
    from workspace_analysis.viz.scene import build_scene_figure,export_scene_html
    model=load_model(ROOT/'config/workspace.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    arrays,metadata=load_results(ROOT/'results/workspace_n100_seed42',model=model)
    assert len(arrays['q'])==100
    old=json.loads(args.c1_report.read_text())
    assert old['model']['expanded_urdf_sha256']==model.metadata()['expanded_urdf_sha256']
    assert old['scene_canonical_sha256']==scene.canonical_scene_sha256
    for i,row in enumerate(old['per_sample']):
        assert row['sample_index']==i and row['q_rad']==arrays['q'][i].tolist()
    parents={j.child:j for j in model.joints.values()}
    def ancestry(link):
        path=[]
        while link in parents:
            joint=parents[link];path.append(joint);link=joint.parent
        return path
    def relationship(a,b):
        aa=[a]+[j.parent for j in ancestry(a)];bb=[b]+[j.parent for j in ancestry(b)]
        lca=next(x for x in aa if x in bb)
        path=ancestry(a)[:aa.index(lca)]+ancestry(b)[:bb.index(lca)]
        return lca,path
    seen={}
    for row in old['per_sample']:
        assert not row['pairs_may_be_truncated']
        for pair in row['colliding_pairs']:
            if pair['kind']=='self':seen.setdefault(tuple(sorted([pair['first'],pair['second']])),[]).append(row['sample_index'])
    allowed_names={tuple(sorted(x)) for x in [('back_shell_black_link','back_shell_green_link'),('gripper_link','link5'),('link1','servo_link1')]}
    audits=[]
    for pair,indices in sorted(seen.items()):
        lca,path=relationship(*pair);moving=[j.name for j in path if j.kind!='fixed']
        allowed=pair in allowed_names
        if allowed:assert not moving,'explicit fixed-assembly candidate must have invariant relative pose'
        category='A' if not moving else 'B' if len(path)==1 else 'C' if len(indices)<100 else 'E'
        audits.append({'links':list(pair),'lca':lca,'joint_path':[j.name for j in path],
                       'fixed_relationship':not moving,'relative_motion_joints':moving,
                       'frequency':len(set(indices)),'sample_indices':sorted(set(indices)),
                       'example_q':arrays['q'][indices[0]].tolist(),
                       'geometry':[g for g in old['backend_ready']['audit']['geometry'] if g['link'] in pair],
                       'category':category,'classification_basis':'URDF connecting path and fixed/moving degrees; frequency is observational only',
                       'recommendation':'provisional model-only assembly exemption' if allowed else 'retain collision detection; inspect meshes/contact region',
                       'review_status':'provisional_model_only' if allowed else 'unresolved'})
    out=args.output_dir or ROOT/'results/c2'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8])
    out.parent.mkdir(parents=True,exist_ok=True);out.mkdir()
    (out/'.incomplete').write_text('C2 in progress')
    coverage=coverage_report(model)
    results={};identities={};ready=None
    with CollisionClient(args.backend) as client:
        ready=client.initialize(model,scene)
        for label in ['baseline','assembly_policy_v0']:
            text,digest=load_policy(ROOT/f'config/collision/{label}.yaml',model)
            client.set_policy(text,digest);identities[label]={'sha256':digest,'policy':json.loads(text)}
            rows=[]
            for i,q in enumerate(arrays['q']):
                try:
                    reply=client.query(model,q)
                    rows.append({'sample_index':i,'q':q.tolist(),'gripper':0.,**reply})
                except Exception as exc:rows.append({'sample_index':i,'q':q.tolist(),'effective_decision':'ERROR','error':str(exc)})
            results[label]=rows
    def summary(rows):
        good=[r for r in rows if 'error' not in r]
        return {'total':len(rows),'raw_self':sum(r['raw_self_collision'] for r in good),
                'effective_self':sum(r['effective_self_collision'] for r in good),
                'environment':sum(r['environment_collision'] for r in good),
                'both':sum(r['effective_self_collision'] and r['environment_collision'] for r in good),
                'decisions':dict(Counter(r['effective_decision'] for r in rows)),
                'mean_query_s':float(np.mean([r['query_elapsed_s'] for r in good])) if good else None,
                'environment_indices':[r['sample_index'] for r in good if r['environment_collision']],
                'effective_pair_frequency':[{'links':list(k),'samples':v} for k,v in Counter(tuple(p) for r in good for p in r['effective_self_pairs']).most_common()]}
    for label,rows in results.items():
        for i,row in enumerate(rows):
            if 'error' not in row:
                assert row['raw_self_collision']==old['per_sample'][i]['self_collision']
                assert row['environment_collision']==old['per_sample'][i]['environment_collision']
    payloads={'pair_audit.json':audits,'coverage.json':coverage,'policy_identity.json':identities,
              'baseline_summary.json':summary(results['baseline']),'policy_summary.json':summary(results['assembly_policy_v0']),
              'queries.json':results,'robot_ready.json':ready}
    for name,value in payloads.items():
        with (out/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
    with (out/'per_sample.csv').open('x',newline='') as f:
        fields=['policy','sample_index','q','raw_self','effective_self','environment','decision','filtered_pairs','error'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for label,rows in results.items():
            for r in rows:writer.writerow(dict(policy=label,sample_index=r['sample_index'],q=json.dumps(r['q']),raw_self=r.get('raw_self_collision'),effective_self=r.get('effective_self_collision'),environment=r.get('environment_collision'),decision=r['effective_decision'],filtered_pairs=json.dumps(r.get('filtered_pairs')),error=r.get('error')))
    with (out/'comparison.csv').open('x',newline='') as f:
        writer=csv.writer(f);writer.writerow(['sample_index','baseline_decision','policy_decision','changed'])
        for a,b in zip(*results.values()):writer.writerow([a['sample_index'],a['effective_decision'],b['effective_decision'],a['effective_decision']!=b['effective_decision']])
    index=args.view_sample
    if not 0<=index<100:raise ValueError('view sample must be in 0..99')
    row=results['assembly_policy_v0'][index];fig=build_scene_figure(scene,axis_length=.015)
    import plotly.graph_objects as go
    transforms=row.get('link_transforms_base',{})
    xyz=np.array([np.array(t)[:3,3] for t in transforms.values()])
    if len(xyz):fig.add_trace(go.Scatter3d(x=xyz[:,0],y=xyz[:,1],z=xyz[:,2],text=list(transforms),mode='markers',name='Link origins ONLY (not collision surfaces)'))
    fig.update_layout(meta={**dict(fig.layout.meta),'collision_diagnostic':row},title=f'C2 sample {index}, q={row["q"]}; {row["effective_decision"]}; pair data in layout.meta')
    export_scene_html(fig,out/f'sample_{index}.html')
    info={k:row.get(k) for k in ['sample_index','q','gripper','raw_self_collision','effective_self_collision',
          'environment_collision','effective_decision','filtered_pairs','effective_self_pairs','colliding_pairs','applied_policy','identity']}
    with (out/f'viewer_{index}.html').open('x') as f:
        f.write('<!doctype html><meta charset="utf-8"><title>C2 diagnostic viewer</title>'
                '<h1>C2 diagnostic — unverified model scope</h1>'
                f'<iframe src="sample_{index}.html" style="width:100%;height:800px;border:0"></iframe>'
                '<h2>Numerical query / policy / collision pairs</h2><pre>'+escape(json.dumps(info,indent=2))+'</pre>')
    (out/'.incomplete').unlink()
    print(json.dumps({'output':str(out),'summaries':{k:summary(v) for k,v in results.items()},'pair_count':len(audits)},indent=2))
    return 1 if any('error' in r for rows in results.values() for r in rows) else 0


if __name__=='__main__':raise SystemExit(main())
