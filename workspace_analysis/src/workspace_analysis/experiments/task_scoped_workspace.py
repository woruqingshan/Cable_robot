"""D2: official geometry, fixed ACM, explicit scope as the sole changed variable."""
import csv,json,subprocess
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
from ..robot.model import load_model
from ..scene.schema import load_scene
from ..workspace.results import load_results,file_sha256
from ..collision.geometry_profile import load_geometry_profile
from ..collision.policy import load_policy
from ..collision.scope import load_scope
from ..collision.evaluator import Evaluator
from ..viz.workspace import prepare_view,build_3d_figure,standalone_html
from .constrained_workspace import summarize
from .geometry_workspace import clearance_distribution


def task_summary(rows):
    count=lambda key:sum(r.get(key) is True for r in rows)
    times=[r['query_time'] for r in rows]
    from collections import Counter
    return {'total':len(rows),'full_effective_self':count('official_effective_self_collision'),'task_blocking_self':count('task_scope_blocking_self_collision'),'gripper_internal':count('gripper_internal_collision'),'environment':count('environment_collision'),'candidates':count('task_scope_candidate'),'rejected':sum(r.get('task_scope_status')=='REJECTED' for r in rows),'unknown':sum(r.get('task_scope_status')=='UNKNOWN' for r in rows),'errors':sum(r.get('task_scope_status')=='ERROR' for r in rows),'candidate_ratio':count('task_scope_candidate')/len(rows) if rows else None,'mean_query_s':float(np.mean(times)) if times else None,'p95_query_s':float(np.percentile(times,95)) if times else None,'blocking_pairs':[{'links':list(p),'count':n} for p,n in Counter(tuple(p) for r in rows for p in r.get('task_scope_blocking_pairs',[])).most_common()],'meaning':'configuration-level explicit research scope, not full robot safety or IDC task success'}


def run_task_pilot(root,backend,prefix,output,*,gripper=.3,clearance=None,batch_size=64):
    root=Path(root);out=Path(output)
    if out.exists():raise ValueError('refusing existing D2 run')
    model=load_model(root/'config/workspace.yaml');scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml');arrays,metadata=load_results(prefix,model=model)
    policy=load_policy(root/'config/collision/assembly_policy_v1.yaml',model);scope=load_scope(root/'config/collision/task_scope_v0.yaml',model,policy);profile=load_geometry_profile(root/'config/collision/geometry_profiles/official_full.yaml',model)
    out.mkdir(parents=True);(out/'.incomplete').write_text('D2 in progress');(out/'figures').mkdir()
    rows={};ids={}
    for label,selected in [('A_official_full',None),('B_task_scope',scope)]:
        with Evaluator(model,scene,policy,backend,gripper=gripper,clearance=clearance,geometry_profile=profile,task_scope=selected) as evaluator:
            rows[label]=evaluator.evaluate(arrays['q'],batch_size=batch_size);ids[label]=evaluator.identity
        if not evaluator.client.cleanup['exit_normal']:raise RuntimeError('backend cleanup abnormal')
    a=rows['A_official_full'];b=rows['B_task_scope'];comparisons=[]
    for i,(x,y) in enumerate(zip(a,b)):
        if x['final_decision']=='ERROR' or y['final_decision']=='ERROR':raise RuntimeError('query error, incomplete D2 retained')
        for key in ['q','raw_self_collision','effective_self_collision','environment_collision','allowed_pairs','remaining_collision_pairs','ee_position','ee_orientation']:assert x[key]==y[key],(i,key)
        np.testing.assert_array_equal(x['q'],arrays['q'][i]);np.testing.assert_allclose(x['ee_position'],arrays['position'][i],atol=1e-5,rtol=0);np.testing.assert_allclose(x['ee_orientation'],arrays['rotation'][i],atol=1e-5,rtol=0)
        full={tuple(p['links']) for p in x['remaining_collision_pairs'] if p['kind']=='self'};blocking={tuple(p) for p in y['task_scope_blocking_pairs']};internal={tuple(p['links']) for p in json.loads(scope[0])['internal_diagnostic_pairs']}
        assert full-blocking<=internal and not blocking-full
        comparisons.append({'sample_index':i,'q':x['q'],'full_rejected':not x['model_scope_collision_free'],'task_scope_candidate':y['task_scope_candidate'],'task_scope_status':y['task_scope_status'],'internal_collision':y['gripper_internal_collision'],'environment_collision':y['environment_collision'],'removed_from_blocking':sorted(full-blocking),'blocking_pairs':y['task_scope_blocking_pairs']})
    summary={'A_official_full':summarize(a),'B_task_scope':task_summary(b)}
    summary['B_task_scope']['clearance_distribution']=clearance_distribution([{**r,'clearance':{'native':r['task_scope_clearance']['native']}} for r in b])
    summary['classification_changed_indices']=[i for i,(x,y) in enumerate(zip(a,b)) if x['model_scope_collision_free']!=y['task_scope_candidate']]
    summary['blocking_pair_set_changed_indices']=[r['sample_index'] for r in comparisons if r['removed_from_blocking']]
    masks={'full_rejected':np.array([not r['model_scope_collision_free'] for r in a]),'task_rejected':np.array([r['task_scope_status']=='REJECTED' for r in b]),'task_candidates':np.array([r['task_scope_candidate'] is True for r in b]),'environment_rejected':np.array([r['environment_collision'] for r in b]),'unknown':np.array([r['task_scope_status'] in ['UNKNOWN','ERROR'] for r in b])}
    assert sum(summary['B_task_scope'][k] for k in ['candidates','rejected','unknown','errors'])==len(b)
    np.savez(out/'workspace_results.npz',**arrays,**masks)
    for name,data in [('per_sample.csv',[{'condition':label,**r} for label,items in rows.items() for r in items]),('comparison.csv',comparisons)]:
        fields=sorted(set().union(*(r.keys() for r in data)))
        with (out/name).open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
            for r in data:writer.writerow({k:json.dumps(v,separators=(',',':')) if isinstance(v,(dict,list)) else v for k,v in r.items()})
    view=prepare_view(arrays,metadata,source_prefix=str(prefix));fig=build_3d_figure(view);fig.data[0].update(name='Original Workspace',visible='legendonly')
    import plotly.graph_objects as go
    for name,key,color in [('Full Model Rejected','full_rejected','#64748b'),('Task-Scope Rejected','task_rejected','#ef4444'),('Task-Scope Candidates (NOT full safe)','task_candidates','#16a34a'),('Environment Rejected','environment_rejected','#f97316'),('Unknown','unknown','#a855f7')]:
        mask=masks[key];xyz=arrays['position'][mask];fig.add_trace(go.Scatter3d(x=xyz[:,0],y=xyz[:,1],z=xyz[:,2],customdata=view.customdata[mask],hovertemplate=fig.data[0].hovertemplate,name=f'{name}: {int(mask.sum())}',mode='markers',marker={'color':color,'size':3}))
    fig.update_layout(title='D2 official full geometry / same ACM v1: independent task scope; internal mechanism UNVERIFIED',meta={'identities':ids,'scope':json.loads(scope[0]),'candidate_is_full_safety':False})
    with (out/'figures/workspace_comparison.html').open('x') as f:f.write(standalone_html(fig))
    sources={str(p.relative_to(root)):file_sha256(p) for folder in ['src/workspace_analysis/collision','src/workspace_analysis/experiments','src/workspace_analysis/viz','cpp/collision_backend','config/collision','scripts'] for p in sorted((root/folder).rglob('*')) if p.suffix in ['.py','.cpp','.hpp','.yaml']}
    manifest={'stage':'D2','created_utc':datetime.now(timezone.utc).isoformat(),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'source_sha256':sources,'input_prefix':str(Path(prefix).absolute()),'input_sha256':{s:file_sha256(str(prefix)+s) for s in ['.json','.npz']},'identities':ids,'scope':json.loads(scope[0]),'geometry_profile':json.loads(profile.canonical_json),'gripper_rad':gripper,'clearance':clearance or {'self_m':0,'environment_m':0},'configurations':len(b),'negative_z_retained':int((arrays['position'][:,2]<0).sum()),'hardware_validated':False}
    for name,value in [('summary.json',summary),('evaluations.json',rows),('manifest.json',manifest)]:
        with (out/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
    (out/'.incomplete').unlink();return summary
