"""D1 paired profile experiments. Uses the original G2 reader and D0 evaluator."""
import csv,json,subprocess
from pathlib import Path
from datetime import datetime,timezone
from hashlib import sha256
import numpy as np
from ..robot.model import load_model
from ..scene.schema import load_scene,parse_scene
from ..collision.geometry_profile import load_geometry_profile
from ..workspace.results import load_results,file_sha256
from ..viz.workspace import prepare_view,build_3d_figure,standalone_html
from .constrained_workspace import run_pilot


def robot_only(scene):
    data=scene.to_dict()
    for key in ['collision_objects','entities','ports']:data['scene'][key]=[]
    data['scene']['coverage']='ROBOT_ONLY diagnostic; no IDC obstacle protection'
    return parse_scene(json.dumps(data))


def _pairs(row):return {(p['kind'],*sorted(p['links'])) for p in row['remaining_collision_pairs']}


def clearance_distribution(rows):
    result={}
    for kind in ['self','environment']:
        values=[r['clearance']['native'][kind]['separation_m'] for r in rows if r['clearance'] and r['clearance']['native'][kind]['available']]
        result[kind]={'available_count':len(values),'missing_count':len(rows)-len(values),'min_p05_median_p95_max_m':np.percentile(values,[0,5,50,95,100]).tolist() if values else None,'meaning':'unsigned geometric separation; overlap zero, not signed depth'}
    return result


def run_comparison(root,backend,prefix,output,*,gripper=.3,batch_size=64,include_robot_only=True):
    root=Path(root);out=Path(output)
    if out.exists():raise ValueError('refusing existing D1 output')
    out.mkdir(parents=True);(out/'.incomplete').write_text('D1 in progress')
    model=load_model(root/'config/workspace.yaml');scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    arrays,metadata=load_results(prefix,model=model)
    profiles={'official':load_geometry_profile(root/'config/collision/geometry_profiles/official_full.yaml',model),
              'refined':load_geometry_profile(root/'config/collision/geometry_profiles/refined_assembly_v0.yaml',model,asset_dir=out/'geometry_assets')}
    scenarios={'idc':scene}
    if include_robot_only:scenarios['robot_only']=robot_only(scene)
    summary={};comparisons=[];masks={};identities={}
    for scenario,snapshot in scenarios.items():
        rows={};summary[scenario]={};identities[scenario]={}
        for label,profile in profiles.items():
            directory=out/scenario/label
            stats=run_pilot(root,backend,prefix,directory,gripper=gripper,policies=['assembly_policy_v1'],batch_size=batch_size,geometry_profile=profile,scene_snapshot=snapshot)
            rows[label]=json.loads((directory/'evaluations.json').read_text())['assembly_policy_v1']
            if any(r['final_decision']=='ERROR' for r in rows[label]):raise ValueError('D1 query ERROR; incomplete run retained')
            summary[scenario][label]={**stats['assembly_policy_v1'],'clearance_distribution':clearance_distribution(rows[label])}
            identities[scenario][label]=json.loads((directory/'manifest.json').read_text())['identities']['assembly_policy_v1']
            np.testing.assert_array_equal([r['q'] for r in rows[label]],arrays['q'])
            np.testing.assert_allclose([r['ee_position'] for r in rows[label]],arrays['position'],atol=1e-5,rtol=0)
        changes=[];target={tuple(sorted(p)) for p in json.loads(profiles['refined'].canonical_json)['target_pairs']}
        for i,(a,b) in enumerate(zip(rows['official'],rows['refined'])):
            x=_pairs(a);y=_pairs(b);removed=sorted(x-y);added=sorted(y-x)
            if a['environment_collision']!=b['environment_collision']:raise ValueError('full official environment guard changed')
            if any(p[0]!='self' or tuple(p[1:]) not in target for p in removed+added):raise ValueError('non-target collision classification changed')
            record={'scenario':scenario,'sample_index':i,'q':a['q'],'official_free':a['model_scope_collision_free'],'refined_free':b['model_scope_collision_free'],'removed_pairs':removed,'added_pairs':added,'official_rejected':not a['model_scope_collision_free'],'refined_rejected':not b['model_scope_collision_free']}
            comparisons.append(record)
            if removed or added:changes.append(i)
        a=np.array([r['model_scope_collision_free'] is True for r in rows['official']]);b=np.array([r['model_scope_collision_free'] is True for r in rows['refined']])
        summary[scenario]['paired']={'recovered_indices':np.flatnonzero(~a&b).tolist(),'new_rejected_indices':np.flatnonzero(a&~b).tolist(),'both_rejected_count':int((~a&~b).sum()),'both_candidate_count':int((a&b).sum()),'changed_pair_indices':changes}
        if scenario=='idc':
            masks={'official_candidate':a,'refined_candidate':b,'official_rejected':~a,'refined_rejected':~b,'environment_collision':np.array([r['environment_collision'] for r in rows['refined']]),'unknown_unverified':np.array([r['final_decision']=='UNVERIFIED' for r in rows['refined']])}
            view=prepare_view(arrays,metadata,source_prefix=str(prefix));fig=build_3d_figure(view)
            fig.data[0].update(name='Original Workspace',visible='legendonly')
            import plotly.graph_objects as go
            for name,mask,color in [('Official Model Rejected',~a,'#e11d48'),('Refined Model Rejected',~b,'#a855f7'),('Model-Scope Candidates (diagnostic)',b,'#16a34a'),('Environment Collision',masks['environment_collision'],'#f97316'),('Unknown / Unverified',masks['unknown_unverified'],'#64748b')]:
                xyz=arrays['position'][mask];fig.add_trace(go.Scatter3d(x=xyz[:,0],y=xyz[:,1],z=xyz[:,2],customdata=view.customdata[mask],hovertemplate=fig.data[0].hovertemplate,mode='markers',marker={'size':3,'color':color},name=f'{name}: {int(mask.sum())}'))
            fig.update_layout(title='D1 same q, same ACM v1: pair-local geometry diagnostic; full official IDC guard',meta={'geometry_identities':identities[scenario],'sample_source':str(prefix),'hardware_validated':False})
            (out/'figures').mkdir()
            with (out/'figures/profile_comparison.html').open('x') as f:f.write(standalone_html(fig))
    np.savez(out/'workspace_results.npz',**arrays,**masks)
    with (out/'comparison.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(comparisons[0]));writer.writeheader()
        for row in comparisons:writer.writerow({k:json.dumps(v) if isinstance(v,(list,dict)) else v for k,v in row.items()})
    source_hashes={str(p.relative_to(root)):file_sha256(p) for base in ['src/workspace_analysis/collision','src/workspace_analysis/experiments','cpp/collision_backend','config/collision/geometry_profiles'] for p in sorted((root/base).rglob('*')) if p.is_file() and p.suffix in ['.py','.cpp','.hpp','.yaml']}
    manifest={'stage':'D1','created_utc':datetime.now(timezone.utc).isoformat(),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'source_sha256':source_hashes,'input_prefix':str(Path(prefix).absolute()),'input_sha256':{s:file_sha256(str(prefix)+s) for s in ['.json','.npz']},'identities':identities,'profiles':{k:json.loads(v.canonical_json) for k,v in profiles.items()},'gripper_rad':gripper,'clearance':{'self_m':0,'environment_m':0},'review_status':'diagnostic_only','hardware_validated':False,'original_indices_retained':True,'negative_z_retained':int((arrays['position'][:,2]<0).sum())}
    for name,value in [('summary.json',summary),('manifest.json',manifest)]:
        with (out/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
    (out/'.incomplete').unlink();return summary
