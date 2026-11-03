"""D0 reproducible pilot, consuming G2 pairs and the reusable collision evaluator."""
from collections import Counter
from datetime import datetime,timezone
import csv
import json
import os
from pathlib import Path
import subprocess
import uuid
import numpy as np
from ..collision.evaluator import Evaluator
from ..collision.policy import load_policy
from ..robot.model import load_model
from ..scene.schema import load_scene
from ..workspace.results import load_results,file_sha256
from ..viz.workspace import prepare_view,build_3d_figure,standalone_html
from .workspace_validation import VoxelGrid,voxel_hits


def summarize(rows):
    known=[r for r in rows if r['verification_status']!='ERROR'];n=len(rows)
    self_count=sum(r['effective_self_collision'] for r in known);env=sum(r['environment_collision'] for r in known)
    both=sum(r['effective_self_collision'] and r['environment_collision'] for r in known)
    union=self_count+env-both;free=sum(r['model_scope_collision_free'] is True for r in rows)
    decisions={k:sum(r['final_decision']==k for r in rows) for k in ['CLEAR','COLLIDING','UNVERIFIED','ERROR']}
    if sum(decisions.values())!=n:raise ValueError('decision conservation failure')
    timing=[r['query_time'] for r in rows]
    return {'total':n,'raw_self':sum(r['raw_self_collision'] for r in known),'effective_self':self_count,'environment':env,'both':both,
            'model_scope_collision_free':free,'decisions':decisions,'collision_rejection_ratio':union/n if n else None,
            'model_scope_feasible_ratio':free/n if n else None,'known_results':len(known),
            'mean_query_s':float(np.mean(timing)) if timing else None,'p95_query_s':float(np.percentile(timing,95)) if timing else None,
            'blocking_pairs':[{'kind':k[0],'links':list(k[1:]),'count':v} for k,v in Counter((p['kind'],*sorted(p['links'])) for r in known for p in r['remaining_collision_pairs']).most_common()],
            'scope':'configuration-level current URDF/ACM/gripper/clearance; not physical safety or RJ45 success probability'}


def run_pilot(root,backend,prefix,output,*,gripper=.3,policies=('baseline','assembly_policy_v0','assembly_policy_v1'),batch_size=64,clearance=None,geometry_profile=None,scene_snapshot=None):
    root=Path(root);out=Path(output)
    if os.path.lexists(out):raise ValueError('refusing existing output directory')
    model=load_model(root/'config/workspace.yaml');scene=scene_snapshot if scene_snapshot is not None else load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    arrays,metadata=load_results(prefix,model=model)
    source_hashes={suffix:file_sha256(str(prefix)+suffix) for suffix in ['.npz','.json']}
    out.parent.mkdir(parents=True,exist_ok=True);out.mkdir();(out/'.incomplete').write_text('D0 in progress')
    (out/'figures').mkdir();summaries={};ids={};all_rows={}
    git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
    for label in policies:
        policy=load_policy(root/f'config/collision/{label}.yaml',model)
        with Evaluator(model,scene,policy,backend,gripper=gripper,clearance=clearance,**({'geometry_profile':geometry_profile} if geometry_profile is not None else {})) as evaluator:
            rows=evaluator.evaluate(arrays['q'],batch_size=batch_size);ids[label]=evaluator.identity
        for i,row in enumerate(rows):
            if row['ee_position'] is not None and not np.allclose(row['ee_position'],arrays['position'][i],atol=1e-5,rtol=0):raise ValueError('G2 EE correspondence failure')
        summaries[label]=summarize(rows);all_rows[label]=rows
        good=np.array([r['model_scope_collision_free'] is True for r in rows])
        self_mask=np.array([r['effective_self_collision'] is True for r in rows]);env=np.array([r['environment_collision'] is True for r in rows]);unknown=np.array([r['final_decision'] in ['UNVERIFIED','ERROR'] for r in rows])
        np.savez(out/f'{label}_workspace_results.npz',**arrays,model_scope_free=good,self_rejected=self_mask,environment_rejected=env,unknown_unverified=unknown)
        view=prepare_view(arrays,metadata,source_prefix=str(prefix));fig=build_3d_figure(view)
        fig.data[0].update(visible='legendonly',name='Original Workspace (all configurations)')
        import plotly.graph_objects as go
        for name,mask,color in [('Model-scope candidates',good,'#16a34a'),('Self rejected',self_mask,'#dc2626'),('Environment rejected',env,'#f97316'),('Unknown / unverified',unknown,'#64748b')]:
            positions=arrays['position'][mask]
            fig.add_trace(go.Scatter3d(x=positions[:,0],y=positions[:,1],z=positions[:,2],mode='markers',name=f'{name}: {int(mask.sum())}',marker={'size':3,'color':color},customdata=view.customdata[mask],hovertemplate=fig.data[0].hovertemplate))
        fig.update_layout(title=f'D0 {label}, N={len(rows)}, gripper={gripper}: model-only; no hardware safety claim',meta={'evaluation_identity':ids[label],'summary':summaries[label]})
        fig.layout.annotations=()
        fig.add_annotation(text='Configuration categories overlap (self/environment); no XYZ deduplication. Candidates remain physically unverified.',xref='paper',yref='paper',x=0,y=-.1,showarrow=False)
        with (out/'figures'/f'{label}.html').open('x') as f:f.write(standalone_html(fig))
        summaries[label]['hit_voxels_all']=voxel_hits(arrays['position'],VoxelGrid())['hit_voxels']
        summaries[label]['hit_voxels_candidates']=voxel_hits(arrays['position'][good],VoxelGrid())['hit_voxels'] if good.any() else 0
        if not evaluator.client.cleanup['exit_normal']:summaries[label]['backend_exit_error']=evaluator.client.cleanup
    for suffix,digest in source_hashes.items():
        if file_sha256(str(prefix)+suffix)!=digest:raise ValueError('G2 source mutated')
    with (out/'per_sample.csv').open('x',newline='') as f:
        fields=['policy','sample_index','q','ee_position','ee_orientation','raw_self_collision','effective_self_collision','environment_collision','model_scope_collision_free','verification_status','final_decision','query_time','clearance','allowed_pairs','remaining_collision_pairs','identity','error'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for label,rows in all_rows.items():
            for row in rows:writer.writerow({'policy':label,**{k:json.dumps(row[k],separators=(',',':')) if isinstance(row.get(k),(list,dict)) else row.get(k) for k in fields if k!='policy'}})
    sources=['src/workspace_analysis/collision/evaluator.py','src/workspace_analysis/experiments/constrained_workspace.py','scripts/evaluate_workspace.py']
    for name,value in [('summary.json',summaries),('evaluations.json',all_rows),('manifest.json',{'stage':'D0','git_head':git_head,'git_status':subprocess.check_output(['git','status','--short'],cwd=root,text=True),'source_sha256':{s:file_sha256(root/s) for s in sources},'created_utc':datetime.now(timezone.utc).isoformat(),'input_prefix':str(prefix),'input_sha256':source_hashes,'identities':ids,'hardware_validated':False,'negative_z_retained':int((arrays['position'][:,2]<0).sum()),'configurations':len(arrays['q'])})]:
        with (out/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
    (out/'.incomplete').unlink();return summaries
