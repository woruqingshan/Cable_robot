#!/usr/bin/env python3
"""C2.8 official gripper assembly audit + real task-scope single-query viewers."""
import argparse,json,sys,uuid,subprocess
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
from itertools import combinations
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',type=Path,required=True);p.add_argument('--output-dir',type=Path);p.add_argument('--q',type=float,nargs=5);p.add_argument('--gripper',type=float,default=.3);a=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.collision.policy import load_policy
    from workspace_analysis.collision.scope import load_scope
    from workspace_analysis.collision.assembly import assembly_topology
    from workspace_analysis.collision.geometry_audit import contact_evidence,topology
    from workspace_analysis.collision.geometry_profile import load_geometry_profile
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.workspace.results import load_results,file_sha256
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    from workspace_analysis.viz.workspace import standalone_html
    import plotly.graph_objects as go
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');policy=load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m);scope=load_scope(ROOT/'config/collision/task_scope_v0.yaml',m,policy);definition=json.loads(scope[0]);tree=assembly_topology(m)
    arrays,_=load_results(ROOT/'results/workspace_n100_seed42',model=m);profile=load_geometry_profile(ROOT/'config/collision/geometry_profiles/official_full.yaml',m)
    out=a.output_dir or ROOT/'results/c2_8'/uuid.uuid4().hex;out.mkdir(parents=True,exist_ok=False);(out/'.incomplete').write_text('C2.8 audit');(out/'figures').mkdir()
    openings=[m.joints['r_joint'].lower,-1.,-.5,0.,.3,.6,1.,m.joints['r_joint'].upper]
    configurations=[('opening_'+str(x),[0]*5,x) for x in openings]+[('sample_'+str(i),arrays['q'][i].tolist(),a.gripper) for i in [8,19,73]]
    if a.q is not None:configurations=[('single',a.q,a.gripper)]
    distance_pairs=[d['links'] for d in definition['internal_diagnostic_pairs']]
    all_pairs=[list(p) for p in combinations(tree['members'],2)];declared={tuple(p) for p in distance_pairs};all_matrix=[]
    records=[];matrix=[]
    with CollisionClient(a.backend,timeout=60) as c:
        ready=c.initialize(m,s,geometry_profile=profile);c.set_policy(*policy)
        for label,q,opening in configurations:
            row=c.query(m,q,auxiliary_positions={'r_joint':opening},task_scope=scope,geometry_audit={'max_contacts_per_pair':32,'max_contacts':20000,'distance_pairs':all_pairs});row.update(q=q,sample_index=label)
            for spec in m.joints.values():
                if spec.mimic_joint:assert abs(row['joint_positions_used'][spec.name]-(row['joint_positions_used'][spec.mimic_joint]*spec.multiplier+spec.offset))<1e-12
            evidence=[]
            for pair in definition['internal_diagnostic_pairs']:
                joint=next(m.joints[name] for name in pair['urdf_joint_path'] if m.joints[name].kind=='revolute')
                evidence.append({**pair,'contact_evidence':contact_evidence(m,row,*pair['links'],joint.name)})
            all_evidence=[]
            for pair in all_pairs:
                relationship=topology(m,*pair);joint=next(m.joints[n] for n in relationship['joint_path'] if m.joints[n].kind=='revolute')
                all_evidence.append({'links':pair,**relationship,'declared_internal_diagnostic':tuple(pair) in declared,'classification':next((p['mechanism'] for p in definition['internal_diagnostic_pairs'] if p['links']==pair),'possible_mechanical_interference_or_unresolved_NOT_excluded'),'contact_evidence':contact_evidence(m,row,*pair,joint.name)})
            records.append({'label':label,'r_joint':opening,'query':row,'internal_evidence':evidence,'all_assembly_pair_evidence':all_evidence})
            all_matrix.append([0 if not e['contact_evidence']['pair_distance']['collision'] else 1 if e['declared_internal_diagnostic'] else 2 for e in all_evidence])
            actual={tuple(p) for p in row['task_scope']['gripper_internal_pairs']};matrix.append([int(tuple(p) in actual) for p in distance_pairs])
            fig=build_collision_figure(s,robot_surfaces(m,ready,row),row);export_collision_viewer(fig,row,out/'figures'/(label+'.html'))
            if label.startswith('opening_') or label=='single':
                members=set(tree['members'])
                # Reuse the same complete-mesh figure; change only visibility for an assembly close-up.
                for trace in fig.data:
                    role=(trace.meta or {}).get('role');name=(trace.meta or {}).get('object_id')
                    if role in ['robot_collision_mesh','robot_link_frame']:trace.visible=name in members
                    elif role not in ['scope_internal_contacts','scope_blocking_contacts','robot_link_frame']:trace.visible=False
                points=np.vstack([x['vertices'] for x in robot_surfaces(m,ready,row) if x['id'] in members]);lo=points.min(0)-.01;hi=points.max(0)+.01
                fig.update_layout(title=f'Assembly official Mesh — r_joint={opening} rad; internal mechanics UNVERIFIED',scene={'aspectmode':'data','xaxis':{'range':[lo[0],hi[0]]},'yaxis':{'range':[lo[1],hi[1]]},'zaxis':{'range':[lo[2],hi[2]]}})
                fig.layout.annotations=();fig.add_annotation(text='Assembly close-up only: official Mesh + native Link Frames. Remaining robot/IDC hidden for viewing, fully included in collision query. Purple is UNVERIFIED internal contact.',xref='paper',yref='paper',x=0,y=-.1,showarrow=False)
                with (out/'figures'/('assembly_'+label+'.html')).open('x') as f:f.write(standalone_html(fig))
    fig=go.Figure(go.Heatmap(z=matrix,x=[' / '.join(p) for p in distance_pairs],y=[r['label'] for r in records],zmin=0,zmax=1,colorscale=[[0,'#e2e8f0'],[1,'#a855f7']],hovertemplate='%{y}<br>%{x}<br>actual FCL contact=%{z}<extra></extra>'))
    fig.update_layout(title='Declared internal contacts: purple is diagnostic, not safety approval')
    with (out/'figures/internal_contact_matrix.html').open('x') as f:f.write(standalone_html(fig))
    fig=go.Figure(go.Heatmap(z=all_matrix,x=[' / '.join(p) for p in all_pairs],y=[r['label'] for r in records],zmin=0,zmax=2,colorscale=[[0,'#e2e8f0'],[.5,'#a855f7'],[1,'#ef4444']],colorbar={'tickvals':[0,1,2],'ticktext':['no contact','diagnostic UNVERIFIED','blocking undeclared pair']}))
    fig.update_layout(title='All 21 gripper pairs: only seven explicit pairs are diagnostic, all others still block')
    with (out/'figures/all_assembly_contact_matrix.html').open('x') as f:f.write(standalone_html(fig))
    manifest={'stage':'C2.8','created_utc':datetime.now(timezone.utc).isoformat(),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'identity':c.identity,'scope_sha256':scope[1],'scope':definition,'geometry':'official_full','gripper_openings_rad':openings,'clearance':{'self_m':0,'environment_m':0},'source_sha256':{str(f.relative_to(ROOT)):file_sha256(f) for folder in ['src/workspace_analysis/collision','src/workspace_analysis/viz','cpp/collision_backend','scripts','config/collision'] for f in (ROOT/folder).rglob('*') if f.suffix in ['.py','.cpp','.hpp','.yaml']},'hardware_validated':False}
    for name,value in [('assembly_topology.json',tree),('internal_contact_audit.json',records),('manifest.json',manifest)]:
        with (out/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
    (out/'.incomplete').unlink();print(out)
if __name__=='__main__':main()
