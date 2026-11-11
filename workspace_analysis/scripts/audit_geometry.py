#!/usr/bin/env python3
"""C2.7 bounded native contact audit, holdouts, gripper scans and paired viewers."""
import argparse,json,sys,uuid,subprocess
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',type=Path,required=True);p.add_argument('--output-dir',type=Path);p.add_argument('--profile',choices=['official_full','refined_assembly_v0'],default='refined_assembly_v0');p.add_argument('--q',type=float,nargs=5,help='Only query this q instead of full scan');p.add_argument('--gripper',type=float,default=.3);p.add_argument('--contacts-per-pair',type=int,default=32)
    a=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.collision.geometry_profile import load_geometry_profile,effective_render_data
    from workspace_analysis.collision.geometry_audit import FOCUS,contact_evidence,topology,surface_topology,joint_frame
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.policy import load_policy
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    from workspace_analysis.viz.geometry_audit import local_pair_figure
    from workspace_analysis.viz.workspace import standalone_html
    from workspace_analysis.workspace.results import load_results,file_sha256
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');arrays,_=load_results(ROOT/'results/workspace_n100_seed42',model=m)
    out=a.output_dir or ROOT/'results/c2_7'/uuid.uuid4().hex;out.mkdir(parents=True,exist_ok=False);(out/'.incomplete').write_text('C2.7 in progress');(out/'figures').mkdir()
    profile=load_geometry_profile(ROOT/f'config/collision/geometry_profiles/{a.profile}.yaml',m,**({'asset_dir':out/'geometry_assets'} if a.profile!='official_full' else {}))
    designs=[('reference',np.zeros(5).tolist(),a.gripper)]
    if a.q:designs=[('single_query',a.q,a.gripper)]
    else:
        for j in range(3):
            for angle in np.linspace(*m.limits[j],17):
                q=np.zeros(5);q[j]=angle;designs.append(('joint'+str(j+1),q.tolist(),a.gripper))
        for opening in [m.joints['r_joint'].lower,-.8,-.4,0,.3,.6,.9,1.3,m.joints['r_joint'].upper]:
            for index in [None,8,73]:designs.append(('gripper_'+str(index),([0]*5 if index is None else arrays['q'][index].tolist()),opening))
        for index in [8,73,19,31,56,90]:designs.append(('heldout_'+str(index),arrays['q'][index].tolist(),a.gripper))
    records=[];summaries={};audit={'max_contacts_per_pair':a.contacts_per_pair,'max_contacts':20000,'distance_pairs':[[x,y] for x,y,_ in FOCUS]}
    with CollisionClient(a.backend,timeout=60) as client:
        ready=client.initialize(m,s,geometry_profile=profile);client.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m))
        for i,(group,q,gripper) in enumerate(designs):
            row=client.query(m,q,auxiliary_positions={'r_joint':gripper},geometry_audit=audit);row.update(q=q,sample_index=i)
            record={'group':group,'q':q,'r_joint':gripper,'query':row,'pairs':[]}
            for first,second,joint in FOCUS:
                original={**row,'contacts_base':row.get('official_contacts_base',row['contacts_base']),'pair_geometry_distances':row.get('official_pair_geometry_distances',row.get('pair_geometry_distances',[]))}
                record['pairs'].append({'links':[first,second],'official':contact_evidence(m,original,first,second,joint),'refined':contact_evidence(m,row,first,second,joint)})
            records.append(record)
            if group in ['reference','single_query','heldout_8','heldout_73','heldout_19']:
                surfaces=robot_surfaces(m,ready,row);derived=effective_render_data(m,ready,row)
                fig=build_collision_figure(s,surfaces,row);fig.update_layout(title=f'C2.7 {a.profile}: {group}; official full environment surfaces; q={q}')
                export_collision_viewer(fig,row,out/'figures'/f'{group}_robot.html')
                if group in ['reference','single_query']:
                    for first,second,joint in FOCUS:
                        fig=local_pair_figure(row,surfaces,derived,first,second,joint,joint_frame(m,row,joint),m.joints[joint].axis)
                        with (out/'figures'/f'local_{first}_{second}.html').open('x') as f:f.write(standalone_html(fig))
                    inv={name:np.linalg.inv(t) for name,t in row['link_transforms_base'].items()}
                    for surface in surfaces:
                        name=surface['id'];t=inv[name];points=surface['vertices']@t[:3,:3].T+t[:3,3]
                        summaries[name]=surface_topology(points,surface['faces'])
    results={'scope':'diagnostic only','pairs':[],'geometry_topology':summaries}
    for first,second,joint in FOCUS:
        index=next(i for i,pair in enumerate(records[0]['pairs']) if pair['links']==[first,second]);values=[r['pairs'][index] for r in records]
        results['pairs'].append({'links':[first,second],**topology(m,first,second),'joint_anchor':joint,'queries':len(records),'official_contact_states':sum(v['official']['contact_count']>0 for v in values),'refined_contact_states':sum(v['refined']['contact_count']>0 for v in values),'local_contact_summaries':[{'group':r['group'],'q':r['q'],'r_joint':r['r_joint'],'official':v['official'],'refined':v['refined']} for r,v in zip(records,values)],'physical_classification':'unresolved articulated / closed-loop assembly; local diagnostic exclusion is not physical proof'})
    if a.q is None:
        from plotly.subplots import make_subplots
        import plotly.graph_objects as go
        fig=make_subplots(rows=5,cols=2,subplot_titles=[x+' / '+y for x,y,_ in FOCUS])
        for index,(first,second,joint) in enumerate(FOCUS):
            group=joint if joint in ['joint1','joint2','joint3'] else 'gripper_None'
            subset=[r for r in records if r['group']==group]
            x=[r['r_joint'] if group.startswith('gripper') else r['q'][int(joint[-1])-1] for r in subset]
            for variant,color in [('official','#ef4444'),('refined','#0284c7')]:
                values=[r['pairs'][index][variant] for r in subset]
                fig.add_trace(go.Scatter(x=x,y=[v['contact_count'] for v in values],customdata=[v['depth_range_m'] or [None,None] for v in values],mode='lines+markers',name=variant,legendgroup=variant,showlegend=index==0,line={'color':color},hovertemplate='angle=%{x} rad<br>returned contacts=%{y}<br>depth min/max=%{customdata} m<extra></extra>'),row=index//2+1,col=index%2+1)
        fig.update_layout(height=1800,title='Bounded actual FCL contact counts by angle; counts are not contact area or complete manifold')
        with (out/'figures/joint_gripper_comparison.html').open('x') as f:f.write(standalone_html(fig))
    manifest={'stage':'C2.7','created_utc':datetime.now(timezone.utc).isoformat(),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'identity':client.identity,'profile_sha256':profile.sha256,'profile':json.loads(profile.canonical_json),'policy':'assembly_policy_v1','contact_limits':audit,'source_sha256':{str(f.relative_to(ROOT)):file_sha256(f) for folder in ['cpp/collision_backend','src/workspace_analysis/collision','src/workspace_analysis/viz','scripts'] for f in sorted((ROOT/folder).rglob('*')) if f.suffix in ['.py','.cpp','.hpp']},'input_g2_sha256':{s:file_sha256(str(ROOT/'results/workspace_n100_seed42')+s) for s in ['.json','.npz']},'hardware_validated':False}
    for name,value in [('joint_gripper_probes.json',records),('contact_region_audit.json',results),('geometry_refinement.json',json.loads(profile.canonical_json)),('manifest.json',manifest)]:
        with (out/name).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)
    (out/'.incomplete').unlink();print(out)


if __name__=='__main__':main()
