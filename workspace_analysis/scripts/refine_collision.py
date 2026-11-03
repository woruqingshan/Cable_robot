#!/usr/bin/env python3
"""Directed joint/gripper evidence, independent of random-sample acceptance ratio."""
import argparse,json,sys,uuid,subprocess,hashlib
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--backend',type=Path,required=True);p.add_argument('--output-dir',type=Path)
    a=p.parse_args()
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.policy import load_policy
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    from workspace_analysis.workspace.results import load_results
    model=load_model(ROOT/'config/workspace.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    out=a.output_dir or ROOT/'results/c2_6'/uuid.uuid4().hex;out.parent.mkdir(parents=True,exist_ok=True);out.mkdir();(out/'figures').mkdir()
    (out/'.incomplete').write_text('refinement in progress')
    joints=[];grippers=[];directed=[]
    arrays,_=load_results(ROOT/'results/workspace_n100_seed42',model=model)
    with CollisionClient(a.backend) as client:
        ready=client.initialize(model,scene);client.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v0.yaml',model))
        for name in ['joint1','joint2','joint3']:
            spec=model.joints[name]
            for angle in np.linspace(spec.lower,spec.upper,17):
                q=np.zeros(5);q[list(model.joint_order).index(name)]=angle
                row=client.query(model,q,auxiliary_positions={'r_joint':.3})
                origin=np.eye(4);origin[:3,:3]=Rotation.from_euler('xyz',spec.rpy).as_matrix();origin[:3,3]=spec.xyz
                base_joint=np.array(row['link_transforms_base'][spec.parent])@origin
                for c in row['contacts_base']:
                    c['position_joint_frame_m']=(np.linalg.inv(base_joint)@np.array([*c['position_base_m'],1]))[:3].tolist()
                joints.append({'joint':name,'angle':float(angle),'q':q.tolist(),'gripper':.3,'query':row})
                if name=='joint3' and angle in [spec.lower,spec.upper]:
                    view={**row,'sample_index':name+'_'+str(angle),'q':q.tolist()}
                    export_collision_viewer(build_collision_figure(scene,robot_surfaces(model,ready,row),view),view,out/'figures'/f'joint3_{angle}.html')
        spec=model.joints['r_joint']
        for angle in sorted(set([spec.lower,-.8,0,.3,.8,spec.upper])):
            for label,q in [('zero',np.zeros(5)),('sample8',arrays['q'][8]),('sample73',arrays['q'][73])]:
                row=client.query(model,q,auxiliary_positions={'r_joint':angle});grippers.append({'r_joint':angle,'arm_reference':label,'q':q.tolist(),'query':row})
        # A declared joint2/joint3 grid probes alternatives without changing ACM.
        client.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',model))
        for a2 in np.linspace(*model.limits[1],9):
            for a3 in np.linspace(*model.limits[2],9):
                q=[0,float(a2),float(a3),0,0];row=client.query(model,q,auxiliary_positions={'r_joint':.3})
                directed.append({'q':q,'query':row})
        fixed_evidence=[]
        for pair in [('back_shell_green_link','base_link'),('link4','servo_link2')]:
            matrices=[];depths=[];points=[]
            for r in joints:
                transforms=r['query']['link_transforms_base'];matrices.append(np.linalg.inv(transforms[pair[0]])@np.array(transforms[pair[1]]))
                for c in r['query']['contacts_base']:
                    if {c['first'],c['second']}==set(pair):depths.append(c['depth_m']);points.append(c['position_base_m'])
            fixed_evidence.append({'links':pair,'relative_transform':matrices[0].tolist(),'max_transform_delta':float(np.max(np.abs(np.array(matrices)-matrices[0]))),'contact_count':len(depths),'max_depth_m':max(depths,default=None),'contact_points_base_m':points})
        identity=client.identity
    import plotly.graph_objects as go
    from workspace_analysis.viz.workspace import standalone_html
    fig=go.Figure()
    for joint,pair in [('joint2',{'link2','servo_link1'}),('joint3',{'link2','link3'}),('joint1',{'back_shell_green_link','link1'})]:
        rows=[r for r in joints if r['joint']==joint]
        fig.add_trace(go.Scatter(x=[r['angle'] for r in rows],y=[int(any({c['first'],c['second']}==pair for c in r['query']['contacts_base'])) for r in rows],mode='lines+markers',name=joint+' '+str(sorted(pair))))
    fig.update_layout(title='Directed joint scans: actual Raw FCL contacts, gripper=0.3 rad',xaxis_title='joint angle [rad]',yaxis_title='contact boolean (not distance)')
    with (out/'figures/joint_scans.html').open('x') as f:f.write(standalone_html(fig))
    # Existing C2.5 viewers remain authoritative renderers; export a real
    # rejected sample for comparison with each constrained-workspace plot.
    with CollisionClient(a.backend) as client:
        ready=client.initialize(model,scene);client.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',model))
        for i in [8,73]:
            row=client.query(model,arrays['q'][i],auxiliary_positions={'r_joint':.3});row.update(sample_index=i,q=arrays['q'][i].tolist())
            export_collision_viewer(build_collision_figure(scene,robot_surfaces(model,ready,row),row),row,out/'figures'/f'sample{i}_v1.html')
    manifest={'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'created_utc':datetime.now(timezone.utc).isoformat(),'identity':identity,'gripper_reference_rad':.3,'clearance_settings':{'self_m':0,'environment_m':0},'input_sha256':{s:hashlib.sha256(Path(str(ROOT/'results/workspace_n100_seed42')+s).read_bytes()).hexdigest() for s in ['.npz','.json']},'software_only':True}
    for name,value in [('manifest.json',manifest),('directed_search.json',directed),('joint_probes.json',joints),('gripper_probes.json',grippers),('pair_refinement.json',{'fixed_evidence':fixed_evidence,'added_fixed_pairs':[['back_shell_green_link','base_link'],['link4','servo_link2']],'evidence':'C2.5 invariant relative matrices, actual Mesh overlap/contact regions; fixed URDF paths; model-only internal rigid assembly exclusions','articulated_pairs':'not allowed: hinge-region interference may be configuration dependent','reference_gripper_rad':.3,'reference_reason':'predeclared legal nonzero model reference, not optimized for feasible ratio or calibrated physical opening'})]:
        with (out/name).open('x') as f:json.dump(value,f,indent=2)
    (out/'.incomplete').unlink();print(out)


if __name__=='__main__':main()
