"""C3 source audit, independent Raw/historical/SRDF comparisons and reused viewers.

No new q sampling. Runs without a trusted normal reference are diagnostic only;
10,000 inputs are refused until a separately reviewed normal-pose gate exists.
"""
from collections import Counter
from hashlib import sha256
import csv
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from ..robot.model import load_model
from ..scene.schema import load_scene
from ..workspace.results import load_results
from ..collision.semantic import SemanticConfig,generate_candidates,parse_srdf
from ..collision.policy import load_policy
from ..collision.client import CollisionClient
from ..collision.evaluator import Evaluator
from ..collision.geometry_audit import FOCUS,topology,contact_evidence
from ..collision.assets import robot_surfaces
from ..viz.collision import build_collision_figure,export_collision_viewer


def digest(path):return sha256(Path(path).read_bytes()).hexdigest()


def write_json(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)


def vendor_audit(model):
    """Discover in official tree, overlays, installed ROS and other local ROS sources.

    No files from trash are restored or treated as active vendor configuration.
    """
    roots=[Path('/home/siyuen/Desktop/code/cable_robot/system_source'),Path('/home/siyuen/Desktop/jetrover_sim_ws'),Path('/opt/ros/humble/share'),Path('/home/siyuen/Desktop/code/ros2_ws/src'),Path('/home/siyuen/Desktop/ros2_ws/src')]
    urdf=ET.fromstring(model.urdf_xml);links={l.get('name') for l in urdf.findall('link')};joints=set(model.joints);inventory={};found=[]
    for root in roots:
        paths=[]
        for parent,dirs,files in os.walk(root,followlinks=True):
            dirs[:]=[n for n in dirs if n not in {'.git','__pycache__','node_modules'}]
            for n in files:
                if '.srdf' in n or n in ['kinematics.yaml','joint_limits.yaml']:paths.append(str(Path(parent)/n))
        inventory[str(root)]=sorted(paths)
        for path in paths:
            p=Path(path)
            if p.suffix!='.srdf':continue
            v=ET.parse(p).getroot();pairs=[{**e.attrib,'unknown_links':sorted({e.get('link1'),e.get('link2')}-links),'adopted':False} for e in v.findall('disable_collisions')]
            unknown_links=sorted({e.get(k) for e in v.iter() for k in ['link1','link2','parent_link','child_link'] if e.get(k)}-links)
            group_joints=sorted({e.get('name') for g in v.findall('group') for e in g.findall('joint')}-joints)
            found.append({'path':path,'sha256':digest(p),'robot_name':v.get('name'),'robot_name_matches':v.get('name')==urdf.get('name'),'unknown_links':unknown_links,'unknown_group_joints':group_joints,'groups':[{**g.attrib,'members':[e.attrib for e in g]} for g in v.findall('group')],'named_states':[{**g.attrib,'values':[e.attrib for e in g]} for g in v.findall('group_state')],'disabled_pairs':pairs,'adopted':False,'decision':'REJECT_INCOMPATIBLE_MODEL' if unknown_links or group_joints or v.get('name')!=urdf.get('name') else 'REQUIRES_REVIEW'})
    return {'search_inventory':inventory,'current_robot_name':urdf.get('name'),'source_urdf_sha256':model.metadata()['expanded_urdf_sha256'],'candidates':found,'matching_approved_vendor_srdf':False,'normal_reference_status':'PENDING','note':'rosmentor configs reference landerpi_description and incompatible gripper/base links; no automatic renaming'}


def _summary(rows,kind):
    valid='model_configuration_valid' if kind=='C_semantic' else 'model_scope_collision_free'
    counts=Counter(tuple(p) for r in rows for p in (r.get('semantic_blocking_pairs',[]) if kind=='C_semantic' else [p['links'] for p in r['remaining_collision_pairs'] if p['kind']=='self']))
    clearance={}
    for name in ['self','environment']:
        values=[(r.get('semantic_clearance') if kind=='C_semantic' else r.get('clearance'))['native'][name]['separation_m'] for r in rows if r.get('clearance') is not None]
        values=[x for x in values if x is not None]
        clearance[name]={'available':len(values),'min_m':min(values) if values else None,'median_m':float(np.median(values)) if values else None,'max_m':max(values) if values else None}
    times=[r['query_time'] for r in rows]
    return {'total':len(rows),'raw_self_collision':sum(r['raw_self_collision'] is True for r in rows),'semantic_self_collision':sum(r.get('semantic_self_collision') is True for r in rows) if kind=='C_semantic' else None,'effective_self_collision':sum(r['effective_self_collision'] is True for r in rows),'environment_collision':sum(r['environment_collision'] is True for r in rows),'valid_configurations':sum(r.get(valid) is True for r in rows),'rejected_configurations':sum(r.get(valid) is False for r in rows),'unknown_or_error':sum(r.get(valid) is None for r in rows),'valid_ratio':sum(r.get(valid) is True for r in rows)/len(rows),'main_blocking_pairs':[{'links':list(p),'count':n} for p,n in counts.most_common()],'clearance_distribution':clearance,'query_latency_s':{'median':float(np.median(times)),'p95':float(np.quantile(times,.95)),'total':sum(times)}}


def run_semantic_study(root,backend,out,inputs=(),*,gripper=.3,candidate_trials=0):
    root=Path(root);out=Path(out)
    model=load_model(root/'config/workspace.yaml');scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    semantic=SemanticConfig.load(root/'config/collision/semantic/jetrover_semantic_v0.srdf',model)
    datasets=[]
    for prefix in inputs:
        a,meta=load_results(prefix,model=model)
        if len(a['q'])>1000:raise ValueError('C3 10,000 gate BLOCKED: no trusted normal reference pose; diagnostic limit is 1,000')
        datasets.append((Path(prefix),a,meta))
    out.parent.mkdir(parents=True,exist_ok=True);out.mkdir();(out/'figures').mkdir();write_json(out/'started.json',{'status':'INCOMPLETE'})
    source_hashes={str(p.relative_to(root)):digest(p) for name in ['src','scripts','cpp','config'] for p in (root/name).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    audit=vendor_audit(model);write_json(out/'srdf_source_audit.json',audit)
    # Known vendor startup action points to a missing /home/ubuntu action group.
    vendor=Path('/home/siyuen/Desktop/code/cable_robot/system_source/driver')
    reference={'normal_reference_status':'PENDING','normal_pose_verified':False,'reason':'No matching named state or confirmed recording. init_pose defaults to action init at /home/ubuntu/software/arm_pc/ActionGroups, unavailable locally. rosmentor home is incompatible. Servo fallback is not active default and joint3 pulse0 maps to +2.0943951 > URDF2.09. No clamping or invented qzero truth.','vendor_evidence':{str(p):digest(p) for p in [vendor/'controller/config/init_pose.yaml',vendor/'controller/controller/init_pose.py',vendor/'servo_controller/config/servo_controller.yaml',vendor/'servo_controller/servo_controller/joint_position_controller.py']},'qzero_role':'diagnostic default only; not known safe','ten_thousand_gate':'BLOCKED'}
    policy=load_policy(root/'config/collision/assembly_policy_v1.yaml',model);raw_policy=load_policy(root/'config/collision/baseline.yaml',model)
    arrays,_=load_results(root/'results/workspace_n100_seed42',model=model)
    selected={};regions=[]
    with CollisionClient(backend,timeout=60) as c:
        ready=c.initialize(model,scene,semantic=semantic);write_json(out/'native_ready.json',ready)
        # Actual standard MoveIt generator runs on isolated scene; recommendations never applied.
        # Candidate tool is isolated from the trusted pose-query session.
        candidate_client=CollisionClient(backend,timeout=120)
        try:
            candidate_client.initialize(model,scene,semantic=semantic)
            candidate_reply=candidate_client.candidates(trials=candidate_trials)
        except Exception as exc:
            candidate_reply={'status':'ERROR','error':str(exc),'recommendations_applied':False}
        finally:candidate_cleanup=candidate_client.close()
        write_json(out/'moveit_candidate_matrix.json',{'result':candidate_reply,'cleanup':candidate_cleanup})
        for index in [8,19,73]:
            q=arrays['q'][index];r=c.query(model,q,auxiliary_positions={'r_joint':gripper},geometry_audit={'max_contacts_per_pair':64,'max_contacts':20000})
            r.update(q=q.tolist(),sample_index=index);selected[str(index)]=r
            if index==19 and ['back_shell_black_link','link3'] not in r['semantic_blocking_pairs']:raise ValueError('Sample19 positive lost')
            if index in [8,73] and not r['environment_collision']:raise ValueError('environment positive lost')
        # Do not manufacture a safe reference: zero is explicitly diagnostic.
        default=c.query(model,[0]*5,auxiliary_positions={'r_joint':gripper},geometry_audit={'max_contacts_per_pair':64,'max_contacts':20000});default.update(q=[0]*5,sample_index='diagnostic_zero_NOT_SAFE_REFERENCE')
        selected['diagnostic_zero']=default
        # Held q and full legal gripper range probes use current URDF, not alternate FK.
        lower=model.joints['r_joint'].lower;upper=model.joints['r_joint'].upper
        for name,value in [('joint1',None),('joint2',None),('joint3',None),('r_joint',None)]:
            spec=model.joints[name]
            for angle in np.linspace(spec.lower,spec.upper,5):
                q=np.zeros(5);opening=gripper
                if name=='r_joint':opening=float(angle)
                else:q[list(model.joint_order).index(name)]=angle
                row=c.query(model,q,auxiliary_positions={'r_joint':opening},geometry_audit={'max_contacts_per_pair':64,'max_contacts':20000})
                regions.append({'joint':name,'angle':float(angle),'q':q.tolist(),'gripper':opening,'raw_self_collision':row['raw_self_collision'],'semantic_self_collision':row['semantic_self_collision'],'semantic_blocking_pairs':row['semantic_blocking_pairs'],'contact_collection':row['contact_collection'],'focus':[{'links':[a,b],'topology':topology(model,a,b),'evidence':contact_evidence(model,row,a,b,j)} for a,b,j in FOCUS]})
        for label,row in selected.items():
            fig=build_collision_figure(scene,robot_surfaces(model,ready,row),row);export_collision_viewer(fig,row,out/'figures'/f'{label}.html')
    reference.update(diagnostic_default={'raw_self_collision':default['raw_self_collision'],'semantic_self_collision':default['semantic_self_collision'],'remaining_blocking_pairs':default['semantic_blocking_pairs']},positive_examples={k:{'semantic_self_collision':r['semantic_self_collision'],'environment_collision':r['environment_collision'],'semantic_blocking_pairs':r['semantic_blocking_pairs']} for k,r in selected.items() if k!='diagnostic_zero'},cleanup=c.cleanup)
    write_json(out/'reference_pose_validation.json',reference);write_json(out/'selected_queries.json',selected);write_json(out/'joint_gripper_contact_audit.json',regions)
    reviews=[];allowed=set(tuple(p) for p in ready['audit']['semantic']['actual_disabled_pairs'])
    for a,b,j in FOCUS:
        reviews.append({'links':sorted([a,b]),'joint':j,'topology':topology(model,a,b),'srdf_disabled':tuple(sorted([a,b])) in allowed,'decision':'RETAIN_CHECKED_PENDING_PHYSICAL_ASSEMBLY_EVIDENCE','rationale':'Moving joint path and finite sampled contacts do not prove entire pair exemption. link2/link3 known state-dependent; gripper linkage closed-loop/calibration unverified. No whole-pair permission inferred from frequency.','contact_evidence':[{'sample_index':r['sample_index'],**contact_evidence(model,r,a,b,j)} for r in selected.values()]})
    write_json(out/'collision_pair_review.json',{'disabled_pair_reviews':json.loads(semantic.wire()['semantic']['review_canonical_json'])['pair_reviews'],'priority_pair_reviews':reviews,'status':'PARTIAL'})
    studies={}
    for prefix,a,meta in datasets:
        tag='n'+str(len(a['q']));directory=out/tag;directory.mkdir();rows={};identities={};cleanups={}
        for condition,p,sem in [('A_raw',raw_policy,None),('B_historical_v1',policy,None),('C_semantic',None,semantic)]:
            options={'gripper':gripper,'timeout':60}
            if sem is not None:options['semantic']=sem
            with Evaluator(model,scene,p,backend,**options) as e:
                values=e.evaluate(a['q']);identities[condition]=e.identity;rows[condition]=values
            cleanups[condition]=e.client.cleanup
            if any('error' in r for r in values):raise ValueError('query error in '+condition)
            for i,r in enumerate(values):
                if r['sample_index']!=i or r['q']!=a['q'][i].tolist():raise ValueError('index/q changed')
                if not np.allclose(r['ee_position'],a['position'][i],rtol=0,atol=1e-10) or not np.allclose(r['ee_orientation'],a['rotation'][i],rtol=0,atol=1e-10):raise ValueError('FK changed')
        comparisons=[]
        for i,(a_row,b_row,c_row) in enumerate(zip(rows['A_raw'],rows['B_historical_v1'],rows['C_semantic'])):
            if a_row['raw_self_collision']!=b_row['raw_self_collision'] or b_row['raw_self_collision']!=c_row['raw_self_collision'] or a_row['environment_collision']!=b_row['environment_collision'] or b_row['environment_collision']!=c_row['environment_collision']:raise ValueError('raw/environment regression')
            old_pairs=sorted(p['links'] for p in b_row['remaining_collision_pairs'] if p['kind']=='self')
            if c_row['semantic_blocking_pairs']!=old_pairs or c_row['semantic_self_collision']!=b_row['effective_self_collision']:raise ValueError('reviewed fixed SRDF differs from equivalent historical explicit matrix')
            comparisons.append({'sample_index':i,'raw_rejected':not a_row['model_scope_collision_free'],'historical_rejected':not b_row['model_scope_collision_free'],'semantic_rejected':not c_row['model_configuration_valid'],'historical_to_semantic_changed':b_row['model_scope_collision_free']!=c_row['model_configuration_valid'],'removed_by_srdf':b_row['allowed_pairs'],'remaining_semantic':c_row['semantic_blocking_pairs'],'environment_collision':c_row['environment_collision']})
        # Deterministic observations supplement standard random candidates without changing ACM.
        observed=[{'colliding_pairs':r['raw_collision_pairs'],'pairs_may_be_truncated':r['pairs_may_be_truncated']} for r in rows['C_semantic']]
        write_json(directory/'observed_pair_candidates.json',generate_candidates(model,observed,default))
        write_json(directory/'evaluations.json',rows);write_json(directory/'comparison.json',comparisons)
        with (directory/'comparison.csv').open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(comparisons[0]));writer.writeheader()
            for row in comparisons:writer.writerow({k:json.dumps(v) if isinstance(v,list) else v for k,v in row.items()})
        np.savez_compressed(directory/'workspace_results.npz',**a,sample_index=np.arange(len(a['q'])),semantic_valid=np.array([r['model_configuration_valid'] for r in rows['C_semantic']],dtype=bool))
        # Use original FK positions and masks, no synthetic points.
        import plotly.graph_objects as go
        fig=go.Figure();positions=a['position'];env=np.array([r['environment_collision'] for r in rows['C_semantic']]);valid=np.array([r['model_configuration_valid'] for r in rows['C_semantic']])
        masks=[('Original workspace',np.ones(len(positions),bool),'gray'),('Semantic rejected',~valid,'red'),('Model-only valid',valid,'green'),('Environment collision',env,'orange')]
        for name,mask,color in masks:
            pts=positions[mask];ids=np.arange(len(positions))[mask]
            fig.add_trace(go.Scatter3d(x=pts[:,0],y=pts[:,1],z=pts[:,2],mode='markers',marker={'size':3,'color':color},name=name,customdata=ids,hovertemplate='Original Sample Index %{customdata}<br>(%{x}, %{y}, %{z}) m<extra></extra>'))
        fig.update_layout(scene={'aspectmode':'data','xaxis_title':'X [m]','yaxis_title':'Y [m]','zaxis_title':'Z [m]'},title=f'C3 {tag} diagnostic — normal reference PENDING',meta={'identities':identities,'input_prefix':str(prefix),'normal_reference_status':'PENDING'})
        with (out/'figures'/f'{tag}_workspace.html').open('x') as f:f.write(fig.to_html(full_html=True,include_plotlyjs=True))
        summary={condition:_summary(v,condition) for condition,v in rows.items()}
        summary.update(identities=identities,cleanups=cleanups,classification='DIAGNOSTIC_ONLY_NORMAL_REFERENCE_PENDING',changed_sample_indices=[r['sample_index'] for r in comparisons if r['historical_to_semantic_changed']],paired_count=len(comparisons),input_prefix=str(prefix),input_hashes={ext:digest(prefix.with_suffix(ext)) for ext in ['.npz','.json']})
        write_json(directory/'summary.json',summary);studies[tag]=summary
    write_json(out/'summary.json',{'software_status':'IMPLEMENTED_TESTING_SEPARATELY_RECORDED','semantic_model_validation':'PARTIAL','normal_reference_pose':'PENDING','hardware_validation':'NOT_PERFORMED','workspace_readiness':'DIAGNOSTIC_ONLY','ten_thousand_gate':'BLOCKED','studies':studies})
    # Evidence files remain append-only; never unlink history, even incomplete marker.
    manifest={'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'identity':ready['identity'],'gripper_state_rad':gripper,'clearance':{'self_m':0,'environment_m':0},'source_sha256':source_hashes,'files':{str(p.relative_to(out)):digest(p) for p in out.rglob('*') if p.is_file()},'completion_status':'COMPLETED_DIAGNOSTIC_PARTIAL','normal_reference_status':'PENDING'}
    write_json(out/'manifest.json',manifest)
    return manifest


def verify_run(out,model):
    out=Path(out);manifest=json.loads((out/'manifest.json').read_text())
    for name,h in manifest['files'].items():
        if digest(out/name)!=h:raise ValueError('output hash changed: '+name)
    for path in out.glob('n*/summary.json'):
        summary=json.loads(path.read_text());arrays,_=load_results(summary['input_prefix'],model=model)
        for ext,h in summary['input_hashes'].items():
            if digest(Path(summary['input_prefix']).with_suffix(ext))!=h:raise ValueError('input changed')
        with np.load(path.parent/'workspace_results.npz',allow_pickle=False) as result:
            for name,value in arrays.items():np.testing.assert_array_equal(result[name],value)
            np.testing.assert_array_equal(result['sample_index'],np.arange(len(arrays['q'])))
    return {'artifact_hashes_verified':len(manifest['files']),'q_fk_arrays_unchanged':True,'normal_reference_status':manifest['normal_reference_status']}
