"""Small actual FCL comparison and source evidence for two joint-interface pairs.

No workspace experiment, candidate regeneration, hardware control or alternate FK.
"""
import ast
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from ..robot.model import load_model
from ..scene.schema import load_scene
from ..workspace.results import load_results
from ..collision.semantic import SemanticConfig
from ..collision.evaluator import Evaluator
from ..collision.assets import read_stl,robot_surfaces
from ..collision.protocol import mesh_resource_path
from ..collision.geometry_audit import topology,surface_topology
from ..viz.collision import build_collision_figure,export_collision_viewer


def _hash(path):return sha256(Path(path).read_bytes()).hexdigest()


def _write(path,value):
    with Path(path).open('x') as f:json.dump(value,f,indent=2,allow_nan=False)


def audit_system_source(model,source):
    """Read Python literals/XML/YAML only; never import vendor node or .so code.

    DH/controller pulse comparison is source-convention evidence, not a calibrated
    mapping to the real robot or authorization to replace URDF limits.
    """
    source=Path(source);transform=source/'driver/kinematics/kinematics/transform.py'
    literals={}
    for node in ast.parse(transform.read_text()).body:
        if isinstance(node,ast.Assign):
            for target in node.targets:
                if isinstance(target,ast.Name) and target.id in {n for i in range(1,6) for n in [f'joint{i}',f'joint{i}_map']}:
                    literals[target.id]=ast.literal_eval(node.value)
    servo=source/'driver/servo_controller/config/servo_controller.yaml'
    config=yaml.safe_load(servo.read_text())['/**']['ros__parameters'];conventions=[]
    for i in range(1,6):
        name=f'joint{i}';mapping=literals[name+'_map'];cfg=config[name];values=[]
        for pulse in [0,500,1000]:
            dh=(pulse-mapping[2])/(mapping[1]-mapping[0])*(mapping[4]-mapping[3])+mapping[5]
            publisher=((cfg['init']-pulse) if cfg['min']>cfg['max'] else (pulse-cfg['init']))*240/1000
            values.append({'pulse':pulse,'dh_degrees':dh,'controller_jointstate_degrees':publisher})
        conventions.append({'joint':name,'dh_range_degrees':literals[name],'urdf_limits_rad':[model.joints[name].lower,model.joints[name].upper],
                            'source_mapping':values,'limits_replaced':False,'physical_calibration_verified':False})
    files=[transform,servo,source/'driver/servo_controller/servo_controller/controller_manager.py',source/'driver/servo_controller/servo_controller/joint_position_controller.py',source/'driver/controller/config/init_pose.yaml',source/'driver/controller/controller/init_pose.py']
    for name in ['connect.urdf.xacro','arm.urdf.xacro','common.urdf.xacro','gripper.urdf.xacro']:files.append(source/'simulations/jetrover_description/urdf'/name)
    focused=[]
    for a,b in [('back_shell_green_link','link1'),('link2','servo_link1'),('link2','link3')]:
        relation=topology(model,a,b);joints=[]
        for name in relation['joint_path']:
            j=model.joints[name];joints.append({'name':name,'type':j.kind,'parent':j.parent,'child':j.child,'origin_xyz_m':j.xyz,'origin_rpy_rad':j.rpy,'axis':j.axis,'limits_rad':[j.lower,j.upper]})
        focused.append({'links':[a,b],'relationship':relation,'joints':joints,'whole_pair_mechanical_approval':'PENDING','candidate_baseline_is_physical_approval':False})
    meshes=[]
    for link in ET.fromstring(model.urdf_xml).findall('link'):
        if link.get('name') not in {'back_shell_green_link','link1','link2','servo_link1'}:continue
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh');path=mesh_resource_path(mesh.get('filename'));v,f,d=read_stl(path);result=surface_topology(v,f)
            meshes.append({'link':link.get('name'),'uri':mesh.get('filename'),'resource_sha256':d,'scale':mesh.get('scale','1 1 1'),'triangles':len(f),
                           'boundary_edges_exact_coordinate_weld':result['boundary_edges'],'nonmanifold_edges':result['nonmanifold_edges'],'degenerate_triangles':result['degenerate_triangles'],
                           'closed_edge_incidence_test':result['boundary_edges']==0 and result['nonmanifold_edges']==0,
                           'volume_validated':False,'note':'STL edge incidence only; no repair, self-intersection, volume or physical-contact inference'})
    return {'source_files_sha256':{str(p):_hash(p) for p in files},'joint_interfaces':focused,'mesh_topology':meshes,'angle_conventions':conventions,
            'findings':['URDF and controller code provide transforms, not permissible contact surfaces or a reviewed collision matrix.',
                        'DH q1/q5 have opposite pulse slope to controller JointState; DH q2/q4 have -90 degree offsets; q3 matches the source pulse convention.',
                        'Do not copy DH limits into URDF coordinates without explicit mapping, physical stops and calibration.',
                        'Changing contact with angle does not by itself establish dangerous interference.',
                        'Open triangle surfaces do not provide a validated physical solid or reliable mechanical penetration depth.'],
            'normal_reference_status':'PENDING','physical_model_validation':'PARTIAL'}


def compare_assembly_semantics(root,backend,out,*,candidate_srdf,source,allow_candidate=False,gripper=.3):
    """Query only diagnostic zero and unchanged G2 sample8/19/73 with two SRDFs."""
    root=Path(root);out=Path(out);model=load_model(root/'config/workspace.yaml');scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    strict=SemanticConfig.load(root/'config/collision/semantic/jetrover_semantic_v0.srdf',model)
    candidate=SemanticConfig.load(candidate_srdf,model,allow_candidate=allow_candidate)
    if candidate.review_status!='candidate_baseline_unverified':raise ValueError('explicit candidate baseline required')
    arrays,meta=load_results(root/'results/workspace_n100_seed42',model=model)
    out.mkdir(parents=True,exist_ok=False);(out/'figures').mkdir()
    _write(out/'system_source_audit.json',audit_system_source(model,source));_write(out/'started.json',{'status':'STARTED','workspace_resampled':False})
    records={};native={};identities={};cleanups={}
    cases=[('diagnostic_zero_NOT_NORMAL_REFERENCE',np.zeros(5),None),('sample8',arrays['q'][8],8),('sample19',arrays['q'][19],19),('sample73',arrays['q'][73],73)]
    for label,sem in [('strict_v0',strict),('standard_candidate_v0',candidate)]:
        records[label]={};native[label]={}
        with Evaluator(model,scene,None,backend,semantic=sem,gripper=gripper) as e:
            identities[label]=e.identity;_write(out/(label+'_ready.json'),e.ready)
            for name,q,index in cases:
                row=e.evaluate(np.asarray(q).reshape(1,5),start_index=0 if index is None else index)[0]
                if 'error' in row:raise ValueError(row['error'])
                row['case_label']=name;row['original_sample_index']=index
                records[label][name]=row;r=dict(e.last_native);r.update(q=q.tolist(),sample_index=name)
                native[label][name]=r
                if label=='standard_candidate_v0':
                    fig=build_collision_figure(scene,robot_surfaces(model,e.ready,r),r);export_collision_viewer(fig,r,out/'figures'/(name+'.html'))
        cleanups[label]=e.client.cleanup
    comparisons=[]
    disabled=set(tuple(p) for p in candidate.wire()['semantic']['disabled_pairs'])
    for name,q,index in cases:
        a=native['strict_v0'][name];b=native['standard_candidate_v0'][name]
        for key in ['raw_self_collision','environment_collision','colliding_pairs','link_transforms_base','joint_positions_used','T_base_to_ee']:
            if a[key]!=b[key]:raise ValueError('raw geometry/state regression: '+key)
        if index in [8,73] and not b['environment_collision']:raise ValueError('environment positive lost')
        if index==19 and ['back_shell_black_link','link3'] not in b['semantic_blocking_pairs']:raise ValueError('sample19 positive lost')
        before={tuple(p) for p in a['semantic_blocking_pairs']};after={tuple(p) for p in b['semantic_blocking_pairs']}
        removed=before-after;added=after-before
        if not removed<=disabled or added:raise ValueError('unexplained semantic pair change')
        comparisons.append({'case':name,'original_sample_index':index,'q':q.tolist(),'raw_self_collision':b['raw_self_collision'],'strict_semantic_self_collision':a['semantic_self_collision'],'candidate_semantic_self_collision':b['semantic_self_collision'],
            'environment_collision':b['environment_collision'],'strict_blocking_count':len(before),'candidate_blocking_count':len(after),'removed_pairs':[list(p) for p in sorted(removed)],'remaining_pairs':[list(p) for p in sorted(after)],
            'configuration_decision':records['standard_candidate_v0'][name]['configuration_decision'],'normal_reference':False})
    _write(out/'evaluations.json',records);_write(out/'native_queries.json',native);_write(out/'comparison.json',comparisons)
    summary={'scope':'FOUR_DIAGNOSTIC_CASES_ONLY_NOT_WORKSPACE_STATISTICS','comparisons':comparisons,'identities':identities,'cleanup':cleanups,'normal_reference_status':'PENDING','formal_srdf_approval':'NOT_CHANGED','whole_pair_physical_review':'PENDING','hardware_validated':False}
    _write(out/'summary.json',summary)
    sources={str(p.relative_to(root)):_hash(p) for parent in ['src','scripts','cpp','config'] for p in (root/parent).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    manifest={'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'identities':identities,'source_sha256':sources,
              'gripper_rad':gripper,'clearance':{'self_m':0,'environment_m':0},'input_hashes':{ext:_hash(root/('results/workspace_n100_seed42'+ext)) for ext in ['.npz','.json']},
              'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()},'status':'COMPLETED_RESEARCH_BASELINE_ONLY'}
    _write(out/'manifest.json',manifest)
    return summary
