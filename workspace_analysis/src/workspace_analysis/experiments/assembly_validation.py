"""Read-only assembly evidence, recorded pose candidates and targeted FCL probes.

Kinematic relationships are confirmed from URDF. Whole-pair physical permissions
remain pending. No vendor node or binary kinematics module is imported.
"""
import ast
import json
import math
from pathlib import Path
import numpy as np
import yaml
from .assembly_semantics import audit_system_source,_hash
from ..collision.client import CollisionClient
from ..collision.geometry_audit import topology,contact_evidence


def _number_expression(node):
    """Evaluate numeric arithmetic only, for the vendor encoder coefficient."""
    if isinstance(node,ast.Constant) and type(node.value) in (float,int):return node.value
    if isinstance(node,ast.Attribute) and isinstance(node.value,ast.Name) and node.value.id=='math' and node.attr=='pi':return math.pi
    if isinstance(node,ast.UnaryOp) and isinstance(node.op,(ast.USub,ast.UAdd)):
        value=_number_expression(node.operand);return -value if isinstance(node.op,ast.USub) else value
    if isinstance(node,ast.BinOp):
        a,b=_number_expression(node.left),_number_expression(node.right)
        if isinstance(node.op,ast.Add):return a+b
        if isinstance(node.op,ast.Sub):return a-b
        if isinstance(node.op,ast.Mult):return a*b
        if isinstance(node.op,ast.Div):return a/b
    raise ValueError('unsupported vendor encoder arithmetic; explicit review required')


def recorded_pose_candidates(root,model,source):
    """Audit bounded examples from existing teaching files, without safety inference."""
    source=Path(source);root=Path(root)
    controller=source/'driver/servo_controller/servo_controller/joint_position_controller.py'
    coefficient=[]
    for node in ast.walk(ast.parse(controller.read_text())):
        if isinstance(node,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='RADIANS_PER_ENCODER_TICK' for t in node.targets):coefficient.append(_number_expression(node.value))
    if len(coefficient)!=1 or not math.isfinite(coefficient[0]) or coefficient[0]<=0:raise ValueError('vendor encoder coefficient unresolved')
    k=coefficient[0]
    config_path=source/'driver/servo_controller/config/servo_controller.yaml'
    config=yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
    def angle(name,pulse):
        c=config[name]
        if type(pulse) not in (int,float) or not math.isfinite(pulse) or not min(c['min'],c['max'])<=pulse<=max(c['min'],c['max']):raise ValueError('pulse outside configured encoder range')
        return ((c['init']-pulse) if c['min']>c['max'] else (pulse-c['init']))*k
    records=[];files={str(controller):_hash(controller),str(config_path):_hash(config_path)}
    for name in ['insert.yaml','pull_out.yaml']:
        path=root.parent/'cable_robot_control/config'/name
        if not path.exists():continue
        files[str(path)]=_hash(path);data=yaml.safe_load(path.read_text()).get('waypoints',{})
        choices=[]
        if 'CABLE_HOLD_READY' in data:choices.append('CABLE_HOLD_READY')
        timestamped=next((n for n,v in data.items() if 'timestamp' in v and n not in choices),None)
        if timestamped:choices.append(timestamped)
        for label in choices:
            value=data[label];row={'source_file':str(path),'source_sha256':files[str(path)],'name':label,
                'timestamp':value.get('timestamp'),'held_after_save':value.get('held_after_save'),
                'joint_pulse':value.get('joint_pulse'),'gripper_pulse':value.get('gripper_pulse'),
                'normal_reference_status':'PENDING','physical_calibration_verified':False,
                'reason':'Teaching record exists; timestamp/name/soft pulse checks are not independent collision/normal-pose confirmation. Stored pose frame/TCP not established.'}
            try:
                pulses=value['joint_pulse'];q=[angle(n,pulses[config[n]['id']]) for n in model.joint_order]
                gripper=angle('r_joint',value['gripper_pulse'][config['r_joint']['id']])
                model.joint_positions(q,{'r_joint':gripper})
                row.update(q_controller_jointstate_rad=q,gripper_rad=gripper,urdf_joint_limits_valid=True)
            except (ValueError,KeyError,TypeError) as exc:row.update(urdf_joint_limits_valid=False,conversion_error=str(exc))
            records.append(row)
    return {'status':'PENDING','coefficient_rad_per_tick':k,'source_sha256':files,'candidates':records,
            'hardware_control_executed':False,'selected_trusted_normal_pose':None,
            'scope':'Source-convention conversion only; no calibrated hardware-to-URDF assertion'}


def pair_review(model,strict,candidate):
    """Keep kinematic certainty separate from mechanical exemption certainty."""
    manifest=json.loads(candidate.wire()['semantic']['review_canonical_json'])
    strict_pairs={tuple(p) for p in strict.wire()['semantic']['disabled_pairs']}
    gripper={'gripper_link','l_link','r_link','l_in_link','r_in_link','l_out_link','r_out_link'}
    entries=[]
    for entry in manifest['pair_reviews']:
        a,b=entry['links'];relation=topology(model,a,b);moving=relation['moving_joints']
        mechanism=('fixed_relative_transform' if not moving else 'coupled_gripper_mechanism' if {a,b}<=gripper else 'single_moving_joint_interface' if len(moving)==1 else 'multi_joint_relation')
        risk=('HIGH_PRIORITY_STATE_DEPENDENT_JOINT_INTERFACE' if {a,b}=={'link2','link3'} else 'WHOLE_PAIR_EXEMPTION_NOT_PHYSICALLY_VERIFIED')
        entries.append({'links':[a,b],'candidate_class':entry['candidate_class'],'urdf_relationship':relation,
            'kinematic_relation_confirmed':True,'mechanism_class':mechanism,'strict_disabled':(a,b) in strict_pairs,
            'candidate_disabled':True,'existing_review_scope':'model_only_partial_fixed_assembly' if (a,b) in strict_pairs else 'unreviewed_mechanical_exemption',
            'whole_pair_physical_approval':'PENDING','risk':risk,
            'hidden_interference_cannot_be_ruled_out':True,'runtime_pair_observations':{}})
    return {'usage':'RESEARCH_ONLY','disabled_pair_count':len(entries),'selected_categories':['Adjacent','Default'],
            'rules_expanded':False,'formal_semantics_upgraded':False,'entries':entries,
            'interpretation':'Removal of raw contacts under candidate ACM is an explicit hypothesis, not a physical safety proof.'}


def validate_assembly(root,backend,out,source,model,scene,strict,candidate):
    """Bounded sweeps for joint1/2/3, positive examples and recorded source poses."""
    source_audit=audit_system_source(model,source)
    poses=recorded_pose_candidates(root,model,source)
    from ..workspace.results import load_results
    arrays,_=load_results(Path(root)/'results/workspace_n100_seed42',model=model)
    scans=[];positives=[];ready={};cleanup={};clients={}
    try:
        for name,config in [('strict',strict),('candidate',candidate)]:
            client=CollisionClient(backend,timeout=60);clients[name]=client
            ready[name]=client.initialize(model,scene,semantic=config)
        if ready['strict']['identity']['robot_geometry_sha256']!=ready['candidate']['identity']['robot_geometry_sha256']:raise ValueError('geometry changed between SRDFs')
        for name in ['joint1','joint2','joint3']:
            pair={'joint1':('back_shell_green_link','link1'),'joint2':('link2','servo_link1'),'joint3':('link2','link3')}[name]
            for anchor,base_q in [('diagnostic_zero',np.zeros(5)),('held_sample19',arrays['q'][19])]:
                for angle in np.linspace(model.joints[name].lower,model.joints[name].upper,7):
                    q=base_q.copy();q[list(model.joint_order).index(name)]=angle
                    audit={'max_contacts_per_pair':64,'max_contacts':20000,'distance_pairs':[list(pair)]}
                    a=clients['strict'].query(model,q,auxiliary_positions={'r_joint':.3},geometry_audit=audit)
                    b=clients['candidate'].query(model,q,auxiliary_positions={'r_joint':.3},geometry_audit=audit)
                    for key in ['colliding_pairs','joint_positions_used','T_base_to_ee','link_transforms_base','pair_geometry_distances']:
                        if a[key]!=b[key]:raise ValueError('scan raw/kinematic mismatch: '+key)
                    if a['contact_collection']['global_limit_reached'] or b['contact_collection']['global_limit_reached']:raise ValueError('global contact cap prevents pair audit')
                    evidence=contact_evidence(model,a,*pair,name)
                    scans.append({'joint':name,'anchor':anchor,'angle_rad':float(angle),'q':q.tolist(),'gripper_rad':.3,
                        'pair':list(pair),'raw_pair_collision':a['pair_geometry_distances'][0]['collision'],
                        'pair_disabled_in_candidate':True,'strict_pair_blocking':sorted(pair) in a['semantic_blocking_pairs'],
                        'candidate_pair_blocking':sorted(pair) in b['semantic_blocking_pairs'],
                        'local_contact_evidence':evidence,'contact_collection':a['contact_collection'],
                        'contact_surface_complete':False,'physical_permission_inferred':False})
        for opening in [0.,.3,1.]:
            for index in [8,19,73]:
                a=clients['strict'].query(model,arrays['q'][index],auxiliary_positions={'r_joint':opening})
                b=clients['candidate'].query(model,arrays['q'][index],auxiliary_positions={'r_joint':opening})
                for key in ['colliding_pairs','joint_positions_used','T_base_to_ee','link_transforms_base','environment_collision']:
                    if a[key]!=b[key]:raise ValueError('positive raw/kinematic mismatch')
                if index in [8,73] and not b['environment_collision']:raise ValueError('environment positive lost')
                if index==19 and ['back_shell_black_link','link3'] not in b['semantic_blocking_pairs']:raise ValueError('folded positive lost')
                positives.append({'sample_index':index,'gripper_rad':opening,'environment_collision':b['environment_collision'],
                    'candidate_self_collision':b['semantic_self_collision'],'remaining_pairs':b['semantic_blocking_pairs'],'check':'PASS'})
        for record in poses['candidates']:
            if not record['urdf_joint_limits_valid']:continue
            record['model_queries']={}
            for name,c in clients.items():
                row=c.query(model,record['q_controller_jointstate_rad'],auxiliary_positions={'r_joint':record['gripper_rad']})
                record['model_queries'][name]={'raw_self_collision':row['raw_self_collision'],'semantic_self_collision':row['semantic_self_collision'],
                    'environment_collision':row['environment_collision'],'blocking_pairs':row['semantic_blocking_pairs'],'T_base_to_ee':row['T_base_to_ee'],
                    'identity':row['identity'],'normal_reference_status':'PENDING'}
    finally:
        for name,c in clients.items():cleanup[name]=c.close()
    if not all(v['exit_normal'] for v in cleanup.values()):raise ValueError('assembly audit backend exited abnormally')
    return {'software_checks':'PASS','usage':'RESEARCH_ONLY','normal_reference':poses,'system_source':source_audit,
        'joint_scans':scans,'positive_examples':positives,'identities':{k:v['identity'] for k,v in ready.items()},'cleanup':cleanup,
        'model_mechanical_validation':'PARTIAL','pair_review':pair_review(model,strict,candidate)}
