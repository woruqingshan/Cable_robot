"""Static vendor Home audit and explicitly labelled reference-to-Ready planning."""
from pathlib import Path
import json
import os
import math
import subprocess
import yaml
from .assembly_semantics import _hash,_write,audit_system_source
from .robustness_benchmark import run_trial,collection,source_hashes,semantic_for
from .path_trial_artifacts import reference_inputs
from .multistart_benchmark import bound_files,verify_multistart
from ..robot.model import load_model


def audit_home(system_source,project_root):
    """Never import or execute vendor controller/action code or a hardware driver."""
    vendor=Path(system_source).resolve();project=Path(project_root).resolve()
    config=vendor/'driver/controller/config/init_pose.yaml'
    script=vendor/'driver/controller/controller/init_pose.py'
    servo=vendor/'driver/servo_controller/config/servo_controller.yaml'
    manager=vendor/'driver/servo_controller/servo_controller/controller_manager.py'
    transform=vendor/'driver/kinematics/kinematics/transform.py'
    urdf=vendor/'simulations/jetrover_description/urdf'
    files=[config,script,servo,manager,transform,*sorted(urdf.glob('*.xacro'))]
    settings=yaml.safe_load(config.read_text())['/**']['ros__parameters']
    action_files=[str(p.relative_to(vendor)) for p in vendor.rglob('*.d6a') if p.is_file()]
    srdf_files=[str(p.relative_to(vendor)) for p in vendor.rglob('*.srdf') if p.is_file()]
    # An inactive pulse block is source evidence, not a verified joint-space Home.
    controller=yaml.safe_load(servo.read_text())['/**']['ros__parameters']
    pulse_evidence=[]
    for joint in ('joint1','joint2','joint3','joint4','joint5'):
        cfg=controller[joint];value=settings['servo']['id'+str(cfg['id'])]
        angle=((cfg['init']-value) if cfg['min']>cfg['max'] else (value-cfg['init']))*240/1000
        pulse_evidence.append({'joint':joint,'inactive_pulse':value,'controller_jointstate_degrees_from_source':angle,
            'source_only_not_a_URDF_Home':True})
    local=[]
    for folder in ('cable_robot_control','waypoints'):
        for p in (project/folder).rglob('*'):
            if not p.is_file() or p.suffix not in ('.yaml','.yml','.json'):continue
            try:
                document=json.loads(p.read_text()) if p.suffix=='.json' else yaml.safe_load(p.read_text())
            except (ValueError,yaml.YAMLError):continue
            if not isinstance(document,dict):continue
            waypoints=document.get('waypoints') or {}
            if not isinstance(waypoints,dict):continue
            for name,record in waypoints.items():
                if not any(token in name.upper() for token in ('HOME','REST','READY')):continue
                item={'path':str(p),'sha256':_hash(p),'name':name,'record':record,
                      'physical_home_verified':False,'representation':'servo pulse / vendor pose; no registered URDF Home'}
                if isinstance(record,dict) and isinstance(record.get('joint_pulse'),dict):
                    q=[]
                    for joint in ('joint1','joint2','joint3','joint4','joint5'):
                        cfg=controller[joint];pulses=record['joint_pulse']
                        pulse=pulses.get(cfg['id'],pulses.get(str(cfg['id'])))
                        if pulse is None:break
                        angle=((cfg['init']-pulse) if cfg['min']>cfg['max'] else (pulse-cfg['init']))*240/1000
                        q.append(math.radians(angle))
                    if len(q)==5:item['controller_convention_q_rad_diagnostic_only']=q
                    grip=record.get('gripper_pulse',{});cfg=controller.get('r_joint',{})
                    if cfg and cfg['id'] in grip:
                        pulse=grip[cfg['id']]
                        item['recorded_gripper_controller_rad']=math.radians(((cfg['init']-pulse) if cfg['min']>cfg['max'] else (pulse-cfg['init']))*240/1000)
                local.append(item)
    return {'status':'PHYSICAL_HOME_PENDING','planning_reference':'REFERENCE_START_TO_READY',
        'authoritative_home_q':None,'physical_home_verified':False,
        'external_default_action_directory_exists':Path('/home/ubuntu/software/arm_pc/ActionGroups').exists(),
        'default_init_type':settings['type'],'default_action':settings.get('action'),
        'vendor_action_files':action_files,'vendor_srdf_files':srdf_files,'local_named_pose_candidates':local,
        'inactive_pulse_block':pulse_evidence,
        'reason':'default init uses external /home/ubuntu/software/arm_pc/ActionGroups/init; no complete calibrated action/q mapping was verified. URDF zero and inactive pulse values are not accepted as physical Home.',
        'source_files':{str(p):_hash(p) for p in files if p.is_file()},'vendor_code_executed':False}


def run_preparation(root,source_run,backend,output,*,expected_source_manifest_sha256,system_source,allow_candidate=False):
    root=Path(root).resolve();source_run=Path(source_run).resolve();out=Path(output).absolute()
    backend=Path(backend).resolve(strict=True)
    if os.path.lexists(out):raise FileExistsError('refusing existing preparation output')
    if not allow_candidate:raise ValueError('explicit research candidate opt-in required')
    if _hash(source_run/'manifest.json')!=expected_source_manifest_sha256:raise ValueError('source B collection anchor differs')
    parent=json.loads((source_run/'manifest.json').read_text());bound_files(source_run,parent)
    model=load_model(root/'config/workspace.yaml');semantic=semantic_for(root,model)
    home=audit_home(system_source,root.parent)
    candidate=[math.radians(v['controller_jointstate_degrees_from_source']) for v in home['inactive_pulse_block']]
    home['inactive_controller_mapping_q_rad']=candidate
    try:
        model.validate_q(candidate);home['inactive_mapping_URDF_limit_check']='WITHIN_URDF_BUT_NOT_A_VERIFIED_HOME'
    except ValueError as exc:
        home['inactive_mapping_URDF_limit_check']='OUTSIDE_URDF_LIMITS'
        home['inactive_mapping_limit_error']=str(exc)
    from ..robot.fk import ForwardKinematics
    import numpy as np
    fk=ForwardKinematics(model)
    for candidate in home['local_named_pose_candidates']:
        q=candidate.get('controller_convention_q_rad_diagnostic_only')
        if q is None:continue
        try:
            model.validate_q(q);t=fk(q);candidate['diagnostic_URDF_EE_position_m']=t[:3,3].tolist()
            position=(candidate['record'].get('pose') or {}).get('position')
            if position is not None:
                vendor_xyz=np.array([position[k] for k in ('x','y','z')])
                candidate['unregistered_coordinate_difference_norm_m']=float(np.linalg.norm(t[:3,3]-vendor_xyz))
                candidate['comparison_scope']='raw numeric difference only; vendor/URDF reference and tool frames are not registered; not a physical FK error metric'
            candidate['planning_reference_selected']=False
        except ValueError as exc:candidate['diagnostic_limit_error']=str(exc)
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip();hashes=source_hashes(root)
    out.mkdir(parents=True,exist_ok=False);_write(out/'home_source_audit.json',home)
    # Existing convention audit includes DH vs controller JointState mapping; evidence only.
    _write(out/'vendor_convention_audit.json',audit_system_source(model,system_source))
    rows=[]
    for station,label,port in [('b0','D0','P22'),('ym70','D1','P23')]:
        for ready in ('S1','S2'):
            condition=station+'_'+label+'_'+ready+'_conditional';verify_multistart(source_run/condition,model)
            for method in ('linear','rrt_connect'):
                inputs=reference_inputs(source_run,condition,goal_id=0,experiment='preparation',case_id='PREP_'+station+'_'+ready,
                    seed=20261104,ready_id=ready,method=method)
                row=run_trial(root,inputs,backend,out/(station+'_S0_to_'+ready+'_'+method),model,semantic,head,hashes)
                rows.append(row)
                if row['termination']=='BACKEND_ERROR':raise RuntimeError('preparation backend error; dependent motions not run')
    collection(out,'T5.3-C_PREPARATION',rows,{'label':'REFERENCE_START_TO_READY','physical_home_verified':False,
        'source_run':str(source_run),'source_manifest_sha256':expected_source_manifest_sha256,'motions':4,'planner_trials':8})
    return rows
