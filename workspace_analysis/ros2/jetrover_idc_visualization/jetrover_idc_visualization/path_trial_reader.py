"""T5.3-C Raw/Shortcut/Reference-to-Ready paths on the existing replay contract."""
import numpy as np
from .data import Catalog,Goal,Port


def path_trial_catalog(root,store,manifest,input_profile,anchor):
    from .reader import read_catalog,_trajectory
    from workspace_analysis.experiments.path_trial_artifacts import verify_path_trial
    from workspace_analysis.experiments.base_pose_context import verify_context
    from workspace_analysis.scene.schema import parse_scene
    from workspace_analysis.task.spec import parse_task
    from workspace_analysis.task.targets import build_targets
    from workspace_analysis.robot.fk import ForwardKinematics
    source=read_catalog(root,manifest['source_condition'],input_profile,
        expected_manifest_sha256=manifest['source_condition_manifest_sha256'])
    verified=verify_path_trial(store.root,source.model);files=manifest['files']
    def obj(name):return store.object(name,digest=files[name])
    result=obj('result.json');inputs=obj('inputs.json');trial=result['trial']
    scene=parse_scene(store.read('scene_snapshot.json',digest=files['scene_snapshot.json']).decode())
    info=verify_context(obj('base_pose_context.json'),scene,source.model)
    spec=parse_task(obj('task_spec.json'));targets=build_targets(spec,scene);fk=ForwardKinematics(source.model)
    info.update(workspace_config=inputs['workspace_config'],workspace_mode=inputs['workspace_mode'],
        start_id=inputs['start_id'],start_world_tcp_xyz_m=(np.asarray(info['world_from_base'])@fk(inputs['start_q'])@spec.T_ee_tcp)[:3,3].tolist())
    paths={'rrt_connect' if inputs['method']=='rrt_connect' else 'linear':_trajectory(store,files,trial['trajectory_file'],'Path')} if trial['success'] else {}
    method=inputs['method'];quality=trial.get('quality',{})
    def metric(name):
        value=quality.get(name)
        return 'n/a' if value is None else f'{value:.4g}'
    detail=(f"T5.3-C {inputs['case_id']} {result['variant']} | {inputs['workspace_mode']} | "
        f"{inputs['start_id']} -> {inputs.get('ready_id',inputs['goal_id'])} | seed={inputs['planning_settings']['random_seed']}\n"
        f"Joint={metric('joint_path_length_rad')} rad; TCP={metric('tcp_path_length_m')} m; DISCRETE_ONLY")
    if inputs['goal_role']=='READY_POSE':
        detail+='\nREFERENCE_START_TO_READY; physical Home NOT verified'
        info.update(preparation_goal_world_tcp_xyz_m=(np.asarray(info['world_from_base'])@fk(inputs['goal_q'])@spec.T_ee_tcp)[:3,3].tolist(),ready_goal_id=inputs['ready_id'])
    else:detail+='\nPre-insertion only; hardware NOT verified'
    goal=Goal(inputs['goal_id'],tuple(inputs['goal_q']),trial['status'],paths,None,
        {method:{'success':trial['success'],'status':trial['status']}},
        result.get('goal_check',{}).get('valid',trial['success']),fk(inputs['goal_q'])@spec.T_ee_tcp)
    return Catalog({inputs['target']:Port(inputs['target'],'T53C_RESEARCH_ONLY',detail,(goal,))},source.model,scene,
        {inputs['target']:targets},tuple(inputs['start_q']),fk(inputs['start_q'])@spec.T_ee_tcp,source.gripper,anchor,
        {'status':'PASS','scope':'T5.3-C full path/shortcut verifier','full_verification':verified,
         'trusted_manifest_bound':True,'native_backend_started':False},np.asarray(info['world_from_base']),info)
