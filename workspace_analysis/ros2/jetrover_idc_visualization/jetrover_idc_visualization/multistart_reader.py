"""Verified multi-start paths, adapted to the existing read-only replay contract."""
import numpy as np
from .data import Catalog, Goal, Port
from .errors import ReplayBlocked


def multistart_catalog(root,store,manifest,input_profile,anchor):
    from .reader import read_catalog, _trajectory
    from workspace_analysis.experiments.multistart_benchmark import verify_multistart
    from workspace_analysis.experiments.base_pose_context import verify_context
    from workspace_analysis.scene.schema import parse_scene
    from workspace_analysis.task.spec import parse_task
    from workspace_analysis.task.targets import build_targets
    from workspace_analysis.robot.fk import ForwardKinematics
    source=read_catalog(root,manifest['source_condition'],input_profile,
        expected_manifest_sha256=manifest['source_condition_manifest_sha256'])
    report=verify_multistart(store.root,source.model)
    files=manifest['files']
    def obj(name):return store.object(name,digest=files[name])
    result=obj('result.json')
    if result['start_q'] is None:
        raise ReplayBlocked('NO_QUALIFIED_READY_POSE','no real start exists to replay')
    scene=parse_scene(store.read('scene_snapshot.json',digest=files['scene_snapshot.json']).decode())
    context=verify_context(obj('base_pose_context.json'),scene,source.model)
    context.update(workspace_config=result['workspace_config'],workspace_mode=result['workspace_mode'],
        start_id=result['start_id'],start_sha256=result['start_sha256'],failure_diagnostics={})
    spec=parse_task(obj('task_spec.json'));targets=build_targets(spec,scene);fk=ForwardKinematics(source.model)
    context['start_world_tcp_xyz_m']=(np.asarray(context['world_from_base'])@fk(result['start_q'])@spec.T_ee_tcp)[:3,3].tolist()
    goals=[]
    for saved in result['goals']:
        paths={};diagnostics=[]
        for trial in saved['paths']:
            if trial['success']:
                paths[trial['method']]=_trajectory(store,files,trial['trajectory_file'],'Path')
            failure=trial.get('first_failure_state')
            if failure is not None:
                diagnostics.append({'stage':trial['method'],'q':failure['q'],'reasons':failure['reasons'],
                    'world_tcp_xyz_m':failure['workspace']['world_tcp_xyz_m'],
                    'environment_pairs':failure['environment_pairs']})
        context['failure_diagnostics'][str(saved['solution_id'])]=diagnostics
        status=saved['planning_status']+' | '+', '.join(t['method']+':'+t['status'] for t in saved['paths'])
        planner_results={}
        for trial in saved['paths']:
            entry={'success':trial['success'],'status':trial['status']}
            if not trial['success']:
                failed=trial.get('first_failure_state')
                entry['reasons']=failed['reasons'] if failed is not None else [trial.get('reason',trial['status'])]
                entry['environment_pairs']=[] if failed is None else failed.get('environment_pairs',[])
            planner_results[trial['method']]=entry
        goals.append(Goal(saved['solution_id'],tuple(saved['q']),status,paths,None,
            planner_results,
            saved['goal_check']['valid'],fk(saved['q'])@spec.T_ee_tcp))
    port=result['target']
    detail=f"T5.3-B {result['scene_label']} | Start={result['start_id']} | Mode={result['workspace_mode']} | start valid={result['start_check']['valid']} | preparation motion NOT verified"
    return Catalog({port:Port(port,'MULTISTART_RESEARCH_ONLY',detail,tuple(goals))},source.model,scene,{port:targets},
        tuple(result['start_q']),fk(result['start_q'])@spec.T_ee_tcp,source.gripper,anchor,
        {'status':'PASS','scope':'T5.3-B grid/FK/constraint verifier','full_verification':report,
         'trusted_manifest_bound':True,'native_backend_started':False},np.asarray(context['world_from_base']),context)
