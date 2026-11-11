"""Verified T5.3-A adaptation to existing immutable replay contracts."""
import json
import numpy as np
from .storage import ReadOnlyStore,sha
from .data import Catalog,Goal,Port,Trajectory
from .errors import ReplayError


def restricted_catalog(root,store,manifest,input_profile,anchor):
    from .reader import read_catalog,_trajectory
    from workspace_analysis.experiments.restricted_benchmark import verify_restricted
    from workspace_analysis.experiments.base_pose_context import verify_context
    from workspace_analysis.scene.schema import parse_scene
    from workspace_analysis.task.spec import parse_task
    from workspace_analysis.task.targets import build_targets
    from workspace_analysis.robot.fk import ForwardKinematics
    source=read_catalog(root,manifest['source_benchmark'],input_profile,
        expected_manifest_sha256=manifest['source_manifest_sha256'])
    report=verify_restricted(store.root,source.model)
    files=manifest['files']
    def obj(name):
        if name not in files:
            raise ReplayError('UNLISTED_ARTIFACT',name)
        return store.object(name,digest=files[name])
    result=obj('result.json');scene=parse_scene(store.read('scene_snapshot.json',digest=files['scene_snapshot.json']).decode())
    context=verify_context(obj('base_pose_context.json'),scene,source.model)
    context['workspace_config']=result['workspace_config']
    context['failure_diagnostics']={}
    spec=parse_task(obj('task_spec.json'));targets=build_targets(spec,scene);fk=ForwardKinematics(source.model)
    goals=[]
    for saved in result['goals']:
        paths={}
        for trial in saved['paths']:
            if trial['success']:
                paths[trial['method']]=_trajectory(store,files,trial['trajectory_file'],'Path')
        historical=saved['historical_approach'];approach=None
        if historical.get('valid'):
            states=historical['states'];n=len(states)
            approach=Trajectory(np.array([s['q'] for s in states]),np.array([s['actual_T_base_tcp'] for s in states]),
                np.ones(n,dtype=bool),np.array([s['query_id'] for s in states],dtype=np.int64),
                *[np.array([s['semantic_clearance']['native'][kind]['separation_m'] for s in states]) for kind in ('self','environment')],
                tuple(('result.json#historical_approach',i) for i in range(n)),('Approach',)*n)
        status=saved['planning_status']
        if saved['paths'] and not paths:
            status='NO_ACCEPTED_PATH'
        if paths:
            status='PATH_AND_HISTORICAL_APPROACH_VALID' if approach is not None else 'PATH_VALID_APPROACH_REJECTED'
        diagnostics=[]
        for label,check in [('goal',{'states':[saved['goal_check']]}),
                            *saved['historical_paths'].items(),('historical_approach',historical)]:
            state=next((s for s in check.get('states',[]) if not s['valid']),None)
            if state is not None:
                diagnostics.append({'stage':label,'q':state['q'],'reasons':state['reasons'],
                    'world_tcp_xyz_m':state['workspace']['world_tcp_xyz_m'],
                    'environment_pairs':state.get('environment_pairs',[])})
        context['failure_diagnostics'][str(saved['solution_id'])]=diagnostics
        if diagnostics:
            status+=' | '+', '.join(diagnostics[0]['reasons'])
        goals.append(Goal(saved['solution_id'],tuple(saved['q']),status,paths,approach,
            {t['method']:{'success':t['success'],'status':t['status']} for t in saved['paths']},
            saved['goal_check']['valid'],fk(saved['q'])@spec.T_ee_tcp))
    port=result['target']
    detail=f"T5.3-A {result['scene_label']} | workspace={result['workspace_config']['enabled']} | start valid={result['start_check']['valid']}"
    catalog=Catalog({port:Port(port,'RESTRICTED_RESEARCH_ONLY',detail,tuple(goals))},source.model,scene,{port:targets},
        tuple(result['start_q']),fk(result['start_q'])@spec.T_ee_tcp,source.gripper,anchor,
        {'status':'PASS','scope':'T5.3-A full evidence/FK/workspace verifier','full_verification':report,
         'trusted_manifest_bound':True,'native_backend_started':False},np.asarray(context['world_from_base']),context)
    for name,h in files.items():store.read(name,digest=h)
    return catalog
