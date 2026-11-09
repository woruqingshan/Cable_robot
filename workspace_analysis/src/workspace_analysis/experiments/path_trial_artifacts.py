"""Compact T5.3-C path evidence and independent readback, no search at import/read."""
from pathlib import Path
import gzip
import json
import numpy as np
from .assembly_semantics import _hash, _write
from .multistart_benchmark import compact, bound_files, open_ledger
from .ready_pose import ready_digest
from .base_pose_context import verify_context
from ..robot.fk import ForwardKinematics
from ..scene.schema import load_scene
from ..task.spec import parse_task
from ..task.targets import build_targets,tcp_errors
from ..planning.validity import assess_task
from ..planning.workspace_constraint import TCPWorkspaceCheck
from ..planning.local_workspace import constraint_from_config
from ..planning.joint_path import sample_path,path_length
from ..planning.path_quality import path_quality


SCENE_FILES=('scene_snapshot.json','world_scene.json','base_pose_context.json','task_spec.json','geometry_audit.json')


def reference_inputs(source_root,condition,*,goal_id,experiment,case_id,seed,budget_s=None,ready_id=None,method='rrt_connect'):
    """Read q from hash-bound B artifacts, never recompute or select new starts/goals."""
    source_root=Path(source_root).resolve();source=source_root/condition
    parent=json.loads((source_root/'manifest.json').read_text())
    if _hash(source/'manifest.json')!=parent['conditions'][condition]:raise ValueError('source condition anchor differs')
    manifest=json.loads((source/'manifest.json').read_text());bound_files(source,manifest)
    result=json.loads((source/'result.json').read_text());goal=next(g for g in result['goals'] if g['solution_id']==goal_id)
    params=dict(result['planning_settings']);params['random_seed']=seed
    if budget_s is not None:params['max_time_s']=budget_s
    inputs={'source_root':str(source_root),'source_root_manifest_sha256':_hash(source_root/'manifest.json'),
        'source_condition':str(source),'source_condition_manifest_sha256':_hash(source/'manifest.json'),
        'experiment':experiment,'case_id':case_id,'method':method,'goal_id':goal_id,
        'base_pose':result['base_pose'],'target':result['target'],'scene_label':result['scene_label'],
        'start_id':result['start_id'],'start_q':result['start_q'],'goal_q':goal['q'],
        'source_start_sha256':result['start_sha256'],'workspace_mode':result['workspace_mode'],
        'workspace_config':result['workspace_config'],'planning_settings':params,
        'goal_role':'PRE_INSERTION','expected_native_identity':result['identity']}
    if ready_id is not None:
        if experiment!='preparation' or ready_id!=result['start_id'] or result['workspace_mode']!='conditional':raise ValueError('preparation Ready binding differs')
        station=result['base_pose']['id'];label='D0' if station=='b0' else 'D1'
        reference=source_root/(station+'_'+label+'_S0_conditional')
        ref_manifest=json.loads((reference/'manifest.json').read_text());bound_files(reference,ref_manifest)
        if _hash(reference/'manifest.json')!=parent['conditions'][reference.name]:raise ValueError('reference anchor differs')
        original=json.loads((reference/'result.json').read_text())
        inputs.update(start_id='S0_REFERENCE',start_q=original['start_q'],goal_q=result['start_q'],
            goal_id=0 if ready_id=='S1' else 1,ready_id=ready_id,goal_role='READY_POSE',
            preparation_label='REFERENCE_START_TO_READY',physical_home_verified=False,
            reference_condition=str(reference),reference_manifest_sha256=_hash(reference/'manifest.json'))
    inputs['sha256']=ready_digest(inputs)
    return inputs


def load_context(root,inputs,model):
    source=Path(inputs['source_condition']);scene=load_scene(source/'scene_snapshot.json')
    spec=parse_task(json.loads((source/'task_spec.json').read_text()))
    context=json.loads((source/'base_pose_context.json').read_text());verify_context(context,scene,model)
    rule=TCPWorkspaceCheck(constraint_from_config(inputs['workspace_config']),context['world_from_base'],spec.T_ee_tcp,context['base_pose'])
    return scene,spec,context,rule,build_targets(spec,scene)


def compress_report(report):
    if report is None:return None
    result=dict(report)
    result['state_query_ids']=[s['query_id'] for s in result.pop('states')]
    return result


def store_ledger(path,checker,identity):
    with Path(path).open('xb') as raw:
        with gzip.GzipFile(fileobj=raw,mode='wb',mtime=0) as output:
            output.write(json.dumps({'identity':identity,'records':[compact(s) for s in checker.queries],
                'statistics':checker.statistics()},allow_nan=False).encode())


def finish_trial(out,inputs,result,model,source_hashes,head):
    out=Path(out)
    _write(out/'inputs.json',inputs);_write(out/'result.json',result)
    _write(out/'manifest.json',{'stage':'T5.3-C','schema_version':1,'usage':'RESEARCH_ONLY','git_head':head,
        'variant':result['variant'],'experiment':inputs['experiment'],'inputs_sha256':inputs['sha256'],
        'source_condition':inputs['source_condition'],'source_condition_manifest_sha256':inputs['source_condition_manifest_sha256'],
        'identity':result['identity'],'source_sha256':source_hashes,
        'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})
    receipt=verify_path_trial(out,model)
    _write(out/'independent_validation.json',receipt)
    _write(out/'validation_manifest.json',{'manifest_sha256':_hash(out/'manifest.json'),'report_sha256':_hash(out/'independent_validation.json')})


def termination(trial):
    if trial.get('status')=='NOT_RUN':return trial.get('reason','NOT_RUN')
    return {'VALID_DISCRETE_PATH':'SUCCESS','PLANNER_TIMEOUT':'TIME_LIMIT','PLANNER_BUDGET_EXHAUSTED':'ITERATION_LIMIT',
        'PATH_VALIDATION_FAILED':'PATH_VALIDATION_FAILURE','INVALID_START':'INVALID_START','INVALID_GOAL':'INVALID_GOAL',
        'BACKEND_ERROR':'BACKEND_ERROR'}.get(trial['status'],trial['status'])


def trial_row(path,inputs,result):
    trial=result['trial'];quality=trial.get('quality',{});search=trial.get('search') or {}
    return {'trial':str(path),'experiment':inputs['experiment'],'case':inputs['case_id'],
        'scene':inputs['scene_label'],'base':inputs['base_pose']['id'],'port':inputs['target'],
        'start':inputs['start_id'],'goal':inputs['goal_id'],'goal_role':inputs['goal_role'],
        'mode':inputs['workspace_mode'],'method':inputs['method'],'variant':result['variant'],
        'seed':inputs['planning_settings']['random_seed'],'max_time_s':inputs['planning_settings']['max_time_s'],
        'max_iterations':inputs['planning_settings']['max_iterations'],'status':trial['status'],
        'termination':termination(trial) if result['variant']!='shortcut' else trial['status'],
        'success':trial['success'],'search_time_s':trial.get('search_elapsed_s'),
        'validation_time_s':trial.get('validation_elapsed_s'),'shortcut_time_s':trial.get('shortcut_elapsed_s'),
        'total_time_s':result.get('total_time_s',trial.get('elapsed_s')),'iterations':search.get('iterations'),
        'native_queries':trial.get('query_statistics',{}).get('native_collision_queries'),
        'workspace_rejections':result.get('workspace_rejections'),
        'joint_length_rad':quality.get('joint_path_length_rad'),'tcp_length_m':quality.get('tcp_path_length_m'),
        'tcp_detour_ratio':quality.get('tcp_detour_ratio'),'waypoint_count':quality.get('raw_waypoint_count'),
        'validation_samples':quality.get('validated_sample_count'),'workspace_exit':quality.get('workspace_exit'),
        'min_self_clearance_m':quality.get('minimum_clearance',{}).get('self',{}).get('minimum_m'),
        'min_environment_clearance_m':quality.get('minimum_clearance',{}).get('environment',{}).get('minimum_m'),
        'manifest_sha256':_hash(Path(path)/'manifest.json')}


def verify_path_trial(run,model):
    """Hash-bound independent FK/grid/decision/NPZ/shortcut audit; no Native calls."""
    run=Path(run).resolve();manifest=json.loads((run/'manifest.json').read_text());bound_files(run,manifest)
    if manifest.get('stage')!='T5.3-C' or manifest.get('usage')!='RESEARCH_ONLY':raise ValueError('T5.3-C research manifest required')
    inputs=json.loads((run/'inputs.json').read_text());values={k:v for k,v in inputs.items() if k!='sha256'}
    if ready_digest(values)!=inputs['sha256'] or inputs['sha256']!=manifest['inputs_sha256']:raise ValueError('trial input identity differs')
    source=Path(inputs['source_condition']);source_parent=Path(inputs['source_root'])
    if _hash(source_parent/'manifest.json')!=inputs['source_root_manifest_sha256']:raise ValueError('B collection anchor changed')
    parent=json.loads((source_parent/'manifest.json').read_text())
    if not source.is_relative_to(source_parent) or _hash(source/'manifest.json')!=parent['conditions'][source.name] or _hash(source/'manifest.json')!=inputs['source_condition_manifest_sha256']:raise ValueError('source condition identity differs')
    sm=json.loads((source/'manifest.json').read_text())
    if _hash(source/'result.json')!=sm['files']['result.json']:raise ValueError('B source q evidence changed')
    saved=json.loads((source/'result.json').read_text());data=json.loads((run/'result.json').read_text())
    if inputs['goal_role']=='PRE_INSERTION':
        goal=next(g for g in saved['goals'] if g['solution_id']==inputs['goal_id'])
        if inputs['start_q']!=saved['start_q'] or inputs['goal_q']!=goal['q']:raise ValueError('fixed Start/Goal changed')
    elif inputs['goal_role']=='READY_POSE':
        ref=Path(inputs['reference_condition'])
        if _hash(ref/'manifest.json')!=inputs['reference_manifest_sha256'] or inputs['reference_manifest_sha256']!=parent['conditions'][ref.name]:raise ValueError('reference source differs')
        rm=json.loads((ref/'manifest.json').read_text())
        if _hash(ref/'result.json')!=rm['files']['result.json']:raise ValueError('reference q evidence differs')
        if inputs['start_q']!=json.loads((ref/'result.json').read_text())['start_q'] or inputs['goal_q']!=saved['start_q'] or inputs['workspace_mode']!='conditional':raise ValueError('preparation endpoints or mode changed')
    else:raise ValueError('unknown goal role')
    for k,v in saved['planning_settings'].items():
        if k not in ('random_seed','max_time_s') and inputs['planning_settings'][k]!=v:raise ValueError('undeclared planner setting change')
    if inputs['workspace_config']!=saved['workspace_config']:raise ValueError('workspace changed')
    for name in SCENE_FILES:
        if _hash(run/name)!=sm['files'][name]:raise ValueError('scene/task/context changed')
    scene=load_scene(run/'scene_snapshot.json');spec=parse_task(json.loads((run/'task_spec.json').read_text()))
    context=verify_context(json.loads((run/'base_pose_context.json').read_text()),scene,model)
    rule=TCPWorkspaceCheck(constraint_from_config(inputs['workspace_config']),context['world_from_base'],spec.T_ee_tcp,context['base_pose'])
    fk=ForwardKinematics(model);targets=build_targets(spec,scene)
    if data['identity']!=inputs['expected_native_identity'] or manifest['identity']!=data['identity']:raise ValueError('Native identity changed')
    if inputs['goal_role']=='PRE_INSERTION' and data['task_check']!=assess_task(tcp_errors(fk(inputs['goal_q'])@spec.T_ee_tcp,targets),targets):raise ValueError('task pose changed')
    ledger=open_ledger(run)
    for index,state in enumerate(ledger):
        if state['query_id']!=index:raise ValueError('query order changed')
        if state['status']=='ERROR':continue
        t=fk(state['q']);np.testing.assert_allclose(t,state['T_base_to_ee'],atol=1e-10,rtol=0)
        expected=rule.assess(t)
        if expected!=state['workspace'] or state['valid']!=(state['fcl_valid'] and expected['valid']):raise ValueError('workspace/FCL conjunction differs')
    checked=0;grid_cache={}
    def verify_report(report,waypoints,*,fresh,complete):
        nonlocal checked
        if report is None:return []
        if report['resolution_max_joint_rad']!=inputs['planning_settings']['verification_resolution_rad']:raise ValueError('validation resolution changed')
        ids=report['state_query_ids'];states=[ledger[i] for i in ids]
        key=(tuple(tuple(float(x) for x in q) for q in waypoints),report['resolution_max_joint_rad'])
        if key not in grid_cache:
            expected,provenance=sample_path(waypoints,report['resolution_max_joint_rad']);expected,provenance=rule.refine(expected,provenance,fk)
            grid_cache[key]=(expected,provenance)
        expected,provenance=grid_cache[key]
        np.testing.assert_array_equal(expected[:len(states)],np.asarray([s['q'] for s in states]))
        if report['provenance']!=provenance[:len(states)]:raise ValueError('sample provenance changed')
        if report['expected_samples']!=len(expected) or report['checked_samples']!=len(states):raise ValueError('sample counts differ')
        if complete and (len(states)!=len(expected) or not report['complete']):raise ValueError('incomplete full validation')
        if report['valid']!=(report['complete'] and all(s['valid'] for s in states)):raise ValueError('path validity differs')
        if fresh and (len(set(ids))!=len(ids) or ids!=sorted(ids)):raise ValueError('fresh validation queries reused')
        for kind in ('self','environment'):
            values=[s['semantic_clearance']['native'][kind]['separation_m'] for s in states]
            if report['clearance'][kind]['minimum_m']!=(min(values) if values else None):raise ValueError('clearance differs')
        checked+=len(states);return states
    trial=data['trial'];report=trial.get('validation');points=trial.get('waypoints')
    if data['variant']!='shortcut' and trial.get('settings')!=inputs['planning_settings']:
        raise ValueError('recorded planner settings changed')
    if trial.get('search') is not None and trial['search']['seed']!=inputs['planning_settings']['random_seed']:
        raise ValueError('seed was not forwarded to search')
    if report is not None:
        states=verify_report(report,points,fresh=True,complete=True)
        if report['state_query_ids']!=list(range(len(ledger)-len(states),len(ledger))):raise ValueError('final verification is not the fresh query tail')
        np.testing.assert_array_equal(points[0],inputs['start_q']);np.testing.assert_array_equal(points[-1],inputs['goal_q'])
        if trial['success']!=report['valid']:raise ValueError('trial classification differs')
        if path_quality({**trial,'validation':{**report,'states':states}},[s['workspace']['world_tcp_xyz_m'] for s in states])!=trial['quality']:raise ValueError('path quality differs')
        with np.load(run/trial['trajectory_file'],allow_pickle=False) as arrays:
            np.testing.assert_array_equal(arrays['q'],[s['q'] for s in states]);np.testing.assert_array_equal(arrays['query_id'],report['state_query_ids'])
            np.testing.assert_array_equal(arrays['valid'],[s['valid'] for s in states])
            for q,t in zip(arrays['q'],arrays['T_base_tcp']):np.testing.assert_allclose(fk(q)@spec.T_ee_tcp,t,atol=1e-10,rtol=0)
            for kind in ('self','environment'):np.testing.assert_array_equal(arrays[kind+'_clearance_m'],[s['semantic_clearance']['native'][kind]['separation_m'] for s in states])
        if trial['success'] and inputs['workspace_mode']=='local_only' and trial['quality']['workspace_exit']:raise ValueError('local-only escaped')
    elif trial['success']:raise ValueError('success without a checked path')
    if data['variant']=='shortcut':
        raw=Path(data['source_raw_trial'])
        if _hash(raw/'manifest.json')!=data['source_raw_manifest_sha256']:raise ValueError('raw source changed')
        raw_manifest=json.loads((raw/'manifest.json').read_text());bound_files(raw,raw_manifest)
        raw_result=json.loads((raw/'result.json').read_text());raw_trial=raw_result['trial']
        if not raw_trial['success']:raise ValueError('shortcut applied to failed search')
        original=np.asarray(raw_trial['waypoints']);current=original.copy();accepted=0
        initial=verify_report(trial['input_validation'],original,fresh=True,complete=True)
        for attempt in trial['attempts']:
            i,j=attempt['i'],attempt['j']
            np.testing.assert_array_equal(attempt['from_q'],current[i]);np.testing.assert_array_equal(attempt['to_q'],current[j])
            edge=attempt['edge_validation']
            if edge is not None:verify_report(edge,[current[i],current[j]],fresh=False,complete=attempt['accepted'])
            if attempt['accepted']:
                if edge is None or not edge['valid']:raise ValueError('unchecked accepted shortcut')
                new=np.concatenate((current[:i+1],current[j:]))
                if path_length(current)-path_length(new)<=trial['settings']['reduction_epsilon_rad']:raise ValueError('nonreducing shortcut')
                current=new;accepted+=1
        if accepted!=trial['accepted_shortcuts']:raise ValueError('accepted attempt count differs')
        if trial['status']=='FALLBACK_ORIGINAL_VALIDATED':
            verify_report(trial['candidate_final_validation'],current,fresh=True,complete=True)
            current=original
        if trial['status']=='FINAL_VALIDATION_FAILURE':
            verify_report(trial['candidate_final_validation'],current,fresh=True,complete=True)
            verify_report(trial['fallback_validation'],original,fresh=True,complete=True)
        if trial['success']:
            np.testing.assert_array_equal(current,trial['waypoints'])
            if path_length(current)>path_length(original)+1e-12:raise ValueError('optimized length increased')
    return {'status':'PASS','query_records_checked':len(ledger),'checked_path_samples':checked,'native_queries_started':0,
            'scope':'source identity, FK, discrete grid, semantic/World decisions, NPZ and shortcut reconstruction'}
