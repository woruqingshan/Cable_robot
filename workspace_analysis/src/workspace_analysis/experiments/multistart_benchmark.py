"""T5.3-B: fixed-goal, preselected-start experiments using existing planners/FCL.

Large duplicate RobotState mesh transforms are not serialized. A compressed,
query-indexed ledger keeps q, authoritative Native EE, semantic decisions,
clearance and World workspace evidence. Path records reference that ledger.
"""
from pathlib import Path
from hashlib import sha256
import csv
import gzip
import json
import os
import subprocess
import numpy as np
from .assembly_semantics import _hash, _write
from .restricted_benchmark import verify_restricted
from .ready_pose import select_ready, ready_digest
from .task_context import TaskContext, require_resolved
from .task_path_planning import run_task_path
from .task_benchmark import save_trajectory
from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..workspace.results import load_results
from ..scene.schema import load_scene
from ..scene.base_pose import BasePose
from ..task.spec import parse_task
from ..task.targets import build_targets, tcp_errors
from ..planning.validity import ConfigurationValidity, assess_task
from ..planning.workspace_constraint import WorkspaceConstraint, TCPWorkspaceCheck
from ..planning.local_workspace import LocalOnlyWorkspaceConstraint, constraint_from_config
from ..planning.joint_path import sample_path
from ..planning.path_quality import path_quality
from ..collision.evaluator import Evaluator
from ..collision.semantic import SemanticConfig


def compact(state):
    return {k:v for k,v in state.items() if k not in ('link_transforms_base','contacts_base','joint_positions_used')}


def open_ledger(run):
    with gzip.open(Path(run)/'collision_queries.json.gz','rt') as f:
        return json.load(f)['records']


def bound_files(run, manifest):
    run=Path(run).resolve()
    for name,digest in manifest['files'].items():
        path=(run/name).resolve()
        if not path.is_relative_to(run) or _hash(path)!=digest:
            raise ValueError('artifact hash/path differs: '+name)


def evaluator_for(model,scene,spec,backend,semantic):
    settings=spec.data['collision']
    return Evaluator(model,scene,None,backend,semantic=semantic,gripper=settings['gripper_rad'],
        clearance={'self_m':settings['self_clearance_m'],'environment_m':settings['environment_clearance_m']},timeout=60)


def save_condition(out, source, source_hash, selected, start_id, mode, model, backend, semantic, head, source_hashes):
    previous=json.loads((source/'result.json').read_text())
    source_manifest=json.loads((source/'manifest.json').read_text())
    scene=load_scene(source/'scene_snapshot.json')
    spec=parse_task(json.loads((source/'task_spec.json').read_text()))
    context_data=json.loads((source/'base_pose_context.json').read_text())
    pose=BasePose.from_dict(previous['base_pose']); fk=ForwardKinematics(model)
    targets=build_targets(spec,scene)
    historical=Path(source_manifest['source_benchmark'])
    old=json.loads((historical/'ports'/previous['target']/'result.json').read_text())
    settings=old['planning_settings']
    constraint=LocalOnlyWorkspaceConstraint() if mode=='local_only' else WorkspaceConstraint()
    rule=TCPWorkspaceCheck(constraint,pose.matrix(),spec.T_ee_tcp,pose.to_dict())
    out.mkdir(parents=True,exist_ok=False)
    for name in ('scene_snapshot.json','world_scene.json','base_pose_context.json','task_spec.json','geometry_audit.json'):
        (out/name).write_bytes((source/name).read_bytes())
    _write(out/'start.json',selected)
    q_start=selected['selected']['q'] if selected['selected'] is not None else None
    result={'usage':'RESEARCH_ONLY','scene_label':previous['scene_label'],'base_pose':pose.to_dict(),
        'target':previous['target'],'start_id':start_id,'start_sha256':selected['sha256'],
        'start_source':selected['selected'],'start_q':q_start,'workspace_mode':mode,
        'workspace_config':constraint.to_dict(),'workspace_sha256':constraint.sha256,
        'planning_settings':settings,'source_start_sample_index':0 if start_id=='S0' else None,
        'preparation_motion_to_this_start_verified':False,'goals':[],
        'multi_start_ik_executed':False,'new_approach_generated':False}
    rows=[]
    with evaluator_for(model,scene,spec,backend,semantic) as evaluator:
        checker=ConfigurationValidity(evaluator,workspace_check=rule)
        context=TaskContext(model,scene,targets,checker);context.validate()
        start=checker.check(q_start,fresh=True) if q_start is not None else {
            'valid':False,'status':'NO_QUALIFIED_READY_POSE','q':None,'query_id':None,'reasons':['NO_QUALIFIED_READY_POSE']}
        require_resolved([start]);result['start_check']=compact(start);result['identity']=evaluator.identity
        for old_goal in previous['goals']:
            q_goal=old_goal['q'];goal=checker.check(q_goal,fresh=True);require_resolved([goal])
            task=assess_task(tcp_errors(fk(q_goal)@spec.T_ee_tcp,targets),targets)
            record={'solution_id':old_goal['solution_id'],'q':q_goal,'goal_check':compact(goal),'task_check':task,'paths':[]}
            eligible=start['valid'] and goal['valid'] and task['task_pose_feasible']
            record['planning_status']='EXECUTED' if eligible else 'INVALID_START' if not start['valid'] else 'INVALID_GOAL'
            for method in ('linear','rrt_connect'):
                if not eligible:
                    trial={'method':method,'status':'NOT_RUN','reason':record['planning_status'],
                        'success':False,'search':None,'validation':None,'settings':settings,'quality':{'status':'NOT_RUN'}}
                else:
                    query_begin=len(checker.queries)
                    trial=run_task_path(context,q_start,q_goal,method=method,settings=settings)
                    trial['query_range']=[query_begin,len(checker.queries)]
                    validation=trial['validation']
                    points=[] if validation is None else [s['workspace']['world_tcp_xyz_m'] for s in validation['states']]
                    trial['quality']=path_quality(trial,points)
                    if validation is not None:
                        filename=f"goal{old_goal['solution_id']}_{method}.npz"
                        save_trajectory(out/filename,validation['states'],spec)
                        trial['trajectory_file']=filename
                        # One canonical record per real query; provenance remains in validation.
                        validation['state_query_ids']=[s['query_id'] for s in validation.pop('states')]
                record['paths'].append(trial)
                failure=(trial.get('validation') or {}).get('first_failure')
                failure_state=None if failure is None else compact(checker.queries[failure['query_id']])
                trial['first_failure_state']=failure_state
                reason=record['planning_status'] if not eligible else trial['status'] if failure is None else '|'.join(failure['reasons'])
                quality=trial['quality']
                rows.append({'condition':out.name,'scene':previous['scene_label'],'base':pose.id,'port':previous['target'],
                    'start':start_id,'goal':old_goal['solution_id'],'mode':mode,'planner':method,
                    'endpoint_valid':eligible,'status':trial['status'],'success':trial['success'],
                    'search_elapsed_s':trial.get('search_elapsed_s'),'validation_elapsed_s':trial.get('validation_elapsed_s'),
                    'joint_length_rad':quality.get('joint_path_length_rad'),'tcp_length_m':quality.get('tcp_path_length_m'),
                    'tcp_detour_ratio':quality.get('tcp_detour_ratio'),'workspace_exit':quality.get('workspace_exit'),
                    'raw_waypoint_count':quality.get('raw_waypoint_count'),'validated_samples':quality.get('validated_sample_count'),
                    'replay_samples':quality.get('replay_sample_count'),
                    'minimum_self_clearance_m':quality.get('minimum_clearance',{}).get('self',{}).get('minimum_m'),
                    'minimum_environment_clearance_m':quality.get('minimum_clearance',{}).get('environment',{}).get('minimum_m'),
                    'native_queries':trial.get('query_statistics',{}).get('native_collision_queries'),
                    'failure_stage':None if trial['success'] else 'ENDPOINT' if not eligible else 'FINAL_VALIDATION' if failure else 'SEARCH',
                    'failure_reason':None if trial['success'] else reason})
                print(json.dumps(rows[-1]),flush=True)
            result['goals'].append(record)
        with (out/'collision_queries.json.gz').open('xb') as raw:
            with gzip.GzipFile(fileobj=raw,mode='wb',mtime=0) as compressed:
                compressed.write(json.dumps({'identity':evaluator.identity,'records':[compact(s) for s in checker.queries],
                    'statistics':checker.statistics(),'scope':'Native semantic evidence; per-link matrices and contacts omitted'},allow_nan=False).encode())
    if not evaluator.client.cleanup['exit_normal']: raise RuntimeError('Native backend exited abnormally')
    result['cleanup']=evaluator.client.cleanup
    _write(out/'result.json',result)
    _write(out/'manifest.json',{'stage':'T5.3-B','schema_version':1,'usage':'RESEARCH_ONLY','git_head':head,
        'source_condition':str(source),'source_condition_manifest_sha256':source_hash,
        'source_sha256':source_hashes,'source_benchmark':str(historical),
        'source_manifest_sha256':source_manifest['source_manifest_sha256'],
        'scene_sha256':scene.canonical_scene_sha256,'start_sha256':selected['sha256'],
        'workspace_sha256':constraint.sha256,'identity':result['identity'],
        'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})
    _write(out/'independent_validation.json',verify_multistart(out,model))
    _write(out/'validation_manifest.json',{'manifest_sha256':_hash(out/'manifest.json'),
        'report_sha256':_hash(out/'independent_validation.json')})
    return rows


def verify_multistart(run,model):
    """No Native process: independent grid/FK/constraint/NPZ/goal/source readback."""
    run=Path(run).resolve();manifest=json.loads((run/'manifest.json').read_text())
    if manifest.get('stage')!='T5.3-B' or manifest.get('usage')!='RESEARCH_ONLY':raise ValueError('research T5.3-B required')
    bound_files(run,manifest)
    source=Path(manifest['source_condition'])
    if _hash(source/'manifest.json')!=manifest['source_condition_manifest_sha256']:raise ValueError('source condition anchor differs')
    previous=json.loads((source/'result.json').read_text())
    old_manifest=json.loads((source/'manifest.json').read_text())
    if _hash(source/'result.json')!=old_manifest['files']['result.json']:raise ValueError('historical goals changed')
    data=json.loads((run/'result.json').read_text())
    historical=Path(old_manifest['source_benchmark'])
    historical_result=json.loads((historical/'ports'/previous['target']/'result.json').read_text())
    if data['planning_settings']!=historical_result['planning_settings']:raise ValueError('historical planner settings changed')
    spec=parse_task(json.loads((run/'task_spec.json').read_text()))
    scene=load_scene(run/'scene_snapshot.json');targets=build_targets(spec,scene);fk=ForwardKinematics(model)
    from .base_pose_context import verify_context
    context=verify_context(json.loads((run/'base_pose_context.json').read_text()),scene,model)
    if scene.canonical_scene_sha256!=manifest['scene_sha256'] or data['identity']!=manifest['identity']:raise ValueError('condition identity differs')
    for name in ('scene_snapshot.json','world_scene.json','base_pose_context.json','task_spec.json'):
        if _hash(run/name)!=old_manifest['files'][name]:raise ValueError('historical scene/task changed')
    selection=json.loads((run/'start.json').read_text());digest=selection.pop('sha256')
    if ready_digest(selection)!=digest or digest!=manifest['start_sha256'] or digest!=data['start_sha256']:raise ValueError('start identity differs')
    selected=selection['selected']
    if selected is not None:
        if selected['q']!=data['start_q']:raise ValueError('start changed')
        prefix=Path(selected['source_pool'])
        arrays,_=load_results(prefix,model=model)
        if any(_hash(prefix.with_suffix('.'+ext))!=selected['source_identity'][ext+'_sha256'] for ext in ('npz','json')):raise ValueError('Ready source input changed')
        np.testing.assert_array_equal(arrays['q'][selected['original_sample_index']],data['start_q'])
    if data['start_id']=='S0' and data['start_q']!=previous['start_q']:raise ValueError('historical sample 0 changed')
    constraint=constraint_from_config(data['workspace_config'])
    if constraint.sha256!=manifest['workspace_sha256']:raise ValueError('constraint identity differs')
    rule=TCPWorkspaceCheck(constraint,context['world_from_base'],spec.T_ee_tcp,context['base_pose'])
    ledger=open_ledger(run)
    for i,state in enumerate(ledger):
        if state['query_id']!=i:raise ValueError('ledger index differs')
        t=fk(state['q']);np.testing.assert_allclose(t,state['T_base_to_ee'],atol=1e-10,rtol=0)
        assessment=rule.assess(t)
        if assessment!=state['workspace'] or state['valid']!=(state['fcl_valid'] and assessment['valid']):raise ValueError('workspace/FCL classification differs')
    count=0
    if data['start_check'].get('query_id') is not None and data['start_check']!=ledger[data['start_check']['query_id']]:raise ValueError('start check ledger differs')
    for goal,old in zip(data['goals'],previous['goals']):
        if goal['goal_check']!=ledger[goal['goal_check']['query_id']]:raise ValueError('goal check ledger differs')
        if goal['q']!=old['q'] or goal['solution_id']!=old['solution_id']:raise ValueError('goal changed')
        if assess_task(tcp_errors(fk(goal['q'])@spec.T_ee_tcp,targets),targets)!=goal['task_check']:raise ValueError('goal pose classification differs')
        for trial in goal['paths']:
            if trial['settings']!=data['planning_settings']:raise ValueError('planner parameters differ')
            if trial['status']=='NOT_RUN':
                if data['start_check']['valid'] and goal['goal_check']['valid'] and goal['task_check']['task_pose_feasible']:raise ValueError('valid endpoints unexpectedly not run')
                continue
            report=trial['validation']
            if report is None:
                if trial['success'] or trial.get('trajectory_file'):raise ValueError('missing path misclassified')
                continue
            states=[ledger[i] for i in report['state_query_ids']]
            lo,hi=trial['query_range']
            if len(set(report['state_query_ids']))!=len(states) or any(not lo<=i<hi for i in report['state_query_ids']):raise ValueError('final check did not use fresh trial queries')
            expected,provenance=sample_path(trial['waypoints'],data['planning_settings']['verification_resolution_rad'])
            expected,provenance=rule.refine(expected,provenance,fk)
            np.testing.assert_array_equal(expected,np.asarray([s['q'] for s in states]))
            if provenance!=report['provenance'] or not report['complete'] or report['valid']!=all(s['valid'] for s in states):raise ValueError('grid or classification differs')
            if trial['success']!=report['valid']:raise ValueError('success classification differs')
            np.testing.assert_array_equal(expected[0],data['start_q']);np.testing.assert_array_equal(expected[-1],goal['q'])
            with np.load(run/trial['trajectory_file'],allow_pickle=False) as arrays:
                np.testing.assert_array_equal(arrays['q'],expected)
                np.testing.assert_array_equal(arrays['valid'],[s['valid'] for s in states])
                np.testing.assert_array_equal(arrays['query_id'],report['state_query_ids'])
                for kind in ('self','environment'):
                    np.testing.assert_array_equal(arrays[kind+'_clearance_m'],[s['semantic_clearance']['native'][kind]['separation_m'] for s in states])
                for q,t in zip(expected,arrays['T_base_tcp']):np.testing.assert_allclose(fk(q)@spec.T_ee_tcp,t,atol=1e-10,rtol=0)
            complete_trial={**trial,'validation':{**report,'states':states}}
            if path_quality(complete_trial,[s['workspace']['world_tcp_xyz_m'] for s in states])!=trial['quality']:raise ValueError('path metrics differ')
            if data['workspace_mode']=='local_only' and trial['success'] and trial['quality']['workspace_exit']:raise ValueError('local-only path exited workspace')
            count+=len(states)
    if len(data['goals'])!=len(previous['goals']):raise ValueError('goal lost')
    return {'status':'PASS','query_records_checked':len(ledger),'path_samples_checked':count,'new_native_queries':0,
        'scope':'source/grid/FK/World constraint and trajectory readback; discrete only'}


def run_multistart(root,source_run,backend,output,*,allow_candidate=False,expected_source_manifest_sha256=None):
    root=Path(root).resolve();source_run=Path(source_run).resolve();out=Path(output).absolute();backend=Path(backend).resolve(strict=True)
    if os.path.lexists(out):raise FileExistsError('refusing existing T5.3-B output')
    if not allow_candidate:raise ValueError('explicit RESEARCH_ONLY candidate opt-in required')
    if not expected_source_manifest_sha256 or _hash(source_run/'manifest.json')!=expected_source_manifest_sha256:raise ValueError('externally selected T5.3-A collection SHA256 required')
    parent=json.loads((source_run/'manifest.json').read_text());bound_files(source_run,parent)
    if parent.get('stage')!='T5.3-A_COLLECTION':raise ValueError('T5.3-A collection required')
    model=load_model(root/'config/workspace.yaml');fk=ForwardKinematics(model)
    semantic=SemanticConfig.load(root/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    config=json.loads((root/'config/restricted_scenes/ready_pose_v0.json').read_text())
    pools=[]
    for prefix in config['pools']:
        path=root/prefix;arrays,_=load_results(path,model=model)
        pools.append((str(path),arrays,{ext+'_sha256':_hash(path.with_suffix('.'+ext)) for ext in ('npz','json')}))
    # All code/config hashes freeze before selection/planning; no concurrent edits.
    source_hashes={str(p.relative_to(root)):_hash(p) for folder in ('src','scripts','config','ros2') for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p) and not p.is_symlink()}
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
    out.mkdir(parents=True,exist_ok=False);starts={};rows=[];conditions={}
    for station,source_name in config['selection_scene'].items():
        source=source_run/source_name;verify_restricted(source,model)
        previous=json.loads((source/'result.json').read_text());pose=BasePose.from_dict(previous['base_pose'])
        scene=load_scene(source/'scene_snapshot.json');spec=parse_task(json.loads((source/'task_spec.json').read_text()))
        targets=build_targets(spec,scene)
        port_world=(pose.matrix()@np.asarray(targets.to_dict()['T_base_port']))[:3,3]
        rules={k:config[k] for k in ('minimum_joint_margin_rad','maximum_abs_joint1_rad','S1','S2','ranking','exhaustion')}
        rules.update(insertion_axis_world=(pose.matrix()[:3,:3]@np.asarray(targets.insertion_axis_base)).tolist(),
            pre_distance_m=spec.data['approach']['distance_m'])
        with evaluator_for(model,scene,spec,backend,semantic) as evaluator:
            checker=ConfigurationValidity(evaluator,workspace_check=TCPWorkspaceCheck(WorkspaceConstraint(),pose.matrix(),spec.T_ee_tcp,pose.to_dict()))
            s0state=checker.check(previous['start_q'],fresh=True);require_resolved([s0state])
            s0={'start_id':'S0','status':'SELECTED','selected':{'q':previous['start_q'],'source_pool':pools[0][0],
                'source_identity':pools[0][2],'original_sample_index':0,
                'world_tcp_xyz_m':s0state['workspace']['world_tcp_xyz_m'],'validation':compact(s0state)},
                'selection_rules':{'rule':'unchanged historical G4 Sample 0'},'not_a_physical_home':True,
                'preparation_motion_to_this_start_verified':False,'planner_results_used_for_selection':False,'usage':'RESEARCH_ONLY'}
            starts[station]={'S0':{**s0,'sha256':ready_digest(s0)}}
            for start_id in ('S1','S2'):
                starts[station][start_id]=select_ready(model,fk,checker,pools,pose.matrix(),spec,port_world,
                    [g['q'] for g in previous['goals']],rules,start_id)
            _write(out/(station+'_selection_queries.json'),{'identity':evaluator.identity,'records':[compact(s) for s in checker.queries]})
        if not evaluator.client.cleanup['exit_normal']:raise RuntimeError('selection Native abnormal exit')
    _write(out/'ready_poses.json',starts)
    # Starts are now frozen; selection has not seen any planner output.
    for source_name in ('b0_D0_workspace_on','b0_D3_workspace_on','ym70_D1_workspace_on'):
        source=source_run/source_name
        station=source_name.split('_')[0];verify_restricted(source,model)
        for start_id,mode in [('S0','conditional'),('S1','conditional'),('S2','conditional')]+([('S2','local_only')] if '_D3_' not in source_name else []):
            name=source_name.replace('_workspace_on','')+'_'+start_id+'_'+mode
            condition=out/name
            rows.extend(save_condition(condition,source,parent['conditions'][source_name],starts[station][start_id],start_id,mode,
                model,backend,semantic,head,source_hashes))
            conditions[name]=_hash(condition/'manifest.json')
    _write(out/'summary.json',{'stage':'T5.3-B','usage':'RESEARCH_ONLY','rows':rows,'conditions':conditions,
        'planner_trial_denominator':len(rows),'scope':'fixed starts/goals, one seed; not statistical success probabilities'})
    with (out/'summary.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    for rel,h in source_hashes.items():
        if _hash(root/rel)!=h:raise RuntimeError('source changed during experiment: '+rel)
    _write(out/'manifest.json',{'stage':'T5.3-B_COLLECTION','usage':'RESEARCH_ONLY','git_head':head,
        'source_run':str(source_run),'source_manifest_sha256':expected_source_manifest_sha256,
        'selection_config_sha256':_hash(root/'config/restricted_scenes/ready_pose_v0.json'),
        'conditions':conditions,'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})
    return rows
