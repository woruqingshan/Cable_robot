"""Saved-goal constrained experiments with fresh Native queries and existing planners."""
from pathlib import Path
from hashlib import sha256
import csv
import json
import os
import subprocess
import numpy as np
from .assembly_semantics import _hash, _write
from .task_benchmark_validation import verify_benchmark
from .task_benchmark import save_trajectory
from .task_context import TaskContext, require_resolved
from .task_path_planning import run_task_path
from .base_pose_context import make_context
from ..scene.schema import load_scene
from ..scene.base_pose import BasePose, derive_scene
from ..scene.restricted_scene import restricted_scene, geometry_audit
from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..task.spec import parse_task
from ..task.targets import build_targets,tcp_errors
from ..planning.validity import ConfigurationValidity,assess_task
from ..planning.workspace_constraint import WorkspaceConstraint,TCPWorkspaceCheck
from ..collision.evaluator import Evaluator
from ..collision.semantic import SemanticConfig


def verify_restricted(run, model):
    run=Path(run).resolve()
    manifest=json.loads((run/'manifest.json').read_text())
    if manifest['stage']!='T5.3-A' or manifest['usage']!='RESEARCH_ONLY':
        raise ValueError('restricted research manifest required')
    for name,h in manifest['files'].items():
        path=(run/name).resolve()
        if not path.is_relative_to(run) or _hash(path)!=h:
            raise ValueError('restricted artifact hash/path differs: '+name)
    source=Path(manifest['source_benchmark'])
    if _hash(source/'manifest.json')!=manifest['source_manifest_sha256']:
        raise ValueError('source benchmark anchor differs')
    verify_benchmark(source,model)
    saved=json.loads((source/'ports'/manifest['target']/'result.json').read_text())
    result=json.loads((run/'result.json').read_text())
    start=json.loads((source/'common_start.json').read_text())
    if result['start_q']!=start['q'] or result['source_start_sample_index']!=0:
        raise ValueError('fixed original start differs')
    scene=load_scene(run/'scene_snapshot.json')
    spec=parse_task(json.loads((run/'task_spec.json').read_text()))
    targets=build_targets(spec,scene)
    from .base_pose_context import verify_context
    context=json.loads((run/'base_pose_context.json').read_text())
    verify_context(context,scene,model)
    rule=TCPWorkspaceCheck(WorkspaceConstraint(**result['workspace_config']),context['world_from_base'],spec.T_ee_tcp,context['base_pose'])
    fk=ForwardKinematics(model)
    ledger=json.loads((run/'collision_queries.json').read_text())['records']
    for index,state in enumerate(ledger):
        if state['query_id']!=index:
            raise ValueError('query index differs')
        expected=fk(state['q'])
        np.testing.assert_allclose(expected,state['T_base_to_ee'],atol=1e-10,rtol=0)
        assessment=rule.assess(expected)
        if assessment!=state['workspace'] or state['valid']!=(state['fcl_valid'] and assessment['valid']):
            raise ValueError('workspace/FCL conjunction differs')
    if len(result['goals'])!=len(saved['goals']):
        raise ValueError('historical goal lost')
    checked=0
    for goal,old in zip(result['goals'],saved['goals']):
        if goal['solution_id']!=old['solution_id'] or goal['q']!=old['q']:
            raise ValueError('historical IK goal changed')
        errors=tcp_errors(fk(goal['q'])@spec.T_ee_tcp,targets)
        if errors!=goal['task_check']['errors']:
            raise ValueError('goal task pose differs')
        reports=[*goal['historical_paths'].values(),goal['historical_approach']]
        reports += [t['validation'] for t in goal['paths'] if t.get('validation') is not None]
        for report in reports:
            if report.get('status')=='NOT_RUN':
                continue
            states=report['states']
            if report['valid']!=all(s['valid'] for s in states) or not report['complete']:
                raise ValueError('path classification differs')
            for state in states:
                native=ledger[state['query_id']]
                ignored={'valid','status','reasons'} if 'approach_check' in state else set()
                if any(state.get(k)!=v for k,v in native.items() if k not in ignored):
                    raise ValueError('path ledger differs')
                if 'approach_check' in state:
                    from ..planning.cartesian_approach import check_approach_sample
                    info=state['approach_check']
                    expected=check_approach_sample(native,targets,info['expected_distance_to_port_m'],fk,
                        waypoint_index=info['waypoint_index'],fraction=info['segment_fraction'])
                    if any(expected[k]!=state[k] for k in ('valid','status','reasons','task_errors')):
                        raise ValueError('approach task classification differs')
            checked+=len(states)
        if not result['start_check']['valid'] or not goal['goal_check']['valid'] or not goal['task_check']['task_pose_feasible']:
            if goal['paths']:
                raise ValueError('invalid endpoints entered search')
    return {'status':'PASS','goals_checked':len(result['goals']),'query_records_checked':len(ledger),
            'trajectory_states_checked':checked,'new_native_queries':0,'scope':'saved evidence, authoritative FK and workspace recomputation; discrete only'}


def run_restricted(root, source_run, backend, output, *, allow_candidate=False, expected_source_manifest_sha256=None):
    root=Path(root).resolve();source_run=Path(source_run).resolve();backend=Path(backend).resolve()
    out=Path(output).absolute()
    if os.path.lexists(out):
        raise FileExistsError('refusing existing restricted output')
    if not allow_candidate:
        raise ValueError('explicit candidate research opt-in required')
    if expected_source_manifest_sha256 is None or _hash(source_run/'manifest.json')!=expected_source_manifest_sha256:
        raise ValueError('externally selected source collection Manifest SHA256 required')
    parent=json.loads((source_run/'manifest.json').read_text())
    for name,h in parent['files'].items():
        path=(source_run/name).resolve()
        if not path.is_relative_to(source_run) or _hash(path)!=h:
            raise ValueError('historical source collection changed')
    model=load_model(root/'config/workspace.yaml');fk=ForwardKinematics(model)
    world=load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    config=json.loads((root/'config/restricted_scenes/d0_d4_v0.json').read_text())
    sources={str(p.relative_to(root)):_hash(p) for folder in ('src','scripts','config','cpp') for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
    bindings=[('b0','P22',['D0','D3','D4']),('xm30','P22',['D0','D3','D4']),('ym70','P23',['D1','D2'])]
    out.mkdir(parents=True,exist_ok=False)
    rows=[];manifests={}
    for station,port,labels in bindings:
        source=source_run/'stations'/station/'benchmark'
        verify_benchmark(source,model)
        old=json.loads((source/'ports'/port/'result.json').read_text())
        source_manifest=_hash(source/'manifest.json')
        old_context=json.loads((source/'base_pose_context.json').read_text())
        pose=BasePose.from_dict(old_context['base_pose'])
        original_start=json.loads((source/'common_start.json').read_text())
        if original_start['original_sample_index']!=0:
            raise ValueError('historical G4 sample 0 required')
        spec=parse_task(json.loads((source/'ports'/port/'task_spec.json').read_text()))
        for label,enabled in [(labels[0],False)]+[(label,True) for label in labels]:
            name=station+'_'+label+('_workspace_on' if enabled else '_workspace_off')
            condition=out/name;condition.mkdir()
            generated=restricted_scene(world,label,config)
            audit=geometry_audit(world,generated,port)
            derived=derive_scene(generated,pose)
            context_data=make_context(generated.to_json(),pose,model)
            targets=build_targets(spec,derived)
            old_target=old['targets']
            for key in ('T_base_pre_tcp','T_base_port','insertion_axis_base','key_axis_base'):
                np.testing.assert_allclose(targets.to_dict()[key],old_target[key],atol=1e-12,rtol=0)
            rule=WorkspaceConstraint(enabled=enabled)
            semantic=SemanticConfig.load(root/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
            collision=spec.data['collision']
            with Evaluator(model,derived,None,backend,semantic=semantic,gripper=collision['gripper_rad'],
                    clearance={'self_m':collision['self_clearance_m'],'environment_m':collision['environment_clearance_m']},timeout=60) as evaluator:
                checker=ConfigurationValidity(evaluator,workspace_check=TCPWorkspaceCheck(rule,pose.matrix(),spec.T_ee_tcp,pose.to_dict()))
                context=TaskContext(model,derived,targets,checker)
                context.validate()
                start=checker.check(original_start['q'],fresh=True);require_resolved([start])
                result={'usage':'RESEARCH_ONLY','scene_label':label,'base_pose':pose.to_dict(),'target':port,
                    'workspace_config':rule.to_dict(),'workspace_sha256':rule.sha256,'identity':evaluator.identity,
                    'source_start_sample_index':0,'start_q':original_start['q'],'start_check':start,'goals':[],
                    'multi_start_ik_executed':False,'new_approach_generated':False}
                for goal in old['goals']:
                    q=goal['q'];goal_check=checker.check(q,fresh=True);require_resolved([goal_check])
                    task_check=assess_task(tcp_errors(fk(q)@spec.T_ee_tcp,targets),targets)
                    record={'solution_id':goal['solution_id'],'q':q,'goal_check':goal_check,'task_check':task_check,
                        'historical_paths':{},'historical_approach':{'status':'NOT_RUN'},'paths':[]}
                    # Recheck every saved original path, even when the new endpoints fail.
                    for trial in goal['paths']:
                        if not trial.get('trajectory_file'):
                            continue
                        with np.load(source/'ports'/port/trial['trajectory_file'],allow_pickle=False) as arrays:
                            check=checker.path(arrays['q'],old['planning_settings']['verification_resolution_rad'],fresh=True)
                        require_resolved(check['states'])
                        check['source_artifact']=str(source/'ports'/port/trial['trajectory_file'])
                        record['historical_paths'][trial['method']]=check
                    approach=goal['approach']
                    if approach.get('trajectory_file'):
                        with np.load(source/'ports'/port/approach['trajectory_file'],allow_pickle=False) as arrays:
                            check=checker.path(arrays['q'],old['approach_settings']['edge_resolution_rad'],fresh=True)
                        require_resolved(check['states'])
                        check['source_artifact']=str(source/'ports'/port/approach['trajectory_file'])
                        # Same interpolation grid and existing Cartesian task checks.
                        from ..planning.cartesian_approach import check_approach_sample
                        source_states=approach['samples']
                        task_states=[]
                        for state,provenance in zip(check['states'],check['provenance']):
                            segment=provenance['segment_index']
                            a=np.asarray(source_states[segment]['q'])
                            b=np.asarray(source_states[min(segment+1,len(source_states)-1)]['q'])
                            delta=b-a
                            fraction=float(np.clip(np.dot(np.asarray(state['q'])-a,delta)/np.dot(delta,delta),0,1)) if np.dot(delta,delta)>0 else 0.
                            start_distance=source_states[segment]['approach_check']['expected_distance_to_port_m']
                            end_distance=source_states[min(segment+1,len(source_states)-1)]['approach_check']['expected_distance_to_port_m']
                            distance=start_distance+fraction*(end_distance-start_distance)
                            checked=check_approach_sample(state,targets,distance,fk,waypoint_index=segment,fraction=fraction)
                            checked['source_sample_segment']=segment
                            task_states.append(checked)
                        check['states']=task_states
                        failures=[i for i,s in enumerate(task_states) if not s['valid']]
                        check['valid']=not failures
                        check['first_failure']=None if not failures else {'path_sample_index':failures[0],
                            'q':task_states[failures[0]]['q'],'reasons':task_states[failures[0]]['reasons'],
                            'query_id':task_states[failures[0]]['query_id']}
                        record['historical_approach']=check
                    valid=start['valid'] and goal_check['valid'] and task_check['task_pose_feasible']
                    if valid:
                        for method in ('linear','rrt_connect'):
                            trial=run_task_path(context,start['q'],q,method=method,settings=old['planning_settings'])
                            if trial['validation'] is not None:
                                filename=f"goal{goal['solution_id']}_{method}.npz"
                                save_trajectory(condition/filename,trial['validation']['states'],spec)
                                trial['trajectory_file']=filename
                            record['paths'].append(trial)
                    record['planning_status']='EXECUTED' if valid else 'INVALID_START' if not start['valid'] else 'INVALID_GOAL'
                    result['goals'].append(record)
                    trials={t['method']:t for t in record['paths']}
                    row={'condition':name,'scene':label,'base_pose':station,'port':port,'goal':goal['solution_id'],
                        'workspace_on':enabled,'start_valid':start['valid'],'goal_pose_valid':task_check['task_pose_feasible'],
                        'goal_fcl_valid':goal_check['fcl_valid'],'goal_workspace_valid':goal_check['workspace']['valid'],
                        'historical_linear_valid':record['historical_paths'].get('linear',{}).get('valid'),
                        'historical_rrt_valid':record['historical_paths'].get('rrt_connect',{}).get('valid'),
                        'linear':trials.get('linear',{}).get('status','NOT_RUN'),
                        'rrt':trials.get('rrt_connect',{}).get('status','NOT_RUN'),
                        'approach_valid':record['historical_approach'].get('valid'),
                        'endpoint_status':record['planning_status'],'goal_reasons':goal_check['reasons']}
                    rows.append(row)
                    print(json.dumps(row),flush=True)
                _write(condition/'collision_queries.json',{'identity':evaluator.identity,'records':checker.queries,'statistics':checker.statistics()})
            if not evaluator.client.cleanup['exit_normal']:
                raise RuntimeError('native abnormal exit')
            _write(condition/'result.json',result)
            _write(condition/'scene_snapshot.json',derived.to_dict())
            _write(condition/'world_scene.json',generated.to_dict())
            _write(condition/'base_pose_context.json',context_data)
            _write(condition/'geometry_audit.json',audit)
            _write(condition/'task_spec.json',spec.data)
            _write(condition/'manifest.json',{'stage':'T5.3-A','schema_version':1,'usage':'RESEARCH_ONLY','git_head':head,
                'source_sha256':sources,'source_benchmark':str(source),'source_manifest_sha256':source_manifest,
                'target':port,'scene_sha256':derived.canonical_scene_sha256,'workspace_sha256':rule.sha256,
                'identity':result['identity'],'files':{str(p.relative_to(condition)):_hash(p) for p in condition.rglob('*') if p.is_file()}})
            validation=verify_restricted(condition,model)
            _write(condition/'independent_validation.json',validation)
            _write(condition/'validation_manifest.json',{'manifest_sha256':_hash(condition/'manifest.json'),
                'report_sha256':_hash(condition/'independent_validation.json')})
            manifests[name]=_hash(condition/'manifest.json')
    _write(out/'summary.json',{'usage':'RESEARCH_ONLY','stage':'T5.3-A','rows':rows,'conditions':manifests})
    with (out/'summary.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    for name,h in sources.items():
        if _hash(root/name)!=h:
            raise RuntimeError('source changed during experiment: '+name)
    _write(out/'manifest.json',{'stage':'T5.3-A_COLLECTION','usage':'RESEARCH_ONLY','git_head':head,'conditions':manifests,
        'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})
    return rows
