"""T5.3-C fixed-input multi-seed search. Original RRT/validity remain unchanged."""
from pathlib import Path
import csv
import json
import os
import subprocess
import numpy as np
from .assembly_semantics import _hash,_write
from .multistart_benchmark import bound_files,evaluator_for,compact,verify_multistart
from .path_trial_artifacts import (reference_inputs,load_context,SCENE_FILES,compress_report,
    store_ledger,finish_trial,trial_row)
from .task_context import TaskContext,require_resolved,TaskSystemError
from .task_path_planning import run_task_path
from .task_benchmark import save_trajectory
from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..task.targets import tcp_errors
from ..planning.validity import ConfigurationValidity,assess_task
from ..planning.path_quality import path_quality
from ..collision.semantic import SemanticConfig


def source_hashes(root):
    return {str(p.relative_to(root)):_hash(p) for folder in ('src','scripts','config','ros2')
            for p in (root/folder).rglob('*') if p.is_file() and not p.is_symlink() and '__pycache__' not in str(p) and '.egg-info' not in str(p)}


def semantic_for(root,model):
    return SemanticConfig.load(root/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)


def collection(out,stage,rows,extra=None):
    _write(out/'summary.json',{'stage':stage,'usage':'RESEARCH_ONLY','rows':rows,**(extra or {})})
    if rows:
        with (out/'summary.csv').open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    _write(out/'manifest.json',{'stage':stage,'usage':'RESEARCH_ONLY',
        'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})


def run_trial(root,inputs,backend,out,model,semantic,head,hashes):
    out=Path(out);out.mkdir(parents=True,exist_ok=False)
    source=Path(inputs['source_condition'])
    for name in SCENE_FILES:(out/name).write_bytes((source/name).read_bytes())
    scene,spec,_,rule,targets=load_context(root,inputs,model);fk=ForwardKinematics(model)
    result={'variant':'preparation' if inputs['experiment']=='preparation' else 'raw_rrt','usage':'RESEARCH_ONLY',
        'task_check':None,'hardware_validated':False,'continuous_collision_proven':False}
    with evaluator_for(model,scene,spec,backend,semantic) as evaluator:
        if evaluator.identity!=inputs['expected_native_identity']:raise TaskSystemError('Native identity differs from fixed B evidence')
        checker=ConfigurationValidity(evaluator,workspace_check=rule);context=TaskContext(model,scene,targets,checker)
        context.validate()
        if inputs['goal_role']=='PRE_INSERTION':
            result['task_check']=assess_task(tcp_errors(fk(inputs['goal_q'])@spec.T_ee_tcp,targets),targets)
        try:
            start,goal=[checker.check(q,fresh=True) for q in (inputs['start_q'],inputs['goal_q'])]
            require_resolved([start,goal]);result['start_check']=compact(start);result['goal_check']=compact(goal)
            invalid='INVALID_START' if not start['valid'] else 'INVALID_GOAL' if not goal['valid'] or (result['task_check'] is not None and not result['task_check']['task_pose_feasible']) else None
            if invalid:
                trial={'method':inputs['method'],'status':'NOT_RUN','reason':invalid,'success':False,
                    'search':None,'validation':None,'waypoints':None,'quality':{'status':'NOT_RUN'},'settings':inputs['planning_settings']}
            else:
                trial=run_task_path(context,inputs['start_q'],inputs['goal_q'],method=inputs['method'],settings=inputs['planning_settings'])
                states=[] if trial['validation'] is None else trial['validation']['states']
                trial['quality']=path_quality(trial,[s['workspace']['world_tcp_xyz_m'] for s in states])
                if trial['validation'] is not None:
                    save_trajectory(out/'trajectory.npz',states,spec);trial['trajectory_file']='trajectory.npz'
                    trial['validation']=compress_report(trial['validation'])
        except TaskSystemError as exc:
            trial={'method':inputs['method'],'status':'BACKEND_ERROR','success':False,'search':None,
                'validation':None,'waypoints':None,'quality':{'status':'BACKEND_ERROR'},'settings':inputs['planning_settings'],
                'error':str(exc)}
        result.update(trial=trial,identity=evaluator.identity,
            workspace_rejections=sum(any(c.startswith('WORKSPACE_') for c in s['reasons']) for s in checker.queries))
        store_ledger(out/'collision_queries.json.gz',checker,evaluator.identity)
    result['cleanup']=evaluator.client.cleanup
    if not result['cleanup']['exit_normal']:
        trial['status']='BACKEND_ERROR';trial['success']=False;trial['error']='abnormal Native cleanup'
    finish_trial(out,inputs,result,model,hashes,head)
    row=trial_row(out,inputs,result);print(json.dumps(row),flush=True)
    return row


def run_search(root,source_run,backend,output,*,expected_source_manifest_sha256,allow_candidate=False):
    root=Path(root).resolve();source_run=Path(source_run).resolve();out=Path(output).absolute();backend=Path(backend).resolve(strict=True)
    if os.path.lexists(out):raise FileExistsError('refusing existing T5.3-C search output')
    if not allow_candidate:raise ValueError('explicit candidate opt-in required')
    if _hash(source_run/'manifest.json')!=expected_source_manifest_sha256:raise ValueError('externally selected B collection SHA256 required')
    parent=json.loads((source_run/'manifest.json').read_text());bound_files(source_run,parent)
    if parent['stage']!='T5.3-B_COLLECTION':raise ValueError('completed B collection required')
    config=json.loads((root/'config/restricted_scenes/robustness_v0.json').read_text())
    if len(config['seeds'])!=10 or len(set(config['seeds']))!=10:raise ValueError('ten distinct preregistered seeds required')
    model=load_model(root/'config/workspace.yaml');semantic=semantic_for(root,model)
    plan=[];verified=set()
    for case in config['cases']:
        for start in config['starts']:
            condition=case['source_prefix']+'_'+start+'_conditional'
            if condition not in verified:verify_multistart(source_run/condition,model);verified.add(condition)
            for seed in config['seeds']:
                plan.append((f"main/{case['id']}_{start}_seed{seed}",reference_inputs(source_run,condition,
                    goal_id=case['goal'],experiment='main',case_id=case['id'],seed=seed)))
    local=config['supplement'];condition=local['source_prefix']+'_'+local['start']+'_local_only'
    verify_multistart(source_run/condition,model)
    for seed in config['seeds']:
        plan.append((f"local/LOCAL_S2_seed{seed}",reference_inputs(source_run,condition,
            goal_id=local['goal'],experiment='local',case_id='LOCAL',seed=seed)))
    if sum(inp['experiment']=='main' for _,inp in plan)!=90:raise ValueError('main denominator must be 90')
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip();hashes=source_hashes(root)
    out.mkdir(parents=True,exist_ok=False)
    _write(out/'settings.json',config);_write(out/'preregistered_trials.json',{'trials':[{'path':n,'inputs':i} for n,i in plan]})
    rows=[]
    for name,inputs in plan:
        path=out/name;path.parent.mkdir(exist_ok=True)
        row=run_trial(root,inputs,backend,path,model,semantic,head,hashes);rows.append(row)
        if row['termination']=='BACKEND_ERROR':
            collection(out,'T5.3-C_SEARCH_BLOCKED',rows,{'scheduled':len(plan),'not_run':len(plan)-len(rows),'blocked':'Native backend error'})
            raise RuntimeError('Native error; remaining dependent searches NOT_RUN')
    main=[r for r in rows if r['experiment']=='main']
    representatives=[]
    for case in config['cases']:
        failed=next((r for r in main if r['case']==case['id'] and not r['success'] and r['termination'] in ('TIME_LIMIT','ITERATION_LIMIT','PATH_VALIDATION_FAILURE')),None)
        if failed is not None:representatives.append(failed)
    budget_rows=[]
    for row in representatives[:config['budget_sensitivity']['maximum_representatives']]:
        # The 20s result is the main observation, not an independent duplicate trial.
        budget_rows.append({**row,'observation_role':'REUSED_MAIN_20S'})
        prior=json.loads((Path(row['trial'])/'inputs.json').read_text())
        for seconds in (5.,60.):
            inputs=reference_inputs(source_run,Path(prior['source_condition']).name,goal_id=prior['goal_id'],
                experiment='budget',case_id=prior['case_id'],seed=prior['planning_settings']['random_seed'],budget_s=seconds)
            path=out/'budget'/f"{inputs['case_id']}_{inputs['start_id']}_seed{inputs['planning_settings']['random_seed']}_t{int(seconds)}"
            path.parent.mkdir(exist_ok=True)
            value=run_trial(root,inputs,backend,path,model,semantic,head,hashes)
            budget_rows.append({**value,'observation_role':'ADDITIONAL_TIME_BUDGET'})
            if value['termination']=='BACKEND_ERROR':raise RuntimeError('budget Native error')
    _write(out/'budget_comparison.json',{'rows':budget_rows,'selection_rule':config['budget_sensitivity'],
        'note':'20s rows reused, not additional independent trials; max_iterations remains 1500'})
    for name,digest in hashes.items():
        if _hash(root/name)!=digest:raise RuntimeError('existing source changed during search: '+name)
    collection(out,'T5.3-C_SEARCH',rows,{'main_denominator':90,'local_denominator':10,
        'budget_additional_trials':sum(r['observation_role']=='ADDITIONAL_TIME_BUDGET' for r in budget_rows),
        'source_run':str(source_run),'source_manifest_sha256':expected_source_manifest_sha256})
    return rows
