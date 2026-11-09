"""Uniform shortcut postprocessing of all accepted main/local RRT observations."""
from pathlib import Path
import json
import os
import subprocess
from .assembly_semantics import _hash,_write
from .robustness_benchmark import collection,source_hashes,semantic_for
from .multistart_benchmark import bound_files,evaluator_for
from .path_trial_artifacts import (SCENE_FILES,load_context,compress_report,store_ledger,
    finish_trial,trial_row,verify_path_trial)
from .ready_pose import ready_digest
from .task_benchmark import save_trajectory
from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..planning.validity import ConfigurationValidity,assess_task
from ..planning.path_shortcut import randomized_shortcut
from ..planning.path_quality import path_quality
from ..task.targets import tcp_errors


def run_shortcuts(root,search_run,backend,output,*,expected_search_manifest_sha256,allow_candidate=False,shortcut_config=None,workers=1):
    root=Path(root).resolve();search_run=Path(search_run).resolve();out=Path(output).absolute();backend=Path(backend).resolve(strict=True)
    if type(workers) is not int or not 1<=workers<=8:raise ValueError('workers must be an integer in [1,8]')
    if os.path.lexists(out):raise FileExistsError('refusing existing shortcut output')
    if not allow_candidate:raise ValueError('explicit research candidate opt-in required')
    if _hash(search_run/'manifest.json')!=expected_search_manifest_sha256:raise ValueError('externally selected search manifest SHA256 required')
    manifest=json.loads((search_run/'manifest.json').read_text());bound_files(search_run,manifest)
    if manifest['stage']!='T5.3-C_SEARCH':raise ValueError('completed search collection required')
    config=json.loads((search_run/'settings.json').read_text())['shortcut'] if shortcut_config is None else json.loads(Path(shortcut_config).read_text())
    source_rows=json.loads((search_run/'summary.json').read_text())['rows']
    model=load_model(root/'config/workspace.yaml');fk=ForwardKinematics(model);semantic=semantic_for(root,model)
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip();hashes=source_hashes(root)
    out.mkdir(parents=True,exist_ok=False);rows=[];index=[]
    _write(out/'shortcut_settings.json',config)
    work=[(position,r) for position,r in enumerate(source_rows) if r['success']]
    results={}
    def arguments(row):
        return (str(root),row['trial'],str(out),str(backend),config,head,hashes,workers)
    if workers==1:
        for position,row in work:results[position]=_optimize_one(arguments(row))
    else:
        from concurrent.futures import ProcessPoolExecutor,wait,FIRST_COMPLETED
        from multiprocessing import get_context
        executor=ProcessPoolExecutor(max_workers=workers,mp_context=get_context('spawn'))
        iterator=iter(work);active={}
        try:
            def submit_next():
                item=next(iterator,None)
                if item is not None:
                    position,row=item;active[executor.submit(_optimize_one,arguments(row))]=position
            for _ in range(workers):submit_next()
            while active:
                done,_=wait(active,return_when=FIRST_COMPLETED)
                for future in done:
                    position=active.pop(future);results[position]=future.result();submit_next()
        except BaseException:
            for future in active:future.cancel()
            executor.shutdown(wait=True,cancel_futures=True)
            raise
        else:executor.shutdown(wait=True)
    for position,source_row in enumerate(source_rows):
        if position in results:
            row,entry=results[position];rows.append(row);index.append(entry)
        else:
            raw=Path(source_row['trial'])
            index.append({'raw_trial':str(raw),'raw_manifest_sha256':_hash(raw/'manifest.json'),
                'status':'NOT_RUN_NO_VALID_RAW_PATH','optimized_trial':None})
    _write(out/'optimization_index.json',{'rows':index,'raw_trials_including_failures':len(source_rows)})
    collection(out,'T5.3-C_SHORTCUT',rows,{'search_run':str(search_run),'search_manifest_sha256':expected_search_manifest_sha256,
        'source_denominator':len(source_rows),'eligible_raw_paths':sum(r['success'] for r in source_rows),
        'ineligible_recorded':sum(not r['success'] for r in source_rows),'shortcut_settings':config,'execution_workers':workers,'timing_scope':'postprocess wall times measured under declared process-pool load; raw search remains serial'})
    return rows


def _optimize_one(args):
    root,raw_path,out,backend,config,head,hashes,workers=args
    root=Path(root);raw=Path(raw_path);out=Path(out);backend=Path(backend)
    model=load_model(root/'config/workspace.yaml');fk=ForwardKinematics(model);semantic=semantic_for(root,model)
    verify_path_trial(raw,model)
    inputs=json.loads((raw/'inputs.json').read_text());inputs.pop('sha256')
    inputs['experiment']='shortcut';inputs['sha256']=ready_digest(inputs)
    source=json.loads((raw/'result.json').read_text());original=source['trial']
    destination=out/(raw.parent.name+'_'+raw.name);destination.mkdir(exist_ok=False)
    for name in SCENE_FILES:(destination/name).write_bytes((raw/name).read_bytes())
    scene,spec,_,rule,targets=load_context(root,inputs,model)
    with evaluator_for(model,scene,spec,backend,semantic) as evaluator:
        if evaluator.identity!=inputs['expected_native_identity']:raise ValueError('postprocess Native identity differs')
        checker=ConfigurationValidity(evaluator,workspace_check=rule)
        trial=randomized_shortcut(original['waypoints'],checker,
            resolution_rad=inputs['planning_settings']['verification_resolution_rad'],
            seed=config['random_seed'],max_attempts=config['max_attempts'],
            reduction_epsilon_rad=config['reduction_epsilon_rad'])
        trial.update(method='rrt_connect',search=original['search'],search_elapsed_s=original['search_elapsed_s'])
        final=trial['validation'];states=[] if final is None else final['states']
        trial['quality']=path_quality(trial,[s['workspace']['world_tcp_xyz_m'] for s in states])
        if final is not None:
            save_trajectory(destination/'trajectory.npz',states,spec);trial['trajectory_file']='trajectory.npz'
        trial['validation']=compress_report(final)
        trial['input_validation']=compress_report(trial['input_validation'])
        for field in ('fallback_validation','candidate_final_validation'):
            if trial.get(field) is not None:trial[field]=compress_report(trial[field])
        for attempt in trial['attempts']:
            attempt['edge_validation']=compress_report(attempt['edge_validation'])
        result={'variant':'shortcut','usage':'RESEARCH_ONLY','trial':trial,'identity':evaluator.identity,
            'source_raw_trial':str(raw),'source_raw_manifest_sha256':_hash(raw/'manifest.json'),
            'raw_quality':original['quality'],'raw_total_time_s':original['elapsed_s'],
            'total_time_s':original['elapsed_s']+trial['elapsed_s'],
            'total_time_scope':'raw planning including raw final check + postprocess including fresh input/final; excludes model init and artifact IO',
            'task_check':assess_task(tcp_errors(fk(inputs['goal_q'])@spec.T_ee_tcp,targets),targets),
            'workspace_rejections':sum(any(c.startswith('WORKSPACE_') for c in s['reasons']) for s in checker.queries),
            'hardware_validated':False,'continuous_collision_proven':False}
        store_ledger(destination/'collision_queries.json.gz',checker,evaluator.identity)
    if not evaluator.client.cleanup['exit_normal']:raise RuntimeError('postprocess Native exit abnormal')
    result['cleanup']=evaluator.client.cleanup
    result['execution_workers']=workers
    result['timing_execution_scope']='postprocess wall time under declared independent process pool; not single-worker latency'
    finish_trial(destination,inputs,result,model,hashes,head)
    row=trial_row(destination,inputs,result);row['execution_workers']=workers
    print(json.dumps(row),flush=True)
    return row,{'raw_trial':str(raw),'raw_manifest_sha256':_hash(raw/'manifest.json'),
        'status':trial['status'],'optimized_trial':str(destination),'optimized_manifest_sha256':_hash(destination/'manifest.json')}
