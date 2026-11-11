"""T1/T2 recorded P22 pre-insertion IK and independent semantic collision screening."""
from hashlib import sha256
import csv
import json
import os
from pathlib import Path
import platform
import subprocess
from time import perf_counter
import numpy as np
import scipy
from .assembly_semantics import _hash,_write
from ..robot.model import load_model
from ..scene.schema import load_scene
from ..workspace.results import load_results
from ..task.spec import load_task,canonical,TaskError
from ..task.targets import build_targets,tcp_errors
from ..planning.ik import NumericalIK,build_seeds
from ..planning.validity import assess_task,screen_solutions
from ..collision.semantic import SemanticConfig
from ..collision.client import CollisionClient
from ..collision.assets import robot_surfaces
from ..viz.task import build_task_figure
from ..viz.collision import export_collision_viewer


def run_p22(root,backend,task_config,seed_prefix,output,*,allow_candidate=False):
    root=Path(root).resolve();out=Path(output).absolute();seed_prefix=Path(seed_prefix).resolve()
    if os.path.lexists(out):raise TaskError('refusing existing P22 output path')
    if not allow_candidate:raise TaskError('explicit allow_candidate required for experimental collision model')
    backend=Path(backend).resolve(strict=True)
    spec=load_task(task_config);model=load_model(root/'config/workspace.yaml');scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml');targets=build_targets(spec,scene)
    if spec.data['target']['port_id']!='P22':raise TaskError('P22 experiment requires P22; no automatic port substitution')
    if model.base_frame!=spec.data['target']['reference_frame'] or model.ee_frame!=spec.data['target']['ee_frame']:raise TaskError('task/model frame mismatch')
    model.joint_positions(model.limits.mean(axis=1),{'r_joint':spec.data['collision']['gripper_rad']})
    arrays,metadata=load_results(seed_prefix,model=model)
    input_hashes={ext:_hash(str(seed_prefix)+ext) for ext in ['.npz','.json']}
    strict=SemanticConfig.load(root/'config/collision/semantic/jetrover_semantic_v0.srdf',model)
    candidate=SemanticConfig.load(root/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    seeds=build_seeds(model,targets,arrays)
    out.mkdir(parents=True);(out/'figures').mkdir();_write(out/'started.json',{'usage':'RESEARCH_ONLY','status':'STARTED'})
    for name,text in [('task_config.yaml',Path(task_config).read_text()),('source_robot.urdf',model.urdf_xml),('scene_snapshot.json',scene.to_json())]:
        with (out/name).open('x') as f:f.write(text)
    _write(out/'task_spec.json',spec.data);_write(out/'targets.json',targets.to_dict());_write(out/'model_metadata.json',model.metadata());_write(out/'seeds.json',seeds)
    print(json.dumps({'stage':'T1','port_position_m':list(targets.base_to_port.position_m),'pre_tcp_position_m':list(targets.pre_tcp.position_m),'seed_count':len(seeds),'usage':'RESEARCH_ONLY'}),flush=True)
    def progress(value):
        if value['completed_seeds']%5==0 or value['completed_seeds']==value['total_seeds']:print(json.dumps({'stage':'IK',**value}),flush=True)
    ik=NumericalIK(model,targets).solve(seeds,progress=progress)
    _write(out/'ik_results.json',ik)
    with (out/'ik_attempts.csv').open('x',newline='') as f:
        fields=['attempt_id','seed_id','stage','initial_q','q','optimizer_success','optimizer_status','nfev','njev','residual_calls_including_numeric_jacobian','iterations','position_mm','insertion_axis_deg','key_axis_deg','task_status','elapsed_s'];writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for a in ik['attempts']:
            row={k:a.get(k) for k in fields};row.update({k:a['errors'][k] for k in ['position_mm','insertion_axis_deg','key_axis_deg']});row['task_status']=a['task_check']['status']
            row['initial_q']=json.dumps(row['initial_q']);row['q']=json.dumps(row['q']);writer.writerow(row)
    solutions=ik['unique_task_solutions'];collision={};latency={}
    for name,semantic in [('strict',strict),('candidate',candidate)]:
        begin=perf_counter();collision[name]=screen_solutions(model,scene,targets,solutions,semantic,backend);latency[name]=perf_counter()-begin
        _write(out/(name+'_collision.json'),collision[name])
    candidates=[r for r in collision['candidate']['results'] if r['task_check']['task_configuration_candidate']]
    by_id={s['solution_id']:s for s in solutions}
    if candidates:
        selected=min(candidates,key=lambda r:ik['attempts'][by_id[r['solution_id']]['best_attempt_id']]['normalized_task_score'])
        best_check=selected['task_check'];selected_id=selected['solution_id'];selected_q=selected['q']
    elif solutions:
        selected=collision['candidate']['results'][0];best_check=selected['task_check'];selected_id=selected['solution_id'];selected_q=selected['q']
    else:
        selected=ik['best_attempt'];best_check=selected['task_check'];selected_id=None;selected_q=selected['q']
    # Geometry queries below are explicitly diagnostic when no task pose passes.
    roles=[('candidate_best' if candidates else 'candidate_pose_rejected' if solutions else 'best_approximation','candidate',selected_q,best_check)]
    if solutions:
        strict_check=next(r['task_check'] for r in collision['strict']['results'] if r['solution_id']==selected_id)
        roles.append(('strict_same_best','strict',selected_q,strict_check))
        rejected=next((r for r in collision['candidate']['results'] if r['task_check']['collision_status']=='REJECTED' and r['q']!=selected_q),None)
        if rejected:roles.append(('candidate_collision_rejected','candidate',rejected['q'],rejected['task_check']))
    partial=next((a for a in sorted(ik['attempts'],key=lambda a:(-a['constraint_tier'],a['normalized_task_score'])) if not a['task_check']['task_pose_feasible'] and a['q']!=selected_q),None)
    if partial:roles.append(('partial_task_pose_diagnostic','candidate',partial['q'],partial['task_check']))
    viewers=[];ready={};cleanup={}
    for name,sem in [('strict',strict),('candidate',candidate)]:
        relevant=[r for r in roles if r[1]==name]
        if not relevant:continue
        with CollisionClient(backend,timeout=60) as client:
            ready[name]=client.initialize(model,scene,semantic=sem);_write(out/(name+'_viewer_ready.json'),ready[name])
            for role,_,q,assessment in relevant:
                row=client.query(model,q,auxiliary_positions={'r_joint':spec.data['collision']['gripper_rad']});row.update(q=q,sample_index=role,task_check=assessment)
                actual=np.array(row['T_base_to_ee'])@spec.T_ee_tcp;errors=tcp_errors(actual,targets)
                for key in ['position_m','insertion_axis_rad','key_axis_rad']:
                    if abs(errors[key]-assessment['errors'][key])>1e-9:raise TaskError('native/FK task error mismatch')
                if assessment['task_pose_feasible']:
                    expected=next((r['collision'] for r in collision[name]['results'] if r['q']==q),None)
                    if expected is not None:
                        for key in ['semantic_self_collision','environment_collision','semantic_blocking_pairs']:
                            if expected[key]!=row[key]:raise TaskError('representative collision requery differs')
                fig=build_task_figure(scene,robot_surfaces(model,ready[name],row),row,targets,assessment);file='figures/'+role+'.html';export_collision_viewer(fig,row,out/file)
                viewers.append({'role':role,'collision_model':name,'q':q,'task_check':assessment,'native_query':row,'T_base_tcp':actual.tolist(),'file':file,'render_diagnostics_only':not assessment['task_pose_feasible']})
        cleanup[name]=client.cleanup
        if not client.cleanup['exit_normal']:raise TaskError('viewer geometry backend exited abnormally')
    _write(out/'viewers.json',viewers)
    counters={}
    for name,result in collision.items():
        rows=result['results'];counters[name]={'task_pose_solutions_checked':result['collision_queries'],'task_configuration_candidates':sum(r['task_check']['task_configuration_candidate'] for r in rows),
            'semantic_self_collision':sum(r['collision'] is not None and r['collision'].get('semantic_self_collision') is True for r in rows),
            'environment_collision':sum(r['collision'] is not None and r['collision'].get('environment_collision') is True for r in rows),
            'rejected':sum(r['task_check']['collision_status']=='REJECTED' for r in rows),'unverified_or_error':sum(r['task_check']['collision_status'] in ['UNVERIFIED','ERROR'] for r in rows),
            'collision_screening_wall_s':latency[name],'usage':'RESEARCH_ONLY'}
    summary={'usage':'RESEARCH_ONLY','task_id':spec.data['id'],'final_status':best_check['status'],'final_task_check':best_check,'selected_solution_id':selected_id,'selected_q':selected_q,
        'seed_count':ik['seed_count'],'optimization_attempts':ik['optimization_count'],
        'position_reachable_attempts':sum(a['task_check']['position_reachable'] for a in ik['attempts']),
        'axis_aligned_attempts':sum(a['task_check']['axis_aligned'] for a in ik['attempts']),
        'task_pose_feasible_attempts':sum(a['task_check']['task_pose_feasible'] for a in ik['attempts']),
        'unique_task_pose_solutions':len(solutions),'collision_comparison':counters,'ik_elapsed_s':ik['total_elapsed_s'],
        'best_ik_attempt_id':ik['best_attempt']['attempt_id'],'best_ik_errors':ik['best_attempt']['errors'],'no_solution_reason':ik['no_solution_reason'],
        'task_identity':{'task_sha256':spec.sha256,'tcp_sha256':spec.tcp_sha256,'raw_task_sha256':spec.raw_sha256,'scene_sha256':scene.canonical_scene_sha256,'urdf_sha256':model.metadata()['expanded_urdf_sha256']},
        'synthetic_uncalibrated':True,'hardware_validated':False,'planning_approved':False,'tool_cable_collision_geometry_modeled':False,
        'mathematical_unreachability_proven':False,'trajectory_checked':False,'viewers':[{'role':v['role'],'file':v['file']} for v in viewers]}
    _write(out/'summary.json',summary)
    if _hash(task_config)!=spec.raw_sha256:raise TaskError('task parameters changed during experiment')
    for ext,h in input_hashes.items():
        if _hash(str(seed_prefix)+ext)!=h:raise TaskError('seed workspace changed')
    sources={str(p.relative_to(root)):_hash(p) for name in ['src','scripts','cpp','config'] for p in (root/name).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    _write(out/'manifest.json',{'status':'COMPLETED','usage':'RESEARCH_ONLY','git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
        'task_identity':summary['task_identity'],'solver_parameters_sha256':sha256(canonical(spec.data['solver']).encode()).hexdigest(),'collision_settings':spec.data['collision'],
        'collision_identities':{k:v['identity'] for k,v in ready.items()},'collision_screening_identities':{k:v['identity'] for k,v in collision.items()},'viewer_cleanup':cleanup,
        'input_prefix':str(seed_prefix),'input_hashes':input_hashes,'source_sha256':sources,'software':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__},
        'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})
    return summary


def verify_p22(output,model):
    """Verify saved bytes, original seeds, FK and all reported task thresholds."""
    out=Path(output).resolve();manifest=json.loads((out/'manifest.json').read_text())
    if manifest.get('status')!='COMPLETED' or manifest.get('usage')!='RESEARCH_ONLY':raise TaskError('not a completed research task run')
    for name,h in manifest['files'].items():
        p=(out/name).resolve()
        if not p.is_relative_to(out) or _hash(p)!=h:raise TaskError('artifact identity/path changed: '+name)
    spec=load_task(out/'task_config.yaml');scene=load_scene(Path(__file__).resolve().parents[3]/'config/scenes/idc_prototype_v0.yaml');targets=build_targets(spec,scene)
    if spec.sha256!=manifest['task_identity']['task_sha256'] or spec.tcp_sha256!=manifest['task_identity']['tcp_sha256'] or scene.canonical_scene_sha256!=manifest['task_identity']['scene_sha256'] or model.metadata()['expanded_urdf_sha256']!=manifest['task_identity']['urdf_sha256']:raise TaskError('verification task/model/scene identity mismatch')
    arrays,_=load_results(manifest['input_prefix'],model=model)
    for ext,h in manifest['input_hashes'].items():
        if _hash(manifest['input_prefix']+ext)!=h:raise TaskError('input workspace identity changed')
    seeds=json.loads((out/'seeds.json').read_text())
    for seed in seeds:
        if seed['source']=='G4_nearest_virtual_TCP' and seed['q']!=arrays['q'][seed['workspace_sample_index']].tolist():raise TaskError('G4 seed/index changed')
    from ..robot.fk import ForwardKinematics
    fk=ForwardKinematics(model);ik=json.loads((out/'ik_results.json').read_text())
    for attempt in ik['attempts']:
        error=tcp_errors(fk(attempt['q'])@spec.T_ee_tcp,targets);check=assess_task(error,targets)
        for key in ['position_m','insertion_axis_rad','key_axis_rad']:
            if abs(error[key]-attempt['errors'][key])>1e-10:raise TaskError('saved IK/FK task metric differs')
        if check['status']!=attempt['task_check']['status']:raise TaskError('saved task classification differs')
    return {'status':'PASS','usage':'RESEARCH_ONLY','files':len(manifest['files']),'seeds':len(seeds),'optimizations':len(ik['attempts']),'all_task_errors_rechecked':True}
