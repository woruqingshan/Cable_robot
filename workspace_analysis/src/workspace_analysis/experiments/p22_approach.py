"""T4 P22 free-space 40->10 mm approach with unchanged T2/T3 provenance."""
from copy import deepcopy
from hashlib import sha256
import csv
import json
import os
from pathlib import Path
import subprocess
import numpy as np
from .assembly_semantics import _hash, _write
from .p22_reachability import verify_p22
from .p22_path_planning import verify_p22_path
from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..scene.schema import load_scene, parse_scene
from ..task.spec import load_task, canonical
from ..task.targets import build_targets, tcp_errors
from ..planning.validity import ConfigurationValidity, assess_task
from ..planning.joint_path import PathError, interpolate
from ..planning.cartesian_approach import approach_settings, cartesian_waypoints, sequential_approach, check_approach_sample
from ..collision.evaluator import Evaluator
from ..collision.semantic import SemanticConfig
from ..collision.assets import robot_surfaces
from ..viz.approach import export_approach_viewer


def run_p22_approach(root, backend, ik_run, path_run, output, *, allow_candidate=False, settings=None):
    root=Path(root).resolve();out=Path(output).absolute();ik_run=Path(ik_run).resolve();path_run=Path(path_run).resolve()
    if os.path.lexists(out): raise PathError('refusing existing T4 output path')
    if not allow_candidate: raise PathError('explicit allow_candidate required for RESEARCH_ONLY SRDF')
    params=approach_settings(settings)
    if params['end_distance_m']<.010: raise PathError('T4 main experiment stops at >=10 mm; no insertion or contact permission')
    model=load_model(root/'config/workspace.yaml');scene=load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    t2_check=verify_p22(ik_run,model);t3_check=verify_p22_path(path_run,model)
    path_manifest=json.loads((path_run/'manifest.json').read_text())
    if Path(path_manifest['ik_run']).resolve()!=ik_run: raise PathError('T2/T3 input run identity differs')
    spec=load_task(ik_run/'task_config.yaml');targets=build_targets(spec,scene)
    if spec.data['target']['port_id']!='P22': raise PathError('P22 experiment requires unchanged P22 target')
    if path_manifest['task_sha256']!=spec.sha256 or path_manifest['tcp_sha256']!=spec.tcp_sha256 or path_manifest['scene_sha256']!=scene.canonical_scene_sha256: raise PathError('task/TCP/scene mismatch with T3')
    points=cartesian_waypoints(targets,params['end_distance_m'],params['cartesian_step_m'])
    solutions=json.loads((ik_run/'ik_results.json').read_text())['unique_task_solutions']
    if len(solutions)!=2: raise PathError('expected two unchanged T2 branches')
    inputs={str(p):_hash(p) for folder in [ik_run,path_run] for p in [folder/'manifest.json']}
    for name in ['task_config.yaml','ik_results.json']:
        inputs[str(ik_run/name)]=_hash(ik_run/name)
    for solution in solutions:
        label='q'+str(solution['solution_id']);path=path_run/('paths/'+label+'_linear.json')
        path_data=json.loads(path.read_text())
        if not path_data['success'] or path_data['waypoints'][-1]!=solution['q']: raise PathError('T3 endpoint does not match original T2 q')
        inputs[str(path)]=_hash(path)
    semantic=SemanticConfig.load(root/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    backend=Path(backend).resolve(strict=True)
    out.mkdir(parents=True);(out/'branches').mkdir();(out/'figures').mkdir()
    _write(out/'started.json',{'stage':'T4','status':'STARTED','usage':'RESEARCH_ONLY'})
    _write(out/'settings.json',params);_write(out/'source_verification.json',{'T2':t2_check,'T3':t3_check})
    _write(out/'cartesian_targets.json',[{'waypoint_index':p['waypoint_index'],'distance_to_port_m':p['distance_to_port_m'],'target':p['target'].to_dict()} for p in points])
    _write(out/'model_metadata.json',model.metadata());_write(out/'goals_from_T2.json',solutions)
    for name,text in [('task_config.yaml',(ik_run/'task_config.yaml').read_text()),('scene_snapshot.json',scene.to_json()),('source_robot.urdf',model.urdf_xml)]:
        with (out/name).open('x') as f:f.write(text)
    semantic_dir=root/'config/collision/semantic/standard_moveit_candidate_v0'
    for name in ['standard_moveit_candidate_v0.srdf','semantic_manifest.json','candidate_source.json']:
        with (out/name).open('xb') as f:f.write((semantic_dir/name).read_bytes())
    comparison=[]
    with Evaluator(model,scene,None,backend,semantic=semantic,gripper=spec.data['collision']['gripper_rad'],
                   clearance={'self_m':params['self_clearance_m'],'environment_m':params['environment_clearance_m']},timeout=60) as evaluator:
        checker=ConfigurationValidity(evaluator)
        for solution in solutions:
            label='q'+str(solution['solution_id']);checker.cache.clear()
            result=sequential_approach(model,targets,solution['q'],checker,params)
            result['solution_id']=solution['solution_id'];result['source_T3_path']=str(path_run/('paths/'+label+'_linear.json'))
            viewer=None
            replay=result['samples'] if result['first_failed_sample_index'] is None else result['samples'][:result['first_failed_sample_index']+1]
            if replay and all('link_transforms_base' in s for s in replay):
                checker.check(solution['q'],fresh=True);native=deepcopy(evaluator.last_native)
                native.update(q=solution['q'],sample_index=label+'_T4_approach')
                initial=assess_task(tcp_errors(np.asarray(native['T_base_to_ee'])@spec.T_ee_tcp,targets),targets)
                viewer='figures/'+label+'_approach.html'
                export_approach_viewer(scene,robot_surfaces(model,evaluator.ready,native),native,targets,initial,result,out/viewer,label=label+' T4 — '+result['status'])
            result['viewer']=viewer
            if viewer is None: result['viewer_unavailable_reason']='No trustworthy native state available; not replaced by invented transforms'
            _write(out/('branches/'+label+'.json'),result)
            states=result['samples'];n=len(states)
            arrays={'q':np.array([s['q'] for s in states]).reshape(n,5),
                'T_base_tcp':np.array([s['actual_T_base_tcp'] if s['actual_T_base_tcp'] is not None else np.full((4,4),np.nan) for s in states]).reshape(n,4,4),
                'expected_T_base_tcp':np.array([s['expected_T_base_tcp'] for s in states]).reshape(n,4,4),
                'valid':np.array([s['valid'] for s in states],dtype=bool),'configuration_valid':np.array([s['configuration_valid'] for s in states],dtype=bool),
                'query_id':np.array([-1 if s['query_id'] is None else s['query_id'] for s in states],dtype=np.int64),
                'waypoint_index':np.array([s['approach_check']['waypoint_index'] for s in states],dtype=np.int64),
                'segment_fraction':np.array([s['approach_check']['segment_fraction'] for s in states])}
            for key in ['expected_distance_to_port_m','actual_axial_distance_to_port_m','cross_track_error_m','signed_along_track_error_m']:
                arrays[key]=np.array([s['approach_check'][key] for s in states],dtype=float)
            for key in ['position_m','insertion_axis_rad','key_axis_rad']:
                arrays[key]=np.array([None if s['task_errors'] is None else s['task_errors'][key] for s in states],dtype=float)
            for kind in ['self','environment']:
                arrays[kind+'_clearance_m']=np.array([(s.get('semantic_clearance') or {}).get('native',{}).get(kind,{}).get('separation_m') for s in states],dtype=float)
            with (out/('branches/'+label+'.npz')).open('xb') as f:np.savez_compressed(f,**arrays)
            with (out/('branches/'+label+'_waypoints.csv')).open('x',newline='') as f:
                fields=['waypoint_index','distance_mm','status','accepted','q','position_mm','axis_deg','key_deg','max_joint_delta_rad','configuration_valid','semantic_self_collision','environment_collision','self_clearance_mm','environment_clearance_mm'];w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
                for point in result['waypoints']:
                    selected=point.get('selected');errors=None if selected is None else selected['errors']
                    endpoint=point.get('endpoint_check',{});clearance=(endpoint.get('semantic_clearance') or {}).get('native',{})
                    w.writerow({'waypoint_index':point['waypoint_index'],'distance_mm':1000*point['distance_to_port_m'],'status':point['status'],'accepted':point['accepted'],
                        'q':None if selected is None else json.dumps(selected['q']),'position_mm':None if errors is None else errors['position_mm'],
                        'axis_deg':None if errors is None else errors['insertion_axis_deg'],'key_deg':None if errors is None else errors['key_axis_deg'],
                        'max_joint_delta_rad':None if selected is None else selected['max_joint_delta_rad'],
                        **{key:endpoint.get(key) for key in ['configuration_valid','semantic_self_collision','environment_collision']},
                        **{kind+'_clearance_mm':None if clearance.get(kind,{}).get('separation_m') is None else 1000*clearance[kind]['separation_m'] for kind in ['self','environment']}})
            row={k:result[k] for k in ['solution_id','status','success','requested_waypoints','accepted_waypoints','last_accepted_waypoint_distance_m',
                'last_valid_expected_distance_m','last_valid_actual_axial_distance_m','validated_progress_m','valid_prefix_sample_count','max_position_error_mm','max_axis_error_deg',
                'max_key_error_deg','valid_prefix_clearance','elapsed_s','ik_elapsed_s','first_failure','query_statistics','viewer']}
            row.update(attempted_waypoints=len(result['waypoints']),task_pose_waypoints=sum(p.get('selected',{}).get('task_check',{}).get('task_pose_feasible',False) for p in result['waypoints']),
                       inspected_samples=n,ik_optimizations=sum(run['optimization_count'] for p in result['waypoints'] for run in p['ik_runs']),
                       max_selected_joint_delta_rad=max((p['selected']['max_joint_delta_rad'] for p in result['waypoints'] if p.get('selected')),default=None))
            comparison.append(row);print(json.dumps({'stage':'T4','branch':label,**{k:row[k] for k in ['status','accepted_waypoints','requested_waypoints','last_valid_expected_distance_m','inspected_samples','first_failure']}}),flush=True)
        _write(out/'collision_queries.json',{'identity':evaluator.identity,'statistics':checker.statistics(),'records':checker.queries})
        identity=evaluator.identity;statistics=checker.statistics()
    if not evaluator.client.cleanup['exit_normal']: raise PathError('T4 backend exited abnormally')
    summary={'stage':'T4','usage':'RESEARCH_ONLY','status':'COMPLETED','comparison':comparison,'settings':params,'identity':identity,
        'task_sha256':spec.sha256,'tcp_sha256':spec.tcp_sha256,'scene_sha256':scene.canonical_scene_sha256,'backend_cleanup':evaluator.client.cleanup,
        'total_query_statistics':statistics,'all_samples_task_constrained':True,'continuous_collision_proven':False,'time_parameterized':False,
        'insertion_success':False,'tool_cable_collision_geometry_modeled':False,'hardware_validated':False,'planning_approved':False}
    _write(out/'summary.json',summary)
    for path,digest in inputs.items():
        if _hash(path)!=digest: raise PathError('historical input changed: '+path)
    sources={str(p.relative_to(root)):_hash(p) for folder in ['src','scripts','cpp','config'] for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    _write(out/'manifest.json',{'stage':'T4','status':'COMPLETED','usage':'RESEARCH_ONLY',
        'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'input_hashes':inputs,'ik_run':str(ik_run),'path_run':str(path_run),
        'settings_sha256':sha256(canonical(params).encode()).hexdigest(),'task_sha256':spec.sha256,'tcp_sha256':spec.tcp_sha256,
        'scene_sha256':scene.canonical_scene_sha256,'identity':identity,'source_sha256':sources,
        'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})
    return summary


def verify_approach(output,model):
    """Check artifacts, unchanged inputs, exact interpolation, FK and all constraints.

    No new FCL query is made; collision provenance is the hashed native ledger.
    """
    out=Path(output).resolve();manifest=json.loads((out/'manifest.json').read_text())
    if manifest.get('stage')!='T4' or manifest.get('status')!='COMPLETED': raise PathError('not a completed T4 run')
    for name,digest in manifest['files'].items():
        p=(out/name).resolve()
        if not p.is_relative_to(out) or _hash(p)!=digest: raise PathError('artifact changed: '+name)
    for name,digest in manifest['input_hashes'].items():
        if _hash(name)!=digest: raise PathError('historical input changed: '+name)
    settings=approach_settings(json.loads((out/'settings.json').read_text()))
    if sha256(canonical(settings).encode()).hexdigest()!=manifest['settings_sha256']: raise PathError('settings differ')
    spec=load_task(out/'task_config.yaml');scene=parse_scene((out/'scene_snapshot.json').read_text());targets=build_targets(spec,scene)
    if spec.sha256!=manifest['task_sha256'] or spec.tcp_sha256!=manifest['tcp_sha256'] or scene.canonical_scene_sha256!=manifest['scene_sha256'] or model.metadata()['expanded_urdf_sha256']!=manifest['identity']['urdf_sha256']: raise PathError('model/task/TCP/scene mismatch')
    points=cartesian_waypoints(targets,settings['end_distance_m'],settings['cartesian_step_m']);fk=ForwardKinematics(model)
    source=json.loads((Path(manifest['ik_run'])/'ik_results.json').read_text())['unique_task_solutions']
    ledger=json.loads((out/'collision_queries.json').read_text());summary=json.loads((out/'summary.json').read_text());checked=0
    if ledger['statistics']['native_collision_queries']!=len(ledger['records']): raise PathError('query count differs')
    for row in summary['comparison']:
        name='q'+str(row['solution_id']);data=json.loads((out/('branches/'+name+'.json')).read_text())
        start=next(s['q'] for s in source if s['solution_id']==row['solution_id'])
        if data['start_q']!=start: raise PathError('original T2 branch changed')
        with np.load(out/('branches/'+name+'.npz'),allow_pickle=False) as arrays:
            if len(arrays['q'])!=len(data['samples']): raise PathError('sample count differs')
            for i,state in enumerate(data['samples']):
                point=state['approach_check'];wi=point['waypoint_index'];fraction=point['segment_fraction'];current=data['waypoints'][wi]['selected']['q']
                if wi==0:
                    expected_q=np.array(start);expected_distance=points[0]['distance_to_port_m']
                else:
                    before=data['waypoints'][wi-1]['selected']['q'];edge=interpolate(before,current,settings['edge_resolution_rad']);k=round(fraction*(len(edge)-1))
                    if abs(fraction-k/(len(edge)-1))>1e-14: raise PathError('non-grid joint interpolation')
                    expected_q=edge[k];expected_distance=points[wi-1]['distance_to_port_m']*(1-fraction)+points[wi]['distance_to_port_m']*fraction
                if not np.array_equal(arrays['q'][i],expected_q) or state['q']!=expected_q.tolist(): raise PathError('joint interpolation differs')
                query=ledger['records'][state['query_id']]
                if query['q']!=state['q']: raise PathError('native query provenance differs')
                expected=check_approach_sample(query,targets,float(expected_distance),fk,waypoint_index=wi,fraction=fraction)
                if state['valid']!=expected['valid'] or state['reasons']!=expected['reasons'] or bool(arrays['valid'][i])!=expected['valid']: raise PathError('task/configuration classification differs')
                for key in ['position_m','insertion_axis_rad','key_axis_rad']:
                    if abs(state['task_errors'][key]-expected['task_errors'][key])>1e-12 or abs(arrays[key][i]-expected['task_errors'][key])>1e-12: raise PathError('task errors differ')
                if not np.allclose(arrays['T_base_tcp'][i],fk(expected_q)@spec.T_ee_tcp,atol=1e-10,rtol=0): raise PathError('FK TCP differs')
                if not np.array_equal(arrays['expected_T_base_tcp'][i],expected['expected_T_base_tcp']): raise PathError('expected Cartesian trajectory differs')
            first_bad=next((i for i,s in enumerate(data['samples']) if not s['valid']),None)
            if first_bad!=data['first_failed_sample_index']: raise PathError('first failure differs')
            accepted=sum(p['accepted'] for p in data['waypoints'])
            if accepted!=row['accepted_waypoints'] or accepted!=data['accepted_waypoints']: raise PathError('waypoint counts differ')
            success=data['first_failure'] is None and accepted==len(points) and bool(arrays['valid'].all())
            if success!=row['success'] or success!=data['success']: raise PathError('approach success differs')
            checked+=len(data['samples'])
    return {'status':'PASS','usage':'RESEARCH_ONLY','branches':len(summary['comparison']),'checked_samples':checked,'all_input_hashes_unchanged':True,'new_collision_queries':0}
