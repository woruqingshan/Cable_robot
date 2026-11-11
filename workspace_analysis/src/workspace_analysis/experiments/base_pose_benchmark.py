"""Bounded station experiments; delegates unchanged task algorithms to T5.1."""
import csv
import json
import os
from pathlib import Path
from collections import Counter
from .assembly_semantics import _hash, _write
from .base_pose_context import make_context, verify_context
from .task_benchmark import run_benchmark
from .task_benchmark_validation import verify_benchmark
from ..scene.base_pose import BasePose, derive_scene
from ..scene.schema import parse_scene
from ..robot.model import load_model
from ..robot.root_frame import fixed_body_links
from ..task.spec import TaskError, canonical


def checked_pilot(config: dict) -> list[tuple[BasePose,list[str]]]:
    if not isinstance(config,dict) or set(config)!={'schema_version','usage','start_sample_index','stations'} or config['schema_version']!=1 or config['usage']!='RESEARCH_ONLY':
        raise TaskError('explicit schema-v1 RESEARCH_ONLY pilot required')
    if type(config['start_sample_index']) is not int or config['start_sample_index']<0:
        raise TaskError('nonnegative fixed G4 sample index required')
    rows=config['stations']
    if not isinstance(rows,list) or not 1<=len(rows)<=12:raise TaskError('bounded pilot requires 1..12 stations')
    result=[];ids=set()
    for row in rows:
        if not isinstance(row,dict) or set(row)!={'pose','ports'}:raise TaskError('station fields differ')
        pose=BasePose.from_dict(row['pose']);ports=row['ports']
        if pose.id in ids:raise TaskError('duplicate station id')
        if not isinstance(ports,list) or not ports or any(not isinstance(p,str) or not p.isalnum() for p in ports) or len(set(ports))!=len(ports):
            raise TaskError('unique explicit port IDs required')
        ids.add(pose.id);result.append((pose,ports))
    return result


def station_assessment(model, start: dict) -> dict:
    """Classify modeled rigid-body pairs separately; never assert hardware coverage."""
    state=start['tested'][-1];body=fixed_body_links(model)
    pairs=state.get('environment_pairs',[])
    body_pairs=[p for p in pairs if {p['first'],p['second']} & body]
    collided=state.get('environment_collision')
    status='MODELED_BODY_COLLISION' if body_pairs else 'MODELED_BODY_CLEAR' if collided is False else 'BODY_COLLISION_UNRESOLVED'
    return {'status':status,'start_status':start['status'],'start_sample_index':start['original_sample_index'],
        'fixed_body_links':sorted(body),'body_environment_pairs':body_pairs,'start_query':state,
        'physical_coverage_status':'PARTIAL_UNVERIFIED','missing_or_unverified_coverage':[
            'lidar_link has no URDF collision geometry','real RJ45/tool/cable absent',
            'mesh/extrinsic/calibration/physical assembly tolerances unverified',
            'ground support and mobile-base travel to station not modeled'],
        'physical_station_safety_verified':False}


def run_base_pose_benchmark(root: Path,backend: Path,world_scene: Path,task_template: Path,
                            seed_prefix: Path,pilot_config: Path,output: Path,*,allow_candidate=False,
                            planning=None,approach=None) -> dict:
    root=Path(root).resolve();out=Path(output).absolute()
    if os.path.lexists(out):raise TaskError('refusing existing pilot output')
    if not allow_candidate:raise TaskError('explicit RESEARCH_ONLY candidate opt-in required')
    config=json.loads(Path(pilot_config).read_text());stations=checked_pilot(config)
    text=Path(world_scene).read_text();world=parse_scene(text);model=load_model(root/'config/workspace.yaml')
    inputs={str(Path(p).resolve()):_hash(p) for p in (world_scene,task_template,pilot_config,backend,Path(str(seed_prefix)+'.npz'),Path(str(seed_prefix)+'.json'))}
    known={p['id'] for p in world.ports}
    if any(not set(ports)<=known for _,ports in stations):raise TaskError('unknown requested port')
    out.mkdir(parents=True,exist_ok=False);_write(out/'pilot_config.json',config)
    rows=[];identities={}
    for pose,ports in stations:
        station=out/'stations'/pose.id;station.mkdir(parents=True)
        derived=derive_scene(world,pose);context=make_context(text,pose,model)
        _write(station/'derived_scene.json',derived.to_dict());_write(station/'base_pose_context.json',context)
        verify_context(context,derived,model)
        child=station/'benchmark'
        summary=run_benchmark(root,backend,station/'derived_scene.json',task_template,seed_prefix,child,
            port_ids=ports,allow_candidate=True,planning=planning,approach=approach,
            start_sample_index=config['start_sample_index'],base_pose_context=station/'base_pose_context.json')
        validation=verify_benchmark(child,model);_write(child/'independent_validation.json',validation)
        _write(child/'validation_manifest.json',{'manifest_sha256':_hash(child/'manifest.json'),'report_sha256':_hash(child/'independent_validation.json')})
        start=json.loads((child/'common_start.json').read_text());assessment=station_assessment(model,start)
        _write(station/'station_assessment.json',assessment)
        identities[pose.id]={'benchmark_manifest_sha256':_hash(child/'manifest.json'),'base_pose_sha256':pose.sha256,
            'world_from_base':pose.matrix().tolist(),'world_scene_sha256':world.canonical_scene_sha256,
            'derived_scene_sha256':derived.canonical_scene_sha256,'validation':validation}
        for row in summary['rows']:
            rows.append({'base_pose_id':pose.id,**pose.to_dict(),'start_valid':start['status']=='VALID_SIMULATION_START',
                'station_body_status':assessment['status'],'coverage':'PARTIAL_UNVERIFIED',**row})
        print(canonical({'station':pose.id,'start':start['status'],'successful_ports':summary['successful_ports']}),flush=True)
    summary={'schema_version':1,'stage':'T5.2-R2','usage':'RESEARCH_ONLY','rows':rows,'stations':identities,
        'successful_ports_by_station':{p.id:sum(r['status']=='SUCCESS_RESEARCH_ONLY' for r in rows if r['base_pose_id']==p.id) for p,_ in stations},
        'failure_distribution':dict(Counter(r['failure_reason'] for r in rows if r['failure_reason'])),
        'fixed_start_sample_index':config['start_sample_index'],'world_scene_sha256':world.canonical_scene_sha256,
        'hardware_validated':False,'continuous_collision_proven':False,'mobile_base_motion_planned':False}
    _write(out/'summary.json',summary)
    with (out/'summary.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader()
        writer.writerows({k:canonical(v) if isinstance(v,(list,dict)) else v for k,v in row.items()} for row in rows)
    for p,h in inputs.items():
        if _hash(p)!=h:raise TaskError('pilot input changed: '+p)
    _write(out/'manifest.json',{'schema_version':1,'stage':'T5.2-R2','usage':'RESEARCH_ONLY','status':'COMPLETED',
        'input_hashes':inputs,'station_identities':identities,
        'files':{str(p.relative_to(out)):_hash(p) for p in out.rglob('*') if p.is_file()}})
    return summary
