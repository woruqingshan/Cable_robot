"""Offline C1 diagnostics over original G2 indices; explicit error denominators."""
import csv
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import numpy as np

from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..scene.schema import load_scene
from ..workspace.results import load_results, file_sha256
from .client import CollisionClient
from .protocol import CollisionError


def summarize(rows):
    """Count overlapping raw collision sets separately from exclusive decisions."""
    decisions={s:sum(r['decision']==s for r in rows) for s in ['CLEAR','COLLIDING','UNVERIFIED','ERROR']}
    if sum(decisions.values())!=len(rows):raise CollisionError('unknown decision')
    known=[r for r in rows if r['decision']!='ERROR']
    self_count=sum(r['self_collision'] is True for r in known)
    env_count=sum(r['environment_collision'] is True for r in known)
    both=sum(r['self_collision'] is True and r['environment_collision'] is True for r in known)
    union=sum(r['collision_detected'] is True for r in known)
    if union!=self_count+env_count-both or union!=decisions['COLLIDING']:
        raise CollisionError('classification conservation failure')
    times=[r['backend_query_elapsed_s'] for r in known]
    frequencies=Counter((p['kind'],p['first'],p['second']) for r in known for p in r.get('colliding_pairs',[]))
    return {'total_samples':len(rows),'known_raw_results':len(known),'decisions':decisions,
            'colliding_samples':union,'self_collision_count':self_count,'environment_collision_count':env_count,
            'pair_sample_frequency':[{'kind':k[0],'first':k[1],'second':k[2],'samples':v} for k,v in frequencies.most_common()],
            'both_collision_count':both,'unverified_count':decisions['UNVERIFIED'],'error_count':decisions['ERROR'],
            'query_timing_s':{'count':len(times),'sum':sum(times),'mean':float(np.mean(times)) if times else None,
                              'p95':float(np.percentile(times,95)) if times else None},
            'scope':'raw full-URDF geometry checks; incomplete coverage and unreviewed empty ACM; no physical safety claim'}


def run_diagnostic(config_path,scene_path,executable,output_dir,*,input_prefix=None,q=None,gripper=0.,
                   timeout=15,client_factory=CollisionClient):
    """Reserve a fresh run directory; retain all input rows even on backend failure."""
    output=Path(output_dir).expanduser().absolute()
    if os.path.lexists(output):raise CollisionError('refusing existing output directory')
    model=load_model(config_path);scene=load_scene(scene_path)
    config_hash=file_sha256(config_path);source=None
    if q is None:
        if input_prefix is None:raise CollisionError('input prefix required')
        arrays,metadata=load_results(input_prefix,model=model)
        if len(arrays['q'])!=100:raise CollisionError('C1 only permits the existing 100-row diagnostic')
        values=arrays['q']
        source={'prefix':str(Path(input_prefix).absolute()),'npz_sha256':file_sha256(str(input_prefix)+'.npz'),
                'json_sha256':file_sha256(str(input_prefix)+'.json'),'metadata':metadata,
                'negative_z_rows_retained':int(np.count_nonzero(arrays['position'][:,2]<0))}
    else:
        if input_prefix is not None:raise CollisionError('choose q or input prefix')
        values=np.array([model.validate_q(q)],dtype=np.float64)
    # Check gripper/mimic semantics before any backend process.
    states=[model.joint_positions(v,{'r_joint':gripper}) for v in values]
    output.parent.mkdir(parents=True,exist_ok=True);output.mkdir()
    marker=output/'.incomplete';marker.write_text('C1 diagnostic in progress\n')
    rows=[];client=None;ready=None;init_error=None;cleanup=None
    fk_checks=[]
    try:
        try:
            client=client_factory(executable,timeout=timeout);ready=client.initialize(model,scene)
            from scipy.spatial.transform import Rotation
            solver=ForwardKinematics(model)
            for name,angles in [('Zero',[0]*5),('Nonzero',[.35,-.6,.8,-.4,.2]),
                                ('Near-limit',[2.08,-.4,.5,-.3,.25])]:
                result=client.query_positions(model.joint_positions(angles,{'r_joint':gripper}))
                actual=np.array(result['T_base_to_ee']);expected=solver(angles)
                position_error=float(np.linalg.norm(actual[:3,3]-expected[:3,3]))
                rotation_error=float(Rotation.from_matrix(expected[:3,:3].T@actual[:3,:3]).magnitude())
                if position_error>1e-5 or rotation_error>1e-4:raise CollisionError('pre-diagnostic FK cross-validation failed')
                fk_checks.append({'name':name,'q_rad':angles,'position_error_m':position_error,
                                  'rotation_error_rad':rotation_error,'T_base_to_ee':actual.tolist()})
        except Exception as exc:init_error=f'{type(exc).__name__}: {exc}'
        for index,(angles,state) in enumerate(zip(values,states)):
            row={'sample_index':index,'q_rad':angles.tolist(),'joint_positions':state,
                 'decision':'ERROR','backend_status':'ERROR','self_collision':None,'environment_collision':None,
                 'collision_detected':None,'colliding_pairs':[],'backend_query_elapsed_s':None,
                 'model_sha256':model.metadata()['expanded_urdf_sha256'],'scene_sha256':scene.canonical_scene_sha256}
            started=time.perf_counter()
            try:
                if init_error:raise CollisionError(init_error)
                reply=client.query_positions(state)
                row.update(decision=reply['decision'],backend_status=reply['backend_status'],
                           self_collision=reply['self_collision'],environment_collision=reply['environment_collision'],
                           collision_detected=reply['collision_detected'],colliding_pairs=reply['colliding_pairs'],
                           backend_query_elapsed_s=reply['query_elapsed_s'],identity=reply['identity'],
                           pairs_may_be_truncated=reply['pairs_may_be_truncated'])
            except Exception as exc:row['error']=f'{type(exc).__name__}: {exc}'
            row['roundtrip_elapsed_s']=time.perf_counter()-started;rows.append(row)
        cleanup=client.close() if client else {'resources_released':True,'exit_normal':False,'exit_code':None}
        for p,h in [(config_path,config_hash),(scene_path,scene.raw_yaml_sha256)]:
            if file_sha256(p)!=h:raise CollisionError('configuration/scene changed during run')
        if source:
            for suffix,key in [('.npz','npz_sha256'),('.json','json_sha256')]:
                if file_sha256(str(input_prefix)+suffix)!=source[key]:raise CollisionError('G2 source changed during run')
        summary=summarize(rows)
        summary.update(stage='P2.3-C1',status='PASS' if summary['error_count']==0 and cleanup['exit_normal'] else 'ERROR',
                       stage_acceptance='pending_user_review',functional_mvp_only=True,
                       hardware_checked=False,collision_backend_validated=False,
                       model_sha256=model.metadata()['expanded_urdf_sha256'],scene_sha256=scene.canonical_scene_sha256,
                       collision_object_count=len(scene.collision_objects))
        diagnostics={'schema_version':1,'stage':'P2.3-C1','created_utc':datetime.now(timezone.utc).isoformat(),
                     'configuration_sha256':config_hash,'model':model.metadata(),'scene_raw_sha256':scene.raw_yaml_sha256,
                     'scene_canonical_sha256':scene.canonical_scene_sha256,'input_source':source,
                     'backend_ready':ready,'initialization_error':init_error,'cleanup':cleanup,'per_sample':rows}
        diagnostics['fk_cross_validation']=fk_checks
        with (output/'diagnostics.json').open('x') as f:json.dump(diagnostics,f,indent=2,allow_nan=False)
        fields=['sample_index','q_rad','joint_positions','self_collision','environment_collision','collision_detected',
                'colliding_pairs','decision','backend_status','model_sha256','scene_sha256',
                'backend_query_elapsed_s','roundtrip_elapsed_s','error']
        with (output/'per_sample.csv').open('x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
            for row in rows:
                item={k:row.get(k) for k in fields}
                for k in ['q_rad','joint_positions','colliding_pairs']:item[k]=json.dumps(item[k],separators=(',',':'))
                writer.writerow(item)
        summary['artifacts']={name:file_sha256(output/name) for name in ['diagnostics.json','per_sample.csv']}
        with (output/'summary.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False)
        marker.unlink()
        return summary
    except BaseException as exc:
        with (output/'failure.json').open('x') as f:json.dump({'status':'ERROR','error':str(exc)},f)
        raise
    finally:
        if client is not None:client.close()
