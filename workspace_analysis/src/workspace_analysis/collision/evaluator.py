"""Reusable, index-preserving model-scope evaluator over a persistent native session."""
from hashlib import sha256
import json
import math
import time
import numpy as np
from .client import CollisionClient
from .protocol import CollisionError


class Evaluator:
    """Model/scene/policy/gripper/thresholds are explicit and immutable per session."""
    def __init__(self,model,scene,policy,backend,*,gripper=.3,clearance=None,timeout=30,geometry_profile=None,task_scope=None):
        self.model=model;self.scene=scene;self.policy=policy;self.gripper=gripper
        self.clearance=dict(clearance) if clearance is not None else {'environment_m':0.,'self_m':0.}
        if set(self.clearance)!={'environment_m','self_m'} or any(type(x) not in (int,float) or not math.isfinite(x) or x<0 for x in self.clearance.values()):
            raise CollisionError('finite nonnegative clearance thresholds required')
        model.joint_positions(model.limits.mean(axis=1),{'r_joint':gripper})
        if task_scope is not None:
            from .scope import canonical_scope
            if canonical_scope(json.loads(task_scope[0]),model,policy)!=task_scope:raise CollisionError('invalid task scope canonical identity')
        self.client=CollisionClient(backend,timeout=timeout)
        try:
            self.ready=self.client.initialize(model,scene,**({'geometry_profile':geometry_profile} if geometry_profile is not None else {}));self.client.set_policy(*policy)
        except BaseException:self.client.close();raise
        self.identity={**self.client.identity,'gripper_state_rad':gripper,'clearance_settings':self.clearance,
                       'contact_mode':'FCL_raw_boolean','hardware_validated':False,'scope':'bare_gripper_official_URDF_24_mesh'}
        self.identity['evaluation_sha256']=sha256(json.dumps(self.identity,sort_keys=True).encode()).hexdigest()
        self.last_native=None
        self.task_scope=task_scope
        if geometry_profile is not None:
            self.identity['scope']='pair-local diagnostic self geometry; official full environment' if geometry_profile.effective_xml is not None else 'official full geometry'
            self.identity['missing_coverage']=['lidar_collision','RJ45_tool_TCP','cable','hardware_calibration']+(['unvalidated_internal_joint_exclusions'] if geometry_profile.effective_xml is not None else [])
            self.identity['evaluation_sha256']=sha256(json.dumps(self.identity,sort_keys=True).encode()).hexdigest()
        if task_scope is not None:
            self.identity['task_scope_sha256']=task_scope[1]
            self.identity['task_scope_id']=json.loads(task_scope[0])['id']
            self.identity['scope']='official full geometry + explicitly unverified task scope'
            self.identity['evaluation_sha256']=sha256(json.dumps(self.identity,sort_keys=True).encode()).hexdigest()

    def evaluate(self,q,*,start_index=0,batch_size=64):
        """Return one record for every q row, including input/protocol failures."""
        if isinstance(batch_size,bool) or not isinstance(batch_size,int) or batch_size<1:raise CollisionError('batch_size must be positive integer')
        values=np.asarray(q)
        if values.ndim!=2 or values.shape[1]!=5:raise CollisionError('expected q[N,5]')
        result=[]
        for begin in range(0,len(values),batch_size):
            for offset,angles in enumerate(values[begin:begin+batch_size],begin):
                row={'sample_index':start_index+offset,'q':angles.tolist(),'identity':self.identity,
                     'raw_self_collision':None,'effective_self_collision':None,'environment_collision':None,
                     'model_scope_collision_free':None,'verification_status':'ERROR','final_decision':'ERROR',
                     'allowed_pairs':[],'remaining_collision_pairs':[],'clearance':None,
                     'ee_position':None,'ee_orientation':None}
                if self.task_scope is not None:row.update(task_scope_candidate=None,task_scope_status='ERROR',task_scope_blocking_self_collision=None,gripper_internal_collision=None,task_scope_blocking_pairs=[],gripper_internal_pairs=[],gripper_internal_verification='UNVERIFIED_MECHANISM',hardware_safe=None)
                started=time.perf_counter()
                try:
                    native=self.client.query(self.model,angles,auxiliary_positions={'r_joint':self.gripper},**({'task_scope':self.task_scope} if self.task_scope is not None else {}))
                    self.last_native=native
                    threshold_met={}
                    for kind,key in [('environment','environment_m'),('self','self_m')]:
                        d=native['clearance'][kind]
                        threshold_met[kind]=True if self.clearance[key]==0 else (d['separation_m']>=self.clearance[key] if d['available'] and d['separation_m'] is not None else None)
                    collision=native['effective_self_collision'] or native['environment_collision']
                    unknown_clearance=any(v is None for v in threshold_met.values())
                    row.update(raw_self_collision=native['raw_self_collision'],effective_self_collision=native['effective_self_collision'],environment_collision=native['environment_collision'],
                        model_scope_collision_free=False if collision else None if unknown_clearance else all(threshold_met.values()),verification_status='CLEARANCE_UNSUPPORTED' if unknown_clearance else 'MODEL_ONLY',
                        final_decision='COLLIDING' if collision else 'UNVERIFIED',allowed_pairs=native['filtered_pairs'],
                        remaining_collision_pairs=[{'kind':'self','links':p} for p in native['effective_self_pairs']]+[{'kind':'environment','links':[p['first'],p['second']]} for p in native['colliding_pairs'] if p['kind']=='environment'],
                        clearance={'native':native['clearance'],'required':self.clearance,'threshold_met':threshold_met},
                        ee_position=np.array(native['T_base_to_ee'])[:3,3].tolist(),
                        ee_orientation=np.array(native['T_base_to_ee'])[:3,:3].tolist(),native_query_time=native['query_elapsed_s'],pairs_may_be_truncated=native['pairs_may_be_truncated'])
                    if 'geometry_profile' in self.identity:
                        row['geometry_profile']=self.identity['geometry_profile'];row['missing_coverage']=self.identity['missing_coverage']
                        row['source_urdf_identity']=self.identity['source_urdf_sha256'];row['effective_collision_geometry_identity']=self.identity['robot_geometry_sha256']
                        if self.identity['geometry_review_status']=='diagnostic_only':row['verification_status']='DIAGNOSTIC_GEOMETRY_ONLY'
                    if self.task_scope is not None:
                        scoped=native['task_scope'];met={}
                        for kind,key in [('self','self_m'),('environment','environment_m')]:
                            d=scoped['clearance'][kind];met[kind]=True if self.clearance[key]==0 else (d['separation_m']>=self.clearance[key] if d['available'] and d['separation_m'] is not None else None)
                        blocked=scoped['task_scope_blocking_self_collision'] or scoped['environment_collision']
                        candidate=False if blocked else None if any(v is None for v in met.values()) else all(met.values())
                        row.update(official_raw_self_collision=scoped['official_raw_self_collision'],official_effective_self_collision=scoped['official_effective_self_collision'],task_scope_blocking_self_collision=scoped['task_scope_blocking_self_collision'],gripper_internal_collision=scoped['gripper_internal_collision'],task_scope_candidate=candidate,task_scope_status='REJECTED' if candidate is False else 'CANDIDATE' if candidate is True else 'UNKNOWN',task_scope_blocking_pairs=scoped['task_scope_blocking_pairs'],gripper_internal_pairs=scoped['gripper_internal_pairs'],gripper_internal_verification=scoped['gripper_internal_verification'],task_scope_clearance={'native':scoped['clearance'],'required':self.clearance,'threshold_met':met},task_scope_diagnostics=scoped,model_scope_status='OFFICIAL_FULL_REJECTED' if collision else 'OFFICIAL_MODEL_ONLY_UNVERIFIED',missing_coverage=scoped['missing_coverage'],hardware_safe=None)
                except Exception as exc:row['error']=f'{type(exc).__name__}: {exc}'
                row['query_time']=time.perf_counter()-started;result.append(row)
        return result

    def close(self):return self.client.close()
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
