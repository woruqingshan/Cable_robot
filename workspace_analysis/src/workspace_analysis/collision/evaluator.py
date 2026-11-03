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
    def __init__(self,model,scene,policy,backend,*,gripper=.3,clearance=None,timeout=30):
        self.model=model;self.scene=scene;self.policy=policy;self.gripper=gripper
        self.clearance=dict(clearance) if clearance is not None else {'environment_m':0.,'self_m':0.}
        if set(self.clearance)!={'environment_m','self_m'} or any(type(x) not in (int,float) or not math.isfinite(x) or x<0 for x in self.clearance.values()):
            raise CollisionError('finite nonnegative clearance thresholds required')
        model.joint_positions(model.limits.mean(axis=1),{'r_joint':gripper})
        self.client=CollisionClient(backend,timeout=timeout)
        try:
            self.ready=self.client.initialize(model,scene);self.client.set_policy(*policy)
        except BaseException:self.client.close();raise
        self.identity={**self.client.identity,'gripper_state_rad':gripper,'clearance_settings':self.clearance,
                       'contact_mode':'FCL_raw_boolean','hardware_validated':False,'scope':'bare_gripper_official_URDF_24_mesh'}
        self.identity['evaluation_sha256']=sha256(json.dumps(self.identity,sort_keys=True).encode()).hexdigest()
        self.last_native=None

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
                started=time.perf_counter()
                try:
                    native=self.client.query(self.model,angles,auxiliary_positions={'r_joint':self.gripper})
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
                except Exception as exc:row['error']=f'{type(exc).__name__}: {exc}'
                row['query_time']=time.perf_counter()-started;result.append(row)
        return result

    def close(self):return self.client.close()
    def __enter__(self):return self
    def __exit__(self,*args):self.close()
