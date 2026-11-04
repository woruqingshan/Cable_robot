"""Strict independent task configuration; synthetic TCP is never calibration."""
from dataclasses import dataclass
from hashlib import sha256
import json
import math
from pathlib import Path
import re
import numpy as np
import yaml
from ..geometry.se3 import Pose


class TaskError(ValueError):
    """Malformed task, inconsistent axes/frames or invalid target input."""


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False,ensure_ascii=False)


def _keys(value,names,label):
    if not isinstance(value,dict) or set(value)!=set(names):raise TaskError(label+': unexpected/missing fields')


def _real(value,label,*,positive=False):
    if type(value) not in (int,float) or not math.isfinite(value) or positive and value<=0:raise TaskError(label+': finite '+('positive ' if positive else '')+'number required')
    return float(value)


def _axis(value,label):
    if not isinstance(value,(list,tuple)) or len(value)!=3:raise TaskError(label+': three real numbers required')
    a=np.array([_real(v,label) for v in value])
    if abs(np.linalg.norm(a)-1)>1e-10:raise TaskError(label+': unit axis required')
    return a


@dataclass(frozen=True)
class TaskSpec:
    """Canonical immutable payload. Accessors return copies."""
    canonical_json:str
    raw_sha256:str|None=None
    source_path:str|None=None

    @property
    def data(self):return json.loads(self.canonical_json)
    @property
    def sha256(self):return sha256(self.canonical_json.encode()).hexdigest()
    @property
    def tcp_sha256(self):return sha256(canonical(self.data['tcp']).encode()).hexdigest()
    @property
    def T_ee_tcp(self):
        d=self.data['tcp']['T_ee_tcp'];return Pose(tuple(d['position_m']),tuple(d['quaternion_xyzw'])).matrix()
    @property
    def tolerance(self):
        t=self.data['tolerances'];return {'position_m':t['position_m'],'axis_rad':math.radians(t['insertion_axis_deg']),'key_rad':math.radians(t['key_axis_deg'])}


def parse_task(data,*,raw_sha256=None,source_path=None):
    _keys(data,['version','id','usage','target','tcp','approach','tolerances','solver','collision'],'task')
    if type(data['version']) is not int or data['version']!=1 or data['usage']!='RESEARCH_ONLY':raise TaskError('task version/usage mismatch')
    if not isinstance(data['id'],str) or not data['id']:raise TaskError('task id required')
    target=data['target'];_keys(target,['port_id','reference_frame','ee_frame','tcp_frame'],'target')
    for key,value in target.items():
        if not isinstance(value,str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_/-]*',value):raise TaskError('invalid target '+key)
    if len({target['reference_frame'],target['ee_frame'],target['tcp_frame']})!=3:raise TaskError('task frames must be distinct')
    tcp=data['tcp'];_keys(tcp,['status','source','T_ee_tcp','insertion_axis_local','key_axis_local'],'tcp')
    if tcp['status']!='synthetic_uncalibrated' or not isinstance(tcp['source'],str) or not tcp['source'].strip():raise TaskError('explicit synthetic/uncalibrated TCP source required')
    pose=tcp['T_ee_tcp'];_keys(pose,['position_m','quaternion_xyzw'],'T_ee_tcp')
    try:Pose(tuple(pose['position_m']),tuple(pose['quaternion_xyzw']))
    except (ValueError,TypeError) as exc:raise TaskError('invalid TCP transform: '+str(exc)) from exc
    a=_axis(tcp['insertion_axis_local'],'tcp insertion');k=_axis(tcp['key_axis_local'],'tcp key')
    if abs(a@k)>1e-10:raise TaskError('TCP insertion/key axes must be perpendicular')
    approach=data['approach'];_keys(approach,['status','distance_m'],'approach')
    if approach['status']!='synthetic_uncalibrated':raise TaskError('approach must declare synthetic_uncalibrated')
    _real(approach['distance_m'],'approach distance',positive=True)
    tol=data['tolerances'];_keys(tol,['status','position_m','insertion_axis_deg','key_axis_deg'],'tolerances')
    if tol['status']!='research_assumption':raise TaskError('tolerance source must be research_assumption')
    _real(tol['position_m'],'position tolerance',positive=True)
    for key in ['insertion_axis_deg','key_axis_deg']:
        if not 0<_real(tol[key],key)<180:raise TaskError('angular tolerance must be between0 and180 degrees')
    solver=data['solver'];_keys(solver,['random_seed','workspace_nearest_seeds','extra_joint_seeds','include_midpoint_seed','max_nfev','ftol','xtol','gtol','deduplicate_joint_rad','residual_weights'],'solver')
    for key,minimum,maximum in [('random_seed',0,2**32-1),('workspace_nearest_seeds',0,1000),('extra_joint_seeds',0,1000),('max_nfev',1,10000)]:
        if type(solver[key]) is not int or not minimum<=solver[key]<=maximum:raise TaskError('invalid solver '+key)
    if type(solver['include_midpoint_seed']) is not bool or solver['workspace_nearest_seeds']+solver['extra_joint_seeds']+int(solver['include_midpoint_seed'])<1:raise TaskError('at least one valid seed required')
    for key in ['ftol','xtol','gtol','deduplicate_joint_rad']:_real(solver[key],key,positive=True)
    if min(solver[k] for k in ['ftol','xtol','gtol'])<=np.finfo(float).eps:raise TaskError('solver stopping tolerances must exceed machine epsilon')
    _keys(solver['residual_weights'],['position','insertion_axis','key_axis'],'residual weights')
    for value in solver['residual_weights'].values():_real(value,'residual weight',positive=True)
    collision=data['collision'];_keys(collision,['gripper_rad','self_clearance_m','environment_clearance_m'],'collision')
    for key,value in collision.items():
        _real(value,key)
        if key!='gripper_rad' and value<0:raise TaskError('clearance must be nonnegative')
    return TaskSpec(canonical(data),raw_sha256,source_path)


def load_task(path):
    path=Path(path).resolve();raw=path.read_bytes();text=raw.decode('utf-8')
    if any(isinstance(event,yaml.events.AliasEvent) for event in yaml.parse(text)):raise TaskError('YAML aliases unsupported for tasks')
    class Loader(yaml.SafeLoader):pass
    def mapping(loader,node):
        result={}
        for k,v in node.value:
            key=loader.construct_object(k)
            if not isinstance(key,str) or key in result:raise TaskError('duplicate/non-string task key')
            result[key]=loader.construct_object(v)
        return result
    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,mapping)
    try:data=yaml.load(text,Loader=Loader)
    except yaml.YAMLError as exc:raise TaskError('invalid task YAML') from exc
    return parse_task(data,raw_sha256=sha256(raw).hexdigest(),source_path=str(path))
