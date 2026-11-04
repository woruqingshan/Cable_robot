"""Scene-derived port/pre-insertion TCP targets and pure directional errors."""
from dataclasses import dataclass
import math
import numpy as np
from ..geometry.se3 import Pose,validate_transform,matrix_to_quaternion,invert_transform
from .spec import TaskSpec,TaskError


@dataclass(frozen=True)
class TaskTargets:
    spec:TaskSpec
    scene_sha256:str
    port_frame:str
    base_to_port:Pose
    port_tcp:Pose
    pre_tcp:Pose
    insertion_axis_base:tuple
    key_axis_base:tuple

    @property
    def T_base_pre_tcp(self):return self.pre_tcp.matrix()
    @property
    def T_base_pre_ee(self):return self.T_base_pre_tcp@invert_transform(self.spec.T_ee_tcp)

    def to_dict(self):
        return {'usage':'RESEARCH_ONLY','task_sha256':self.spec.sha256,'tcp_sha256':self.spec.tcp_sha256,'scene_sha256':self.scene_sha256,
            'port_frame':self.port_frame,'T_base_port':self.base_to_port.matrix().tolist(),'T_base_port_tcp':self.port_tcp.matrix().tolist(),
            'T_base_pre_tcp':self.T_base_pre_tcp.tolist(),'T_base_pre_ee':self.T_base_pre_ee.tolist(),'T_ee_tcp':self.spec.T_ee_tcp.tolist(),
            'insertion_axis_base':list(self.insertion_axis_base),'key_axis_base':list(self.key_axis_base),
            'tcp_parameters_status':'synthetic_uncalibrated','tool_collision_geometry_modeled':False}


def _pose(t):return Pose(tuple(t[:3,3]),tuple(matrix_to_quaternion(t[:3,:3])))


def build_targets(spec,scene):
    data=spec.data;target=data['target']
    if target['reference_frame']!=scene.reference_frame:raise TaskError('task/scene base frame mismatch')
    ports=[p for p in scene.ports if p['id']==target['port_id']]
    if len(ports)!=1:raise TaskError('task port missing or ambiguous')
    port=ports[0];t=scene.frame_graph.resolve(target['reference_frame'],port['frame'])
    insertion=t[:3,:3]@np.asarray(port['insertion_axis_local']);key=t[:3,:3]@np.asarray(port['key_axis_local'])
    local_a=np.asarray(data['tcp']['insertion_axis_local']);local_k=np.asarray(data['tcp']['key_axis_local'])
    desired_basis=np.column_stack((insertion,np.cross(key,insertion),key));local_basis=np.column_stack((local_a,np.cross(local_k,local_a),local_k))
    desired=np.eye(4);desired[:3,:3]=desired_basis@local_basis.T;desired[:3,3]=t[:3,3]
    desired=validate_transform(desired);pre=desired.copy();pre[:3,3]-=data['approach']['distance_m']*insertion
    return TaskTargets(spec,scene.canonical_scene_sha256,port['frame'],_pose(t),_pose(desired),_pose(pre),tuple(insertion.tolist()),tuple(key.tolist()))


def angle_between(first,second):
    a=np.asarray(first,dtype=float);b=np.asarray(second,dtype=float)
    if a.shape!=(3,) or b.shape!=(3,) or not np.isfinite(a).all() or not np.isfinite(b).all() or np.linalg.norm(a)==0 or np.linalg.norm(b)==0:raise TaskError('finite nonzero direction vectors required')
    a=a/np.linalg.norm(a);b=b/np.linalg.norm(b)
    return float(np.arctan2(np.linalg.norm(np.cross(a,b)),np.clip(a@b,-1,1)))


def tcp_errors(transform,targets):
    t=validate_transform(transform);tcp=targets.spec.data['tcp']
    a=t[:3,:3]@np.asarray(tcp['insertion_axis_local']);k=t[:3,:3]@np.asarray(tcp['key_axis_local'])
    position=float(np.linalg.norm(t[:3,3]-np.asarray(targets.pre_tcp.position_m)))
    axis=angle_between(a,targets.insertion_axis_base);key=angle_between(k,targets.key_axis_base)
    return {'position_m':position,'position_mm':1000*position,'insertion_axis_rad':axis,'insertion_axis_deg':math.degrees(axis),
            'key_axis_rad':key,'key_axis_deg':math.degrees(key),'actual_tcp_position_m':t[:3,3].tolist(),
            'actual_insertion_axis_base':a.tolist(),'actual_key_axis_base':k.tolist(),
            'orientation_metric':'direct signed-direction alignment; 180deg key flip fails; not projected roll'}
