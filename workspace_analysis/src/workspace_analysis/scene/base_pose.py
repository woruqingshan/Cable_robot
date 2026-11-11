"""Fixed World (= B0) -> planar base_link scene adaptation; Scene Schema v1 stays intact."""
from dataclasses import dataclass
from hashlib import sha256
import json
import math
import numbers
import re
import numpy as np
from ..geometry.se3 import Pose, invert_transform, matrix_to_quaternion
from .model import SceneSnapshot, SceneError
from .schema import parse_scene


@dataclass(frozen=True)
class BasePose:
    """Pose of base_link in World=B0 [m, rad], NOT the wheel-ground contact point.

    No Z/roll/pitch search and no silent yaw wrapping. World z=0 is B0's base_link.
    """
    id: str
    x_m: float = 0.
    y_m: float = 0.
    yaw_rad: float = 0.

    def __post_init__(self) -> None:
        if not isinstance(self.id,str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]{0,39}',self.id):
            raise SceneError('invalid BasePose ID')
        for name in ('x_m','y_m','yaw_rad'):
            value=getattr(self,name)
            if isinstance(value,(bool,np.bool_)) or not isinstance(value,numbers.Real) or not math.isfinite(value):
                raise SceneError('BasePose '+name+' must be finite real, excluding bool')
            object.__setattr__(self,name,float(value)+0.)
        if not -math.pi <= self.yaw_rad <= math.pi:
            raise SceneError('BasePose yaw_rad must be in [-pi,pi]; no implicit wrapping')

    @property
    def is_identity(self) -> bool:
        return self.x_m == self.y_m == self.yaw_rad == 0.

    def matrix(self) -> np.ndarray:
        c,s=math.cos(self.yaw_rad),math.sin(self.yaw_rad)
        t=np.eye(4);t[:3,:3]=[[c,-s,0.],[s,c,0.],[0.,0.,1.]];t[:3,3]=[self.x_m,self.y_m,0.]
        return t

    def to_dict(self) -> dict:
        return {'id':self.id,'x_m':self.x_m,'y_m':self.y_m,'yaw_rad':self.yaw_rad}

    @property
    def sha256(self) -> str:
        return sha256(json.dumps(self.to_dict(),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()

    @classmethod
    def from_dict(cls, data: dict) -> 'BasePose':
        if not isinstance(data,dict) or set(data) != {'id','x_m','y_m','yaw_rad'}:
            raise SceneError('BasePose fields must be id/x_m/y_m/yaw_rad')
        return cls(**data)


def frame_mapping(world: SceneSnapshot, pose: BasePose) -> dict[str,str]:
    """Preserve hierarchy through a bijective namespace change for nonzero poses."""
    prefix=world.scene_id if pose.is_identity else world.scene_id+'__base_'+pose.id
    return {'base_link':'base_link',**{f.id:prefix+f.id[len(world.scene_id):] for f in world.frames}}


def _pose_dict(t: np.ndarray) -> dict:
    return {'position_m':t[:3,3].tolist(),'quaternion_xyzw':matrix_to_quaternion(t[:3,:3]).tolist()}


def derive_scene(world: SceneSnapshot, pose: BasePose) -> SceneSnapshot:
    """Apply inverse(W_T_B) ONCE to root edges/direct objects, never each child.

    B0 returns the original immutable snapshot, preserving its exact identity.
    Object/entity/port IDs and local dimensions stay stable for paired comparisons.
    """
    if not isinstance(world,SceneSnapshot) or not isinstance(pose,BasePose):
        raise SceneError('validated World SceneSnapshot and BasePose required')
    if pose.is_identity:return world
    data=world.to_dict();scene=data['scene'];names=frame_mapping(world,pose);b_w=invert_transform(pose.matrix())
    scene['id']=world.scene_id+'__base_'+pose.id
    for f in scene['frames']:
        if f['parent']=='base_link':f['pose']=_pose_dict(b_w@Pose(**{'position_m':tuple(f['pose']['position_m']),'quaternion_xyzw':tuple(f['pose']['quaternion_xyzw'])}).matrix())
        f['id']=names[f['id']];f['parent']=names[f['parent']]
    for obj in scene['collision_objects']:
        if obj['parent_frame']=='base_link':
            p=obj['pose'];obj['pose']=_pose_dict(b_w@Pose(tuple(p['position_m']),tuple(p['quaternion_xyzw'])).matrix())
        obj['parent_frame']=names[obj['parent_frame']]
    for item in scene['entities']:item['frame']=names[item['frame']]
    for item in scene['ports']:item['frame']=names[item['frame']]
    return parse_scene(json.dumps(data,allow_nan=False))


def verify_derivation(world: SceneSnapshot, derived: SceneSnapshot, pose: BasePose) -> dict:
    """Check full tree, ALL world object transforms and port direction invariance."""
    names=frame_mapping(world,pose);w_b=pose.matrix();maximum=0.
    if derived.reference_frame != world.reference_frame:raise SceneError('root frame changed')
    expected={names[f.id]:names[f.parent] for f in world.frames}
    if {f.id:f.parent for f in derived.frames} != expected:raise SceneError('frame hierarchy changed')
    def equal(a,b,label):
        nonlocal maximum
        error=float(np.max(np.abs(np.asarray(a)-b)));maximum=max(maximum,error)
        if error>1e-10:raise SceneError('World/Base derivation mismatch: '+label)
    for f in world.frames:
        equal(w_b@derived.frame_graph.resolve('base_link',names[f.id]),world.frame_graph.resolve('base_link',f.id),f.id)
    if {o['id'] for o in derived.collision_objects}!={o['id'] for o in world.collision_objects}:raise SceneError('object IDs changed')
    for obj in world.collision_objects:
        equal(w_b@derived.object_transform(obj['id']),world.object_transform(obj['id']),obj['id'])
        other=next(o for o in derived.collision_objects if o['id']==obj['id'])
        for key in ('type','size_m','radius_m','length_m'):
            if obj.get(key)!=other.get(key):raise SceneError('geometry dimensions changed')
    for port in world.ports:
        original=world.frame_graph.resolve('base_link',port['frame'])
        other=next(p for p in derived.ports if p['id']==port['id'])
        current=derived.frame_graph.resolve('base_link',other['frame'])
        equal(w_b@current,original,port['id'])
        for axis in ('insertion_axis_local','key_axis_local'):
            equal(w_b[:3,:3]@current[:3,:3]@np.asarray(other[axis]),original[:3,:3]@np.asarray(port[axis]),port['id']+'/'+axis)
    expected_scene=derive_scene(world,pose)
    if expected_scene.canonical_scene_sha256!=derived.canonical_scene_sha256:raise SceneError('derived snapshot identity differs')
    return {'status':'PASS','frames_checked':len(world.frames),'objects_checked':len(world.collision_objects),
            'ports_checked':len(world.ports),'maximum_world_transform_element_error':maximum,
            'identity_baseline_preserved':pose.is_identity,'world_cabinet_fixed':True}
