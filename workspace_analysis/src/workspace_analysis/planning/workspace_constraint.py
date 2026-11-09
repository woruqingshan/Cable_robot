"""World-frame conditional TCP boundary, independent of collision and search."""
from dataclasses import dataclass
from hashlib import sha256
import json
import math
import numpy as np
from ..geometry.se3 import validate_transform


@dataclass(frozen=True)
class WorkspaceConstraint:
    enabled: bool = True
    x_trigger_m: float = .250
    y_min_m: float = -.150
    y_max_m: float = .150
    z_min_m: float = .040
    z_max_m: float = .400
    switch_band_m: float = .002
    switch_resolution_rad: float = .001

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('workspace enabled must be boolean')
        for name,value in self.__dict__.items():
            if name != 'enabled' and (type(value) not in (int,float) or not math.isfinite(value)):
                raise ValueError('finite workspace parameter required: '+name)
        if self.y_min_m >= self.y_max_m or self.z_min_m >= self.z_max_m:
            raise ValueError('workspace lower bound must precede upper bound')
        if self.switch_band_m <= 0 or self.switch_resolution_rad <= 0:
            raise ValueError('positive switch refinement settings required')

    def to_dict(self):
        return dict(self.__dict__)

    @property
    def sha256(self):
        return sha256(json.dumps(self.to_dict(),sort_keys=True,allow_nan=False).encode()).hexdigest()

    def assess(self, world_tcp):
        point=np.asarray(world_tcp)
        if point.shape != (3,) or point.dtype.kind not in 'fiu' or not np.isfinite(point).all():
            raise ValueError('finite world TCP XYZ required')
        active=self.enabled and point[0] >= self.x_trigger_m
        violations={}
        if active:
            for axis,index,lower,upper in [('Y',1,self.y_min_m,self.y_max_m),('Z',2,self.z_min_m,self.z_max_m)]:
                excess=max(lower-float(point[index]),float(point[index])-upper,0.)
                if excess > 0:
                    violations[axis]={'outside_m':excess,'lower_m':lower,'upper_m':upper}
        return {'valid':not violations,'active':bool(active),'world_tcp_xyz_m':point.tolist(),
                'reasons':['WORKSPACE_'+axis+'_BOUNDARY' for axis in violations],
                'violations':violations,'config':self.to_dict(),'config_sha256':self.sha256}


class TCPWorkspaceCheck:
    """Bind immutable task TCP and base transform to one evaluator session."""
    def __init__(self, constraint, world_from_base, ee_from_tcp, base_pose):
        self.constraint=constraint
        self.world_from_base=validate_transform(world_from_base).copy()
        self.ee_from_tcp=validate_transform(ee_from_tcp).copy()
        self.world_from_base.flags.writeable=False
        self.ee_from_tcp.flags.writeable=False
        self.base_pose=dict(base_pose)

    def assess(self, base_from_ee):
        transform=self.world_from_base@validate_transform(base_from_ee)@self.ee_from_tcp
        return {**self.constraint.assess(transform[:3,3]),'base_pose':dict(self.base_pose)}

    def refine(self, states, provenance, fk):
        """Additional .001-rad samples near detected switch crossings, discrete only."""
        if not self.constraint.enabled or len(states)<2:
            return states,provenance
        from .joint_path import interpolate
        output=[states[0]]
        origins=[provenance[0]]
        def x_value(q):
            return self.assess(fk(q))['world_tcp_xyz_m'][0]
        for index in range(1,len(states)):
            a,b=states[index-1],states[index]
            xs=[x_value(a),x_value((a+b)/2),x_value(b)]
            trigger=self.constraint.x_trigger_m
            near=min(xs)<=trigger+self.constraint.switch_band_m and max(xs)>=trigger-self.constraint.switch_band_m
            edge=interpolate(a,b,self.constraint.switch_resolution_rad) if near else np.array([a,b])
            for offset,q in enumerate(edge[1:],1):
                output.append(q)
                origins.append({**provenance[index],'switch_refined':near,
                    'coarse_interval_end_index':index,'local_fraction':offset/(len(edge)-1)})
        return np.asarray(output),origins
