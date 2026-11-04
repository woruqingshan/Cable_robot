"""Immutable replay data contracts. No ROS, Qt, IK or collision session."""
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Mapping
import numpy as np
from .errors import ReplayError

PHASES = ('Path', 'Approach', 'Path + Approach')


def frozen_array(value: Any, shape: tuple, kind: str) -> np.ndarray:
    raw = np.asarray(value)
    if raw.shape != shape or raw.dtype.kind not in kind:
        raise ReplayError('TRAJECTORY_TYPE', f'expected {shape}/{kind}, got {raw.shape}/{raw.dtype}')
    if raw.dtype.kind in 'fiu' and not np.isfinite(raw).all():
        raise ReplayError('TRAJECTORY_NONFINITE', 'nonfinite saved sample')
    # Immutable byte backing: callers cannot re-enable ndarray.writeable.
    return np.frombuffer(raw.tobytes(), dtype=raw.dtype).reshape(shape)


@dataclass(frozen=True)
class Trajectory:
    q: np.ndarray
    tcp: np.ndarray
    valid: np.ndarray
    query_id: np.ndarray
    self_clearance: np.ndarray
    environment_clearance: np.ndarray
    source_frames: tuple[tuple[str, int], ...]
    phases: tuple[str, ...]

    def __post_init__(self) -> None:
        from workspace_analysis.geometry.se3 import validate_transform
        q = np.asarray(self.q)
        if q.ndim != 2 or q.shape[1:] != (5,) or not 1 <= len(q) <= 100000:
            raise ReplayError('TRAJECTORY_SHAPE', 'nonempty saved q[N,5] required')
        n = len(q)
        for key, value, shape, kind in [
            ('q', self.q, (n,5), 'f'), ('tcp', self.tcp, (n,4,4), 'f'),
            ('valid', self.valid, (n,), 'b'), ('query_id', self.query_id, (n,), 'iu'),
            ('self_clearance', self.self_clearance, (n,), 'f'),
            ('environment_clearance', self.environment_clearance, (n,), 'f')]:
            object.__setattr__(self, key, frozen_array(value, shape, kind))
        if not self.valid.all() or np.any(self.query_id < 0):
            raise ReplayError('UNVALIDATED_TRAJECTORY', 'every replay sample must be saved valid')
        if np.any(self.self_clearance < 0) or np.any(self.environment_clearance < 0):
            raise ReplayError('INVALID_CLEARANCE', 'unsigned model separations required')
        for t in self.tcp: validate_transform(t)
        if len(self.source_frames) != n or len(self.phases) != n or any(p not in PHASES[:2] for p in self.phases):
            raise ReplayError('FRAME_PROVENANCE', 'one source/index/phase required per sample')
        for name, index in self.source_frames:
            if not isinstance(name, str) or type(index) is not int or index < 0:
                raise ReplayError('FRAME_PROVENANCE', 'invalid original sample reference')


def joined(path: Trajectory, approach: Trajectory) -> Trajectory:
    """Concatenate ORIGINAL samples; share one exact joint boundary, no interpolation."""
    if not np.array_equal(path.q[-1], approach.q[0]) or not np.allclose(path.tcp[-1], approach.tcp[0], atol=1e-10, rtol=0):
        raise ReplayError('PATH_APPROACH_MISMATCH', 'saved boundary configurations/TCP disagree')
    values = {key: np.concatenate([getattr(path,key), getattr(approach,key)[1:]], axis=0)
              for key in ['q','tcp','valid','query_id','self_clearance','environment_clearance']}
    return Trajectory(**values, source_frames=path.source_frames+approach.source_frames[1:],
                      phases=path.phases+approach.phases[1:])


@dataclass(frozen=True)
class Goal:
    solution_id: int
    q: tuple[float, ...]
    status: str
    paths: Mapping[str, Trajectory]
    approach: Trajectory | None
    planner_results: Mapping[str, dict] = field(default_factory=dict)
    configuration_candidate: bool = True
    display_tcp: np.ndarray | None = None

    def __post_init__(self) -> None:
        if type(self.solution_id) is not int or self.solution_id < 0:
            raise ReplayError('GOAL_ID', 'nonnegative integer required')
        if set(self.paths)-{'linear','rrt_connect'}: raise ReplayError('PLANNER', 'unsupported saved planner')
        object.__setattr__(self, 'paths', MappingProxyType(dict(self.paths)))
        results = dict(self.planner_results) or {m:{'success':True,'status':'SUCCESS'} for m in self.paths}
        if set(results)-{'linear','rrt_connect'} or not set(self.paths)<=set(results):
            raise ReplayError('PLANNER','planner results/path methods differ')
        for method, result in results.items():
            if type(result.get('success')) is not bool or not isinstance(result.get('status'),str) or result['success'] != (method in self.paths):
                raise ReplayError('PLANNER','success must correspond to a real saved trajectory')
        object.__setattr__(self,'planner_results',MappingProxyType({k:MappingProxyType(dict(v)) for k,v in results.items()}))
        if self.display_tcp is not None:
            from workspace_analysis.geometry.se3 import validate_transform
            validate_transform(self.display_tcp)
            object.__setattr__(self,'display_tcp',frozen_array(self.display_tcp,(4,4),'f'))
        for path in self.paths.values():
            if not np.array_equal(path.q[-1], self.q): raise ReplayError('INVALID_GOAL', 'path endpoint differs from IK goal')
            if self.approach is not None: joined(path, self.approach)

    def sequence(self, planner: str, phase: str) -> Trajectory:
        if planner not in self.paths or phase not in PHASES:
            raise ReplayError('UNAVAILABLE_SELECTION', 'planner/phase not present in saved valid evidence')
        if phase == 'Path': return self.paths[planner]
        if self.approach is None: raise ReplayError('UNAVAILABLE_SELECTION', 'no accepted saved approach')
        if phase == 'Approach': return self.approach
        return joined(self.paths[planner], self.approach)


@dataclass(frozen=True)
class Port:
    port_id: str
    status: str
    detail: str
    goals: tuple[Goal, ...]

    def __post_init__(self) -> None:
        if len({g.solution_id for g in self.goals}) != len(self.goals):
            raise ReplayError('GOAL_ID', 'duplicate goal')


@dataclass(frozen=True)
class Catalog:
    ports: Mapping[str, Port]
    model: Any
    scene: Any
    targets: Mapping[str, Any]
    start_q: tuple[float, ...]
    start_tcp: np.ndarray
    gripper: float
    manifest_sha256: str
    verification: dict
    world_from_base: np.ndarray = field(default_factory=lambda: np.eye(4))
    base_pose_info: dict = field(default_factory=lambda: {'base_pose':{'id':'b0','x_m':0.,'y_m':0.,'yaw_rad':0.},'world_cabinet_fixed':True})

    def __post_init__(self) -> None:
        if not self.ports or set(self.ports) != set(self.targets): raise ReplayError('PORTS', 'catalog/targets differ')
        if any(k != p.port_id for k,p in self.ports.items()): raise ReplayError('PORTS', 'port key differs')
        self.model.validate_q(self.start_q)
        object.__setattr__(self,'ports',MappingProxyType(dict(self.ports)))
        object.__setattr__(self,'targets',MappingProxyType(dict(self.targets)))
        object.__setattr__(self,'start_tcp',frozen_array(self.start_tcp,(4,4),'f'))
        from workspace_analysis.geometry.se3 import validate_transform
        validate_transform(self.world_from_base)
        object.__setattr__(self,'world_from_base',frozen_array(self.world_from_base,(4,4),'f'))

    def describe(self) -> dict:
        return {name: {'status':p.status, 'detail':p.detail, 'goals':[
            {'solution_id':g.solution_id,'status':g.status,'planners':list(g.planner_results),
             'planner_results':{m:dict(v) for m,v in g.planner_results.items()},
             'configuration_candidate':g.configuration_candidate,'playable_planners':list(g.paths),
             'phases':list(PHASES if g.approach is not None else PHASES[:1]),
             'path_samples':{m:len(t.q) for m,t in g.paths.items()},
             'approach_samples':0 if g.approach is None else len(g.approach.q)} for g in p.goals]}
            for name,p in self.ports.items()}


def joint_values(model: Any, q: Any, gripper: float) -> tuple[tuple[str, ...], tuple[float, ...]]:
    """URDF-derived independent joints; RSP computes mimic transforms itself."""
    values = model.joint_positions(q, {'r_joint':gripper})
    independent = [n for n,j in model.joints.items() if j.kind != 'fixed' and not j.mimic_joint]
    names = tuple(model.joint_order) + tuple(sorted(set(independent)-set(model.joint_order)))
    return names, tuple(float(values[n]) for n in names)
