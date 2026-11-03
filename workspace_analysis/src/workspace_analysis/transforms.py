"""SE(3), immutable poses and rooted frame graphs. NumPy/SciPy only.

T_target_source maps source-frame column vectors into target coordinates.
Every public array result is a new allocation; immutable objects store tuples.
"""

from collections import deque
from dataclasses import dataclass, field
import math
import numbers
import re
from types import MappingProxyType
from typing import Mapping, Protocol, Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial.transform import Rotation


QUATERNION_NORM_TOLERANCE = 1e-8
ROTATION_TOLERANCE = 1e-10


class TransformError(ValueError):
    """Invalid rotation, rigid transform, vector or frame graph."""


def _array(value: ArrayLike, shape: tuple[int, ...], label: str) -> NDArray[np.float64]:
    try:
        # Preserve bools in mixed Python lists before float conversion.
        raw = np.asarray(value, dtype=object)
        if raw.shape != shape or any(isinstance(x, (bool, np.bool_)) or not isinstance(x, numbers.Real)
                                     for x in raw.flat):
            raise ValueError(f'expected real numbers with shape {shape}, excluding bool')
        result = raw.astype(np.float64)
        if not np.isfinite(result).all():
            raise ValueError('expected finite values')
        result[result == 0] = 0.0
        return result
    except (TypeError, ValueError, OverflowError) as exc:
        raise TransformError(f'{label}: {exc}') from exc


def _frame_name(name: str, label: str) -> str:
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*(?:/[A-Za-z_][A-Za-z0-9_-]*)*', name):
        raise TransformError(f'{label}: expected a nonempty frame identifier without whitespace/leading slash')
    return name


def canonical_quaternion(quaternion_xyzw: ArrayLike) -> NDArray[np.float64]:
    """Validate norm within 1e-8, normalize, choose first nonzero (w,x,y,z)>0.

    q and -q yield identical results. Signed zero is canonicalized to +0.
    """
    q = _array(quaternion_xyzw, (4,), 'quaternion_xyzw')
    norm = math.hypot(*q)
    if not math.isfinite(norm) or abs(norm - 1) > QUATERNION_NORM_TOLERANCE:
        raise TransformError('quaternion_xyzw: expected unit norm within 1e-8 (no zero quaternion)')
    q /= norm
    for component in q[[3, 0, 1, 2]]:
        if component != 0:
            if component < 0:
                q *= -1
            break
    q[q == 0] = 0.0
    return q


def quaternion_to_matrix(quaternion_xyzw: ArrayLike) -> NDArray[np.float64]:
    """Convert a validated unit XYZW quaternion to a new SO(3) matrix."""
    return Rotation.from_quat(canonical_quaternion(quaternion_xyzw)).as_matrix()


def validate_rotation(rotation: ArrayLike) -> NDArray[np.float64]:
    """Copy a finite 3x3 SO(3) matrix; never repair an invalid rotation."""
    r = _array(rotation, (3, 3), 'rotation')
    with np.errstate(over='ignore', invalid='ignore'):
        if (not np.allclose(r.T @ r, np.eye(3), atol=ROTATION_TOLERANCE, rtol=0)
                or not abs(np.linalg.det(r) - 1) <= ROTATION_TOLERANCE):
            raise TransformError('rotation: expected SO(3), orthogonal with determinant +1')
    return r


def matrix_to_quaternion(rotation: ArrayLike) -> NDArray[np.float64]:
    """Convert SO(3) to canonical XYZW without accepting SciPy's matrix repair."""
    return canonical_quaternion(Rotation.from_matrix(validate_rotation(rotation)).as_quat())


def pose_to_matrix(position: ArrayLike, quaternion_xyzw: ArrayLike) -> NDArray[np.float64]:
    """Return ^parent T_child from child's origin and XYZW orientation in parent."""
    result = np.eye(4)
    result[:3, 3] = _array(position, (3,), 'position_m')
    result[:3, :3] = quaternion_to_matrix(quaternion_xyzw)
    return result


def validate_transform(transform: ArrayLike) -> NDArray[np.float64]:
    """Copy an SE(3) matrix, rejecting invalid shape, SO(3) or homogeneous row."""
    result = _array(transform, (4, 4), 'transform')
    if not np.array_equal(result[3], [0., 0., 0., 1.]):
        raise TransformError('transform: bottom row must be exactly [0,0,0,1]')
    validate_rotation(result[:3, :3])
    return result


def compose_transforms(first: ArrayLike, second: ArrayLike) -> NDArray[np.float64]:
    """Return first @ second: ^A T_B @ ^B T_C = ^A T_C."""
    a, b = validate_transform(first), validate_transform(second)
    with np.errstate(over='ignore', invalid='ignore'):
        return validate_transform(a @ b)


def invert_transform(transform: ArrayLike) -> NDArray[np.float64]:
    """Return ^source T_target from ^target T_source using the rigid inverse."""
    t = validate_transform(transform)
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    with np.errstate(over='ignore', invalid='ignore'):
        result[:3, 3] = -t[:3, :3].T @ t[:3, 3]
    return validate_transform(result)


def transform_point(transform: ArrayLike, point: ArrayLike) -> NDArray[np.float64]:
    """Map a single source point: p_target = R*p_source + translation."""
    t, p = validate_transform(transform), _array(point, (3,), 'point')
    with np.errstate(over='ignore', invalid='ignore'):
        return _array(t[:3, :3] @ p + t[:3, 3], (3,), 'transformed point')


def transform_direction(transform: ArrayLike, direction: ArrayLike) -> NDArray[np.float64]:
    """Rotate a source direction, without translation or implicit normalization."""
    t, d = validate_transform(transform), _array(direction, (3,), 'direction')
    with np.errstate(over='ignore', invalid='ignore'):
        return _array(t[:3, :3] @ d, (3,), 'transformed direction')


@dataclass(frozen=True, slots=True)
class Pose:
    """Deeply immutable local pose; matrix() returns a fresh mutable array."""

    position_m: tuple[float, ...]
    quaternion_xyzw: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, 'position_m', tuple(_array(self.position_m, (3,), 'position_m').tolist()))
        object.__setattr__(self, 'quaternion_xyzw', tuple(canonical_quaternion(self.quaternion_xyzw).tolist()))

    def matrix(self) -> NDArray[np.float64]:
        """Return the new 4x4 matrix represented by this pose."""
        return pose_to_matrix(self.position_m, self.quaternion_xyzw)


@dataclass(frozen=True, slots=True)
class Frame:
    """A single edge: pose is ^parent T_id. The base_link root is implicit."""

    id: str
    parent: str
    pose: Pose

    def __post_init__(self) -> None:
        _frame_name(self.id, 'frame.id')
        _frame_name(self.parent, f'frames[{self.id}].parent')
        if not isinstance(self.pose, Pose):
            raise TransformError(f'frames[{self.id}].pose: expected immutable Pose')


@dataclass(frozen=True, slots=True)
class FrameGraph:
    """Immutable single-parent tree rooted at base_link; resolve in either direction."""

    frames: tuple[Frame, ...]
    root: str = 'base_link'
    _to_root: Mapping[str, tuple[tuple[float, ...], ...]] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.root != 'base_link':
            raise TransformError('reference_frame: B1 requires base_link')
        frames = tuple(self.frames)
        if any(not isinstance(f, Frame) for f in frames):
            raise TransformError('frames: expected immutable Frame records')
        by_id: dict[str, Frame] = {}
        for frame in frames:
            if frame.id == self.root or frame.id in by_id:
                raise TransformError(f'frames[{frame.id}]: duplicate ID/multiple parents or redefined root')
            by_id[frame.id] = frame
        children: dict[str, list[Frame]] = {key: [] for key in [self.root, *by_id]}
        for frame in frames:
            if frame.parent not in children:
                raise TransformError(f'frames[{frame.id}].parent: missing frame {frame.parent!r}')
            children[frame.parent].append(frame)
        transforms = {self.root: np.eye(4)}
        pending = deque([self.root])
        while pending:
            parent = pending.popleft()
            for frame in children[parent]:
                transforms[frame.id] = compose_transforms(transforms[parent], frame.pose.matrix())
                pending.append(frame.id)
        unresolved = set(by_id) - transforms.keys()
        if unresolved:
            raise TransformError(f'frames: cycle/disconnected from base_link: {sorted(unresolved)}')
        object.__setattr__(self, 'frames', tuple(sorted(frames, key=lambda f: f.id)))
        object.__setattr__(self, '_to_root', MappingProxyType({
            name: tuple(tuple(row) for row in value.tolist()) for name, value in transforms.items()}))

    def resolve(self, target_frame: str, source_frame: str) -> NDArray[np.float64]:
        """Return ^target T_source = inverse(^base T_target) @ ^base T_source."""
        for label, name in [('target_frame', target_frame), ('source_frame', source_frame)]:
            if not isinstance(name, str) or name not in self._to_root:
                raise TransformError(f'{label}: unknown frame {name!r}')
        return compose_transforms(invert_transform(self._to_root[target_frame]), self._to_root[source_frame])

    def is_descendant(self, child: str, ancestor: str) -> bool:
        """Return strict ancestry (a frame is not its own descendant)."""
        if child not in self._to_root or ancestor not in self._to_root:
            raise TransformError(f'frames: unknown child/ancestor {child!r}/{ancestor!r}')
        parents = {f.id: f.parent for f in self.frames}
        while child != self.root:
            child = parents[child]
            if child == ancestor:
                return True
        return False


class _HasFrameGraph(Protocol):
    frame_graph: FrameGraph


def resolve_transform(scene: _HasFrameGraph | FrameGraph, target_frame: str,
                      source_frame: str) -> NDArray[np.float64]:
    """Resolve ^target T_source from a SceneSnapshot or validated FrameGraph."""
    graph = scene if isinstance(scene, FrameGraph) else getattr(scene, 'frame_graph', None)
    if not isinstance(graph, FrameGraph):
        raise TransformError('scene: expected a SceneSnapshot or FrameGraph')
    return graph.resolve(target_frame, source_frame)
