"""Pure SE(3): T_target_source maps column vectors to target coordinates."""

from dataclasses import dataclass
import math
import numbers

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


