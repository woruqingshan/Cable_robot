"""G2 independent uniform joint sampling and sequential reuse of G1-A FK."""

from dataclasses import dataclass, replace
from datetime import datetime, timezone
import numbers
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .fk import ForwardKinematics


ALGORITHM = "independent_uniform_joint_space"
SCOPE_LIMITATIONS = [
    "Joint-limit-constrained geometric FK samples only; no collision checking.",
    "Self/environment collision, motor safety limits and dynamics are not evaluated.",
    "RJ45 orientation, approach, contact and insertion/extraction constraints are not evaluated.",
    "Finite samples do not establish a strict reachable workspace boundary or volume.",
    "The URDF EE frame is not a calibrated RJ45 TCP.",
    "Uniform joint samples are not uniform Cartesian-position samples.",
]


class SamplingError(ValueError):
    """Invalid sampling input or an identified failed sample."""


@dataclass(frozen=True)
class WorkspaceSamples:
    q: np.ndarray
    position: np.ndarray
    rotation: np.ndarray
    quaternion_xyzw: np.ndarray
    fk_elapsed_s: float
    completed_utc: str
    seed: object = None
    bit_generator: object = None

    def arrays(self):
        return {key: getattr(self, key) for key in ("q", "position", "rotation", "quaternion_xyzw")}


def validate_integer(value, label, *, minimum):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Integral) or value < minimum:
        raise SamplingError(f"{label} must be an integer >= {minimum}")
    return int(value)


def checked_limits(limits):
    values = np.asarray(limits)
    if values.ndim != 2 or values.shape[0] < 1 or values.shape[1] != 2 or values.dtype.kind not in "iuf":
        raise SamplingError("Joint limits must be a real (J, 2) array with J >= 1")
    values = values.astype(np.float64)
    if not np.isfinite(values).all() or np.any(values[:, 0] > values[:, 1]):
        raise SamplingError("Joint limits must be finite, with lower <= upper")
    with np.errstate(over="ignore"):
        if not np.isfinite(values[:, 1] - values[:, 0]).all():
            raise SamplingError("Joint limit interval width overflows float64")
    return values


def sample_joint_positions(limits, samples, seed):
    count = validate_integer(samples, "samples", minimum=1)
    seed = validate_integer(seed, "seed", minimum=0)
    bounds = checked_limits(limits)
    rng = np.random.default_rng(seed)
    q = rng.uniform(bounds[:, 0], bounds[:, 1], size=(count, bounds.shape[0])).astype(np.float64)
    if not np.isfinite(q).all() or np.any(q < bounds[:, 0]) or np.any(q > bounds[:, 1]):
        raise SamplingError("Random generator returned invalid joint samples")
    return q


def validate_arrays(arrays, limits):
    """Validate encodings/row correspondence without recomputing FK."""
    bounds = checked_limits(limits)
    if set(arrays) != {"q", "position", "rotation", "quaternion_xyzw"}:
        raise SamplingError("Dataset must contain q, position, rotation and quaternion_xyzw")
    q = arrays["q"]
    if not isinstance(q, np.ndarray) or q.ndim != 2 or q.shape[0] < 1 or q.shape[1] != len(bounds):
        raise SamplingError("q must have shape (N, J), with N >= 1 and the model's J joints")
    count = q.shape[0]
    shapes = {"q": (count, len(bounds)), "position": (count, 3), "rotation": (count, 3, 3),
              "quaternion_xyzw": (count, 4)}
    for name, expected in shapes.items():
        value = arrays[name]
        if not isinstance(value, np.ndarray) or value.shape != expected or value.dtype != np.float64:
            raise SamplingError(f"{name}: expected float64 {expected}")
        invalid = ~np.isfinite(value).reshape(count, -1).all(axis=1)
        if invalid.any():
            raise SamplingError(f"{name}: non-finite value at sample_index={np.flatnonzero(invalid)[0]}")
    outside = ((q < bounds[:, 0]) | (q > bounds[:, 1])).any(axis=1)
    if outside.any():
        raise SamplingError(f"Joint limit violation at sample_index={np.flatnonzero(outside)[0]}")
    rotation = arrays["rotation"]
    orthogonal = np.max(np.abs(np.swapaxes(rotation, 1, 2) @ rotation - np.eye(3)), axis=(1, 2)) <= 1e-10
    proper = np.abs(np.linalg.det(rotation) - 1) <= 1e-10
    if not (orthogonal & proper).all():
        raise SamplingError(f"Invalid rotation at sample_index={np.flatnonzero(~(orthogonal & proper))[0]}")
    quat = arrays["quaternion_xyzw"]
    normalized = np.abs(np.linalg.norm(quat, axis=1) - 1) <= 1e-10
    if not normalized.all():
        raise SamplingError(f"Non-unit quaternion at sample_index={np.flatnonzero(~normalized)[0]}")
    consistent = np.max(np.abs(Rotation.from_quat(quat).as_matrix() - rotation), axis=(1, 2)) <= 1e-10
    if not consistent.all():
        raise SamplingError(f"Quaternion/rotation mismatch at sample_index={np.flatnonzero(~consistent)[0]}")
    return shapes


def evaluate_joint_samples(model, q, *, fk=None, progress=None):
    """Evaluate explicit inputs, including Zero/joint5 tests. No ROS nodes."""
    values = np.asarray(q)
    if (values.ndim != 2 or values.shape[0] < 1 or values.shape[1] != len(model.joint_order)
            or values.dtype.kind not in "iuf"):
        raise SamplingError("q must be a nonempty real (N, J) array in model joint order")
    values = values.astype(np.float64, copy=True)
    solver = ForwardKinematics(model) if fk is None else fk
    count = len(values)
    position = np.empty((count, 3), dtype=np.float64)
    rotation = np.empty((count, 3, 3), dtype=np.float64)
    quaternion = np.empty((count, 4), dtype=np.float64)
    started = time.perf_counter()
    for index, angles in enumerate(values):
        try:
            model.validate_q(angles)
            pose = solver.pose(angles)
            for name, shape in (("position", (3,)), ("rotation", (3, 3)), ("transform", (4, 4))):
                value = np.asarray(getattr(pose, name))
                if value.shape != shape or value.dtype.kind not in "iuf" or not np.isfinite(value).all():
                    raise SamplingError(f"FK returned invalid {name}")
            r, transform = pose.rotation, pose.transform
            if (not np.allclose(r.T @ r, np.eye(3), atol=1e-10, rtol=0)
                    or abs(np.linalg.det(r) - 1) > 1e-10
                    or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
                    or not np.allclose(transform[:3, :3], r, atol=1e-12, rtol=0)
                    or not np.allclose(transform[:3, 3], pose.position, atol=1e-12, rtol=0)):
                raise SamplingError("FK returned an invalid or inconsistent homogeneous transform")
            position[index], rotation[index] = pose.position, r
            quaternion[index] = Rotation.from_matrix(r).as_quat()
            if progress is not None:
                progress(index + 1, count)
        except Exception as exc:
            raise SamplingError(f"FK failed at sample_index={index}, q={angles.tolist()}: {exc}") from exc
    elapsed = time.perf_counter() - started
    result = WorkspaceSamples(values, position, rotation, quaternion, elapsed,
                              datetime.now(timezone.utc).isoformat())
    validate_arrays(result.arrays(), model.limits)
    return result


def sample_workspace(model, samples=10000, seed=42, *, fk=None, progress=None):
    q = sample_joint_positions(model.limits, samples, seed)
    result = evaluate_joint_samples(model, q, fk=fk, progress=progress)
    generator = np.random.default_rng(int(seed))
    return replace(result, seed=int(seed), bit_generator=type(generator.bit_generator).__name__)


def recheck_fk(model, arrays):
    """Recompute up to three recorded rows using the existing G1-A solver."""
    validate_arrays(arrays, model.limits)
    indices = sorted({0, len(arrays["q"]) // 2, len(arrays["q"]) - 1})
    fk = ForwardKinematics(model)
    positions, angles = [], []
    for index in indices:
        try:
            pose = fk.pose(arrays["q"][index])
            position_error = float(np.linalg.norm(pose.position - arrays["position"][index]))
            angle_error = float(Rotation.from_matrix(pose.rotation.T @ arrays["rotation"][index]).magnitude())
            if position_error > 1e-5 or angle_error > 1e-4:
                raise SamplingError(f"FK mismatch: {position_error} m, {angle_error} rad")
            positions.append(position_error)
            angles.append(angle_error)
        except Exception as exc:
            raise SamplingError(f"Stored FK recheck failed at sample_index={index}: {exc}") from exc
    return {"indices": indices, "max_position_error_m": max(positions), "max_rotation_error_rad": max(angles),
            "position_tolerance_m": 1e-5, "rotation_tolerance_rad": 1e-4}
