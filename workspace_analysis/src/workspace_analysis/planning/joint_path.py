"""Bounded revolute joint-space geometry in radians; no angle wrapping or FK."""
import math
import numpy as np


class PathError(ValueError):
    """Invalid path/settings, not evidence of task unreachability."""


class PlanningStopped(RuntimeError):
    """Cooperative planning deadline reached between collision queries."""


def positive(value, name):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise PathError(name + ' must be finite and positive')
    return float(value)


def configuration(q):
    a = np.asarray(q)
    if a.shape != (5,) or a.dtype.kind not in 'fiu' or not np.isfinite(a).all():
        raise PathError('expected five finite real joint angles')
    return a.astype(float)


def interpolate(a, b, resolution_rad):
    """Include BOTH exact endpoints; max per-joint step <= resolution_rad.

    These URDF revolute joints have finite stops: crossing +/-pi is not a shortcut.
    The 100,000-interval cap rejects accidental unbounded allocations.
    """
    a, b = configuration(a), configuration(b)
    resolution_rad = positive(resolution_rad, 'resolution_rad')
    count = max(1, math.ceil(float(np.max(np.abs(b-a))) / resolution_rad))
    if count > 100000:
        raise PathError('edge exceeds 100000 interpolation intervals')
    q = a + np.linspace(0., 1., count+1)[:, None]*(b-a)
    q[0], q[-1] = a, b
    return q


def sample_path(waypoints, resolution_rad):
    """Return dense states and their original segment/fraction provenance."""
    positive(resolution_rad, 'resolution_rad')
    q = np.asarray(waypoints)
    if q.ndim != 2 or q.shape[1:] != (5,) or len(q) == 0:
        raise PathError('nonempty waypoints[N,5] required')
    q = np.array([configuration(row) for row in q])
    if len(q) == 1:
        return q, [{'segment_index': 0, 'fraction': 0.}]
    states, provenance = [], []
    for segment in range(len(q)-1):
        edge = interpolate(q[segment], q[segment+1], resolution_rad)
        for i in range(0 if segment == 0 else 1, len(edge)):
            states.append(edge[i]); provenance.append({'segment_index': segment, 'fraction': i/(len(edge)-1)})
    return np.array(states), provenance


def path_length(waypoints):
    """Sum Euclidean five-joint distances [rad], NOT TCP length or execution time."""
    q = np.asarray(waypoints)
    if q.ndim != 2 or q.shape[1:] != (5,) or len(q) == 0:
        raise PathError('nonempty waypoints[N,5] required')
    q = np.array([configuration(row) for row in q])
    return float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
