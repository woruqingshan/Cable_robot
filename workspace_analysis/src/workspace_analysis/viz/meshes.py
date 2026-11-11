"""NumPy-only local box/cylinder triangulation and wire topology; no Figure imports."""

import math
import numbers

import numpy as np
from numpy.typing import ArrayLike, NDArray


class SceneVisualizationError(ValueError):
    """Invalid display input, unavailable Plotly, or failed HTML export."""


def _positive(value: float, label: str) -> float:
    if (isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real)
            or not math.isfinite(float(value)) or value <= 0):
        raise SceneVisualizationError(f'{label}: expected a finite positive number')
    return float(value)


def _segments(value: int) -> int:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Integral) or not 8 <= value <= 256:
        raise SceneVisualizationError('cylinder_segments: expected integer in [8,256]')
    return int(value)


def box_mesh(size_m: ArrayLike) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """Return 8 centered local vertices and 12 explicit triangles for a box."""
    raw = np.asarray(size_m, dtype=object)
    if raw.shape != (3,):
        raise SceneVisualizationError('size_m: expected three positive full edge lengths')
    size = np.array([_positive(v, 'size_m') for v in raw], dtype=np.float64)
    vertices = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                         [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]], dtype=np.float64) * (size / 2)
    faces = np.array([[0,2,1], [0,3,2], [4,5,6], [4,6,7], [0,1,5], [0,5,4],
                      [1,2,6], [1,6,5], [2,3,7], [2,7,6], [3,0,4], [3,4,7]], dtype=np.int64)
    return vertices, faces


def cylinder_mesh(radius_m: float, length_m: float, segments: int = 48
                  ) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """Return centered +Z cylinder mesh: two rings, two centers, sides and caps."""
    radius, length, n = _positive(radius_m, 'radius_m'), _positive(length_m, 'length_m'), _segments(segments)
    angles = np.arange(n) * (2 * np.pi / n)
    ring = np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))
    vertices = np.vstack((np.column_stack((ring, np.full(n, -length / 2))),
                          np.column_stack((ring, np.full(n, length / 2))), [[0,0,-length/2], [0,0,length/2]]))
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces.extend([(i,j,j+n), (i,j+n,i+n), (2*n,j,i), (2*n+1,i+n,j+n)])
    return vertices, np.asarray(faces, dtype=np.int64)


def _wire_edges(obj: object, n: int) -> list[tuple[int, int]]:
    if obj['type'] == 'box':
        return [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    return ([(i, (i+1) % n) for i in range(n)] + [(i+n, (i+1) % n+n) for i in range(n)]
            + [(i, i+n) for i in (0, n//4, n//2, 3*n//4)])


