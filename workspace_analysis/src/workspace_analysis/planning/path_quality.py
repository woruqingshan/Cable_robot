"""Metrics on actual checked paths, separate from search nodes and display frames."""
import numpy as np
from .joint_path import path_length


def tcp_quality(points, *, near_zero_m=1e-6, x_trigger_m=.250):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1:] != (3,) or not len(points) or not np.isfinite(points).all():
        raise ValueError('nonempty finite TCP XYZ[N,3] required')
    if not np.isfinite(near_zero_m) or near_zero_m <= 0 or not np.isfinite(x_trigger_m):
        raise ValueError('finite quality thresholds required')
    length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    direct = float(np.linalg.norm(points[-1]-points[0]))
    return {'tcp_path_length_m': length, 'endpoint_tcp_distance_m': direct,
            'tcp_detour_ratio': None if direct <= near_zero_m else length/direct,
            'detour_undefined_reason': 'NEAR_ZERO_ENDPOINT_DISTANCE' if direct <= near_zero_m else None,
            'workspace_exit': bool(np.any(points[:, 0] < x_trigger_m)),
            'tcp_x_min_m': float(points[:, 0].min()), 'tcp_x_max_m': float(points[:, 0].max()),
            'scope': 'checked discrete samples; no continuous guarantee'}


def path_quality(trial, world_points):
    validation = trial.get('validation')
    if validation is None:
        return {'status': 'NO_RETURNED_PATH', 'raw_waypoint_count': 0,
                'validated_sample_count': 0, 'replay_sample_count': 0}
    return {**tcp_quality(world_points), 'status': 'ACCEPTED' if trial['success'] else 'REJECTED_ATTEMPT',
            'joint_path_length_rad': path_length(trial['waypoints']),
            'raw_waypoint_count': len(trial['waypoints']),
            'validated_sample_count': len(validation['states']),
            'replay_sample_count': len(validation['states']) if trial['success'] else 0,
            'minimum_clearance': validation['clearance']}
