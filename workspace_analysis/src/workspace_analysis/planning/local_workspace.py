"""Explicit experiment-only near-cabinet constraint; conditional defaults stay intact."""
import numpy as np
from .workspace_constraint import WorkspaceConstraint


class LocalOnlyWorkspaceConstraint(WorkspaceConstraint):
    """Require TCP x >= trigger AND the inclusive Y/Z bounds at every query."""
    def to_dict(self):
        return {**super().to_dict(), 'mode': 'local_only'}

    def assess(self, world_tcp):
        # First use the base class to reject bad shapes/types/nonfinite values.
        result = super().assess(world_tcp)
        point = np.asarray(world_tcp, dtype=float)
        projected = point.copy()
        projected[0] = max(point[0], self.x_trigger_m)
        result = super().assess(projected)
        result['world_tcp_xyz_m'] = point.tolist()
        if self.enabled and point[0] < self.x_trigger_m:
            result['violations']['X'] = {'outside_m': self.x_trigger_m-float(point[0]),
                                         'lower_m': self.x_trigger_m, 'upper_m': None}
            result['reasons'].insert(0, 'WORKSPACE_X_BOUNDARY')
        result['valid'] = not result['reasons']
        result['active'] = self.enabled
        return result


def constraint_from_config(data):
    """Decode an explicit local-only identity without changing old config decoding."""
    values = dict(data)
    mode = values.pop('mode', 'conditional')
    if mode not in ('conditional', 'local_only'):
        raise ValueError('unknown workspace mode')
    cls = LocalOnlyWorkspaceConstraint if mode == 'local_only' else WorkspaceConstraint
    return cls(**values)
