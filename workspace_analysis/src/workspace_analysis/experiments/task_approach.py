"""Generic adapter to unchanged sequential Cartesian IK; no RRT detours."""
from .task_context import TaskContext, require_resolved
from ..planning.cartesian_approach import sequential_approach, approach_settings
from ..planning.joint_path import PathError


def run_task_approach(context: TaskContext, start_q, *, settings: dict | None = None) -> dict:
    """Use an already reached pre-insertion configuration, checking every sample."""
    context.validate(); params = approach_settings(settings)
    if params['end_distance_m'] < .010:
        raise PathError('research free-space approach must stop at >=10 mm')
    q = context.model.validate_q(start_q)
    begin = len(context.checker.queries)
    result = sequential_approach(context.model, context.targets, q, context.checker, params)
    require_resolved(context.checker.queries[begin:])
    return result
