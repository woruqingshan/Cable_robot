"""Generic Linear and RRT trials; final checks always bypass the validity cache."""
from time import perf_counter
import numpy as np
from .task_context import TaskContext, require_resolved
from .p22_path_planning import settings_checked
from ..planning.rrt_connect import rrt_connect
from ..planning.joint_path import path_length, PathError


def run_task_path(context: TaskContext, start_q, goal_q, *, method: str,
                  settings: dict | None = None) -> dict:
    """Return search and independent final validation separately, with exact endpoints."""
    context.validate(); params = settings_checked(settings or {})
    if method not in ('linear', 'rrt_connect'):
        raise PathError('unsupported planner method')
    model, checker = context.model, context.checker
    start = model.validate_q(start_q); goal = model.validate_q(goal_q)
    before = checker.statistics(); query_begin = len(checker.queries)
    checker.cache.clear(); begin = perf_counter()
    endpoints = [checker.check(q, fresh=True) for q in (start, goal)]
    require_resolved(endpoints)
    search = None; validation = None; path = None; search_elapsed = 0.
    if not endpoints[0]['valid']:
        status = 'INVALID_START'
    elif not endpoints[1]['valid']:
        status = 'INVALID_GOAL'
    else:
        t = perf_counter()
        if method == 'linear':
            path = [start.tolist(), goal.tolist()]
        else:
            search = rrt_connect(start, goal, model.limits, checker,
                resolution_rad=params['edge_resolution_rad'], step_rad=params['step_rad'],
                seed=params['random_seed'], max_iterations=params['max_iterations'],
                max_time_s=params['max_time_s'], goal_bias=params['goal_bias'])
            path = search['waypoints']
        search_elapsed = perf_counter() - t
        require_resolved(checker.queries[query_begin:])
        if path is None:
            status = 'PLANNER_TIMEOUT' if search['status'] == 'TIME_BUDGET_EXHAUSTED' else 'PLANNER_BUDGET_EXHAUSTED'
        else:
            if not np.array_equal(path[0], start) or not np.array_equal(path[-1], goal):
                raise PathError('planner changed problem endpoints')
            validation = checker.path(path, params['verification_resolution_rad'], fresh=True, stop_first=False)
            require_resolved(validation['states'])
            status = 'VALID_DISCRETE_PATH' if validation['valid'] else 'PATH_VALIDATION_FAILED'
    return {'method': method, 'status': status, 'success': status == 'VALID_DISCRETE_PATH',
            'start_q': start.tolist(), 'goal_q': goal.tolist(), 'settings': params,
            'search': search, 'waypoints': path, 'validation': validation,
            'search_elapsed_s': search_elapsed, 'validation_elapsed_s': None if validation is None else validation['elapsed_s'],
            'elapsed_s': perf_counter()-begin, 'path_length_rad': None if path is None else path_length(path),
            'query_statistics': {k: checker.statistics()[k]-v for k, v in before.items()},
            'continuous_collision_proven': False, 'usage': 'RESEARCH_ONLY'}
