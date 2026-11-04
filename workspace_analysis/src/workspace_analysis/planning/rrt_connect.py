"""Small deterministic bidirectional RRT-Connect, using caller-supplied validity.

No alternate FK/collision implementation. Results are candidate polylines requiring
fresh full validation. Euclidean radians, bounded joints, no periodic wrapping.
Algorithm: Kuffner/LaValle (2000), alternating EXTEND and greedy CONNECT.
"""
from time import perf_counter
import numpy as np
from .joint_path import PathError, PlanningStopped, configuration, positive


def rrt_connect(start, goal, limits, validity, *, resolution_rad=.02, step_rad=.25,
                seed=20261104, max_iterations=1500, max_time_s=20., goal_bias=.05):
    start, goal = configuration(start), configuration(goal)
    limits = np.asarray(limits, dtype=float)
    if limits.shape != (5, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
        raise PathError('expected finite ordered limits[5,2]')
    positive(resolution_rad, 'resolution_rad'); positive(step_rad, 'step_rad'); positive(max_time_s, 'max_time_s')
    if type(max_iterations) is not int or max_iterations < 1: raise PathError('positive max_iterations required')
    if type(seed) is not int or seed < 0: raise PathError('nonnegative integer seed required')
    if type(goal_bias) not in (int, float) or not np.isfinite(goal_bias) or not 0 <= goal_bias <= 1:
        raise PathError('goal_bias must be in [0,1]')
    for q in (start, goal):
        if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]): raise PathError('endpoint outside joint limits')
    begin = perf_counter(); deadline = begin+max_time_s; before = validity.statistics()
    rng = np.random.default_rng(seed)
    trees = [{'q': [start], 'parent': [-1], 'start_root': True},
             {'q': [goal], 'parent': [-1], 'start_root': False}]
    iterations = 0; extensions = 0

    def result(status, path=None):
        return {'status': status, 'search_found': path is not None, 'waypoints': None if path is None else np.asarray(path).tolist(),
                'iterations': iterations, 'extensions': extensions, 'tree_sizes': [len(t['q']) for t in trees],
                'elapsed_s': perf_counter()-begin, 'seed': seed, 'fully_revalidated': False,
                'query_statistics': {k: validity.statistics()[k]-v for k, v in before.items()},
                'timeout_scope': 'cooperative search wall time, checked between native calls; final revalidation is separate'}

    def extend(tree, target):
        nonlocal extensions
        if perf_counter() >= deadline: raise PlanningStopped('RRT search deadline')
        distances = np.linalg.norm(np.asarray(tree['q'])-target, axis=1)
        nearest = int(np.argmin(distances)); distance = float(distances[nearest])
        if distance <= 1e-12: return 'REACHED', nearest
        q = target.copy() if distance <= step_rad else tree['q'][nearest]+step_rad/distance*(target-tree['q'][nearest])
        extensions += 1
        if not validity.edge(tree['q'][nearest], q, resolution_rad, deadline=deadline)['valid']:
            return 'TRAPPED', nearest
        tree['q'].append(q); tree['parent'].append(nearest)
        return ('REACHED' if distance <= step_rad else 'ADVANCED'), len(tree['q'])-1

    def branch(tree, index):
        path = []
        while index != -1:
            path.append(tree['q'][index]); index = tree['parent'][index]
        return path[::-1]

    try:
        if not validity.check(start, deadline=deadline)['valid']: return result('INVALID_START')
        if not validity.check(goal, deadline=deadline)['valid']: return result('INVALID_GOAL')
        if np.array_equal(start, goal): return result('PATH_FOUND_UNVERIFIED', [start])
        for iterations in range(1, max_iterations+1):
            a, b = trees if iterations % 2 else trees[::-1]
            target = b['q'][0].copy() if rng.random() < goal_bias else rng.uniform(limits[:, 0], limits[:, 1])
            state, ia = extend(a, target)
            if state == 'TRAPPED': continue
            while True:
                state, ib = extend(b, a['q'][ia])
                if state != 'ADVANCED': break
            if state == 'REACHED':
                first, second = branch(a, ia), branch(b, ib)
                path = first+second[-2::-1] if a['start_root'] else second+first[-2::-1]
                # Enforce exact problem endpoints even for tolerance-sized tree joins.
                path[0], path[-1] = start, goal
                return result('PATH_FOUND_UNVERIFIED', path)
    except PlanningStopped:
        return result('TIME_BUDGET_EXHAUSTED')
    return result('ITERATION_BUDGET_EXHAUSTED')
