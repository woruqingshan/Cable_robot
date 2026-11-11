"""Standard randomized vertex shortcutting using the caller's immutable checker.

This post-process never changes RRT search. All accepted replacements have a
complete valid discrete edge and strictly reduce joint polyline length. Fresh
input/final checks are independent of the exact-q cache. No dynamic smoothness or
continuous safety claim is made.
"""
from time import perf_counter
import numpy as np
from .joint_path import PathError, configuration, path_length, positive


def _resolved(states):
    for state in states:
        if state.get('status') == 'ERROR' or any(code in state.get('reasons', []) for code in
                ('COLLISION_QUERY_ERROR', 'CLEARANCE_OR_COLLISION_UNVERIFIED',
                 'INCONSISTENT_SEMANTIC_RESULT', 'INVALID_VALIDITY_FIELD')):
            raise PathError('unresolved collision result during shortcutting')


def randomized_shortcut(waypoints, validity, *, resolution_rad=.01, seed=20261114,
                        max_attempts=100, reduction_epsilon_rad=1e-12):
    """Return a fresh-validated path or an explicit failed result, never unchecked q.

    Input is an already independently validated raw path. It is also checked fresh
    here before accepting any replacement, so an optimistic external success flag
    cannot authorize postprocessing. Candidate index pairs refer to the CURRENT
    waypoint array. Repeated edges still call the same validity interface; exact-q
    cache reuse is recorded by that checker.
    """
    positive(resolution_rad, 'resolution_rad')
    positive(reduction_epsilon_rad, 'reduction_epsilon_rad')
    if type(seed) is not int or seed < 0 or type(max_attempts) is not int or max_attempts < 0:
        raise PathError('nonnegative integer seed/attempt budget required')
    original = np.asarray(waypoints)
    if original.ndim != 2 or original.shape[1:] != (5,) or not len(original):
        raise PathError('nonempty waypoints[N,5] required')
    original = np.array([configuration(q) for q in original], copy=True)
    for q in original:
        validity.model.validate_q(q)
    settings = {'resolution_rad': resolution_rad, 'random_seed': seed,
                'max_attempts': max_attempts, 'reduction_epsilon_rad': reduction_epsilon_rad,
                'pair_selection': 'uniform over unordered nonadjacent current vertex pairs'}
    before = validity.statistics(); begin = perf_counter()
    initial = validity.path(original, resolution_rad, fresh=True, stop_first=False)
    _resolved(initial['states'])
    if not initial['valid'] or not initial['complete']:
        return {'status': 'INVALID_INPUT_PATH', 'success': False, 'waypoints': None,
                'settings': settings, 'attempts': [], 'accepted_shortcuts': 0,
                'input_validation': initial, 'validation': None,
                'elapsed_s': perf_counter()-begin, 'usage': 'RESEARCH_ONLY'}
    path = original.copy(); rng = np.random.default_rng(seed); attempts = []
    accepted = 0; loop_begin = perf_counter(); stop = 'ATTEMPT_BUDGET'
    for index in range(max_attempts):
        pairs = [(i, j) for i in range(len(path)) for j in range(i+2, len(path))]
        if not pairs:
            stop = 'NO_NONADJACENT_VERTEX_PAIRS'
            break
        i, j = pairs[int(rng.integers(len(pairs)))]
        previous = path_length(path); removed = path_length(path[i:j+1])
        direct = float(np.linalg.norm(path[j]-path[i])); candidate = np.concatenate((path[:i+1], path[j:]))
        row = {'attempt': index, 'i': i, 'j': j, 'from_q': path[i].tolist(), 'to_q': path[j].tolist(),
               'length_before_rad': previous, 'removed_length_rad': removed,
               'replacement_length_rad': direct, 'accepted': False, 'edge_validation': None}
        if previous-path_length(candidate) > reduction_epsilon_rad:
            edge = validity.edge(path[i], path[j], resolution_rad, fresh=False, stop_first=True)
            _resolved(edge['states'])
            # The authoritative checker ledger owns every full query record. Keep
            # lightweight per-attempt references instead of 100 repeated RobotStates.
            edge['states'] = [{'query_id':state['query_id'], 'q':state['q'],
                               'valid':state['valid'], 'reasons':state['reasons']}
                              for state in edge['states']]
            row['edge_validation'] = edge
            if edge['valid'] and edge['complete']:
                path = candidate; accepted += 1; row['accepted'] = True
        row['length_after_rad'] = path_length(path)
        attempts.append(row)
    shortcut_elapsed = perf_counter()-loop_begin
    np.testing.assert_array_equal(path[0], original[0]); np.testing.assert_array_equal(path[-1], original[-1])
    final = validity.path(path, resolution_rad, fresh=True, stop_first=False)
    _resolved(final['states']); fallback = None
    candidate_final = final if not final['valid'] else None
    if not final['valid']:
        fallback = validity.path(original, resolution_rad, fresh=True, stop_first=False)
        _resolved(fallback['states'])
        if fallback['valid']:
            path = original.copy(); final = fallback; status = 'FALLBACK_ORIGINAL_VALIDATED'
        else:
            return {'status': 'FINAL_VALIDATION_FAILURE', 'success': False, 'waypoints': None,
                    'settings': settings, 'attempts': attempts, 'accepted_shortcuts': accepted,
                    'input_validation': initial, 'validation': None, 'candidate_final_validation': candidate_final, 'failed_candidate_waypoints': path.tolist(), 'fallback_validation': fallback,
                    'elapsed_s': perf_counter()-begin, 'usage': 'RESEARCH_ONLY'}
    else:
        status = 'SHORTCUT_VALIDATED' if accepted else 'NO_IMPROVEMENT'
    return {'status': status, 'success': True, 'waypoints': path.tolist(), 'settings': settings,
            'attempts': attempts, 'attempts_executed': len(attempts), 'stop_reason': stop,
            'accepted_shortcuts': accepted, 'original_waypoint_count': len(original),
            'optimized_waypoint_count': len(path), 'input_validation': initial, 'validation': final,
            'fallback_validation': fallback, 'candidate_final_validation': candidate_final, 'shortcut_elapsed_s': shortcut_elapsed,
            'validation_elapsed_s': final['elapsed_s'], 'input_validation_elapsed_s': initial['elapsed_s'],
            'elapsed_s': perf_counter()-begin,
            'query_statistics': {k: validity.statistics()[k]-v for k,v in before.items()},
            'continuous_collision_proven': False, 'usage': 'RESEARCH_ONLY'}
