"""Sequential bounded IK plus discrete Cartesian/task/semantic-FCL validation.

No RRT, alternative FK, tool calibration or contact permission. Targets derive
from the selected port; every joint-interpolated sample has its own expected TCP.
"""
from copy import deepcopy
from dataclasses import replace
import math
from time import perf_counter
import numpy as np
from ..geometry.se3 import Pose, matrix_to_quaternion
from ..robot.fk import ForwardKinematics
from ..task.targets import tcp_errors
from .ik import NumericalIK
from .validity import assess_task
from .joint_path import PathError, positive, path_length


def approach_settings(values=None):
    result = {'end_distance_m': .010, 'cartesian_step_m': .010,
              'edge_resolution_rad': .010, 'max_joint_delta_rad': .35,
              'backup_seeds': 2, 'backup_radius_rad': .03, 'random_seed': 20261104,
              'self_clearance_m': 0., 'environment_clearance_m': 0.}
    values = {} if values is None else values
    if set(values)-set(result): raise PathError('unknown Cartesian approach setting')
    result.update(values)
    for key in ['cartesian_step_m', 'edge_resolution_rad', 'max_joint_delta_rad', 'backup_radius_rad']:
        positive(result[key], key)
    for key in ['end_distance_m', 'self_clearance_m', 'environment_clearance_m']:
        v = result[key]
        if type(v) not in (int, float) or not math.isfinite(v) or v < 0: raise PathError('invalid '+key)
    for key in ['backup_seeds', 'random_seed']:
        if type(result[key]) is not int or result[key] < 0: raise PathError('invalid '+key)
    if result['backup_seeds'] > 8: raise PathError('backup_seeds must be <=8')
    return result


def target_at_distance(targets, distance_m):
    """Independent derived waypoint; original task spec/Scene/TCP remain immutable."""
    if type(distance_m) not in (int, float) or not math.isfinite(distance_m) or distance_m < 0:
        raise PathError('finite nonnegative distance to port required')
    t = targets.port_tcp.matrix()
    t[:3, 3] -= distance_m*np.asarray(targets.insertion_axis_base)
    return replace(targets, pre_tcp=Pose(tuple(t[:3, 3]), tuple(matrix_to_quaternion(t[:3, :3]))))


def cartesian_waypoints(targets, end_distance_m=.010, max_step_m=.010):
    """Start exactly at declared Pre-insertion; append exact end for residual step."""
    positive(max_step_m, 'max_step_m')
    start = targets.spec.data['approach']['distance_m']
    target_at_distance(targets, end_distance_m)
    if end_distance_m > start: raise PathError('approach cannot move away from the port')
    count = max(0, math.ceil((start-end_distance_m)/max_step_m-1e-12))
    if start > end_distance_m: count = max(1, count)
    if count > 10000: raise PathError('approach exceeds 10000 Cartesian intervals')
    distances = [start] + [max(end_distance_m, start-i*max_step_m) for i in range(1, count+1)]
    if count: distances[-1] = end_distance_m
    return [{'waypoint_index': i, 'distance_to_port_m': d, 'target': target_at_distance(targets, d)} for i, d in enumerate(distances)]


def check_approach_sample(record, targets, distance_m, fk, *, waypoint_index, fraction):
    """Add task constraints to a copied FCL result; preserve configuration validity.

    Position uses the corresponding Cartesian interpolation fraction, not only
    distance to the infinite line. Cross-track and along-track errors are extra
    diagnostics; neither replaces the original full 3D position tolerance.
    """
    state = deepcopy(record); derived = target_at_distance(targets, distance_m)
    state['configuration_valid'] = bool(record['valid'])
    state['expected_T_base_tcp'] = derived.T_base_pre_tcp.tolist()
    reasons = list(record['reasons']); errors = None; assessment = None
    try:
        actual = fk(record['q'])@targets.spec.T_ee_tcp
        errors = tcp_errors(actual, derived); assessment = assess_task(errors, derived)
        reasons += assessment['rejection_reasons']
        if 'T_base_to_ee' in record:
            native = np.asarray(record['T_base_to_ee'])@targets.spec.T_ee_tcp
            if not np.allclose(native, actual, atol=1e-10, rtol=0): reasons.append('NATIVE_KDL_TCP_MISMATCH')
        else: reasons.append('NATIVE_STATE_UNAVAILABLE')
        delta = actual[:3, 3]-derived.T_base_pre_tcp[:3, 3]
        along = float(delta@np.asarray(targets.insertion_axis_base))
        cross = float(np.linalg.norm(delta-along*np.asarray(targets.insertion_axis_base)))
        actual_distance = float((np.asarray(targets.base_to_port.position_m)-actual[:3, 3])@np.asarray(targets.insertion_axis_base))
    except ValueError as exc:
        actual = None; along = cross = actual_distance = None
        reasons.append('FK_OR_JOINT_LIMIT_ERROR'); state['task_error_detail'] = str(exc)
    state.update(valid=bool(record['valid'] and assessment is not None and assessment['task_pose_feasible'] and not reasons),
                 reasons=list(dict.fromkeys(reasons)), task_errors=errors,
                 actual_T_base_tcp=None if actual is None else actual.tolist())
    state['status'] = 'APPROACH_SAMPLE_PASS' if state['valid'] else 'APPROACH_SAMPLE_REJECTED'
    state['approach_check'] = {'waypoint_index': waypoint_index, 'segment_fraction': fraction,
        'expected_distance_to_port_m': distance_m, 'actual_axial_distance_to_port_m': actual_distance,
        'progress_m': targets.spec.data['approach']['distance_m']-distance_m,
        'cross_track_error_m': cross, 'signed_along_track_error_m': along,
        'task_pose_feasible': False if assessment is None else assessment['task_pose_feasible'],
        'configuration_valid': state['configuration_valid'], 'all_samples_task_constrained': True,
        'usage': 'RESEARCH_ONLY', 'continuous_collision_proven': False}
    return state


def sequential_approach(model, targets, start_q, checker, settings=None, *, ik_factory=NumericalIK, fk=None):
    """Try local continuation, then bounded deterministic backup seeds on IK failure.

    A waypoint is accepted only after ALL samples of its preceding edge pass.
    A collision/task failure stops the sequence; no global detour is attempted.
    """
    params = approach_settings(settings); points = cartesian_waypoints(targets, params['end_distance_m'], params['cartesian_step_m'])
    evaluator = checker.evaluator
    expected_clearance = {'self_m': params['self_clearance_m'], 'environment_m': params['environment_clearance_m']}
    if evaluator.model is not model or evaluator.scene.canonical_scene_sha256 != targets.scene_sha256:
        raise PathError('continuation model/scene differs from collision session')
    if evaluator.clearance != expected_clearance or evaluator.gripper != targets.spec.data['collision']['gripper_rad']:
        raise PathError('declared approach clearance/gripper differs from collision session')
    fk = ForwardKinematics(model) if fk is None else fk
    start = model.validate_q(start_q); previous = start.copy()
    rng = np.random.default_rng(params['random_seed']); begin = perf_counter(); before = checker.statistics()
    waypoints = []; samples = []; failure = None; last_accepted = None
    for point in points:
        index = point['waypoint_index']; target = point['target']; distance = point['distance_to_port_m']
        entry = {'waypoint_index': index, 'distance_to_port_m': distance, 'target': target.to_dict(),
                 'initial_q': previous.tolist(), 'ik_runs': [], 'candidates': [], 'accepted': False}
        solver = ik_factory(model, target); selected = None
        for retry in range(params['backup_seeds']+1):
            seed = previous.copy() if retry == 0 else np.clip(previous+rng.uniform(-params['backup_radius_rad'], params['backup_radius_rad'], size=5), model.limits[:, 0], model.limits[:, 1])
            solve_begin = perf_counter()
            try:
                ik = solver.solve([{'q': seed.tolist(), 'seed_id': retry, 'source': 'previous_success' if retry == 0 else 'bounded_local_backup',
                                    'random_seed': params['random_seed'], 'backup_radius_rad': params['backup_radius_rad']}])
            except (ValueError, RuntimeError) as exc:
                ik = {'unique_task_solutions': [], 'best_attempt': None, 'total_elapsed_s': perf_counter()-solve_begin,
                      'optimization_count': 0, 'attempts': [], 'error': f'{type(exc).__name__}: {exc}', 'initial_q': seed.tolist()}
            entry['ik_runs'].append(ik)
            candidates = []
            # The first source configuration is immutable; solving it is diagnostic.
            solutions = [{'q': start.tolist(), 'source': 'unchanged_T2_start'}] if index == 0 else ik['unique_task_solutions']
            for solution in solutions:
                q = model.validate_q(solution['q']); errors = tcp_errors(fk(q)@targets.spec.T_ee_tcp, target)
                delta = q-previous; maximum = float(np.max(np.abs(delta)))
                assessment = assess_task(errors, target)
                candidate = {'q': q.tolist(), 'errors': errors, 'task_check': assessment,
                    'joint_delta_rad': delta.tolist(), 'max_joint_delta_rad': maximum, 'joint_delta_l2_rad': float(np.linalg.norm(delta)),
                    'branch_jump_suspected': maximum > params['max_joint_delta_rad'], 'retry': retry}
                entry['candidates'].append(candidate)
                if assessment['task_pose_feasible'] and not candidate['branch_jump_suspected']: candidates.append(candidate)
            if candidates:
                selected = min(candidates, key=lambda s: (s['joint_delta_l2_rad'], s['errors']['position_m']))
                break
        if selected is None:
            reason = 'JOINT_CONTINUITY_LIMIT' if any(c['task_check']['task_pose_feasible'] for c in entry['candidates']) else 'IK_SOLVER_ERROR' if all('error' in run for run in entry['ik_runs']) else 'IK_TASK_CONSTRAINT_FAILED'
            failure = {'kind': 'CONTINUITY' if reason.startswith('JOINT') else 'IK', 'waypoint_index': index,
                       'expected_distance_to_port_m': distance, 'expected_T_base_tcp': target.T_base_pre_tcp.tolist(),
                       'reasons': [reason], 'best_approximation': entry['ik_runs'][-1]['best_attempt']}
            entry['status'] = reason; waypoints.append(entry); break
        q = selected['q']; entry['selected'] = selected
        if index == 0:
            records = [checker.check(q, fresh=True)]; fractions = [0.]
        else:
            edge = checker.edge(previous, q, params['edge_resolution_rad'], fresh=True, stop_first=False)
            entry['collision_edge_validation'] = {k: v for k, v in edge.items() if k != 'states'}
            records = edge['states']; fractions = [p['fraction'] for p in edge['provenance']]
        annotated = []
        for record, fraction in zip(records, fractions):
            d = distance if index == 0 else points[index-1]['distance_to_port_m']*(1-fraction)+distance*fraction
            state = check_approach_sample(record, targets, float(d), fk, waypoint_index=index, fraction=fraction)
            # Shared boundary is rechecked fresh, but included only once in replay.
            if index == 0 or fraction != 0. or not state['valid']:
                annotated.append(state)
            if not state['valid'] and failure is None:
                kind = 'QUERY_ERROR' if 'COLLISION_QUERY_ERROR' in state['reasons'] else 'COLLISION_OR_CLEARANCE' if not state['configuration_valid'] else 'FK_VALIDATION' if any(r in state['reasons'] for r in ['NATIVE_KDL_TCP_MISMATCH', 'NATIVE_STATE_UNAVAILABLE', 'FK_OR_JOINT_LIMIT_ERROR']) else 'CARTESIAN_TASK_CONSTRAINT'
                failure = {'kind': kind,
                    'waypoint_index': index, 'sample_index': len(samples)+len(annotated)-1,
                    'expected_distance_to_port_m': d, 'actual_axial_distance_to_port_m': state['approach_check']['actual_axial_distance_to_port_m'],
                    'expected_T_base_tcp': state['expected_T_base_tcp'], 'q': state['q'], 'query_id': state['query_id'],
                    'task_errors': state['task_errors'], 'reasons': state['reasons']}
        entry['endpoint_task_check'] = selected['task_check']; entry['checked_samples'] = len(records)
        entry['endpoint_check'] = {k: annotated[-1][k] for k in ['q','query_id','configuration_valid','valid','status','reasons',
            'task_errors','semantic_self_collision','environment_collision','semantic_clearance','approach_check']}
        entry['task_constrained_edge_valid'] = all(s['valid'] for s in annotated)
        entry['accepted'] = failure is None; entry['status'] = 'ACCEPTED_DISCRETE_APPROACH_WAYPOINT' if failure is None else 'EDGE_REJECTED'
        samples.extend(annotated); waypoints.append(entry)
        if failure is not None: break
        previous = np.asarray(q); last_accepted = distance
    # An edge can be fully inspected diagnostically; only its prefix before first
    # failure counts toward validated progress and is used for approach playback.
    first_bad = next((i for i, s in enumerate(samples) if not s['valid']), None)
    valid_prefix = samples if first_bad is None else samples[:first_bad]
    last_valid = valid_prefix[-1] if valid_prefix else None
    success = failure is None and len(waypoints) == len(points) and all(w['accepted'] for w in waypoints)
    def maximum(key):
        values = [s['task_errors'][key] for s in samples if s['task_errors'] is not None]
        return max(values) if values else None
    clearance = {}
    for kind in ['self', 'environment']:
        values = [(s.get('semantic_clearance') or {}).get('native', {}).get(kind, {}) for s in valid_prefix]
        distances = [v['separation_m'] for v in values if v.get('available') and v.get('separation_m') is not None]
        clearance[kind] = {'minimum_m': min(distances) if distances else None, 'all_valid_prefix_samples_available': len(distances) == len(values)}
    return {'usage': 'RESEARCH_ONLY', 'success': success, 'status': 'VALID_DISCRETE_CARTESIAN_APPROACH' if success else 'APPROACH_STOPPED',
        'settings': params, 'start_q': start.tolist(), 'requested_waypoints': len(points), 'waypoints': waypoints, 'samples': samples,
        'accepted_waypoints': sum(w['accepted'] for w in waypoints), 'last_accepted_waypoint_distance_m': last_accepted,
        'last_valid_sample': last_valid, 'valid_prefix_sample_count': len(valid_prefix), 'first_failed_sample_index': first_bad,
        'last_valid_expected_distance_m': None if last_valid is None else last_valid['approach_check']['expected_distance_to_port_m'],
        'last_valid_actual_axial_distance_m': None if last_valid is None else last_valid['approach_check']['actual_axial_distance_to_port_m'],
        'validated_progress_m': 0. if last_valid is None else last_valid['approach_check']['progress_m'], 'first_failure': failure,
        'max_position_error_mm': maximum('position_mm'), 'max_axis_error_deg': maximum('insertion_axis_deg'), 'max_key_error_deg': maximum('key_axis_deg'),
        'valid_prefix_clearance': clearance, 'elapsed_s': perf_counter()-begin,
        'ik_elapsed_s': sum(run['total_elapsed_s'] for w in waypoints for run in w['ik_runs']),
        'joint_waypoint_length_rad': path_length([start.tolist()]+[w['selected']['q'] for w in waypoints if w['accepted']]),
        'query_statistics': {k: checker.statistics()[k]-v for k, v in before.items()},
        'branch_continuity_is_discrete_only': True, 'continuous_collision_proven': False, 'insertion_success': False,
        'hardware_validated': False, 'planning_approved': False, 'mathematical_unreachability_proven': False}
