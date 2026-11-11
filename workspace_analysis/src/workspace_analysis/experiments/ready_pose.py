"""Deterministic pre-planning Ready Pose selection from identity-checked samples."""
from hashlib import sha256
import json
import numpy as np
from .task_context import require_resolved


def ready_digest(record):
    return sha256(json.dumps(record, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def eligible_ready(q, xyz, limits, port_xyz, goals, start_id, rules):
    """Geometry-only filter; no planner result is accepted as an input."""
    q = np.asarray(q, dtype=float); xyz = np.asarray(xyz, dtype=float)
    limits = np.asarray(limits, dtype=float); port_xyz = np.asarray(port_xyz, dtype=float)
    goals = np.asarray(goals, dtype=float)
    if q.shape != (5,) or xyz.shape != (3,) or limits.shape != (5, 2) or port_xyz.shape != (3,) or goals.ndim != 2 or goals.shape[1:] != (5,):
        raise ValueError('invalid Ready Pose inputs')
    if not all(np.isfinite(v).all() for v in (q, xyz, limits, port_xyz, goals)) or np.any(limits[:, 0] >= limits[:, 1]):
        raise ValueError('finite ordered Ready Pose inputs required')
    if start_id not in ('S1', 'S2'): raise ValueError('Ready Pose must be S1 or S2')
    margin = float(np.min(np.minimum(q-limits[:, 0], limits[:, 1]-q)))
    region = rules[start_id]
    good = region['x_m'][0] <= xyz[0] <= region['x_m'][1]
    good = good and abs(xyz[1]-port_xyz[1]) <= region['y_offset_m'] and abs(xyz[2]-port_xyz[2]) <= region['z_offset_m']
    good = good and margin >= rules['minimum_joint_margin_rad'] and abs(q[0]) <= rules['maximum_abs_joint1_rad']
    if start_id == 'S2':
        pre = port_xyz-np.asarray(rules['insertion_axis_world'])*rules['pre_distance_m']
        distance = float(np.linalg.norm(xyz-pre))
        good = good and region['pre_distance_m'][0] <= distance <= region['pre_distance_m'][1]
        good = good and np.min(np.linalg.norm(goals-q, axis=1)) >= region['minimum_goal_joint_distance_rad']
        good = good and xyz[0] >= .250 and -.150 <= xyz[1] <= .150 and .040 <= xyz[2] <= .400
    return bool(good), margin


def select_ready(model, fk, checker, pools, world_from_base, spec, port_xyz, goals, rules, start_id):
    """First pool with verified candidates; rank by margin, clearance, twist, index.

    pools are already validated by load_results(model=...). Every eligible row
    is queried before sorting. Exhaustion stays explicit rather than relaxing rules.
    """
    tested = []; pool_reports = []; selected = None
    for pool_id, arrays, identity in pools:
        eligible = []
        for index, q in enumerate(arrays['q']):
            xyz = (world_from_base@fk(q)@spec.T_ee_tcp)[:3, 3]
            good, margin = eligible_ready(q, xyz, model.limits, port_xyz, goals, start_id, rules)
            if not good: continue
            state = checker.check(q, fresh=True); require_resolved([state])
            row = {'source_pool': pool_id, 'source_identity': identity, 'original_sample_index': index,
                   'q': q.tolist(), 'world_tcp_xyz_m': xyz.tolist(), 'joint_margin_rad': margin,
                   'validation': state}
            tested.append(row)
            if state['valid']: eligible.append(row)
        pool_reports.append({'source_pool': pool_id, 'rows_screened': len(arrays['q']),
                             'region_eligible': sum(t['source_pool'] == pool_id for t in tested),
                             'verified_valid': len(eligible)})
        if eligible:
            def rank(row):
                clearance = row['validation']['semantic_clearance']['native']['environment']
                distance = clearance['separation_m'] if clearance['available'] and clearance['separation_m'] is not None else -1.
                return (-row['joint_margin_rad'], -distance, abs(row['q'][0]), row['original_sample_index'])
            selected = sorted(eligible, key=rank)[0]
            break
    record = {'start_id': start_id, 'status': 'SELECTED' if selected else 'NO_QUALIFIED_READY_POSE',
              'selection_rules': rules, 'pool_reports': pool_reports, 'tested_candidates': tested,
              'selected': selected, 'not_a_physical_home': True,
              'preparation_motion_to_this_start_verified': False,
              'planner_results_used_for_selection': False, 'usage': 'RESEARCH_ONLY'}
    return {**record, 'sha256': ready_digest(record)}
