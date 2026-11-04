"""Independent readback of T5.1 files, provenance, FK, classifications and totals."""
import csv
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
from .assembly_semantics import _hash
from .task_benchmark import benchmark_summary, checked_settings
from ..task.spec import TaskError, canonical, parse_task
from ..task.targets import build_targets, tcp_errors
from ..scene.schema import parse_scene
from ..robot.fk import ForwardKinematics
from ..planning.ik import build_seeds
from ..planning.validity import assess_task
from ..planning.joint_path import sample_path, interpolate, path_length
from ..planning.cartesian_approach import cartesian_waypoints, check_approach_sample
from ..workspace.results import load_results


def verify_benchmark(output: Path, model, *, input_bindings: dict[str, Path] | None = None) -> dict:
    """Read-only evidence verification; optional explicit relocation of ALL inputs.

    Bindings map original manifest input names to caller-declared current files.
    Every original SHA256 still must match. No legacy path is followed when a
    mapping is supplied, and the saved Manifest/settings remain unchanged.
    """
    out = Path(output).resolve(); manifest = json.loads((out/'manifest.json').read_text())
    if manifest.get('stage') != 'T5.1' or manifest.get('schema_version') != 1 or manifest.get('usage') != 'RESEARCH_ONLY':
        raise TaskError('not a T5.1 research benchmark')
    for name, h in manifest['files'].items():
        p = (out/name).resolve()
        if not p.is_relative_to(out) or _hash(p) != h: raise TaskError('artifact hash/path mismatch: '+name)
    if input_bindings is not None and set(input_bindings) != set(manifest['input_hashes']):
        raise TaskError('complete explicit input bindings required')
    for name, h in manifest['input_hashes'].items():
        current = name if input_bindings is None else input_bindings[name]
        if _hash(current) != h: raise TaskError('input identity mismatch: '+name)
    if model.metadata()['expanded_urdf_sha256'] != manifest['identity']['urdf_sha256']:
        raise TaskError('model identity mismatch')
    if sha256(canonical(manifest['source_sha256']).encode()).hexdigest() != manifest['source_set_sha256']:
        raise TaskError('source set identity mismatch')
    for kind in ('strict', 'candidate'):
        payload = json.loads((out/(kind+'_semantic.json')).read_text())
        if sha256(canonical(payload).encode()).hexdigest() != manifest[kind+'_semantic_payload_sha256']:
            raise TaskError('semantic snapshot identity mismatch')
    scene = parse_scene((out/'scene_snapshot.json').read_text())
    if scene.canonical_scene_sha256 != manifest['identity']['scene_sha256']:
        raise TaskError('scene identity mismatch')
    if _hash(out/'settings.json') != manifest['settings_sha256']: raise TaskError('settings identity mismatch')
    settings = json.loads((out/'settings.json').read_text())
    seed_prefix = settings['seed_prefix']
    if input_bindings is not None:
        names = [settings['seed_prefix']+ext for ext in ('.npz', '.json')]
        if any(name not in input_bindings for name in names): raise TaskError('seed input bindings missing')
        npz, metadata = (Path(input_bindings[name]) for name in names)
        if npz.suffix != '.npz' or metadata != npz.with_suffix('.json'):
            raise TaskError('relocated seed NPZ/JSON must retain a common prefix')
        seed_prefix = npz.with_suffix('')
    arrays, _ = load_results(seed_prefix, model=model)
    ledger = json.loads((out/'collision_queries.json').read_text()); queries = ledger['records']
    if ledger['identity'] != manifest['identity'] or ledger['statistics']['native_collision_queries'] != len(queries):
        raise TaskError('query ledger identity/count mismatch')
    fk = ForwardKinematics(model)
    for i, query in enumerate(queries):
        if query['query_id'] != i: raise TaskError('query index mismatch')
        model.validate_q(query['q'])
        if 'T_base_to_ee' in query and not np.allclose(fk(query['q']), query['T_base_to_ee'], atol=1e-10, rtol=0):
            raise TaskError('native/KDL mismatch')
        if query['valid'] and (query['reasons'] or query['semantic_self_collision'] is not False or query['environment_collision'] is not False):
            raise TaskError('optimistic semantic pass')
    start = json.loads((out/'common_start.json').read_text())
    for i, state in enumerate(start['tested']):
        if state['q'] != arrays['q'][i].tolist() or state != queries[state['query_id']]:
            raise TaskError('start original index/query provenance differs')
    if start['q'] is not None:
        index = start['original_sample_index']
        if len(start['tested']) != index+1 or any(s['valid'] for s in start['tested'][:-1]) or not start['tested'][-1]['valid']:
            raise TaskError('first-valid start rule differs')
        if start['q'] != arrays['q'][index].tolist(): raise TaskError('common start changed')

    def check_states(port_out, name, states, spec):
        with np.load(port_out/name, allow_pickle=False) as data:
            if len(data['q']) != len(states): raise TaskError('trajectory length differs')
            for i, state in enumerate(states):
                source = queries[state['query_id']]
                if source['q'] != state['q'] or source['T_base_to_ee'] != state['T_base_to_ee']:
                    raise TaskError('trajectory native provenance differs')
                if not np.array_equal(data['q'][i], state['q']) or bool(data['valid'][i]) != state['valid']:
                    raise TaskError('NPZ joint/validity differs')
                if data['query_id'][i] != state['query_id']: raise TaskError('NPZ query differs')
                if not np.allclose(data['T_base_tcp'][i], fk(state['q']) @ spec.T_ee_tcp, atol=1e-10, rtol=0):
                    raise TaskError('NPZ TCP differs')
                for kind in ('self', 'environment'):
                    value = source['semantic_clearance']['native'][kind]['separation_m']
                    if value is not None and data[kind+'_clearance_m'][i] != value:
                        raise TaskError('NPZ clearance differs')

    records = []; path_count = 0; approach_count = 0; ik_count = 0
    for port in settings['ports']:
        port_out = out/'ports'/port; spec = parse_task(json.loads((port_out/'task_spec.json').read_text()))
        expected_id = manifest['tasks'][port]
        if spec.data['target']['port_id'] != port or spec.sha256 != expected_id['task_sha256'] or spec.tcp_sha256 != expected_id['tcp_sha256']:
            raise TaskError('port/task identity differs')
        targets = build_targets(spec, scene)
        if targets.to_dict() != json.loads((port_out/'targets.json').read_text()): raise TaskError('derived targets differ')
        record = json.loads((port_out/'result.json').read_text()); records.append(record)
        if record['port_id'] != port: raise TaskError('record port differs')
        if record['experiment_identity'] != {k:manifest[k] for k in ('git_head','source_set_sha256')}:
            raise TaskError('per-port source identity differs')
        reach = record['reachability']; ik = reach.get('ik')
        if ik is None: continue
        if record['identity'] != manifest['identity'] or record['start_q'] != start['q']:
            raise TaskError('port model/start identity differs')
        checked_settings(record['planning_settings'], record['approach_settings'], spec)
        if reach['seeds'] != build_seeds(model, targets, arrays): raise TaskError('seed set differs')
        for attempt in ik['attempts']:
            errors = tcp_errors(fk(attempt['q']) @ spec.T_ee_tcp, targets); assessment = assess_task(errors, targets)
            for key in ('position_m', 'insertion_axis_rad', 'key_axis_rad'):
                if abs(errors[key]-attempt['errors'][key]) > 1e-10: raise TaskError('IK errors differ')
            if assessment['status'] != attempt['task_check']['status']: raise TaskError('IK status differs')
            ik_count += 1
        solutions = {s['solution_id']: s for s in ik['unique_task_solutions']}
        if len(record['goals']) != len(solutions) or len(reach['screened']) != len(solutions): raise TaskError('goal count differs')
        for screened in reach['screened']:
            collision = screened['collision']
            if collision != queries[collision['query_id']] or screened['candidate'] != collision['valid']:
                raise TaskError('screen query differs')
        if reach['candidate_count'] != sum(s['candidate'] for s in reach['screened']): raise TaskError('candidate count differs')
        for goal in record['goals']:
            if goal['q'] != solutions[goal['solution_id']]['q']: raise TaskError('IK goal changed')
            eligible = next(s['candidate'] for s in reach['screened'] if s['solution_id'] == goal['solution_id'])
            if goal['candidate'] != eligible: raise TaskError('goal screening differs')
            if goal['candidate'] and [p['method'] for p in goal['paths']] != ['linear', 'rrt_connect']:
                raise TaskError('both planners must run')
            if not eligible and (goal['paths'] or goal['approach']['status'] != 'NOT_RUN' or goal['status'] != 'COLLISION_REJECTED'):
                raise TaskError('rejected goal ran downstream')
            for trial in goal['paths']:
                validation = trial['validation']
                if validation is None: continue
                expected, provenance = sample_path(trial['waypoints'], trial['settings']['verification_resolution_rad'])
                states = validation['states']
                if not np.array_equal(expected, [s['q'] for s in states]) or provenance != validation['provenance']:
                    raise TaskError('path interpolation differs')
                if not np.array_equal(expected[0], start['q']) or not np.array_equal(expected[-1], goal['q']):
                    raise TaskError('path endpoints differ')
                if len({s['query_id'] for s in states}) != len(states): raise TaskError('final validation reused query')
                for s in states:
                    if s != queries[s['query_id']]: raise TaskError('path query differs')
                if trial['success'] != all(s['valid'] for s in states) or not validation['complete']:
                    raise TaskError('path final approval differs')
                if abs(path_length(trial['waypoints'])-trial['path_length_rad']) > 1e-10: raise TaskError('path length differs')
                check_states(port_out, trial['trajectory_file'], states, spec); path_count += 1
            approach = goal['approach']
            if approach['status'] == 'NOT_RUN': continue
            if not any(t['success'] for t in goal['paths']): raise TaskError('approach without valid reached goal')
            points = cartesian_waypoints(targets, approach['settings']['end_distance_m'], approach['settings']['cartesian_step_m'])
            for state in approach['samples']:
                point = state['approach_check']; wi = point['waypoint_index']; fraction = point['segment_fraction']
                current = approach['waypoints'][wi]['selected']['q']
                if wi == 0:
                    expected_q = goal['q']; distance = points[0]['distance_to_port_m']
                else:
                    edge = interpolate(approach['waypoints'][wi-1]['selected']['q'], current, approach['settings']['edge_resolution_rad'])
                    k = round(fraction*(len(edge)-1))
                    if abs(fraction-k/(len(edge)-1)) > 1e-14: raise TaskError('approach non-grid fraction')
                    expected_q = edge[k]; distance = points[wi-1]['distance_to_port_m']*(1-fraction)+points[wi]['distance_to_port_m']*fraction
                if not np.array_equal(expected_q, state['q']): raise TaskError('approach joint interpolation differs')
                check = check_approach_sample(queries[state['query_id']], targets, float(distance), fk, waypoint_index=wi, fraction=fraction)
                if check['valid'] != state['valid'] or check['reasons'] != state['reasons']: raise TaskError('approach classification differs')
                for key in ('position_m', 'insertion_axis_rad', 'key_axis_rad'):
                    if abs(check['task_errors'][key]-state['task_errors'][key]) > 1e-12: raise TaskError('approach task errors differ')
            success = approach['first_failure'] is None and approach['accepted_waypoints'] == len(points) and all(s['valid'] for s in approach['samples'])
            if success != approach['success']: raise TaskError('approach final approval differs')
            if goal['status'] != ('SUCCESS_RESEARCH_ONLY' if success else 'APPROACH_FAILED'):
                raise TaskError('goal final status differs')
            check_states(port_out, approach['trajectory_file'], approach['samples'], spec); approach_count += 1
        if not solutions:
            expected_status = 'IK_NO_SOLUTION_WITHIN_BUDGET'
        elif not any(g['candidate'] for g in record['goals']):
            expected_status = 'COLLISION_REJECTED'
        elif any(g['approach'].get('success') is True for g in record['goals']):
            expected_status = 'SUCCESS_RESEARCH_ONLY'
        elif any(g['approach']['status'] != 'NOT_RUN' for g in record['goals']):
            expected_status = 'APPROACH_FAILED'
        else:
            expected_status = next(g['status'] for g in record['goals'] if g['candidate'])
        if record['status'] != expected_status: raise TaskError('port final status differs')
    summary = json.loads((out/'summary.json').read_text()); rebuilt = benchmark_summary(records)
    if any(summary[k] != v for k, v in rebuilt.items()): raise TaskError('summary totals differ')
    with (out/'summary.csv').open() as f: csv_rows = list(csv.DictReader(f))
    expected_rows = [{k: '' if v is None else canonical(v) if isinstance(v, list) else str(v) for k, v in row.items()} for row in rebuilt['rows']]
    if csv_rows != expected_rows: raise TaskError('CSV differs')
    return {'status': 'PASS', 'usage': 'RESEARCH_ONLY', 'ports_checked': len(records),
            'IK_attempts_rechecked': ik_count, 'paths_checked': path_count, 'approaches_checked': approach_count,
            'native_ledger_records': len(queries), 'new_collision_queries': 0,
            'verification_scope': 'hashed evidence/FK/thresholds/grid/query provenance/totals; not fresh FCL or hardware safety'}
