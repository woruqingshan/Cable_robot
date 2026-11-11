"""T3: unchanged T2 IK goals, deterministic G4 start, actual semantic FCL paths."""
from copy import deepcopy
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
from time import perf_counter
import numpy as np
from .assembly_semantics import _hash, _write
from .p22_reachability import verify_p22
from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..scene.schema import load_scene
from ..workspace.results import load_results
from ..task.spec import load_task, canonical
from ..task.targets import build_targets, tcp_errors
from ..planning.validity import ConfigurationValidity, assess_task
from ..planning.joint_path import PathError, positive, path_length
from ..planning.rrt_connect import rrt_connect
from ..collision.evaluator import Evaluator
from ..collision.semantic import SemanticConfig
from ..collision.assets import robot_surfaces
from ..viz.joint_path import build_path_figure, export_path_viewer


def settings_checked(settings):
    defaults = {'edge_resolution_rad': .02, 'verification_resolution_rad': .01,
                'step_rad': .25, 'random_seed': 20261104, 'max_iterations': 1500,
                'max_time_s': 20., 'goal_bias': .05, 'rrt_mode': 'on-failure',
                'self_clearance_m': 0., 'environment_clearance_m': 0.}
    if set(settings)-set(defaults): raise PathError('unknown planning setting')
    defaults.update(settings)
    for key in ['edge_resolution_rad', 'verification_resolution_rad', 'step_rad', 'max_time_s']:
        positive(defaults[key], key)
    if defaults['verification_resolution_rad'] > defaults['edge_resolution_rad']:
        raise PathError('verification resolution must be no coarser than search resolution')
    for key in ['random_seed', 'max_iterations']:
        if type(defaults[key]) is not int or defaults[key] < (1 if key == 'max_iterations' else 0):
            raise PathError('invalid '+key)
    for key in ['self_clearance_m', 'environment_clearance_m', 'goal_bias']:
        if type(defaults[key]) not in (int, float) or not np.isfinite(defaults[key]) or defaults[key] < 0:
            raise PathError('invalid '+key)
    if defaults['goal_bias'] > 1: raise PathError('goal_bias must be <=1')
    if defaults['rrt_mode'] not in ['on-failure', 'always']: raise PathError('invalid rrt_mode')
    return defaults


def run_p22_path(root, backend, ik_run, seed_prefix, output, *, allow_candidate=False, settings=None):
    root = Path(root).resolve(); out = Path(output).absolute()
    ik_run = Path(ik_run).resolve(); seed_prefix = Path(seed_prefix).resolve()
    if os.path.lexists(out): raise PathError('refusing existing T3 output path')
    if not allow_candidate: raise PathError('explicit allow_candidate required; RESEARCH_ONLY')
    params = settings_checked(settings or {})
    backend = Path(backend).resolve(strict=True)
    model = load_model(root/'config/workspace.yaml'); fk = ForwardKinematics(model)
    source_verification = verify_p22(ik_run, model)
    spec = load_task(ik_run/'task_config.yaml'); scene = load_scene(root/'config/scenes/idc_prototype_v0.yaml')
    targets = build_targets(spec, scene)
    if spec.data['target']['port_id'] != 'P22': raise PathError('only unchanged P22 task is supported')
    source_ik = json.loads((ik_run/'ik_results.json').read_text())
    solutions = source_ik['unique_task_solutions']
    if len(solutions) != 2: raise PathError('expected the two existing T2 solutions, not newly generated IK')
    goals = []
    for s in solutions:
        q = model.validate_q(s['q']); errors = tcp_errors(fk(q)@spec.T_ee_tcp, targets)
        check = assess_task(errors, targets)
        if not check['task_pose_feasible']: raise PathError('source IK goal no longer satisfies unchanged task')
        goals.append({'solution_id': s['solution_id'], 'q': q.tolist(), 'task_check': check})
    arrays, _ = load_results(seed_prefix, model=model)
    input_hashes = {str(seed_prefix)+ext: _hash(str(seed_prefix)+ext) for ext in ['.npz', '.json']}
    input_hashes.update({str(ik_run/name): _hash(ik_run/name) for name in ['manifest.json', 'ik_results.json', 'task_config.yaml']})
    semantic = SemanticConfig.load(root/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf', model, allow_candidate=True)
    out.mkdir(parents=True); (out/'figures').mkdir(); (out/'paths').mkdir()
    _write(out/'started.json', {'status': 'STARTED', 'usage': 'RESEARCH_ONLY'})
    _write(out/'settings.json', params); _write(out/'goals.json', goals); _write(out/'targets.json', targets.to_dict())
    _write(out/'source_ik_verification.json', source_verification)
    _write(out/'model_metadata.json', model.metadata())
    with (out/'source_robot.urdf').open('x') as f: f.write(model.urdf_xml)
    with (out/'scene_snapshot.json').open('x') as f: f.write(scene.to_json())
    srdf_source = root/'config/collision/semantic/standard_moveit_candidate_v0'
    for name in ['standard_moveit_candidate_v0.srdf', 'semantic_manifest.json', 'candidate_source.json']:
        with (out/name).open('xb') as f: f.write((srdf_source/name).read_bytes())
    with (out/'task_config.yaml').open('x') as f: f.write((ik_run/'task_config.yaml').read_text())
    results = []; selection = []; start = None; source_index = None
    with Evaluator(model, scene, None, backend, semantic=semantic, gripper=spec.data['collision']['gripper_rad'],
                   clearance={'self_m': params['self_clearance_m'], 'environment_m': params['environment_clearance_m']}, timeout=60) as evaluator:
        checker = ConfigurationValidity(evaluator)
        # Deterministic and outcome-independent selection; never choose by path success.
        for index, q in enumerate(arrays['q']):
            state = checker.check(q); selection.append(state)
            if state['valid']:
                start = q.copy(); source_index = index; break
        _write(out/'start_selection.json', {'rule': 'first valid G4 row in original ascending zero-based index order',
            'input_prefix': str(seed_prefix), 'tested': selection, 'original_sample_index': source_index,
            'not_a_physical_home': True})
        endpoint_checks = [checker.check(g['q'], fresh=True) for g in goals]
        _write(out/'goal_checks.json', endpoint_checks)
        if start is not None:
            _write(out/'start.json', {'original_sample_index': source_index, 'q': start.tolist(),
                'T_base_ee': fk(start).tolist(), 'T_base_tcp': (fk(start)@spec.T_ee_tcp).tolist(),
                'collision_check': selection[-1], 'not_a_physical_home': True, 'usage': 'RESEARCH_ONLY'})
            for goal, endpoint in zip(goals, endpoint_checks):
                for method in ['linear', 'rrt_connect']:
                    label = 'q'+str(goal['solution_id'])+'_'+method
                    previous = next((r for r in results if r['solution_id'] == goal['solution_id'] and r['method'] == 'linear'), None)
                    if method == 'rrt_connect' and params['rrt_mode'] == 'on-failure' and previous['success']:
                        results.append({'label': label, 'solution_id': goal['solution_id'], 'method': method,
                                        'success': None, 'status': 'NOT_REQUIRED_LINEAR_PASSED'}); continue
                    before = checker.statistics(); checker.cache.clear(); begin = perf_counter()
                    search = None; validation = None; waypoints = None
                    if not endpoint['valid']:
                        status = 'INVALID_GOAL'
                    else:
                        if method == 'linear':
                            waypoints = [start.tolist(), goal['q']]
                        else:
                            search = rrt_connect(start, goal['q'], model.limits, checker,
                                resolution_rad=params['edge_resolution_rad'], step_rad=params['step_rad'],
                                seed=params['random_seed'], max_iterations=params['max_iterations'],
                                max_time_s=params['max_time_s'], goal_bias=params['goal_bias'])
                            waypoints = search['waypoints']
                        if waypoints is not None:
                            # Fresh exhaustive fine recheck, NOT cached tree-edge approval.
                            validation = checker.path(waypoints, params['verification_resolution_rad'], fresh=True, stop_first=False)
                            status = 'VALID_DISCRETE_PATH' if validation['valid'] else 'DISCRETE_PATH_REJECTED'
                            if validation['states'][0]['q'] != start.tolist() or validation['states'][-1]['q'] != goal['q']:
                                raise PathError('path changed the original problem endpoints')
                            for s in validation['states']:
                                s['task_errors'] = tcp_errors(np.asarray(s['T_base_to_ee'])@spec.T_ee_tcp, targets) if 'T_base_to_ee' in s else None
                        else: status = search['status']
                    planning_elapsed = perf_counter()-begin
                    result = {'label': label, 'solution_id': goal['solution_id'], 'method': method,
                        'success': status == 'VALID_DISCRETE_PATH', 'status': status, 'search': search,
                        'waypoints': waypoints, 'joint_path_length_rad': path_length(waypoints) if waypoints is not None else None,
                        'waypoint_count': len(waypoints) if waypoints is not None else 0,
                        'planning_and_validation_elapsed_s': planning_elapsed, 'validation': validation,
                        'query_statistics': {k: checker.statistics()[k]-v for k, v in before.items()},
                        'usage': 'RESEARCH_ONLY', 'continuous_collision_proven': False, 'hardware_validated': False,
                        'only_endpoint_task_constrained': True, 'time_parameterized': False}
                    # Replay exactly the freshly checked states, never interpolation of poses.
                    if validation and all('link_transforms_base' in s for s in validation['states']):
                        first = checker.check(start, fresh=True)
                        native = deepcopy(evaluator.last_native); native.update(q=start.tolist(), sample_index=label)
                        start_check = assess_task(tcp_errors(np.asarray(native['T_base_to_ee'])@spec.T_ee_tcp, targets), targets)
                        fig, local = build_path_figure(scene, robot_surfaces(model, evaluator.ready, native), native,
                            targets, start_check, validation['states'], label+' — '+status)
                        viewer = 'figures/'+label+'.html'
                        export_path_viewer(fig, local, validation['states'], targets, out/viewer, label=label+' — '+status)
                        result.update(viewer=viewer, viewer_native_query_id=first['query_id'])
                    if validation:
                        states = validation['states']; q = np.asarray([s['q'] for s in states])
                        tcp = np.asarray([np.asarray(s['T_base_to_ee'])@spec.T_ee_tcp if 'T_base_to_ee' in s else np.full((4,4), np.nan) for s in states])
                        distances = {}
                        for kind in ['self', 'environment']:
                            distances[kind] = np.array([((s.get('semantic_clearance') or {}).get('native', {}).get(kind, {}).get('separation_m')) for s in states], dtype=float)
                        npz_file = 'paths/'+label+'.npz'
                        with (out/npz_file).open('xb') as f:
                            np.savez_compressed(f, q=q, T_base_tcp=tcp, tcp_xyz=tcp[:, :3, 3],
                                valid=np.array([s['valid'] for s in states]), query_id=np.array([s['query_id'] for s in states]),
                                self_clearance_m=distances['self'], environment_clearance_m=distances['environment'],
                                waypoints=np.asarray(waypoints), goal_solution_id=goal['solution_id'], start_sample_index=source_index)
                        result['npz'] = npz_file
                    _write(out/('paths/'+label+'.json'), result); results.append(result)
                    print(json.dumps({'stage': 'T3', 'label': label, 'status': status, 'length_rad': result['joint_path_length_rad'],
                                      'samples': 0 if validation is None else validation['checked_samples'], 'elapsed_s': planning_elapsed}), flush=True)
        _write(out/'collision_queries.json', {'identity': evaluator.identity, 'records': checker.queries, 'statistics': checker.statistics()})
        identity = evaluator.identity; total_stats = checker.statistics()
    if not evaluator.client.cleanup['exit_normal']: raise PathError('backend did not exit normally')
    comparison = []
    for r in results:
        v = r.get('validation') or {}
        comparison.append({k: r.get(k) for k in ['label', 'solution_id', 'method', 'success', 'status', 'joint_path_length_rad',
            'waypoint_count', 'planning_and_validation_elapsed_s', 'query_statistics', 'viewer', 'npz']} |
            {'checked_path_samples': v.get('checked_samples'), 'minimum_clearance': v.get('clearance'), 'first_failure': v.get('first_failure'),
             'search_elapsed_s': (r.get('search') or {}).get('elapsed_s'), 'verification_elapsed_s': v.get('elapsed_s')})
    summary = {'status': 'COMPLETED' if start is not None else 'NO_VALID_G4_START', 'stage': 'T3', 'usage': 'RESEARCH_ONLY',
        'start_sample_index': source_index, 'q_start': None if start is None else start.tolist(), 'goals': goals,
        'comparison': comparison, 'settings': params, 'identity': identity, 'backend_cleanup': evaluator.client.cleanup,
        'total_query_statistics': total_stats, 'any_valid_discrete_path': any(r['success'] is True for r in results),
        'continuous_collision_proven': False, 'hardware_validated': False, 'planning_approved': False,
        'tool_cable_collision_geometry_modeled': False, 'time_parameterized': False, 'insertion_planned': False,
        'normal_physical_start_verified': False, 'planning_failure_proves_unreachable': False}
    _write(out/'summary.json', summary)
    for name, digest in input_hashes.items():
        if _hash(name) != digest: raise PathError('historical input changed: '+name)
    sources = {str(p.relative_to(root)): _hash(p) for folder in ['src', 'scripts', 'cpp', 'config']
               for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    _write(out/'manifest.json', {'status': 'COMPLETED', 'stage': 'T3', 'usage': 'RESEARCH_ONLY',
        'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
        'input_hashes': input_hashes, 'ik_run': str(ik_run), 'seed_prefix': str(seed_prefix), 'source_sha256': sources,
        'settings_sha256': sha256(canonical(params).encode()).hexdigest(), 'identity': identity,
        'task_sha256': spec.sha256, 'tcp_sha256': spec.tcp_sha256, 'scene_sha256': scene.canonical_scene_sha256,
        'files': {str(p.relative_to(out)): _hash(p) for p in out.rglob('*') if p.is_file()}})
    return summary


def verify_p22_path(output, model):
    """Independent artifact/FK/endpoint/sampling/statistics checks; no new FCL run."""
    out = Path(output).resolve(); manifest = json.loads((out/'manifest.json').read_text())
    if manifest.get('stage') != 'T3' or manifest.get('status') != 'COMPLETED': raise PathError('not a completed T3 run')
    for name, digest in manifest['files'].items():
        p = (out/name).resolve()
        if not p.is_relative_to(out) or _hash(p) != digest: raise PathError('artifact changed: '+name)
    for name, digest in manifest['input_hashes'].items():
        if _hash(name) != digest: raise PathError('historical input changed: '+name)
    summary = json.loads((out/'summary.json').read_text()); params = settings_checked(summary['settings'])
    if sha256(canonical(params).encode()).hexdigest() != manifest['settings_sha256']: raise PathError('settings identity differs')
    spec = load_task(out/'task_config.yaml'); fk = ForwardKinematics(model)
    if spec.sha256 != manifest['task_sha256'] or spec.tcp_sha256 != manifest['tcp_sha256'] or model.metadata()['expanded_urdf_sha256'] != manifest['identity']['urdf_sha256']:
        raise PathError('model/task identity differs')
    from ..scene.schema import parse_scene
    if parse_scene((out/'scene_snapshot.json').read_text()).canonical_scene_sha256 != manifest['scene_sha256']:
        raise PathError('scene snapshot identity differs')
    arrays, _ = load_results(manifest['seed_prefix'], model=model)
    if summary['q_start'] is None: return {'status': 'PASS', 'paths_checked': 0, 'no_valid_start': True}
    start = np.asarray(summary['q_start']); original = arrays['q'][summary['start_sample_index']]
    if not np.array_equal(start, original): raise PathError('start/sample index changed')
    selection = json.loads((out/'start_selection.json').read_text())
    if len(selection['tested']) != summary['start_sample_index']+1 or any(r['valid'] for r in selection['tested'][:-1]) or not selection['tested'][-1]['valid']:
        raise PathError('deterministic start-selection rule differs')
    for i, record in enumerate(selection['tested']):
        if record['q'] != arrays['q'][i].tolist(): raise PathError('start selection lost original G4 indices')
    ledger = json.loads((out/'collision_queries.json').read_text())
    if ledger['statistics']['native_collision_queries'] != len(ledger['records']): raise PathError('query count differs')
    goals = json.loads((Path(manifest['ik_run'])/'ik_results.json').read_text())['unique_task_solutions']
    checked = 0; states_checked = 0
    from ..planning.joint_path import sample_path
    for row in summary['comparison']:
        if not row.get('npz'): continue
        data = json.loads((out/('paths/'+row['label']+'.json')).read_text())
        with np.load(out/row['npz'], allow_pickle=False) as a:
            q = a['q']; goal = next(g['q'] for g in goals if g['solution_id'] == row['solution_id'])
            expected, provenance = sample_path(data['waypoints'], params['verification_resolution_rad'])
            if not np.array_equal(q, expected) or not np.array_equal(q[0], start) or not np.array_equal(q[-1], goal): raise PathError('path/endpoints changed')
            if provenance != data['validation']['provenance']: raise PathError('sample provenance differs')
            if len(q)>1 and np.max(np.abs(np.diff(q, axis=0))) > params['verification_resolution_rad']+1e-12: raise PathError('sampling resolution exceeded')
            if abs(path_length(data['waypoints'])-row['joint_path_length_rad']) > 1e-10: raise PathError('path length differs')
            for i, angles in enumerate(q):
                model.validate_q(angles)
                if not np.allclose(fk(angles)@spec.T_ee_tcp, a['T_base_tcp'][i], atol=1e-10, rtol=0): raise PathError('native/KDL TCP path differs')
                record = data['validation']['states'][i]
                if record['q'] != angles.tolist() or record['valid'] != bool(a['valid'][i]): raise PathError('saved path record differs')
                query = ledger['records'][record['query_id']]
                if record['q'] != query['q'] or record['valid'] != query['valid']: raise PathError('path/native query provenance differs')
                if not np.allclose(np.asarray(record['T_base_to_ee'])@spec.T_ee_tcp, a['T_base_tcp'][i], atol=1e-12, rtol=0): raise PathError('saved native TCP differs')
                for kind in ['self', 'environment']:
                    value = record['semantic_clearance']['native'][kind]['separation_m']
                    if value is not None and value != a[kind+'_clearance_m'][i]: raise PathError('saved clearance differs')
                if record['valid'] and (record['reasons'] or record['semantic_self_collision'] is not False or record['environment_collision'] is not False): raise PathError('optimistic semantic pass')
            if row['success'] != bool(a['valid'].all()) or not data['validation']['complete']: raise PathError('path success differs')
            states_checked += len(q); checked += 1
    return {'status': 'PASS', 'usage': 'RESEARCH_ONLY', 'paths_checked': checked, 'states_checked': states_checked,
            'all_input_hashes_unchanged': True, 'native_KDL_TCP_agree': True, 'new_collision_queries': 0}
