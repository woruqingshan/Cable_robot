"""T5.1 explicit task orchestration, versioned records and new-only artifacts.

Algorithms live in existing planning/robot/collision modules. The runner owns one
candidate evaluator; generic functions borrow it. Strict screening owns separate
short-lived evaluators and cannot alter the candidate ACM.
"""
from copy import deepcopy
from hashlib import sha256
import csv
import json
import os
from pathlib import Path
import subprocess
from time import perf_counter
from typing import Any, Iterable
import numpy as np
from .assembly_semantics import _hash, _write
from .task_context import TaskContext, TaskSystemError, require_resolved, select_common_start
from .task_reachability import run_task_reachability
from .task_path_planning import run_task_path
from .task_approach import run_task_approach
from .p22_path_planning import settings_checked
from ..planning.cartesian_approach import approach_settings
from ..planning.validity import ConfigurationValidity, screen_solutions
from ..task.spec import TaskSpec, TaskError, canonical, load_task, parse_task
from ..task.template import tasks_for_ports
from ..task.targets import build_targets
from ..robot.model import load_model
from ..robot.fk import ForwardKinematics
from ..scene.schema import load_scene
from ..workspace.results import load_results
from ..collision.evaluator import Evaluator
from ..collision.semantic import SemanticConfig

SCHEMA_VERSION = 1


def checked_settings(planning: dict | None, approach: dict | None,
                     spec: TaskSpec) -> tuple[dict, dict]:
    """Validate policy before any process/output is created; never repair inputs."""
    inherited = {'random_seed': spec.data['solver']['random_seed'],
                 **{k: spec.data['collision'][k] for k in ('self_clearance_m', 'environment_clearance_m')}}
    path = settings_checked({'rrt_mode': 'always', **inherited, **(planning or {})})
    cart = approach_settings({**inherited, **(approach or {})})
    if path['rrt_mode'] != 'always':
        raise TaskError('Benchmark requires explicit Linear AND RRT trials')
    if path['verification_resolution_rad'] != .01:
        raise TaskError('T5.1 final joint path resolution must be 0.01 rad')
    if cart['end_distance_m'] != .01:
        raise TaskError('T5.1 free-space approach ends at 10 mm')
    for values in (path, cart):
        if values['random_seed'] != spec.data['solver']['random_seed']:
            raise TaskError('explicit IK/path/approach random seeds must agree')
        for key in ['self_clearance_m', 'environment_clearance_m']:
            if values[key] != spec.data['collision'][key]:
                raise TaskError('task and planning clearance mismatch')
    return path, cart


def run_task(context: TaskContext, workspace_arrays: dict[str, Any], start_q,
             *, planning: dict | None = None, approach: dict | None = None) -> dict:
    """Evaluate all unique collision-valid goals with BOTH unchanged planners.

    One approach is run per reached goal (not once per successful planner).
    Normal budget/geometry failures are data; identity/backend failures propagate.
    """
    context.validate()
    path_settings, cart_settings = checked_settings(planning, approach, context.targets.spec)
    start = context.model.validate_q(start_q)
    begin = perf_counter(); before = context.checker.statistics()
    start_check = context.checker.check(start, fresh=True); require_resolved([start_check])
    result = {'schema_version': SCHEMA_VERSION, 'usage': 'RESEARCH_ONLY',
              'port_id': context.targets.spec.data['target']['port_id'],
              'task_id': context.targets.spec.data['id'], 'scene_id': context.scene.scene_id,
              'task_sha256': context.targets.spec.sha256, 'tcp_sha256': context.targets.spec.tcp_sha256,
              'targets': context.targets.to_dict(), 'start_q': start.tolist(),
              'identity': deepcopy(context.checker.evaluator.identity), 'start_check': start_check,
              'reachability': {'status': 'NOT_RUN'}, 'goals': [],
              'planning_settings': path_settings, 'approach_settings': cart_settings,
              'hardware_validated': False, 'continuous_collision_proven': False}
    if not start_check['valid']:
        result['status'] = 'INVALID_START'
    else:
        reach = run_task_reachability(context, workspace_arrays)
        result['reachability'] = reach
        for solution in reach['screened']:
            goal = {'solution_id': solution['solution_id'], 'q': solution['q'],
                    'candidate': solution['candidate'], 'paths': [], 'approach': {'status': 'NOT_RUN'}}
            result['goals'].append(goal)
            if not goal['candidate']:
                goal['status'] = 'COLLISION_REJECTED'
                continue
            for method in ('linear', 'rrt_connect'):
                trial = run_task_path(context, start, solution['q'], method=method, settings=path_settings)
                goal['paths'].append(trial)
            if any(t['success'] for t in goal['paths']):
                cart = run_task_approach(context, solution['q'], settings=cart_settings)
                goal['approach'] = cart
                goal['status'] = 'SUCCESS_RESEARCH_ONLY' if cart['success'] else 'APPROACH_FAILED'
            else:
                # Prefer concrete failed final validation over finite search budget.
                statuses = [t['status'] for t in goal['paths']]
                goal['status'] = next((s for s in ('INVALID_START', 'INVALID_GOAL',
                    'PATH_VALIDATION_FAILED', 'PLANNER_TIMEOUT', 'PLANNER_BUDGET_EXHAUSTED') if s in statuses), 'ERROR')
        if reach['status'] != 'TASK_CANDIDATES':
            result['status'] = reach['status']
        elif any(g['status'] == 'SUCCESS_RESEARCH_ONLY' for g in result['goals']):
            result['status'] = 'SUCCESS_RESEARCH_ONLY'
        elif any(g['status'] == 'APPROACH_FAILED' for g in result['goals']):
            result['status'] = 'APPROACH_FAILED'
        else:
            result['status'] = next(g['status'] for g in result['goals'] if g['candidate'])
    result.update(elapsed_s=perf_counter()-begin,
        query_statistics={k: context.checker.statistics()[k]-v for k, v in before.items()})
    return result


def run_tasks(contexts: Iterable[TaskContext], workspace_arrays: dict[str, Any], start_q,
              *, planning: dict | None = None, approach: dict | None = None):
    """Yield each port, including failures; stop querying after a shared system fault."""
    blocked = None
    for context in contexts:
        port = context.targets.spec.data['target']['port_id']
        if blocked is not None:
            yield {'port_id': port, 'status': 'BLOCKED', 'error': blocked,
                   'reachability': {'status': 'NOT_RUN'}, 'goals': [], 'elapsed_s': 0.}
            continue
        try:
            yield run_task(context, workspace_arrays, start_q, planning=planning, approach=approach)
        except TaskSystemError as exc:
            blocked = str(exc)
            yield {'port_id': port, 'status': 'ERROR', 'error': blocked,
                   'reachability': {'status': 'NOT_RUN'}, 'goals': [], 'elapsed_s': None}


def _minimum(records: list[dict], kind: str) -> float | None:
    values = [(s.get('semantic_clearance') or {}).get('native', {}).get(kind, {}) for s in records]
    distances = [s['separation_m'] for s in values if s.get('available') and s.get('separation_m') is not None]
    return min(distances) if distances else None


def summary_row(record: dict) -> dict:
    """Aggregate without mixing attempt, solution, planner and port denominators."""
    reach = record['reachability']; ik = reach.get('ik')
    goals = record['goals']; trials = [t for g in goals for t in g['paths']]
    approaches = [g['approach'] for g in goals if g['approach']['status'] != 'NOT_RUN']
    states = [s for t in trials for s in (t.get('validation') or {}).get('states', [])]
    states += [s for a in approaches for s in a['samples']]
    row = {'port_id': record['port_id'], 'status': record['status'],
           'ik_seeds': None if ik is None else ik['seed_count'],
           'ik_attempts': None if ik is None else ik['optimization_count'],
           'task_feasible_attempts': None if ik is None else sum(a['task_check']['task_pose_feasible'] for a in ik['attempts']),
           'ik_solutions': None if ik is None else len(ik['unique_task_solutions']),
           'collision_candidates': None if ik is None else reach['candidate_count'],
           'approach_trials': len(approaches), 'approach_successes': sum(a['success'] for a in approaches),
           'elapsed_s': record['elapsed_s'], 'native_queries': record.get('query_statistics', {}).get('native_collision_queries'),
           'minimum_self_clearance_m': _minimum(states, 'self'),
           'minimum_environment_clearance_m': _minimum(states, 'environment'),
           'failure_reason': None if record['status'] == 'SUCCESS_RESEARCH_ONLY' else record['status']}
    for method in ('linear', 'rrt_connect'):
        rows = [t for t in trials if t['method'] == method]
        row[method+'_trials'] = len(rows); row[method+'_successes'] = sum(t['success'] for t in rows)
        row[method+'_status'] = 'NOT_RUN' if not rows else 'ALL_PASS' if all(t['success'] for t in rows) else 'PARTIAL' if any(t['success'] for t in rows) else 'ALL_FAILED'
        row[method+'_search_s'] = sum(t['search_elapsed_s'] for t in rows)
        row[method+'_validation_s'] = sum(t['validation_elapsed_s'] or 0 for t in rows)
        row[method+'_path_lengths_rad'] = [t['path_length_rad'] for t in rows]
    row['approach_status'] = 'NOT_RUN' if not approaches else 'ALL_PASS' if all(a['success'] for a in approaches) else 'PARTIAL' if any(a['success'] for a in approaches) else 'ALL_FAILED'
    best = None if ik is None else ik['best_attempt']
    row['best_ik_task_status'] = None if best is None else best['task_check']['status']
    row['best_ik_rejection_reasons'] = [] if best is None else best['task_check']['rejection_reasons']
    for key in ('position_mm', 'insertion_axis_deg', 'key_axis_deg'):
        row['best_ik_'+key] = None if best is None else best['errors'][key]
    for key in ('max_position_error_mm', 'max_axis_error_deg', 'max_key_error_deg'):
        values = [a[key] for a in approaches if a[key] is not None]
        row[key] = max(values) if values else None
    return row


def benchmark_summary(records: list[dict]) -> dict:
    ports = [r['port_id'] for r in records]
    if len(set(ports)) != len(ports): raise TaskError('duplicate benchmark port records')
    rows = [summary_row(r) for r in records]
    reasons = {}
    for row in rows:
        if row['failure_reason'] is not None:
            reasons[row['failure_reason']] = reasons.get(row['failure_reason'], 0)+1
    return {'schema_version': SCHEMA_VERSION, 'usage': 'RESEARCH_ONLY', 'rows': rows,
            'port_denominator': len(rows), 'successful_ports': sum(r['status'] == 'SUCCESS_RESEARCH_ONLY' for r in rows),
            'failure_reasons_by_port': reasons, 'single_scene_single_start_single_seed': True,
            'statistical_generalization_claimed': False, 'continuous_collision_proven': False, 'hardware_validated': False}


def save_trajectory(path: Path, states: list[dict], spec: TaskSpec) -> None:
    """Complete precision, no object arrays; empty sequences remain shaped arrays."""
    tcp = np.array([np.asarray(s['T_base_to_ee']) @ spec.T_ee_tcp for s in states]).reshape((-1, 4, 4))
    with path.open('xb') as f:
        np.savez_compressed(f, q=np.array([s['q'] for s in states]).reshape((-1, 5)),
            T_base_tcp=tcp, valid=np.array([s['valid'] for s in states], dtype=bool),
            query_id=np.array([s['query_id'] for s in states], dtype=np.int64),
            **{kind+'_clearance_m': np.array([(s.get('semantic_clearance') or {}).get('native', {}).get(kind, {}).get('separation_m')
                for s in states], dtype=float) for kind in ('self', 'environment')})


def _viewers(context: TaskContext, record: dict, out: Path) -> list[str]:
    """Opt-in complete-mesh replay of authoritative checked states only."""
    from ..collision.assets import robot_surfaces
    from ..viz.joint_path import build_path_figure, export_path_viewer
    from ..viz.approach import export_approach_viewer
    from ..planning.validity import assess_task
    from ..task.targets import tcp_errors
    files = []
    reached = any(t['success'] for g in record['goals'] for t in g['paths'])
    if not reached:
        ik = record['reachability'].get('ik')
        if ik is None: return files
        from ..viz.task import build_task_figure
        from ..viz.collision import export_collision_viewer
        q = ik['best_attempt']['q']
        require_resolved([context.checker.check(q, fresh=True)])
        native = deepcopy(context.checker.evaluator.last_native)
        native.update(q=q, sample_index=record['port_id']+'_best_approximation')
        assessment = assess_task(tcp_errors(np.asarray(native['T_base_to_ee']) @ context.targets.spec.T_ee_tcp, context.targets), context.targets)
        figure = build_task_figure(context.scene, robot_surfaces(context.model, context.checker.evaluator.ready, native), native, context.targets, assessment)
        figure.update_layout(title=record['port_id']+' best approximation — NOT an accepted task/path')
        export_collision_viewer(figure, native, out/'best_approximation.html')
        return ['best_approximation.html']
    for goal in record['goals']:
        trial = next((p for p in goal['paths'] if p['success']), None)
        if trial is None: continue
        label = 'goal'+str(goal['solution_id'])
        for role, states in [('path', trial['validation']['states']),
                             ('approach', goal['approach'].get('samples', []))]:
            if not states: continue
            context.checker.check(states[0]['q'], fresh=True)
            native = deepcopy(context.checker.evaluator.last_native)
            native.update(q=states[0]['q'], sample_index=label+'_'+role)
            surfaces = robot_surfaces(context.model, context.checker.evaluator.ready, native)
            task = assess_task(tcp_errors(np.asarray(native['T_base_to_ee']) @ context.targets.spec.T_ee_tcp, context.targets), context.targets)
            name = label+'_'+role+'.html'
            if role == 'approach':
                export_approach_viewer(context.scene, surfaces, native, context.targets, task,
                    goal['approach'], out/name, label=record['port_id']+' '+label+' RESEARCH_ONLY')
            else:
                figure, local = build_path_figure(context.scene, surfaces, native, context.targets,
                    task, states, record['port_id']+' '+label+' RESEARCH_ONLY')
                export_path_viewer(figure, local, states, context.targets, out/name, label=record['port_id']+' '+label)
            files.append(name)
    return files


def run_benchmark(root: Path, backend: Path, scene_path: Path, task_template: Path,
                  seed_prefix: Path, output: Path, *, port_ids: list[str],
                  allow_candidate: bool = False, planning: dict | None = None,
                  approach: dict | None = None, viewer_ports: tuple[str, ...] = (),
                  random_seed: int | None = None, summary_viewer: bool = False,
                  start_sample_index: int | None = None, base_pose_context: Path | None = None) -> dict:
    """Own sessions/output. All supplied files are bound by SHA256; no history mutation."""
    root = Path(root).resolve(); out = Path(output).absolute()
    if os.path.lexists(out): raise TaskError('refusing existing benchmark output')
    if not allow_candidate: raise TaskError('explicit candidate RESEARCH_ONLY opt-in required')
    if type(summary_viewer) is not bool: raise TaskError('summary_viewer must be boolean')
    backend = Path(backend).resolve(strict=True); seed_prefix = Path(seed_prefix).resolve()
    model = load_model(root/'config/workspace.yaml'); scene = load_scene(scene_path); template = load_task(task_template)
    original_template_sha256 = template.sha256
    if random_seed is not None:
        data = template.data
        if random_seed != data['solver']['random_seed']:
            data['id'] += '__seed'+str(random_seed)
        data['solver']['random_seed'] = random_seed
        template = parse_task(data)
    specs = tasks_for_ports(template, scene, port_ids)
    if len(set(viewer_ports)) != len(viewer_ports) or not set(viewer_ports) <= set(port_ids):
        raise TaskError('Viewer ports must be a unique subset of requested ports')
    path_settings, cart_settings = checked_settings(planning, approach, template)
    for spec in specs: checked_settings(path_settings, cart_settings, spec)
    arrays, _ = load_results(seed_prefix, model=model)
    if start_sample_index is not None and (type(start_sample_index) is not int or not 0 <= start_sample_index < len(arrays['q'])):
        raise TaskError('fixed start sample index outside original G4 data')
    context_data = None
    if base_pose_context is not None:
        from .base_pose_context import verify_context
        context_data = json.loads(Path(base_pose_context).read_text())
        verify_context(context_data, scene, model)
    strict = SemanticConfig.load(root/'config/collision/semantic/jetrover_semantic_v0.srdf', model)
    candidate = SemanticConfig.load(root/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf', model, allow_candidate=True)
    inputs = {str(Path(p).resolve()): _hash(p) for p in [task_template, scene_path, str(seed_prefix)+'.npz', str(seed_prefix)+'.json', backend]}
    if base_pose_context is not None: inputs[str(Path(base_pose_context).resolve())] = _hash(base_pose_context)
    sources = {str(p.relative_to(root)): _hash(p) for folder in ('src', 'scripts', 'cpp', 'config')
               for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    experiment_identity = {'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip(),
                           'source_set_sha256': sha256(canonical(sources).encode()).hexdigest()}
    out.mkdir(parents=True, exist_ok=False); (out/'ports').mkdir()
    _write(out/'settings.json', {'planning': path_settings, 'approach': cart_settings, 'ports': port_ids,
        'viewer_ports': list(viewer_ports), 'goal_selection': 'all unique task-feasible collision-valid solutions',
        'template_sha256': template.sha256, 'input_template_sha256': original_template_sha256,
        'random_seed': template.data['solver']['random_seed'], 'seed_prefix': str(seed_prefix),
        **({'fixed_start_sample_index': start_sample_index} if start_sample_index is not None else {})})
    embedded = {}
    if context_data is not None:
        for source, local in [(scene_path, 'source_scene_input.json'), (base_pose_context, 'base_pose_context.json')]:
            (out/local).write_bytes(Path(source).read_bytes())
            embedded[str(Path(source).resolve())] = local
    _write(out/'scene_snapshot.json', scene.to_dict()); _write(out/'model_metadata.json', model.metadata())
    _write(out/'task_template.json', template.data)
    _write(out/'strict_semantic.json', json.loads(strict.payload_json))
    _write(out/'candidate_semantic.json', json.loads(candidate.payload_json))
    begin = perf_counter(); records = []
    with Evaluator(model, scene, None, backend, semantic=candidate, gripper=template.data['collision']['gripper_rad'],
            clearance={'self_m': template.data['collision']['self_clearance_m'],
                       'environment_m': template.data['collision']['environment_clearance_m']}, timeout=60) as evaluator:
        checker = ConfigurationValidity(evaluator)
        contexts = [TaskContext(model, scene, build_targets(spec, scene), checker) for spec in specs]
        for ctx in contexts: ctx.validate()
        if start_sample_index is None:
            start = select_common_start(checker, arrays['q'])
        else:
            q = arrays['q'][start_sample_index]
            state = checker.check(q, fresh=True); require_resolved([state])
            start = {'status': 'VALID_SIMULATION_START' if state['valid'] else 'INVALID_START',
                     'q': q.tolist(), 'original_sample_index': start_sample_index, 'tested': [state],
                     'not_a_physical_home': True, 'rule': 'fixed original G4 row; no replacement'}
        if start['q'] is not None:
            fk = ForwardKinematics(model)
            start.update(T_base_ee=fk(start['q']).tolist(),
                         T_base_tcp=(fk(start['q']) @ template.T_ee_tcp).tolist(), identity=evaluator.identity)
        _write(out/'common_start.json', start)
        if start['status'] != 'VALID_SIMULATION_START':
            stream = ({'port_id': p, 'status': 'INVALID_START', 'reachability': {'status': 'NOT_RUN'},
                       'goals': [], 'elapsed_s': 0.} for p in port_ids)
        else:
            stream = run_tasks(contexts, arrays, start['q'], planning=path_settings, approach=cart_settings)
        for context, record in zip(contexts, stream):
            port_out = out/'ports'/record['port_id']; port_out.mkdir()
            _write(port_out/'task_spec.json', context.targets.spec.data)
            _write(port_out/'targets.json', context.targets.to_dict())
            ik = record['reachability'].get('ik')
            if ik is not None:
                record['strict_screening'] = screen_solutions(model, scene, context.targets,
                    ik['unique_task_solutions'], strict, backend)
                for row in record['strict_screening']['results']:
                    if row['task_check']['collision_status'] in ('ERROR', 'UNVERIFIED'):
                        raise TaskSystemError('strict control query unresolved')
            else: record['strict_screening'] = {'status': 'NOT_RUN'}
            for goal in record['goals']:
                for trial in goal['paths']:
                    if trial['validation'] is not None:
                        name = 'goal'+str(goal['solution_id'])+'_'+trial['method']+'.npz'
                        save_trajectory(port_out/name, trial['validation']['states'], context.targets.spec)
                        trial['trajectory_file'] = name
                if goal['approach']['status'] != 'NOT_RUN':
                    name = 'goal'+str(goal['solution_id'])+'_approach.npz'
                    save_trajectory(port_out/name, goal['approach']['samples'], context.targets.spec)
                    goal['approach']['trajectory_file'] = name
            record['viewers'] = _viewers(context, record, port_out) if record['port_id'] in viewer_ports else []
            record['experiment_identity'] = experiment_identity
            _write(port_out/'result.json', record); records.append(record)
            print(json.dumps({'port': record['port_id'], **summary_row(record)}), flush=True)
        _write(out/'collision_queries.json', {'identity': evaluator.identity, 'statistics': checker.statistics(), 'records': checker.queries})
        identity = evaluator.identity
    if not evaluator.client.cleanup['exit_normal']: raise TaskSystemError('candidate Native abnormal shutdown')
    summary = benchmark_summary(records)
    summary.update(elapsed_s=perf_counter()-begin, identity=identity, backend_cleanup=evaluator.client.cleanup)
    _write(out/'summary.json', summary)
    if summary_viewer:
        from ..viz.task_benchmark import export_benchmark_summary
        export_benchmark_summary(summary, out/'overview.html')
    with (out/'summary.csv').open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(summary['rows'][0])); writer.writeheader()
        for row in summary['rows']: writer.writerow({k: canonical(v) if isinstance(v, list) else v for k, v in row.items()})
    for p, h in inputs.items():
        if _hash(p) != h: raise TaskSystemError('input changed: '+p)
    for name, h in sources.items():
        if _hash(root/name) != h: raise TaskSystemError('source changed while running: '+name)
    _write(out/'manifest.json', {'schema_version': SCHEMA_VERSION, 'stage': 'T5.1',
        'status': 'COMPLETED_WITH_SYSTEM_ERRORS' if any(r['status'] in ('ERROR', 'BLOCKED') for r in records) else 'COMPLETED',
        'usage': 'RESEARCH_ONLY', **experiment_identity,
        'input_hashes': inputs, 'source_sha256': sources, 'identity': identity,
        **({'embedded_input_files': embedded, 'base_pose_context_file': 'base_pose_context.json'} if embedded else {}),
        'settings_sha256': _hash(out/'settings.json'),
        'tasks': {s.data['target']['port_id']: {'task_sha256': s.sha256, 'tcp_sha256': s.tcp_sha256} for s in specs},
        'strict_semantic_payload_sha256': sha256(strict.payload_json.encode()).hexdigest(),
        'candidate_semantic_payload_sha256': sha256(candidate.payload_json.encode()).hexdigest(),
        'files': {str(p.relative_to(out)): _hash(p) for p in out.rglob('*') if p.is_file()}})
    return summary
