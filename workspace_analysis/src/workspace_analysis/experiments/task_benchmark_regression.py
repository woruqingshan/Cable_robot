"""Explicit historical P22 equivalence; no implicit results lookup or tolerances edits."""
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
from .assembly_semantics import _hash
from .p22_reachability import verify_p22
from .p22_path_planning import verify_p22_path
from .p22_approach import verify_approach
from .task_benchmark_validation import verify_benchmark
from ..task.spec import TaskError, load_task, parse_task


def compare_p22(benchmark: Path, ik_run: Path, path_run: Path,
                approach_run: Path, model) -> dict:
    """Check full precision attempts/goals/paths/approach and model identities.

    Same algorithms and same inputs should give exact q here. Explicit thresholds
    allow only the existing saved-FK verification precision, never display rounding.
    Wall time/query IDs are intentionally not equivalence criteria.
    """
    benchmark, ik_run, path_run, approach_run = map(Path, (benchmark, ik_run, path_run, approach_run))
    missing = [str(p/'manifest.json') for p in (benchmark, ik_run, path_run, approach_run) if not (p/'manifest.json').is_file()]
    if missing: return {'status': 'BLOCKED', 'reason': 'MISSING_ARTIFACT', 'missing': missing}
    checks = {'T2': verify_p22(ik_run, model), 'T3': verify_p22_path(path_run, model),
              'T4': verify_approach(approach_run, model), 'T5.1': verify_benchmark(benchmark, model)}
    read = lambda p: json.loads(p.read_text())
    record = read(benchmark/'ports/P22/result.json'); spec = parse_task(read(benchmark/'ports/P22/task_spec.json'))
    if spec.sha256 != load_task(ik_run/'task_config.yaml').sha256: raise TaskError('historical task identity mismatch')
    if record['targets'] != read(ik_run/'targets.json'): raise TaskError('historical targets/TCP differ')
    if record['reachability']['seeds'] != read(ik_run/'seeds.json'): raise TaskError('historical seeds differ')
    old_ik = read(ik_run/'ik_results.json'); new_ik = record['reachability']['ik']
    maxima = {}; thresholds = {'joint_rad':1e-10, 'transform':1e-10, 'error':1e-10, 'clearance_m':1e-10}
    def near(label, actual, expected, kind):
        a, b = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
        if a.shape != b.shape or not np.isfinite(a).all() or not np.isfinite(b).all(): raise TaskError(label+': shape/nonfinite mismatch')
        delta = float(np.max(np.abs(a-b))) if a.size else 0.
        maxima[kind] = max(maxima.get(kind,0.), delta)
        if delta > thresholds[kind]: raise TaskError(label+': numeric difference '+str(delta))
    if len(new_ik['attempts']) != len(old_ik['attempts']): raise TaskError('optimization count differs')
    for a, b in zip(new_ik['attempts'], old_ik['attempts']):
        for key in ['q', 'initial_q']: near('IK '+key, a[key], b[key], 'joint_rad')
        for key in ['stage','seed_id','optimizer_success','optimizer_status','nfev','njev']:
            if a[key] != b[key]: raise TaskError('IK '+key+' differs')
        for key in ['position_m','insertion_axis_rad','key_axis_rad']: near('IK error', a['errors'][key], b['errors'][key], 'error')
    if len(new_ik['unique_task_solutions']) != len(old_ik['unique_task_solutions']): raise TaskError('solution count differs')
    old_paths = read(path_run/'summary.json')
    if record['start_q'] != old_paths['q_start'] or record['planning_settings'] != old_paths['settings']:
        raise TaskError('historical start/planning settings differ')
    if record['approach_settings'] != read(approach_run/'settings.json'): raise TaskError('historical approach settings differ')
    for key in ['urdf_sha256','robot_geometry_sha256','scene_sha256','srdf_sha256','semantic_acm_sha256','semantic_review_sha256','backend_sha256','gripper_state_rad','clearance_settings']:
        if record['identity'][key] != old_paths['identity'][key]: raise TaskError('historical identity differs: '+key)
    paths = 0; samples = 0
    for goal in record['goals']:
        i = goal['solution_id']; old_solution = next(s for s in old_ik['unique_task_solutions'] if s['solution_id'] == i)
        near('unique goal', goal['q'], old_solution['q'], 'joint_rad')
        screen = next(s for s in record['reachability']['screened'] if s['solution_id'] == i)
        old_screen = next(s for s in read(ik_run/'candidate_collision.json')['results'] if s['solution_id'] == i)
        strict = next(s for s in record['strict_screening']['results'] if s['solution_id'] == i)
        old_strict = next(s for s in read(ik_run/'strict_collision.json')['results'] if s['solution_id'] == i)
        for actual, expected in [(screen['collision'], old_screen['collision']), (strict['collision'], old_strict['collision'])]:
            for key in ['semantic_self_collision','environment_collision','semantic_blocking_pairs']:
                if actual[key] != expected[key]: raise TaskError('historical collision differs')
        for trial in goal['paths']:
            old = read(path_run/'paths'/('q'+str(i)+'_'+trial['method']+'.json'))
            if trial['success'] != old['success']: raise TaskError('historical path outcome differs')
            near('path waypoints', trial['waypoints'], old['waypoints'], 'joint_rad')
            near('path samples', [s['q'] for s in trial['validation']['states']], [s['q'] for s in old['validation']['states']], 'joint_rad')
            for a, b in zip(trial['validation']['states'], old['validation']['states']):
                if a['valid'] != b['valid']: raise TaskError('historical sample validity differs')
                for kind in ['self','environment']:
                    near('path separation', a['semantic_clearance']['native'][kind]['separation_m'], b['semantic_clearance']['native'][kind]['separation_m'], 'clearance_m')
            paths += 1
        old = read(approach_run/'branches'/('q'+str(i)+'.json')); new = goal['approach']
        for key in ['success','accepted_waypoints','requested_waypoints','first_failed_sample_index']:
            if new[key] != old[key]: raise TaskError('historical approach '+key+' differs')
        for a,b in zip(new['waypoints'],old['waypoints']):
            near('Cartesian target', a['target']['T_base_pre_tcp'], b['target']['T_base_pre_tcp'], 'transform')
        if len(new['samples']) != len(old['samples']): raise TaskError('approach sample count differs')
        for a,b in zip(new['samples'],old['samples']):
            near('approach q', a['q'], b['q'], 'joint_rad')
            near('approach TCP', a['actual_T_base_tcp'], b['actual_T_base_tcp'], 'transform')
            if a['valid'] != b['valid'] or a['reasons'] != b['reasons']: raise TaskError('approach classification differs')
            for key in ['position_m','insertion_axis_rad','key_axis_rad']: near('approach error', a['task_errors'][key], b['task_errors'][key], 'error')
            for kind in ['self','environment']: near('approach separation', a['semantic_clearance']['native'][kind]['separation_m'], b['semantic_clearance']['native'][kind]['separation_m'], 'clearance_m')
            samples += 1
    return {'status':'PASS', 'usage':'RESEARCH_ONLY', 'verification':checks,
            'input_manifests':{str(p/'manifest.json'):_hash(p/'manifest.json') for p in (benchmark,ik_run,path_run,approach_run)},
            'maximum_absolute_differences':maxima, 'thresholds':thresholds, 'paths_compared':paths,
            'approach_samples_compared':samples, 'ik_attempts_compared':len(new_ik['attempts']),
            'wall_time_excluded':True, 'hardware_validated':False}
