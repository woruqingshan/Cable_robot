"""Task hierarchy plus fail-closed configuration/discrete-edge validity."""
import math
import numpy as np
from ..task.targets import tcp_errors
from ..task.spec import TaskError
from ..robot.fk import ForwardKinematics
from ..collision.evaluator import Evaluator
from .joint_path import configuration, sample_path, PathError, PlanningStopped
from time import perf_counter
from copy import deepcopy


def assess_task(errors,targets,*,joint_limits_valid=True,collision=None):
    """Single final status with atomic errors and cumulative reachability levels.

    Collision rejection retains TASK_POSE_FEASIBLE and a separate rejection status.
    POSITION_REACHABLE/AXIS_ALIGNED always imply all earlier requirements hold.
    """
    tol=targets.spec.tolerance
    for key in ['position_m','insertion_axis_rad','key_axis_rad']:
        if type(errors.get(key)) not in (int,float) or not math.isfinite(errors[key]) or errors[key]<0:raise TaskError('invalid task error metric')
    if type(joint_limits_valid) is not bool:raise TaskError('joint-limit check must be boolean')
    atomic={'position':errors['position_m']<=tol['position_m'],'insertion_axis':errors['insertion_axis_rad']<=tol['axis_rad'],'key_axis':errors['key_axis_rad']<=tol['key_rad']}
    position=joint_limits_valid and atomic['position'];axis=position and atomic['insertion_axis'];pose=axis and atomic['key_axis']
    status='TASK_POSE_FEASIBLE' if pose else 'AXIS_ALIGNED' if axis else 'POSITION_REACHABLE' if position else 'IK_FAILED'
    reasons=[]
    if not joint_limits_valid:reasons.append('URDF_JOINT_LIMIT')
    reasons.extend(code for kind,code in [('position','TCP_POSITION_TOLERANCE'),('insertion_axis','INSERTION_AXIS_TOLERANCE'),('key_axis','KEY_DIRECTION_TOLERANCE')] if not atomic[kind])
    collision_status='NOT_CHECKED' if pose else 'NOT_RUN_TASK_POSE_FAILED';candidate=False
    if collision is not None:
        if not pose:raise TaskError('collision approval cannot be attached to a failed task pose')
        decision=collision.get('configuration_decision',{})
        if decision.get('planning_approved') is not False or decision.get('hardware_validated') is not False:raise TaskError('collision result lacks research-only decision identity')
        if 'error' in collision or decision.get('status')=='ERROR':collision_status='ERROR';reasons.append('COLLISION_QUERY_ERROR')
        elif collision.get('model_configuration_valid') is None:collision_status='UNVERIFIED';reasons.append('COLLISION_CLEARANCE_UNVERIFIED')
        elif collision['model_configuration_valid'] is False:
            collision_status='REJECTED'
            if collision.get('semantic_self_collision'):reasons.append('SEMANTIC_SELF_COLLISION')
            if collision.get('environment_collision'):reasons.append('ROBOT_ENVIRONMENT_COLLISION')
            if not collision.get('semantic_self_collision') and not collision.get('environment_collision'):reasons.append('CLEARANCE_REQUIREMENT')
        elif collision['model_configuration_valid'] is True:
            if collision.get('semantic_self_collision') is not False or collision.get('environment_collision') is not False:raise TaskError('optimistic/inconsistent collision pass')
            candidate=True;collision_status='MODEL_PASS_RESEARCH_ONLY';status='TASK_CONFIGURATION_CANDIDATE'
        else:raise TaskError('invalid model configuration validity')
    return {'usage':'RESEARCH_ONLY','status':status,'joint_limits_valid':joint_limits_valid,'within_tolerance':atomic,
        'position_reachable':position,'axis_aligned':axis,'task_pose_feasible':pose,'task_configuration_candidate':candidate,
        'collision_status':collision_status,'rejection_reasons':reasons,'errors':errors,
        'tolerances':tol,'hardware_validated':False,'planning_approved':False,'tcp_collision_geometry_modeled':False}


def screen_solutions(model,scene,targets,solutions,semantic,backend):
    """Recheck task errors using the authoritative FK before invoking Evaluator.

    If no pose satisfies all tolerances no native collision session is created.
    The returned record count and order match the input solution list.
    """
    if model.base_frame!=targets.spec.data['target']['reference_frame'] or model.ee_frame!=targets.spec.data['target']['ee_frame'] or scene.canonical_scene_sha256!=targets.scene_sha256:raise TaskError('task/model/scene mismatch')
    fk=ForwardKinematics(model);result=[];eligible=[]
    for i,solution in enumerate(solutions):
        q=model.validate_q(solution['q']);t=fk(q)@targets.spec.T_ee_tcp;errors=tcp_errors(t,targets);assessment=assess_task(errors,targets)
        result.append({'solution_id':solution.get('solution_id',i),'q':q.tolist(),'T_base_tcp':t.tolist(),'task_check':assessment,'collision':None})
        if assessment['task_pose_feasible']:eligible.append(i)
    if not eligible:return {'results':result,'identity':None,'cleanup':None,'collision_queries':0,'usage':'RESEARCH_ONLY'}
    settings=targets.spec.data['collision']
    with Evaluator(model,scene,None,backend,semantic=semantic,gripper=settings['gripper_rad'],
                   clearance={'self_m':settings['self_clearance_m'],'environment_m':settings['environment_clearance_m']},timeout=60) as evaluator:
        rows=evaluator.evaluate(np.array([result[i]['q'] for i in eligible]))
        for index,row in zip(eligible,rows):
            record=result[index];record['collision']=row;record['task_check']=assess_task(record['task_check']['errors'],targets,collision=row)
        identity=evaluator.identity
    if not evaluator.client.cleanup['exit_normal']:raise TaskError('collision backend exited abnormally')
    return {'results':result,'identity':identity,'cleanup':evaluator.client.cleanup,'collision_queries':len(eligible),'usage':'RESEARCH_ONLY'}


class ConfigurationValidity:
    """Use ONE immutable semantic Evaluator session; never use legacy final_decision.

    Cache keys are exact float tuples, never rounded. Fresh final validation bypasses
    the cache. Every real query retains the native RobotState transforms for replay.
    This object does not own/close the caller's evaluator and is not thread-safe.
    """
    def __init__(self, evaluator):
        if evaluator.semantic is None:
            raise PathError('path validity requires explicit semantic SRDF')
        self.evaluator = evaluator
        self.model = evaluator.model
        self.cache = {}
        self.queries = []
        self.requests = 0
        self.cache_hits = 0

    def statistics(self):
        return {'configuration_requests': self.requests, 'cache_hits': self.cache_hits,
                'native_collision_queries': len(self.queries)}

    def check(self, q, *, fresh=False, deadline=None):
        if deadline is not None and perf_counter() >= deadline:
            raise PlanningStopped('planning deadline reached between native queries')
        self.requests += 1
        q = configuration(q)
        try:
            self.model.validate_q(q)
        except ValueError as exc:
            return {'q': q.tolist(), 'valid': False, 'status': 'REJECTED',
                    'reasons': ['URDF_JOINT_LIMIT'], 'error': str(exc), 'query_id': None}
        key = tuple(q)
        if not fresh and key in self.cache:
            self.cache_hits += 1
            return deepcopy(self.cache[key])
        row = self.evaluator.evaluate(q.reshape(1, 5))[0]
        reasons = []
        decision = row.get('configuration_decision', {})
        research_identity = decision.get('planning_approved') is False and decision.get('hardware_validated') is False
        if 'error' in row or not research_identity or decision.get('status') == 'ERROR':
            reasons.append('COLLISION_QUERY_ERROR')
        else:
            if row.get('semantic_self_collision') is True: reasons.append('SEMANTIC_SELF_COLLISION')
            if row.get('environment_collision') is True: reasons.append('ROBOT_ENVIRONMENT_COLLISION')
            if row.get('model_configuration_valid') is None: reasons.append('CLEARANCE_OR_COLLISION_UNVERIFIED')
            if row.get('model_configuration_valid') is False and not reasons: reasons.append('CLEARANCE_REQUIREMENT')
            if row.get('model_configuration_valid') is True and (
                    row.get('semantic_self_collision') is not False or row.get('environment_collision') is not False):
                reasons.append('INCONSISTENT_SEMANTIC_RESULT')
            if type(row.get('model_configuration_valid')) not in (bool, type(None)):
                reasons.append('INVALID_VALIDITY_FIELD')
        valid = not reasons and row.get('model_configuration_valid') is True
        native = self.evaluator.last_native if 'error' not in row else None
        record = {'query_id': len(self.queries), 'q': q.tolist(), 'valid': valid,
                  'status': 'MODEL_PASS_RESEARCH_ONLY' if valid else 'ERROR' if 'COLLISION_QUERY_ERROR' in reasons else 'REJECTED',
                  'reasons': reasons, 'semantic_self_collision': row.get('semantic_self_collision'),
                  'environment_collision': row.get('environment_collision'),
                  'semantic_blocking_pairs': row.get('semantic_blocking_pairs', []),
                  'environment_pairs': [p for p in row.get('raw_collision_pairs', []) if p['kind'] == 'environment'],
                  'pairs_may_be_truncated': row.get('pairs_may_be_truncated'),
                  'semantic_pairs_may_be_truncated': row.get('semantic_pairs_may_be_truncated'),
                  'contact_surface_complete': False,
                  'semantic_clearance': row.get('semantic_clearance'), 'query_time_s': row.get('query_time'),
                  'error': row.get('error'), 'usage': 'RESEARCH_ONLY', 'hardware_validated': False,
                  'planning_approved': False}
        if native is not None:
            record.update(T_base_to_ee=native['T_base_to_ee'], link_transforms_base=native['link_transforms_base'],
                          contacts_base=native.get('contacts_base', []), joint_positions_used=native['joint_positions_used'])
        self.queries.append(deepcopy(record))
        # Query failures are never cached, so a stale error cannot masquerade as a fresh check.
        if record['status'] != 'ERROR': self.cache[key] = deepcopy(record)
        return record

    def edge(self, a, b, resolution_rad, *, fresh=False, stop_first=True, deadline=None):
        return self.path([a, b], resolution_rad, fresh=fresh, stop_first=stop_first, deadline=deadline)

    def path(self, waypoints, resolution_rad, *, fresh=False, stop_first=False, deadline=None):
        """Check sampled states, including both endpoints; no continuous guarantee."""
        begin = perf_counter(); before = self.statistics()
        states, provenance = sample_path(waypoints, resolution_rad)
        records = []; first_failure = None
        for index, q in enumerate(states):
            record = self.check(q, fresh=fresh, deadline=deadline)
            records.append(record)
            if not record['valid'] and first_failure is None:
                first_failure = {'path_sample_index': index, **provenance[index], 'q': q.tolist(),
                                 'reasons': record['reasons'], 'query_id': record['query_id']}
                if stop_first: break
        complete = len(records) == len(states)
        clearance = {}
        for kind in ['self', 'environment']:
            values = [(r.get('semantic_clearance') or {}).get('native', {}).get(kind, {}) for r in records]
            distances = [v.get('separation_m') for v in values if v.get('available') and v.get('separation_m') is not None]
            clearance[kind] = {'minimum_m': min(distances) if distances else None,
                               'all_checked_samples_available': len(distances) == len(records),
                               'scope': 'unsigned FCL separation at sampled states only'}
        return {'valid': complete and first_failure is None, 'complete': complete,
                'validation': 'DISCRETE_ONLY', 'continuous_collision_proven': False,
                'resolution_max_joint_rad': resolution_rad, 'expected_samples': len(states),
                'checked_samples': len(records), 'states': records, 'provenance': provenance[:len(records)],
                'first_failure': first_failure, 'clearance': clearance,
                'elapsed_s': perf_counter()-begin,
                'query_statistics': {k: self.statistics()[k]-v for k, v in before.items()}}
