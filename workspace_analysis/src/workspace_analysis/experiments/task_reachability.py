"""Generic IK and fresh semantic screening, using the unchanged T2 algorithms."""
from typing import Any
import numpy as np
from .task_context import TaskContext, require_resolved
from ..planning.ik import NumericalIK, build_seeds
from ..planning.validity import assess_task
from ..robot.fk import ForwardKinematics
from ..task.targets import tcp_errors


def run_task_reachability(context: TaskContext, workspace_arrays: dict[str, Any],
                          *, progress=None) -> dict:
    """Solve every declared seed, recheck FK thresholds, screen all unique poses."""
    context.validate()
    seeds = build_seeds(context.model, context.targets, workspace_arrays)
    result = NumericalIK(context.model, context.targets).solve(seeds, progress=progress)
    fk = ForwardKinematics(context.model)
    screened = []
    for solution in result['unique_task_solutions']:
        q = context.model.validate_q(solution['q'])
        errors = tcp_errors(fk(q) @ context.targets.spec.T_ee_tcp, context.targets)
        task = assess_task(errors, context.targets)
        if not task['task_pose_feasible']:
            raise ValueError('IK solution failed authoritative task recheck')
        collision = context.checker.check(q, fresh=True)
        require_resolved([collision])
        screened.append({'solution_id': solution['solution_id'], 'q': q.tolist(),
                         'errors': errors, 'task_check': task, 'collision': collision,
                         'candidate': collision['valid']})
    return {'seeds': seeds, 'ik': result, 'screened': screened,
            'candidate_count': sum(s['candidate'] for s in screened),
            'status': 'IK_NO_SOLUTION_WITHIN_BUDGET' if not screened else
                      'COLLISION_REJECTED' if not any(s['candidate'] for s in screened) else 'TASK_CANDIDATES',
            'usage': 'RESEARCH_ONLY', 'mathematical_unreachability_proven': False}
