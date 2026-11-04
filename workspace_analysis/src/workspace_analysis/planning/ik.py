"""Bounded deterministic multi-start IK using the existing PyKDL FK only."""
from time import perf_counter
import math
import numpy as np
from scipy.optimize import least_squares
from ..robot.fk import ForwardKinematics
from ..task.spec import TaskError
from ..task.targets import tcp_errors
from .validity import assess_task


def build_seeds(model,targets,workspace_arrays):
    """Rank existing q by virtual TCP position, then add deterministic joint seeds.

    G4 pose arrays are consumed, not recomputed as a second batch FK implementation.
    """
    settings=targets.spec.data['solver'];limits=model.limits
    q=np.asarray(workspace_arrays['q']);p=np.asarray(workspace_arrays['position']);r=np.asarray(workspace_arrays['rotation'])
    if q.ndim!=2 or q.shape[1]!=len(model.joint_order) or p.shape!=(len(q),3) or r.shape!=(len(q),3,3) or not np.isfinite(p).all() or not np.isfinite(r).all():raise TaskError('invalid seed workspace arrays')
    positions=p+np.einsum('nij,j->ni',r,targets.spec.T_ee_tcp[:3,3]);dist=np.linalg.norm(positions-np.array(targets.pre_tcp.position_m),axis=1)
    order=np.argsort(dist,kind='stable')[:settings['workspace_nearest_seeds']]
    seeds=[{'q':model.validate_q(q[i]).tolist(),'source':'G4_nearest_virtual_TCP','workspace_sample_index':int(i),'initial_tcp_distance_m':float(dist[i])} for i in order]
    rng=np.random.default_rng(settings['random_seed'])
    for i,value in enumerate(rng.uniform(limits[:,0],limits[:,1],size=(settings['extra_joint_seeds'],len(model.joint_order)))):
        seeds.append({'q':value.tolist(),'source':'deterministic_joint_seed','generated_index':i,'random_seed':settings['random_seed']})
    if settings['include_midpoint_seed']:seeds.append({'q':limits.mean(axis=1).tolist(),'source':'URDF_midpoint_diagnostic_not_safe_reference'})
    unique=[]
    for seed in seeds:
        if not any(np.max(np.abs(np.asarray(seed['q'])-u['q']))<=1e-12 for u in unique):unique.append(seed)
    if not unique:raise TaskError('no IK seeds available')
    for i,seed in enumerate(unique):seed['seed_id']=i
    return unique


class NumericalIK:
    """One private PyKDL solver instance; no mutable global solver/state."""
    def __init__(self,model,targets):
        self.model=model;self.targets=targets;self.fk=ForwardKinematics(model)
        target=targets.spec.data['target']
        if model.base_frame!=target['reference_frame'] or model.ee_frame!=target['ee_frame']:raise TaskError('IK model/task frame mismatch')
        self.tool=targets.spec.T_ee_tcp;self.data=targets.spec.data;self.limits=model.limits
        self.a=np.array(self.data['tcp']['insertion_axis_local']);self.k=np.array(self.data['tcp']['key_axis_local'])
        self.goal=np.array(targets.pre_tcp.position_m);self.goal_a=np.array(targets.insertion_axis_base);self.goal_k=np.array(targets.key_axis_base)
        self.tolerance=targets.spec.tolerance

    def residual(self,q,stage):
        t=self.fk(q)@self.tool;weights=self.data['solver']['residual_weights']
        parts=[math.sqrt(weights['position'])*(t[:3,3]-self.goal)/self.tolerance['position_m']]
        if stage in ['axis','task']:
            parts.append(math.sqrt(weights['insertion_axis'])*(t[:3,:3]@self.a-self.goal_a)/(2*math.sin(self.tolerance['axis_rad']/2)))
        if stage=='task':parts.append(math.sqrt(weights['key_axis'])*(t[:3,:3]@self.k-self.goal_k)/(2*math.sin(self.tolerance['key_rad']/2)))
        if stage not in ['position','axis','task']:raise TaskError('unsupported IK objective')
        return np.concatenate(parts)

    def solve(self,seeds,*,progress=None):
        if not seeds:raise TaskError('at least one IK seed required')
        settings=self.data['solver'];attempts=[];started=perf_counter()
        for seed_index,seed in enumerate(seeds):
            x=self.model.validate_q(seed['q'])
            for stage in ['position','axis','task']:
                calls=0;begin=perf_counter()
                def residual(value):
                    nonlocal calls
                    calls+=1;return self.residual(value,stage)
                initial=x.copy()
                result=least_squares(residual,x,bounds=(self.limits[:,0],self.limits[:,1]),method='trf',jac='2-point',loss='linear',
                    x_scale=self.limits[:,1]-self.limits[:,0],max_nfev=settings['max_nfev'],ftol=settings['ftol'],xtol=settings['xtol'],gtol=settings['gtol'])
                x=self.model.validate_q(result.x);t_ee=self.fk(x);t_tcp=t_ee@self.tool;errors=tcp_errors(t_tcp,self.targets);check=assess_task(errors,self.targets)
                score=math.sqrt((errors['position_m']/self.tolerance['position_m'])**2+(errors['insertion_axis_rad']/self.tolerance['axis_rad'])**2+(errors['key_axis_rad']/self.tolerance['key_rad'])**2)
                tier=3 if check['task_pose_feasible'] else 2 if check['axis_aligned'] else 1 if check['position_reachable'] else 0
                attempts.append({'attempt_id':len(attempts),'seed_id':seed.get('seed_id',seed_index),'seed_source':seed,'stage':stage,'initial_q':initial.tolist(),'q':x.tolist(),
                    'optimizer_success':bool(result.success),'optimizer_status':int(result.status),'optimizer_message':str(result.message),
                    'nfev':int(result.nfev),'njev':None if result.njev is None else int(result.njev),'residual_calls_including_numeric_jacobian':calls,
                    'iterations':None,'iteration_count_available':False,'iteration_count_note':'SciPy1.8 least_squares does not expose accepted iteration count; nfev/njev/call counts are distinct.',
                    'cost':float(result.cost),'optimality':float(result.optimality),'active_mask':result.active_mask.tolist(),'elapsed_s':perf_counter()-begin,
                    'T_base_ee':t_ee.tolist(),'T_base_tcp':t_tcp.tolist(),'errors':errors,'task_check':check,'normalized_task_score':score,'constraint_tier':tier,'usage':'RESEARCH_ONLY'})
            if progress is not None:progress({'completed_seeds':seed_index+1,'total_seeds':len(seeds),'optimization_count':len(attempts)})
        ranked=sorted(attempts,key=lambda a:(-a['constraint_tier'],a['normalized_task_score'],a['attempt_id']))
        unique=[]
        for attempt in ranked:
            if not attempt['task_check']['task_pose_feasible']:continue
            near=next((u for u in unique if np.max(np.abs(np.array(u['q'])-attempt['q']))<=settings['deduplicate_joint_rad']),None)
            if near is not None:near['attempt_ids'].append(attempt['attempt_id'])
            else:unique.append({'solution_id':len(unique),'q':attempt['q'],'best_attempt_id':attempt['attempt_id'],'attempt_ids':[attempt['attempt_id']],'errors':attempt['errors'],'T_base_tcp':attempt['T_base_tcp'],'usage':'RESEARCH_ONLY'})
        return {'usage':'RESEARCH_ONLY','attempts':attempts,'unique_task_solutions':unique,'best_attempt':ranked[0],
            'seed_count':len(seeds),'optimization_count':len(attempts),'stages_per_seed':['position','axis','task'],
            'total_elapsed_s':perf_counter()-started,'solver':'scipy.optimize.least_squares/trf + existing PyKDL FK',
            'pose_success_is_threshold_based':True,'mathematical_unreachability_proven':False,
            'no_solution_reason':None if unique else 'No task-feasible solution found within recorded seeds/bounds/evaluation budget; inspect separate position/axis/key errors.'}
