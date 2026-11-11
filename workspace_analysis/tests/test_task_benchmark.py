"""T5.1 contracts/orchestration using production FK/planners and explicit test oracles."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from workspace_analysis.robot.model import model_from_xml
from workspace_analysis.robot.fk import ForwardKinematics
from workspace_analysis.scene.schema import load_scene, parse_scene
from workspace_analysis.task.spec import load_task, parse_task, TaskError
from workspace_analysis.task.targets import build_targets
from workspace_analysis.task.template import tasks_for_ports
from workspace_analysis.experiments.task_context import TaskContext, TaskSystemError, select_common_start
from workspace_analysis.experiments.task_reachability import run_task_reachability
from workspace_analysis.experiments.task_path_planning import run_task_path
from workspace_analysis.experiments.task_benchmark import run_task, run_tasks, benchmark_summary, checked_settings, run_benchmark
from workspace_analysis.planning.validity import ConfigurationValidity

ROOT = Path(__file__).resolve().parents[1]
PORTS = ['P11','P12','P13','P21','P22','P23','P31','P32','P33']


def inputs():
    return load_task(ROOT/'config/tasks/p22_research_v0.yaml'), load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')


def fixture_context(blocked=lambda q: False):
    spec, scene = inputs()
    # Minimal five-joint fixture, with zero FK matching the existing P22 virtual TCP.
    links = '<link name="base_link"/>' + ''.join(f'<link name="l{i}"/>' for i in range(5)) + '<link name="end_effector_link"/>'
    joints = ''.join(f'<joint name="joint{i+1}" type="revolute"><parent link="'+('base_link' if i == 0 else f'l{i-1}')+
        f'"/><child link="l{i}"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" effort="1" velocity="1"/></joint>' for i in range(5))
    fixed = '<joint name="tip" type="fixed"><parent link="l4"/><child link="end_effector_link"/><origin xyz=".26 0 .22" rpy="0 1.5707963267948966 0"/></joint>'
    model = model_from_xml('<robot name="fixture">'+links+joints+fixed+'</robot>'); fk = ForwardKinematics(model)
    class Oracle:
        """Synthetic collision geometry only; all planning/validity code is real."""
        def __init__(self):
            self.model = model; self.scene = scene; self.gripper = .3; self.clearance = {'self_m':0., 'environment_m':0.}
            self.semantic = SimpleNamespace(payload_json=json.dumps({'srdf_sha256':'srdf', 'review_sha256':'review'}))
            self.identity = {'urdf_sha256': model.metadata()['expanded_urdf_sha256'], 'scene_sha256': scene.canonical_scene_sha256,
                             'srdf_sha256':'srdf', 'semantic_review_sha256':'review'}
            self.blocked = blocked; self.error = False; self.unknown = False
        def evaluate(self, values):
            q = values[0]; collision = bool(self.blocked(q))
            self.last_native = {'T_base_to_ee': fk(q).tolist(), 'link_transforms_base': {'base_link': np.eye(4).tolist()}, 'joint_positions_used': {}}
            if self.error: return [{'error':'fixture backend fault'}]
            return [{'model_configuration_valid': None if self.unknown else not collision,
                'semantic_self_collision': collision, 'environment_collision': False, 'semantic_blocking_pairs': [], 'raw_collision_pairs': [],
                'configuration_decision': {'planning_approved':False,'hardware_validated':False},
                'semantic_clearance': {'native': {k: {'available':True, 'separation_m':0. if collision else .01} for k in ('self','environment')}}}]
    checker = ConfigurationValidity(Oracle())
    context = TaskContext(model, scene, build_targets(spec, scene), checker)
    q = np.zeros((1,5)); t = fk(q[0])
    arrays = {'q':q, 'position':t[None,:3,3], 'rotation':t[None,:3,:3]}
    return context, arrays


@pytest.mark.parametrize('port', PORTS)
def test_arbitrary_ports_use_framegraph_and_distinct_identity(port):
    spec, scene = inputs(); task = tasks_for_ports(spec, scene, [port])[0]
    target = build_targets(task, scene); actual_port = next(p for p in scene.ports if p['id'] == port)
    t = scene.frame_graph.resolve('base_link', actual_port['frame'])
    np.testing.assert_allclose(target.base_to_port.matrix(), t, atol=1e-12)
    np.testing.assert_allclose(target.pre_tcp.matrix()[:3,3], t[:3,3]-.04*np.asarray(target.insertion_axis_base), atol=1e-12)
    assert task.tcp_sha256 == spec.tcp_sha256
    assert (task.sha256 == spec.sha256) == (port == 'P22')


def test_p22_exact_identity_and_rotated_translated_scene():
    spec, scene = inputs(); assert tasks_for_ports(spec, scene, ['P22'])[0] == spec
    data = scene.to_dict(); original = build_targets(spec, scene)
    frame = next(f for f in data['scene']['frames'] if f['parent'] == 'base_link')
    r = Rotation.from_euler('z', .4); frame['pose']['quaternion_xyzw'] = r.as_quat().tolist()
    frame['pose']['position_m'] = [.1,-.2,.3]
    moved = parse_scene(json.dumps(data)); target = build_targets(spec, moved)
    port = next(p for p in moved.ports if p['id'] == 'P22')
    t = moved.frame_graph.resolve('base_link', port['frame'])
    np.testing.assert_allclose(target.insertion_axis_base, t[:3,:3] @ port['insertion_axis_local'], atol=1e-12)
    assert target.scene_sha256 != original.scene_sha256


@pytest.mark.parametrize('ports', [[], ['P22','P22'], ['missing'], [None]])
def test_invalid_ports_rejected(ports):
    spec, scene = inputs()
    with pytest.raises(TaskError): tasks_for_ports(spec, scene, ports)


def test_invalid_task_frame_targets_and_session_identity_rejected():
    context, _ = fixture_context(); context.validate()
    data = context.targets.spec.data; data['tcp']['T_ee_tcp']['position_m'][0] = float('nan')
    with pytest.raises(TaskError): tasks_for_ports(parse_task(data), context.scene, ['P22'])
    with pytest.raises(TaskError): replace(context, targets=replace(context.targets, scene_sha256='wrong')).validate()
    context.checker.evaluator.identity['srdf_sha256'] = 'wrong'
    with pytest.raises(TaskSystemError): context.validate()


@pytest.mark.parametrize('count', [0,1,2])
def test_zero_one_multiple_ik_solutions_and_collision_screening(monkeypatch, count):
    import workspace_analysis.experiments.task_reachability as module
    context, arrays = fixture_context(lambda q: q[0] > .1)
    # Only solver return cardinality is controlled; real FK/threshold/checker screen it.
    solutions = [{'solution_id':i,'q':[.2*i,-.2*i,0.,0.,0.]} for i in range(count)]
    monkeypatch.setattr(module.NumericalIK, 'solve', lambda self,seeds,**kw: {'unique_task_solutions':solutions})
    result = run_task_reachability(context, arrays)
    assert len(result['screened']) == count
    assert result['candidate_count'] == min(count,1)
    if count == 0: assert result['status'] == 'IK_NO_SOLUTION_WITHIN_BUDGET' and not context.checker.queries
    if count == 2: assert result['screened'][1]['task_check']['task_pose_feasible'] and not result['screened'][1]['candidate']


def test_common_start_once_original_index_and_invalid_start():
    context, _ = fixture_context(lambda q: q[0] < 0)
    q = [[-.2,0,0,0,0],[0,0,0,0,0],[.2,0,0,0,0]]
    selected = select_common_start(context.checker, q)
    assert selected['original_sample_index'] == 1 and len(selected['tested']) == 2
    result = run_task(context, {'q':[]}, q[0])
    assert result['status'] == 'INVALID_START' and result['reachability']['status'] == 'NOT_RUN'
    with pytest.raises(ValueError): select_common_start(context.checker, [[3]*5])


def test_linear_blocked_rrt_succeeds_and_fresh_validation():
    context, _ = fixture_context(lambda q: abs(q[0]) < .12 and abs(q[1]) < .3)
    start = [-.8,0,0,0,0]; goal = [.8,0,0,0,0]
    params = {'random_seed':20261104, 'max_time_s':10., 'max_iterations':1000}
    linear = run_task_path(context, start, goal, method='linear', settings=params)
    rrt = run_task_path(context, start, goal, method='rrt_connect', settings=params)
    assert linear['status'] == 'PATH_VALIDATION_FAILED' and rrt['success']
    assert rrt['validation']['query_statistics']['cache_hits'] == 0
    assert not rrt['continuous_collision_proven']


def test_search_success_cannot_override_failed_final_check(monkeypatch):
    context, _ = fixture_context(); original = context.checker.path
    def changed_geometry(*args, **kwargs):
        if kwargs.get('fresh'): context.checker.evaluator.blocked = lambda q: True
        return original(*args, **kwargs)
    monkeypatch.setattr(context.checker, 'path', changed_geometry)
    result = run_task_path(context, [-.2,0,0,0,0], [.2,0,0,0,0], method='rrt_connect')
    assert result['search']['search_found'] and not result['success']
    assert result['status'] == 'PATH_VALIDATION_FAILED'


@pytest.mark.parametrize('kind', ['error','unknown'])
def test_unknown_native_is_system_error_not_collision_failure(kind):
    context, _ = fixture_context(); setattr(context.checker.evaluator, kind, True)
    with pytest.raises(TaskSystemError): run_task_path(context, [0]*5, [0]*5, method='linear')


def test_one_failed_port_continues_but_backend_error_blocks_followers(monkeypatch):
    import workspace_analysis.experiments.task_benchmark as module
    context, arrays = fixture_context()
    first = replace(context, targets=build_targets(tasks_for_ports(context.targets.spec, context.scene, ['P11'])[0], context.scene))
    def failed_task(ctx, *args, **kwargs):
        return {'port_id':ctx.targets.spec.data['target']['port_id'], 'status':'IK_NO_SOLUTION_WITHIN_BUDGET'}
    monkeypatch.setattr(module, 'run_task', failed_task)
    rows = list(module.run_tasks([first,context], arrays, [0]*5))
    assert [r['port_id'] for r in rows] == ['P11','P22']
    def system_fault(*args, **kwargs): raise TaskSystemError('fixture identity mismatch')
    monkeypatch.setattr(module, 'run_task', system_fault)
    assert [r['status'] for r in module.run_tasks([first,context], arrays, [0]*5)] == ['ERROR','BLOCKED']


def test_seed_reproducibility_and_explicit_parameter_rejection():
    from workspace_analysis.planning.ik import build_seeds
    context, arrays = fixture_context(); targets = context.targets
    a = build_seeds(context.model, targets, arrays); assert a == build_seeds(context.model, targets, arrays)
    data = targets.spec.data; data['solver']['random_seed'] += 1
    other = build_targets(parse_task(data), context.scene)
    b = build_seeds(context.model, other, arrays)
    assert a != b and b[-2]['random_seed'] == data['solver']['random_seed']
    for path in [{'rrt_mode':'on-failure'}, {'verification_resolution_rad':.02}, {'random_seed':3}]:
        with pytest.raises(TaskError): checked_settings(path, None, targets.spec)


def test_existing_output_rejected_before_loading_model(tmp_path):
    out = tmp_path/'exists'; out.mkdir()
    with pytest.raises(TaskError, match='existing'):
        run_benchmark(tmp_path, tmp_path/'absent', tmp_path/'scene', tmp_path/'task', tmp_path/'seed', out, port_ids=['P22'], allow_candidate=True)


def test_summary_denominators_and_not_run_are_explicit():
    record = {'port_id':'P11', 'status':'IK_NO_SOLUTION_WITHIN_BUDGET', 'elapsed_s':.1,
        'reachability':{'ik':{'seed_count':4,'optimization_count':12,'attempts':[], 'unique_task_solutions':[], 'best_attempt':None}, 'candidate_count':0}, 'goals':[]}
    summary = benchmark_summary([record])
    assert summary['port_denominator'] == 1 and summary['successful_ports'] == 0
    row = summary['rows'][0]
    assert row['ik_attempts'] == 12 and row['linear_status'] == 'NOT_RUN' and row['approach_status'] == 'NOT_RUN'
    with pytest.raises(TaskError): benchmark_summary([record,record])


def test_actual_numerical_ik_through_generic_interface_is_deterministic():
    context, arrays = fixture_context()
    data = context.targets.spec.data
    data['solver'].update(workspace_nearest_seeds=1, extra_joint_seeds=0, include_midpoint_seed=False, max_nfev=10)
    context = replace(context, targets=build_targets(parse_task(data), context.scene))
    a = run_task_reachability(context, arrays); b = run_task_reachability(context, arrays)
    assert a['candidate_count'] == b['candidate_count'] == 1
    assert a['ik']['optimization_count'] == 3
    assert [r['q'] for r in a['ik']['attempts']] == [r['q'] for r in b['ik']['attempts']]


def test_generic_approach_rejects_interior_task_error(monkeypatch):
    from workspace_analysis.experiments.task_approach import run_task_approach
    import workspace_analysis.planning.cartesian_approach as module
    import workspace_analysis.planning.ik as ik_module
    from test_cartesian_approach import toy
    context, _ = fixture_context()
    model, fk, solver, checker = toy(context.targets, kind='cross')
    # Borrow the established synthetic FK/collision oracle, retaining real
    # sequential_approach and every production interpolation/threshold check.
    model.base_frame = 'base_link'; model.ee_frame = 'end_effector_link'
    model.metadata = context.model.metadata
    checker.evaluator.scene = context.scene
    checker.evaluator.semantic = context.checker.evaluator.semantic
    checker.evaluator.identity = context.checker.evaluator.identity
    derived = TaskContext(model, context.scene, context.targets, checker)
    monkeypatch.setattr(module, 'ForwardKinematics', lambda model: fk)
    monkeypatch.setattr(ik_module, 'ForwardKinematics', lambda model: fk)
    monkeypatch.setattr(module.NumericalIK, 'solve', lambda self,seeds,**kwargs: solver(self.model,self.targets).solve(seeds))
    result = run_task_approach(derived, [.29,0,0,0,0], settings={'edge_resolution_rad':.002, 'backup_seeds':0})
    assert not result['success'] and result['first_failure']['kind'] == 'CARTESIAN_TASK_CONSTRAINT'
    assert 'TCP_POSITION_TOLERANCE' in result['first_failure']['reasons']
