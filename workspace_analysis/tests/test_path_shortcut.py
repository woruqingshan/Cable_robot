"""Standard shortcut evidence, unchanged search and explicit termination semantics."""
from copy import deepcopy
from pathlib import Path
import json
import numpy as np
import pytest
from test_path_planning import FakeEvaluator,FakeModel
from workspace_analysis.planning.validity import ConfigurationValidity
from workspace_analysis.planning.path_shortcut import randomized_shortcut
from workspace_analysis.planning.joint_path import path_length,PathError
from workspace_analysis.planning.rrt_connect import rrt_connect
from workspace_analysis.experiments.path_trial_artifacts import termination
ROOT=Path(__file__).resolve().parents[1]


def bent():return np.array([[0.,0,0,0,0],[0,1,0,0,0],[1,1,0,0,0]])


def test_shortcut_preserves_input_endpoints_and_fresh_final_evidence():
    q=bent();old=q.copy();checker=ConfigurationValidity(FakeEvaluator())
    result=randomized_shortcut(q,checker,resolution_rad=.05,max_attempts=100,seed=11)
    np.testing.assert_array_equal(q,old)
    assert result['status']=='SHORTCUT_VALIDATED' and result['success']
    assert path_length(result['waypoints'])<path_length(q)
    np.testing.assert_array_equal(result['waypoints'][0],q[0]);np.testing.assert_array_equal(result['waypoints'][-1],q[-1])
    assert result['accepted_shortcuts']==1 and result['attempts_executed']==1
    assert result['validation']['query_statistics']['cache_hits']==0
    assert result['validation']['query_statistics']['native_collision_queries']==result['validation']['checked_samples']
    for a in result['attempts']:
        if a['accepted']:assert a['edge_validation']['valid'] and a['edge_validation']['complete']


def test_blocked_direct_connection_is_not_accepted_and_no_improvement_keeps_raw():
    blocked=lambda q:.3<q[0]<.7 and abs(q[0]-q[1])<.08
    q=bent();checker=ConfigurationValidity(FakeEvaluator(blocked))
    result=randomized_shortcut(q,checker,resolution_rad=.02,max_attempts=8)
    assert result['status']=='NO_IMPROVEMENT' and result['success']
    np.testing.assert_array_equal(result['waypoints'],q)
    assert all(not a['accepted'] and not a['edge_validation']['valid'] for a in result['attempts'])
    assert result['validation']['valid']


def test_fixed_seed_shortcut_decisions_are_reproducible():
    q=np.array([[0,0,0,0,0],[0,.5,0,0,0],[.4,.8,0,0,0],[.7,.5,0,0,0],[1,1,0,0,0]])
    outputs=[randomized_shortcut(q,ConfigurationValidity(FakeEvaluator()),seed=77,resolution_rad=.05) for _ in range(2)]
    assert outputs[0]['waypoints']==outputs[1]['waypoints']
    assert [(a['i'],a['j'],a['accepted']) for a in outputs[0]['attempts']]==[(a['i'],a['j'],a['accepted']) for a in outputs[1]['attempts']]


@pytest.mark.parametrize('settings',[{'max_attempts':-1},{'max_attempts':True},{'seed':-1},{'resolution_rad':0},{'reduction_epsilon_rad':float('nan')}])
def test_shortcut_rejects_invalid_settings(settings):
    with pytest.raises(PathError):randomized_shortcut(bent(),ConfigurationValidity(FakeEvaluator()),**settings)


def test_invalid_raw_input_or_backend_error_cannot_produce_playable_path():
    result=randomized_shortcut(bent(),ConfigurationValidity(FakeEvaluator(lambda q:q[1]>.4)),resolution_rad=.05)
    assert not result['success'] and result['status']=='INVALID_INPUT_PATH' and result['waypoints'] is None
    with pytest.raises(PathError,match='unresolved'):
        randomized_shortcut(bent(),ConfigurationValidity(FakeEvaluator(error=True)))


def test_zero_attempt_budget_and_direct_path_still_get_final_validation():
    q=bent();result=randomized_shortcut(q,ConfigurationValidity(FakeEvaluator()),max_attempts=0)
    assert result['status']=='NO_IMPROVEMENT' and result['validation']['valid']
    result=randomized_shortcut(q[[0,2]],ConfigurationValidity(FakeEvaluator()))
    assert result['attempts_executed']==0 and result['stop_reason']=='NO_NONADJACENT_VERTEX_PAIRS'


@pytest.mark.parametrize('status,expected',[('PLANNER_TIMEOUT','TIME_LIMIT'),('PLANNER_BUDGET_EXHAUSTED','ITERATION_LIMIT'),
    ('PATH_VALIDATION_FAILED','PATH_VALIDATION_FAILURE'),('INVALID_START','INVALID_START'),('INVALID_GOAL','INVALID_GOAL'),('BACKEND_ERROR','BACKEND_ERROR')])
def test_trial_termination_reasons_are_not_conflated(status,expected):
    assert termination({'status':status})==expected
    assert termination({'status':'NOT_RUN','reason':'INVALID_START'})=='INVALID_START'


def test_distinct_rrt_seeds_are_forwarded_and_repeat_seed_preserves_path():
    a=[-.8,0,0,0,0];b=[.8,0,0,0,0]
    blocked=lambda q:abs(q[0])<.15 and abs(q[1])<.3
    out=[]
    for seed in (17,18,17):
        checker=ConfigurationValidity(FakeEvaluator(blocked))
        trial=rrt_connect(a,b,FakeModel.limits,checker,seed=seed,max_iterations=500,max_time_s=10,step_rad=.2,resolution_rad=.05)
        assert trial['seed']==seed and trial['search_found'];out.append(trial['waypoints'])
    assert out[0]==out[2] and out[0]!=out[1]


def test_preregistered_matrix_has_ninety_main_trials_and_shared_ten_seed_list():
    cfg=json.loads((ROOT/'config/restricted_scenes/robustness_v0.json').read_text())
    assert cfg['seeds']==list(range(20261104,20261114)) and len(set(cfg['seeds']))==10
    assert len(cfg['cases'])*len(cfg['starts'])*len(cfg['seeds'])==90
    assert cfg['shortcut']['max_attempts']==100 and cfg['budget_sensitivity']['max_iterations_unchanged']==1500


def test_local_only_shortcut_uses_production_fk_and_unified_constraint():
    from test_task_benchmark import fixture_context
    from workspace_analysis.planning.local_workspace import LocalOnlyWorkspaceConstraint
    from workspace_analysis.planning.workspace_constraint import TCPWorkspaceCheck
    original,_=fixture_context()
    checker=ConfigurationValidity(original.checker.evaluator,workspace_check=TCPWorkspaceCheck(LocalOnlyWorkspaceConstraint(),np.eye(4),np.eye(4),{}))
    q=np.array([[0,0,0,0,0],[.15,-.1,0,0,0],[.1,0,0,0,0]])
    result=randomized_shortcut(q,checker,max_attempts=20)
    assert result['success'] and all(s['workspace']['valid'] for s in result['validation']['states'])
    bad=np.array([[0]*5,[1,0,0,0,0]])
    result=randomized_shortcut(bad,checker,max_attempts=1)
    assert result['status']=='INVALID_INPUT_PATH' and not result['success']


def test_failed_fresh_candidate_can_only_fall_back_to_fresh_valid_raw():
    checker=ConfigurationValidity(FakeEvaluator());real=checker.path;calls=0
    def simulated(*args,**kwargs):
        nonlocal calls
        report=real(*args,**kwargs)
        if kwargs.get('fresh'):
            calls+=1
            if calls==2:
                report['valid']=False;report['states'][0]['valid']=False
                report['states'][0]['reasons']=['SEMANTIC_SELF_COLLISION']
        return report
    checker.path=simulated
    q=bent();result=randomized_shortcut(q,checker,resolution_rad=.05)
    assert result['success'] and result['status']=='FALLBACK_ORIGINAL_VALIDATED'
    np.testing.assert_array_equal(result['waypoints'],q)
    assert result['validation']['valid'] and not result['candidate_final_validation']['valid']


def test_failed_candidate_and_failed_fallback_never_return_a_path():
    checker=ConfigurationValidity(FakeEvaluator());real=checker.path;calls=0
    def simulated(*args,**kwargs):
        nonlocal calls
        report=real(*args,**kwargs)
        if kwargs.get('fresh'):
            calls+=1
            if calls>=2:
                report['valid']=False;report['states'][0]['valid']=False
                report['states'][0]['reasons']=['SEMANTIC_SELF_COLLISION']
        return report
    checker.path=simulated
    result=randomized_shortcut(bent(),checker,resolution_rad=.05)
    assert not result['success'] and result['status']=='FINAL_VALIDATION_FAILURE' and result['waypoints'] is None
