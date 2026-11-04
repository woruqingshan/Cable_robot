"""T3 geometry, fail-closed semantic validity, real native checks and replay."""
from copy import deepcopy
import json
import os
from pathlib import Path
from types import SimpleNamespace
import subprocess
import sys
import numpy as np
import pytest
from workspace_analysis.planning.joint_path import interpolate, sample_path, path_length, PathError
from workspace_analysis.planning.validity import ConfigurationValidity, assess_task
from workspace_analysis.planning.rrt_connect import rrt_connect
from workspace_analysis.experiments.p22_path_planning import settings_checked

ROOT = Path(__file__).resolve().parents[1]


class FakeModel:
    limits = np.array([[-1., 1.]]*5)
    def validate_q(self, q):
        if np.any(np.abs(q)>1): raise ValueError('out of limits')
        return np.asarray(q)


class FakeEvaluator:
    """Test-only collision oracle; planning and checker are the production code."""
    semantic = object()
    model = FakeModel()
    def __init__(self, blocked=lambda q: False, *, unknown=False, error=False, clearance=False):
        self.blocked=blocked; self.unknown=unknown; self.error=error; self.clearance=clearance
        self.calls=0; self.last_native=None
    def evaluate(self, q):
        self.calls+=1; q=q[0]; blocked=bool(self.blocked(q)); t=np.eye(4); t[:3,3]=q[:3]
        self.last_native={'T_base_to_ee':t.tolist(),'link_transforms_base':{'base_link':np.eye(4).tolist()},
                          'contacts_base':[],'joint_positions_used':{}}
        if self.error: return [{'error':'native synthetic failure','configuration_decision':{}}]
        distance={'self':{'available':not self.unknown,'separation_m':None if self.unknown else 0. if blocked else .01},
                  'environment':{'available':True,'separation_m':.02}}
        return [{'model_configuration_valid':None if self.unknown else not(blocked or self.clearance),
            'semantic_self_collision':blocked,'environment_collision':False,
            'final_decision':'COLLIDING', # Deliberately opposite to successful semantic validity.
            'semantic_blocking_pairs':[['a','b']] if blocked else [],'raw_collision_pairs':[],
            'configuration_decision':{'planning_approved':False,'hardware_validated':False,'status':'RESEARCH_ONLY'},
            'semantic_clearance':{'native':distance},'query_time':0.}]


def test_interpolation_preserves_endpoints_resolution_no_wrap_and_length():
    a=np.array([-2., .2, 0., -.5, 0.]); b=np.array([2., -.1, .9, .5, .01])
    q=interpolate(a,b,.03)
    np.testing.assert_array_equal(q[0],a);np.testing.assert_array_equal(q[-1],b)
    assert np.max(np.abs(np.diff(q,axis=0)))<=.03+1e-12
    assert len(q)==135 and path_length(q)==pytest.approx(np.linalg.norm(b-a))
    states,source=sample_path([a,b,a],.03)
    assert len(states)==269 and source[134]=={'segment_index':0,'fraction':1.}
    assert source[-1]=={'segment_index':1,'fraction':1.}
    assert len(sample_path([a],.03)[0])==1


@pytest.mark.parametrize('bad',[0.,-1.,float('nan'),float('inf'),True])
def test_bad_resolution_is_rejected(bad):
    with pytest.raises(PathError):interpolate([0]*5,[1]*5,bad)


@pytest.mark.parametrize('bad',[[0]*4,[0]*6,[True]*5,[0,0,0,0,float('nan')],['0']*5])
def test_bad_configurations_are_rejected(bad):
    with pytest.raises(PathError):interpolate(bad,[0]*5,.1)


def test_edge_checks_interior_and_reports_first_failure_not_just_endpoints():
    e=FakeEvaluator(lambda q:abs(q[0])<.1);v=ConfigurationValidity(e)
    a=[-.8,0,0,0,0];b=[.8,0,0,0,0]
    assert v.check(a)['valid'] and v.check(b)['valid']
    edge=v.edge(a,b,.05,stop_first=False,fresh=True)
    assert not edge['valid'] and edge['complete']
    assert edge['first_failure']['path_sample_index']>0
    assert edge['first_failure']['reasons']==['SEMANTIC_SELF_COLLISION']
    assert edge['clearance']['self']['minimum_m']==0.
    assert not edge['continuous_collision_proven']


def test_cache_is_exact_fresh_bypasses_and_out_of_limits_never_queries():
    e=FakeEvaluator();v=ConfigurationValidity(e)
    assert v.check([0]*5)['valid'];v.check([0]*5)
    assert e.calls==1 and v.statistics()['cache_hits']==1
    v.check([1e-14,0,0,0,0]);v.check([0]*5,fresh=True)
    assert e.calls==3
    result=v.check([1.001,0,0,0,0])
    assert not result['valid'] and result['reasons']==['URDF_JOINT_LIMIT'] and e.calls==3


@pytest.mark.parametrize('option,reason',[('unknown','CLEARANCE_OR_COLLISION_UNVERIFIED'),('error','COLLISION_QUERY_ERROR'),('clearance','CLEARANCE_REQUIREMENT')])
def test_unknown_errors_and_nonzero_clearance_fail_closed(option,reason):
    checker=ConfigurationValidity(FakeEvaluator(**{option:True}))
    record=checker.check([0]*5)
    assert not record['valid'] and reason in record['reasons']


def test_missing_semantic_and_optimistic_result_are_rejected():
    e=FakeEvaluator();e.semantic=None
    with pytest.raises(PathError):ConfigurationValidity(e)
    e=FakeEvaluator();old=e.evaluate
    def optimistic(q):
        rows=old(q);rows[0].update(model_configuration_valid=True,semantic_self_collision=True);return rows
    e.evaluate=optimistic
    assert not ConfigurationValidity(e).check([0]*5)['valid']


def test_rrt_connect_deterministic_detour_and_full_independent_recheck():
    # Cylinder in q1/q2: a straight line is blocked; a path around it exists.
    blocked=lambda q:q[0]**2+q[1]**2<.30**2
    a=[-.8,0,0,0,0];b=[.8,0,0,0,0]
    outputs=[]
    for _ in range(2):
        v=ConfigurationValidity(FakeEvaluator(blocked))
        assert not v.edge(a,b,.03)['valid']
        result=rrt_connect(a,b,FakeModel.limits,v,seed=17,step_rad=.2,resolution_rad=.03,max_iterations=500,max_time_s=10.)
        assert result['search_found'] and not result['fully_revalidated']
        assert result['waypoints'][0]==a and result['waypoints'][-1]==b
        full=v.path(result['waypoints'],.01,fresh=True)
        assert full['valid'] and full['query_statistics']['cache_hits']==0
        assert full['query_statistics']['native_collision_queries']==full['checked_samples']
        assert path_length(result['waypoints'])>1.6
        outputs.append(result['waypoints'])
    assert outputs[0]==outputs[1]


def test_rrt_invalid_endpoint_iteration_budget_and_time_budget():
    v=ConfigurationValidity(FakeEvaluator(lambda q:q[0]<0))
    assert rrt_connect([-.8,0,0,0,0],[.8,0,0,0,0],FakeModel.limits,v)['status']=='INVALID_START'
    assert rrt_connect([.8,0,0,0,0],[-.8,0,0,0,0],FakeModel.limits,v)['status']=='INVALID_GOAL'
    # Complete separating slab: no path; budget exhaustion is not an unreachable proof.
    v=ConfigurationValidity(FakeEvaluator(lambda q:abs(q[0])<.1))
    r=rrt_connect([-.8,0,0,0,0],[.8,0,0,0,0],FakeModel.limits,v,max_iterations=2,max_time_s=10.)
    assert r['status']=='ITERATION_BUDGET_EXHAUSTED' and r['waypoints'] is None
    r=rrt_connect([-.8,0,0,0,0],[.8,0,0,0,0],FakeModel.limits,v,max_time_s=1e-12)
    assert r['status']=='TIME_BUDGET_EXHAUSTED'


@pytest.mark.parametrize('change',[{'verification_resolution_rad':.1},{'step_rad':0.},{'max_iterations':True},{'random_seed':-1},{'self_clearance_m':-.1},{'environment_clearance_m':float('nan')},{'max_time_s':float('inf')},{'goal_bias':2.},{'rrt_mode':'magic'},{'undocumented':1}])
def test_settings_reject_unsafe_or_undeclared_values(change):
    with pytest.raises(PathError):settings_checked(change)


def test_real_native_semantics_limits_clearance_environment_and_full_mesh_replay(tmp_path):
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.task.spec import load_task
    from workspace_analysis.task.targets import build_targets,tcp_errors
    from workspace_analysis.collision.semantic import SemanticConfig
    from workspace_analysis.collision.evaluator import Evaluator
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.joint_path import build_path_figure,export_path_viewer
    model=load_model(ROOT/'config/workspace.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    target=build_targets(load_task(ROOT/'config/tasks/p22_research_v0.yaml'),scene)
    goals=json.loads((ROOT/'results/task_reachability/p22_20261104_02/ik_results.json').read_text())['unique_task_solutions']
    q=goals[0]['q'];candidate=SemanticConfig.load(ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    strict=SemanticConfig.load(ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf',model)
    with Evaluator(model,scene,None,os.environ['C1_BACKEND'],semantic=candidate) as e:
        v=ConfigurationValidity(e);a=v.check(q)
        assert a['valid'] and not a['semantic_self_collision']
        bq=np.array(q);bq[0]+=.03
        path=v.path([q,bq],.01,fresh=True)
        assert path['complete'] and len(path['states'])>=4
        from workspace_analysis.workspace.results import load_results
        arrays,_=load_results(ROOT/'results/workspace_n100_seed42',model=model)
        assert 'ROBOT_ENVIRONMENT_COLLISION' in v.check(arrays['q'][8])['reasons']
        assert 'SEMANTIC_SELF_COLLISION' in v.check(arrays['q'][19])['reasons']
        v.check(q,fresh=True);native=deepcopy(e.last_native);native.update(q=q,sample_index='path_test')
        surfaces=robot_surfaces(model,e.ready,native)
        assessment=assess_task(tcp_errors(np.asarray(native['T_base_to_ee'])@target.spec.T_ee_tcp,target),target)
        figure,local=build_path_figure(scene,surfaces,native,target,assessment,path['states'],'test')
        assert len(local)==24 and figure.layout.meta['all_robot_meshes_replayed']==24
        # Native second-state transforms reconstruct exactly the renderer's meshes.
        v.check(bq,fresh=True);expected=robot_surfaces(model,e.ready,e.last_native)
        for surface,real in zip(local,expected):
            t=np.asarray(path['states'][-1]['link_transforms_base'][surface['link']])
            np.testing.assert_allclose(np.asarray(surface['vertices'])@t[:3,:3].T+t[:3,3],real['vertices'],atol=1e-12,rtol=0)
        assert any(not np.array_equal(path['states'][0]['link_transforms_base'][k],path['states'][-1]['link_transforms_base'][k]) for k in path['states'][0]['link_transforms_base'])
        file=export_path_viewer(figure,local,path['states'],target,tmp_path/'path.html',label='test')
        text=file.read_text();assert 'window.pathViewer' in text and 'Plotly.react' in text and 'Actual raw FCL contacts' in text
        with pytest.raises(ValueError):export_path_viewer(figure,local,path['states'],target,file,label='test')
    assert e.client.cleanup['exit_normal']
    with Evaluator(model,scene,None,os.environ['C1_BACKEND'],semantic=strict) as e:
        assert not ConfigurationValidity(e).check(q)['valid']
    with Evaluator(model,scene,None,os.environ['C1_BACKEND'],semantic=candidate,clearance={'self_m':.002,'environment_m':0.}) as e:
        row=ConfigurationValidity(e).check(q)
        assert not row['valid'] and row['reasons']==['CLEARANCE_REQUIREMENT']


def test_t3_cli_help_and_bad_settings_from_other_cwd_do_not_write(tmp_path):
    cli=ROOT/'scripts/evaluate_p22_path.py'
    r=subprocess.run([sys.executable,'-B',str(cli),'--help'],cwd=tmp_path,capture_output=True,text=True)
    assert r.returncode==0 and '--verification-resolution-rad' in r.stdout
    r=subprocess.run([sys.executable,'-B',str(cli),'--backend','unused','--allow-candidate-baseline','--edge-resolution-rad','nan'],cwd=tmp_path,capture_output=True,text=True)
    assert r.returncode==2 and 'finite and positive' in r.stderr and not list(tmp_path.iterdir())


def test_t3_refuses_overwrite_and_requires_explicit_candidate_before_work(tmp_path):
    from workspace_analysis.experiments.p22_path_planning import run_p22_path
    with pytest.raises(PathError,match='existing'):run_p22_path(ROOT,'unused','unused','unused',tmp_path)
    with pytest.raises(PathError,match='explicit'):run_p22_path(ROOT,'unused','unused','unused',tmp_path/'never')
    assert not list(tmp_path.iterdir())
