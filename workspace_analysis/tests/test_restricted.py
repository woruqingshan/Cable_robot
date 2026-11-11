"""Conditional boundaries, deterministic obstacle identity and unified validity."""
from pathlib import Path
import json
import numpy as np
import pytest
from workspace_analysis.planning.workspace_constraint import WorkspaceConstraint,TCPWorkspaceCheck
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.scene.restricted_scene import restricted_scene,geometry_audit
from workspace_analysis.scene.base_pose import BasePose,derive_scene,verify_derivation
ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('xyz,valid',[
    ([.249,10,-10],True),([.250,-.15,.04],True),([.250,.15,.4],True),
    ([.250,.150001,.22],False),([.3,0,.03999],False),([.3,0,.40001],False),
    ([.3,-.15001,.22],False)])
def test_conditional_inclusive_boundaries(xyz,valid):
    result=WorkspaceConstraint().assess(xyz)
    assert result['valid']==valid
    assert result['active']==(xyz[0]>=.25)
    if not valid:assert result['violations'] and result['reasons']


@pytest.mark.parametrize('bad',[[np.nan,0,0],[np.inf,0,0],[0,0],['1',0,0],[True,False,True]])
def test_workspace_bad_positions_rejected(bad):
    with pytest.raises(ValueError):WorkspaceConstraint().assess(bad)


@pytest.mark.parametrize('kwargs',[{'enabled':1},{'x_trigger_m':float('nan')},{'y_min_m':.2},{'switch_resolution_rad':0}])
def test_invalid_workspace_config(kwargs):
    with pytest.raises(ValueError):WorkspaceConstraint(**kwargs)


@pytest.mark.parametrize('pose',[BasePose('b0'),BasePose('xm30',-.03),BasePose('ym70',0,-.07)])
def test_world_tcp_in_each_station(pose):
    t=np.eye(4);t[:3,3]=[.3,.14,.22]
    mount=np.eye(4);mount[2,3]=.03
    rule=TCPWorkspaceCheck(WorkspaceConstraint(),pose.matrix(),mount,pose.to_dict())
    np.testing.assert_allclose(rule.assess(t)['world_tcp_xyz_m'],(pose.matrix()@t@mount)[:3,3],atol=1e-12)
    assert WorkspaceConstraint(enabled=False).assess([.3,10,-10])['valid']


@pytest.mark.parametrize('label,count,ports',[('D0',38,[]),('D1',38,[]),('D2',39,['P22']),('D3',40,['P21','P23']),('D4',42,['P12','P21','P23','P32'])])
def test_scene_exact_objects_and_world_invariance(label,count,ports):
    world=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');original=world.to_json()
    config=json.loads((ROOT/'config/restricted_scenes/d0_d4_v0.json').read_text())
    scene=restricted_scene(world,label,config)
    assert len(scene.collision_objects)==count and world.to_json()==original
    assert scene.canonical_scene_sha256==restricted_scene(world,label,config).canonical_scene_sha256
    assert scene.canonical_scene_sha256!=world.canonical_scene_sha256
    assert geometry_audit(world,scene,config['scenes'][label]['target'])['status']=='PASS'
    for port in ports:
        obj=next(o for o in scene.collision_objects if o['id']=='restricted_'+label+'_'+port)
        assert tuple(obj['size_m'])==(.035,.03,.024)
        assert obj['pose']['position_m'][0]==pytest.approx(.3125)
    pose=BasePose('shift',-.03,-.07,.3)
    assert verify_derivation(scene,derive_scene(scene,pose),pose)['world_cabinet_fixed']


def test_composed_rule_cache_fresh_and_switch_refinement():
    from test_task_benchmark import fixture_context
    from workspace_analysis.planning.validity import ConfigurationValidity
    context,_=fixture_context()
    rule=TCPWorkspaceCheck(WorkspaceConstraint(z_min_m=.3),np.eye(4),np.eye(4),BasePose('b0').to_dict())
    checker=ConfigurationValidity(context.checker.evaluator,workspace_check=rule)
    state=checker.check([0]*5)
    assert state['fcl_valid'] and not state['valid'] and 'WORKSPACE_Z_BOUNDARY' in state['reasons']
    assert checker.check([0]*5)==state and checker.statistics()['cache_hits']==1
    assert checker.check([0]*5,fresh=True)['query_id']==1
    off=ConfigurationValidity(context.checker.evaluator,workspace_check=TCPWorkspaceCheck(WorkspaceConstraint(enabled=False),np.eye(4),np.eye(4),{}))
    assert off.check([0]*5)['valid']
    def fk(q):
        t=np.eye(4);t[0,3]=.25+q[0];return t
    states=np.array([[-.005,0,0,0,0],[.005,0,0,0,0]])
    expanded,origins=rule.refine(states,[{'segment_index':0,'fraction':0.},{'segment_index':0,'fraction':1.}],fk)
    assert len(expanded)>2 and len(origins)==len(expanded)
    assert np.max(np.abs(np.diff(expanded,axis=0)))<=.001+1e-12


def test_restriction_marker_world_geometry_and_disabled_boundary():
    from jetrover_idc_visualization.markers import restriction_specs
    pose=BasePose('ym70',0,-.07)
    info={'world_from_base':pose.matrix().tolist(),'workspace_config':WorkspaceConstraint().to_dict()}
    specs=restriction_specs(info)
    points=np.asarray(specs[0].points)
    world=points@pose.matrix()[:3,:3].T+pose.matrix()[:3,3]
    assert world[:,0].min()==pytest.approx(.25)
    assert world[:,1].min()==pytest.approx(-.15)
    assert world[:,2].min()==pytest.approx(.04)
    info['workspace_config']['enabled']=False
    assert restriction_specs(info)==()


def test_workspace_invalid_goal_never_enters_rrt_and_approach_uses_same_rule(monkeypatch):
    from types import SimpleNamespace
    from test_task_benchmark import fixture_context
    from workspace_analysis.planning.validity import ConfigurationValidity
    from workspace_analysis.experiments.task_context import TaskContext
    from workspace_analysis.experiments.task_path_planning import run_task_path
    import workspace_analysis.experiments.task_path_planning as path_module
    from workspace_analysis.planning.cartesian_approach import sequential_approach
    original,_=fixture_context()
    rule=TCPWorkspaceCheck(WorkspaceConstraint(z_min_m=.3),np.eye(4),original.targets.spec.T_ee_tcp,{})
    checker=ConfigurationValidity(original.checker.evaluator,workspace_check=rule)
    context=TaskContext(original.model,original.scene,original.targets,checker)
    def forbidden(*args,**kwargs):
        raise AssertionError('invalid endpoint must not enter RRT search')
    monkeypatch.setattr(path_module,'rrt_connect',forbidden)
    trial=run_task_path(context,[-1,0,0,0,0],[0]*5,method='rrt_connect')
    assert trial['status']=='INVALID_GOAL' and trial['search'] is None
    assert 'WORKSPACE_Z_BOUNDARY' in checker.queries[-1]['reasons']
    solver=lambda *args:SimpleNamespace(solve=lambda seeds:{'unique_task_solutions':[],
        'best_attempt':None,'attempts':[],'optimization_count':0,'total_elapsed_s':0.})
    approach=sequential_approach(original.model,original.targets,[0]*5,checker,ik_factory=solver)
    assert not approach['success']
    assert any('WORKSPACE_Z_BOUNDARY' in state['reasons'] for state in checker.queries)
