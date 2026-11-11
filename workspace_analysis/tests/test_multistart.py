"""Ready-pose rules, local-only composition and discrete path quality."""
from pathlib import Path
import json
from types import SimpleNamespace
import numpy as np
import pytest
from workspace_analysis.planning.local_workspace import LocalOnlyWorkspaceConstraint, constraint_from_config
from workspace_analysis.planning.workspace_constraint import WorkspaceConstraint, TCPWorkspaceCheck
from workspace_analysis.planning.path_quality import tcp_quality, path_quality
from workspace_analysis.experiments.ready_pose import eligible_ready, select_ready, ready_digest
ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('xyz,valid',[
    ([.249,0,.22],False),([.250,-.150,.040],True),([.250,.150,.400],True),
    ([.3,.150001,.22],False),([.3,0,.039999],False),([.3,0,.400001],False),
    ([.249,1,-1],False)])
def test_local_only_boundaries_and_original_semantics(xyz,valid):
    local=LocalOnlyWorkspaceConstraint();old=WorkspaceConstraint()
    assert local.assess(xyz)['valid']==valid
    if xyz[0]<.250:assert old.assess(xyz)['valid']
    assert constraint_from_config(local.to_dict()).sha256==local.sha256
    assert local.sha256!=old.sha256
    assert old.to_dict().get('mode') is None


@pytest.mark.parametrize('point',[[np.nan,0,0],[np.inf,0,0],[0,0],['x',0,0],[True,False,True]])
def test_local_only_rejects_bad_points(point):
    with pytest.raises(ValueError):LocalOnlyWorkspaceConstraint().assess(point)


def test_local_off_and_unknown_mode():
    assert LocalOnlyWorkspaceConstraint(enabled=False).assess([-.1,20,-20])['valid']
    with pytest.raises(ValueError):constraint_from_config({'mode':'typo'})


def test_quality_separates_waypoints_samples_exit_and_undefined_ratio():
    p=[[.26,0,.2],[.24,.01,.2],[.29,0,.2]]
    report=tcp_quality(p)
    assert report['workspace_exit'] and report['tcp_detour_ratio']>1
    assert tcp_quality([p[0],p[0]])['tcp_detour_ratio'] is None
    with pytest.raises(ValueError):tcp_quality([[np.nan,0,0]])
    trial={'success':True,'waypoints':[[0]*5,[1,0,0,0,0]],
        'validation':{'states':[{}, {}, {}],'clearance':{'self':{'minimum_m':0.}}}}
    result=path_quality(trial,p)
    assert result['raw_waypoint_count']==2 and result['validated_sample_count']==3
    assert result['joint_path_length_rad']==1.
    assert path_quality({'validation':None},[])['status']=='NO_RETURNED_PATH'


def rules():
    c=json.loads((ROOT/'config/restricted_scenes/ready_pose_v0.json').read_text())
    return {**c,'insertion_axis_world':[1,0,0],'pre_distance_m':.04}


def test_ready_region_and_non_goal_rule():
    limit=np.array([[-2,2]]*5);q=np.array([0,0,1,0,0.]);goals=np.array([[0,1,0,0,0.]])
    assert eligible_ready(q,[.21,0,.22],limit,[.33,0,.22],goals,'S1',rules())[0]
    assert eligible_ready(q,[.26,0,.22],limit,[.33,0,.22],goals,'S2',rules())[0]
    assert not eligible_ready(goals[0],[.26,0,.22],limit,[.33,0,.22],goals,'S2',rules())[0]
    assert not eligible_ready(q,[.29,0,.22],limit,[.33,0,.22],goals,'S2',rules())[0]
    assert not eligible_ready([0,0,1.95,0,0],[.21,0,.22],limit,[.33,0,.22],goals,'S1',rules())[0]
    with pytest.raises(ValueError):eligible_ready(q,[np.nan,0,.22],limit,[.33,0,.22],goals,'S1',rules())


def test_ready_selection_is_deterministic_before_planning_and_exhaustion_is_explicit():
    model=SimpleNamespace(limits=np.array([[-2,2]]*5))
    def fk(q):
        t=np.eye(4);t[:3,3]=[.21,0,.22];return t
    class Checker:
        def check(self,q,**kwargs):
            return {'valid':True,'status':'MODEL_PASS_RESEARCH_ONLY','reasons':[],
                'semantic_clearance':{'native':{'environment':{'available':True,'separation_m':.1}}}}
    pools=[('original',{'q':np.array([[.1,0,0,0,0],[.2,0,0,0,0]])},{'npz_sha256':'test'})]
    args=(model,fk,Checker(),pools,np.eye(4),SimpleNamespace(T_ee_tcp=np.eye(4)),np.array([.33,0,.22]),[[1]*5],rules(),'S1')
    a=select_ready(*args);b=select_ready(*args)
    assert a==b and a['selected']['original_sample_index']==0
    assert a['planner_results_used_for_selection'] is False
    empty=select_ready(*args[:-1],'S2')
    assert empty['status']=='NO_QUALIFIED_READY_POSE' and empty['selected'] is None
    assert ready_digest({'q':[1,2]})==ready_digest({'q':[1,2]})


def test_local_composition_uses_same_search_and_fresh_validation(monkeypatch):
    from test_task_benchmark import fixture_context
    from workspace_analysis.planning.validity import ConfigurationValidity
    from workspace_analysis.experiments.task_context import TaskContext
    from workspace_analysis.experiments.task_path_planning import run_task_path
    original,_=fixture_context()
    # Toy FK uses x=.3+q0*.05; start x=.25 and goal x=.30.
    rule=TCPWorkspaceCheck(LocalOnlyWorkspaceConstraint(),np.eye(4),np.eye(4),{})
    checker=ConfigurationValidity(original.checker.evaluator,workspace_check=rule)
    bad=checker.check([-2,0,0,0,0])
    assert not bad['valid'] and 'WORKSPACE_X_BOUNDARY' in bad['reasons']
    assert checker.check([-2,0,0,0,0])==bad
    fresh=checker.check([-2,0,0,0,0],fresh=True)
    assert fresh['query_id']!=bad['query_id']
    context=TaskContext(original.model,original.scene,original.targets,checker)
    for method in ('linear','rrt_connect'):
        trial=run_task_path(context,[-2,0,0,0,0],[0]*5,method=method)
        assert trial['status']=='INVALID_START' and trial['search'] is None
    for method in ('linear','rrt_connect'):
        trial=run_task_path(context,[0]*5,[.1,0,0,0,0],method=method)
        assert trial['success'] and all(s['workspace']['valid'] for s in trial['validation']['states'])
    def crossing_fk(q):
        t=np.eye(4);t[:3,3]=[.25+q[0],0,.22];return t
    states=np.array([[-.005,0,0,0,0],[.005,0,0,0,0]])
    refined,provenance=rule.refine(states,[{'segment_index':0,'fraction':0.},{'segment_index':0,'fraction':1.}],crossing_fk)
    assert len(refined)>len(states) and len(refined)==len(provenance)
    assert np.max(np.abs(np.diff(refined,axis=0)))<=.001+1e-12


def test_ready_marker_identifies_actual_start_and_local_mode():
    from jetrover_idc_visualization.markers import restriction_specs
    info={'world_from_base':np.eye(4).tolist(),'workspace_config':LocalOnlyWorkspaceConstraint().to_dict(),
        'start_world_tcp_xyz_m':[.26,0,.22],'start_id':'S2','workspace_mode':'local_only'}
    markers=restriction_specs(info)
    assert any('LOCAL-ONLY' in m.text for m in markers)
    ready=next(m for m in markers if m.namespace=='ready_pose' and m.kind=='SPHERE')
    assert tuple(ready.position)==(.26,0,.22)


def test_cli_help_from_other_directory_and_no_overwrite(tmp_path):
    import subprocess
    import sys
    from workspace_analysis.experiments.multistart_benchmark import run_multistart
    result=subprocess.run([sys.executable,'-B',str(ROOT/'scripts/evaluate_multistart_benchmark.py'),'--help'],cwd=tmp_path,text=True,capture_output=True)
    assert result.returncode==0 and '--expected-source-manifest-sha256' in result.stdout
    out=tmp_path/'existing';out.mkdir()
    with pytest.raises(FileExistsError):run_multistart(ROOT,tmp_path/'missing',sys.executable,out,allow_candidate=True)
    fresh=tmp_path/'fresh'
    with pytest.raises(ValueError,match='opt-in'):run_multistart(ROOT,tmp_path/'missing',sys.executable,fresh)
    assert not fresh.exists()
