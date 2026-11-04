"""T2 bounded numerical IK, independent thresholds, dedup and collision hierarchy."""
from dataclasses import replace
from pathlib import Path
import os
import numpy as np
import pytest
from workspace_analysis.robot.model import load_model
from workspace_analysis.robot.fk import ForwardKinematics
from workspace_analysis.task.spec import load_task,parse_task,TaskError
from workspace_analysis.task.targets import build_targets,tcp_errors
from workspace_analysis.geometry.se3 import Pose,matrix_to_quaternion
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.planning.ik import NumericalIK,build_seeds
from workspace_analysis.planning.validity import assess_task,screen_solutions
from workspace_analysis.collision.semantic import SemanticConfig
from workspace_analysis.workspace.results import load_results
ROOT=Path(__file__).resolve().parents[1]


def setup():
    model=load_model(ROOT/'config/workspace.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');spec=load_task(ROOT/'config/tasks/p22_research_v0.yaml')
    return model,scene,build_targets(spec,scene)


def known_fk_target(model,targets,q):
    t=ForwardKinematics(model)(q)@targets.spec.T_ee_tcp
    pose=Pose(tuple(t[:3,3]),tuple(matrix_to_quaternion(t[:3,:3])))
    tcp=targets.spec.data['tcp'];return replace(targets,pre_tcp=pose,insertion_axis_base=tuple(t[:3,:3]@np.array(tcp['insertion_axis_local'])),key_axis_base=tuple(t[:3,:3]@np.array(tcp['key_axis_local'])))


@pytest.mark.parametrize('position,axis,key,expected',[(.01,0,0,'IK_FAILED'),(.0005,20,0,'POSITION_REACHABLE'),(.0005,1,30,'AXIS_ALIGNED'),(.0005,1,2,'TASK_POSE_FEASIBLE')])
def test_threshold_hierarchy_is_cumulative_not_optimizer_success(position,axis,key,expected):
    _,_,target=setup();r=assess_task({'position_m':position,'insertion_axis_rad':np.deg2rad(axis).item(),'key_axis_rad':np.deg2rad(key).item()},target)
    assert r['status']==expected and not r['task_configuration_candidate']
    if r['axis_aligned']:assert r['position_reachable']
    if r['task_pose_feasible']:assert r['axis_aligned']


def test_staged_multistart_recovers_known_fk_pose_and_deduplicates():
    model,_,target=setup();q=np.array([.2,-.5,1.,.3,-.2]);target=known_fk_target(model,target,q)
    result=NumericalIK(model,target).solve([{'q':q.tolist(),'seed_id':0},{'q':q.tolist(),'seed_id':1}])
    assert result['optimization_count']==6 and len(result['unique_task_solutions'])==1
    assert len(result['unique_task_solutions'][0]['attempt_ids'])==6
    assert result['best_attempt']['task_check']['task_pose_feasible']
    for attempt in result['attempts']:
        model.validate_q(attempt['q']);assert attempt['residual_calls_including_numeric_jacobian']>=attempt['nfev']
        assert attempt['iterations'] is None and not attempt['iteration_count_available']


def test_unreachable_target_returns_best_approximation_without_false_success():
    model,_,target=setup();d=target.spec.data;d['solver']['max_nfev']=30;target=replace(target,spec=parse_task(d),pre_tcp=Pose((3.,0.,0.),(0.,0.,0.,1.)))
    result=NumericalIK(model,target).solve([{'q':[0]*5}])
    assert not result['unique_task_solutions'] and result['best_attempt']['task_check']['status']=='IK_FAILED'
    assert result['best_attempt']['errors']['position_m']>1 and result['no_solution_reason']
    assert not result['mathematical_unreachability_proven']


def test_seeds_use_tcp_positions_and_record_original_g4_indices_deterministically():
    model,_,target=setup();arrays,_=load_results(ROOT/'results/g4_run01/experiments/workspace_n1000_seed42',model=model)
    a=build_seeds(model,target,arrays);b=build_seeds(model,target,arrays)
    assert a==b and len(a)==41
    selected=[s for s in a if s['source']=='G4_nearest_virtual_TCP']
    assert len(selected)==16 and all(s['q']==arrays['q'][s['workspace_sample_index']].tolist() for s in selected)
    assert [s['initial_tcp_distance_m'] for s in selected]==sorted(s['initial_tcp_distance_m'] for s in selected)


def test_antipodal_key_residual_is_nonzero():
    model,_,target=setup();q=np.zeros(5);target=known_fk_target(model,target,q)
    target=replace(target,key_axis_base=tuple(-np.array(target.key_axis_base)))
    assert np.linalg.norm(NumericalIK(model,target).residual(q,'task'))>1
    assert tcp_errors(ForwardKinematics(model)(q)@target.spec.T_ee_tcp,target)['key_axis_deg']==pytest.approx(180)


def test_task_pose_and_collision_are_separate_and_legacy_decision_is_ignored():
    _,_,target=setup();errors={'position_m':0.,'insertion_axis_rad':0.,'key_axis_rad':0.}
    collision={'model_configuration_valid':True,'semantic_self_collision':False,'environment_collision':False,'final_decision':'COLLIDING',
        'configuration_decision':{'planning_approved':False,'hardware_validated':False}}
    result=assess_task(errors,target,collision=collision)
    assert result['status']=='TASK_CONFIGURATION_CANDIDATE' and not result['planning_approved']
    collision.update(model_configuration_valid=False,environment_collision=True)
    result=assess_task(errors,target,collision=collision)
    assert result['status']=='TASK_POSE_FEASIBLE' and result['collision_status']=='REJECTED' and not result['task_configuration_candidate']
    with pytest.raises(TaskError):assess_task({**errors,'position_m':1.},target,collision=collision)


def test_no_pose_feasible_solutions_means_no_backend_is_started():
    model,scene,target=setup();d=target.spec.data;d['approach']['distance_m']=3.;target=build_targets(parse_task(d),scene)
    sem=SemanticConfig.load(ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    result=screen_solutions(model,scene,target,[{'q':[0]*5}],sem,'/not/a/backend')
    assert result['collision_queries']==0 and result['cleanup'] is None
    assert result['results'][0]['task_check']['collision_status']=='NOT_RUN_TASK_POSE_FAILED'


def test_optimizer_success_alone_never_overrides_task_errors(monkeypatch):
    import workspace_analysis.planning.ik as module
    from types import SimpleNamespace
    model,_,target=setup();target=replace(target,pre_tcp=Pose((3.,0.,0.),(0.,0.,0.,1.)))
    def stationary(fun,x,**kwargs):
        residual=fun(x)
        return SimpleNamespace(x=x,success=True,status=1,message='synthetic stationary success',nfev=1,njev=0,cost=float(residual@residual/2),optimality=0.,active_mask=np.zeros(5,dtype=int))
    monkeypatch.setattr(module,'least_squares',stationary)
    result=module.NumericalIK(model,target).solve([{'q':[0]*5}])
    assert all(a['optimizer_success'] for a in result['attempts'])
    assert not result['unique_task_solutions'] and result['best_attempt']['task_check']['status']=='IK_FAILED'


def test_p22_real_native_screening_strict_vs_candidate_and_environment_positive(tmp_path):
    import json
    from workspace_analysis.scene.schema import parse_scene
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.viz.task import build_task_figure
    from workspace_analysis.viz.collision import export_collision_viewer
    model,scene,target=setup();result=NumericalIK(model,target).solve([{'q':[0]*5}]);solutions=result['unique_task_solutions']
    assert solutions,'P22 virtual task must be solved by the real numerical solver in this regression'
    strict=SemanticConfig.load(ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf',model)
    candidate=SemanticConfig.load(ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    a=screen_solutions(model,scene,target,solutions,strict,os.environ['C1_BACKEND']);b=screen_solutions(model,scene,target,solutions,candidate,os.environ['C1_BACKEND'])
    assert a['results'][0]['task_check']['task_pose_feasible'] and not a['results'][0]['task_check']['task_configuration_candidate']
    assert b['results'][0]['task_check']['status']=='TASK_CONFIGURATION_CANDIDATE'
    assert a['cleanup']['exit_normal'] and b['cleanup']['exit_normal']
    # Test-only detached obstacle; production S1 and P22 files remain unchanged.
    data=scene.to_dict();data['scene']['collision_objects'].append({'id':'task_test_block','type':'box','parent_frame':'base_link','pose':{'position_m':list(target.pre_tcp.position_m),'quaternion_xyzw':[0,0,0,1]},'size_m':[.08,.08,.08]})
    blocked=parse_scene(json.dumps(data));new_target=build_targets(target.spec,blocked)
    c=screen_solutions(model,blocked,new_target,solutions,candidate,os.environ['C1_BACKEND'])
    assert c['results'][0]['task_check']['task_pose_feasible'] and c['results'][0]['collision']['environment_collision']
    assert c['results'][0]['task_check']['collision_status']=='REJECTED'
    with CollisionClient(os.environ['C1_BACKEND']) as client:
        ready=client.initialize(model,scene,semantic=candidate);row=client.query(model,solutions[0]['q'],auxiliary_positions={'r_joint':.3})
        row.update(q=solutions[0]['q'],sample_index='test_P22',task_check=b['results'][0]['task_check'])
        fig=build_task_figure(scene,robot_surfaces(model,ready,row),row,target,row['task_check'])
    assert len([t for t in fig.data if (t.meta or {}).get('role')=='robot_collision_mesh'])==24
    assert fig.layout.meta['task_targets']['task_sha256']==target.spec.sha256
    assert not fig.layout.meta['tcp_geometry_modeled']
    path=export_collision_viewer(fig,row,tmp_path/'task.html')
    assert 'P22 task:' in path.read_text() and 'Pre-insertion target TCP' in path.read_text()


def test_complete_failure_run_keeps_best_viewer_and_skips_task_collision_screening(tmp_path):
    import yaml
    from workspace_analysis.experiments.p22_reachability import run_p22,verify_p22
    model,_,target=setup();data=target.spec.data
    data['solver'].update(workspace_nearest_seeds=0,extra_joint_seeds=0,include_midpoint_seed=True,max_nfev=1)
    path=tmp_path/'budget_limited_task.yaml';path.write_text(yaml.safe_dump(data,sort_keys=False));out=tmp_path/'failure_run'
    summary=run_p22(ROOT,os.environ['C1_BACKEND'],path,ROOT/'results/g4_run01/experiments/workspace_n1000_seed42',out,allow_candidate=True)
    assert summary['final_status']=='IK_FAILED' and summary['unique_task_pose_solutions']==0
    assert summary['collision_comparison']['candidate']['task_pose_solutions_checked']==0
    assert summary['collision_comparison']['candidate']['task_configuration_candidates']==0
    assert summary['no_solution_reason'] and not summary['mathematical_unreachability_proven']
    assert (out/'figures/best_approximation.html').is_file()
    assert verify_p22(out,model)['all_task_errors_rechecked']
    with pytest.raises(TaskError,match='existing'):run_p22(ROOT,os.environ['C1_BACKEND'],path,ROOT/'results/g4_run01/experiments/workspace_n1000_seed42',out,allow_candidate=True)
