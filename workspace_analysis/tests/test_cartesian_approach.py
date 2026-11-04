"""T4 derived targets, continuation, interior failures and real native replay."""
from copy import deepcopy
from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene, parse_scene
from workspace_analysis.geometry.se3 import Pose, matrix_to_quaternion
from workspace_analysis.task.spec import load_task
from workspace_analysis.task.targets import build_targets, tcp_errors
from workspace_analysis.planning.cartesian_approach import (approach_settings, cartesian_waypoints,
    target_at_distance, check_approach_sample, sequential_approach)
from workspace_analysis.planning.validity import ConfigurationValidity, assess_task
from workspace_analysis.planning.joint_path import PathError

ROOT=Path(__file__).resolve().parents[1]


def inputs():
    model=load_model(ROOT/'config/workspace.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    return model,scene,build_targets(load_task(ROOT/'config/tasks/p22_research_v0.yaml'),scene)


def test_default_targets_and_finer_nondivisible_grid_preserve_task_and_scene():
    _,scene,targets=inputs();before=scene.to_json();sha=targets.spec.sha256
    points=cartesian_waypoints(targets)
    np.testing.assert_allclose([p['distance_to_port_m'] for p in points],[.04,.03,.02,.01],rtol=0,atol=1e-15)
    np.testing.assert_allclose([p['target'].pre_tcp.position_m for p in points],[[.29,0,.22],[.30,0,.22],[.31,0,.22],[.32,0,.22]],atol=1e-15,rtol=0)
    for point in points:
        assert point['target'].spec.sha256==sha
        np.testing.assert_array_equal(point['target'].T_base_pre_tcp[:3,:3],targets.T_base_pre_tcp[:3,:3])
        # Rigid inverse/composition incurs machine rounding; task tolerances stay unchanged.
        np.testing.assert_allclose(point['target'].T_base_pre_ee@targets.spec.T_ee_tcp,point['target'].T_base_pre_tcp,atol=1e-14,rtol=0)
    finer=cartesian_waypoints(targets,.01,.007)
    assert finer[0]['distance_to_port_m']==.04 and finer[-1]['distance_to_port_m']==.01
    assert np.max(-np.diff([p['distance_to_port_m'] for p in finer]))<=.007+1e-15
    assert scene.to_json()==before and targets.spec.sha256==sha


def test_target_generation_follows_rotated_port_not_hardcoded_world_x():
    _,_,targets=inputs();r=Rotation.from_euler('xyz',[.3,.4,1.1]).as_matrix();p=np.array([.1,.2,.3])
    pose=Pose(tuple(p),tuple(matrix_to_quaternion(r)))
    rotated=replace(targets,base_to_port=pose,port_tcp=pose,insertion_axis_base=tuple(r[:,0]),key_axis_base=tuple(r[:,2]))
    for point in cartesian_waypoints(rotated):
        np.testing.assert_allclose(point['target'].pre_tcp.position_m,p-point['distance_to_port_m']*r[:,0],atol=1e-15,rtol=0)
        np.testing.assert_allclose(point['target'].T_base_pre_tcp[:3,:3],r,atol=1e-15,rtol=0)


@pytest.mark.parametrize('change',[{'cartesian_step_m':0},{'edge_resolution_rad':float('nan')},{'end_distance_m':-.01},
    {'max_joint_delta_rad':False},{'backup_seeds':9},{'backup_seeds':True},{'backup_radius_rad':float('inf')},
    {'random_seed':-1},{'environment_clearance_m':-.1},{'undeclared_tolerance':1}])
def test_invalid_settings_are_rejected(change):
    with pytest.raises(PathError):approach_settings(change)


def test_generator_rejects_reversed_and_unbounded_grid():
    _,_,targets=inputs()
    with pytest.raises(PathError):cartesian_waypoints(targets,.05)
    with pytest.raises(PathError):cartesian_waypoints(targets,.01,1e-9)


class ToyModel:
    limits=np.array([[-2.,2.]]*5)
    def validate_q(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(5,) or not np.isfinite(q).all() or np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):raise ValueError('toy joint limit')
        return q.copy()


class ToyEvaluator:
    """Test oracle only; production continuation and validity code are unchanged."""
    semantic=object()
    def __init__(self,model,fk,blocked=lambda q:False):self.model=model;self.fk=fk;self.blocked=blocked;self.last_native=None
    def evaluate(self,values):
        q=values[0];blocked=bool(self.blocked(q));t=self.fk(q)
        self.last_native={'T_base_to_ee':t.tolist(),'link_transforms_base':{'base_link':np.eye(4).tolist()},'contacts_base':[],'joint_positions_used':{}}
        return [{'model_configuration_valid':not blocked,'semantic_self_collision':blocked,'environment_collision':False,
            'configuration_decision':{'planning_approved':False,'hardware_validated':False,'status':'RESEARCH_ONLY'},
            'semantic_blocking_pairs':[['a','b']] if blocked else [],'raw_collision_pairs':[],'query_time':0.,
            'semantic_clearance':{'native':{'self':{'available':True,'separation_m':0. if blocked else .01},'environment':{'available':True,'separation_m':.02}}}}]


def toy(targets,kind=None,ik_mode=None):
    model=ToyModel();tool_inv=np.linalg.inv(targets.spec.T_ee_tcp)
    def fk(q):
        wave=np.sin(np.pi*(q[0]-.29)/.01);tcp=np.eye(4);tcp[:3,3]=[q[0],0.,.22]
        if kind=='cross':tcp[1,3]+=.002*wave
        if kind=='along':tcp[0,3]+=.004*wave
        if kind=='axis':tcp[:3,:3]=Rotation.from_euler('y',.10*wave).as_matrix()
        return tcp@tool_inv
    class Solver:
        def __init__(self,model,target):self.target=target
        def solve(self,seeds):
            q=np.array([self.target.pre_tcp.position_m[0],0,0,0,0]);fail=ik_mode=='fail' and q[0]>.295
            if ik_mode=='jump' and q[0]>.295:q[1]=1.
            return {'unique_task_solutions':[] if fail else [{'q':q.tolist()}],'best_attempt':{'q':seeds[0]['q'],'no_solution':fail},
                    'total_elapsed_s':0.,'optimization_count':1,'attempts':[{'seed':deepcopy(seeds[0])}]}
    evaluator=ToyEvaluator(model,fk)
    evaluator.scene=SimpleNamespace(canonical_scene_sha256=targets.scene_sha256)
    evaluator.gripper=targets.spec.data['collision']['gripper_rad']
    evaluator.clearance={'self_m':0.,'environment_m':0.}
    return model,fk,Solver,ConfigurationValidity(evaluator)


def test_collision_session_must_match_declared_scene_gripper_and_clearance():
    _,_,targets=inputs();model,fk,solver,checker=toy(targets)
    with pytest.raises(PathError,match='clearance/gripper'):
        sequential_approach(model,targets,[.29,0,0,0,0],checker,{'self_clearance_m':.002},ik_factory=solver,fk=fk)
    checker.evaluator.scene=SimpleNamespace(canonical_scene_sha256='wrong')
    with pytest.raises(PathError,match='model/scene'):
        sequential_approach(model,targets,[.29,0,0,0,0],checker,ik_factory=solver,fk=fk)


@pytest.mark.parametrize('kind,reason',[('cross','TCP_POSITION_TOLERANCE'),('along','TCP_POSITION_TOLERANCE'),('axis','INSERTION_AXIS_TOLERANCE')])
def test_valid_endpoint_iks_do_not_hide_interior_cartesian_or_orientation_failure(kind,reason):
    _,_,targets=inputs();model,fk,solver,checker=toy(targets,kind)
    start=[.29,0,0,0,0]
    # Every declared Cartesian waypoint has correct FK in this adversarial model.
    for p in cartesian_waypoints(targets):
        q=[p['target'].pre_tcp.position_m[0],0,0,0,0]
        assert assess_task(tcp_errors(fk(q)@targets.spec.T_ee_tcp,p['target']),p['target'])['task_pose_feasible']
    result=sequential_approach(model,targets,start,checker,{'edge_resolution_rad':.001},ik_factory=solver,fk=fk)
    assert not result['success'] and result['accepted_waypoints']==1 and len(result['waypoints'])==2
    assert result['first_failure']['kind']=='CARTESIAN_TASK_CONSTRAINT' and reason in result['first_failure']['reasons']
    assert result['validated_progress_m']<.01
    if kind=='along':assert result['samples'][result['first_failed_sample_index']]['approach_check']['cross_track_error_m']<1e-12


def test_continuation_uses_previous_success_and_deterministic_bounded_backups():
    _,_,targets=inputs();outputs=[]
    for _ in range(2):
        model,fk,solver,checker=toy(targets,ik_mode='fail')
        result=sequential_approach(model,targets,[.29,0,0,0,0],checker,ik_factory=solver,fk=fk)
        assert result['first_failure']['kind']=='IK' and result['accepted_waypoints']==1
        assert len(result['waypoints'][1]['ik_runs'])==3
        seeds=[run['attempts'][0]['seed']['q'] for run in result['waypoints'][1]['ik_runs']]
        assert seeds[0]==[.29,0,0,0,0]
        assert all(np.max(np.abs(np.asarray(s)-seeds[0]))<=.03 for s in seeds)
        assert result['last_valid_expected_distance_m']==.04 and result['validated_progress_m']==0.
        outputs.append(seeds)
    assert outputs[0]==outputs[1]


def test_branch_jump_is_rejected_even_when_all_task_errors_pass():
    _,_,targets=inputs();model,fk,solver,checker=toy(targets,ik_mode='jump')
    result=sequential_approach(model,targets,[.29,0,0,0,0],checker,ik_factory=solver,fk=fk)
    assert result['first_failure']['kind']=='CONTINUITY' and result['accepted_waypoints']==1
    assert all(c['task_check']['task_pose_feasible'] and c['branch_jump_suspected'] for c in result['waypoints'][1]['candidates'])


def test_solver_exception_is_reported_with_bounded_retries_not_false_unreachable():
    _,_,targets=inputs();model,fk,solver,checker=toy(targets)
    class BrokenSolver(solver):
        def solve(self,seeds):
            if self.target.pre_tcp.position_m[0]>.295:raise RuntimeError('test numerical failure')
            return super().solve(seeds)
    result=sequential_approach(model,targets,[.29,0,0,0,0],checker,ik_factory=BrokenSolver,fk=fk)
    assert result['accepted_waypoints']==1 and result['first_failure']['reasons']==['IK_SOLVER_ERROR']
    assert len(result['waypoints'][1]['ik_runs'])==3 and not result['mathematical_unreachability_proven']


def test_toy_success_checks_all_samples_and_native_fk_mismatch_fails_closed():
    _,_,targets=inputs();model,fk,solver,checker=toy(targets)
    result=sequential_approach(model,targets,[.29,0,0,0,0],checker,{'edge_resolution_rad':.001},ik_factory=solver,fk=fk)
    assert result['success'] and result['accepted_waypoints']==4 and all(s['valid'] for s in result['samples'])
    assert result['last_valid_expected_distance_m']==.01 and result['validated_progress_m']==pytest.approx(.03)
    assert all(run['attempts'][0]['seed']['q']==point['initial_q'] for point in result['waypoints'] for run in point['ik_runs'])
    record=checker.check([.29,0,0,0,0],fresh=True);record['T_base_to_ee'][0][3]+=.001
    state=check_approach_sample(record,targets,.04,fk,waypoint_index=0,fraction=0.)
    assert not state['valid'] and 'NATIVE_KDL_TCP_MISMATCH' in state['reasons']


def test_real_both_branches_and_native_collision_failure_keep_complete_replay(tmp_path):
    from workspace_analysis.collision.evaluator import Evaluator
    from workspace_analysis.collision.semantic import SemanticConfig
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.approach import export_approach_viewer
    model,scene,targets=inputs();semantic=SemanticConfig.load(ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    goals=json.loads((ROOT/'results/task_reachability/p22_20261104_02/ik_results.json').read_text())['unique_task_solutions']
    with Evaluator(model,scene,None,os.environ['C1_BACKEND'],semantic=semantic) as e:
        checker=ConfigurationValidity(e)
        for goal in goals:
            result=sequential_approach(model,targets,goal['q'],checker)
            assert result['start_q']==goal['q'] and result['waypoints'][0]['selected']['q']==goal['q']
            if result['success']:
                assert result['accepted_waypoints']==4 and all(s['valid'] for s in result['samples'])
                assert result['max_position_error_mm']<=1 and result['max_axis_error_deg']<=3 and result['max_key_error_deg']<=5
            else:assert result['first_failure'] and not result['insertion_success']
            assert result['query_statistics']['cache_hits']==0 and not result['continuous_collision_proven']
    assert e.client.cleanup['exit_normal']
    # Detached test obstacle; official S1 is unchanged. Initial task pose passes,
    # but real environment collision must reject it and still produce a Viewer.
    data=scene.to_dict();data['scene']['collision_objects'].append({'id':'t4_test_block','type':'box','parent_frame':'base_link',
        'pose':{'position_m':list(targets.pre_tcp.position_m),'quaternion_xyzw':[0,0,0,1]},'size_m':[.08,.08,.08]})
    blocked=parse_scene(json.dumps(data));blocked_target=build_targets(targets.spec,blocked)
    with Evaluator(model,blocked,None,os.environ['C1_BACKEND'],semantic=semantic) as e:
        checker=ConfigurationValidity(e);result=sequential_approach(model,blocked_target,goals[0]['q'],checker)
        assert not result['success'] and result['accepted_waypoints']==0
        assert result['first_failure']['kind']=='COLLISION_OR_CLEARANCE'
        assert 'ROBOT_ENVIRONMENT_COLLISION' in result['first_failure']['reasons']
        checker.check(goals[0]['q'],fresh=True);native=deepcopy(e.last_native);native.update(q=goals[0]['q'],sample_index='blocked_T4')
        assessment=assess_task(tcp_errors(np.asarray(native['T_base_to_ee'])@targets.spec.T_ee_tcp,blocked_target),blocked_target)
        file=export_approach_viewer(blocked,robot_surfaces(model,e.ready,native),native,blocked_target,assessment,result,tmp_path/'blocked.html',label='blocked')
        text=file.read_text();assert 'First failed expected TCP' in text and 'all_samples_task_constrained' in text and 'window.pathViewer' in text
    with Evaluator(model,scene,None,os.environ['C1_BACKEND'],semantic=semantic,clearance={'self_m':.002,'environment_m':0.}) as e:
        result=sequential_approach(model,targets,goals[0]['q'],ConfigurationValidity(e),{'self_clearance_m':.002})
        assert result['accepted_waypoints']==0 and 'CLEARANCE_REQUIREMENT' in result['first_failure']['reasons']
        assert not result['samples'][0]['semantic_self_collision'] and not result['samples'][0]['environment_collision']


def test_complete_continuity_failure_experiment_is_reproducible_and_verifiable(tmp_path):
    from workspace_analysis.experiments.p22_approach import run_p22_approach,verify_approach
    model,_,_=inputs();out=tmp_path/'failure_run'
    args=(ROOT,os.environ['C1_BACKEND'],ROOT/'results/task_reachability/p22_20261104_02',ROOT/'results/task_planning/p22_20261104_02',out)
    result=run_p22_approach(*args,allow_candidate=True,settings={'max_joint_delta_rad':1e-6,'backup_seeds':0})
    assert all(not r['success'] and r['first_failure']['kind']=='CONTINUITY' and r['viewer'] for r in result['comparison'])
    assert verify_approach(out,model)['branches']==2
    with pytest.raises(PathError,match='existing'):run_p22_approach(*args,allow_candidate=True)
    with pytest.raises(PathError,match='explicit'):run_p22_approach(*args[:-1],tmp_path/'never')
    summary=out/'summary.json';summary.write_text(summary.read_text()+'\n')
    with pytest.raises(PathError,match='artifact changed'):verify_approach(out,model)


def test_cli_from_other_cwd_and_near_port_scope_rejection_do_not_write(tmp_path):
    cli=ROOT/'scripts/evaluate_p22_approach.py'
    r=subprocess.run([sys.executable,'-B',str(cli),'--help'],cwd=tmp_path,capture_output=True,text=True)
    assert r.returncode==0 and '--cartesian-step-mm' in r.stdout
    for args in [['--end-distance-mm','5'],['--cartesian-step-mm','nan']]:
        r=subprocess.run([sys.executable,'-B',str(cli),'--backend','unused','--allow-candidate-baseline',*args],cwd=tmp_path,capture_output=True,text=True)
        assert r.returncode==2
    assert not list(tmp_path.iterdir())
