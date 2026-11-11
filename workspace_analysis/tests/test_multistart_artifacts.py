"""Actual frozen-input multi-start artifacts, semantic FCL and exact-sample replay."""
from pathlib import Path
from hashlib import sha256
import json
import os
import shutil
import numpy as np
import pytest
from test_resources import ARTIFACTS
from workspace_analysis.robot.model import load_model
from workspace_analysis.robot.fk import ForwardKinematics
from workspace_analysis.experiments.multistart_benchmark import verify_multistart, bound_files
from workspace_analysis.experiments.ready_pose import eligible_ready
ROOT=Path(__file__).resolve().parents[1]
RUN='multistart_benchmark/t53_b_20261109_01'


def test_all_trials_preserve_starts_goals_scene_and_planner_identity():
    run=ARTIFACTS/RUN;parent=json.loads((run/'manifest.json').read_text());bound_files(run,parent)
    assert len(parent['conditions'])==11
    rows=json.loads((run/'summary.json').read_text())['rows']
    assert len(rows)==44 and sum(r['mode']=='local_only' for r in rows)==8
    model=load_model(ROOT/'config/workspace.yaml');starts={}
    for name,anchor in parent['conditions'].items():
        condition=run/name
        assert sha256((condition/'manifest.json').read_bytes()).hexdigest()==anchor
        assert verify_multistart(condition,model)['status']=='PASS'
        result=json.loads((condition/'result.json').read_text())
        key=(result['base_pose']['id'],result['start_id'])
        starts.setdefault(key,result['start_q']);assert starts[key]==result['start_q']
        assert result['multi_start_ik_executed'] is False and result['new_approach_generated'] is False
        assert result['preparation_motion_to_this_start_verified'] is False
        for goal in result['goals']:
            assert {t['method'] for t in goal['paths']}=={'linear','rrt_connect'}
            for trial in goal['paths']:
                assert trial['settings']==result['planning_settings']
                if trial['success'] and result['workspace_mode']=='local_only':
                    assert trial['quality']['workspace_exit'] is False
                    assert trial['quality']['tcp_x_min_m']>=.250


def test_ready_poses_are_real_g4_samples_not_goals_and_independently_native_valid():
    from workspace_analysis.workspace.results import load_results
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.task.spec import parse_task
    from workspace_analysis.task.targets import build_targets
    from workspace_analysis.collision.semantic import SemanticConfig
    from workspace_analysis.experiments.multistart_benchmark import evaluator_for
    from workspace_analysis.planning.validity import ConfigurationValidity
    from workspace_analysis.planning.workspace_constraint import WorkspaceConstraint,TCPWorkspaceCheck
    run=ARTIFACTS/RUN;ready=json.loads((run/'ready_poses.json').read_text())
    model=load_model(ROOT/'config/workspace.yaml');fk=ForwardKinematics(model)
    semantic=SemanticConfig.load(ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    for station,label in [('b0','D0'),('ym70','D1')]:
        condition=run/(station+'_'+label+'_S2_conditional')
        data=json.loads((condition/'result.json').read_text());scene=load_scene(condition/'scene_snapshot.json')
        context=json.loads((condition/'base_pose_context.json').read_text());world=np.array(context['world_from_base'])
        spec=parse_task(json.loads((condition/'task_spec.json').read_text()));targets=build_targets(spec,scene)
        port=(world@targets.base_to_port.matrix())[:3,3]
        with evaluator_for(model,scene,spec,Path(os.environ['C1_BACKEND']),semantic) as evaluator:
            checker=ConfigurationValidity(evaluator,workspace_check=TCPWorkspaceCheck(WorkspaceConstraint(),world,spec.T_ee_tcp,context['base_pose']))
            for start_id in ('S0','S1','S2'):
                selected=ready[station][start_id];row=selected['selected'];assert selected['status']=='SELECTED'
                arrays,_=load_results(row['source_pool'],model=model)
                np.testing.assert_array_equal(arrays['q'][row['original_sample_index']],row['q'])
                assert checker.check(row['q'],fresh=True)['valid']
                if start_id!='S0':
                    xyz=(world@fk(row['q'])@spec.T_ee_tcp)[:3,3]
                    assert eligible_ready(row['q'],xyz,model.limits,port,[g['q'] for g in data['goals']],start_id,selected['selection_rules'])[0]
                    assert selected['planner_results_used_for_selection'] is False
        assert evaluator.client.cleanup['exit_normal']


def test_multistart_replay_never_plans_and_preserves_actual_start_and_npz(monkeypatch):
    from jetrover_idc_visualization.reader import read_catalog
    from jetrover_idc_visualization.controller import ReplayController
    from workspace_analysis.planning.ik import NumericalIK
    import workspace_analysis.planning.rrt_connect as planner
    def forbidden(*args,**kwargs):raise AssertionError('replay must not solve IK or plan')
    monkeypatch.setattr(NumericalIK,'solve',forbidden);monkeypatch.setattr(planner,'rrt_connect',forbidden)
    run=ARTIFACTS/RUN
    starts=[]
    for start_id in ('S0','S1','S2'):
        condition=run/('b0_D0_'+start_id+'_conditional')
        anchor=sha256((condition/'manifest.json').read_bytes()).hexdigest()
        catalog=read_catalog(ROOT,condition,ROOT/'ros2/jetrover_idc_visualization/config/t51_inputs.json',expected_manifest_sha256=anchor)
        result=json.loads((condition/'result.json').read_text());assert list(catalog.start_q)==result['start_q']
        starts.append(catalog.start_q);controller=ReplayController(catalog)
        for goal in catalog.ports['P22'].goals:
            saved=next(g for g in result['goals'] if g['solution_id']==goal.solution_id)
            for trial in saved['paths']:
                if not trial['success']:
                    reasons=goal.planner_results[trial['method']]['reasons']
                    assert reasons==trial['first_failure_state']['reasons']
                    assert goal.planner_results[trial['method']]['environment_pairs']==trial['first_failure_state']['environment_pairs']
            for method,path in goal.paths.items():
                with np.load(condition/path.source_frames[0][0],allow_pickle=False) as a:
                    np.testing.assert_array_equal(path.q,a['q']);np.testing.assert_array_equal(path.tcp,a['T_base_tcp'])
                controller.select('P22',goal.solution_id,method,'Path');controller.play();observed=[]
                while controller.state!='FINISHED':observed.append(controller.tick()['q'])
                np.testing.assert_array_equal(observed,path.q)
    assert len(set(starts))==3


def test_validator_rejects_corrupt_npz_even_with_updated_file_hash(tmp_path):
    run=tmp_path/'condition';shutil.copytree(ARTIFACTS/RUN/'b0_D0_S2_local_only',run)
    result=json.loads((run/'result.json').read_text())
    trial=next(t for g in result['goals'] for t in g['paths'] if t.get('trajectory_file'))
    name=trial['trajectory_file']
    with np.load(run/name,allow_pickle=False) as a:arrays={k:a[k].copy() for k in a.files}
    arrays['q'][0,0]+=.001
    with (run/name).open('wb') as f:np.savez_compressed(f,**arrays)
    manifest=json.loads((run/'manifest.json').read_text());manifest['files'][name]=sha256((run/name).read_bytes()).hexdigest()
    (run/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(AssertionError):verify_multistart(run,load_model(ROOT/'config/workspace.yaml'))


def test_real_ros_multistart_local_only_exact_samples_start_tf_and_scene(tmp_path):
    from test_replay_ros import probe
    condition=ARTIFACTS/RUN/'b0_D0_S2_local_only'
    report=probe('multistart',tmp_path,96,benchmark=condition,anchor=sha256((condition/'manifest.json').read_bytes()).hexdigest())
    assert report['start_id']=='S2' and report['workspace_mode']=='local_only'
    assert report['actual_start_q_verified'] and report['minimum_world_tcp_x']>=.250
    assert report['world_fixed_scene_markers_checked']==38 and report['max_ee_tf_error']<=1e-9
    assert report['cases'][0]['all_original_indices_seen']
