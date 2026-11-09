"""Complete actual C datasets: fixed inputs, failed seeds, verified shortcuts and preparation."""
from pathlib import Path
import json
import os
import shutil
from hashlib import sha256
import numpy as np
import pytest
from test_resources import ARTIFACTS
from workspace_analysis.robot.model import load_model
from workspace_analysis.experiments.multistart_benchmark import bound_files
from workspace_analysis.experiments.path_trial_artifacts import verify_path_trial
ROOT=Path(__file__).resolve().parents[1]
SEARCH='robustness_benchmark/t53_c_search_20261109_01'
SHORT='robustness_benchmark/t53_c_shortcut_20261109_03'
PREP='robustness_benchmark/t53_c_preparation_20261109_01'


def test_complete_ninety_main_and_ten_local_seeds_keep_inputs_and_failure_denominators():
    run=ARTIFACTS/SEARCH;manifest=json.loads((run/'manifest.json').read_text());bound_files(run,manifest)
    data=json.loads((run/'summary.json').read_text());rows=data['rows'];model=load_model(ROOT/'config/workspace.yaml')
    assert len(rows)==100 and sum(r['experiment']=='main' for r in rows)==90
    for case in ('C1','C2','C3'):
        for start in ('S0','S1','S2'):
            subset=[r for r in rows if r['case']==case and r['start']==start]
            assert [r['seed'] for r in subset]==list(range(20261104,20261114))
            inputs=[json.loads((Path(r['trial'])/'inputs.json').read_text()) for r in subset]
            assert all(i['start_q']==inputs[0]['start_q'] and i['goal_q']==inputs[0]['goal_q'] and i['workspace_config']==inputs[0]['workspace_config'] for i in inputs)
    for r in rows:
        folder=Path(r['trial']);assert verify_path_trial(folder,model)['status']=='PASS'
        result=json.loads((folder/'result.json').read_text());trial=result['trial']
        if trial['search'] is not None:
            assert trial['search']['seed']==r['seed']
            assert trial['settings']['max_time_s']==20 and trial['settings']['max_iterations']==1500
            if r['termination']=='TIME_LIMIT':assert trial['search']['status']=='TIME_BUDGET_EXHAUSTED'
            if r['termination']=='ITERATION_LIMIT':assert trial['search']['status']=='ITERATION_BUDGET_EXHAUSTED'
        assert sha256((folder/'manifest.json').read_bytes()).hexdigest()==r['manifest_sha256']
    budget=json.loads((run/'budget_comparison.json').read_text())['rows']
    for r in budget:
        assert r['max_iterations']==1500 and r['max_time_s'] in (5,20,60)
        if r['observation_role']=='REUSED_MAIN_20S':assert r['trial'] in {v['trial'] for v in rows}


def test_all_eligible_raw_paths_have_uniform_reconstructible_shortcuts_and_preserve_raw_bytes():
    raw=ARTIFACTS/SEARCH;out=ARTIFACTS/SHORT;model=load_model(ROOT/'config/workspace.yaml')
    original=json.loads((raw/'manifest.json').read_text());bound_files(raw,original)
    manifest=json.loads((out/'manifest.json').read_text());bound_files(out,manifest)
    rows=json.loads((raw/'summary.json').read_text())['rows'];index=json.loads((out/'optimization_index.json').read_text())['rows']
    assert len(index)==len(rows)==100
    for pair in index:
        source=json.loads((Path(pair['raw_trial'])/'result.json').read_text())
        if not source['trial']['success']:
            assert pair['optimized_trial'] is None and pair['status']=='NOT_RUN_NO_VALID_RAW_PATH';continue
        folder=Path(pair['optimized_trial']);assert verify_path_trial(folder,model)['status']=='PASS'
        data=json.loads((folder/'result.json').read_text());trial=data['trial']
        assert trial['settings']['max_attempts']==20 and trial['settings']['random_seed']==20261114
        assert trial['success'] and trial['quality']['joint_path_length_rad']<=source['trial']['quality']['joint_path_length_rad']+1e-12
        if 'local' in pair['raw_trial']:
            assert trial['quality']['workspace_exit'] is False and trial['quality']['tcp_x_min_m']>=.250
    bound_files(raw,original)


def test_preparation_is_reference_only_with_original_ready_q_and_conditional_mode():
    run=ARTIFACTS/PREP;model=load_model(ROOT/'config/workspace.yaml')
    bound_files(run,json.loads((run/'manifest.json').read_text()))
    audit=json.loads((run/'home_source_audit.json').read_text())
    assert audit['physical_home_verified'] is False and audit['planning_reference']=='REFERENCE_START_TO_READY'
    assert audit['vendor_code_executed'] is False
    rows=json.loads((run/'summary.json').read_text())['rows'];assert len(rows)==8
    for r in rows:
        folder=Path(r['trial']);assert verify_path_trial(folder,model)['status']=='PASS'
        inputs=json.loads((folder/'inputs.json').read_text())
        assert inputs['workspace_mode']=='conditional' and inputs['goal_role']=='READY_POSE'
        assert inputs['preparation_label']=='REFERENCE_START_TO_READY' and inputs['physical_home_verified'] is False
        source=json.loads((Path(inputs['source_condition'])/'result.json').read_text())
        reference=json.loads((Path(inputs['reference_condition'])/'result.json').read_text())
        assert inputs['goal_q']==source['start_q'] and inputs['start_q']==reference['start_q']


def test_c_raw_shortcut_and_preparation_replay_saved_npz_without_planning(monkeypatch):
    from workspace_analysis.planning.ik import NumericalIK
    import workspace_analysis.planning.rrt_connect as rrt
    from jetrover_idc_visualization.reader import read_catalog
    from jetrover_idc_visualization.controller import ReplayController
    def forbidden(*args,**kwargs):raise AssertionError('replay cannot search or optimize')
    monkeypatch.setattr(NumericalIK,'solve',forbidden);monkeypatch.setattr(rrt,'rrt_connect',forbidden)
    paths=[ARTIFACTS/SEARCH/'main/C1_S0_seed20261104',ARTIFACTS/SHORT/'main_C1_S0_seed20261104',
        ARTIFACTS/PREP/'b0_S0_to_S2_rrt_connect']
    for folder in paths:
        anchor=sha256((folder/'manifest.json').read_bytes()).hexdigest()
        catalog=read_catalog(ROOT,folder,ROOT/'ros2/jetrover_idc_visualization/config/t51_inputs.json',expected_manifest_sha256=anchor)
        data=json.loads((folder/'result.json').read_text());inputs=json.loads((folder/'inputs.json').read_text())
        assert list(catalog.start_q)==inputs['start_q'];goal=catalog.ports[inputs['target']].goals[0]
        if not data['trial']['success']:assert not goal.paths;continue
        path=goal.paths[inputs['method']]
        with np.load(folder/data['trial']['trajectory_file'],allow_pickle=False) as arrays:
            np.testing.assert_array_equal(path.q,arrays['q']);np.testing.assert_array_equal(path.tcp,arrays['T_base_tcp'])
        controller=ReplayController(catalog);controller.select(inputs['target'],inputs['goal_id'],inputs['method'],'Path')
        controller.play();observed=[]
        while controller.state!='FINISHED':observed.append(controller.tick()['q'])
        np.testing.assert_array_equal(observed,path.q)


def test_shortcut_tampering_cannot_pass_even_after_rehashing_changed_npz(tmp_path):
    source=ARTIFACTS/SHORT/'main_C1_S0_seed20261104';folder=tmp_path/'trial';shutil.copytree(source,folder)
    with np.load(folder/'trajectory.npz',allow_pickle=False) as arrays:data={k:arrays[k].copy() for k in arrays.files}
    data['q'][0,0]+=.001
    with (folder/'trajectory.npz').open('wb') as f:np.savez_compressed(f,**data)
    manifest=json.loads((folder/'manifest.json').read_text());manifest['files']['trajectory.npz']=sha256((folder/'trajectory.npz').read_bytes()).hexdigest()
    (folder/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(AssertionError):verify_path_trial(folder,load_model(ROOT/'config/workspace.yaml'))


def test_actual_ros_optimized_local_path_preserves_samples_and_world_tf(tmp_path):
    from test_replay_ros import probe
    folder=ARTIFACTS/SHORT/'local_LOCAL_S2_seed20261104'
    report=probe('path_trial',tmp_path,97,benchmark=folder,anchor=sha256((folder/'manifest.json').read_bytes()).hexdigest())
    assert report['actual_start_q_verified'] and report['minimum_world_tcp_x']>=.250
    assert report['workspace_mode']=='local_only' and report['cases'][0]['all_original_indices_seen']
    assert report['max_ee_tf_error']<=1e-9 and report['world_fixed_scene_markers_checked']==38
