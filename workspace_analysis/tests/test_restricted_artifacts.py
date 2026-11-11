"""Explicit fixed-goal collection and read-only replay evidence."""
import json
from hashlib import sha256
from pathlib import Path
import numpy as np
from test_resources import ARTIFACTS
from workspace_analysis.robot.model import load_model
from workspace_analysis.experiments.restricted_benchmark import verify_restricted
from jetrover_idc_visualization.reader import read_catalog
from jetrover_idc_visualization.controller import ReplayController
ROOT=Path(__file__).resolve().parents[1]
RUN='restricted_benchmark/t53_a_20261109_02'


def test_all_eleven_conditions_keep_original_goals_and_complete_fresh_evidence():
    run=ARTIFACTS/RUN
    manifest=json.loads((run/'manifest.json').read_text())
    assert len(manifest['conditions'])==11
    model=load_model(ROOT/'config/workspace.yaml')
    for name,anchor in manifest['conditions'].items():
        condition=run/name
        assert sha256((condition/'manifest.json').read_bytes()).hexdigest()==anchor
        assert verify_restricted(condition,model)['status']=='PASS'
        result=json.loads((condition/'result.json').read_text())
        assert len(result['goals'])==2 and result['source_start_sample_index']==0
        assert result['multi_start_ik_executed'] is False and result['new_approach_generated'] is False
        for goal in result['goals']:
            if result['start_check']['valid'] and goal['goal_check']['valid'] and goal['task_check']['task_pose_feasible']:
                assert {t['method'] for t in goal['paths']}=={'linear','rrt_connect'}
            else:
                assert goal['paths']==[]


def test_restricted_reader_never_searches_and_plays_exact_saved_npz(monkeypatch):
    from workspace_analysis.planning.ik import NumericalIK
    import workspace_analysis.planning.rrt_connect as planner
    def forbidden(*args,**kwargs):
        raise AssertionError('read-only replay must not solve IK or plan paths')
    monkeypatch.setattr(NumericalIK,'solve',forbidden)
    monkeypatch.setattr(planner,'rrt_connect',forbidden)
    run=ARTIFACTS/RUN/'b0_D0_workspace_on'
    anchor=sha256((run/'manifest.json').read_bytes()).hexdigest()
    catalog=read_catalog(ROOT,run,ROOT/'ros2/jetrover_idc_visualization/config/t51_inputs.json',expected_manifest_sha256=anchor)
    controller=ReplayController(catalog)
    for goal in catalog.ports['P22'].goals:
        for method,path in goal.paths.items():
            name=path.source_frames[0][0]
            with np.load(run/name,allow_pickle=False) as data:
                np.testing.assert_array_equal(path.q,data['q'])
                np.testing.assert_array_equal(path.tcp,data['T_base_tcp'])
            controller.select('P22',goal.solution_id,method,'Path')
            controller.play();observed=[]
            while controller.state!='FINISHED':
                observed.append(controller.tick()['q'])
            np.testing.assert_array_equal(observed,path.q)
    assert catalog.base_pose_info['workspace_config']['enabled']
