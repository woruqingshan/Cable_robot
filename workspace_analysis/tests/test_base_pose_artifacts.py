"""Explicit new station evidence and historical B0 byte/numeric equivalence."""
from pathlib import Path
import json
import numpy as np
from test_resources import ARTIFACTS
from workspace_analysis.experiments.task_benchmark_validation import verify_benchmark
from workspace_analysis.robot.model import load_model
from workspace_analysis.experiments.base_pose_context import verify_context
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.experiments.assembly_semantics import _hash
from jetrover_idc_visualization.reader import read_catalog
ROOT=Path(__file__).resolve().parents[1]
PILOT='base_pose_benchmark/t52_r2_s1_20261104_01'


def test_b0_all_nine_ports_preserve_ik_goals_paths_approach_and_npz():
    old=ARTIFACTS/'task_benchmark/t5_1_s1_20261104_03';new=ARTIFACTS/PILOT/'stations/b0/benchmark'
    model=load_model(ROOT/'config/workspace.yaml');assert verify_benchmark(new,model)['status']=='PASS'
    for port in json.loads((old/'settings.json').read_text())['ports']:
        a=json.loads((old/'ports'/port/'result.json').read_text());b=json.loads((new/'ports'/port/'result.json').read_text())
        assert a['status']==b['status'] and a['identity']==b['identity'] and a['targets']==b['targets']
        assert a['reachability']['seeds']==b['reachability']['seeds']
        for x,y in zip(a['reachability']['ik']['attempts'],b['reachability']['ik']['attempts']):
            np.testing.assert_array_equal(x['q'],y['q']);assert x['task_check']==y['task_check']
        assert len(a['goals'])==len(b['goals'])
        for x,y in zip(a['goals'],b['goals']):
            assert x['q']==y['q'] and x['status']==y['status'] and x['candidate']==y['candidate']
        for p in (old/'ports'/port).glob('*.npz'):
            with np.load(p,allow_pickle=False) as x,np.load(new/'ports'/port/p.name,allow_pickle=False) as y:
                assert set(x.files)==set(y.files)
                # Query IDs index each run's own ledger, not physical samples.
                # Full verifier above binds every new ID to its exact q/TF/flags.
                for key in set(x.files)-{'query_id'}:np.testing.assert_array_equal(x[key],y[key])


def test_pilot_all_station_hashes_inputs_start_and_geometry_are_verified():
    run=ARTIFACTS/PILOT;manifest=json.loads((run/'manifest.json').read_text());model=load_model(ROOT/'config/workspace.yaml')
    for name,h in manifest['files'].items():assert _hash(run/name)==h
    for name,h in manifest['input_hashes'].items():assert _hash(name)==h
    summary=json.loads((run/'summary.json').read_text());assert len(summary['rows'])==21
    for station,identity in summary['stations'].items():
        base=run/'stations'/station;child=base/'benchmark'
        assert _hash(child/'manifest.json')==identity['benchmark_manifest_sha256']
        assert verify_benchmark(child,model)['status']=='PASS'
        context=json.loads((base/'base_pose_context.json').read_text())
        assert verify_context(context,load_scene(base/'derived_scene.json'),model)['world_cabinet_fixed']
        start=json.loads((child/'common_start.json').read_text())
        assert start['original_sample_index']==0 and len(start['tested'])==1
        assessment=json.loads((base/'station_assessment.json').read_text())
        assert assessment['physical_coverage_status']=='PARTIAL_UNVERIFIED'


def test_new_scene_replay_uses_embedded_bound_inputs_and_preserves_goals():
    child=ARTIFACTS/PILOT/'stations/xm30/benchmark'
    catalog=read_catalog(ROOT,child,ROOT/'ros2/jetrover_idc_visualization/config/t51_inputs.json',
        expected_manifest_sha256='d39077e52aef45339dd0b167ce1f12650f07301ef9dee34d100c58f58c39919f')
    assert catalog.base_pose_info['base_pose']['x_m']==-.03 and catalog.verification['trusted_manifest_bound']
    assert catalog.world_from_base[0,3]==-.03
    for port,p in catalog.ports.items():
        saved=json.loads((child/'ports'/port/'result.json').read_text())
        assert len(p.goals)==len(saved['goals'])
        for goal,source in zip(p.goals,saved['goals']):
            assert goal.q==tuple(source['q'])
            assert set(goal.planner_results)=={x['method'] for x in source['paths']}
