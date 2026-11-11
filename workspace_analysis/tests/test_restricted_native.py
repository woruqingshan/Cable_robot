"""Real FCL evidence for a clear state and a newly added neighboring-port Box."""
import json
import os
from pathlib import Path
from test_resources import ARTIFACTS
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.scene.restricted_scene import restricted_scene
from workspace_analysis.collision.semantic import SemanticConfig
from workspace_analysis.collision.evaluator import Evaluator
ROOT=Path(__file__).resolve().parents[1]


def test_added_box_is_loaded_and_has_real_fcl_collision():
    model=load_model(ROOT/'config/workspace.yaml')
    config=json.loads((ROOT/'config/restricted_scenes/d0_d4_v0.json').read_text())
    scene=restricted_scene(load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml'),'D2',config)
    source=ARTIFACTS/'base_pose_benchmark/t52_r2_s1_20261104_01/stations/b0/benchmark'
    start=json.loads((source/'common_start.json').read_text())['q']
    goals=json.loads((source/'ports/P22/result.json').read_text())['goals']
    semantic=SemanticConfig.load(ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf',model,allow_candidate=True)
    with Evaluator(model,scene,None,Path(os.environ['C1_BACKEND']),semantic=semantic,gripper=.3,
            clearance={'self_m':0.,'environment_m':0.},timeout=60) as evaluator:
        rows=evaluator.evaluate([start]+[g['approach']['samples'][-1]['q'] for g in goals])
        assert rows[0]['environment_collision'] is False
        collisions=[p for row in rows[1:] for p in row['raw_collision_pairs'] if p['kind']=='environment']
        assert any('restricted_D2_P22' in (p['first'],p['second']) for p in collisions)
        assert evaluator.identity['scene_sha256']==scene.canonical_scene_sha256
    assert evaluator.client.cleanup['exit_normal']
