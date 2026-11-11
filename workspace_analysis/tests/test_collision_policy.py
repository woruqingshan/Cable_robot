"""C2 policy configuration and real-backend switch/recovery tests."""

from test_resources import ARTIFACTS
import copy
import json
import os
from pathlib import Path
import numpy as np
import pytest
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.collision.client import CollisionClient
from workspace_analysis.collision.policy import load_policy,canonical_policy
from workspace_analysis.collision.protocol import CollisionError
from workspace_analysis.collision.coverage import coverage_report

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def model():return load_model(ROOT/'config/workspace.yaml')


def test_normalized_duplicates_reverse_pairs_and_identity(model):
    text,h=load_policy(ROOT/'config/collision/assembly_policy_v0.yaml',model);data=json.loads(text)
    duplicate=copy.deepcopy(data['allowed_pairs'][0]);duplicate['links'].reverse();data['allowed_pairs'].append(duplicate)
    assert canonical_policy(data,model)==(text,h)
    data=json.loads(text);data['allowed_pairs'][0]['reason']+=' reviewed evidence update'
    assert canonical_policy(data,model)[1]!=h


@pytest.mark.parametrize('case',['unknown_link','clearance','scope','validated','unresolved'])
def test_invalid_and_unverified_policy_claims_rejected(model,case):
    text,_=load_policy(ROOT/'config/collision/assembly_policy_v0.yaml',model);data=json.loads(text)
    if case=='unknown_link':data['allowed_pairs'][0]['links'][0]='not_present'
    elif case=='clearance':data['clearance']['self_m']=.01
    elif case=='scope':data['scope']='gripper_with_RJ45'
    elif case=='validated':data['review_status']='validated'
    else:data['unresolved_pairs'].append(data['allowed_pairs'][0]['links'])
    with pytest.raises(CollisionError):canonical_policy(data,model)


def test_lidar_visual_bounds_are_only_candidate_not_coverage(model):
    report=coverage_report(model)
    assert report['modeled_links']==24 and not report['coverage_validated']
    assert any(x['link']=='lidar_link' and x['classification']=='missing_physical_geometry' for x in report['links'])
    bounds=np.array(report['lidar_visual_evidence']['mesh_local_bounds_m'])
    assert np.all(bounds[1]>bounds[0])
    assert not report['lidar_visual_evidence']['overlay_implemented']
    assert report['missing_task_geometry']


def test_real_policy_switch_baseline_restore_env_and_gripper(model):
    binary=os.environ.get('C1_BACKEND');assert binary
    scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    baseline=load_policy(ROOT/'config/collision/baseline.yaml',model)
    assembly=load_policy(ROOT/'config/collision/assembly_policy_v0.yaml',model)
    prefix=ARTIFACTS / 'workspace_n100_seed42.npz'
    with np.load(prefix,allow_pickle=False) as f:q=f['q']
    with CollisionClient(binary) as client:
        client.initialize(model,scene)
        for index in [8,73]:
            client.set_policy(*baseline);a=client.query(model,q[index])
            client.set_policy(*assembly);b=client.query(model,q[index])
            assert a['environment_collision'] is b['environment_collision'] is True
            assert a['raw_self_collision']==b['raw_self_collision']
            assert b['effective_decision']=='COLLIDING'
            assert all(tuple(sorted(p)) not in {tuple(sorted(x)) for x in b['effective_self_pairs']} for p in b['filtered_pairs'])
            client.set_policy(*baseline);c=client.query(model,q[index])
            assert a['effective_self_pairs']==c['effective_self_pairs']
        a=client.query(model,[0]*5,auxiliary_positions={'r_joint':0})
        b=client.query(model,[0]*5,auxiliary_positions={'r_joint':.3})
        assert a['link_transforms_base']['r_link']!=b['link_transforms_base']['r_link']
        np.testing.assert_allclose(a['T_base_to_ee'],b['T_base_to_ee'])
