"""Real native evaluator, identity, row preservation and statistical conservation."""

from test_resources import ARTIFACTS
import json
import os
from pathlib import Path
import numpy as np
import pytest
from workspace_analysis.collision.evaluator import Evaluator
from workspace_analysis.collision.policy import load_policy
from workspace_analysis.collision.protocol import CollisionError
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene,parse_scene
from workspace_analysis.workspace.results import load_results
from workspace_analysis.experiments.constrained_workspace import summarize,run_pilot

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def inputs():
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');a,_=load_results(ARTIFACTS / 'workspace_n100_seed42',model=m)
    return m,s,a


def test_v1_only_adds_two_fixed_pairs_and_preserves_unresolved(inputs):
    m,_,_=inputs;v0=json.loads(load_policy(ROOT/'config/collision/assembly_policy_v0.yaml',m)[0]);v1=json.loads(load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m)[0])
    old={tuple(p['links']) for p in v0['allowed_pairs']};new={tuple(p['links']) for p in v1['allowed_pairs']}
    assert new-old=={('back_shell_green_link','base_link'),('link4','servo_link2')}
    assert old<=new and ['link2','link3'] in v1['unresolved_pairs']


def test_batch_size_order_consistency_clearance_and_scope(inputs):
    m,s,a=inputs;binary=os.environ.get('C1_BACKEND');assert binary
    policy=load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m)
    with Evaluator(m,s,policy,binary,gripper=.3) as evaluator:
        first=evaluator.evaluate(a['q'][:12],batch_size=1);second=evaluator.evaluate(a['q'][:12],batch_size=7)
        for i,(x,y) in enumerate(zip(first,second)):
            assert x['sample_index']==y['sample_index']==i and x['q']==a['q'][i].tolist()
            for key in ['raw_self_collision','effective_self_collision','environment_collision','model_scope_collision_free','final_decision','allowed_pairs','remaining_collision_pairs','clearance']:assert x[key]==y[key]
            assert x['final_decision']!='CLEAR'
            np.testing.assert_allclose(x['ee_position'],a['position'][i],atol=1e-5)
        err=evaluator.evaluate([[99,0,0,0,0],[0,0,0,0,0]])
        assert err[0]['final_decision']=='ERROR' and err[1]['final_decision']!='ERROR'
        summary=summarize(first);assert sum(summary['decisions'].values())==12
        assert summary['effective_self']+summary['environment']-summary['both']==round(summary['collision_rejection_ratio']*12)
        identity=evaluator.identity
    with Evaluator(m,s,policy,binary,gripper=.3,clearance={'environment_m':.01,'self_m':0}) as evaluator:
        assert evaluator.identity['evaluation_sha256']!=identity['evaluation_sha256']
    d=s.to_dict();d['scene']['frames'][0]['pose']['position_m'][0]+=.01
    with Evaluator(m,parse_scene(json.dumps(d)),policy,binary,gripper=.3) as evaluator:
        assert evaluator.identity['scene_sha256']!=identity['scene_sha256']


@pytest.mark.parametrize('clearance',[{'self_m':-1,'environment_m':0},{'self_m':float('nan'),'environment_m':0},{'self_m':True,'environment_m':0}])
def test_invalid_threshold_rejected_before_backend(inputs,clearance):
    m,s,_=inputs
    with pytest.raises(CollisionError):Evaluator(m,s,load_policy(ROOT/'config/collision/baseline.yaml',m),'unused',clearance=clearance)


def test_pilot_no_overwrite_and_configuration_npz_roundtrip(inputs,tmp_path):
    m,_,a=inputs;out=tmp_path/'pilot'
    summaries=run_pilot(ROOT,os.environ['C1_BACKEND'],ARTIFACTS / 'workspace_n100_seed42',out,policies=['assembly_policy_v1'])
    assert summaries['assembly_policy_v1']['total']==100
    manifest=json.loads((out/'manifest.json').read_text());assert manifest['negative_z_retained']==14
    with np.load(out/'assembly_policy_v1_workspace_results.npz',allow_pickle=False) as f:
        np.testing.assert_array_equal(f['q'],a['q']);assert len(f['model_scope_free'])==100
    with pytest.raises(ValueError,match='existing'):run_pilot(ROOT,os.environ['C1_BACKEND'],ARTIFACTS / 'workspace_n100_seed42',out)


def test_directed_native_motion_mimic_and_fixed_assembly(inputs):
    from workspace_analysis.collision.client import CollisionClient
    m,s,_=inputs
    with CollisionClient(os.environ['C1_BACKEND']) as client:
        client.initialize(m,s)
        references={}
        for angle in [m.joints['r_joint'].lower,0.,.3,m.joints['r_joint'].upper]:
            for j in [1,2]:
                for value in m.limits[j]:
                    q=np.zeros(5);q[j]=value
                    row=client.query(m,q,auxiliary_positions={'r_joint':angle})
                    for spec in m.joints.values():
                        if spec.mimic_joint:
                            assert row['joint_positions_used'][spec.name]==pytest.approx(row['joint_positions_used'][spec.mimic_joint]*spec.multiplier+spec.offset,abs=1e-12)
                    for pair in [('back_shell_green_link','base_link'),('link4','servo_link2')]:
                        t=row['link_transforms_base'];relative=np.linalg.inv(t[pair[0]])@np.array(t[pair[1]])
                        if pair in references:np.testing.assert_allclose(relative,references[pair],atol=1e-12,rtol=0)
                        references[pair]=relative
                    assert all(c['depth_m']>=0 and np.isfinite(c['position_base_m']).all() for c in row['contacts_base'])


def test_thousand_source_rows_preserved(inputs):
    m,s,_=inputs;a,_=load_results(ARTIFACTS / 'g4_run01/experiments/workspace_n1000_seed42',model=m)
    with Evaluator(m,s,load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m),os.environ['C1_BACKEND']) as evaluator:
        rows=evaluator.evaluate(a['q'],batch_size=73)
    assert [r['sample_index'] for r in rows]==list(range(1000))
    np.testing.assert_array_equal([r['q'] for r in rows],a['q'])
    np.testing.assert_allclose([r['ee_position'] for r in rows],a['position'],atol=1e-5,rtol=0)
    assert summarize(rows)['decisions']['ERROR']==0


@pytest.mark.parametrize('distance,available,expected',[(.02,True,True),(.005,True,False),(None,False,None)])
def test_positive_clearance_model_scope_remains_physically_unverified(inputs,monkeypatch,distance,available,expected):
    from workspace_analysis.collision import evaluator as module
    m,s,_=inputs
    class FixtureClient:
        identity={'fixture':True}
        def __init__(self,*args,**kwargs):pass
        def initialize(self,*args):return {}
        def set_policy(self,*args):pass
        def close(self):return {}
        def query(self,*args,**kwargs):
            return {'raw_self_collision':False,'effective_self_collision':False,'environment_collision':False,
                    'clearance':{k:{'available':available,'separation_m':distance} for k in ['self','environment']},
                    'filtered_pairs':[],'effective_self_pairs':[],'colliding_pairs':[],'T_base_to_ee':np.eye(4).tolist(),
                    'query_elapsed_s':0,'pairs_may_be_truncated':False}
    monkeypatch.setattr(module,'CollisionClient',FixtureClient)
    with Evaluator(m,s,('fixture','sha'),'fixture',clearance={'self_m':.01,'environment_m':.01}) as evaluator:
        row=evaluator.evaluate([[0]*5])[0]
    assert row['model_scope_collision_free'] is expected
    assert row['final_decision']=='UNVERIFIED'
    assert row['verification_status']==('MODEL_ONLY' if available else 'CLEARANCE_UNSUPPORTED')
