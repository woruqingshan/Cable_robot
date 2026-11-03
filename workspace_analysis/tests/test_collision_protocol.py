"""Protocol and scene adapter tests; no Python collision implementation."""
from pathlib import Path
import json
import numpy as np
import pytest
from workspace_analysis.collision.protocol import (CollisionError,strict_json,make_init,check_response,check_query)
from workspace_analysis.collision.diagnostics import summarize,run_diagnostic
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('text',['[]','oops','{"x":1,"x":2}','{"x":NaN}','{"x":Infinity}'])
def test_malformed_json_is_error(text):
    with pytest.raises(CollisionError):strict_json(text)


@pytest.mark.parametrize('change',[{'id':'wrong'},{'protocol_version':True},{'protocol_version':2},
                                 {'status':'ERROR','error':'bad model'},{'identity':{'x':2}}])
def test_reply_envelope_rejects_mismatch(change):
    r={'protocol_version':1,'id':'id','status':'OK','identity':{'x':1},**change}
    with pytest.raises(CollisionError):check_response(r,'id',{'x':1})


def test_valid_envelope_and_unverified_decision_preserved():
    r={'protocol_version':1,'id':'id','status':'OK','identity':{'x':1}}
    assert check_response(r,'id',{'x':1}) is r
    reply={'collision_detected':False,'self_collision':False,'environment_collision':False,
           'pairs_may_be_truncated':False,'backend_status':'OK','decision':'UNVERIFIED',
           'T_base_to_ee':np.eye(4).tolist(),'colliding_pairs':[],'query_elapsed_s':0.01}
    assert check_query(reply)['decision']=='UNVERIFIED'
    for change in [{'decision':'CLEAR'},{'self_collision':True},{'collision_detected':0},{'query_elapsed_s':-1}]:
        with pytest.raises(CollisionError):check_query({**reply,**change})


def test_s1_adapter_exact_38_objects_and_full_parent_rotations():
    model=load_model(ROOT/'config/workspace.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    request=make_init(model,scene)
    assert len(request['objects'])==38
    assert {o['id'] for o in request['objects']}=={o['id'] for o in scene.collision_objects}
    for obj in request['objects']:
        np.testing.assert_array_equal(obj['transform'],scene.object_transform(obj['id']))
        assert obj['type']=='box' and len(obj['size_m'])==3
    front=next(o for o in request['objects'] if o['id']=='router_1_front_top')
    np.testing.assert_allclose(np.array(front['transform'])[:3,3],[.3325,0,.339],atol=1e-14)
    np.testing.assert_allclose(np.array(front['transform'])[:3,:3],[[0,0,-1],[0,1,0],[1,0,0]],atol=1e-14)
    assert json.loads(request['scene_canonical_json'])==scene.to_dict()
    with pytest.raises(CollisionError):make_init(model,{})


def test_summary_union_and_error_denominator():
    def row(s,e,decision):return {'self_collision':s,'environment_collision':e,'collision_detected':s or e,
                                 'decision':decision,'backend_query_elapsed_s':.001 if decision!='ERROR' else None}
    rows=[row(True,False,'COLLIDING'),row(False,True,'COLLIDING'),row(True,True,'COLLIDING'),
          row(False,False,'UNVERIFIED'),row(None,None,'ERROR')]
    s=summarize(rows)
    assert s['total_samples']==5 and s['colliding_samples']==3
    assert s['self_collision_count']==s['environment_collision_count']==2
    assert s['both_collision_count']==1 and s['error_count']==s['unverified_count']==1


def test_init_failure_retains_all_100_indices_and_refuses_overwrite(tmp_path):
    class Broken:
        def __init__(self,*args,**kwargs):pass
        def initialize(self,*args):raise CollisionError('fixture model error')
        def close(self):return {'exit_normal':False,'exit_code':7,'resources_released':True}
    out=tmp_path/'run'
    kwargs=dict(input_prefix=ROOT/'results/workspace_n100_seed42',client_factory=Broken)
    s=run_diagnostic(ROOT/'config/workspace.yaml',ROOT/'config/scenes/idc_prototype_v0.yaml','unused',out,**kwargs)
    assert s['total_samples']==s['error_count']==100 and s['status']=='ERROR'
    d=json.loads((out/'diagnostics.json').read_text())
    assert [r['sample_index'] for r in d['per_sample']]==list(range(100))
    assert all(r['self_collision'] is None and r['decision']=='ERROR' for r in d['per_sample'])
    with pytest.raises(CollisionError,match='existing'):run_diagnostic(
        ROOT/'config/workspace.yaml',ROOT/'config/scenes/idc_prototype_v0.yaml','unused',out,**kwargs)
