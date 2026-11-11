"""R2 replay fixes: visible goals without paths, trust anchors and finite JSON."""
from dataclasses import replace
import json
import numpy as np
import pytest
from replay_fixtures import catalog_fixture
from jetrover_idc_visualization.data import Goal
from jetrover_idc_visualization.controller import ReplayController
from jetrover_idc_visualization.errors import ReplayError
from jetrover_idc_visualization.storage import json_object,ReadOnlyStore,sha
from jetrover_idc_visualization.reader import read_catalog,bind_inputs


@pytest.mark.parametrize('raw',['{"x":1e999}','{"nested":[{"x":-1e999}]}','{"x":NaN}','{"x":Infinity}'])
def test_json_exponent_overflow_and_nested_nonfinite_rejected(raw):
    with pytest.raises(ReplayError,match='nonfinite'):json_object(raw)
    assert json_object('{"x":1e308,"y":[true,2]}')['x']==1e308


def test_formal_reader_requires_external_manifest_anchor_before_any_file_access():
    with pytest.raises(ReplayError,match='externally selected'):
        read_catalog('/nonexistent','/nonexistent','/nonexistent')


def test_failed_planners_preserve_goal_and_disable_nonexistent_playback():
    catalog=catalog_fixture();original=catalog.ports['P22'].goals[0]
    goal=Goal(original.solution_id,original.q,'PLANNER_TIMEOUT',{},None,
        {'linear':{'success':False,'status':'COLLISION'},'rrt_connect':{'success':False,'status':'PLANNER_TIMEOUT'}},
        True,original.paths['linear'].tcp[-1])
    ports=dict(catalog.ports);ports['P22']=replace(ports['P22'],goals=(goal,))
    catalog=replace(catalog,ports=ports)
    description=catalog.describe()['P22']['goals'][0]
    assert description['planners']==['linear','rrt_connect'] and description['playable_planners']==[]
    c=ReplayController(catalog)
    for method in ('linear','rrt_connect'):
        c.select('P22',goal.solution_id,method,'Path')
        assert not c.status()['playable'] and c.status()['inspection_only']
        assert c.current()['q']==list(goal.q)
        np.testing.assert_array_equal(c.current()['tcp'],goal.display_tcp)
        assert c.status()['planner_result']['success'] is False
        with pytest.raises(ReplayError):c.play()


def test_world_tcp_uses_station_pose_without_changing_saved_joint_samples():
    from workspace_analysis.scene.base_pose import BasePose
    catalog=catalog_fixture();pose=BasePose('ym70',0,-.07,.3)
    catalog=replace(catalog,world_from_base=pose.matrix(),base_pose_info={'base_pose':pose.to_dict()})
    c=ReplayController(catalog);c.select('P22',0,'linear','Path');c.play();c.tick()
    np.testing.assert_array_equal(c.current()['q'],catalog.ports['P22'].goals[0].paths['linear'].q[0])
    np.testing.assert_allclose(c.status()['world_tcp'],pose.matrix()@c.current()['tcp'],atol=1e-12)


def test_embedded_scene_binding_is_hash_checked_and_never_follows_old_path(tmp_path):
    workspace=tmp_path/'workspace';workspace.mkdir();run=tmp_path/'run';run.mkdir()
    (workspace/'input').write_bytes(b'input');(run/'scene.json').write_bytes(b'scene')
    h=sha(b'scene');manifest={'input_hashes':{'/old/scene':h},'files':{'scene.json':h},'embedded_input_files':{'/old/scene':'scene.json'}}
    profile={'schema_version':1,'workspace_relative_files':['input']}
    assert bind_inputs(ReadOnlyStore(workspace),profile,manifest,ReadOnlyStore(run))=={'/old/scene':run/'scene.json'}
    (run/'scene.json').write_bytes(b'changed')
    with pytest.raises(ReplayError):bind_inputs(ReadOnlyStore(workspace),profile,manifest,ReadOnlyStore(run))
