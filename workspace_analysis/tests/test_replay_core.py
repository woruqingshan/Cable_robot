"""Replay contracts and state/geometry tests with no ROS or historical artifacts."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from replay_fixtures import catalog_fixture
from jetrover_idc_visualization.controller import ReplayController
from jetrover_idc_visualization.data import joined, joint_values, Trajectory
from jetrover_idc_visualization.errors import ReplayError, ReplayBlocked
from jetrover_idc_visualization.markers import scene_specs, port_specs
from jetrover_idc_visualization.protocol import command
from jetrover_idc_visualization.storage import ReadOnlyStore, sha
from jetrover_idc_visualization.reader import bind_inputs


def test_state_machine_original_samples_pause_resume_reset_and_end_restart():
    catalog = catalog_fixture(); c = ReplayController(catalog)
    c.select('P22',1,'rrt_connect','Path + Approach'); t = c.sequence
    c.play(); assert c.tick()['sample_index']==0
    c.tick(); before = c.current(); c.pause()
    for _ in range(3): assert c.tick()['q']==before['q']
    assert c.state=='PAUSED'; c.resume(); assert c.tick()['q']==before['q']
    while c.state!='FINISHED': c.tick()
    np.testing.assert_array_equal(c.current()['q'],t.q[-1])
    for _ in range(3): assert c.tick()['q']==t.q[-1].tolist()
    c.play(); assert c.tick()['sample_index']==0
    c.reset(); assert c.state=='STOPPED' and c.index==0


@pytest.mark.parametrize('phase',['Path','Approach','Path + Approach'])
def test_phase_data_and_boundary_provenance(phase):
    catalog = catalog_fixture(); c = ReplayController(catalog); c.select('P12',0,'linear',phase)
    goal = catalog.ports['P12'].goals[0]
    expected = goal.paths['linear'] if phase=='Path' else goal.approach if phase=='Approach' else joined(goal.paths['linear'],goal.approach)
    c.play(); observed = []
    while c.state!='FINISHED': observed.append(c.tick()['q'])
    np.testing.assert_array_equal(observed,expected.q)
    if phase=='Path + Approach':
        assert len(observed)==len(goal.paths['linear'].q)+len(goal.approach.q)-1
        assert c.sequence.source_frames[len(goal.paths['linear'].q)]==goal.approach.source_frames[1]


def test_task_switch_stops_old_playback_and_failed_ports_cannot_play():
    c = ReplayController(catalog_fixture()); c.select('P22',0,'linear','Path'); c.play(); c.tick()
    c.select('P12',0,'rrt_connect','Approach')
    assert c.state=='STOPPED' and c.index==0 and c.sequence.phases[0]=='Approach'
    c.select('P11',None,None,'Path'); assert c.sequence is None and c.state=='STOPPED'
    with pytest.raises(ReplayError,match='synthetic'): c.play()
    assert c.status()['task_status']=='IK_NO_SOLUTION_WITHIN_BUDGET'


@pytest.mark.parametrize('selection',[('bad',0,'linear','Path'),('P22',999,'linear','Path'),
    ('P22',True,'linear','Path'),('P22',0,'missing','Path'),('P22',0,'linear','missing')])
def test_invalid_selection_never_continues_old_animation(selection):
    c = ReplayController(catalog_fixture()); c.select('P22',0,'linear','Path'); c.play()
    with pytest.raises(ReplayError): c.select(*selection)
    assert c.state=='STOPPED'


def test_joint_mapping_comes_from_model_and_excludes_mimic_publishers():
    catalog = catalog_fixture(); names,values = joint_values(catalog.model,[.1,.2,.3,.4,.5],.3)
    assert names==('joint1','joint2','joint3','joint4','joint5','r_joint')
    assert values==(.1,.2,.3,.4,.5,.3)
    assert catalog.model.joint_positions([0]*5,{'r_joint':.3})['l_joint']==pytest.approx(-.2)
    with pytest.raises(ValueError): joint_values(catalog.model,[3]*5,.3)


@pytest.mark.parametrize('change',['q','tcp','valid','clearance','boundary'])
def test_untrusted_trajectory_and_mismatched_join_rejected(change):
    goal = catalog_fixture().ports['P22'].goals[0]; t = goal.paths['linear']
    if change=='boundary':
        with pytest.raises(ReplayError): joined(goal.paths['linear'],replace(goal.approach,q=goal.approach.q+.01))
    elif change=='q':
        q=t.q.copy();q[0,0]=np.nan
        with pytest.raises(ReplayError): replace(t,q=q)
    elif change=='tcp':
        tcp=t.tcp.copy();tcp[0,3,3]=0
        with pytest.raises(ValueError): replace(t,tcp=tcp)
    elif change=='valid':
        values=t.valid.copy();values[0]=False
        with pytest.raises(ReplayError): replace(t,valid=values)
    else:
        with pytest.raises(ReplayError): replace(t,self_clearance=-t.self_clearance)
    with pytest.raises(ValueError): t.q.flags.writeable=True


@pytest.mark.parametrize('name',['../escape','/etc/passwd','a/../b','a//b','./x','a\\b'])
def test_store_rejects_traversal_and_ambiguous_names(tmp_path,name):
    with pytest.raises(ReplayError): ReadOnlyStore(tmp_path).read(name)


def test_store_missing_hash_lfs_symlinks_and_json_duplicates(tmp_path):
    store=ReadOnlyStore(tmp_path)
    with pytest.raises(ReplayBlocked): store.read('missing')
    (tmp_path/'data').write_bytes(b'data')
    with pytest.raises(ReplayError): store.read('data',digest='0'*64)
    (tmp_path/'link').symlink_to(tmp_path/'data')
    with pytest.raises(ReplayError): store.read('link')
    (tmp_path/'pointer').write_text('version https://git-lfs.github.com/spec/v1\n')
    with pytest.raises(ReplayBlocked): store.read('pointer')
    (tmp_path/'bad.json').write_text('{"x":1,"x":2}')
    with pytest.raises(ReplayError): store.object('bad.json')


def test_explicit_input_bindings_preserve_hashes_and_do_not_follow_original_paths(tmp_path):
    (tmp_path/'current').write_bytes(b'input'); h=sha(b'input')
    bindings=bind_inputs(ReadOnlyStore(tmp_path),{'schema_version':1,'workspace_relative_files':['current']},
                         {'input_hashes':{'/not/a/readable/historical/path':h}})
    assert bindings=={'/not/a/readable/historical/path':tmp_path/'current'}
    with pytest.raises(ReplayBlocked):
        bind_inputs(ReadOnlyStore(tmp_path),{'schema_version':1,'workspace_relative_files':['current']}, {'input_hashes':{'old':'0'*64}})


def test_scene_boxes_ports_and_rotated_cylinder_markers_use_framegraph():
    from workspace_analysis.scene.schema import parse_scene
    from scipy.spatial.transform import Rotation
    catalog=catalog_fixture(); scene=catalog.scene
    specs=scene_specs(scene); assert len(specs)==38
    for spec,obj in zip(specs,scene.collision_objects):
        t=scene.object_transform(obj['id']);np.testing.assert_allclose(spec.position,t[:3,3],atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(spec.orientation).as_matrix(),t[:3,:3],atol=1e-12)
    markers=port_specs(scene,'P22',catalog.targets['P22'])
    labels=[m.text for m in markers if m.kind=='TEXT_VIEW_FACING']
    assert set(labels)==set(catalog.ports)
    data=scene.to_dict();data['scene']['collision_objects'].append({'id':'test_cylinder','parent_frame':'base_link','type':'cylinder',
        'radius_m':.02,'length_m':.1,'pose':{'position_m':[.1,.2,.3],'quaternion_xyzw':Rotation.from_euler('y',.4).as_quat().tolist()}})
    cylinder=next(s for s in scene_specs(parse_scene(json.dumps(data))) if s.kind=='CYLINDER')
    assert cylinder.scale==(.04,.04,.1)
    np.testing.assert_allclose(cylinder.position,[.1,.2,.3],atol=1e-12)


def test_protocol_rejects_stale_sessions_unknown_actions_and_noninteger_revision():
    base={'version':1,'session_id':'session','request_id':'1','selection_revision':2,'action':'play'}
    assert command(json.dumps(base),'session',2)==base
    for changed in [{**base,'session_id':'old'},{**base,'action':'servo'},{**base,'selection_revision':True},{**base,'extra':0}]:
        with pytest.raises(ReplayError): command(json.dumps(changed),'session',2)
