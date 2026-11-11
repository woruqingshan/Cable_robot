"""Owned-process failures and real MoveIt/FCL JetRover integration."""
import json
import os
from pathlib import Path
import sys
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from workspace_analysis.collision.client import CollisionClient
from workspace_analysis.collision.protocol import CollisionError
from workspace_analysis.robot.model import load_model
from workspace_analysis.robot.fk import ForwardKinematics
from workspace_analysis.scene.schema import load_scene

ROOT=Path(__file__).resolve().parents[1]


def fake_backend(tmp_path,mode):
    p=tmp_path/'backend'
    p.write_text(f'''#!{sys.executable}
import sys,json,time,os
for line in sys.stdin:
 r=json.loads(line)
 if {mode!r}=='exit':os._exit(7)
 if {mode!r}=='timeout':time.sleep(5)
 if {mode!r}=='malformed':print('not json',flush=True);continue
 print(json.dumps({{'protocol_version':1,'id':r['id'] if {mode!r}!='id' else 'wrong',
 'identity':None,'status':'OK','backend_status':'STOPPED' if r['op']=='SHUTDOWN' else 'OK'}}),flush=True)
 if r['op']=='SHUTDOWN':break
''')
    p.chmod(0o755);return p


@pytest.mark.parametrize('mode',['exit','timeout','malformed','id','identity'])
def test_owned_process_failures_never_clear_and_are_reaped(tmp_path,mode):
    c=CollisionClient(fake_backend(tmp_path,mode),timeout=.15)
    if mode=='identity':c.identity={'expected':'identity'}
    with pytest.raises(CollisionError):c._exchange({'op':'PING'})
    cleanup=c.close()
    assert cleanup['resources_released'] and not cleanup['exit_normal']
    assert c.process.poll() is not None


def test_normal_shutdown_and_unique_requests(tmp_path):
    c=CollisionClient(fake_backend(tmp_path,'good'))
    a=c._exchange({'op':'PING'});b=c._exchange({'op':'PING'})
    assert a['id']!=b['id']
    cleanup=c.close()
    assert cleanup['exit_code']==0 and cleanup['exit_normal'] and c.close()==cleanup


@pytest.fixture(scope='module')
def native():
    # Native integration is required, not skipped or replaced by mocks.
    p=os.environ.get('C1_BACKEND')
    assert p and Path(p).is_file(),'Build backend and set C1_BACKEND to run the full suite'
    return Path(p)


@pytest.fixture(scope='module')
def model():return load_model(ROOT/'config/workspace.yaml')


def full_tree_reference(model,q,gripper):
    positions=model.joint_positions(q,{'r_joint':gripper})
    children={j.child for j in model.joints.values()}
    root=(set(j.parent for j in model.joints.values())-children).pop()
    transforms={root:np.eye(4)}
    pending=list(model.joints.values())
    while pending:
        for joint in list(pending):
            if joint.parent not in transforms:continue
            origin=np.eye(4);origin[:3,:3]=Rotation.from_euler('xyz',joint.rpy).as_matrix();origin[:3,3]=joint.xyz
            motion=np.eye(4)
            if joint.kind!='fixed':motion[:3,:3]=Rotation.from_rotvec(np.array(joint.axis)/np.linalg.norm(joint.axis)*positions[joint.name]).as_matrix()
            transforms[joint.child]=transforms[joint.parent]@origin@motion;pending.remove(joint)
    inverse=np.linalg.inv(transforms[model.base_frame])
    return {k:inverse@v for k,v in transforms.items()}


def test_real_native_robot_model_fk_full_tree_meshes_and_s1(native,model):
    scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    with CollisionClient(native) as client:
        ready=client.initialize(model,scene);audit=ready['audit']
        assert audit['urdf_link_count']==29 and audit['urdf_joint_count']==28
        assert audit['collision_geometry_count']==audit['moveit_shape_count']==24
        assert audit['missing_visual_collision_links']==['lidar_link']
        assert audit['root_frame']=='base_footprint' and audit['acm_size']==0
        assert len(audit['loaded_objects_base'])==38 and audit['detector']=='FCL'
        loaded={o['id']:o for o in audit['loaded_objects_base']}
        for o in scene.collision_objects:
            assert loaded[o['id']]['size_m']==list(o['size_m'])
            np.testing.assert_allclose(loaded[o['id']]['transform'],scene.object_transform(o['id']),atol=1e-12)
        fk=ForwardKinematics(model)
        for q in [[0]*5,[.35,-.6,.8,-.4,.2],[2.08,-.4,.5,-.3,.25]]:
            for gripper in [0,.3]:
                result=client.query(model,q,auxiliary_positions={'r_joint':gripper})
                t=np.array(result['T_base_to_ee']);reference=fk(q)
                assert np.linalg.norm(t[:3,3]-reference[:3,3])<=1e-5
                assert Rotation.from_matrix(reference[:3,:3].T@t[:3,:3]).magnitude()<=1e-4
                full=full_tree_reference(model,q,gripper)
                assert set(full)==set(result['link_transforms_base'])
                for link,value in full.items():np.testing.assert_allclose(result['link_transforms_base'][link],value,atol=1e-10,rtol=0)
                assert result['object_count']==38
                assert result['decision'] in {'COLLIDING','UNVERIFIED'}
    assert client.cleanup['exit_normal']


def test_native_invalid_full_state_is_error(native,model):
    with CollisionClient(native) as client:
        client.initialize(model,load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml'))
        positions=model.joint_positions([0]*5);positions['joint1']=10
        with pytest.raises(CollisionError):client.query_positions(positions)


def test_native_missing_resource_cannot_be_ready(native,model,tmp_path):
    from dataclasses import replace
    import re
    xml=re.sub(r'filename="[^"]+"','filename="file:///c1_absent_mesh.stl"',model.urdf_xml,count=1)
    # First filename can be visual; remove ALL resource paths to guarantee collision failure.
    xml=re.sub(r'filename="[^"]+"','filename="file:///c1_absent_mesh.stl"',xml)
    with CollisionClient(native) as client:
        with pytest.raises(Exception):client.initialize(replace(model,urdf_xml=xml),load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml'))
    assert client.cleanup['resources_released']


@pytest.mark.parametrize('sign',[-1,1])
def test_real_native_rotated_parent_and_cylinder_dimensions(native,model,sign):
    from workspace_analysis.scene.schema import parse_scene
    data=load_scene(ROOT/'config/scenes/synthetic_dev.yaml').to_dict()
    data['scene']['frames'][0]['pose']['quaternion_xyzw']=[0,sign*np.sqrt(.5),0,np.sqrt(.5)]
    scene=parse_scene(json.dumps(data))
    with CollisionClient(native) as client:
        ready=client.initialize(model,scene)
        objects={o['id']:o for o in ready['audit']['loaded_objects_base']}
        for o in scene.collision_objects:
            np.testing.assert_allclose(objects[o['id']]['transform'],scene.object_transform(o['id']),atol=1e-12)
        assert objects['synthetic_cylinder']['radius_m']==.015
        assert objects['synthetic_cylinder']['length_m']==.08


def test_native_rejects_canonical_and_flat_geometry_disagreement(native,model):
    from workspace_analysis.collision.protocol import make_init
    init=make_init(model,load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml'))
    init['objects'][0]['transform'][0][3]+=.1
    with CollisionClient(native) as client:
        with pytest.raises(CollisionError,match='transform mismatch'):
            client._exchange({'op':'INIT',**init})
