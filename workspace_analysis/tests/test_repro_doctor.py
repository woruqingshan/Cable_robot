"""R2 doctor negative cases use isolation/mocks, never alter installed resources."""
from copy import deepcopy
from hashlib import sha256
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace
import pytest
from repro import doctor
from workspace_analysis.robot.model import model_from_xml

ROOT=Path(__file__).resolve().parents[1]


def fixture_model(folder):
    (folder/'urdf').mkdir(parents=True)
    source=folder/'urdf/model.xacro';source.write_text('synthetic source with portable resource spelling')
    mesh=folder/'shape.stl';mesh.write_bytes(b' '*80+struct.pack('<I',1)+struct.pack('<12fH',0,0,1,0,0,0,1,0,0,0,1,0,0))
    xml=f'<robot name="fixture"><link name="base_link"/><link name="tip"><collision><geometry><mesh filename="{mesh.as_uri()}"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base_link"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>'
    return model_from_xml(xml,ee_frame='tip',joint_order=['j'],source_path=str(source),source_hashes={str(source):sha256(source.read_bytes()).hexdigest()})


def test_portable_identity_distinguishes_relocation_from_mesh_and_include_change(tmp_path):
    a=fixture_model(tmp_path/'a');b=fixture_model(tmp_path/'b')
    first=doctor.portable_robot_identity(a);second=doctor.portable_robot_identity(b)
    assert first['raw_urdf_sha256']!=second['raw_urdf_sha256']
    comparison=doctor.compare_robot_identity(second,first)
    assert comparison['status']=='PASS' and comparison['details']['source_path_changed']
    mesh=tmp_path/'b/shape.stl';mesh.write_bytes(mesh.read_bytes()+b'changed')
    assert doctor.compare_robot_identity(doctor.portable_robot_identity(b),first)['status']=='FAIL'
    changed=deepcopy(first);changed['include_sha256']['urdf/other.xacro']='changed'
    assert 'include_sha256' in doctor.compare_robot_identity(changed,first)['details']['differences']


@pytest.fixture
def full_stubs(monkeypatch,tmp_path):
    monkeypatch.setenv('ROS_DISTRO','humble')
    monkeypatch.setattr(doctor,'probe_modules',lambda modules:[doctor.check('packages','PASS','fixture imports')])
    monkeypatch.setattr(doctor,'probe_sources',lambda root:[doctor.check('sources','PASS','fixture source')])
    monkeypatch.setattr(doctor,'probe_ros_packages',lambda:[doctor.check('ros.packages','PASS','fixture package index')])
    monkeypatch.setattr(doctor,'toolchain_snapshot',lambda:{'compiler':{'returncode':0},'cmake':{'returncode':0}})
    identity={'raw_urdf_sha256':'raw','portable_model_sha256':'portable','source_path':'/fixture/source',
              'include_sha256':{},'resources':[],'base_frame':'base_link','ee_frame':'end_effector_link','joint_order':list('12345'),
              'chain':[],'auxiliary_positions':{},'xacro_environment':{},'collision_mesh_count':24,
              'model_config_sha256':doctor.digest(ROOT/'config/workspace.yaml')}
    asset=tmp_path/'identity.json';asset.write_text(json.dumps(identity))
    monkeypatch.setattr(doctor,'load_robot',lambda *args:SimpleNamespace())
    monkeypatch.setattr(doctor,'portable_robot_identity',lambda model:deepcopy(identity))
    from workspace_analysis.collision.semantic import SemanticConfig
    monkeypatch.setattr(SemanticConfig,'load',lambda *args,**kwargs:SimpleNamespace(wire=lambda:{'semantic':{'fixture':True}}))
    monkeypatch.setattr(doctor,'native_probe',lambda *args:{'fixture_native':True,'queries':0})
    binary=tmp_path/'backend';binary.write_text('fixture binary, not executed');binary.chmod(0o755)
    return asset,binary


def status(report,name):return next(row['status'] for row in report['checks'] if row['id']==name)


def test_full_environment_mock_is_ready_without_safety_claim(full_stubs):
    asset,binary=full_stubs;r=doctor.run_doctor(ROOT,'full',backend=binary,assets_file=asset)
    assert r['status']=='READY_FOR_RESEARCH_REPRODUCTION' and r['exit_code']==0
    assert not r['hardware_validated'] and not r['historical_artifacts_checked']


def test_missing_official_model_is_blocked_not_pass(monkeypatch,full_stubs):
    asset,binary=full_stubs
    def missing(*args):raise FileNotFoundError('official description absent')
    monkeypatch.setattr(doctor,'load_robot',missing)
    r=doctor.run_doctor(ROOT,'full',backend=binary,assets_file=asset)
    assert status(r,'robot.identity')=='BLOCKED' and status(r,'native')=='BLOCKED' and r['exit_code']!=0


def test_missing_native_is_explicitly_blocked(full_stubs,tmp_path):
    asset,_=full_stubs;r=doctor.run_doctor(ROOT,'full',backend=tmp_path/'missing',assets_file=asset)
    assert r['status']=='BLOCKED' and status(r,'native')=='BLOCKED' and r['exit_code']==3


def test_wrong_model_identity_is_fail_not_a_geometry_bypass(monkeypatch,full_stubs):
    asset,binary=full_stubs;expected=json.loads(asset.read_text());expected['portable_model_sha256']='wrong';asset.write_text(json.dumps(expected))
    r=doctor.run_doctor(ROOT,'full',backend=binary,assets_file=asset)
    assert r['status']=='FAIL' and status(r,'robot.identity')=='FAIL' and status(r,'native')=='BLOCKED'


def test_core_does_not_request_ros_model_native_or_historical_artifacts(monkeypatch,tmp_path):
    monkeypatch.setattr(doctor,'probe_modules',lambda modules:[doctor.check('packages','PASS','fixture')])
    monkeypatch.setattr(doctor,'probe_sources',lambda root:[doctor.check('sources','PASS','fixture')])
    def forbidden(*args):raise AssertionError('Full dependency requested in core mode')
    monkeypatch.setattr(doctor,'load_robot',forbidden);monkeypatch.setattr(doctor,'native_probe',forbidden)
    r=doctor.run_doctor(tmp_path,'core')
    assert r['status']=='READY_FOR_CORE_TESTS' and status(r,'robot.identity')=='NOT_CHECKED' and status(r,'native')=='NOT_CHECKED'
    assert not list(tmp_path.iterdir())


def test_cli_json_output_is_parseable_readonly_default_and_no_overwrite(tmp_path):
    cli=ROOT/'repro/scripts/doctor.py';path=tmp_path/'report.json'
    command=[sys.executable,'-B',str(cli),'--mode','core','--json','--json-output',str(path)]
    r=subprocess.run(command,cwd=tmp_path,capture_output=True,text=True,timeout=30)
    assert r.returncode==0,r.stderr
    assert json.loads(r.stdout)==json.loads(path.read_text())
    before=path.read_bytes();again=subprocess.run(command,cwd=tmp_path,capture_output=True,text=True,timeout=30)
    assert again.returncode==2 and path.read_bytes()==before
