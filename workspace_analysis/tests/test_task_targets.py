"""T1 scene-derived port axes, nontrivial TCP and immutable task provenance."""
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest,yaml
from workspace_analysis.task.spec import load_task,parse_task,TaskError
from workspace_analysis.task.targets import build_targets,tcp_errors,angle_between
from workspace_analysis.scene.schema import load_scene
ROOT=Path(__file__).resolve().parents[1]


def setup():
    spec=load_task(ROOT/'config/tasks/p22_research_v0.yaml');scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    return spec,scene,build_targets(spec,scene)


def test_real_p22_port_pre_tcp_and_frame_axes():
    spec,scene,targets=setup()
    np.testing.assert_allclose(targets.base_to_port.position_m,[.33,0,.22],atol=1e-12)
    np.testing.assert_allclose(targets.insertion_axis_base,[1,0,0],atol=1e-12)
    np.testing.assert_allclose(targets.key_axis_base,[0,0,1],atol=1e-12)
    np.testing.assert_allclose(targets.pre_tcp.position_m,[.29,0,.22],atol=1e-12)
    np.testing.assert_allclose(targets.T_base_pre_tcp[:3,:3],np.eye(3),atol=1e-12)
    np.testing.assert_allclose(targets.T_base_pre_ee@spec.T_ee_tcp,targets.T_base_pre_tcp,atol=1e-12)
    np.testing.assert_allclose(targets.T_base_pre_ee[:3,3],[.26,0,.22],atol=1e-12)
    assert targets.to_dict()['tcp_parameters_status']=='synthetic_uncalibrated'
    assert not targets.to_dict()['tool_collision_geometry_modeled']
    assert targets.to_dict()['scene_sha256']==scene.canonical_scene_sha256


def test_independent_tcp_and_approach_edits_do_not_mutate_scene_or_prior_spec():
    spec,scene,old=setup();raw=scene.to_json();data=spec.data
    data['approach']['distance_m']=.06;data['tcp']['T_ee_tcp']['position_m']=[.01,.02,.04]
    new=build_targets(parse_task(data),scene)
    np.testing.assert_allclose(new.pre_tcp.position_m,[.27,0,.22],atol=1e-12)
    np.testing.assert_allclose(new.T_base_pre_ee@new.spec.T_ee_tcp,new.T_base_pre_tcp,atol=1e-12)
    assert old.spec.sha256!=new.spec.sha256 and old.spec.tcp_sha256!=new.spec.tcp_sha256
    assert old.pre_tcp.position_m!=new.pre_tcp.position_m and scene.to_json()==raw


def test_nonstandard_local_tool_axes_are_mapped_without_hardcoding_tcp_basis():
    spec,scene,_=setup();d=spec.data;d['tcp']['insertion_axis_local']=[0,0,1];d['tcp']['key_axis_local']=[1,0,0]
    t=build_targets(parse_task(d),scene)
    np.testing.assert_allclose(t.T_base_pre_tcp[:3,:3]@np.array([0,0,1]),[1,0,0],atol=1e-12)
    np.testing.assert_allclose(t.T_base_pre_tcp[:3,:3]@np.array([1,0,0]),[0,0,1],atol=1e-12)


@pytest.mark.parametrize('change',['port','axis','parallel','pose','distance','tolerance','nan','bool','extra','calibration'])
def test_invalid_tasks_fail_explicitly(change):
    spec,scene,_=setup();d=spec.data
    if change=='port':d['target']['port_id']='P99'
    if change=='axis':d['tcp']['insertion_axis_local']=[2,0,0]
    if change=='parallel':d['tcp']['key_axis_local']=[1,0,0]
    if change=='pose':d['tcp']['T_ee_tcp']['quaternion_xyzw']=[0,0,0,0]
    if change=='distance':d['approach']['distance_m']=-.04
    if change=='tolerance':d['tolerances']['key_axis_deg']=180
    if change=='nan':d['solver']['ftol']=float('nan')
    if change=='bool':d['tcp']['T_ee_tcp']['position_m']=[False,0,0]
    if change=='extra':d['new_field']='unreviewed'
    if change=='calibration':d['tcp']['status']='hardware_calibrated'
    with pytest.raises((TaskError,ValueError)):build_targets(parse_task(d),scene)


def test_key_flip_is_180deg_error_and_axis_flip_is_not_zero():
    _,_,targets=setup();t=targets.T_base_pre_tcp.copy();t[:3,:3]=np.diag([1,-1,-1])
    errors=tcp_errors(t,targets)
    assert errors['insertion_axis_deg']<1e-10 and errors['key_axis_deg']==pytest.approx(180)
    assert angle_between([1,0,0],[-1,0,0])==pytest.approx(np.pi)


def test_yaml_duplicates_and_aliases_rejected(tmp_path):
    source=(ROOT/'config/tasks/p22_research_v0.yaml').read_text();p=tmp_path/'task.yaml'
    p.write_text(source+'\nid: duplicate\n')
    with pytest.raises(TaskError):load_task(p)
    p.write_text('x: &a [1]\ny: *a\n')
    with pytest.raises(TaskError):load_task(p)
