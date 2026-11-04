"""World/Base invariants, direct root objects and fixed-start station contracts."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from workspace_analysis.scene.base_pose import BasePose, derive_scene, verify_derivation, frame_mapping
from workspace_analysis.scene.schema import load_scene, parse_scene
from workspace_analysis.scene.model import SceneError
from workspace_analysis.experiments.base_pose_benchmark import checked_pilot, station_assessment
from workspace_analysis.experiments.base_pose_context import make_context, verify_context
from workspace_analysis.robot.root_frame import root_mount, world_root_transform
from workspace_analysis.robot.model import load_model
from workspace_analysis.task.spec import load_task, TaskError
from workspace_analysis.task.template import tasks_for_ports
from workspace_analysis.task.targets import build_targets
ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('pose',[BasePose('b0'),BasePose('x',.03,-.07),BasePose('yaw',-.03,.07,.6)])
def test_full_tree_world_invariance_and_axes(pose):
    world=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');before=world.to_json()
    derived=derive_scene(world,pose);report=verify_derivation(world,derived,pose)
    assert report['status']=='PASS' and report['objects_checked']==38 and report['ports_checked']==9
    assert world.to_json()==before
    if pose.is_identity:assert derived is world
    else:assert derived.canonical_scene_sha256!=world.canonical_scene_sha256
    names=frame_mapping(world,pose)
    for f in world.frames:
        moved=next(g for g in derived.frames if g.id==names[f.id])
        if f.parent!='base_link':assert moved.pose==f.pose
    spec=load_task(ROOT/'config/tasks/p22_research_v0.yaml')
    target=build_targets(tasks_for_ports(spec,derived,['P22'])[0],derived)
    np.testing.assert_allclose(pose.matrix()@target.pre_tcp.matrix(),build_targets(spec,world).pre_tcp.matrix(),atol=1e-12)


def test_direct_base_collision_object_transformed_once():
    world=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');data=world.to_dict()
    obj=data['scene']['collision_objects'][0];obj['parent_frame']='base_link'
    source=parse_scene(json.dumps(data));pose=BasePose('direct',.04,-.03,-.4)
    assert verify_derivation(source,derive_scene(source,pose),pose)['status']=='PASS'


@pytest.mark.parametrize('kwargs',[{'x_m':True},{'x_m':float('nan')},{'y_m':float('inf')},{'yaw_rad':4},{'id':'../x'},{'x_m':'1'}])
def test_invalid_base_pose_rejected(kwargs):
    with pytest.raises(SceneError):BasePose(**{'id':'test',**kwargs})


def test_bounded_pilot_explicit_fixed_index():
    data=json.loads((ROOT/'config/base_pose/s1_pilot_v0.json').read_text());rows=checked_pilot(data)
    assert len(rows)==5 and sum(len(p) for _,p in rows)==21 and data['start_sample_index']==0
    for key,value in [('start_sample_index',True),('usage','SAFE')]:
        bad=deepcopy(data);bad[key]=value
        with pytest.raises(TaskError):checked_pilot(bad)
    bad=deepcopy(data);bad['stations'].append(bad['stations'][0])
    with pytest.raises(TaskError):checked_pilot(bad)


def test_summary_plot_uses_recorded_rows_and_refuses_overwrite(tmp_path):
    from workspace_analysis.viz.base_pose import export_base_pose_summary
    row={'base_pose_id':'b0','port_id':'P22','status':'SUCCESS_RESEARCH_ONLY','ik_solutions':2,'collision_candidates':1,
        'linear_successes':1,'linear_trials':1,'rrt_connect_successes':1,'rrt_connect_trials':1,
        'approach_successes':1,'approach_trials':1,'start_valid':True,'coverage':'PARTIAL_UNVERIFIED'}
    data={'stage':'T5.2-R2','usage':'RESEARCH_ONLY','rows':[row],'stations':{'b0':{}}}
    output=tmp_path/'summary.html';export_base_pose_summary(data,output)
    html=output.read_text();assert 'RESEARCH_ONLY' in html and 'collision-valid 1' in html
    with pytest.raises(FileExistsError):export_base_pose_summary(data,output)


def test_real_root_mount_and_context_tamper_rejected():
    model=load_model(ROOT/'config/workspace.yaml');root,mount=root_mount(model)
    assert root=='base_footprint'
    assert mount[2,3]==pytest.approx(.11609108215767461)
    pose=BasePose('offset',.03,-.07,.3);_,transform=world_root_transform(model,pose.matrix())
    np.testing.assert_allclose(transform@mount,pose.matrix(),atol=1e-12)
    text=(ROOT/'config/scenes/idc_prototype_v0.yaml').read_text();world=parse_scene(text)
    context=make_context(text,pose,model);derived=derive_scene(world,pose)
    assert verify_context(context,derived,model)['world_cabinet_fixed']
    context['world_from_base'][0][3]+=.01
    with pytest.raises(TaskError):verify_context(context,derived,model)


def test_station_body_classification_is_not_hardware_safety():
    model=load_model(ROOT/'config/workspace.yaml')
    for collision,pairs,status in [(False,[],'MODELED_BODY_CLEAR'),(True,[{'first':'base_link','second':'rack'}],'MODELED_BODY_COLLISION'),(True,[{'first':'link3','second':'rack'}],'BODY_COLLISION_UNRESOLVED')]:
        state={'environment_collision':collision,'environment_pairs':pairs}
        result=station_assessment(model,{'tested':[state],'status':'INVALID_START','original_sample_index':0})
        assert result['status']==status and result['physical_station_safety_verified'] is False


def test_invalid_fixed_start_never_replaced_or_planned(tmp_path,monkeypatch):
    import os
    from hashlib import sha256
    from workspace_analysis.workspace.sampling import sample_workspace
    from workspace_analysis.workspace.results import save_results
    from workspace_analysis.experiments.task_benchmark import run_benchmark
    from workspace_analysis.experiments.task_benchmark_validation import verify_benchmark
    import workspace_analysis.experiments.task_benchmark as benchmark
    model=load_model(ROOT/'config/workspace.yaml')
    data=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml').to_dict()
    data['scene']['collision_objects'].append({'id':'blocking_box','parent_frame':'base_link','type':'box','size_m':[3.,3.,3.],
        'pose':{'position_m':[0.,0.,0.],'quaternion_xyzw':[0.,0.,0.,1.]}})
    scene=tmp_path/'blocked.json';scene.write_text(json.dumps(data))
    # Existing sampler is used only for this tiny isolated fixture, not Pilot inputs.
    run=sample_workspace(model,samples=2,seed=42);arrays=run.arrays()
    config=ROOT/'config/workspace.yaml'
    prefix=tmp_path/'seeds';save_results(prefix,run,model,config_path=config,config_sha256=sha256(config.read_bytes()).hexdigest())
    def forbidden(*args,**kwargs):raise AssertionError('invalid start must not enter task pipeline')
    monkeypatch.setattr(benchmark,'run_tasks',forbidden)
    output=tmp_path/'benchmark'
    run_benchmark(ROOT,Path(os.environ['C1_BACKEND']),scene,ROOT/'config/tasks/p22_research_v0.yaml',prefix,output,
        port_ids=['P22'],allow_candidate=True,start_sample_index=1)
    start=json.loads((output/'common_start.json').read_text())
    assert start['status']=='INVALID_START' and start['original_sample_index']==1 and len(start['tested'])==1
    assert start['q']==arrays['q'][1].tolist()
    result=json.loads((output/'ports/P22/result.json').read_text())
    assert result['status']=='INVALID_START' and result['reachability']['status']=='NOT_RUN'
    assert verify_benchmark(output,model)['status']=='PASS'
