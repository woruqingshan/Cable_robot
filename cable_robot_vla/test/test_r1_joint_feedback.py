import math
import struct
from types import SimpleNamespace as S
from pathlib import Path

import pytest
from cable_robot_vla.hardware_feedback import (
    RawFeedbackAdapter, JointFeedbackConversion, samples_from_diagnostic_report,
    SERVO_IDS, JOINT_NAMES)
from cable_robot_vla.robot_state_provider import RobotStateProvider


def config():
    return {name:dict(id=i,init=700 if i==10 else 500,min=1000,max=0)
            for i,name in zip(SERVO_IDS,JOINT_NAMES)}


def report(now=1_000_000_000):
    rows=[]
    for index,i in enumerate(SERVO_IDS):
        pulse=700 if i==10 else 502
        started=now-20_000_000+index*1_000_000
        rows.append(dict(servo_id=i,raw_pulse=pulse,read_started_at_ns=started,
            response_received_at_ns=started+500_000,error=None,
            outer_frame_validated=True,raw_payload_hex=struct.pack("<BBbh",i,5,0,pulse).hex(),
            source_type="unknown",clock_domain="host_monotonic"))
    return dict(sequence=1,complete=True,source_type="unknown",
        canonical_observation_eligible=False,channels=rows)


def snapshot():
    adapter=RawFeedbackAdapter(clock_ns=lambda:1_000_000_000)
    adapter.offer(samples_from_diagnostic_report(report()),sequence=1)
    return adapter.snapshot()


def test_vendor_candidate_units_mapping_and_gripper_are_explicit():
    model=JointFeedbackConversion(config(),configuration_sha256="a"*64)
    converted=model.convert(snapshot())
    assert converted["joint_names"]==["joint1","joint2","joint3","joint4","joint5","r_joint"]
    assert converted["positions"][:5]==pytest.approx([-2*math.radians(240)/1000]*5)
    assert converted["positions"][5]==0
    assert converted["source_type"]=="unknown"
    assert not converted["physical_calibrated"]
    assert not converted["canonical_observation_eligible"]
    assert converted["ros_joint_state_stamp_ns"] is None


def test_unflipped_direction_and_zero_are_configuration_derived():
    settings=config();settings["joint1"].update(min=0,max=1000,init=500)
    result=JointFeedbackConversion(settings,configuration_sha256="a"*64).convert(snapshot())
    assert result["positions"][0]==pytest.approx(2*math.radians(240)/1000)


@pytest.mark.parametrize("bad",[
    {"joint1":dict(id=6,init=500,min=1000,max=0)},
    {"joint1":dict(id=1,init=True,min=1000,max=0)},
    {"joint1":dict(id=1,init=500,min=500,max=500)}])
def test_bad_mapping_refused(bad):
    settings=config();settings.update(bad)
    with pytest.raises(ValueError):JointFeedbackConversion(settings,configuration_sha256="a"*64)


def test_out_of_range_values_are_not_silently_clipped():
    value=snapshot();value["channels"][0]["raw_pulse"]=1001
    with pytest.raises(ValueError):
        JointFeedbackConversion(config(),configuration_sha256="a"*64).convert(value)


def test_partial_or_cached_reports_cannot_become_raw_feedback():
    value=report();value["source_type"]="controller_cached"
    with pytest.raises(ValueError):samples_from_diagnostic_report(value)
    value=report();value["complete"]=False
    with pytest.raises(ValueError):samples_from_diagnostic_report(value)


def test_provider_keeps_candidate_feedback_separate_and_expires_it():
    ticks=[1.0]
    provider=RobotStateProvider(clock=lambda:ticks[0])
    provider.conversion=JointFeedbackConversion(config(),configuration_sha256="a"*64)
    converted=provider.offer_diagnostic_report(report())
    assert "positions" in converted
    assert provider.read_state_snapshot()["raw_values"]==[None]*6
    assert provider.get_source_status()["hardware_readback"]=="DIAGNOSTIC_ONLY"
    ticks[0]=2
    assert provider.read_converted_feedback()["validity"]=="unavailable"


def test_fk_uses_only_five_arm_joints_and_passes_gripper_as_auxiliary():
    import numpy as np
    seen=[]
    def pose(q,**kwargs):
        seen.append((q,kwargs))
        return S(position=np.zeros(3),rotation=np.eye(3))
    fk=S(pose=pose,model=S(base_frame="base_link",ee_frame="end_effector_link",
                         metadata=lambda:{"reference":"fixture"}))
    result=JointFeedbackConversion(config(),configuration_sha256="a"*64,fk=fk).convert(snapshot())
    assert len(seen[0][0])==5 and seen[0][1]=={"auxiliary_positions":{"r_joint":0}}
    assert result["fk_candidate"]["base_frame"]=="base_link"
    assert "not measured" in result["fk_candidate"]["interpretation"]


def test_real_vendor_formula_matches_candidate_without_loading_a_driver():
    p=Path("/home/ubuntu/ros2_ws/src/driver/servo_controller/servo_controller/joint_position_controller.py")
    if not p.exists():pytest.skip("vendor reference unavailable")
    namespace={};exec(compile(p.read_text(),str(p),"exec"),namespace)
    model=JointFeedbackConversion(config(),configuration_sha256="a"*64)
    result=model.convert(snapshot())
    for index,name in enumerate(JOINT_NAMES):
        original=namespace["JointPositionController"](
            {k:S(value=v) for k,v in config()[name].items()},name)
        assert result["positions"][index]==original.pos_pulse_to_rad(result["raw_pulses"][index])


def test_real_urdf_fk_reference_starts_from_declared_model():
    pytest.importorskip("PyKDL")
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.robot.fk import ForwardKinematics
    path=Path(__file__).resolve().parents[2]/"workspace_analysis/config/workspace.yaml"
    model=load_model(path)
    solver=ForwardKinematics(model)
    pose=solver.pose([0]*5,auxiliary_positions={"r_joint":0})
    assert pose.position.shape==(3,)
    assert model.joint_order==tuple(JOINT_NAMES[:5])



def test_real_ros_candidate_topic_has_unset_stamp_and_cannot_feed_observation(tmp_path):
    rclpy=pytest.importorskip('rclpy')
    import json,time,yaml
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from cable_robot_vla.real_observation_node import RealObservationNode
    path=tmp_path/'mapping.yaml'
    path.write_text(yaml.safe_dump({'/**':{'ros__parameters':config()}}))
    rclpy.init();node=monitor=None
    try:
        node=RealObservationNode(parameter_overrides=[
            Parameter('camera_topic',value='/fixture/image'),
            Parameter('scene_sha256',value='a'*64),
            Parameter('output_directory',value=str(tmp_path/'run')),
            Parameter('raw_feedback_topic',value='/idc_vla/real/raw_feedback'),
            Parameter('raw_feedback_configuration_path',value=str(path))])
        owned={p.topic_name for p in node.publishers}
        assert owned <= {'/rosout','/parameter_events','/idc_vla/real/diagnostics',
                         '/idc_vla/real/diagnostic_joint_states'}
        assert '/idc_vla/real/diagnostic_joint_states' in owned
        received=[]
        monitor=rclpy.create_node('joint_diagnostic_test_monitor')
        subscription=monitor.create_subscription(JointState,
            '/idc_vla/real/diagnostic_joint_states',received.append,10)
        end=time.monotonic()+.7
        while time.monotonic()<end:rclpy.spin_once(monitor,timeout_sec=.05)
        message=String();message.data=json.dumps(report(time.monotonic_ns()))
        def forbidden(_):raise AssertionError('raw diagnostics cannot feed canonical joints')
        node.core.joints=forbidden
        node._raw_feedback(message)
        end=time.monotonic()+1
        while not received and time.monotonic()<end:rclpy.spin_once(monitor,timeout_sec=.05)
        assert len(received)==1
        assert received[0].name==list(JOINT_NAMES)
        assert received[0].header.stamp.sec==0 and received[0].header.stamp.nanosec==0
        assert received[0].position[0]==pytest.approx(-2*math.radians(240)/1000)
        assert not node.recorder.path.joinpath('observations.jsonl').read_text()
    finally:
        if node:node.close();node.destroy_node()
        if monitor:monitor.destroy_node()
        rclpy.shutdown()


def test_source_or_device_timestamp_cannot_be_silently_relabelled():
    value=snapshot();value["channels"][0]["source_type"]="controller_cached"
    with pytest.raises(ValueError):
        JointFeedbackConversion(config(),configuration_sha256="a"*64).convert(value)
    value=report();value["channels"][0]["device_sample_time_ns"]=123
    with pytest.raises(ValueError):samples_from_diagnostic_report(value)
