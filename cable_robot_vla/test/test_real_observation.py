"""Fixtures test protocol behavior only; they are not hardware validation."""
import copy
import json
from hashlib import sha256
from types import SimpleNamespace as S

import pytest
from vla_protocol.errors import VLAError
from vla_protocol.observation import Observation
from vla_protocol import codec
from cable_robot_vla.real_observation import RealObservationCore, load_evidence, rgb_message
from cable_robot_vla.real_recording import RealRecorder, replay


def header(ns=10_000_000_000):
    return S(stamp=S(sec=ns//10**9, nanosec=ns%10**9), frame_id="fixture_camera")


def evidence(gripper=""):
    return {"schema_version": 1, "topic": "/fixture/measured", "gripper_topic": gripper,
        "source_kind": "measured_feedback", "stamp_semantics": "driver_acquisition", "clock_domain": "ros_system",
        "joint_mapping": {f"joint{i}": f"measured{i}" for i in range(1,6)}, "gripper_name": "measured_grip",
        "units": "rad", "evidence_reference": "TEST_FIXTURE_ONLY_NOT_HARDWARE"}


def image(ns=10_000_000_000, encoding="rgb8"):
    return S(header=header(ns), encoding=encoding, width=2, height=1, step=8, is_bigendian=0,
             data=bytes([3,2,1,6,5,4,99,99]))


def joints(ns=10_000_000_000, include_grip=True):
    return S(header=header(ns), name=[f"measured{i}" for i in range(1,6)]+(["measured_grip"] if include_grip else []),
             position=[0.,.1,.2,.3,.4]+([.5] if include_grip else []))


def core(gripper="", now=None):
    return RealObservationCore(state_topic="/fixture/measured", gripper_topic=gripper, evidence=evidence(gripper),
        scene_id="fixture_scene", scene_sha256="a"*64, model_id="fixture_model", instruction="Fixture observation",
        robot_id="fixture_robot", camera_id="fixture_rgb", clock=(lambda:now[0]) if now else (lambda:1.))


def observation():
    c=core(); c.joints(joints()); c.image(image()); return c.snapshot(0,10_000_000_000)


def test_real_roundtrip_and_original_stamps():
    obs=observation(); again=codec.loads(codec.dumps(obs),Observation)
    assert again == obs
    assert not obs.simulated and not obs.gripper.calibrated and not obs.gripper.simulated
    assert obs.source_profile.profile_id == "real_uncalibrated_v1"
    assert obs.calibration_status == "real_uncalibrated" and obs.robot_id == "fixture_robot"
    assert obs.joints.stamp.nanoseconds == obs.images[0].stamp.nanoseconds == 10_000_000_000


def test_bgr_rows_are_explicitly_converted_without_changing_stamp():
    original=image(encoding="bgr8"); converted=rgb_message(original)
    assert converted.data == bytes([1,2,3,4,5,6])
    assert converted.header.stamp.sec == original.header.stamp.sec
    assert converted.header.frame_id == original.header.frame_id
    assert converted.step == 6 and converted.encoding == "rgb8"
    c=core();c.image(original);assert c.image_provenance["original_encoding"]=="bgr8"


@pytest.mark.parametrize("mutation",[lambda m:setattr(m,"encoding","mono8"),lambda m:setattr(m,"data",b""),
    lambda m:setattr(m,"step",2),lambda m:setattr(m.header,"frame_id",""),lambda m:setattr(m,"width",2048)])
def test_bad_image_clears_previous_image(mutation):
    c=core();c.image(image());bad=image();mutation(bad)
    with pytest.raises(Exception):c.image(bad)
    assert "image" not in c.source.collector.parts


def test_missing_gripper_and_untrusted_cache_rejected():
    c=core()
    with pytest.raises(VLAError,match="gripper"):c.joints(joints(include_grip=False))
    c.state_topic="/controller_manager/joint_states"
    with pytest.raises(VLAError) as err:c.joints(joints())
    assert err.value.code == "CACHED_COMMAND_STATE"
    c.state_topic="/unknown";c.evidence=None
    with pytest.raises(VLAError) as err:c.snapshot(0,10**10)
    assert err.value.code == "UNTRUSTED_STATE"


def test_separate_gripper_preserves_independent_stamp_and_receive_age():
    now=[1.];c=core("/fixture/gripper",now)
    grip=S(header=header(9_995_000_000),name=["measured_grip"],position=[.4])
    c.gripper(grip); c.joints(joints(include_grip=False));c.image(image())
    obs=c.snapshot(0,10**10)
    assert obs.gripper.stamp.nanoseconds==9_995_000_000
    assert obs.joints.stamp.nanoseconds==10**10
    now[0]=1.6;c.joints(joints(include_grip=False));c.image(image())
    with pytest.raises(VLAError) as err:c.snapshot(1,10**10)
    assert err.value.code=="STALE_OBSERVATION"


@pytest.mark.parametrize("joint_ns,now_ns,code",[(9_970_000_000,10**10,"OBSERVATION_SKEW"),
    (10**10,10_600_000_000,"STALE_OBSERVATION"),(10**10,9_900_000_000,"STALE_OBSERVATION")])
def test_thresholds_not_relaxed(joint_ns,now_ns,code):
    c=core();c.joints(joints(joint_ns));c.image(image())
    with pytest.raises(VLAError) as err:c.snapshot(0,now_ns)
    assert err.value.code==code


def test_recording_replay_tamper_and_exclusive_directory(tmp_path):
    recorder=RealRecorder(tmp_path,{"hardware_validation":False},run_name="fixture")
    obs=observation();recorder.record(obs,{"origin":"TEST_FIXTURE"});recorder.close()
    assert list(replay(recorder.path))==[obs]
    assert "data_base64" not in (recorder.path/"observations.jsonl").read_text()
    with pytest.raises(FileExistsError):RealRecorder(tmp_path,{},run_name="fixture")
    next((recorder.path/"images").glob("*.png")).write_bytes(b"corrupt")
    with pytest.raises(Exception):list(replay(recorder.path))


def test_replay_rejects_escape_and_duplicate(tmp_path):
    r=RealRecorder(tmp_path,{});r.record(observation(),{});r.close()
    path=r.path/"observations.jsonl";row=json.loads(path.read_text());row["observation"]["images"][0]["png_path"]="../outside.png"
    path.write_text(json.dumps(row)+"\n")
    with pytest.raises(Exception):list(replay(r.path))


def test_provenance_hash_and_cache_cannot_be_overridden(tmp_path):
    path=tmp_path/"evidence.json";data=evidence();path.write_text(json.dumps(data))
    digest=sha256(path.read_bytes()).hexdigest();assert load_evidence(path,digest)==data
    with pytest.raises(VLAError):load_evidence(path,"0"*64)
    data["topic"]="/joint_states";path.write_text(json.dumps(data))
    with pytest.raises(VLAError) as err:load_evidence(path,sha256(path.read_bytes()).hexdigest())
    assert err.value.code=="CACHED_COMMAND_STATE"


def test_native_output_never_maps_to_real_robot():
    from cable_robot_vla.action_adapter import ActionAdapterRegistry
    from vla_protocol.action import PolicyActionChunk,PolicyIdentity,NativeHorizon
    from vla_protocol.observation import identity_fields
    candidate=PolicyActionChunk(**identity_fields(observation()),action_id="fixture_action",
        policy=PolicyIdentity(policy_type="smolvla",policy_version="fixture",framework_revision=None,checkpoint_revision=None,processor_id="unknown",normalization_id="unknown"),
        native_action_space="unknown_six_channels",channel_names=[f"channel{i}" for i in range(6)],units=["unknown"]*6,
        semantics="unknown",shape=[50,6],values=[[0.]*6 for _ in range(50)],time_offsets_s=[(i+1)*.02 for i in range(50)],horizon=NativeHorizon(generated=50,transmitted=50),
        synthetic=False,inference_duration_s=.1)
    with pytest.raises(VLAError) as err:ActionAdapterRegistry().adapt(candidate)
    assert err.value.code=="UNSUPPORTED_ACTION_PROFILE"


def test_bgr_conversion_preserves_stamp_and_padding():
    message=image(encoding="bgr8")
    message.data=bytes([1,2,3,4,5,6,99,99])
    converted=rgb_message(message)
    assert converted.data==bytes([3,2,1,6,5,4])
    assert converted.header.stamp.sec==message.header.stamp.sec
    assert converted.header.stamp.nanosec==message.header.stamp.nanosec
    assert converted.header.frame_id==message.header.frame_id
    assert converted.step==6 and converted.encoding=="rgb8"


@pytest.mark.parametrize("change",[{"encoding":"mono8"},{"step":2},{"data":b''},{"is_bigendian":1}])
def test_invalid_image_invalidates_previous_frame(change):
    c=core();c.joints(joints());c.image(image())
    bad=image()
    for name,value in change.items():setattr(bad,name,value)
    with pytest.raises(Exception):c.image(bad)
    with pytest.raises(VLAError):c.snapshot(0,10**10)


def test_nonfinite_joints_clear_previous_valid_state():
    c=core();c.joints(joints());c.image(image())
    bad=joints();bad.position[0]=float('nan')
    with pytest.raises(VLAError):c.joints(bad)
    with pytest.raises(VLAError):c.snapshot(1,10**10)


def test_calibration_manifest_is_integrity_checked(tmp_path):
    r=RealRecorder(tmp_path,{});r.record(observation(),{});r.close()
    (r.path/'calibration/calibration_manifest.json').write_text('{}')
    with pytest.raises(VLAError):list(replay(r.path))


def test_session_reset_requires_new_run(tmp_path):
    r=RealRecorder(tmp_path,{})
    try:
        obs=observation();r.record(obs,{})
        changed=obs.model_copy(update={"session_id":"different_session","request_id":"different_request","observation_seq":1})
        with pytest.raises(VLAError) as error:r.record(changed,{})
        assert error.value.code=="SESSION_CHANGED"
    finally:r.close()
