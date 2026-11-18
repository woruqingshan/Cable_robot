"""Boundary regressions; all validators here are explicit CPU doubles, not P2 evidence."""
from types import SimpleNamespace as NS
import pytest
from vla_protocol import codec
from vla_protocol.observation import Observation
from vla_protocol.errors import VLAError
from vla_protocol.profiles import JOINT_NAMES, RuntimeLimits
from deployment.local.test_support import observation
from cable_robot_vla.integration.sources import SensorObservationSource
from cable_robot_vla.integration.interfaces import execution_backend
from cable_robot_vla.integration.execution import GazeboBackend, known_simulation_candidate
from cable_robot_vla.adapters.p2 import P2Adapter, P2Config


def joint_message(t=1.0, q=0.0):
    return NS(header=NS(stamp=NS(sec=int(t),nanosec=int(round((t-int(t))*1e9))),frame_id="base_link"),
        name=[*JOINT_NAMES,"r_joint"], position=[q,0.,0.,0.,0.,0.3])


def image_message(t=1.0):
    return NS(header=NS(stamp=NS(sec=int(t),nanosec=int(round((t-int(t))*1e9))),frame_id="camera_frame"),
        encoding="rgb8",is_bigendian=0,width=2,height=2,step=6,data=bytes([10,20,30]*4))


def source(kind="gazebo",clock=lambda:1.0):
    return SensorObservationSource(kind,scene_id="fixture",scene_sha256="0"*64,model_id="test",instruction="inspect",clock=clock)


def test_source_pairs_original_samples_and_preserves_frames_and_clock():
    s=source()
    s.joint_state(joint_message(1.0,0.1));s.joint_state(joint_message(1.2,0.2));s.image(image_message(1.0))
    obs=s.snapshot(0,1_200_000_000)
    assert obs.joints.positions[0]==0.1 and obs.joints.stamp.nanoseconds==1_000_000_000
    assert obs.images[0].frame_id=="camera_frame" and obs.joints.stamp.clock_domain=="ros_sim"
    assert codec.loads(codec.dumps(obs),Observation)==obs


@pytest.mark.parametrize("kind,simulated,clock",[("synthetic",True,"ros_system"),("gazebo",True,"ros_sim"),("real",False,"ros_system")])
def test_sensor_backends_share_unchanged_protocol(kind,simulated,clock):
    s=source(kind);s.joint_state(joint_message());s.image(image_message())
    obs=codec.loads(codec.dumps(s.snapshot(0,1_000_000_000)),Observation)
    assert obs.simulated is simulated and obs.joints.stamp.clock_domain==clock
    assert obs.gripper.simulated is simulated and not obs.gripper.calibrated


def test_missing_stale_and_invalid_sensor_streams_fail_closed():
    now=[1.0];s=source(clock=lambda:now[0])
    with pytest.raises(VLAError):s.snapshot(0,1_000_000_000)
    s.joint_state(joint_message());s.image(image_message());now[0]=2.0
    with pytest.raises(VLAError) as e:s.snapshot(0,1_000_000_000)
    assert e.value.code=="STALE_OBSERVATION"
    bad=joint_message();bad.position[0]=float("nan")
    with pytest.raises(ValueError):s.joint_state(bad)
    assert not s.joint_history


class Validator:
    config=P2Config(mode="required",workspace_enabled=True)
    calls=0
    cancel_callback=None
    def validate(self,action,obs):
        self.calls+=1
        if self.cancel_callback:self.cancel_callback()
        record=P2Adapter().validate(action,obs)
        return record.model_copy(update={"status":"PASS_RESEARCH_ONLY","discrete_only":True,"provenance":{"workspace_checked":True}})
    def close(self):pass


def backend(enabled=True,guard=lambda:True):
    validator=Validator();sent=[];now=[1.0]
    b=GazeboBackend(validator=validator,enabled=enabled,simulation_guard=guard,publish=lambda q:sent.append(q),clock=lambda:now[0])
    return b,validator,sent,now


def test_unknown_native_actions_never_enter_validator_or_execution():
    b,v,sent,now=backend();obs=observation()
    native=known_simulation_candidate(obs,[0.]*5).model_copy(update={"native_action_space":"smolvla_unmapped","synthetic":False,"semantics":"unknown"})
    with pytest.raises(VLAError) as e:b.submit(native,obs)
    assert e.value.code=="UNSUPPORTED_ACTION_PROFILE" and v.calls==0 and not sent


@pytest.mark.parametrize("enabled,guard",[(False,True),(True,False)])
def test_gazebo_execution_requires_explicit_enablement_and_live_guard(enabled,guard):
    b,v,sent,now=backend(enabled,lambda:guard);obs=observation()
    with pytest.raises(VLAError):b.submit(known_simulation_candidate(obs,[0.]*5),obs)
    assert not sent and v.calls==0


def test_cancel_during_validation_cannot_resurrect_execution():
    b,v,sent,now=backend();obs=observation()
    v.cancel_callback=lambda:b.cancel("SENSOR_LOST")
    with pytest.raises(VLAError) as e:b.submit(known_simulation_candidate(obs,[0.]*5),obs)
    assert e.value.code=="REQUEST_INVALIDATED" and not b.active and not sent


def test_execution_uses_observed_feedback_and_cancellation_discards_pending_targets():
    b,v,sent,now=backend();obs=observation()
    result=b.submit(known_simulation_candidate(obs,[0.05,0.,0.,0.,0.]),obs)
    assert result.status=="PASS_RESEARCH_ONLY" and not sent
    now[0]=2.0;b.tick(obs)
    assert sent and sent[-1][0]==pytest.approx(0.025)
    b.cancel("SENSOR_STALE");before=len(sent);now[0]=3.0;b.tick(obs)
    assert len(sent)==before and not b.active


def test_hardware_backend_is_disabled_and_inspection_backend_has_no_publisher():
    with pytest.raises(VLAError) as e:execution_backend("hardware",validator=Validator())
    assert e.value.code=="HARDWARE_EXECUTION_DISABLED"
    b=execution_backend("inspection",validator=Validator())
    assert not hasattr(b,"publish") and not b.active


def test_simulation_clock_rewind_retires_sensor_session_and_requires_new_image():
    s=source();s.joint_state(joint_message(2.0));s.image(image_message(2.0))
    old=s.snapshot(0,2_000_000_000).session_id
    s.joint_state(joint_message(1.0))
    with pytest.raises(VLAError):s.snapshot(1,1_000_000_000)
    s.image(image_message(1.0))
    assert s.snapshot(1,1_000_000_000).session_id != old


def test_invalid_sensor_stamp_and_camera_frame_are_rejected():
    s=source();msg=joint_message();msg.header.stamp.nanosec=1_000_000_000
    with pytest.raises(VLAError) as e:s.joint_state(msg)
    assert e.value.code=="INVALID_SENSOR_STAMP"
    image=image_message();image.header.frame_id=""
    with pytest.raises(VLAError):s.image(image)
