"""Fixtures validate quality contracts, never claim physical calibration."""
from types import SimpleNamespace as S
import json
import pytest
np = pytest.importorskip("numpy")
from cable_robot_vla.real_camera import CameraRecordAdapter, StreamStats
from cable_robot_vla.real_recording import RealRecorder
from cable_robot_vla.robot_state_provider import RobotStateProvider


def msg(encoding="rgb8"):
    return S(header=S(frame_id="actual_optical_frame",stamp=S(sec=10,nanosec=123)),
        width=2,height=1,step=6,encoding=encoding,is_bigendian=0,data=bytes(6))


class Bridge:
    def imgmsg_to_cv2(self, message, desired_encoding):
        if desired_encoding == "bgr8":
            return np.array([[[30,20,10],[60,50,40]]], dtype=np.uint8)
        return np.array([[1000,0]], dtype=np.uint16)


def test_camera_only_records_original_metadata_without_complete_observation(tmp_path):
    recorder=RealRecorder(tmp_path,{})
    adapter=CameraRecordAdapter(recorder,bridge=Bridge())
    original=msg()
    rgb=adapter.rgb(original)
    assert rgb.data == bytes([10,20,30,40,50,60])
    assert rgb.header.stamp.nanosec == 123 and rgb.header.frame_id == original.header.frame_id
    assert adapter.provenance["original_encoding"] == "rgb8"
    assert (recorder.path/adapter.provenance["png_path"]).is_file()
    assert (recorder.path/"observations.jsonl").read_text() == ""
    recorder.close()


def test_depth_is_separate_lossless_uint16(tmp_path):
    import cv2
    recorder=RealRecorder(tmp_path,{})
    adapter=CameraRecordAdapter(recorder,bridge=Bridge())
    image=msg("16UC1");image.step=4;image.data=bytes(4)
    adapter.depth(image)
    path=next((recorder.path/"depth").glob("*.png"))
    restored=cv2.imread(str(path),cv2.IMREAD_UNCHANGED)
    assert restored.dtype == np.uint16 and restored.tolist() == [[1000,0]]
    assert (recorder.path/"observations.jsonl").read_text() == ""
    recorder.close()


def test_unknown_encoding_fails_explicitly(tmp_path):
    recorder=RealRecorder(tmp_path,{})
    adapter=CameraRecordAdapter(recorder,bridge=Bridge())
    with pytest.raises(ValueError):adapter.rgb(msg("mono8"))
    with pytest.raises(ValueError):adapter.depth(msg("32FC1"))
    recorder.close()


def test_sampling_skips_are_not_claimed_driver_drops(tmp_path):
    recorder=RealRecorder(tmp_path,{})
    now=[1.];adapter=CameraRecordAdapter(recorder,bridge=Bridge(),clock=lambda:now[0])
    assert adapter.rgb(msg()) is not None
    now[0]=1.01;assert adapter.rgb(msg()) is None
    status=adapter.status()
    assert status["intentional_sampling_skips"]["rgb"]==1
    assert status["streams"]["rgb"]["received"]==2
    assert status["streams"]["rgb"]["exact_driver_drop_count"] is None
    recorder.close()


def test_state_cache_preserves_ids_pulses_and_publication_stamp_without_trust():
    provider=RobotStateProvider(clock=lambda:1.)
    message=S(header=S(stamp=S(sec=10,nanosec=12)),
        servo_state=[S(id=i,position=500+i) for i in [1,2,3,4,5,10]])
    provider.offer_controller_cache(message)
    snapshot=provider.read_state_snapshot()
    assert snapshot["servo_ids"]==[1,2,3,4,5,10]
    assert snapshot["raw_values"]==[501,502,503,504,505,510]
    assert snapshot["units"]=="pulse" and snapshot["timestamp_ns"]==10000000012
    assert snapshot["source_type"]=="controller_cached"
    assert snapshot["canonical_observation_eligible"] is False
    assert provider.read_gripper_state()["raw_values"]==[510]
    assert provider.get_source_status()["rpc_requests_sent"]==0
    assert provider.get_source_status()["hardware_readback"]=="BLOCKED"
    for method in ["move_joint","set_gripper","release_arm","hold_arm"]:
        assert not hasattr(provider,method)


def test_invalid_cache_clears_previous_values():
    provider=RobotStateProvider()
    message=S(header=S(stamp=S(sec=1,nanosec=0)),servo_state=[S(id=1,position=500),S(id=1,position=501)])
    with pytest.raises(ValueError):provider.offer_controller_cache(message)
    assert provider.read_state_snapshot()["source_type"]=="unknown"
