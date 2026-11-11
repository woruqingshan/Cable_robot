import base64
from copy import deepcopy
from hashlib import sha256
import json
import struct
import zlib
import pytest
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk, RobotActionChunk, ValidatedTrajectory
from vla_protocol.errors import VLAError
from vla_protocol.observation import Observation
from vla_protocol.profiles import RuntimeLimits
from vla_protocol.png import SIGNATURE, chunk
from deployment.local.test_support import observation, moved_candidate


def test_roundtrip_preserves_stamps_frames_and_separate_gripper():
    obs = observation()
    assert codec.loads(codec.dumps(obs), Observation) == obs
    assert obs.images[0].frame_id == "mock_camera_optical_frame"
    assert obs.joints.stamp == obs.gripper.stamp == obs.images[0].stamp
    candidate = moved_candidate(obs, [[0.01] * 5, [0.02] * 5])
    assert codec.loads(codec.dumps(candidate), PolicyActionChunk) == candidate


@pytest.mark.parametrize("mutate", [
    lambda d: d.update(schema_version=2), lambda d: d.update(schema_version=True),
    lambda d: d.update(unexpected=1), lambda d: d.pop("request_id"),
    lambda d: d.update(observation_seq=-1), lambda d: d.update(task_revision="0"),
    lambda d: d.update(simulated=1), lambda d: d.update(robot_profile_id="unknown"),
    lambda d: d["joints"].update(unit="degree"),
    lambda d: d["joints"]["names"].reverse(),
    lambda d: d["joints"].update(positions=[0.] * 4),
    lambda d: d["joints"]["positions"].__setitem__(0, True),
    lambda d: d["joints"]["positions"].__setitem__(0, "0"),
    lambda d: d["joints"]["positions"].__setitem__(0, float("nan")),
    lambda d: d["joints"]["stamp"].update(nanoseconds=-1),
    lambda d: d["images"][0]["stamp"].update(clock_domain="remote_monotonic"),
    lambda d: d["images"][0].update(sha256="f" * 64),
    lambda d: d["images"][0].update(data_base64="!!!!"),
    lambda d: d["images"][0].update(width=3),
    lambda d: d["images"].extend(deepcopy(d["images"]) * 4),
])
def test_invalid_observations(mutate):
    data = observation().model_dump()
    mutate(data)
    with pytest.raises(VLAError):
        codec.validate(data, Observation)


@pytest.mark.parametrize("mutate,code", [
    (lambda d: d.update(shape=[2, 5]), "ACTION_SHAPE_MISMATCH"),
    (lambda d: d["values"][0].append(0.), "ACTION_SHAPE_MISMATCH"),
    (lambda d: d.update(time_offsets_s=[0.]), "INVALID_ACTION_TIMING"),
    (lambda d: d.update(time_offsets_s=[11.]), "INVALID_ACTION_TIMING"),
    (lambda d: d.update(values=[[0.] * 5] * 33, shape=[33, 5], time_offsets_s=[0.1] * 33), "INVALID_SCHEMA"),
    (lambda d: d["values"][0].__setitem__(0, float("inf")), "NONFINITE_NUMBER"),
    (lambda d: d.update(channel_names=["same"] * 5), "ACTION_SHAPE_MISMATCH"),
])
def test_action_shapes_and_error_codes(mutate, code):
    data = moved_candidate(observation(), [[0.] * 5]).model_dump()
    mutate(data)
    with pytest.raises(VLAError) as err:
        codec.validate(data, PolicyActionChunk)
    assert err.value.code == code


def test_strictly_increasing_times_and_native_dimensions_are_not_assumed():
    data = moved_candidate(observation(), [[0.] * 5, [0.] * 5]).model_dump()
    data["time_offsets_s"] = [0.1, 0.1]
    with pytest.raises(VLAError, match="increasing"):
        codec.validate(data, PolicyActionChunk)
    data.update(native_action_space="future_model_7d", shape=[1, 7], channel_names=[f"a{i}" for i in range(7)],
                units=["native"] * 7, values=[[0.] * 7], time_offsets_s=[0.1])
    assert codec.validate(data, PolicyActionChunk).shape == [1, 7]


def test_limits_and_duplicate_json_keys():
    obs = observation()
    with pytest.raises(VLAError) as err:
        codec.loads(b" " * 1025, Observation, RuntimeLimits(max_body_bytes=1024))
    assert err.value.code == "PAYLOAD_TOO_LARGE"
    with pytest.raises(VLAError, match="duplicate"):
        codec.loads('{"schema_version":1,"schema_version":1}', Observation)
    with pytest.raises(VLAError) as err:
        codec.loads('{"x":NaN}', Observation)
    assert err.value.code == "NONFINITE_NUMBER"
    with pytest.raises(VLAError):
        codec.validate(obs.model_dump(), Observation, RuntimeLimits(max_image_bytes=64))
    candidate = moved_candidate(obs, [[0.] * 5] * 2)
    with pytest.raises(VLAError):
        codec.validate(candidate.model_dump(), PolicyActionChunk, RuntimeLimits(max_horizon=1))


def test_png_crc_trailing_bytes_and_inflation_bomb_are_rejected():
    obs = observation().model_dump()
    original = base64.b64decode(obs["images"][0]["data_base64"])
    corrupt = bytearray(original)
    corrupt[-1] ^= 1
    bomb = SIGNATURE + chunk(b"IHDR", struct.pack("!IIBBBBB", 2, 2, 8, 2, 0, 0, 0))
    bomb += chunk(b"IDAT", zlib.compress(b"\0" * 1000000)) + chunk(b"IEND", b"")
    for raw in (bytes(corrupt), original + b"trailing", bomb):
        obs["images"][0].update(data_base64=base64.b64encode(raw).decode(), sha256=sha256(raw).hexdigest())
        with pytest.raises(VLAError):
            codec.validate(obs, Observation)
