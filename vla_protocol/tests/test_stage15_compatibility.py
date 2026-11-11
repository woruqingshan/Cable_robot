from copy import deepcopy
import pytest
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk, NativeHorizon
from vla_protocol.observation import Observation
from vla_protocol.profiles import RuntimeLimits
from vla_protocol.errors import VLAError
from deployment.local.test_support import observation, moved_candidate


@pytest.mark.parametrize("profile,calibrated", [("real_uncalibrated_v1", False), ("real_calibrated_v1", True)])
def test_explicit_real_source_profiles(profile, calibrated):
    data = observation().model_dump()
    data.update(simulated=False, calibration_status=profile.removesuffix("_v1"),
        capabilities=["source_profiles_v1"],
        source_profile={"profile_id": profile, "calibration_id": "calibration_01" if calibrated else None,
                        "calibration_sha256": "a" * 64 if calibrated else None})
    data["gripper"].update(simulated=False, calibrated=calibrated)
    result = codec.validate(data, Observation)
    assert not result.simulated and result.source_profile.profile_id == profile
    assert codec.loads(codec.dumps(result), Observation) == result


@pytest.mark.parametrize("change", ["missing_profile", "missing_evidence", "synthetic_relabel", "gripper_disagrees", "uncalibrated_evidence"])
def test_calibration_cannot_be_silently_relabelled(change):
    data = observation().model_dump()
    if change == "missing_profile": data.update(simulated=False, calibration_status="real_uncalibrated")
    if change == "missing_evidence": data.update(simulated=False, calibration_status="real_calibrated", source_profile={"profile_id": "real_calibrated_v1"})
    if change == "synthetic_relabel": data.update(source_profile={"profile_id": "real_uncalibrated_v1"})
    if change == "gripper_disagrees": data["gripper"]["calibrated"] = True
    if change == "uncalibrated_evidence": data.update(source_profile={"profile_id": "synthetic_uncalibrated_v1", "calibration_id": "invented", "calibration_sha256": "a" * 64})
    with pytest.raises(VLAError): codec.validate(data, Observation)


def test_long_native_chunk_requires_metadata_and_configured_limit():
    data = moved_candidate(observation(), [[0.] * 5]).model_dump()
    data.update(shape=[50, 7], values=[[0.] * 7] * 50,
        native_action_space="model_native_7d", channel_names=[f"native_{i}" for i in range(7)],
        units=["native"] * 7, time_offsets_s=[0.02 * (i + 1) for i in range(50)])
    with pytest.raises(VLAError): codec.validate(data, PolicyActionChunk, RuntimeLimits(max_horizon=64))
    data["horizon"] = {"generated": 50, "transmitted": 50, "transmission": "full"}
    with pytest.raises(VLAError): codec.validate(data, PolicyActionChunk)
    result = codec.validate(data, PolicyActionChunk, RuntimeLimits(max_horizon=64))
    assert result.shape == [50, 7] and result.horizon.generated == 50
    data["horizon"]["transmitted"] = 32
    with pytest.raises(VLAError): codec.validate(data, PolicyActionChunk, RuntimeLimits(max_horizon=64))


def test_stage1_profile_roundtrips_unchanged_without_new_fields():
    data = observation().model_dump(exclude={"source_profile"})
    result = codec.validate(data, Observation)
    assert result.source_profile is None and result.simulated
    candidate = moved_candidate(result, [[0.] * 5])
    legacy = candidate.model_dump(exclude={"horizon", "session_generation"})
    assert codec.validate(legacy, PolicyActionChunk).shape == [1, 5]
    import json
    wire = json.loads(codec.dumps(result))
    assert "source_profile" not in wire and "capabilities" not in wire
    action = json.loads(codec.dumps(candidate))
    assert "horizon" not in action and "session_generation" not in action
    assert "framework_revision" in action["policy"] and action["policy"]["framework_revision"] is None
