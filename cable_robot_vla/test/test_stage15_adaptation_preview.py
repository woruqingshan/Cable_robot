import json
from copy import deepcopy
from pathlib import Path
import pytest
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk, ValidatedTrajectory
from vla_protocol.errors import VLAError
from vla_protocol.profiles import RuntimeLimits
from cable_robot_vla.action_adapter import adapt_action, ActionAdapterRegistry, ExecutionSelection
from cable_robot_vla.visualization.playback import TrajectoryPlayback, load_record
from deployment.local.make_preview_fixture import synthetic_fixture
from deployment.local.test_support import observation, moved_candidate


def test_registry_describes_all_mapping_requirements():
    mapping = ActionAdapterRegistry().describe()[0]
    assert mapping["native_space"] == "jetrover_joint_abs_rad_v1"
    assert mapping["calibration_requirement"] == "synthetic_only"
    assert mapping["processor_id"] == "mock_identity_v1"
    assert mapping["normalization_id"] == "none"
    assert mapping["semantics"] == "absolute"


def test_explicit_local_prefix_records_three_horizons_without_resampling():
    data = moved_candidate(observation(), [[0.01] * 5]).model_dump()
    data.update(shape=[50, 5], values=[[i * 0.001] * 5 for i in range(50)],
        time_offsets_s=[0.02 * (i + 1) for i in range(50)], horizon={"generated": 50, "transmitted": 50, "transmission": "full"})
    candidate = codec.validate(data, PolicyActionChunk, RuntimeLimits(max_horizon=64))
    with pytest.raises(VLAError) as err: adapt_action(candidate)
    assert err.value.code == "EXECUTION_HORIZON_UNSUPPORTED"
    result = adapt_action(candidate, selection=ExecutionSelection(mode="prefix", steps=8))
    assert result.shape == [8, 5] and result.values == candidate.values[:8]
    assert result.time_offsets_s == candidate.time_offsets_s[:8]
    assert result.horizon.model_dump() == {"generated": 50, "transmitted": 50, "selected": 8, "selection": "prefix"}
    with pytest.raises(VLAError): adapt_action(candidate, target_profile="other_robot")
    with pytest.raises((VLAError, ValueError)): ExecutionSelection(mode="prefix")


def test_unmapped_native_is_structurally_valid_but_never_executable():
    from vla_server.policy_factory import create_policy
    from vla_server.config import PolicyConfig
    native = create_policy(PolicyConfig(type="fake")).predict(observation())
    assert codec.validate(native.model_dump(), PolicyActionChunk, RuntimeLimits(max_horizon=64)).shape == [50, 7]
    with pytest.raises(VLAError) as err: adapt_action(native)
    assert err.value.code == "UNSUPPORTED_ACTION_PROFILE"


def test_synthetic_preview_timing_and_provenance_are_read_only():
    fixture = synthetic_fixture()
    with pytest.raises(VLAError): TrajectoryPlayback(fixture)
    playback = TrajectoryPlayback(fixture, allow_synthetic=True)
    assert playback.at(0.) == (0.,) * 5
    assert playback.at(1.) == tuple(fixture.trajectory.values[0])
    assert playback.at(1.5) == pytest.approx(tuple((a + b) / 2 for a, b in zip(fixture.trajectory.values[0], fixture.trajectory.values[1])))
    assert playback.at(4.) == tuple(fixture.trajectory.values[-1])
    assert playback.record == fixture and playback.label == "SYNTHETIC / NOT_CHECKED"
    for time in (-1., float("nan"), float("inf"), True):
        with pytest.raises(VLAError): playback.at(time)


def test_offline_record_selection_and_raw_native_rejection(tmp_path):
    fixture = synthetic_fixture()
    path = tmp_path / "fixture.json"
    path.write_bytes(codec.dumps(fixture))
    assert load_record(path) == fixture
    logs = tmp_path / "loop.jsonl"
    logs.write_text(json.dumps({"event": "execution", "validated_trajectory": fixture.model_dump()}) + "\n")
    assert load_record(logs) == fixture
    second = synthetic_fixture()
    with logs.open("a") as stream: stream.write(json.dumps({"validated_trajectory": second.model_dump()}) + "\n")
    with pytest.raises(VLAError, match="select"): load_record(logs)
    assert load_record(logs, action_id=second.source_action_id) == second
    path.write_bytes(codec.dumps(moved_candidate(observation(), [[0.] * 5])))
    with pytest.raises(VLAError): load_record(path)


@pytest.mark.parametrize("change", ["order", "units", "timing", "nonfinite", "rejected"])
def test_invalid_trajectory_cannot_enter_preview(change):
    data = synthetic_fixture().model_dump()
    if change == "order": data["trajectory"]["channel_names"].reverse()
    if change == "units": data["trajectory"]["units"] = ["degree"] * 5
    if change == "timing": data["trajectory"]["time_offsets_s"] = [1., 1., 2.]
    if change == "nonfinite": data["trajectory"]["values"][0][0] = float("nan")
    if change == "rejected": data.update(status="REJECTED", trajectory=None, failure_reasons=["explicit rejection"])
    with pytest.raises(VLAError): TrajectoryPlayback(codec.validate(data, ValidatedTrajectory), allow_synthetic=True)
