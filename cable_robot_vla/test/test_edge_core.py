from copy import deepcopy
import json
import time
from types import SimpleNamespace
import httpx
import pytest
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk, ValidatedTrajectory
from vla_protocol.errors import VLAError
from vla_protocol.profiles import RuntimeLimits
from vla_server.policies.mock import HoldPositionPolicy
from cable_robot_vla.action_adapter import adapt_action
from cable_robot_vla.policy_client import PolicyClient, InferenceResult
from cable_robot_vla.mock_executor import MockExecutor
from cable_robot_vla.adapters.p2 import P2Adapter, P2Config
from cable_robot_vla.observation_node import ObservationAdapter, StateMetadata
from cable_robot_vla.recording import LoopRecorder
from cable_robot_vla.simulation import SimulationState
from deployment.local.test_support import observation, moved_candidate


def poll(client):
    end = time.monotonic() + 3
    while time.monotonic() < end:
        result = client.poll()
        if result is not None:
            return result
        time.sleep(0.005)
    raise AssertionError("client did not finish")


def response_for(request):
    from vla_protocol.observation import Observation
    obs = codec.loads(request.content, Observation)
    return httpx.Response(200, content=codec.dumps(HoldPositionPolicy().predict(obs)), headers={"Content-Type": "application/json"})


def test_async_client_actual_codec_and_one_outstanding_request():
    def delayed(request):
        time.sleep(0.05)
        return response_for(request)
    client = PolicyClient(transport=httpx.MockTransport(delayed))
    try:
        obs = observation()
        begin = time.monotonic()
        client.submit(obs)
        assert time.monotonic() - begin < 0.04
        with pytest.raises(VLAError) as err:
            client.submit(observation())
        assert err.value.code == "CLIENT_BUSY"
        result = poll(client)
        assert result.candidate.values == [obs.joints.positions]
        with pytest.raises(VLAError) as err:
            client.correlate(obs, result.candidate)
        assert err.value.code == "DUPLICATE_RESPONSE"
    finally:
        client.close()


@pytest.mark.parametrize("field", ["request_id", "session_id", "task_revision", "observation_seq", "scene_sha256", "model_id", "state_revision"])
def test_response_identity_mismatch(field):
    obs = observation()
    def wrong(request):
        data = HoldPositionPolicy().predict(obs).model_dump()
        data[field] = data[field] + 1 if type(data[field]) is int else "f" * 64 if field == "scene_sha256" else "wrong"
        return httpx.Response(200, json=data)
    client = PolicyClient(transport=httpx.MockTransport(wrong))
    try:
        client.submit(obs)
        with pytest.raises(VLAError) as err:
            poll(client)
        assert err.value.code == "IDENTITY_MISMATCH"
    finally:
        client.close()


@pytest.mark.parametrize("mode,code", [("unavailable", "NETWORK_ERROR"), ("timeout", "TIMEOUT"),
    ("malformed", "INVALID_JSON"), ("oversize", "PAYLOAD_TOO_LARGE"), ("http_error", "SERVER_ERROR"),
    ("compressed", "MALFORMED_RESPONSE")])
def test_transport_failures(mode, code):
    def handler(request):
        if mode == "unavailable": raise httpx.ConnectError("unavailable", request=request)
        if mode == "timeout": raise httpx.ReadTimeout("timed out", request=request)
        if mode == "malformed": return httpx.Response(200, content=b"{", headers={"Content-Type": "application/json"})
        if mode == "oversize": return httpx.Response(200, content=b" " * 6_000_001)
        if mode == "compressed": return httpx.Response(200, content=b"", headers={"Content-Encoding": "gzip"})
        return httpx.Response(500, json={"schema_version": 1, "code": "POLICY_ERROR", "message": "failed", "request_id": "test"})
    client = PolicyClient(transport=httpx.MockTransport(handler))
    try:
        client.submit(observation())
        with pytest.raises(VLAError) as err:
            poll(client)
        assert err.value.code == code
        assert not client.busy
    finally:
        client.close()


@pytest.mark.parametrize("invalidate", [True, False])
def test_task_invalidation_and_late_response(invalidate):
    now = [0.]
    def slow(request):
        time.sleep(0.05)
        return response_for(request)
    client = PolicyClient(transport=httpx.MockTransport(slow), clock=lambda: now[0])
    try:
        client.submit(observation())
        if invalidate:
            client.invalidate()
        else:
            now[0] = 3.
        with pytest.raises(VLAError) as err:
            poll(client)
        assert err.value.code in ("REQUEST_INVALIDATED", "STALE_RESPONSE", "TIMEOUT")
    finally:
        client.close()


@pytest.mark.parametrize("changes,code", [
    ({"native_action_space": "other_5d"}, "UNSUPPORTED_ACTION_PROFILE"),
    ({"semantics": "delta"}, "INCOMPATIBLE_ACTION_SEMANTICS"),
    ({"synthetic": False}, "INCOMPATIBLE_ACTION_SEMANTICS"),
    ({"units": ["degree"] * 5}, "INCOMPATIBLE_ACTION_SEMANTICS")])
def test_no_implicit_native_mapping(changes, code):
    data = HoldPositionPolicy().predict(observation()).model_dump()
    data.update(changes)
    with pytest.raises(VLAError) as err:
        adapt_action(PolicyActionChunk.model_validate(data))
    assert err.value.code == code


def test_normalization_and_seven_dimensional_model_outputs_blocked():
    data = HoldPositionPolicy().predict(observation()).model_dump()
    data["policy"]["normalization_id"] = "unverified_stats"
    with pytest.raises(VLAError) as err:
        adapt_action(PolicyActionChunk.model_validate(data))
    assert err.value.code == "UNSUPPORTED_NORMALIZATION"
    data.update(native_action_space="model_7d", shape=[1, 7], channel_names=[f"a{i}" for i in range(7)], units=["native"] * 7, values=[[0.] * 7])
    with pytest.raises(VLAError):
        adapt_action(PolicyActionChunk.model_validate(data))


def prepare(obs=None, waypoints=None, config=None):
    obs = obs or observation()
    inference = InferenceResult(obs, moved_candidate(obs, waypoints or [[0.01] * 5]), time.monotonic(), 0., 0.)
    executor = MockExecutor(P2Adapter(config), advance_simulation=True, allow_unvalidated=True)
    return executor, inference, executor.prepare(inference)


def test_disabled_is_not_checked_and_simulation_requires_explicit_opt_in():
    executor, inference, result = prepare()
    assert result.status == "NOT_CHECKED" and not result.discrete_only
    assert not result.hardware_validated and not result.continuous_collision_proven
    executor.allow_unvalidated = False
    assert executor.consume(result, inference, inference.observation, time.monotonic(), 5.) == "BLOCKED_UNVALIDATED"


@pytest.mark.parametrize("change", ["task", "session", "state", "scene", "gripper", "position"])
def test_current_state_recheck_blocks_changes(change):
    executor, inference, result = prepare()
    data = inference.observation.model_dump()
    if change == "task": data["task_revision"] += 1
    if change == "session": data["session_id"] = "new"
    if change == "state": data["state_revision"] += 1
    if change == "scene": data["scene_sha256"] = "f" * 64
    if change == "gripper": data["gripper"]["position"] += 0.01
    if change == "position": data["joints"]["positions"][0] += 0.01
    with pytest.raises(VLAError, match="changed"):
        executor.consume(result, inference, type(inference.observation).model_validate(data), time.monotonic(), 5.)


def test_stale_and_duplicate_candidates_cannot_advance():
    executor, inference, result = prepare()
    with pytest.raises(VLAError) as err:
        executor.consume(result, inference, inference.observation, inference.submitted_at + 6, 5.)
    assert err.value.code == "STALE_ACTION"
    assert executor.consume(result, inference, inference.observation, inference.submitted_at, 5.) == "SIMULATE_UNVALIDATED"
    with pytest.raises(VLAError) as err:
        executor.consume(result, inference, inference.observation, inference.submitted_at, 5.)
    assert err.value.code == "DUPLICATE_ACTION"


def test_required_missing_p2_blocks_and_does_not_update_state():
    executor, inference, result = prepare(config=P2Config(mode="required"))
    assert result.status == "ERROR"
    assert result.failure_reasons == ["P2_RESOURCES_MISSING"]
    assert executor.consume(result, inference, inference.observation, time.monotonic(), 5.) == "REJECTED"
    state = SimulationState([0.] * 5)
    with pytest.raises(VLAError):
        state.accept(result, time.monotonic(), time.time_ns(), 5.)
    assert state.q == [0.] * 5


def test_waypoint_timing_state_updates_reset_and_task_cancel():
    state = SimulationState([0.] * 5, allow_unvalidated=True)
    obs = observation(session_id=state.session_id)
    executor, inference, result = prepare(obs, [[0.01] * 5, [0.02] * 5])
    now = time.monotonic()
    state.accept(result, now, obs.joints.stamp.nanoseconds, 5.)
    assert state.tick(now + 0.05) is None and state.q == [0.] * 5
    assert state.tick(now + 0.15) is None and state.q == [0.01] * 5
    event = state.tick(now + 0.3)
    assert event["status"] == "SIMULATED_UNVALIDATED" and state.q == [0.02] * 5
    with pytest.raises(VLAError): state.accept(result, now, obs.joints.stamp.nanoseconds, 5.)
    state.set_instruction("Remove the cable")
    assert state.task_revision == 1 and state.active is None
    old_session = state.session_id
    state.reset()
    assert old_session != state.session_id and state.q == [0.] * 5


def test_consistent_snapshot_age_skew_and_clock_policy():
    obs = observation()
    now = [0.]
    collector = ObservationAdapter(clock=lambda: now[0])
    metadata = StateMetadata(schema_version=1, session_id=obs.session_id, task_revision=obs.task_revision,
        state_revision=obs.state_revision, robot_id=obs.robot_id, scene_id=obs.scene_id, scene_sha256=obs.scene_sha256,
        model_id=obs.model_id, instruction=obs.instruction, stamp=obs.joints.stamp, busy=False)
    with pytest.raises(VLAError): collector.snapshot(0, obs.joints.stamp.nanoseconds)
    for kind, value in (("metadata", metadata), ("joints", obs.joints), ("gripper", obs.gripper), ("image", obs.images[0])):
        collector.offer(kind, value)
    assert collector.snapshot(1, obs.joints.stamp.nanoseconds).observation_seq == 1
    image = obs.images[0].model_copy(update={"stamp": obs.joints.stamp.model_copy(update={"nanoseconds": obs.joints.stamp.nanoseconds - 100_000_000})})
    collector.offer("image", image)
    with pytest.raises(VLAError) as err: collector.snapshot(1, obs.joints.stamp.nanoseconds)
    assert err.value.code == "OBSERVATION_SKEW"
    collector.offer("image", obs.images[0])
    now[0] = 1.
    with pytest.raises(VLAError) as err: collector.snapshot(1, obs.joints.stamp.nanoseconds)
    assert err.value.code == "STALE_OBSERVATION"


def test_jsonl_contains_image_hash_but_no_base64_and_refuses_overwrite(tmp_path):
    path = tmp_path / "loop.jsonl"
    recorder = LoopRecorder(path)
    obs = observation()
    recorder.observation(obs)
    recorder.close()
    row = json.loads(path.read_text())
    assert row["images"][0]["sha256"] == obs.images[0].sha256
    assert "data_base64" not in path.read_text()
    with pytest.raises(FileExistsError): LoopRecorder(path)
