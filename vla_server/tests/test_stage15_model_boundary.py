from copy import deepcopy
from pathlib import Path
import sys
import time
import pytest
from fastapi.testclient import TestClient
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.errors import VLAError
from vla_protocol.profiles import RuntimeLimits, JOINT_NAMES
from vla_server.app import create_app
from vla_server.config import ServerConfig, PolicyConfig, ServiceTiming, load_config
from vla_server.policy_factory import create_policy
from vla_server.observation_adapter import ModelFeatureSpec, FeatureObservationAdapter
from vla_server.policies.fake import FakeNativePolicy
from vla_server.policies.smolvla import SmolVLAPolicyAdapter
from deployment.local.test_support import observation


def features(**updates):
    data = dict(model_id="fake_model_v1", processor_id="fake_features_v1",
        state_channels=list(JOINT_NAMES) + ["r_joint"], state_units=["rad"] * 6, state_dimension=6,
        cameras=[{"camera_id": "front", "feature_name": "observation.images.front", "shape_chw": [3, 2, 2]}])
    data.update(updates)
    return ModelFeatureSpec.model_validate(data)


def test_feature_binding_is_named_and_does_no_model_preprocessing():
    obs = observation([0.1, 0.2, 0.3, 0.4, 0.5])
    adapter = FeatureObservationAdapter(features())
    bound = adapter.bind(obs)
    assert bound.state == (0.1, 0.2, 0.3, 0.4, 0.5, 0.3)
    assert bound.instruction == obs.instruction and bound.processor_id == "fake_features_v1"
    calls = []
    def official_prepare(frame, device, **kwargs):
        calls.append((frame, device, kwargs))
        return "official_result"
    assert adapter.official_frame(obs, official_prepare, "fake_device") == "official_result"
    assert calls[0][0]["observation.images.front"].shape == (2, 2, 3)
    assert calls[0][0]["observation.images.front"].dtype.name == "uint8"
    assert calls[0][2]["task"] == obs.instruction


@pytest.mark.parametrize("kind", ["missing_camera", "wrong_shape", "wrong_state_dimension", "wrong_units", "duplicate_features"])
def test_feature_spec_failures(kind):
    data = features().model_dump()
    if kind == "missing_camera": data["cameras"][0]["camera_id"] = "absent"
    if kind == "wrong_shape": data["cameras"][0]["shape_chw"] = [3, 3, 3]
    if kind == "wrong_state_dimension": data["state_dimension"] = 5
    if kind == "wrong_units": data["state_units"][0] = "degree"
    if kind == "duplicate_features": data["cameras"] *= 2
    with pytest.raises((VLAError, ValueError)):
        FeatureObservationAdapter(ModelFeatureSpec.model_validate(data)).bind(observation())


def test_mock_factory_is_gpu_independent_and_fake_native_is_inspectable():
    assert create_policy(PolicyConfig()).model_type == "mock"
    cfg = ServerConfig(policy=PolicyConfig(type="fake"), limits=RuntimeLimits(max_horizon=64))
    with TestClient(create_app(config=cfg)) as client:
        health = client.get("/health").json()
        assert health["http_service_available"] and health["policy_ready"]
        assert health["model_type"] == "fake" and not health["gpu_model_ready"] and not health["genuine_inference_verified"]
        response = client.post("/v1/infer", json=observation(capabilities=["native_actions_v1"]).model_dump())
        assert response.status_code == 200
        action = codec.loads(response.content, PolicyActionChunk, cfg.limits)
        assert action.shape == [50, 7] and action.horizon.generated == 50
        assert action.native_action_space == "fake_model_native_v1"


def test_stateful_policy_reset_idempotency_retired_sessions_and_task_revisions():
    policy = FakeNativePolicy(PolicyConfig(type="fake", fake_horizon=2))
    cfg = ServerConfig(policy=PolicyConfig(type="fake"))
    with TestClient(create_app(policy, config=cfg)) as client:
        obs = observation(session_id="first_session")
        def infer(value):
            value = value.model_copy(update={"capabilities": ["native_actions_v1", "session_state_v1"]})
            response = client.post("/v1/infer", json=value.model_dump())
            assert response.status_code == 200, response.text
            return response.json()
        first = infer(obs)
        assert infer(obs)["values"] == first["values"] and policy.counter == 1
        second_obs = observation(session_id="first_session", observation_seq=1)
        second = infer(second_obs)
        assert second["values"][0][0] == 2.0 and second["session_generation"] == first["session_generation"]
        changed = infer(observation(session_id="first_session", observation_seq=2, task_revision=1))
        assert changed["values"][0][0] == 1.0 and changed["session_generation"] > second["session_generation"]
        stale = client.post("/v1/infer", json=obs.model_dump())
        assert stale.status_code == 409 and stale.json()["code"] == "REQUEST_INVALIDATED"
        reset = infer(observation(session_id="reset_session", observation_seq=0))
        assert reset["values"][0][0] == 1.0
        assert client.post("/v1/infer", json=second_obs.model_dump()).json()["code"] == "SESSION_RETIRED"


def test_timeout_discards_output_resets_temporal_state_and_closes_serially():
    policy = FakeNativePolicy(PolicyConfig(type="fake", fake_horizon=1, fake_delay_s=0.06))
    cfg = ServerConfig(timing=ServiceTiming(inference_timeout_s=0.02))
    with TestClient(create_app(policy, config=cfg)) as client:
        first = observation(session_id="timeout_session")
        first = first.model_copy(update={"capabilities": ["native_actions_v1", "session_state_v1"]})
        result = client.post("/v1/infer", json=first.model_dump())
        assert result.status_code == 504
        time.sleep(0.08)
        policy.config = policy.config.model_copy(update={"fake_delay_s": 0.0})
        next_obs = observation(session_id="timeout_session", observation_seq=1, capabilities=["native_actions_v1", "session_state_v1"])
        result = client.post("/v1/infer", json=next_obs.model_dump())
        assert result.status_code == 200, result.text
        assert result.json()["values"][0][0] == 1.0
        assert "late_or_invalidated_request" in policy.reset_reasons


def test_smolvla_unresolved_is_not_ready_and_never_falls_back_to_mock():
    cfg = ServerConfig(policy=PolicyConfig(type="smolvla"))
    with TestClient(create_app(config=cfg)) as client:
        health = client.get("/health").json()
        assert health["http_service_available"] and not health["policy_ready"]
        assert health["model_type"] == "smolvla" and not health["gpu_model_ready"]
        assert health["startup_error"]["code"] == "DEPLOYMENT_UNRESOLVED"
        assert client.post("/v1/infer", json=observation().model_dump()).status_code == 503


def test_missing_checkpoint_and_missing_dependencies_are_explicit(tmp_path, monkeypatch):
    from vla_server.config import NativeActionSpec
    native = NativeActionSpec(space_id="unverified_model", channels=["a"], units=["native"], semantics="absolute", step_s=0.02,
        processor_id="fake_features_v1", normalization_id="unknown", semantics_source="explicit unit fixture")
    cfg = PolicyConfig(type="smolvla", lerobot_checkout=str(tmp_path), lerobot_revision="a" * 40,
        checkpoint_path=str(tmp_path / "missing"), checkpoint_revision="b" * 40, checkpoint_inventory_sha256="c" * 64,
        feature_spec=features(), native_spec=native)
    adapter = SmolVLAPolicyAdapter(cfg)
    with pytest.raises(VLAError) as err: adapter.start()
    assert err.value.code == "CHECKPOINT_MISSING"
    import importlib
    def missing(name): raise ModuleNotFoundError("explicit missing GPU dependency")
    monkeypatch.setattr(importlib, "import_module", missing)
    with pytest.raises(VLAError) as err: adapter._dependencies()
    assert err.value.code == "MODEL_DEPENDENCY_MISSING"


def test_strict_policy_configuration(tmp_path):
    for content in ("policy: {type: unsupported}\n", "port: 8000\nport: 8001\n", "host: 0.0.0.0\n", "policy: {type: smolvla, lerobot_revision: main}\n"):
        path = tmp_path / "bad.yaml"
        path.write_text(content)
        with pytest.raises(VLAError): load_config(path)


def test_official_prediction_boundary_uses_whole_chunk_and_processors_with_explicit_spies():
    """Binding test doubles only: this is NOT SmolVLA inference evidence."""
    from contextlib import nullcontext
    from types import SimpleNamespace
    from vla_server.config import NativeActionSpec
    calls = []
    native = NativeActionSpec(space_id="declared_model_7d", channels=[f"a{i}" for i in range(7)],
        units=["model_native"] * 7, semantics="absolute", step_s=0.02,
        processor_id="fake_features_v1", normalization_id="test_stats_only", semantics_source="explicit test fixture, no checkpoint")
    adapter = SmolVLAPolicyAdapter(PolicyConfig(type="smolvla", device="cpu", feature_spec=features(), native_spec=native))
    class TensorSpy:
        shape = (1, 50, 7)
        def detach(self): return self
        def cpu(self): return self
        def __getitem__(self, index):
            assert index == 0
            return self
        def tolist(self): return [[1.25] * 7 for _ in range(50)]
    class ModelSpy:
        config = SimpleNamespace(chunk_size=50)
        def predict_action_chunk(self, batch):
            calls.append(("predict_action_chunk", batch))
            return TensorSpy()
        def reset(self): calls.append(("reset", None))
    def prepare(frame, device, **kwargs):
        calls.append(("official_prepare", kwargs))
        return {"prepared_by_upstream_spy": frame}
    def preprocess(frame): calls.append(("preprocess", None)); return frame
    def postprocess(actions): calls.append(("postprocess", None)); return actions
    adapter.model, adapter.model_loaded = ModelSpy(), True
    adapter.torch = SimpleNamespace(device=lambda value: value, inference_mode=nullcontext)
    adapter.adapter = FeatureObservationAdapter(features())
    adapter.prepare_observation, adapter.preprocess, adapter.postprocess = prepare, preprocess, postprocess
    action = adapter.predict(observation())
    assert action.shape == [50, 7] and action.horizon.generated == 50
    assert action.values == [[1.25] * 7] * 50 and action.synthetic is False
    assert [name for name, _ in calls[:4]] == ["official_prepare", "preprocess", "predict_action_chunk", "postprocess"]
    adapter.close()
    assert adapter.model is None and adapter.model_loaded is False


def test_default_wire_remains_readable_by_the_strict_stage1_peer(tmp_path):
    import json
    import os
    import subprocess
    root = Path(__file__).resolve().parents[2]
    old = tmp_path / "old_peer"
    package = old / "vla_protocol"
    package.mkdir(parents=True)
    reference = "b48c99e5bc1affcbddd4cfaee9f5fe4e966c0964"
    paths = subprocess.check_output(["git", "ls-tree", "-r", "--name-only", reference, "vla_protocol/src/vla_protocol"], cwd=root, text=True).splitlines()
    for path in paths:
        (package / Path(path).name).write_bytes(subprocess.check_output(["git", "show", reference + ":" + path], cwd=root))
    obs = observation()
    (tmp_path / "request.json").write_bytes(codec.dumps(obs))
    with TestClient(create_app()) as client:
        result = client.post("/v1/infer", content=codec.dumps(obs), headers={"Content-Type": "application/json"})
        assert result.status_code == 200
        (tmp_path / "response.json").write_bytes(result.content)
    code = "from pathlib import Path; from vla_protocol.codec import loads; from vla_protocol.observation import Observation; from vla_protocol.action import PolicyActionChunk; "
    code += f"assert loads(Path({str(tmp_path / 'request.json')!r}).read_bytes(),Observation).simulated; "
    code += f"assert loads(Path({str(tmp_path / 'response.json')!r}).read_bytes(),PolicyActionChunk).shape==[1,5]"
    env = dict(os.environ, PYTHONPATH=str(old), PYTHONDONTWRITEBYTECODE="1")
    subprocess.run([sys.executable, "-B", "-c", code], cwd=tmp_path, env=env, check=True)


def test_warmup_is_explicit_validated_and_does_not_leak_into_session(tmp_path):
    warmup = tmp_path / "warmup.json"
    warmup.write_bytes(codec.dumps(observation()))
    policy = FakeNativePolicy(PolicyConfig(type="fake", fake_horizon=1))
    cfg = ServerConfig(timing=ServiceTiming(warmup_observation_path=str(warmup)))
    with TestClient(create_app(policy, config=cfg)) as client:
        health = client.get("/health").json()
        assert health["warmup_verified"] and health["policy_ready"] and not health["gpu_model_ready"]
        obs = observation(capabilities=["native_actions_v1"])
        action = client.post("/v1/infer", json=obs.model_dump())
        assert action.status_code == 200 and action.json()["values"][0][0] == 1.0
        assert policy.reset_reasons[:2] == ["warmup_start", "warmup_end"]


def test_startup_timeout_retains_http_health_and_waits_for_resource_cleanup():
    from vla_server.policies.mock import HoldPositionPolicy
    class SlowStart(HoldPositionPolicy):
        active = False
        closed = False
        def start(self):
            self.active = True
            time.sleep(0.05)
            self.active = False
        def close(self):
            assert not self.active
            self.closed = True
    policy = SlowStart()
    with TestClient(create_app(policy, config=ServerConfig(timing=ServiceTiming(startup_timeout_s=0.01)))) as client:
        health = client.get("/health").json()
        assert health["http_service_available"] and not health["policy_ready"]
        assert health["startup_error"]["code"] == "POLICY_STARTUP_TIMEOUT"
        assert client.post("/v1/infer", json=observation().model_dump()).status_code == 503
    assert policy.closed


def test_concurrent_http_requests_still_use_one_serial_policy_owner():
    from concurrent.futures import ThreadPoolExecutor
    class TrackedFake(FakeNativePolicy):
        active = maximum = 0
        def predict(self, obs):
            self.active += 1
            self.maximum = max(self.maximum, self.active)
            try: return super().predict(obs)
            finally: self.active -= 1
    policy = TrackedFake(PolicyConfig(type="fake", fake_horizon=1, fake_delay_s=0.03))
    with TestClient(create_app(policy)) as client:
        def infer(i):
            return client.post("/v1/infer", json=observation(session_id=f"session_{i}", capabilities=["native_actions_v1"]).model_dump())
        with ThreadPoolExecutor(max_workers=2) as workers:
            responses = list(workers.map(infer, [0, 1]))
        assert [r.status_code for r in responses] == [200, 200]
        assert policy.maximum == 1 and policy.active == 0
