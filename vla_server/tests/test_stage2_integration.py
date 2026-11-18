"""CPU regression tests with explicit doubles; never evidence of GPU inference."""
from hashlib import sha256
import json
from types import SimpleNamespace
import pytest
from fastapi.testclient import TestClient
from deployment.local.test_support import observation
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.errors import VLAError
from vla_server.app import create_app
from vla_server.artifacts import inventory, inventory_hash, verify_checkpoint
from vla_server.config import PolicyConfig, NativeActionSpec
from vla_server.deployment import DeploymentManifest
from vla_server.observation_adapter import ModelFeatureSpec, FeatureObservationAdapter
from vla_server.policies.fake import FakeNativePolicy
from vla_server.policies.smolvla import SmolVLAPolicyAdapter


def features():
    return ModelFeatureSpec(model_id="synthetic_test", processor_id="official_test",
        state_channels=[f"joint{i}" for i in range(1, 6)] + ["r_joint"], state_units=["rad"] * 6,
        state_dimension=6, cameras=[
            {"camera_id": "front", "feature_name": "observation.images.camera1", "shape_chw": [3, 2, 2]},
            {"camera_id": "side", "feature_name": "observation.images.camera2", "shape_chw": [3, 2, 2], "required": False}])


def test_optional_camera_is_omitted_and_unknown_camera_and_blank_task_are_rejected():
    adapter = FeatureObservationAdapter(features())
    bound = adapter.bind(observation())
    assert list(bound.images) == ["observation.images.camera1"]
    for obs, code in [
        (observation(instruction=" \t"), "INVALID_TASK"),
        (observation().model_copy(update={"images": [observation().images[0].model_copy(update={"camera_id": "unexpected"})]}), "CAMERA_MAPPING_UNKNOWN"),
    ]:
        with pytest.raises(VLAError) as error:
            adapter.bind(obs)
        assert error.value.code == code
    data = features().model_dump()
    data["cameras"] = [data["cameras"][1]]
    with pytest.raises(VLAError) as error:
        FeatureObservationAdapter(ModelFeatureSpec.model_validate(data)).bind(observation())
    assert error.value.code == "CAMERA_MAPPING_UNKNOWN"


def test_feature_validation_accepts_declared_subset_without_inventing_slots():
    native = NativeActionSpec(space_id="unknown_6d", channels=[f"native_{i}" for i in range(6)],
        units=["unknown"] * 6, semantics="unknown", step_s=0.02, processor_id="official_test",
        normalization_id="test_only", semantics_source="Explicit test fixture; no robot mapping")
    spec = features()
    adapter = SmolVLAPolicyAdapter(PolicyConfig(type="smolvla", feature_spec=spec, native_spec=native))
    cfg = SimpleNamespace(input_features={"observation.state": SimpleNamespace(shape=[6])},
        image_features={f"observation.images.camera{i}": SimpleNamespace(shape=[3, 2, 2]) for i in range(1, 4)},
        action_feature=SimpleNamespace(shape=[6]), chunk_size=50, resize_imgs_with_padding=[512, 512])
    adapter.model = SimpleNamespace(config=cfg)
    adapter._validate_features()
    cfg.image_features.pop("observation.images.camera1")
    with pytest.raises(VLAError) as error:
        adapter._validate_features()
    assert error.value.code == "MODEL_FEATURE_MISMATCH"


def test_model_observation_errors_are_422_and_model_failures_are_500():
    class ValidatedFake(FakeNativePolicy):
        def predict(self, obs):
            FeatureObservationAdapter(features()).bind(obs)
            if obs.instruction == "inject_failure":
                raise RuntimeError("explicit CPU test double failure")
            return super().predict(obs)
    policy = ValidatedFake(PolicyConfig(type="fake", fake_horizon=1))
    with TestClient(create_app(policy)) as client:
        for text, status, code in [(" ", 422, "INVALID_TASK"), ("inject_failure", 500, "POLICY_ERROR")]:
            response = client.post("/v1/infer", json=observation(instruction=text, capabilities=["native_actions_v1"]).model_dump())
            assert response.status_code == status and response.json()["code"] == code
        response = client.post("/v1/infer", json=observation(capabilities=["native_actions_v1"]).model_dump())
        assert response.status_code == 200
        assert "policy;dur=" in response.headers["server-timing"]
        assert "http;dur=" in response.headers["server-timing"]


def test_external_receipt_verifies_uploaded_assets_without_changing_model_directory(tmp_path):
    model = tmp_path / "model"
    model.mkdir()
    (model / "config.json").write_text("{}")
    (model / "model.safetensors").write_bytes(b"explicit hash fixture, not model weights")
    files = inventory(model)
    receipt = tmp_path / "receipt.json"
    receipt.write_text(json.dumps({"repository": "lerobot/smolvla_base", "revision": "b" * 40, "files": files}))
    assert verify_checkpoint(model, "lerobot/smolvla_base", "b" * 40, inventory_hash(files), receipt) == files
    assert not (model / "checkpoint_receipt.json").exists()
    (model / "config.json").write_text('{"modified":true}')
    with pytest.raises(VLAError) as error:
        verify_checkpoint(model, "lerobot/smolvla_base", "b" * 40, inventory_hash(files), receipt)
    assert error.value.code == "CHECKPOINT_INVALID"


def test_existing_venv_manifest_keeps_install_evidence_distinct_from_hash_locked_installation(tmp_path):
    freeze = tmp_path / "freeze.txt"
    freeze.write_text("torch==2.8.0+cu128\n")
    data = dict(cable_robot_revision="a" * 40, lerobot={"revision": "b" * 40},
        checkpoint={"revision": "c" * 40, "inventory_sha256": "d" * 64}, server_config="server.yaml",
        environment={"kind": "existing_venv", "requirements_format": "installed_freeze",
            "python_version": "3.12.3", "torch_version": "2.8.0+cu128", "cuda_runtime": "12.8",
            "requirements_lock": str(freeze), "requirements_sha256": sha256(freeze.read_bytes()).hexdigest()})
    manifest = DeploymentManifest.model_validate(data)
    manifest.require_resolved()
    assert manifest.verify_lock(tmp_path / "manifest.yaml") == freeze
    data["environment"]["kind"] = "container"
    with pytest.raises(VLAError):
        DeploymentManifest.model_validate(data).require_resolved()
    freeze.write_text("changed")
    with pytest.raises(VLAError):
        manifest.verify_lock(tmp_path / "manifest.yaml")


def test_unknown_real_native_semantics_remain_unexecutable():
    from cable_robot_vla.action_adapter import adapt_action
    from vla_server.policies.mock import HoldPositionPolicy
    action = HoldPositionPolicy().predict(observation()).model_copy(update={
        "synthetic": False, "semantics": "unknown", "native_action_space": "unmapped_model_native"})
    action = codec.validate(action.model_dump(), PolicyActionChunk)
    with pytest.raises(VLAError) as error:
        adapt_action(action)
    assert error.value.code == "UNSUPPORTED_ACTION_PROFILE"


@pytest.mark.parametrize("shape,value,code", [
    ((1, 49, 6), 0.0, "MODEL_ACTION_MISMATCH"),
    ((2, 50, 6), 0.0, "MODEL_ACTION_MISMATCH"),
    ((1, 50, 6), float("nan"), "MODEL_ACTION_NONFINITE"),
    ((1, 50, 6), float("inf"), "MODEL_ACTION_NONFINITE"),
])
def test_adapter_rejects_invalid_native_output_before_verification(shape, value, code):
    from contextlib import nullcontext
    native = NativeActionSpec(space_id="unmapped_test", channels=[f"native_{i}" for i in range(6)],
        units=["unknown"] * 6, semantics="unknown", step_s=0.02, processor_id="official_test",
        normalization_id="test_only", semantics_source="Explicit CPU boundary double")
    adapter = SmolVLAPolicyAdapter(PolicyConfig(type="smolvla", device="cpu", feature_spec=features(), native_spec=native))
    class Tensor:
        def detach(self): return self
        def cpu(self): return self
        def __getitem__(self, index): return self
        def tolist(self): return [[value] * 6 for _ in range(50)]
    tensor = Tensor()
    tensor.shape = shape
    adapter.model = SimpleNamespace(config=SimpleNamespace(chunk_size=50), predict_action_chunk=lambda _: tensor)
    adapter.model_loaded = True
    adapter.torch = SimpleNamespace(device=lambda x: x, inference_mode=nullcontext)
    adapter.adapter = FeatureObservationAdapter(features())
    adapter.prepare_observation = lambda frame, device, **kw: frame
    adapter.preprocess = adapter.postprocess = lambda x: x
    with pytest.raises(VLAError) as error:
        adapter.predict(observation())
    assert error.value.code == code
    assert not adapter.genuine_inference_verified
