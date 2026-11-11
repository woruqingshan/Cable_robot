"""Optional official-LeRobot binding. Never downloads or imports GPU code in Mock mode.

The calls here were reviewed against upstream ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2.
That is an API-review reference, not a verified deployment/checkpoint pairing.
"""
import importlib
import os
from pathlib import Path
import subprocess
import sys
from time import perf_counter
from uuid import uuid4
from vla_protocol.action import PolicyActionChunk, PolicyIdentity, NativeHorizon
from vla_protocol.observation import identity_fields
from vla_protocol.errors import VLAError
from ..policy_base import Policy
from ..observation_adapter import FeatureObservationAdapter
from ..artifacts import verify_checkpoint


class SmolVLAPolicyAdapter(Policy):
    model_type = "smolvla"

    def __init__(self, config):
        self.settings = config
        self.model = self.preprocess = self.postprocess = self.torch = None
        self.identity = PolicyIdentity(policy_type="smolvla", policy_version="official_lerobot_adapter_v1",
            framework_revision=config.lerobot_revision, checkpoint_revision=config.checkpoint_revision,
            processor_id=config.native_spec.processor_id if config.native_spec else "unresolved",
            normalization_id=config.native_spec.normalization_id if config.native_spec else "unresolved")
        self.native_action_spaces = [config.native_spec.space_id] if config.native_spec else []

    def _dependencies(self):
        try:
            torch = importlib.import_module("torch")
            lerobot = importlib.import_module("lerobot")
            policy = importlib.import_module("lerobot.policies.smolvla.modeling_smolvla").SmolVLAPolicy
            config = importlib.import_module("lerobot.policies.smolvla.configuration_smolvla").SmolVLAConfig
            processors = importlib.import_module("lerobot.policies.factory").make_pre_post_processors
            prepare = importlib.import_module("lerobot.policies.utils").prepare_observation_for_inference
        except ImportError as exc:
            raise VLAError("MODEL_DEPENDENCY_MISSING", "install the separately pinned LeRobot/SmolVLA environment; " + str(exc)) from exc
        return torch, lerobot, policy, config, processors, prepare

    def start(self):
        c = self.settings
        c.require_model_pins()
        verify_checkpoint(c.checkpoint_path, c.checkpoint_repo, c.checkpoint_revision, c.checkpoint_inventory_sha256)
        checkout = Path(c.lerobot_checkout).resolve()
        try:
            actual = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
            dirty = subprocess.check_output(["git", "-C", str(checkout), "status", "--porcelain", "--untracked-files=no"], text=True).strip()
        except (OSError, subprocess.CalledProcessError) as exc:
            raise VLAError("LEROBOT_CHECKOUT_MISSING", "explicit independent LeRobot checkout is unavailable") from exc
        if actual != c.lerobot_revision or dirty:
            raise VLAError("LEROBOT_REVISION_MISMATCH", "LeRobot checkout differs from the exact clean revision")
        if sys.version_info < (3, 12):
            raise VLAError("MODEL_PYTHON_INCOMPATIBLE", "the reviewed current LeRobot API needs its independent Python 3.12+ environment")
        os.environ["HF_HUB_OFFLINE"] = "1"
        os.environ["TRANSFORMERS_OFFLINE"] = "1"
        sys.path.insert(0, str(checkout / "src"))
        torch, lerobot, policy_class, config_class, make_processors, prepare = self._dependencies()
        if not Path(lerobot.__file__).resolve().is_relative_to(checkout / "src"):
            raise VLAError("LEROBOT_REVISION_MISMATCH", "imported LeRobot is not the pinned checkout")
        if c.device == "cuda" and not torch.cuda.is_available():
            raise VLAError("GPU_UNAVAILABLE", "CUDA requested but unavailable")
        try:
            cfg = config_class.from_pretrained(c.checkpoint_path, local_files_only=True, revision=c.checkpoint_revision)
            cfg.device = c.device
            if getattr(cfg, "rtc_config", None) is not None:
                raise VLAError("MODEL_CONFIGURATION_UNSUPPORTED", "RTC is outside Stage 1.5")
            model = policy_class.from_pretrained(c.checkpoint_path, config=cfg,
                local_files_only=True, revision=c.checkpoint_revision, strict=True)
            self.model = model.to(torch.device(c.device)).eval()
            self.torch, self.prepare_observation = torch, prepare
            self._validate_features()
            self.preprocess, self.postprocess = make_processors(self.model.config,
                pretrained_path=c.checkpoint_path, pretrained_revision=c.checkpoint_revision,
                preprocessor_overrides={"device_processor": {"device": c.device}})
            self.adapter = FeatureObservationAdapter(c.feature_spec)
            self.model_loaded = True
            self.gpu_loaded = c.device == "cuda"
            self.reset(reason="model_start")
        except VLAError:
            raise
        except Exception as exc:
            raise VLAError("MODEL_LOAD_FAILED", "official checkpoint/processor loading failed: " + str(exc)) from exc

    def _validate_features(self):
        c, cfg = self.settings, self.model.config
        if c.feature_spec.processor_id != c.native_spec.processor_id:
            raise VLAError("MODEL_FEATURE_MISMATCH", "input/output processor identities differ")
        state = cfg.input_features.get("observation.state")
        if state is None or list(state.shape) != [c.feature_spec.state_dimension]:
            raise VLAError("MODEL_FEATURE_MISMATCH", "checkpoint state shape differs; no padding is invented")
        expected = {f.feature_name: f for f in c.feature_spec.cameras}
        if set(expected) != set(cfg.image_features):
            raise VLAError("MODEL_FEATURE_MISMATCH", "camera feature names differ from checkpoint")
        for name, feature in cfg.image_features.items():
            mapped = expected[name]
            if list(feature.shape) != mapped.shape_chw:
                raise VLAError("MODEL_FEATURE_MISMATCH", "checkpoint image shape differs")
            if mapped.shape_policy == "official_processor" and cfg.resize_imgs_with_padding is None:
                raise VLAError("MODEL_FEATURE_MISMATCH", "no official checkpoint image resize policy is configured")
        if cfg.action_feature is None or list(cfg.action_feature.shape) != [len(c.native_spec.channels)]:
            raise VLAError("MODEL_ACTION_MISMATCH", "native declaration differs from effective checkpoint action feature")
        if not 1 <= cfg.chunk_size <= 256:
            raise VLAError("MODEL_ACTION_MISMATCH", "checkpoint horizon exceeds supported native wire ceiling")
        if cfg.chunk_size * c.native_spec.step_s > 10:
            raise VLAError("MODEL_ACTION_MISMATCH", "declared sample timing exceeds the native wire duration ceiling")

    def reset(self, context=None, reason="session_change"):
        if self.model is not None:
            self.model.reset()
        for processor in (self.preprocess, self.postprocess):
            if callable(getattr(processor, "reset", None)):
                processor.reset()

    def predict(self, observation):
        if not self.model_loaded:
            raise VLAError("POLICY_NOT_READY", "SmolVLA has not loaded")
        c, begin = self.settings, perf_counter()
        frame = self.adapter.official_frame(observation, self.prepare_observation, self.torch.device(c.device))
        batch = self.preprocess(frame)
        with self.torch.inference_mode():
            actions = self.model.predict_action_chunk(batch)
            actions = self.postprocess(actions)
        h, d = self.model.config.chunk_size, len(c.native_spec.channels)
        if tuple(actions.shape) != (1, h, d):
            raise VLAError("MODEL_ACTION_MISMATCH", "official model/postprocessor returned unexpected [B,H,D]")
        values = actions.detach().cpu()[0].tolist()
        result = PolicyActionChunk(**identity_fields(observation), action_id=str(uuid4()), policy=self.identity,
            native_action_space=c.native_spec.space_id, channel_names=c.native_spec.channels,
            units=c.native_spec.units, semantics=c.native_spec.semantics, shape=[h, d], values=values,
            time_offsets_s=[(i + 1) * c.native_spec.step_s for i in range(h)],
            horizon=NativeHorizon(generated=h, transmitted=h), inference_duration_s=perf_counter() - begin,
            synthetic=False)
        self.genuine_inference_verified = True
        return result

    def close(self):
        self.reset(reason="shutdown")
        self.model = self.preprocess = self.postprocess = None
        self.model_loaded = self.gpu_loaded = False
        if self.torch is not None and self.settings.device == "cuda" and self.torch.cuda.is_available():
            self.torch.cuda.empty_cache()
