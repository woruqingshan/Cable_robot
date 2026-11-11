"""Clearly synthetic model-native producer for compatibility/lifecycle testing."""
from time import sleep
from uuid import uuid4
from vla_protocol.action import PolicyActionChunk, PolicyIdentity, NativeHorizon
from vla_protocol.observation import identity_fields
from ..policy_base import Policy
from ..observation_adapter import FeatureObservationAdapter


class FakeNativePolicy(Policy):
    model_type = "fake"

    def __init__(self, config):
        self.config = config
        self.counter = 0
        self.reset_reasons = []
        self.adapter = FeatureObservationAdapter(config.feature_spec) if config.feature_spec else None
        self.identity = PolicyIdentity(policy_type="fake_native", policy_version="1.0.0",
            framework_revision=None, checkpoint_revision=None,
            processor_id="fake_features_v1", normalization_id=config.fake_normalization_id)
        self.native_action_spaces = ["fake_model_native_v1"]

    def reset(self, context=None, reason="session_change"):
        self.counter = 0
        self.reset_reasons.append(reason)

    def predict(self, observation):
        if self.adapter:
            self.adapter.bind(observation)
        sleep(self.config.fake_delay_s)
        self.counter += 1
        h, d = self.config.fake_horizon, self.config.fake_dimension
        return PolicyActionChunk(**identity_fields(observation), action_id=str(uuid4()), policy=self.identity,
            native_action_space="fake_model_native_v1", channel_names=[f"native_{i}" for i in range(d)],
            units=["model_native"] * d, semantics=self.config.fake_semantics, shape=[h, d],
            values=[[float(self.counter) + i * 0.001 + j * 0.00001 for j in range(d)] for i in range(h)],
            time_offsets_s=[0.02 * (i + 1) for i in range(h)], horizon=NativeHorizon(generated=h, transmitted=h),
            inference_duration_s=0.0, synthetic=True)
