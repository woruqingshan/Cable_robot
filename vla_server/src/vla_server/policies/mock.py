"""A deterministic hold-position policy; no arbitrary movement mode."""
from uuid import NAMESPACE_URL, uuid5
from vla_protocol.action import PolicyActionChunk, PolicyIdentity
from vla_protocol.observation import Observation, identity_fields
from vla_protocol.profiles import JOINT_NAMES, ROBOT_PROFILE
from ..policy_base import Policy


class HoldPositionPolicy(Policy):
    identity = PolicyIdentity(policy_type="mock_hold", policy_version="1.0.0",
                              framework_revision=None, checkpoint_revision=None,
                              processor_id="mock_identity_v1", normalization_id="none")
    native_action_spaces = [ROBOT_PROFILE]

    def predict(self, observation: Observation) -> PolicyActionChunk:
        return PolicyActionChunk(**identity_fields(observation),
            action_id=str(uuid5(NAMESPACE_URL, observation.session_id + "/" + observation.request_id)),
            policy=self.identity, native_action_space=ROBOT_PROFILE,
            channel_names=list(JOINT_NAMES), units=["rad"] * 5, semantics="absolute",
            shape=[1, 5], values=[list(observation.joints.positions)], time_offsets_s=[0.1],
            inference_duration_s=0.0, synthetic=True)
