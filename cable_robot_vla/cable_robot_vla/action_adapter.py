"""Explicit synthetic mapping, never dimension-based inference of semantics."""
from uuid import uuid4
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk, RobotActionChunk
from vla_protocol.observation import identity_fields
from vla_protocol.profiles import ROBOT_PROFILE, JOINT_NAMES
from vla_protocol.errors import VLAError


def adapt_action(candidate: PolicyActionChunk) -> RobotActionChunk:
    candidate = codec.validate(candidate.model_dump(), PolicyActionChunk)
    if candidate.native_action_space != ROBOT_PROFILE:
        raise VLAError("UNSUPPORTED_ACTION_PROFILE", "no registered native-to-JetRover mapping")
    if (not candidate.synthetic or candidate.semantics != "absolute"
            or candidate.channel_names != JOINT_NAMES or candidate.units != ["rad"] * 5
            or candidate.shape[1] != 5):
        raise VLAError("INCOMPATIBLE_ACTION_SEMANTICS", "only synthetic absolute five-joint radians are mapped")
    if candidate.policy.processor_id != "mock_identity_v1" or candidate.policy.normalization_id != "none":
        raise VLAError("UNSUPPORTED_NORMALIZATION", "processor/normalization has no verified mapping")
    return RobotActionChunk(**identity_fields(candidate), action_id=str(uuid4()),
        source_policy_action_id=candidate.action_id, action_profile=ROBOT_PROFILE,
        channel_names=list(JOINT_NAMES), units=["rad"] * 5, semantics="absolute",
        shape=list(candidate.shape), values=[list(row) for row in candidate.values],
        time_offsets_s=list(candidate.time_offsets_s), mapping_id="synthetic_identity_v1",
        calibration_status="synthetic_uncalibrated")
