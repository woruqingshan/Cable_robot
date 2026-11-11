"""Explicit mapping registry; tensor dimensions never select a conversion."""
from dataclasses import dataclass
from typing import Literal
from uuid import uuid4
from pydantic import Field, model_validator
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk, RobotActionChunk, ExecutionHorizon
from vla_protocol.observation import identity_fields
from vla_protocol.profiles import ROBOT_PROFILE, JOINT_NAMES, MAX_HORIZON, StrictModel, RuntimeLimits
from vla_protocol.errors import VLAError


class ExecutionSelection(StrictModel):
    mode: Literal["full", "prefix"] = "full"
    steps: int | None = Field(default=None, ge=1, le=MAX_HORIZON)

    @model_validator(mode="after")
    def explicit_steps(self):
        if (self.mode == "prefix") != (self.steps is not None):
            raise VLAError("INVALID_EXECUTION_SELECTION", "prefix requires explicit steps; full requires no step limit")
        return self


@dataclass(frozen=True)
class MappingSpec:
    native_space: str
    channels: tuple[str, ...]
    units: tuple[str, ...]
    semantics: str
    processor_id: str
    normalization_id: str
    robot_profile: str
    mapping_version: str
    calibration_requirement: str


SYNTHETIC_IDENTITY = MappingSpec(ROBOT_PROFILE, tuple(JOINT_NAMES), ("rad",) * 5, "absolute",
    "mock_identity_v1", "none", ROBOT_PROFILE, "synthetic_identity_v1", "synthetic_only")


class ActionAdapterRegistry:
    """Only implemented conversions may be registered. No generic scaler exists."""
    def __init__(self):
        self.mappings = {SYNTHETIC_IDENTITY.native_space: SYNTHETIC_IDENTITY}

    def describe(self):
        from dataclasses import asdict
        return [asdict(spec) for spec in self.mappings.values()]

    def adapt(self, candidate, *, target_profile=ROBOT_PROFILE, selection=None):
        candidate = codec.validate(candidate.model_dump(), PolicyActionChunk, RuntimeLimits(max_horizon=256))
        spec = self.mappings.get(candidate.native_action_space)
        if spec is None or target_profile != spec.robot_profile:
            raise VLAError("UNSUPPORTED_ACTION_PROFILE", "no registered native-to-robot mapping")
        if (not candidate.synthetic or candidate.semantics != spec.semantics
                or tuple(candidate.channel_names) != spec.channels or tuple(candidate.units) != spec.units
                or candidate.shape[1] != len(spec.channels)):
            raise VLAError("INCOMPATIBLE_ACTION_SEMANTICS", "mapping requires synthetic absolute five-joint radians")
        if candidate.policy.processor_id != spec.processor_id or candidate.policy.normalization_id != spec.normalization_id:
            raise VLAError("UNSUPPORTED_NORMALIZATION", "processor/normalization has no verified mapping")
        selection = selection or ExecutionSelection()
        transmitted = candidate.shape[0]
        count = transmitted if selection.mode == "full" else selection.steps
        if count > transmitted or count > MAX_HORIZON:
            raise VLAError("EXECUTION_HORIZON_UNSUPPORTED", "choose an explicit prefix within transmitted and robot horizons")
        horizon = None
        if candidate.horizon is not None or selection.mode != "full":
            horizon = ExecutionHorizon(generated=candidate.horizon.generated if candidate.horizon else transmitted,
                transmitted=transmitted, selected=count, selection=selection.mode)
        return RobotActionChunk(**identity_fields(candidate), action_id=str(uuid4()),
            source_policy_action_id=candidate.action_id, action_profile=spec.robot_profile,
            channel_names=list(spec.channels), units=list(spec.units), semantics=spec.semantics,
            shape=[count, 5], values=[list(row) for row in candidate.values[:count]],
            time_offsets_s=list(candidate.time_offsets_s[:count]), mapping_id=spec.mapping_version,
            calibration_status="synthetic_uncalibrated", horizon=horizon)


DEFAULT_REGISTRY = ActionAdapterRegistry()


def adapt_action(candidate: PolicyActionChunk, *, selection=None, target_profile=ROBOT_PROFILE) -> RobotActionChunk:
    return DEFAULT_REGISTRY.adapt(candidate, selection=selection, target_profile=target_profile)
