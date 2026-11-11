"""Policy-native candidates, robot-space actions and local validation evidence."""
from typing import Literal, Any
from pydantic import Field, model_validator
from .observation import Identity, JointPositions, GripperState
from .profiles import StrictModel, Identifier, Finite, MAX_HORIZON, MAX_DURATION_S, JOINT_NAMES, ROBOT_PROFILE
from .errors import VLAError


class PolicyIdentity(StrictModel):
    policy_type: Identifier
    policy_version: Identifier
    framework_revision: Identifier | None
    checkpoint_revision: Identifier | None
    processor_id: Identifier
    normalization_id: Identifier


class PolicyActionChunk(Identity):
    action_id: Identifier
    policy: PolicyIdentity
    native_action_space: Identifier
    channel_names: list[Identifier] = Field(min_length=1, max_length=256)
    units: list[Identifier] = Field(min_length=1, max_length=256)
    semantics: Literal["absolute", "delta"]
    shape: list[int] = Field(min_length=2, max_length=2)
    values: list[list[Finite]] = Field(min_length=1, max_length=MAX_HORIZON)
    time_offsets_s: list[Finite] = Field(min_length=1, max_length=MAX_HORIZON)
    inference_duration_s: float = Field(ge=0, le=3600)
    synthetic: bool

    @model_validator(mode="after")
    def dimensions_and_timing(self):
        h, d = self.shape
        if not 1 <= h <= MAX_HORIZON or not 1 <= d <= 256 or len(self.values) != h:
            raise VLAError("ACTION_SHAPE_MISMATCH", "invalid action horizon/dimension")
        if any(len(row) != d for row in self.values) or len(self.channel_names) != d or len(self.units) != d:
            raise VLAError("ACTION_SHAPE_MISMATCH", "tensor/channels/units differ")
        if len(set(self.channel_names)) != d:
            raise VLAError("ACTION_SHAPE_MISMATCH", "duplicate action channels")
        offsets = self.time_offsets_s
        if len(offsets) != h or offsets[0] <= 0 or offsets[-1] > MAX_DURATION_S or any(b <= a for a, b in zip(offsets, offsets[1:])):
            raise VLAError("INVALID_ACTION_TIMING", "require positive, strictly increasing offsets within 10s")
        return self


class RobotActionChunk(Identity):
    action_id: Identifier
    source_policy_action_id: Identifier
    action_profile: Literal["jetrover_joint_abs_rad_v1"]
    channel_names: list[Identifier] = Field(min_length=5, max_length=5)
    units: list[Identifier] = Field(min_length=5, max_length=5)
    semantics: Literal["absolute"]
    shape: list[int] = Field(min_length=2, max_length=2)
    values: list[list[Finite]] = Field(min_length=1, max_length=MAX_HORIZON)
    time_offsets_s: list[Finite] = Field(min_length=1, max_length=MAX_HORIZON)
    mapping_id: Literal["synthetic_identity_v1"]
    calibration_status: Literal["synthetic_uncalibrated"]

    @model_validator(mode="after")
    def robot_profile(self):
        if self.channel_names != JOINT_NAMES or self.units != ["rad"] * 5 or self.shape != [len(self.values), 5]:
            raise VLAError("UNSUPPORTED_ACTION_PROFILE", "robot profile requires exact five-joint absolute radians")
        if any(len(row) != 5 for row in self.values):
            raise VLAError("ACTION_SHAPE_MISMATCH", "expected [H,5]")
        offsets = self.time_offsets_s
        if len(offsets) != len(self.values) or offsets[0] <= 0 or offsets[-1] > MAX_DURATION_S or any(b <= a for a, b in zip(offsets, offsets[1:])):
            raise VLAError("INVALID_ACTION_TIMING", "invalid robot action offsets")
        return self


class ValidatedTrajectory(StrictModel):
    schema_version: Literal[1]
    source_action_id: Identifier
    starting_joints: JointPositions
    starting_gripper: GripperState
    status: Literal["NOT_CHECKED", "PASS_RESEARCH_ONLY", "REJECTED", "ERROR"]
    trajectory: RobotActionChunk | None
    failure_reasons: list[str]
    scene_id: Identifier
    scene_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    model_id: Identifier
    validation_config: dict[str, Any]
    provenance: dict[str, Any]
    discrete_only: bool
    continuous_collision_proven: Literal[False] = False
    hardware_validated: Literal[False] = False

    @model_validator(mode="after")
    def consistent_verdict(self):
        rejected = self.status in ("REJECTED", "ERROR")
        if rejected != (self.trajectory is None) or rejected != bool(self.failure_reasons):
            raise VLAError("INVALID_VALIDATION_RESULT", "verdict/trajectory/reasons inconsistent")
        if self.trajectory is not None:
            a = self.trajectory
            if (a.action_id, a.scene_id, a.scene_sha256, a.model_id) != (self.source_action_id, self.scene_id, self.scene_sha256, self.model_id):
                raise VLAError("IDENTITY_MISMATCH", "validation source/context differs from trajectory")
        if self.status == "PASS_RESEARCH_ONLY" and not self.discrete_only:
            raise VLAError("INVALID_VALIDATION_RESULT", "P2 research acceptance must declare discrete checking")
        return self
