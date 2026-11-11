"""Snapshot collector used by the VLA ROS node; no network work in callbacks."""
from time import monotonic
from uuid import uuid4
from pydantic import Field
from vla_protocol.observation import Observation, JointPositions, GripperState, ImagePayload, SourceStamp
from vla_protocol.profiles import StrictModel, Identifier, RuntimeLimits
from vla_protocol.errors import VLAError


class StateMetadata(StrictModel):
    schema_version: int = Field(ge=1, le=1)
    session_id: Identifier
    task_revision: int = Field(ge=0)
    state_revision: int = Field(ge=0)
    robot_id: Identifier
    scene_id: Identifier
    scene_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    model_id: Identifier
    instruction: str = Field(min_length=1, max_length=2048)
    stamp: SourceStamp
    busy: bool

    @property
    def token(self):
        return (self.session_id, self.task_revision, self.state_revision, self.scene_sha256, self.model_id)


class ObservationAdapter:
    def __init__(self, limits=None, clock=monotonic):
        self.limits = limits or RuntimeLimits()
        self.clock = clock
        self.parts = {}

    def offer(self, kind, value):
        expected = {"metadata": StateMetadata, "joints": JointPositions, "gripper": GripperState, "image": ImagePayload}
        if kind not in expected or not isinstance(value, expected[kind]):
            raise VLAError("INVALID_OBSERVATION_SOURCE", "unexpected source type")
        self.parts[kind] = (value, self.clock())

    def snapshot(self, sequence, now_ros_ns):
        if set(self.parts) != {"metadata", "joints", "gripper", "image"}:
            raise VLAError("OBSERVATION_INCOMPLETE", "waiting for all ROS observation sources")
        values = {k: v[0] for k, v in self.parts.items()}
        if any(self.clock() - received > self.limits.max_observation_age_s for _, received in self.parts.values()):
            raise VLAError("STALE_OBSERVATION", "source receive age exceeds limit")
        stamps = [v.stamp for v in values.values()]
        if len({s.clock_domain for s in stamps}) != 1:
            raise VLAError("CLOCK_DOMAIN_MISMATCH", "source clock domains differ")
        ns = [s.nanoseconds for s in stamps]
        if max(ns) - min(ns) > self.limits.max_sync_skew_s * 1e9:
            raise VLAError("OBSERVATION_SKEW", "source timestamp skew exceeds configured policy")
        if any(now_ros_ns - n > self.limits.max_observation_age_s * 1e9 or n - now_ros_ns > self.limits.max_sync_skew_s * 1e9 for n in ns):
            raise VLAError("STALE_OBSERVATION", "ROS source age/future timestamp invalid")
        meta = values["metadata"]
        if meta.busy:
            raise VLAError("ROBOT_BUSY", "mock trajectory is executing")
        return Observation(schema_version=1, request_id=str(uuid4()), session_id=meta.session_id,
            observation_seq=sequence, task_revision=meta.task_revision, state_revision=meta.state_revision,
            robot_id=meta.robot_id, robot_profile_id="jetrover_joint_abs_rad_v1",
            scene_id=meta.scene_id, scene_sha256=meta.scene_sha256, model_id=meta.model_id,
            instruction=meta.instruction, joints=values["joints"], gripper=values["gripper"],
            images=[values["image"]], simulated=True, calibration_status="synthetic_uncalibrated")
