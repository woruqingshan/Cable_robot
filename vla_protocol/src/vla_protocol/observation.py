"""Observation snapshots preserve source clocks; transport deadlines stay local."""
import base64
import binascii
from hashlib import sha256
from typing import Literal
from pydantic import Field, model_validator
from .profiles import StrictModel, Identifier, Finite, NonnegativeInt, JOINT_NAMES, ROBOT_PROFILE, MAX_IMAGE_BYTES
from .png import validate_png
from .errors import VLAError


class SourceStamp(StrictModel):
    nanoseconds: NonnegativeInt
    clock_domain: Identifier


class Identity(StrictModel):
    schema_version: Literal[1]
    request_id: Identifier
    session_id: Identifier
    observation_seq: NonnegativeInt
    task_revision: NonnegativeInt
    robot_id: Identifier
    robot_profile_id: Literal["jetrover_joint_abs_rad_v1"]
    state_revision: NonnegativeInt
    scene_id: Identifier
    scene_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    model_id: Identifier


class JointPositions(StrictModel):
    names: list[Identifier] = Field(min_length=5, max_length=5)
    positions: list[Finite] = Field(min_length=5, max_length=5)
    unit: Literal["rad"]
    stamp: SourceStamp

    @model_validator(mode="after")
    def exact_order(self):
        if self.names != JOINT_NAMES:
            raise VLAError("JOINT_ORDER_MISMATCH", "expected joint1..joint5 in exact order")
        return self


class GripperState(StrictModel):
    name: Literal["r_joint"]
    position: Finite
    unit: Literal["rad"]
    stamp: SourceStamp
    simulated: bool
    calibrated: bool


class ObservationSource(StrictModel):
    profile_id: Literal["synthetic_uncalibrated_v1", "real_uncalibrated_v1", "real_calibrated_v1"]
    calibration_id: Identifier | None = None
    calibration_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")

    @model_validator(mode="after")
    def calibration_evidence(self):
        supplied = self.calibration_id is not None and self.calibration_sha256 is not None
        if self.profile_id == "real_calibrated_v1":
            if not supplied:
                raise VLAError("CALIBRATION_REQUIRED", "calibrated input needs an explicit calibration ID and content hash")
        elif self.calibration_id is not None or self.calibration_sha256 is not None:
            raise VLAError("CALIBRATION_PROFILE_MISMATCH", "uncalibrated profile cannot carry calibrated status")
        return self


class ImagePayload(StrictModel):
    camera_id: Identifier
    frame_id: Identifier
    stamp: SourceStamp
    width: int = Field(gt=0, le=1024)
    height: int = Field(gt=0, le=1024)
    codec: Literal["png"]
    pixel_encoding: Literal["rgb8"]
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    data_base64: str = Field(min_length=1, max_length=4 * ((MAX_IMAGE_BYTES + 2) // 3))

    def payload(self) -> bytes:
        try:
            return base64.b64decode(self.data_base64, validate=True)
        except (ValueError, binascii.Error) as exc:
            raise VLAError("INVALID_IMAGE", "invalid base64") from exc

    @model_validator(mode="after")
    def valid_png(self):
        raw = self.payload()
        if len(raw) > MAX_IMAGE_BYTES:
            raise VLAError("PAYLOAD_TOO_LARGE", "image exceeds byte ceiling")
        if sha256(raw).hexdigest() != self.sha256:
            raise VLAError("IMAGE_HASH_MISMATCH", "image content does not match descriptor")
        validate_png(raw, self.width, self.height)
        return self


class Observation(Identity):
    instruction: str = Field(min_length=1, max_length=2048)
    joints: JointPositions
    gripper: GripperState
    images: list[ImagePayload] = Field(min_length=1, max_length=4)
    simulated: bool
    calibration_status: Literal["synthetic_uncalibrated", "real_uncalibrated", "real_calibrated"]
    source_profile: ObservationSource | None = None
    capabilities: list[Literal["native_actions_v1", "source_profiles_v1", "session_state_v1"]] = Field(default_factory=list, max_length=3)

    @model_validator(mode="after")
    def camera_identity(self):
        source = self.source_profile
        if len(set(self.capabilities)) != len(self.capabilities):
            raise VLAError("INVALID_PROTOCOL_CAPABILITIES", "duplicate protocol capability")
        if source is not None and "source_profiles_v1" not in self.capabilities:
            raise VLAError("PROTOCOL_EXTENSION_REQUIRED", "explicit source profile requires source_profiles_v1")
        if source is None:
            if not self.simulated or self.calibration_status != "synthetic_uncalibrated":
                raise VLAError("SOURCE_PROFILE_REQUIRED", "non-synthetic observations require an explicit source profile")
            expected_simulated, expected_calibrated = True, False
        else:
            expected_simulated = source.profile_id == "synthetic_uncalibrated_v1"
            expected_calibrated = source.profile_id == "real_calibrated_v1"
            expected_status = source.profile_id.removesuffix("_v1")
            if self.simulated != expected_simulated or self.calibration_status != expected_status:
                raise VLAError("CALIBRATION_PROFILE_MISMATCH", "source profile and legacy status fields disagree")
        if self.gripper.simulated != expected_simulated or self.gripper.calibrated != expected_calibrated:
            raise VLAError("CALIBRATION_PROFILE_MISMATCH", "gripper source/calibration disagrees with observation profile")
        if len({i.camera_id for i in self.images}) != len(self.images):
            raise VLAError("INVALID_IMAGE", "duplicate camera IDs")
        stamps = [self.joints.stamp, self.gripper.stamp, *(i.stamp for i in self.images)]
        if len({s.clock_domain for s in stamps}) != 1:
            raise VLAError("CLOCK_DOMAIN_MISMATCH", "observation sources require one declared clock domain")
        return self


IDENTITY_FIELDS = tuple(Identity.model_fields)


def identity_fields(value: Identity) -> dict:
    return value.model_dump(include=set(IDENTITY_FIELDS))
