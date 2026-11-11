"""Canonical-to-model feature binding, without speculative model preprocessing."""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from io import BytesIO
from typing import Literal
from pydantic import Field, model_validator
from vla_protocol.profiles import StrictModel, Identifier, JOINT_NAMES
from vla_protocol.errors import VLAError


class CameraFeature(StrictModel):
    camera_id: Identifier
    feature_name: Identifier
    shape_chw: list[int] = Field(min_length=3, max_length=3)
    shape_policy: Literal["exact", "official_processor"] = "exact"

    @model_validator(mode="after")
    def rgb_shape(self):
        if self.shape_chw[0] != 3 or any(n <= 0 or n > 1024 for n in self.shape_chw[1:]):
            raise VLAError("INVALID_FEATURE_SPEC", "RGB feature requires declared [3,H,W]")
        if not self.feature_name.startswith("observation.images."):
            raise VLAError("INVALID_FEATURE_SPEC", "camera feature must use the official observation.images namespace")
        return self


class ModelFeatureSpec(StrictModel):
    model_id: Identifier
    processor_id: Identifier
    state_feature: Literal["observation.state"] = "observation.state"
    state_channels: list[Identifier] = Field(min_length=1, max_length=6)
    state_units: list[Identifier] = Field(min_length=1, max_length=6)
    state_dimension: int = Field(ge=1, le=6)
    cameras: list[CameraFeature] = Field(min_length=1, max_length=4)
    source_profiles: list[Identifier] = Field(default_factory=lambda: ["synthetic_uncalibrated_v1"])

    @model_validator(mode="after")
    def feature_identity(self):
        if (len(self.state_channels) != self.state_dimension or len(self.state_units) != self.state_dimension
                or len(set(self.state_channels)) != self.state_dimension
                or not set(self.state_channels) <= set(JOINT_NAMES + ["r_joint"])
                or any(unit != "rad" for unit in self.state_units)):
            raise VLAError("INVALID_FEATURE_SPEC", "explicit named state binding required; no padding, scaling or inferred order")
        if len({c.feature_name for c in self.cameras}) != len(self.cameras) or len({c.camera_id for c in self.cameras}) != len(self.cameras):
            raise VLAError("INVALID_FEATURE_SPEC", "duplicate camera or feature binding")
        return self


@dataclass(frozen=True)
class CanonicalFeatures:
    images: dict
    state: tuple
    instruction: str
    model_id: str
    processor_id: str


class ObservationAdapter(ABC):
    @abstractmethod
    def bind(self, observation) -> CanonicalFeatures:
        """Validate names/shapes; model processors own normalization/tokenization."""


class FeatureObservationAdapter(ObservationAdapter):
    def __init__(self, specification):
        self.specification = specification

    def bind(self, observation):
        spec = self.specification
        profile = observation.source_profile.profile_id if observation.source_profile else "synthetic_uncalibrated_v1"
        if profile not in spec.source_profiles:
            raise VLAError("OBSERVATION_SOURCE_UNSUPPORTED", "model feature specification does not accept this source profile")
        cameras = {image.camera_id: image for image in observation.images}
        bound = {}
        for feature in spec.cameras:
            image = cameras.get(feature.camera_id)
            if image is None:
                raise VLAError("CAMERA_MAPPING_MISSING", "missing camera: " + feature.camera_id)
            if feature.shape_policy == "exact" and [3, image.height, image.width] != feature.shape_chw:
                raise VLAError("IMAGE_FEATURE_SHAPE_MISMATCH", "camera shape differs from declared feature")
            bound[feature.feature_name] = image
        joints = dict(zip(observation.joints.names, observation.joints.positions))
        joints[observation.gripper.name] = observation.gripper.position
        state = tuple(joints[channel] for channel in spec.state_channels)
        return CanonicalFeatures(bound, state, observation.instruction, spec.model_id, spec.processor_id)

    def official_frame(self, observation, prepare_observation, device):
        """Decode lossless RGB, then use LeRobot's official tensor preparation.

        No local image resizing, language tokenization or normalization is added.
        The callback is lerobot.policies.utils.prepare_observation_for_inference.
        """
        bound = self.bind(observation)
        import numpy as np
        from PIL import Image
        frame = {self.specification.state_feature: np.asarray(bound.state, dtype=np.float32)}
        for name, payload in bound.images.items():
            with Image.open(BytesIO(payload.payload())) as image:
                if image.mode != "RGB" or image.size != (payload.width, payload.height):
                    raise VLAError("INVALID_IMAGE", "decoded RGB differs from descriptor")
                frame[name] = np.asarray(image).copy()
        return prepare_observation(frame, device, task=bound.instruction, robot_type=observation.robot_id)
