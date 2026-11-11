"""Wire ceilings are fixed; runtime settings may tighten them."""
from typing import Annotated
from pydantic import BaseModel, ConfigDict, Field, model_validator

SCHEMA_VERSION = 1
ROBOT_PROFILE = "jetrover_joint_abs_rad_v1"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
MAX_IMAGE_BYTES = 1_048_576
MAX_IMAGE_PIXELS = 1_048_576
MAX_BODY_BYTES = 6_000_000
MAX_HORIZON = 32
MAX_DURATION_S = 10.0
Identifier = Annotated[str, Field(min_length=1, max_length=128, pattern=r"^[A-Za-z0-9_./:-]+$")]
Finite = Annotated[float, Field(allow_inf_nan=False)]
NonnegativeInt = Annotated[int, Field(ge=0)]


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, frozen=True, allow_inf_nan=False)

    @model_validator(mode="before")
    @classmethod
    def literal_types(cls, value):
        # Literal[1]/Literal[True] alone can equate booleans with integers.
        if isinstance(value, dict):
            for key in ("schema_version",):
                if key in value and type(value[key]) is not int:
                    raise ValueError(key + " must be an integer")
            for key in ("simulated", "calibrated", "hardware_validated", "continuous_collision_proven"):
                if key in value and type(value[key]) is not bool:
                    raise ValueError(key + " must be boolean")
        return value


class RuntimeLimits(StrictModel):
    max_body_bytes: int = Field(default=MAX_BODY_BYTES, ge=1024, le=MAX_BODY_BYTES)
    max_image_bytes: int = Field(default=MAX_IMAGE_BYTES, ge=64, le=MAX_IMAGE_BYTES)
    max_horizon: int = Field(default=MAX_HORIZON, ge=1, le=MAX_HORIZON)
    request_timeout_s: float = Field(default=2.0, gt=0, le=30)
    max_observation_age_s: float = Field(default=0.5, gt=0, le=10)
    max_sync_skew_s: float = Field(default=0.02, ge=0, le=0.5)
    max_action_age_s: float = Field(default=5.0, gt=0, le=30)
