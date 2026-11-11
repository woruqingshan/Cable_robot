"""Strict policy/service configuration. Unresolved GPU settings remain explicit."""
from pathlib import Path
from typing import Literal
import yaml
from pydantic import Field, model_validator
from vla_protocol.profiles import StrictModel, Identifier, RuntimeLimits
from vla_protocol.errors import VLAError
from .observation_adapter import ModelFeatureSpec


class NativeActionSpec(StrictModel):
    space_id: Identifier
    channels: list[Identifier] = Field(min_length=1, max_length=256)
    units: list[Identifier] = Field(min_length=1, max_length=256)
    semantics: Literal["absolute", "delta"]
    step_s: float = Field(gt=0, le=10)
    processor_id: Identifier
    normalization_id: Identifier
    semantics_source: str = Field(min_length=1, max_length=1024)

    @model_validator(mode="after")
    def dimensions(self):
        if len(self.channels) != len(self.units) or len(set(self.channels)) != len(self.channels):
            raise VLAError("INVALID_ACTION_SPEC", "native channel/unit declarations disagree")
        return self


class PolicyConfig(StrictModel):
    type: Literal["mock", "fake", "smolvla"] = "mock"
    feature_spec: ModelFeatureSpec | None = None
    native_spec: NativeActionSpec | None = None
    fake_horizon: int = Field(default=50, ge=1, le=256)
    fake_dimension: int = Field(default=7, ge=1, le=256)
    fake_delay_s: float = Field(default=0.0, ge=0, le=60)
    fake_semantics: Literal["absolute", "delta"] = "absolute"
    fake_normalization_id: Identifier = "fake_native_stats_v1"
    lerobot_checkout: str | None = None
    lerobot_revision: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")
    checkpoint_path: str | None = None
    checkpoint_repo: str = "lerobot/smolvla_base"
    checkpoint_revision: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")
    checkpoint_inventory_sha256: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    device: Literal["cpu", "cuda"] = "cuda"
    offline: Literal[True] = True

    def require_model_pins(self):
        fields = ("lerobot_checkout", "lerobot_revision", "checkpoint_path", "checkpoint_revision",
                  "checkpoint_inventory_sha256", "feature_spec", "native_spec")
        missing = [name for name in fields if getattr(self, name) is None]
        if missing:
            raise VLAError("DEPLOYMENT_UNRESOLVED", "resolve and verify model settings: " + ", ".join(missing))


class ServiceTiming(StrictModel):
    inference_timeout_s: float | None = Field(default=None, gt=0, le=300)
    startup_timeout_s: float = Field(default=120.0, gt=0, le=600)
    warmup_timeout_s: float = Field(default=30.0, gt=0, le=300)
    warmup_observation_path: str | None = None


class ServerConfig(StrictModel):
    schema_version: Literal[1] = 1
    host: Literal["127.0.0.1", "::1"] = "127.0.0.1"
    port: int = Field(default=8000, ge=1, le=65535)
    source_revision: str | None = Field(default=None, pattern=r"^[0-9a-f]{40}$")
    policy: PolicyConfig = Field(default_factory=PolicyConfig)
    limits: RuntimeLimits = Field(default_factory=RuntimeLimits)
    timing: ServiceTiming = Field(default_factory=ServiceTiming)


def read_yaml(path):
    path = Path(path).expanduser().resolve()
    if path.stat().st_size > 131072:
        raise VLAError("INVALID_CONFIG", "configuration is too large")
    class Loader(yaml.SafeLoader):
        pass
    def mapping(loader, node):
        result = {}
        for key_node, value_node in node.value:
            key = loader.construct_object(key_node)
            if not isinstance(key, str) or key in result:
                raise VLAError("INVALID_CONFIG", "duplicate/non-string YAML key")
            result[key] = loader.construct_object(value_node)
        return result
    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, mapping)
    try:
        text = path.read_text()
        if any(isinstance(e, yaml.events.AliasEvent) for e in yaml.parse(text)):
            raise VLAError("INVALID_CONFIG", "YAML aliases are unsupported")
        result = yaml.load(text, Loader=Loader)
    except VLAError:
        raise
    except Exception as exc:
        raise VLAError("INVALID_CONFIG", str(exc)) from exc
    return result


def load_config(path):
    path = Path(path).expanduser().resolve()
    try:
        config = ServerConfig.model_validate(read_yaml(path))
    except VLAError:
        raise
    except Exception as exc:
        raise VLAError("INVALID_CONFIG", str(exc)) from exc
    data = config.model_dump()
    for name in ("lerobot_checkout", "checkpoint_path"):
        value = data["policy"][name]
        if value:
            data["policy"][name] = str((path.parent / Path(value).expanduser()).resolve())
    warmup = data["timing"]["warmup_observation_path"]
    if warmup:
        data["timing"]["warmup_observation_path"] = str((path.parent / Path(warmup).expanduser()).resolve())
    return ServerConfig.model_validate(data)
