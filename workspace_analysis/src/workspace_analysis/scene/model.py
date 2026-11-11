"""Immutable SceneSnapshot data and geometry access; no YAML parser dependency."""

from dataclasses import dataclass, field
import json
from types import MappingProxyType
from typing import Mapping

import numpy as np
from numpy.typing import NDArray

from ..geometry.se3 import Pose, compose_transforms
from .frames import Frame, FrameGraph


class SceneError(ValueError):
    """Malformed YAML, unsupported schema, or inconsistent scene references."""


def _freeze(value: object) -> object:
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


@dataclass(frozen=True, slots=True)
class SceneSnapshot:
    """Validated snapshot returned by load_scene/parse_scene; nested data is immutable.

    to_dict()/to_json() export ONLY the canonical schema, not file paths or hashes.
    Raw/canonical hashes remain separate metadata, avoiding circular hashing.
    Provenance/coverage are input declarations, not hardware/collision certificates.
    """

    scene_id: str
    provenance: Mapping[str, object]
    reference_frame: str
    units: Mapping[str, str]
    coverage: str
    frames: tuple[Frame, ...]
    collision_objects: tuple[Mapping[str, object], ...]
    entities: tuple[Mapping[str, object], ...]
    ports: tuple[Mapping[str, object], ...]
    clearance: Mapping[str, float]
    tool: Mapping[str, object]
    frame_graph: FrameGraph = field(repr=False, compare=False)
    raw_yaml_sha256: str
    canonical_scene_sha256: str
    source_path: str
    _canonical_json: str = field(repr=False)

    def to_dict(self) -> dict:
        """Return a detached JSON-serializable copy; editing it cannot change this snapshot."""
        return json.loads(self._canonical_json)

    def to_json(self) -> str:
        """Return the exact UTF-8 JSON text whose SHA256 is canonical_scene_sha256."""
        return self._canonical_json

    def object_transform(self, object_id: str, target_frame: str = 'base_link') -> NDArray[np.float64]:
        """Return ^target T_object including its local pose, without collision testing."""
        for obj in self.collision_objects:
            if obj['id'] == object_id:
                pose = obj['pose']
                return compose_transforms(self.frame_graph.resolve(target_frame, obj['parent_frame']),
                                          Pose(pose['position_m'], pose['quaternion_xyzw']).matrix())
        raise SceneError(f'collision_objects: unknown object {object_id!r}')


