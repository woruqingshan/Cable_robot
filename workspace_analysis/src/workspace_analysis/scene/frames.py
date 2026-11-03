"""Immutable scene frame trees rooted at base_link, using geometry.se3."""

from collections import deque
from dataclasses import dataclass, field
import re
from types import MappingProxyType
from typing import Mapping, Protocol

import numpy as np
from numpy.typing import NDArray

from ..geometry.se3 import Pose, TransformError, compose_transforms, invert_transform


def _frame_name(name: str, label: str) -> str:
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*(?:/[A-Za-z_][A-Za-z0-9_-]*)*', name):
        raise TransformError(f'{label}: expected a nonempty frame identifier without whitespace/leading slash')
    return name


@dataclass(frozen=True, slots=True)
class Frame:
    """A single edge: pose is ^parent T_id. The base_link root is implicit."""

    id: str
    parent: str
    pose: Pose

    def __post_init__(self) -> None:
        _frame_name(self.id, 'frame.id')
        _frame_name(self.parent, f'frames[{self.id}].parent')
        if not isinstance(self.pose, Pose):
            raise TransformError(f'frames[{self.id}].pose: expected immutable Pose')


@dataclass(frozen=True, slots=True)
class FrameGraph:
    """Immutable single-parent tree rooted at base_link; resolve in either direction."""

    frames: tuple[Frame, ...]
    root: str = 'base_link'
    _to_root: Mapping[str, tuple[tuple[float, ...], ...]] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.root != 'base_link':
            raise TransformError('reference_frame: B1 requires base_link')
        frames = tuple(self.frames)
        if any(not isinstance(f, Frame) for f in frames):
            raise TransformError('frames: expected immutable Frame records')
        by_id: dict[str, Frame] = {}
        for frame in frames:
            if frame.id == self.root or frame.id in by_id:
                raise TransformError(f'frames[{frame.id}]: duplicate ID/multiple parents or redefined root')
            by_id[frame.id] = frame
        children: dict[str, list[Frame]] = {key: [] for key in [self.root, *by_id]}
        for frame in frames:
            if frame.parent not in children:
                raise TransformError(f'frames[{frame.id}].parent: missing frame {frame.parent!r}')
            children[frame.parent].append(frame)
        transforms = {self.root: np.eye(4)}
        pending = deque([self.root])
        while pending:
            parent = pending.popleft()
            for frame in children[parent]:
                transforms[frame.id] = compose_transforms(transforms[parent], frame.pose.matrix())
                pending.append(frame.id)
        unresolved = set(by_id) - transforms.keys()
        if unresolved:
            raise TransformError(f'frames: cycle/disconnected from base_link: {sorted(unresolved)}')
        object.__setattr__(self, 'frames', tuple(sorted(frames, key=lambda f: f.id)))
        object.__setattr__(self, '_to_root', MappingProxyType({
            name: tuple(tuple(row) for row in value.tolist()) for name, value in transforms.items()}))

    def resolve(self, target_frame: str, source_frame: str) -> NDArray[np.float64]:
        """Return ^target T_source = inverse(^base T_target) @ ^base T_source."""
        for label, name in [('target_frame', target_frame), ('source_frame', source_frame)]:
            if not isinstance(name, str) or name not in self._to_root:
                raise TransformError(f'{label}: unknown frame {name!r}')
        return compose_transforms(invert_transform(self._to_root[target_frame]), self._to_root[source_frame])

    def is_descendant(self, child: str, ancestor: str) -> bool:
        """Return strict ancestry (a frame is not its own descendant)."""
        if child not in self._to_root or ancestor not in self._to_root:
            raise TransformError(f'frames: unknown child/ancestor {child!r}/{ancestor!r}')
        parents = {f.id: f.parent for f in self.frames}
        while child != self.root:
            child = parents[child]
            if child == ancestor:
                return True
        return False


class _HasFrameGraph(Protocol):
    frame_graph: FrameGraph


def resolve_transform(scene: _HasFrameGraph | FrameGraph, target_frame: str,
                      source_frame: str) -> NDArray[np.float64]:
    """Resolve ^target T_source from a SceneSnapshot or validated FrameGraph."""
    graph = scene if isinstance(scene, FrameGraph) else getattr(scene, 'frame_graph', None)
    if not isinstance(graph, FrameGraph):
        raise TransformError('scene: expected a SceneSnapshot or FrameGraph')
    return graph.resolve(target_frame, source_frame)
