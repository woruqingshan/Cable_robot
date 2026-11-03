"""Strict Scene Schema v1 -> immutable snapshot, independent of ROS/backends.

Canonical form: normalized float64 numbers/XYZW sign, collections sorted by ID,
components sorted by ID, json.dumps(sort_keys=True, ensure_ascii=False,
separators=(',', ':'), allow_nan=False), UTF-8 without a trailing newline.
Raw YAML bytes have a separate SHA256 and are never included in canonical data.
"""

from dataclasses import dataclass, field
from hashlib import sha256
import json
import math
import numbers
from pathlib import Path
import re
from types import MappingProxyType
from typing import Mapping

import numpy as np
from numpy.typing import NDArray
import yaml
from yaml.events import AliasEvent
from yaml.nodes import MappingNode, ScalarNode, SequenceNode

from .transforms import Frame, FrameGraph, Pose, TransformError, compose_transforms


MAX_YAML_BYTES = 1024 * 1024
MAX_YAML_DEPTH = 64
MAX_YAML_NODES = 20000


class SceneError(ValueError):
    """Malformed YAML, unsupported schema, or inconsistent scene references."""


class _Loader(yaml.SafeLoader):
    def __init__(self, text: str) -> None:
        super().__init__(text)
        self._depth = self._nodes = 0

    def compose_node(self, parent: object, index: object) -> object:
        """Reject alias expansion and bound YAML composition resources."""
        if self.check_event(AliasEvent):
            event = self.peek_event()
            raise SceneError(f'YAML line {event.start_mark.line + 1}: aliases are not supported in v1')
        self._depth += 1
        self._nodes += 1
        try:
            if self._depth > MAX_YAML_DEPTH or self._nodes > MAX_YAML_NODES:
                raise SceneError('YAML: nesting/node limit exceeded')
            return super().compose_node(parent, index)
        finally:
            self._depth -= 1


# YAML 1.1's default resolver misses JSON numbers like 1e-07. Restrict this
# extension to unquoted scientific numeric tokens; quoted strings stay strings.
_Loader.add_implicit_resolver('tag:yaml.org,2002:float', re.compile(
    r'^[-+]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)[eE][-+]?[0-9]+$'), list('-+0123456789.'))


def _decode_node(loader: _Loader, node: object, path: str) -> object:
    if isinstance(node, MappingNode) and node.tag == 'tag:yaml.org,2002:map':
        result = {}
        for key_node, value_node in node.value:
            if not isinstance(key_node, ScalarNode) or key_node.tag != 'tag:yaml.org,2002:str':
                raise SceneError(f'{path}: YAML mapping keys must be strings; merge keys are unsupported')
            key = loader.construct_object(key_node)
            if key in result:
                raise SceneError(f'{path}.{key}: duplicate YAML key at line {key_node.start_mark.line + 1}')
            result[key] = _decode_node(loader, value_node, f'{path}.{key}')
        return result
    if isinstance(node, SequenceNode) and node.tag == 'tag:yaml.org,2002:seq':
        return [_decode_node(loader, child, f'{path}[{index}]') for index, child in enumerate(node.value)]
    if isinstance(node, ScalarNode) and node.tag in {
            'tag:yaml.org,2002:' + tag for tag in ['str', 'bool', 'int', 'float', 'null']}:
        try:
            return loader.construct_object(node)
        except (ValueError, OverflowError, yaml.YAMLError) as exc:
            raise SceneError(f'{path}: invalid YAML scalar: {exc}') from exc
    raise SceneError(f'{path}: unsupported YAML type/tag {getattr(node, "tag", None)!r}')


def _mapping(value: object, path: str, keys: set[str]) -> dict:
    if not isinstance(value, dict):
        raise SceneError(f'{path}: expected mapping')
    missing, unknown = keys - value.keys(), value.keys() - keys
    if missing or unknown:
        raise SceneError(f'{path}: missing fields {sorted(missing)}; unknown fields {sorted(unknown)}')
    return value


def _sequence(value: object, path: str) -> list:
    if not isinstance(value, list):
        raise SceneError(f'{path}: expected sequence')
    return value


def _string(value: object, path: str) -> str:
    if not isinstance(value, str) or not value.strip() or value != value.strip() or any(ord(c) < 32 for c in value):
        raise SceneError(f'{path}: expected nonempty string without surrounding whitespace/control characters')
    return value


def _identifier(value: object, path: str) -> str:
    name = _string(value, path)
    if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_-]*(?:/[A-Za-z_][A-Za-z0-9_-]*)*', name):
        raise SceneError(f'{path}: invalid identifier {name!r}')
    return name


def _number(value: object, path: str, *, positive: bool = False) -> float:
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, numbers.Real):
        raise SceneError(f'{path}: expected real number, excluding bool')
    try:
        number = float(value)
    except (OverflowError, ValueError) as exc:
        raise SceneError(f'{path}: cannot represent a float64 number') from exc
    if not math.isfinite(number) or (number <= 0 if positive else False):
        raise SceneError(f'{path}: expected finite {"positive " if positive else ""}number')
    return 0.0 if number == 0 else number


def _vector(value: object, length: int, path: str, *, positive: bool = False) -> list[float]:
    values = _sequence(value, path)
    if len(values) != length:
        raise SceneError(f'{path}: expected {length} components')
    return [_number(v, f'{path}[{i}]', positive=positive) for i, v in enumerate(values)]


def _pose(value: object, path: str) -> Pose:
    pose = _mapping(value, path, {'position_m', 'quaternion_xyzw'})
    position = _vector(pose['position_m'], 3, path + '.position_m')
    quaternion = _vector(pose['quaternion_xyzw'], 4, path + '.quaternion_xyzw')
    try:
        return Pose(tuple(position), tuple(quaternion))
    except TransformError as exc:
        raise SceneError(f'{path}: {exc}') from exc


def _pose_dict(pose: Pose) -> dict:
    return {'position_m': list(pose.position_m), 'quaternion_xyzw': list(pose.quaternion_xyzw)}


def _records(value: object, path: str) -> list[dict]:
    result, seen = [], set()
    for i, record in enumerate(_sequence(value, path)):
        location = f'{path}[{i}]'
        if not isinstance(record, dict) or 'id' not in record:
            raise SceneError(f'{location}.id: missing record identifier')
        name = _identifier(record['id'], location + '.id')
        if name in seen:
            raise SceneError(f'{location}.id: duplicate ID {name!r}')
        seen.add(name)
        result.append(record)
    return result


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


def _snapshot(document: object, raw_hash: str, source: str) -> SceneSnapshot:
    root = _mapping(document, '$', {'schema_version', 'scene'})
    if type(root['schema_version']) is not int or root['schema_version'] != 1:
        raise SceneError('schema_version: expected integer 1, excluding bool')
    s = _mapping(root['scene'], 'scene', {'id', 'provenance', 'reference_frame', 'units', 'coverage',
                                       'frames', 'entities', 'collision_objects', 'ports', 'clearance', 'tool'})
    name = _identifier(s['id'], 'scene.id')
    if '/' in name:
        raise SceneError('scene.id: scene namespace must be one identifier segment')
    provenance = _mapping(s['provenance'], 'scene.provenance', {'kind', 'hardware_verified'})
    if provenance['kind'] not in ('synthetic', 'measured', 'vision') or type(provenance['hardware_verified']) is not bool:
        raise SceneError('scene.provenance: expected known kind and boolean hardware_verified')
    if provenance['kind'] == 'synthetic' and provenance['hardware_verified']:
        raise SceneError('scene.provenance.hardware_verified: synthetic input cannot claim hardware verification')
    if s['reference_frame'] != 'base_link':
        raise SceneError('scene.reference_frame: expected base_link')
    units = _mapping(s['units'], 'scene.units', {'length', 'angle'})
    if units != {'length': 'm', 'angle': 'rad'}:
        raise SceneError('scene.units: expected length=m and angle=rad; no implicit conversions')
    coverage = _string(s['coverage'], 'scene.coverage')
    frames = []
    for i, record in enumerate(_records(s['frames'], 'scene.frames')):
        path = f'scene.frames[{i}]'
        _mapping(record, path, {'id', 'parent', 'pose'})
        if not record['id'].startswith(name + '/'):
            raise SceneError(f'{path}.id: expected namespace {name}/, not a robot Link name')
        parent = _identifier(record['parent'], path + '.parent')
        frames.append(Frame(record['id'], parent, _pose(record['pose'], path + '.pose')))
    try:
        graph = FrameGraph(tuple(frames))
    except TransformError as exc:
        raise SceneError(f'scene.{exc}') from exc
    frame_ids = {'base_link', *(f.id for f in frames)}

    def frame_ref(value: object, path: str) -> str:
        """Validate a scene reference against the already-resolved frame graph."""
        frame = _identifier(value, path)
        if frame not in frame_ids:
            raise SceneError(f'{path}: unknown frame {frame!r}')
        return frame

    objects = []
    for i, record in enumerate(_records(s['collision_objects'], 'scene.collision_objects')):
        path = f'scene.collision_objects[{i}]'
        kind = record.get('type')
        if kind not in ('box', 'cylinder'):
            raise SceneError(f'{path}.type: expected box or cylinder')
        geometry = {'size_m'} if kind == 'box' else {'radius_m', 'length_m'}
        _mapping(record, path, {'id', 'type', 'parent_frame', 'pose'} | geometry)
        local_pose = _pose(record['pose'], path + '.pose')
        item = {'id': record['id'], 'type': kind,
                'parent_frame': frame_ref(record['parent_frame'], path + '.parent_frame'),
                'pose': _pose_dict(local_pose)}
        try:
            compose_transforms(graph.resolve('base_link', item['parent_frame']), local_pose.matrix())
        except TransformError as exc:
            raise SceneError(f'{path}.pose: nonfinite/invalid derived base transform: {exc}') from exc
        if kind == 'box':
            item['size_m'] = _vector(record['size_m'], 3, path + '.size_m', positive=True)
        else:
            for field_name in geometry:
                item[field_name] = _number(record[field_name], path + '.' + field_name, positive=True)
        objects.append(item)
    object_ids = {obj['id'] for obj in objects}
    entities = []
    for i, record in enumerate(_records(s['entities'], 'scene.entities')):
        path = f'scene.entities[{i}]'
        _mapping(record, path, {'id', 'kind', 'frame', 'components'})
        if record['kind'] not in ('cabinet', 'patch_panel'):
            raise SceneError(f'{path}.kind: expected cabinet or patch_panel (semantic assembly, not solid)')
        components = [_identifier(v, path + '.components') for v in _sequence(record['components'], path + '.components')]
        if not components or len(set(components)) != len(components) or set(components) - object_ids:
            raise SceneError(f'{path}.components: expected unique existing collision object IDs')
        entities.append({'id': record['id'], 'kind': record['kind'],
                         'frame': frame_ref(record['frame'], path + '.frame'), 'components': sorted(components)})
    by_entity = {entity['id']: entity for entity in entities}
    ports = []
    for i, record in enumerate(_records(s['ports'], 'scene.ports')):
        path = f'scene.ports[{i}]'
        _mapping(record, path, {'id', 'entity', 'frame', 'outward_axis_local', 'insertion_axis_local',
                               'key_axis_local', 'task_parameters_status'})
        entity_id = _identifier(record['entity'], path + '.entity')
        entity = by_entity.get(entity_id)
        if entity is None or entity['kind'] != 'patch_panel':
            raise SceneError(f'{path}.entity: expected existing patch_panel ID')
        port_frame = frame_ref(record['frame'], path + '.frame')
        if not graph.is_descendant(port_frame, entity['frame']):
            raise SceneError(f'{path}.frame: port must be a descendant of its panel frame {entity["frame"]!r}')
        if record['task_parameters_status'] != 'unmeasured':
            raise SceneError(f'{path}.task_parameters_status: v1 B1 supports unmeasured only')
        item = {'id': record['id'], 'entity': entity_id, 'frame': port_frame, 'task_parameters_status': 'unmeasured'}
        for axis, expected in [('outward_axis_local', [0., 0., 1.]), ('insertion_axis_local', [0., 0., -1.]),
                               ('key_axis_local', [1., 0., 0.])]:
            values = _vector(record[axis], 3, path + '.' + axis)
            if not np.allclose(values, expected, atol=1e-12, rtol=0):
                raise SceneError(f'{path}.{axis}: expected fixed port convention {expected}')
            item[axis] = expected
        ports.append(item)
    clearance = _mapping(s['clearance'], 'scene.clearance', {'robot_environment_m', 'self_m', 'contact_epsilon_m'})
    clearance = {key: _number(value, 'scene.clearance.' + key) for key, value in clearance.items()}
    for key, value in clearance.items():
        if value < 0:
            raise SceneError(f'scene.clearance.{key}: must be nonnegative')
    tool = _mapping(s['tool'], 'scene.tool', {'status', 'ee_frame', 'tcp_frame'})
    if tool != {'status': 'uncalibrated', 'ee_frame': 'end_effector_link', 'tcp_frame': None}:
        raise SceneError('scene.tool: B1 requires uncalibrated, end_effector_link, tcp_frame=null; no invented TCP')
    canonical = {'schema_version': 1, 'scene': {
        'id': name, 'provenance': dict(provenance), 'reference_frame': 'base_link', 'units': dict(units),
        'coverage': coverage,
        'frames': [{'id': f.id, 'parent': f.parent, 'pose': _pose_dict(f.pose)} for f in graph.frames],
        'collision_objects': sorted(objects, key=lambda obj: obj['id']),
        'entities': sorted(entities, key=lambda obj: obj['id']), 'ports': sorted(ports, key=lambda obj: obj['id']),
        'clearance': clearance, 'tool': dict(tool)}}
    text = json.dumps(canonical, sort_keys=True, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
    data = _freeze(canonical['scene'])
    return SceneSnapshot(name, data['provenance'], 'base_link', data['units'], coverage, graph.frames,
                         data['collision_objects'], data['entities'], data['ports'], data['clearance'], data['tool'],
                         graph, raw_hash, sha256(text.encode('utf-8')).hexdigest(), source, text)


def parse_scene(text: str | bytes, *, source_name: str = '<memory>') -> SceneSnapshot:
    """Validate UTF-8 YAML/JSON text (<=1 MiB), rejecting aliases and duplicate keys.

    Mapping/sequence nesting is limited to 64 and total YAML nodes to 20,000.
    source_name is diagnostic metadata and excluded from both content hashes.
    """
    if not isinstance(text, (str, bytes)):
        raise SceneError(f'{source_name}: expected UTF-8 text or bytes')
    loader = None
    try:
        raw = text.encode('utf-8') if isinstance(text, str) else text
        if len(raw) > MAX_YAML_BYTES:
            raise SceneError('YAML: exceeds 1 MiB size limit')
        loader = _Loader(raw.decode('utf-8'))
        node = loader.get_single_node()
        if node is None:
            raise SceneError('YAML: empty document')
        return _snapshot(_decode_node(loader, node, '$'), sha256(raw).hexdigest(), str(source_name))
    except (SceneError, TransformError, yaml.YAMLError, UnicodeError, RecursionError) as exc:
        raise SceneError(f'{source_name}: {exc}') from exc
    finally:
        if loader is not None:
            loader.dispose()


def load_scene(path: str | Path) -> SceneSnapshot:
    """Read an explicit scene path; no model loading, ROS setup, nodes or writes."""
    source = Path(path).expanduser().absolute()
    try:
        with source.open('rb') as stream:
            raw = stream.read(MAX_YAML_BYTES + 1)
    except OSError as exc:
        raise SceneError(f'{source}: cannot read scene: {exc}') from exc
    return parse_scene(raw, source_name=str(source))
