"""Strict schema, immutable snapshots and analytic synthetic-scene truth."""

from copy import deepcopy
from dataclasses import FrozenInstanceError
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
import yaml

from workspace_analysis.scene.schema import SceneError, load_scene, parse_scene
from workspace_analysis.scene.frames import resolve_transform
from workspace_analysis.geometry.se3 import transform_direction


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'config/scenes/synthetic_dev.yaml'


@pytest.fixture
def document():
    return yaml.safe_load(SCENE.read_text())


def parsed(document):
    return parse_scene(yaml.safe_dump(document, sort_keys=False))


def set_value(document, trail, value):
    parent = document
    for key in trail[:-1]:
        parent = parent[key]
    parent[trail[-1]] = value


def test_synthetic_scene_geometry_and_hand_computed_base_transforms():
    scene = load_scene(SCENE)
    assert scene.scene_id == 'synthetic_demo'
    assert scene.reference_frame == 'base_link'
    assert scene.provenance == {'kind': 'synthetic', 'hardware_verified': False}
    assert scene.coverage == 'partial_synthetic_fixture'
    assert scene.tool['status'] == 'uncalibrated' and scene.tool['tcp_frame'] is None
    assert len(scene.frames) == 3 and len(scene.collision_objects) == 9
    assert len(scene.entities) == 2 and len(scene.ports) == 1
    objects = {obj['id']: obj for obj in scene.collision_objects}
    assert objects['cab_left']['size_m'] == (.30, .02, .40)
    assert objects['synthetic_cylinder']['type'] == 'cylinder'
    assert objects['synthetic_cylinder']['radius_m'] == .015
    assert objects['synthetic_cylinder']['length_m'] == .08
    assert objects['oriented_test_box']['type'] == 'box'
    # Hand truth: cabinet translation (.45,0,.20), panel offset (-.15,0,0), Ry(-90).
    cabinet = np.eye(4)
    cabinet[:3, 3] = [.45, 0, .20]
    panel = np.array([[0., 0, -1, .30], [0, 1, 0, 0], [1, 0, 0, .20], [0, 0, 0, 1]])
    np.testing.assert_allclose(resolve_transform(scene, 'base_link', 'synthetic_demo/cabinet'), cabinet, atol=1e-12)
    for frame in ['synthetic_demo/panel', 'synthetic_demo/port_01_frame']:
        result = resolve_transform(scene, 'base_link', frame)
        np.testing.assert_allclose(result, panel, atol=1e-12, rtol=0)
        np.testing.assert_allclose(transform_direction(result, [0, 0, 1]), [-1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(transform_direction(result, [0, 0, -1]), [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(scene.object_transform('cab_right')[:3, 3], [.45, -.18, .20], atol=1e-12)
    np.testing.assert_allclose(scene.object_transform('panel_left')[:3, 3], [.30, 0, .155], atol=1e-12)
    a = np.sqrt(.5)
    np.testing.assert_allclose(scene.object_transform('oriented_test_box')[:3, :3],
                               [[a, -a, 0], [a, a, 0], [0, 0, 1]], atol=1e-12)
    with pytest.raises(SceneError, match='unknown object'):
        scene.object_transform('not_present')


def test_raw_and_canonical_hash_definitions_and_reload_determinism(tmp_path):
    first, second = load_scene(SCENE), load_scene(SCENE)
    assert first.raw_yaml_sha256 == second.raw_yaml_sha256 == sha256(SCENE.read_bytes()).hexdigest()
    assert first.to_json() == second.to_json()
    expected = json.dumps(first.to_dict(), ensure_ascii=False, sort_keys=True, separators=(',', ':'), allow_nan=False)
    assert first.to_json() == expected and not expected.endswith('\n')
    assert first.canonical_scene_sha256 == sha256(expected.encode('utf-8')).hexdigest()
    other = tmp_path / 'other_path.yaml'
    other.write_bytes(SCENE.read_bytes())
    assert load_scene(other).canonical_scene_sha256 == first.canonical_scene_sha256
    comment = parse_scene('# formatting change\n' + SCENE.read_text() + '\n')
    assert comment.raw_yaml_sha256 != first.raw_yaml_sha256
    assert comment.canonical_scene_sha256 == first.canonical_scene_sha256


def test_mapping_key_and_collection_order_do_not_change_canonical_hash(document):
    first = parsed(document)
    def reverse_keys(value):
        if isinstance(value, dict): return {k: reverse_keys(v) for k, v in reversed(list(value.items()))}
        if isinstance(value, list): return [reverse_keys(v) for v in value]
        return value
    changed = reverse_keys(document)
    for field in ['frames', 'entities', 'collision_objects', 'ports']:
        changed['scene'][field].reverse()
    for entity in changed['scene']['entities']:
        entity['components'].reverse()
    second = parsed(changed)
    assert first.raw_yaml_sha256 != second.raw_yaml_sha256
    assert first.canonical_scene_sha256 == second.canonical_scene_sha256


@pytest.mark.parametrize('mutation', ['geometry', 'frame', 'clearance', 'coverage'])
def test_semantic_change_changes_canonical_hash(document, mutation):
    first = parsed(document)
    if mutation == 'geometry': document['scene']['collision_objects'][0]['size_m'][0] += .01
    elif mutation == 'frame': document['scene']['frames'][0]['pose']['position_m'][0] += .01
    elif mutation == 'clearance': document['scene']['clearance']['self_m'] = .001
    else: document['scene']['coverage'] = 'changed_explicit_scope'
    assert parsed(document).canonical_scene_sha256 != first.canonical_scene_sha256


def test_quaternion_sign_and_negative_zero_do_not_change_canonical_hash(document):
    first = parsed(document)
    for record in document['scene']['frames'] + document['scene']['collision_objects']:
        record['pose']['quaternion_xyzw'] = [-v for v in record['pose']['quaternion_xyzw']]
    document['scene']['clearance']['self_m'] = -0.0
    assert parsed(document).canonical_scene_sha256 == first.canonical_scene_sha256


def test_scientific_numeric_tokens_roundtrip_but_quoted_numbers_are_rejected():
    text = SCENE.read_text().replace('contact_epsilon_m: 0.0000001', 'contact_epsilon_m: 1e-07')
    assert load_scene(SCENE).canonical_scene_sha256 == parse_scene(text).canonical_scene_sha256
    with pytest.raises(SceneError, match='contact_epsilon_m'):
        parse_scene(text.replace('contact_epsilon_m: 1e-07', 'contact_epsilon_m: "1e-07"'))
    with pytest.raises(SceneError, match='invalid YAML scalar'):
        parse_scene('scene: {id: !!float not_a_number}')


def test_snapshot_is_deeply_immutable_and_export_is_detached():
    scene = load_scene(SCENE)
    before = scene.to_json()
    with pytest.raises(FrozenInstanceError): scene.scene_id = 'changed'
    with pytest.raises(FrozenInstanceError): scene.frames[0].parent = 'changed'
    with pytest.raises(TypeError): scene.frames[0].pose.position_m[0] = 3
    with pytest.raises(TypeError): scene.provenance['kind'] = 'changed'
    with pytest.raises(TypeError): scene.units['length'] = 'mm'
    with pytest.raises(TypeError): scene.collision_objects[0]['id'] = 'changed'
    with pytest.raises(TypeError): scene.collision_objects[0]['pose']['position_m'][0] = 3
    with pytest.raises(TypeError): scene.collision_objects[0]['size_m'][0] = 3
    with pytest.raises(TypeError): scene.entities[0]['components'][0] = 'changed'
    with pytest.raises(TypeError): scene.ports[0]['insertion_axis_local'][0] = 1
    with pytest.raises(TypeError): scene.clearance['self_m'] = 3
    with pytest.raises(TypeError): scene.tool['status'] = 'calibrated'
    exported = scene.to_dict()
    exported['scene']['frames'][0]['pose']['position_m'][0] = 300
    exported['scene']['provenance']['kind'] = 'changed'
    result = resolve_transform(scene, 'base_link', 'synthetic_demo/panel')
    result.setflags(write=True)
    result[:] = 0
    assert scene.to_json() == before
    assert not hasattr(scene, '__dict__')
    # JSON is valid YAML too: canonical serialization can be reloaded independently.
    assert parse_scene(scene.to_json()).canonical_scene_sha256 == scene.canonical_scene_sha256


INVALID_VALUES = [
    (['schema_version'], 2), (['schema_version'], True), (['schema_version'], 1.0),
    (['scene', 'reference_frame'], 'base_footprint'),
    (['scene', 'units', 'length'], 'mm'), (['scene', 'units', 'angle'], 'deg'),
    (['scene', 'provenance', 'hardware_verified'], 'false'),
    (['scene', 'provenance', 'hardware_verified'], True),
    (['scene', 'frames', 0, 'pose', 'quaternion_xyzw'], [0, 0, 0, 0]),
    (['scene', 'frames', 0, 'pose', 'quaternion_xyzw'], [0, 0, 0, 2]),
    (['scene', 'frames', 0, 'pose', 'quaternion_xyzw'], [0, False, 0, 1]),
    (['scene', 'frames', 0, 'pose', 'position_m'], [0, 0, float('nan')]),
    (['scene', 'frames', 0, 'pose', 'position_m'], [0, True, 0]),
    (['scene', 'collision_objects', 0, 'size_m'], [0, .02, .4]),
    (['scene', 'collision_objects', 0, 'size_m'], [-.3, .02, .4]),
    (['scene', 'collision_objects', 0, 'size_m'], [.3, float('inf'), .4]),
    (['scene', 'collision_objects', 0, 'size_m'], [True, .02, .4]),
    (['scene', 'collision_objects', 0, 'size_m'], [.3, .4]),
    (['scene', 'collision_objects', 7, 'radius_m'], 0),
    (['scene', 'collision_objects', 7, 'length_m'], -.1),
    (['scene', 'collision_objects', 7, 'radius_m'], True),
    (['scene', 'collision_objects', 7, 'length_m'], '0.08'),
    (['scene', 'clearance', 'robot_environment_m'], -.001),
    (['scene', 'clearance', 'contact_epsilon_m'], False),
    (['scene', 'clearance', 'self_m'], float('nan')),
    (['scene', 'tool', 'status'], 'calibrated'), (['scene', 'tool', 'tcp_frame'], 'made_up_tcp'),
    (['scene', 'ports', 0, 'outward_axis_local'], [0, 0, -1]),
    (['scene', 'ports', 0, 'insertion_axis_local'], [0, 0, 1]),
    (['scene', 'ports', 0, 'key_axis_local'], [0, 0, 1]),
    (['scene', 'ports', 0, 'key_axis_local'], [2, 0, 0]),
]


@pytest.mark.parametrize('trail,value', INVALID_VALUES)
def test_illegal_fields_rejected_with_location(document, trail, value):
    set_value(document, trail, value)
    with pytest.raises(SceneError) as error:
        parsed(document)
    message = str(error.value)
    assert 'scene' in message or 'schema_version' in message
    assert trail[-1] in message or (trail[-1] == 'status' and 'scene.tool' in message)


@pytest.mark.parametrize('collection', ['frames', 'collision_objects', 'entities', 'ports'])
def test_duplicate_ids_rejected(document, collection):
    document['scene'][collection].append(deepcopy(document['scene'][collection][0]))
    with pytest.raises(SceneError, match='duplicate ID'):
        parsed(document)


@pytest.mark.parametrize('trail', [[], ['scene'], ['scene', 'provenance'], ['scene', 'frames', 0],
                                  ['scene', 'frames', 0, 'pose'], ['scene', 'collision_objects', 0],
                                  ['scene', 'entities', 0], ['scene', 'ports', 0], ['scene', 'tool']])
def test_unknown_fields_are_never_ignored(document, trail):
    parent = document
    for key in trail: parent = parent[key]
    parent['unexpected'] = 1
    with pytest.raises(SceneError, match='unknown fields.*unexpected'):
        parsed(document)


@pytest.mark.parametrize('case', ['missing_parent', 'self_cycle', 'cycle', 'namespace', 'object_frame', 'port_frame'])
def test_invalid_or_disconnected_frames_rejected(document, case):
    s = document['scene']
    if case == 'missing_parent': s['frames'][0]['parent'] = 'missing'
    elif case == 'self_cycle': s['frames'][0]['parent'] = s['frames'][0]['id']
    elif case == 'cycle': s['frames'][0]['parent'] = s['frames'][1]['id']
    elif case == 'namespace': s['frames'][0]['id'] = 'other_scene/cabinet'
    elif case == 'object_frame': s['collision_objects'][0]['parent_frame'] = 'missing'
    else: s['ports'][0]['frame'] = 'missing'
    with pytest.raises(SceneError):
        parsed(document)


@pytest.mark.parametrize('components', [[], ['not_present'], ['cab_left', 'cab_left']])
def test_component_references_rejected(document, components):
    document['scene']['entities'][0]['components'] = components
    with pytest.raises(SceneError, match='components'):
        parsed(document)


@pytest.mark.parametrize('case', ['missing_entity', 'cabinet_entity', 'wrong_panel', 'panel_frame_is_not_port'])
def test_port_must_belong_to_its_declared_panel(document, case):
    s = document['scene']
    if case == 'missing_entity': s['ports'][0]['entity'] = 'missing'
    elif case == 'cabinet_entity': s['ports'][0]['entity'] = 'cabinet_demo'
    elif case == 'panel_frame_is_not_port': s['ports'][0]['frame'] = 'synthetic_demo/panel'
    else:
        frame = deepcopy(s['frames'][1]); frame['id'] = 'synthetic_demo/other_panel'
        s['frames'].append(frame)
        panel = deepcopy(s['entities'][1]); panel['id'] = 'other_panel'; panel['frame'] = frame['id']
        s['entities'].append(panel)
        s['ports'][0]['entity'] = 'other_panel'
    with pytest.raises(SceneError, match='scene.ports'):
        parsed(document)


@pytest.mark.parametrize('text', [
    'schema_version: 1\nschema_version: 1\nscene: {}',
    'schema_version: 1\nscene: {id: a, id: b}',
    'scene: &anchor {}\ncopy: *anchor',
    'scene: {<<: {id: a}}',
    'scene: !unsupported {}',
    'scene: !!python/object:dangerous {}',
    'scene: {true: 1}',
    'scene: [unclosed', '', '---\nscene: {}\n---\nscene: {}',
])
def test_yaml_duplicates_aliases_tags_merge_and_syntax_errors(text):
    with pytest.raises(SceneError):
        parse_scene(text)


def test_duplicate_nested_pose_key_reports_path():
    text = SCENE.read_text().replace('position_m: [0.45, 0.0, 0.20]',
                                    'position_m: [0.45, 0.0, 0.20], position_m: [0, 0, 0]', 1)
    with pytest.raises(SceneError, match=r'frames\[0\].pose.position_m: duplicate YAML key'):
        parse_scene(text)


def test_missing_fields_bad_utf8_size_depth_and_missing_file(document, tmp_path):
    del document['scene']['frames'][0]['pose']['quaternion_xyzw']
    with pytest.raises(SceneError, match='quaternion_xyzw'): parsed(document)
    with pytest.raises(SceneError): parse_scene(b'\xff')
    with pytest.raises(SceneError, match='size limit'): parse_scene(' ' * (1024 * 1024 + 1))
    with pytest.raises(SceneError, match='limit'): parse_scene('[' * 70 + '0' + ']' * 70)
    with pytest.raises(SceneError, match='cannot read scene'): load_scene(tmp_path / 'missing.yaml')


def test_scene_only_import_and_loading_without_ros_or_backend():
    source = str(ROOT / 'src')
    code = (f'import sys; sys.path.insert(0,{source!r}); '
            'from workspace_analysis.scene.schema import load_scene; '
            f's = load_scene({str(SCENE)!r}); assert len(s.frames)==3; '
            "assert not any(n in sys.modules for n in ['workspace_analysis.robot.model','rclpy','PyKDL','fcl','moveit','plotly'])")
    result = subprocess.run([sys.executable, '-I', '-B', '-c', code], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr
