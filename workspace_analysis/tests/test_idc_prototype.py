"""S1 fixture geometry checks only; no collision backend, IK or reachability claim."""

from test_resources import ARTIFACTS
from hashlib import sha256
import json
from pathlib import Path

import numpy as np
import pytest

from workspace_analysis.scene.schema import load_scene, parse_scene
from workspace_analysis.scene.frames import resolve_transform
from workspace_analysis.viz.scene import build_scene_figure, load_scene_figure, port_directions

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'config/scenes/idc_prototype_v0.yaml'
R = np.array([[0,0,-1],[0,1,0],[1,0,0]])


@pytest.fixture(scope='module')
def scene():
    return load_scene(SCENE)


def test_schema_entities_frames_and_unique_references(scene):
    assert scene.reference_frame == 'base_link'
    assert scene.provenance['kind'] == 'synthetic' and scene.provenance['hardware_verified'] is False
    assert len(scene.entities) == 4 and len(scene.ports) == 9
    assert len(scene.collision_objects) == 38 and len(scene.frames) == 13
    assert {p['id'] for p in scene.ports} == {f'P{i}{j}' for i in range(1,4) for j in range(1,4)}
    panels = {e['id']: e for e in scene.entities if e['kind'] == 'patch_panel'}
    assert set(panels) == {'router_1','router_2','router_3'}
    objects = {o['id'] for o in scene.collision_objects}
    for panel in panels.values():
        assert len(panel['components']) == 11 and set(panel['components']) <= objects
    assert set().union(*(set(e['components']) for e in scene.entities)) == objects
    assert sum(len(e['components']) for e in scene.entities) == len(objects)
    for port in scene.ports:
        assert port['entity'] in panels
        assert scene.frame_graph.is_descendant(port['frame'],panels[port['entity']]['frame'])
    for frame in scene.frames:
        matrix = resolve_transform(scene,'base_link',frame.id)
        assert np.isfinite(matrix).all() and matrix.shape == (4,4)


@pytest.mark.parametrize('row,col',[(i,j) for i in range(1,4) for j in range(1,4)])
def test_port_hand_truth_order_plane_axes_and_open_tunnel(scene,row,col):
    name = f'P{row}{col}'
    expected = [.33, {1:.07,2:0,3:-.07}[col], {1:.32,2:.22,3:.12}[row]]
    transform = resolve_transform(scene,'base_link','idc_prototype_v0/'+name)
    np.testing.assert_allclose(transform[:3,3],expected,atol=1e-14,rtol=0)
    np.testing.assert_allclose(transform[:3,:3],R,atol=1e-14,rtol=0)
    origin,directions = port_directions(scene,name)
    np.testing.assert_allclose(origin,expected,atol=1e-14,rtol=0)
    for axis,truth in [('outward',[-1,0,0]),('insertion',[1,0,0]),('key',[0,0,1])]:
        np.testing.assert_allclose(directions[axis],truth,atol=1e-14,rtol=0)
    # Independent local-coordinate construction checks: panel strips surround
    # a 16 mm (up) x 18 mm (horizontal) aperture, through 5 mm thickness.
    parts = {o['id'].removeprefix(f'router_{row}_'):o for o in scene.collision_objects
             if o['id'].startswith(f'router_{row}_')}
    bounds = {k:(np.array(o['pose']['position_m'])-np.array(o['size_m'])/2,
                 np.array(o['pose']['position_m'])+np.array(o['size_m'])/2) for k,o in parts.items()}
    np.testing.assert_allclose(bounds['front_top'][0],[.008,-.13,-.005],atol=1e-14)
    np.testing.assert_allclose(bounds['front_bottom'][1],[-.008,.13,0],atol=1e-14)
    horizontal = sorted((bounds[f'front_bar_{k}'][0][1],bounds[f'front_bar_{k}'][1][1]) for k in range(1,5))
    gaps = [(horizontal[k][1],horizontal[k+1][0]) for k in range(3)]
    y = {1:.07,2:0,3:-.07}[col]
    matching = [(lo,hi) for lo,hi in gaps if lo < y < hi]
    assert len(matching) == 1
    np.testing.assert_allclose(matching[0],[y-.009,y+.009],atol=1e-14)
    # Tunnel has the same open cross-section until local Z=-.135;
    # rear wall then stops it. No statement about robot/plug clearance.
    assert bounds['shell_top'][0][0] > .008
    assert bounds['shell_bottom'][1][0] < -.008
    assert bounds['shell_left'][0][1] > y+.009
    assert bounds['shell_right'][1][1] < y-.009
    np.testing.assert_allclose(bounds['back'][1][2],-.135,atol=1e-14)


def test_p22_complete_matrix_and_parent_transform(scene):
    expected = np.array([[0,0,-1,.33],[0,1,0,0],[1,0,0,.22],[0,0,0,1]])
    np.testing.assert_allclose(resolve_transform(scene,'base_link','idc_prototype_v0/P22'),expected,atol=1e-14)
    np.testing.assert_allclose(resolve_transform(scene,'base_link','idc_prototype_v0/cabinet')[:3,3],[.43,0,.22])
    for row in range(1,4):
        for col in range(1,4):
            local = resolve_transform(scene,'idc_prototype_v0/cabinet',f'idc_prototype_v0/P{row}{col}')
            np.testing.assert_allclose(local[:3,3],[-.10,{1:.07,2:0,3:-.07}[col],{1:.10,2:0,3:-.10}[row]],atol=1e-14)


def test_box_dimensions_shell_coverage_and_empty_cabinet(scene):
    assert all(o['type']=='box' and np.all(np.array(o['size_m'])>0) for o in scene.collision_objects)
    cabinet = [o for o in scene.collision_objects if o['id'].startswith('cab_')]
    assert len(cabinet) == 5
    expected = {'cab_left':([0,.145,0],[.20,.01,.34]),'cab_right':([0,-.145,0],[.20,.01,.34]),
                'cab_top':([0,0,.175],[.20,.30,.01]),'cab_bottom':([0,0,-.175],[.20,.30,.01]),
                'cab_back':([.095,0,0],[.01,.28,.34])}
    for obj in cabinet:
        xyz,size = expected[obj['id']]
        np.testing.assert_allclose(obj['pose']['position_m'],xyz)
        np.testing.assert_allclose(obj['size_m'],size)
    assert sum(np.prod(o['size_m']) for o in cabinet) < .25*(.20*.30*.36)
    # All three devices have identical local geometry and remain inside the
    # cabinet's clear width/height, with rear gap .05 m and vertical gaps .04 m.
    shape = lambda i: {o['id'].removeprefix(f'router_{i}_'):(o['size_m'],o['pose'])
                       for o in scene.collision_objects if o['id'].startswith(f'router_{i}_')}
    assert shape(1) == shape(2) == shape(3)
    from workspace_analysis.viz.scene import object_mesh
    extents = []
    for i in range(1,4):
        vertices = np.vstack([object_mesh(scene,o['id'])[0] for o in scene.collision_objects
                              if o['id'].startswith(f'router_{i}_')])
        lower,upper = vertices.min(axis=0),vertices.max(axis=0)
        np.testing.assert_allclose(upper-lower,[.14,.26,.06],atol=1e-14)
        assert lower[1] > -.14 and upper[1] < .14
        assert lower[2] > .05 and upper[2] < .39
        np.testing.assert_allclose(.52-upper[0],.05,atol=1e-14)
        extents.append((lower,upper))
    for upper_device,lower_device in zip(extents,extents[1:]):
        np.testing.assert_allclose(upper_device[0][2]-lower_device[1][2],.04,atol=1e-14)


def test_canonical_hash_reproducibility_and_synthetic_dev_unchanged(scene):
    assert load_scene(SCENE).canonical_scene_sha256 == scene.canonical_scene_sha256
    assert parse_scene(scene.to_json()).canonical_scene_sha256 == scene.canonical_scene_sha256
    assert sha256(scene.to_json().encode()).hexdigest() == scene.canonical_scene_sha256
    original = ROOT/'config/scenes/synthetic_dev.yaml'
    assert sha256(original.read_bytes()).hexdigest() == 'cae02112d6a051b0bff3a6c7c8698ee1acc9a12e226fd75bd4df9b683ab5fba4'
    assert load_scene(original).canonical_scene_sha256 == '0196cd18fc10ef9e4a23cc054e1aecfcb4e3509f220344ee538d152a5fb210a6'


def test_scene_only_uses_actual_solids_and_all_semantic_ports(scene):
    figure = build_scene_figure(scene,axis_length=.015)
    solids = [t for t in figure.data if t.meta and t.meta.get('role')=='scene_object']
    directions = [t for t in figure.data if t.meta and t.meta.get('role')=='port_direction']
    assert len(solids) == 38 and len(directions) == 27
    assert {t.meta['port_id'] for t in directions} == {p['id'] for p in scene.ports}
    assert figure.layout.meta['collision_checked'] is False and figure.layout.meta['workspace'] is None


def test_existing_100_points_overlay_without_resampling(monkeypatch):
    from workspace_analysis.workspace import sampling
    from workspace_analysis.viz.workspace import load_workspace
    def forbidden(*args,**kwargs):
        pytest.fail('S1 must reuse accepted G2 data')
    monkeypatch.setattr(sampling,'sample_workspace',forbidden)
    prefix = ARTIFACTS / 'workspace_n100_seed42'
    paths = [SCENE,Path(str(prefix)+'.npz'),Path(str(prefix)+'.json')]
    before = [sha256(p.read_bytes()).hexdigest() for p in paths]
    view = load_workspace(prefix,ROOT/'config/workspace.yaml')
    figure = load_scene_figure(SCENE,input_prefix=prefix,config_path=ROOT/'config/workspace.yaml',axis_length=.015)
    trace = next(t for t in figure.data if t.meta and t.meta.get('role')=='workspace_samples')
    assert len(trace.x) == 100
    np.testing.assert_array_equal(trace.customdata,view.customdata)
    np.testing.assert_array_equal(trace.z,view.position[:,2])
    assert np.any(np.array(trace.z)<0)
    assert before == [sha256(p.read_bytes()).hexdigest() for p in paths]
