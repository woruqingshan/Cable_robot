"""B2 analytic geometry, real small Plotly export and G2 loader integration."""

from collections import Counter
from dataclasses import replace
from hashlib import sha256
from html.parser import HTMLParser
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

from workspace_analysis.scene.schema import load_scene, parse_scene
from workspace_analysis.viz import scene as viz
from workspace_analysis.viz import meshes


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'config/scenes/synthetic_dev.yaml'
CONFIG = ROOT / 'config/workspace.yaml'


@pytest.fixture
def scene():
    return load_scene(SCENE)


@pytest.fixture
def copied_pair(tmp_path):
    source = Path(os.environ.get('G3_TEST_INPUT_PREFIX', ROOT / 'results/workspace_n100_seed42'))
    prefix = tmp_path / source.name
    for extension in ('.npz', '.json'):
        original = Path(str(source) + extension)
        assert original.is_file(), f'Missing existing G2 fixture: {original}; tests never generate replacement sampling'
        shutil.copyfile(original, Path(str(prefix) + extension))
    return prefix


def traces(figure, role):
    return [trace for trace in figure.data if trace.meta and trace.meta.get('role') == role]


def test_box_eight_vertices_full_dimensions_and_closed_twelve_triangle_surface():
    vertices, faces = meshes.box_mesh([2, 4, 6])
    assert vertices.shape == (8, 3) and faces.shape == (12, 3)
    assert len(np.unique(vertices, axis=0)) == 8
    np.testing.assert_array_equal(vertices.min(axis=0), [-1, -2, -3])
    np.testing.assert_array_equal(vertices.max(axis=0), [1, 2, 3])
    counts = Counter(tuple(sorted((int(a), int(b)))) for f in faces for a, b in zip(f, np.roll(f, -1)))
    assert set(counts.values()) == {2}


def test_rotated_box_and_parent_frame_rotation_have_analytic_world_vertices(scene):
    vertices, _ = viz.object_mesh(scene, 'oriented_test_box')
    s = np.sqrt(.5)
    # Rz(45)*(-.04,-.01,-.03) + (.26,-.12,.12), hand-calculated.
    np.testing.assert_allclose(vertices[0], [.26 - .03*s, -.12 - .05*s, .09], atol=1e-12)
    panel_vertices, _ = viz.object_mesh(scene, 'panel_left')
    np.testing.assert_allclose(panel_vertices.min(axis=0), [.295, -.04, .140], atol=1e-12)
    np.testing.assert_allclose(panel_vertices.max(axis=0), [.305, .04, .170], atol=1e-12)


def test_cylinder_radius_length_caps_and_rotated_world_axis(scene):
    n = 16
    vertices, faces = meshes.cylinder_mesh(.015, .08, n)
    assert vertices.shape == (2*n+2, 3) and faces.shape == (4*n, 3)
    np.testing.assert_allclose(np.linalg.norm(vertices[:2*n, :2], axis=1), .015, atol=1e-12)
    np.testing.assert_array_equal(vertices[-2:, 2], [-.04, .04])
    counts = Counter(tuple(sorted((int(a), int(b)))) for f in faces for a, b in zip(f, np.roll(f, -1)))
    assert set(counts.values()) == {2}
    data = scene.to_dict()
    cylinder = next(o for o in data['scene']['collision_objects'] if o['id'] == 'synthetic_cylinder')
    cylinder['pose']['quaternion_xyzw'] = [0, np.sqrt(.5), 0, np.sqrt(.5)]
    rotated = parse_scene(json.dumps(data))
    world, _ = viz.object_mesh(rotated, 'synthetic_cylinder', n)
    # Ry(+90) sends local +Z to base +X, with center (.30,.10,.10).
    np.testing.assert_allclose(world[-2:], [[.26,.10,.10], [.34,.10,.10]], atol=1e-12)
    np.testing.assert_allclose(np.linalg.norm(world[:2*n, 1:] - [.10,.10], axis=1), .015, atol=1e-12)


def test_frame_axes_and_port_directions_are_dynamic_not_world_constants(scene):
    origin, ends = viz.frame_axes(scene, 'synthetic_demo/panel', .05)
    np.testing.assert_allclose(origin, [.3,0,.2], atol=1e-12)
    np.testing.assert_allclose(ends, [[.3,0,.25], [.3,.05,.2], [.25,0,.2]], atol=1e-12)
    port_origin, directions = viz.port_directions(scene, 'port_01')
    np.testing.assert_allclose(port_origin, [.3,0,.2], atol=1e-12)
    for name, expected in [('outward', [-1,0,0]), ('insertion', [1,0,0]), ('key', [0,0,1])]:
        np.testing.assert_allclose(directions[name], expected, atol=1e-12)
    document = scene.to_dict()
    document['scene']['frames'][0]['pose']['quaternion_xyzw'] = [0,0,np.sqrt(.5),np.sqrt(.5)]
    changed = parse_scene(json.dumps(document))
    _, changed_directions = viz.port_directions(changed, 'port_01')
    np.testing.assert_allclose(changed_directions['outward'], [0,-1,0], atol=1e-12)
    np.testing.assert_allclose(changed_directions['insertion'], [0,1,0], atol=1e-12)


def test_scene_figure_has_exactly_nine_solids_semantic_colors_and_frame_groups(scene):
    before = scene.to_json()
    figure = viz.build_scene_figure(scene, cylinder_segments=16)
    solids = traces(figure, 'scene_object')
    assert len(solids) == 9 and all(t.type == 'mesh3d' for t in solids)
    assert {t.meta['object_id'] for t in solids} == {o['id'] for o in scene.collision_objects}
    assert Counter(t.meta['category'] for t in solids) == {'cabinet': 3, 'patch_panel': 4, 'independent_obstacle': 2}
    assert len(traces(figure, 'object_edges')) == 9
    assert len(traces(figure, 'frame_axis')) == 12  # root plus all three B1 frames
    assert len(traces(figure, 'port_direction')) == 3
    assert len(traces(figure, 'base_origin')) == 1
    for trace in traces(figure, 'frame_axis'):
        assert trace.line.color == {'X':'#dc2626','Y':'#16a34a','Z':'#2563eb'}[trace.meta['axis']]
        assert trace.legendgroup == 'frame:' + trace.meta['frame_id']
    for trace in solids:
        assert trace.showlegend and 'Object ID:' in trace.hovertemplate and 'Geometry [m]' in trace.hovertemplate
    assert figure.layout.legend.groupclick == 'togglegroup' and figure.layout.legend.orientation == 'v'
    assert figure.layout.scene.aspectmode == 'data'
    assert figure.layout.scene.xaxis.title.text == 'X [m]'
    assert 'Synthetic Scene / Not Hardware Verified' in figure.layout.title.text
    assert 'overlap does not prove collision' in figure.layout.annotations[0].text
    assert figure.layout.meta['workspace'] is None and figure.layout.meta['collision_checked'] is False
    assert scene.to_json() == before


def test_workspace_uses_g3_loader_without_resampling_and_preserves_all_rows(copied_pair, monkeypatch):
    from workspace_analysis.viz import workspace as g3
    from workspace_analysis.workspace import sampling as sampling
    actual, observed = g3.load_workspace, []
    def checked(prefix, config, max_points=None):
        view = actual(prefix, config, max_points=max_points)
        observed.append(view)
        return view
    def forbidden(*args, **kwargs):
        raise AssertionError('B2 must not resample')
    monkeypatch.setattr(g3, 'load_workspace', checked)
    monkeypatch.setattr(sampling, 'sample_workspace', forbidden)
    figure = viz.load_scene_figure(SCENE, input_prefix=copied_pair, config_path=CONFIG)
    assert len(observed) == 1
    view = observed[0]
    trace = traces(figure, 'workspace_samples')[0]
    assert len(trace.x) == view.total_samples == 100
    np.testing.assert_array_equal(trace.customdata, view.customdata)
    np.testing.assert_array_equal(trace.z, view.position[:,2])
    assert (np.asarray(trace.z) < 0).sum() == (view.position[:,2] < 0).sum() > 0
    for name in view.joint_order: assert name in trace.hovertemplate
    assert 'Sample Index' in trace.hovertemplate
    assert figure.layout.meta['workspace']['npz_sha256'] == view.metadata['dataset']['sha256']
    assert len(traces(figure, 'base_origin')) == 1


def test_display_limit_keeps_deterministic_original_indices(copied_pair):
    first = viz.load_scene_figure(SCENE, input_prefix=copied_pair, config_path=CONFIG, max_points=7)
    second = viz.load_scene_figure(SCENE, input_prefix=copied_pair, config_path=CONFIG, max_points=7)
    a, b = traces(first, 'workspace_samples')[0], traces(second, 'workspace_samples')[0]
    np.testing.assert_array_equal(a.customdata, b.customdata)
    np.testing.assert_array_equal(a.customdata[:,0], np.linspace(0,99,7,dtype=np.int64))
    meta = first.layout.meta['workspace']
    assert meta['displayed_samples'] == 7 and meta['total_samples'] == 100
    assert '7 shown / 100 total' in first.layout.title.text


def test_reference_frame_mismatch_is_rejected(scene, copied_pair):
    with pytest.raises(viz.SceneVisualizationError, match='reference frame'):
        viz.build_scene_figure(replace(scene, reference_frame='other'))
    from workspace_analysis.viz.workspace import load_workspace
    view = load_workspace(copied_pair, CONFIG)
    view.metadata['robot_model']['base_frame'] = 'base_footprint'
    with pytest.raises(viz.SceneVisualizationError, match='Base Frames'):
        viz.build_scene_figure(scene, view)


@pytest.mark.parametrize('damage', ['missing', 'npz_hash', 'json', 'model'])
def test_corrupt_or_incompatible_g2_inputs_rejected(copied_pair, damage, tmp_path):
    config = CONFIG
    if damage == 'missing': Path(str(copied_pair) + '.npz').unlink()
    elif damage == 'npz_hash':
        path = Path(str(copied_pair) + '.npz'); path.write_bytes(path.read_bytes() + b'corrupt')
    elif damage == 'json': Path(str(copied_pair) + '.json').write_text('invalid json')
    else:
        config = tmp_path / 'different.yaml'
        config.write_text(CONFIG.read_text().replace('base_frame: base_link', 'base_frame: base_footprint'))
    with pytest.raises(ValueError):
        viz.load_scene_figure(SCENE, input_prefix=copied_pair, config_path=config)


@pytest.mark.parametrize('options', [{'axis_length':0}, {'axis_length':True}, {'axis_length':float('nan')},
                                    {'cylinder_segments':7}, {'cylinder_segments':257}, {'cylinder_segments':True}])
def test_bad_display_options_rejected(scene, options):
    with pytest.raises(viz.SceneVisualizationError): viz.build_scene_figure(scene, **options)


class ScriptSources(HTMLParser):
    def __init__(self): super().__init__(); self.sources = []
    def handle_starttag(self, tag, attrs):
        if tag == 'script': self.sources += [value for key,value in attrs if key == 'src']


def test_real_html_offline_export_and_existing_output_protection(scene, tmp_path):
    output = tmp_path / 'scene.html'
    path = viz.export_scene_html(viz.build_scene_figure(scene, cylinder_segments=8), output)
    text = path.read_text()
    parser = ScriptSources(); parser.feed(text)
    assert not parser.sources
    assert 'Plotly.newPlot' in text and 'plotly.js' in text and len(text)>10000
    assert 'Synthetic Scene' in text and 'canonical_scene_sha256' in text
    before = output.read_bytes()
    with pytest.raises(viz.SceneVisualizationError, match='Refusing to overwrite'):
        viz.export_scene_html(viz.build_scene_figure(scene), output)
    assert output.read_bytes() == before


def test_scene_only_cli_works_in_isolated_python_without_ros_or_g2(tmp_path):
    output = tmp_path / 'isolated.html'
    source = str(ROOT / 'src')
    code = (f'import sys; sys.path.insert(0,{source!r}); '
            'from workspace_analysis.viz.scene import load_scene_figure, export_scene_html; '
            f'fig=load_scene_figure({str(SCENE)!r},config_path="missing_unused.yaml",cylinder_segments=8); '
            f'export_scene_html(fig,{str(output)!r}); '
            "assert not any(n in sys.modules for n in ['rclpy','PyKDL','workspace_analysis.robot.model','workspace_analysis.viz.workspace'])")
    result = subprocess.run([sys.executable,'-I','-B','-c',code],cwd=tmp_path,text=True,capture_output=True,timeout=30)
    assert result.returncode == 0, result.stderr
    assert output.stat().st_size>10000
    cli_output = tmp_path / 'scene_only_cli.html'
    cli = subprocess.run([sys.executable,'-I','-B',str(ROOT/'scripts/inspect_scene.py'),
                          '--scene',str(SCENE),'--output',str(cli_output),'--config','unused_missing.yaml',
                          '--cylinder-segments','8'],cwd=tmp_path,text=True,capture_output=True,timeout=30)
    assert cli.returncode == 0, cli.stderr
    assert 'Mode: Scene Only' in cli.stdout and 'status: PASS' in cli.stdout


def test_cli_100_point_overlay_preserves_yaml_npz_json_and_errors_nonzero(copied_pair, tmp_path):
    inputs = [SCENE, Path(str(copied_pair)+'.npz'), Path(str(copied_pair)+'.json')]
    before = [sha256(p.read_bytes()).hexdigest() for p in inputs]
    output = tmp_path / 'workspace.html'
    command = [sys.executable,str(ROOT/'scripts/inspect_scene.py'),'--scene',str(SCENE),'--config',str(CONFIG),
               '--input-prefix',str(copied_pair),'--output',str(output),'--cylinder-segments','8']
    result = subprocess.run(command,cwd=tmp_path,text=True,capture_output=True,timeout=30)
    assert result.returncode == 0, result.stderr
    assert 'status: PASS' in result.stdout and '100 displayed / 100 total' in result.stdout
    assert before == [sha256(p.read_bytes()).hexdigest() for p in inputs]
    existing = output.read_bytes()
    again = subprocess.run(command,cwd=tmp_path,text=True,capture_output=True,timeout=30)
    assert again.returncode != 0 and 'status: ERROR' in again.stderr
    assert 'status: PASS' not in again.stdout and output.read_bytes() == existing
    # Missing source input must fail, never silently regenerate sampling.
    invalid = subprocess.run([sys.executable,str(ROOT/'scripts/inspect_scene.py'),'--scene',str(SCENE),
                              '--input-prefix',str(tmp_path/'missing'),'--output',str(tmp_path/'missing.html')],
                             cwd=tmp_path,text=True,capture_output=True,timeout=30)
    assert invalid.returncode != 0 and 'status: PASS' not in invalid.stdout
    assert not (tmp_path/'missing.html').exists()


def test_publication_failure_and_racing_writer_do_not_overwrite_or_leave_partial_files(scene, tmp_path, monkeypatch):
    figure = viz.build_scene_figure(scene, cylinder_segments=8)
    real = os.link
    def racing(source, target):
        target.write_text('other writer')
        real(source,target)
    monkeypatch.setattr(os,'link',racing)
    target = tmp_path/'race.html'
    with pytest.raises(viz.SceneVisualizationError): viz.export_scene_html(figure,target)
    assert target.read_text() == 'other writer'
    assert [p.name for p in tmp_path.iterdir()] == ['race.html']


def test_render_failure_is_not_success_and_dangling_symlink_is_protected(scene, tmp_path, monkeypatch):
    import plotly.io as pio
    def failed(*args, **kwargs): raise OSError('injected renderer failure')
    monkeypatch.setattr(pio,'to_html',failed)
    with pytest.raises(OSError):
        viz.export_scene_html(viz.build_scene_figure(scene), tmp_path/'failed.html')
    assert list(tmp_path.iterdir()) == []
    link = tmp_path/'existing.html'; link.symlink_to(tmp_path/'not_present')
    with pytest.raises(viz.SceneVisualizationError, match='Refusing to overwrite'): viz.check_html_available(link)


def test_cli_export_failure_returns_nonzero_without_printing_pass(tmp_path):
    blocker = tmp_path/'not_a_directory'
    blocker.write_text('keep')
    result = subprocess.run([sys.executable,str(ROOT/'scripts/inspect_scene.py'), '--scene',str(SCENE),
                             '--output',str(blocker/'scene.html'),'--cylinder-segments','8'],
                            cwd=tmp_path,text=True,capture_output=True,timeout=30)
    assert result.returncode != 0 and 'status: ERROR' in result.stderr
    assert 'status: PASS' not in result.stdout and blocker.read_text() == 'keep'
