"""R1 import boundaries, package DAG, and frozen pre-refactor numeric evidence."""

from test_resources import ARTIFACTS

import ast
from hashlib import sha256
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/workspace_analysis'
SOURCE = Path(os.environ.get('G4_TEST_SOURCE_ROOT', ROOT))
BASELINE = json.loads((ROOT / 'tests/fixtures/r1_numeric_baseline.json').read_text())
PURE_MODULES = (
    'geometry.se3', 'scene.frames', 'scene.model', 'scene.schema', 'task.spec', 'task.targets',
    'workspace.artifacts', 'workspace.sampling', 'workspace.results',
    'viz.meshes', 'viz.scene', 'viz.workspace', 'viz.task', 'viz.joint_path', 'viz.approach',
    'planning.joint_path', 'planning.rrt_connect',
    'experiments.statistics', 'experiments.reporting', 'experiments.tf_validation',
)


def test_root_has_only_public_entrypoint_and_subpackages_do_not_eagerly_load_features():
    assert {p.name for p in PACKAGE.glob('*.py')} == {'__init__.py'}
    for path in PACKAGE.glob('*/__init__.py'):
        nodes = ast.parse(path.read_text()).body
        assert len(nodes) == 1 and isinstance(nodes[0], ast.Expr)
        assert isinstance(nodes[0].value, ast.Constant) and isinstance(nodes[0].value.value, str)


def test_all_static_imports_including_lazy_imports_form_a_dag_and_obey_domain_boundaries():
    paths = {'.'.join(p.relative_to(PACKAGE.parent).with_suffix('').parts): p
             for p in PACKAGE.rglob('*.py') if p.name != '__init__.py'}
    allowed = {
        'geometry': {'geometry'}, 'robot': {'robot', 'geometry'},
        'scene': {'scene', 'geometry'}, 'workspace': {'workspace', 'robot', 'geometry'},
        'task': {'task', 'scene', 'geometry'},
        'planning': {'planning', 'task', 'robot', 'collision', 'geometry'},
        'viz': {'viz', 'workspace', 'scene', 'robot', 'geometry', 'task'},
        'experiments': {'experiments', 'workspace', 'scene', 'robot', 'geometry', 'viz', 'collision', 'task', 'planning'},
        'collision': {'collision', 'workspace', 'scene', 'robot', 'geometry'},
    }
    edges = {name: set() for name in paths}
    for name, path in paths.items():
        for node in ast.walk(ast.parse(path.read_text())):
            targets = []
            if isinstance(node, ast.ImportFrom):
                target = (importlib.util.resolve_name('.' * node.level + (node.module or ''), name.rsplit('.', 1)[0])
                          if node.level else node.module)
                targets = [target] + [target + '.' + alias.name for alias in node.names]
            elif isinstance(node, ast.Import):
                targets = [alias.name for alias in node.names]
            elif isinstance(node, ast.Call) and (
                    isinstance(node.func, ast.Name) and node.func.id in {'__import__', 'import_module'}
                    or isinstance(node.func, ast.Attribute) and node.func.attr == 'import_module'):
                pytest.fail(f'Feature modules must use inspectable imports: {name}:{node.lineno}')
            for target in targets:
                if target and target.startswith('workspace_analysis.'):
                    assert target.split('.')[1] in allowed[name.split('.')[1]], (name, target)
                    if target in paths:
                        edges[name].add(target)
    visited, active = set(), set()
    def visit(name):
        assert name not in active, f'Import cycle at {name}'
        if name not in visited:
            active.add(name)
            for target in edges[name]:
                visit(target)
            active.remove(name)
            visited.add(name)
    for name in paths:
        visit(name)
    assert edges['workspace_analysis.experiments.reporting'] == {
        'workspace_analysis.experiments.statistics', 'workspace_analysis.workspace.artifacts'}
    assert 'workspace_analysis.scene.schema' not in edges['workspace_analysis.scene.model']


@pytest.mark.parametrize('module', PURE_MODULES)
def test_import_with_optional_dependencies_blocked_and_no_process_network_or_environment_side_effects(module, tmp_path):
    code = f'''
import importlib, importlib.abc, os, sys
sys.path.insert(0, {str(ROOT / 'src')!r})
blocked = {{'rclpy', 'PyKDL', 'urdf_parser_py', 'ament_index_python', 'xacro', 'plotly', 'moveit', 'fcl'}}
class BlockOptional(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in blocked:
            raise AssertionError('Unexpected optional import: ' + fullname)
sys.meta_path.insert(0, BlockOptional())
def audit(event, args):
    if event in ('subprocess.Popen', 'socket.bind', 'socket.connect', 'os.system', 'os.fork'):
        raise AssertionError('Unexpected import side effect: ' + event)
sys.addaudithook(audit)
before = dict(os.environ)
for name in ('geometry', 'robot', 'scene', 'workspace', 'viz', 'experiments'):
    importlib.import_module('workspace_analysis.' + name)
importlib.import_module('workspace_analysis.' + {module!r})
assert dict(os.environ) == before
assert not blocked.intersection(n.split('.')[0] for n in sys.modules)
'''
    result = subprocess.run([sys.executable, '-I', '-B', '-c', code], cwd=tmp_path,
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr


def test_pure_scene_load_and_existing_report_read_work_without_robot_or_plotly(tmp_path):
    code = f'''
import importlib.abc, sys
sys.path.insert(0, {str(ROOT / 'src')!r})
class BlockRobot(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('workspace_analysis.robot', 'workspace_analysis.experiments.workspace_validation')) or fullname.split('.')[0] in {{'rclpy','PyKDL','urdf_parser_py','plotly','moveit','fcl'}}:
            raise AssertionError('Unexpected dependency: ' + fullname)
sys.meta_path.insert(0, BlockRobot())
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.experiments.reporting import load_validation_bundle
s = load_scene({str(ROOT / 'config/scenes/synthetic_dev.yaml')!r})
assert s.canonical_scene_sha256 == {BASELINE['scene']['canonical']!r}
r = load_validation_bundle({str(ARTIFACTS / 'g4_run01')!r})
assert r['status'] == 'PASS' and len(r['experiments']) == 9
'''
    result = subprocess.run([sys.executable, '-I', '-B', '-c', code], cwd=tmp_path,
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr


def test_frozen_scene_frame_port_mesh_baseline():
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.scene.frames import resolve_transform
    from workspace_analysis.viz.scene import object_mesh, port_directions
    scene = load_scene(ROOT / 'config/scenes/synthetic_dev.yaml')
    assert scene.raw_yaml_sha256 == BASELINE['scene']['raw']
    assert scene.canonical_scene_sha256 == BASELINE['scene']['canonical']
    for frame, matrix in BASELINE['frames'].items():
        np.testing.assert_array_equal(resolve_transform(scene, 'base_link', frame), matrix)
    for port, (origin, directions) in BASELINE['port'].items():
        actual_origin, actual_directions = port_directions(scene, port)
        np.testing.assert_array_equal(actual_origin, origin)
        for key, value in directions.items():
            np.testing.assert_array_equal(actual_directions[key], value)
    for name, expected in BASELINE['meshes'].items():
        vertices, faces = object_mesh(scene, name)
        for key, value in [('vertices', vertices), ('faces', faces)]:
            assert list(value.shape) == expected[key]['shape']
            assert str(value.dtype) == expected[key]['dtype']
            assert sha256(value.tobytes()).hexdigest() == expected[key]['sha256']


def test_frozen_three_pose_and_small_sampling_baseline():
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.robot.fk import ForwardKinematics
    from workspace_analysis.workspace.sampling import sample_workspace
    model = load_model(ROOT / 'config/workspace.yaml')
    fk = ForwardKinematics(model)
    for name, q in [('zero', [0]*5), ('nonzero', [.35, -.6, .8, -.4, .2]),
                    ('near_limit', [2.08, -.4, .5, -.3, .25])]:
        np.testing.assert_array_equal(fk(q), BASELINE['fk'][name])
    for key, value in sample_workspace(model, 16, 42).arrays().items():
        expected = BASELINE['sampling16'][key]
        assert list(value.shape) == expected['shape']
        assert str(value.dtype) == expected['dtype']
        assert sha256(value.tobytes()).hexdigest() == expected['sha256']


def test_code_provenance_covers_all_nested_package_sources():
    from workspace_analysis.experiments.workspace_validation import _code_hashes
    hashes = _code_hashes()
    expected = {str(p.resolve()): sha256(p.read_bytes()).hexdigest() for p in PACKAGE.rglob('*.py')}
    assert hashes == expected


def test_default_scene_overlay_config_still_resolves_to_workspace_root(monkeypatch):
    from workspace_analysis.viz import scene, workspace
    called = []
    def stop(prefix, config, *, max_points):
        called.append((prefix, config, max_points))
        raise RuntimeError('captured before loading')
    monkeypatch.setattr(workspace, 'load_workspace', stop)
    with pytest.raises(RuntimeError, match='captured before loading'):
        scene.load_scene_figure(ROOT / 'config/scenes/synthetic_dev.yaml', input_prefix='sentinel')
    assert called == [('sentinel', ROOT / 'config/workspace.yaml', None)]


@pytest.mark.parametrize('name', ['check_fk', 'validate_tf', 'sample_workspace', 'plot_workspace',
                                'validate_workspace', 'inspect_scene'])
def test_all_cli_help_from_another_working_directory(name, tmp_path):
    result = subprocess.run([sys.executable, '-I', '-B', str(ROOT / f'scripts/{name}.py'), '--help'],
                            cwd=tmp_path, text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stderr
    assert 'usage:' in result.stdout and '--help' in result.stdout
    assert not list(tmp_path.iterdir())


def test_fk_cli_reference_output_matches_pre_refactor_baseline(tmp_path):
    command = [sys.executable, '-B', str(ROOT / 'scripts/check_fk.py'), '--reference']
    result = subprocess.run(command, cwd=tmp_path, text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report['status'] == 'SOFTWARE_CHECKS_PASSED'
    assert report['reference_checked'] is True and report['tf_checked'] is False
    for row, key in zip(report['poses'], ('zero', 'nonzero', 'near_limit')):
        np.testing.assert_array_equal(row['T_base_to_ee'], BASELINE['fk'][key])
        assert row['position_error_m'] <= 1e-5 and row['rotation_error_rad'] <= 1e-4
    assert not list(tmp_path.iterdir())


def test_tf_cli_invalid_timeout_fails_before_creating_report_or_starting_ros(tmp_path):
    output = tmp_path / 'must_not_exist.json'
    command = [sys.executable, '-B', str(ROOT / 'scripts/validate_tf.py'),
               '--timeout', 'nan', '--output', str(output)]
    result = subprocess.run(command, cwd=tmp_path, text=True, capture_output=True, timeout=20)
    assert result.returncode == 2
    assert '--timeout must be finite and positive' in result.stderr
    assert not output.exists() and not list(tmp_path.iterdir())
