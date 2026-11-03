"""Analytic SE(3) truth and malformed inputs; no ROS nodes or collision backend."""

from dataclasses import FrozenInstanceError
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from workspace_analysis.geometry import se3 as t
from workspace_analysis.scene import frames as frame_graph


QZ90 = [0, 0, np.sqrt(.5), np.sqrt(.5)]
RZ90 = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])


def test_identity_translation_and_analytic_90_degree_rotation():
    np.testing.assert_array_equal(t.pose_to_matrix([0, 0, 0], [0, 0, 0, 1]), np.eye(4))
    transform = t.pose_to_matrix([1, 2, 3], QZ90)
    # Hand truth: Rz(90)*(2,0,1)=(0,2,1); then add (1,2,3).
    np.testing.assert_allclose(transform[:3, :3], RZ90, atol=1e-12, rtol=0)
    np.testing.assert_allclose(t.transform_point(transform, [2, 0, 1]), [1, 4, 4], atol=1e-12)
    np.testing.assert_allclose(t.transform_direction(transform, [2, 0, 1]), [0, 2, 1], atol=1e-12)


def test_composition_is_noncommutative_and_inverse_is_rigid():
    translation = t.pose_to_matrix([1, 0, 0], [0, 0, 0, 1])
    rotation = t.pose_to_matrix([0, 0, 0], QZ90)
    a, b = t.compose_transforms(translation, rotation), t.compose_transforms(rotation, translation)
    np.testing.assert_allclose(a[:3, 3], [1, 0, 0], atol=1e-12)
    np.testing.assert_allclose(b[:3, 3], [0, 1, 0], atol=1e-12)
    assert not np.allclose(a, b)
    np.testing.assert_allclose(t.compose_transforms(a, t.invert_transform(a)), np.eye(4), atol=1e-12)
    np.testing.assert_allclose(t.transform_point(t.invert_transform(a), t.transform_point(a, [1, 2, 3])),
                               [1, 2, 3], atol=1e-12)


def test_parent_chain_matches_hand_calculated_point_and_reverse_direction():
    frames = (
        frame_graph.Frame('cabinet', 'base_link', t.Pose((1, 0, 0), tuple(QZ90))),
        frame_graph.Frame('panel', 'cabinet', t.Pose((2, 0, 0), (0, 0, 0, 1))),
        frame_graph.Frame('port', 'panel', t.Pose((0, 3, 0), (0, 0, 0, 1))),
    )
    graph = frame_graph.FrameGraph(frames[::-1])  # Input order cannot be mistaken for parent order.
    # cabinet-origin + Rz90*(2,3,0) = (1,0,0)+(-3,2,0)=(-2,2,0).
    transform = frame_graph.resolve_transform(graph, 'base_link', 'port')
    np.testing.assert_allclose(transform[:3, 3], [-2, 2, 0], atol=1e-12)
    np.testing.assert_allclose(t.transform_point(transform, [1, 0, 0]), [-2, 3, 0], atol=1e-12)
    np.testing.assert_allclose(t.transform_point(graph.resolve('port', 'base_link'), [-2, 3, 0]),
                               [1, 0, 0], atol=1e-12)
    assert graph.is_descendant('port', 'cabinet') and not graph.is_descendant('port', 'port')
    with pytest.raises(t.TransformError, match='unknown frame'):
        graph.resolve('missing', 'missing')


@pytest.mark.parametrize('quaternion', [QZ90, [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, -0.]])
def test_quaternion_sign_canonicalization_and_matrix_roundtrip(quaternion):
    q = np.array(quaternion)
    first, second = t.canonical_quaternion(q), t.canonical_quaternion(-q)
    np.testing.assert_array_equal(first, second)
    np.testing.assert_allclose(t.quaternion_to_matrix(first), t.quaternion_to_matrix(second), atol=1e-12)
    np.testing.assert_allclose(t.matrix_to_quaternion(t.quaternion_to_matrix(first)), first, atol=1e-12)
    assert not np.signbit(first[first == 0]).any()


@pytest.mark.parametrize('quaternion', [[0, 0, 0, 0], [0, 0, 0, 2], [0, 0, 0], [0, 0, 0, np.inf],
                                        [0, 0, 0, np.nan], [0, 0, False, 1], [0, 0, 0, '1']])
def test_invalid_quaternions_rejected(quaternion):
    with pytest.raises(t.TransformError, match='quaternion_xyzw'):
        t.quaternion_to_matrix(quaternion)


@pytest.mark.parametrize('damage', ['shape', 'reflection', 'scaled', 'shear', 'last_row', 'nan', 'mixed_bool'])
def test_invalid_transform_not_repaired(damage):
    matrix = np.eye(4)
    if damage == 'shape': matrix = matrix[:3]
    elif damage == 'reflection': matrix[0, 0] = -1
    elif damage == 'scaled': matrix[0, 0] = 2
    elif damage == 'shear': matrix[0, 1] = .1
    elif damage == 'last_row': matrix[3, 0] = 1e-12
    elif damage == 'nan': matrix[0, 3] = np.nan
    else:
        matrix = matrix.tolist()
        matrix[0][0] = True
    with pytest.raises(t.TransformError):
        t.invert_transform(matrix)


@pytest.mark.parametrize('point', [[1, 2], [1, 2, np.inf], [True, 0, 0], ['1', 0, 0], [1j, 0, 0]])
def test_invalid_vectors_rejected(point):
    for function in (t.transform_point, t.transform_direction):
        with pytest.raises(t.TransformError):
            function(np.eye(4), point)


def test_inputs_and_immutable_records_are_not_mutated():
    position, quaternion = np.array([1., 2, 3]), np.array(QZ90)
    p0, q0 = position.copy(), quaternion.copy()
    pose = t.Pose(position, quaternion)
    position[:] = 9
    quaternion[:] = 0
    np.testing.assert_array_equal(pose.position_m, p0)
    np.testing.assert_allclose(pose.quaternion_xyzw, q0, atol=1e-12)
    matrix = pose.matrix()
    before = matrix.copy()
    t.invert_transform(matrix)
    t.transform_point(matrix, p0)
    t.compose_transforms(matrix, np.eye(4))
    np.testing.assert_array_equal(before, matrix)
    graph = frame_graph.FrameGraph([frame_graph.Frame('cabinet', 'base_link', pose)])
    exposed = graph.resolve('base_link', 'cabinet')
    exposed[:] = 0
    np.testing.assert_allclose(graph.resolve('base_link', 'cabinet'), before, atol=1e-12)
    with pytest.raises(FrozenInstanceError): pose.position_m = (0, 0, 0)
    with pytest.raises(TypeError): graph._to_root['base_link'] = ()


@pytest.mark.parametrize('case', ['duplicate', 'missing', 'self_cycle', 'cycle', 'root_redefined', 'wrong_root'])
def test_invalid_frame_graphs_rejected(case):
    pose = t.Pose((0, 0, 0), (0, 0, 0, 1))
    frames = [frame_graph.Frame('a', 'base_link', pose)]
    if case == 'duplicate': frames += [frame_graph.Frame('a', 'base_link', pose)]
    elif case == 'missing': frames = [frame_graph.Frame('a', 'unknown', pose)]
    elif case == 'self_cycle': frames = [frame_graph.Frame('a', 'a', pose)]
    elif case == 'cycle': frames = [frame_graph.Frame('a', 'b', pose), frame_graph.Frame('b', 'a', pose)]
    elif case == 'root_redefined': frames = [frame_graph.Frame('base_link', 'a', pose)]
    with pytest.raises(t.TransformError):
        frame_graph.FrameGraph(frames, root='other' if case == 'wrong_root' else 'base_link')


def test_scene_transform_import_is_independent_of_ros_environment():
    source = str(Path(__file__).resolve().parents[1] / 'src')
    code = f"import sys; sys.path.insert(0,{source!r}); import workspace_analysis.geometry.se3; import workspace_analysis.scene.frames; " \
           "assert not any(n in sys.modules for n in ['workspace_analysis.robot.model','rclpy','PyKDL','urdf_parser_py'])"
    result = subprocess.run([sys.executable, '-I', '-B', '-c', code], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stderr


def test_lazy_g1_exports_preserve_identity():
    import workspace_analysis
    from workspace_analysis.robot import model as model
    assert workspace_analysis.__all__ == ['KinematicModel', 'ModelError', 'load_model', 'model_from_xml']
    for name in workspace_analysis.__all__:
        assert getattr(workspace_analysis, name) is getattr(model, name)
