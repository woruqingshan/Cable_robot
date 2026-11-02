"""G1-A tests. Nothing executes until the user invokes pytest."""

from pathlib import Path
import os

import numpy as np
import pytest

from workspace_analysis.model import JOINT_ORDER, ModelError, load_model, model_from_xml
from workspace_analysis.fk import ForwardKinematics
from reference_fk import pose_errors, reference_fk


ROOT = Path(__file__).resolve().parents[1]
CASES = [
    pytest.param([0, 0, 0, 0, 0], id="Zero"),
    pytest.param([0.35, -0.60, 0.80, -0.40, 0.20], id="Nonzero"),
    pytest.param([2.08, -0.40, 0.50, -0.30, 0.25], id="Near-limit"),
]

# Deliberately unlike JetRover: translated/rotated fixed segments before,
# between and after rotations; a non-unit, non-cardinal joint axis.
MINIMAL = """<robot name="adapter_fixture">
<link name="base"/><link name="mount"/><link name="first"/>
<link name="spacer"/><link name="second"/><link name="tip"/>
<joint name="mount_fixed" type="fixed"><parent link="base"/><child link="mount"/>
 <origin xyz="0.11 -0.07 0.20" rpy="0.20 -0.30 0.40"/></joint>
<joint name="j1" type="revolute"><parent link="mount"/><child link="first"/>
 <origin xyz="0.03 0.04 -0.02" rpy="-0.40 0.25 0.15"/>
 <axis xyz="1 2 -3"/><limit lower="-1.5" upper="1.5" effort="1" velocity="1"/></joint>
<joint name="spacer_fixed" type="fixed"><parent link="first"/><child link="spacer"/>
 <origin xyz="0.06 -0.02 0.09" rpy="0.35 0.10 -0.20"/></joint>
<joint name="j2" type="revolute"><parent link="spacer"/><child link="second"/>
 <origin xyz="-0.02 0.08 0.05" rpy="0.10 -0.45 0.30"/>
 <axis xyz="-2 1 0.5"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>
<joint name="tip_fixed" type="fixed"><parent link="second"/><child link="tip"/>
 <origin xyz="0.04 0.01 0.12" rpy="-0.15 0.20 0.05"/></joint>
</robot>"""


@pytest.fixture(scope="module")
def robot():
    # Missing dependencies/overlay/model fail the test; never silently skip.
    return load_model(ROOT / "config/workspace.yaml")


def assert_valid_transform(transform):
    assert transform.shape == (4, 4)
    assert np.isfinite(transform).all()
    np.testing.assert_allclose(transform[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
    rotation = transform[:3, :3]
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12, rtol=0)
    assert abs(np.linalg.det(rotation) - 1) <= 1e-12


def assert_reference(model, q):
    actual = ForwardKinematics(model)(q)
    expected = reference_fk(model.urdf_xml, q, base_frame=model.base_frame,
                            ee_frame=model.ee_frame, joint_order=model.joint_order)
    assert_valid_transform(actual)
    position, angle = pose_errors(actual, expected)
    assert position <= 1e-5, f"position error {position} m"
    assert angle <= 1e-4, f"rotation error {angle} rad"
    return actual


def test_official_chain_and_dynamic_limits(robot):
    assert robot.joint_order == JOINT_ORDER
    assert [j.name for j in robot.chain] == [
        "joint1", "servo_joint1", "joint2", "joint3", "joint4",
        "servo_joint2", "joint5", "end_effector_joint"]
    # Modify the XML's joint1 limits and verify they are re-read, not literals.
    modified = robot.urdf_xml.replace('lower="-2.09"', 'lower="-1.95"')
    assert modified != robot.urdf_xml
    changed = model_from_xml(modified)
    assert changed.limits[0, 0] == -1.95
    assert changed.limits[0, 0] != robot.limits[0, 0]


@pytest.mark.parametrize("q", CASES)
def test_official_three_poses(robot, q):
    actual = assert_reference(robot, q)
    result = ForwardKinematics(robot).pose(q)
    np.testing.assert_allclose(result.position, actual[:3, 3], atol=1e-12, rtol=0)
    np.testing.assert_allclose(result.rotation, actual[:3, :3], atol=1e-12, rtol=0)


def test_deterministic(robot):
    fk = ForwardKinematics(robot)
    q = [0.35, -0.6, 0.8, -0.4, 0.2]
    np.testing.assert_array_equal(fk(q), fk(q))


@pytest.mark.parametrize("index", range(5))
@pytest.mark.parametrize("bound", ["lower", "upper"])
def test_limits_and_out_of_range(robot, index, bound):
    q = np.zeros(5)
    value = robot.limits[index, 0 if bound == "lower" else 1]
    q[index] = value
    assert_valid_transform(ForwardKinematics(robot)(q))
    q[index] = value + (-1e-6 if bound == "lower" else 1e-6)
    with pytest.raises(ModelError, match=robot.joint_order[index]):
        ForwardKinematics(robot)(q)


@pytest.mark.parametrize("q", [[], [0] * 4, [[0] * 5], [0, 0, np.nan, 0, 0],
                               [0, np.inf, 0, 0, 0], [0j] * 5, [True] * 5,
                               ["0"] * 5])
def test_bad_joint_vectors(robot, q):
    with pytest.raises(ModelError):
        ForwardKinematics(robot)(q)


def test_joint5_position_invariance(robot):
    fk = ForwardKinematics(robot)
    q = [0.35, -0.6, 0.8, -0.4, 0.2]
    first = fk(q)
    q[4] = -0.9
    second = fk(q)
    np.testing.assert_allclose(first[:3, 3], second[:3, 3], atol=1e-12, rtol=0)
    assert pose_errors(first, second)[1] > 0.1


def test_gripper_and_mimic_invariance(robot):
    fk = ForwardKinematics(robot)
    q = [0.35, -0.6, 0.8, -0.4, 0.2]
    neutral = fk(q, auxiliary_positions={"r_joint": 0.0})
    for value in [-0.6, 0.6]:
        states = robot.joint_positions(q, {"r_joint": value})
        for name in ["l_joint", "l_in_joint", "r_in_joint"]:
            assert states[name] == -value
        for name in ["l_out_joint", "r_out_joint"]:
            assert states[name] == value
        np.testing.assert_allclose(fk(q, auxiliary_positions={"r_joint": value}),
                                   neutral, atol=1e-12, rtol=0)
    for state in [{"r_joint": 2.0}, {"l_joint": 0.0}, {"joint1": 0.0}, {"typo": 0.0}]:
        with pytest.raises(ModelError):
            fk(q, auxiliary_positions=state)


def test_fixed_segments_and_zero_pose(robot):
    # Expected zero geometry read from URDF, no embedded JetRover link lengths.
    q = np.zeros(5)
    expected = np.sum([j.xyz for j in robot.chain], axis=0)
    np.testing.assert_allclose(ForwardKinematics(robot).pose(q).position,
                               expected, atol=1e-12, rtol=0)
    footprint = model_from_xml(robot.urdf_xml, base_frame="base_footprint")
    base_transform = reference_fk(robot.urdf_xml, [], base_frame="base_footprint",
                                  ee_frame="base_link", joint_order=[])
    q = [0.35, -0.6, 0.8, -0.4, 0.2]
    np.testing.assert_allclose(ForwardKinematics(footprint)(q),
                               base_transform @ ForwardKinematics(robot)(q), atol=1e-12, rtol=0)


@pytest.mark.parametrize("q", [[0, 0], [0.65, -0.45], [-1.4, 0.9]])
def test_rotated_origins_and_oblique_axes(q):
    model = model_from_xml(MINIMAL, base_frame="base", ee_frame="tip", joint_order=["j1", "j2"])
    assert len(model.chain) == 5
    assert_reference(model, q)
    # Remove each fixture fixed transform separately. Every omission must be
    # observable; agreement must not come from two solvers ignoring fixed links.
    import xml.etree.ElementTree as ET
    baseline = ForwardKinematics(model)(q)
    for name in ["mount_fixed", "spacer_fixed", "tip_fixed"]:
        root = ET.fromstring(MINIMAL)
        for joint in root.findall("joint"):
            if joint.get("name") == name:
                joint.remove(joint.find("origin"))
        changed = model_from_xml(ET.tostring(root, encoding="unicode"), base_frame="base",
                                 ee_frame="tip", joint_order=["j1", "j2"])
        assert np.linalg.norm(ForwardKinematics(changed)(q) - baseline) > 1e-3


@pytest.mark.parametrize("xml", [
    "<robot>",
    MINIMAL.replace('name="tip"', 'name="base"'),
    MINIMAL.replace('link="second"', 'link="missing"'),
    MINIMAL.replace('xyz="1 2 -3"', 'xyz="0 0 0"'),
    MINIMAL.replace('rpy="-0.40 0.25 0.15"', 'rpy="nan 0 0"'),
    MINIMAL.replace('lower="-1.5"', 'lower="2"'),
    MINIMAL.replace('upper="1.5"', 'upper="inf"'),
    MINIMAL.replace('type="revolute"', 'type="floating"'),
    MINIMAL.replace('<axis xyz="1 2 -3"/>', '<mimic joint="j2"/><axis xyz="1 2 -3"/>'),
    MINIMAL.replace('<axis xyz="1 2 -3"/>', '<mimic joint="missing"/><axis xyz="1 2 -3"/>'),
])
def test_invalid_models(xml):
    with pytest.raises(ModelError):
        model_from_xml(xml, base_frame="base", ee_frame="tip", joint_order=["j1", "j2"])


def test_missing_frames_order_and_mimic_cycle():
    for kwargs in [{"base_frame": "missing"}, {"ee_frame": "missing"},
                   {"joint_order": ["j2", "j1"]}, {"base_frame": "tip", "ee_frame": "base"}]:
        settings = dict(base_frame="base", ee_frame="tip", joint_order=["j1", "j2"])
        settings.update(kwargs)
        with pytest.raises(ModelError):
            model_from_xml(MINIMAL, **settings)
    xml = MINIMAL.replace('<axis xyz="1 2 -3"/>', '<mimic joint="j2"/><axis xyz="1 2 -3"/>')
    xml = xml.replace('<axis xyz="-2 1 0.5"/>', '<mimic joint="j1"/><axis xyz="-2 1 0.5"/>')
    with pytest.raises(ModelError, match="cyclic mimic"):
        model_from_xml(xml, base_frame="base", ee_frame="tip", joint_order=["j1", "j2"])


def test_loader_from_another_cwd_without_need_compile(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("need_compile", raising=False)
    monkeypatch.setenv("MACHINE_TYPE", "preserve-me")
    monkeypatch.setenv("LIDAR_TYPE", "preserve-me-too")
    model = load_model(ROOT / "config/workspace.yaml")
    assert model.joint_order == JOINT_ORDER
    assert "need_compile" not in os.environ
    assert os.environ["MACHINE_TYPE"] == "preserve-me"
    assert os.environ["LIDAR_TYPE"] == "preserve-me-too"
    assert model.source_hashes
    assert model.xacro_environment["MACHINE_TYPE"] == "JetRover_Mecanum"


def test_plain_urdf_and_relative_config_path(tmp_path):
    (tmp_path / "fixture.urdf").write_text(MINIMAL, encoding="utf-8")
    config = tmp_path / "model.yaml"
    config.write_text("model:\n  path: fixture.urdf\n  base_frame: base\n"
                      "  ee_frame: tip\n  joint_order: [j1, j2]\n", encoding="utf-8")
    model = load_model(config)
    assert model.xacro_environment == {}
    assert_reference(model, [0.65, -0.45])
    override = load_model(config, model_path=tmp_path / "fixture.urdf")
    assert override.urdf_xml == model.urdf_xml
    with pytest.raises(ModelError, match="Cannot load/expand"):
        load_model(config, model_path=tmp_path / "missing.urdf")


def test_urdf_default_axis_and_origin():
    xml = MINIMAL.replace('<origin xyz="0.03 0.04 -0.02" rpy="-0.40 0.25 0.15"/>', '')
    xml = xml.replace('<axis xyz="1 2 -3"/>', '')
    model = model_from_xml(xml, base_frame="base", ee_frame="tip", joint_order=["j1", "j2"])
    assert model.joints["j1"].axis == (1.0, 0.0, 0.0)
    assert_reference(model, [0.65, -0.45])


def test_xacro_failure_restores_environment(tmp_path, monkeypatch):
    broken = tmp_path / "broken.xacro"
    broken.write_text('<robot name="broken" xmlns:xacro="http://www.ros.org/wiki/xacro">'
                      '<xacro:include filename="missing_include.xacro"/></robot>', encoding="utf-8")
    config = tmp_path / "broken.yaml"
    config.write_text("model:\n  path: broken.xacro\nxacro:\n"
                      "  MACHINE_TYPE: JetRover_Mecanum\n  LIDAR_TYPE: A1\n", encoding="utf-8")
    monkeypatch.setenv("MACHINE_TYPE", "preserve-on-failure")
    monkeypatch.delenv("LIDAR_TYPE", raising=False)
    with pytest.raises(ModelError, match="Cannot load/expand"):
        load_model(config)
    assert os.environ["MACHINE_TYPE"] == "preserve-on-failure"
    assert "LIDAR_TYPE" not in os.environ


@pytest.mark.parametrize("config_text", [
    "model: null\n",
    "model: {}\nunknown_key: true\n",
    "model: {}\nxacro:\n  need_compile: True\n",
    "model: {}\nxacro:\n  MACHINE_TYPE: 123\n",
])
def test_invalid_config(tmp_path, config_text):
    config = tmp_path / "invalid.yaml"
    config.write_text(config_text, encoding="utf-8")
    with pytest.raises(ModelError):
        load_model(config)
