"""Test-only FK: ElementTree + NumPy/SciPy, with no production/KDL imports.

URDF specifies T_parent_child(q) = T_origin * T_motion_local(q).
Compose that equation directly, rather than reproducing KDL's parent-frame
joint-pivot representation. This checks software agreement, not hardware.
"""

import numpy as np
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET


def reference_fk(xml, q, *, base_frame, ee_frame, joint_order):
    root = ET.fromstring(xml)
    by_child = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    angles = dict(zip(joint_order, q))
    path, link = [], ee_frame
    while link != base_frame:
        joint = by_child[link]
        path.append(joint)
        link = joint.find("parent").get("link")
    transform = np.eye(4)
    for joint in reversed(path):
        origin_node = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin_node is not None:
            xyz = [float(v) for v in origin_node.get("xyz", "0 0 0").split()]
            rpy = [float(v) for v in origin_node.get("rpy", "0 0 0").split()]
        origin = np.eye(4)
        # Lowercase xyz means fixed/extrinsic axes: Rz(yaw) Ry(pitch) Rx(roll).
        origin[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        origin[:3, 3] = xyz
        motion = np.eye(4)
        if joint.get("type") == "revolute":
            axis_node = joint.find("axis")
            axis = np.array([float(v) for v in
                             (axis_node.get("xyz") if axis_node is not None else "1 0 0").split()])
            axis /= np.linalg.norm(axis)
            motion[:3, :3] = Rotation.from_rotvec(axis * angles[joint.get("name")]).as_matrix()
        elif joint.get("type") != "fixed":
            raise ValueError("Test reference supports only fixed/revolute joints")
        transform = transform @ origin @ motion
    return transform


def pose_errors(actual, expected):
    position = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    # Rotation.magnitude handles the small-angle case without raw acos rounding.
    angle = float(Rotation.from_matrix(expected[:3, :3].T @ actual[:3, :3]).magnitude())
    return position, angle
