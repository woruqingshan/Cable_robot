"""URDF -> KDL adapter and position FK using the installed PyKDL solver.

The adapter follows ros/kdl_parser's parent-to-joint origin convention:
joint origin p and axis R_origin * axis_local are in the parent frame;
the segment's zero-position frame is the complete URDF origin.
No JetRover geometry is embedded here.
"""

from dataclasses import dataclass
import math

import numpy as np
import PyKDL as kdl

from .model import KinematicModel, ModelError


@dataclass(frozen=True)
class FKResult:
    position: np.ndarray
    rotation: np.ndarray
    transform: np.ndarray


class ForwardKinematics:
    """Reusable solver; use separate instances for concurrent computations."""

    def __init__(self, model: KinematicModel):
        self.model = model
        self._chain = kdl.Chain()
        moving = []
        for spec in model.chain:
            origin = kdl.Frame(kdl.Rotation.RPY(*spec.rpy), kdl.Vector(*spec.xyz))
            if spec.kind == "fixed":
                # 'None' is exposed as 'Fixed' in PyKDL's Python bindings.
                joint = kdl.Joint(spec.name, kdl.Joint.Fixed)
            elif spec.kind == "revolute":
                length = math.hypot(*spec.axis)
                axis_local = kdl.Vector(*(v / length for v in spec.axis))
                joint = kdl.Joint(spec.name, origin.p, origin.M * axis_local,
                                  kdl.Joint.RotAxis)
                moving.append(spec.name)
            else:
                raise ModelError(f"{spec.name}: unsupported KDL conversion type {spec.kind!r}")
            self._chain.addSegment(kdl.Segment(spec.child, joint, origin))
        if tuple(moving) != model.joint_order or self._chain.getNrOfJoints() != len(moving):
            raise ModelError("KDL chain joint order/count differs from parsed URDF")
        if self._chain.getNrOfSegments() != len(model.chain):
            raise ModelError("KDL chain lost a fixed or movable segment")
        self._solver = kdl.ChainFkSolverPos_recursive(self._chain)

    def pose(self, q, *, auxiliary_positions=None):
        """Return XYZ (m), R (3x3), and T_base_to_ee (4x4).

        q is a finite 1-D real vector (rad) in model.joint_order, within URDF
        bounds. Optional auxiliary states only validate off-chain/mimic inputs.
        They never add degrees of freedom to this FK chain.
        """
        positions = self.model.joint_positions(q, auxiliary_positions)
        values = kdl.JntArray(len(self.model.joint_order))
        for index, name in enumerate(self.model.joint_order):
            values[index] = positions[name]
        frame = kdl.Frame()
        code = self._solver.JntToCart(values, frame)
        if code < 0:
            raise RuntimeError(f"PyKDL JntToCart failed with code {code}")
        position = np.array([frame.p[i] for i in range(3)], dtype=float)
        rotation = np.array([[frame.M[i, j] for j in range(3)] for i in range(3)], dtype=float)
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = position
        if not np.isfinite(transform).all():
            raise RuntimeError("PyKDL returned a non-finite transform")
        return FKResult(position, rotation, transform)

    def __call__(self, q, *, auxiliary_positions=None):
        """FK(q) -> T_base_to_ee, a new 4x4 NumPy array."""
        return self.pose(q, auxiliary_positions=auxiliary_positions).transform
