"""URDF fixed root-to-base mount only; no moving-joint FK or model duplication."""
import numpy as np
from scipy.spatial.transform import Rotation
from ..geometry.se3 import pose_to_matrix, invert_transform, validate_transform
from .model import KinematicModel, ModelError


def root_mount(model: KinematicModel) -> tuple[str,np.ndarray]:
    """Return actual URDF root and ^root T_base; reject a moving base mount."""
    parents={j.child:j for j in model.joints.values()};link=model.base_frame;path=[];seen=set()
    while link in parents:
        if link in seen:raise ModelError('cyclic root mount')
        seen.add(link);joint=parents[link]
        if joint.kind!='fixed':raise ModelError('World replay requires a fixed root-to-base URDF chain')
        path.append(joint);link=joint.parent
    t=np.eye(4)
    for joint in reversed(path):t=t@pose_to_matrix(joint.xyz,Rotation.from_euler('xyz',joint.rpy).as_quat())
    return link,validate_transform(t)


def world_root_transform(model: KinematicModel, world_from_base) -> tuple[str,np.ndarray]:
    root,mount=root_mount(model)
    return root,validate_transform(validate_transform(world_from_base)@invert_transform(mount))


def fixed_body_links(model: KinematicModel) -> frozenset[str]:
    """Rigid component attached to the URDF root; membership is not physical coverage."""
    root,_=root_mount(model);links={root}
    while True:
        before=len(links)
        for j in model.joints.values():
            if j.kind=='fixed' and j.parent in links:links.add(j.child)
        if len(links)==before:return frozenset(links)
