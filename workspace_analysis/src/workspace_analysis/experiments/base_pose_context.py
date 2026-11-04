"""Hash-bound World/Base provenance shared by experiment verification and replay."""
from hashlib import sha256
import json
import numpy as np
from ..scene.base_pose import BasePose, derive_scene, verify_derivation
from ..scene.schema import parse_scene
from ..robot.root_frame import root_mount, world_root_transform
from ..task.spec import TaskError, canonical


def make_context(world_source_text: str, pose: BasePose, model) -> dict:
    world=parse_scene(world_source_text);derived=derive_scene(world,pose)
    root,mount=root_mount(model);_,w_root=world_root_transform(model,pose.matrix())
    return {'schema_version':1,'usage':'RESEARCH_ONLY','pose_reference':'base_link_at_B0',
        'world_frame':'world','base_frame':model.base_frame,'base_pose':pose.to_dict(),'base_pose_sha256':pose.sha256,
        'world_from_base':pose.matrix().tolist(),'urdf_root':root,'root_from_base':mount.tolist(),
        'world_from_urdf_root':w_root.tolist(),'urdf_sha256':model.metadata()['expanded_urdf_sha256'],
        'world_source_text':world_source_text,'world_raw_sha256':sha256(world_source_text.encode()).hexdigest(),
        'world_scene_id':world.scene_id,'world_scene_sha256':world.canonical_scene_sha256,
        'derived_scene_id':derived.scene_id,'derived_scene_sha256':derived.canonical_scene_sha256,
        'geometry_verification':verify_derivation(world,derived,pose),
        'height_search_performed':False,'base_motion_planned':False,'physical_station_safety_verified':False}


def verify_context(data: dict, derived, model) -> dict:
    if not isinstance(data,dict) or data.get('schema_version')!=1:raise TaskError('invalid base pose context')
    pose=BasePose.from_dict(data['base_pose'])
    expected=make_context(data['world_source_text'],pose,model)
    if canonical(expected)!=canonical(data):raise TaskError('base pose context identity/geometry/model differs')
    if derived.canonical_scene_sha256!=data['derived_scene_sha256']:raise TaskError('base pose/scene binding differs')
    return {'status':'PASS','base_pose':pose.to_dict(),'world_scene_sha256':data['world_scene_sha256'],
            'derived_scene_sha256':derived.canonical_scene_sha256,'world_from_base':pose.matrix().tolist(),
            'urdf_root':data['urdf_root'],'world_from_urdf_root':data['world_from_urdf_root'],
            'world_cabinet_fixed':True,'physical_station_safety_verified':False}
