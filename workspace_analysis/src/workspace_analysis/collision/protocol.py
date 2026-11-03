"""Versioned wire schema and verified SceneSnapshot-to-native adapter."""
from hashlib import sha256
import json
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np

from ..scene.model import SceneSnapshot
from ..geometry.se3 import validate_transform

VERSION = 1
STATES = {'CLEAR', 'COLLIDING', 'UNVERIFIED', 'ERROR'}


class CollisionError(RuntimeError):
    """Runtime, identity or protocol failure; never interpreted as CLEAR."""


def strict_json(text):
    """Reject duplicate keys and non-JSON NaN/Infinity constants."""
    def pairs(items):
        result = {}
        for key,value in items:
            if key in result:
                raise CollisionError('duplicate JSON key: '+key)
            result[key]=value
        return result
    def reject(value):
        raise CollisionError('invalid JSON constant: '+value)
    try:
        value=json.loads(text,object_pairs_hook=pairs,parse_constant=reject)
        if not isinstance(value,dict):raise CollisionError('JSON response must be object')
        return value
    except (ValueError,TypeError) as exc:
        raise CollisionError('malformed JSON response') from exc


def make_init(model, scene):
    """Use G1 expanded XML and B1 full object transforms; no new FK or schema."""
    if not isinstance(scene,SceneSnapshot):raise CollisionError('SceneSnapshot required')
    if model.base_frame!=scene.reference_frame:raise CollisionError('base frame mismatch')
    if any(scene.clearance[k]!=0 for k in ['robot_environment_m','self_m']):
        raise CollisionError('C1 does not implement positive clearance; refusing requested margin')
    objects=[]
    for obj in scene.collision_objects:
        item={'id':obj['id'],'type':obj['type'],
              'transform':scene.object_transform(obj['id']).tolist()}
        if obj['type']=='box':item['size_m']=list(obj['size_m'])
        else:item.update(radius_m=obj['radius_m'],length_m=obj['length_m'])
        objects.append(item)
    return {'model':{'urdf_xml':model.urdf_xml,'urdf_sha256':model.metadata()['expanded_urdf_sha256'],
                     'base_frame':model.base_frame,'ee_frame':model.ee_frame,'joint_order':list(model.joint_order)},
            'scene_canonical_json':scene.to_json(),'scene_sha256':scene.canonical_scene_sha256,'objects':objects}


def mesh_resource_path(uri):
    """Resolve only local resources; never fetch remote robot geometry."""
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package,relative=uri[len('package://'):].split('/',1)
        return Path(get_package_share_directory(package))/relative
    parsed=urlparse(uri)
    if parsed.scheme=='file' and parsed.netloc in ('','localhost'):return Path(unquote(parsed.path))
    if not parsed.scheme and Path(uri).is_absolute():return Path(uri)
    raise CollisionError('only absolute file or package mesh URIs supported')


def check_ready(reply, init, executable):
    """Verify identity, all URDF joints and independently hashed mesh bytes."""
    identity=reply.get('identity',{})
    expected={'urdf_sha256':init['model']['urdf_sha256'],'scene_sha256':init['scene_sha256'],
              'backend_sha256':sha256(Path(executable).read_bytes()).hexdigest(),
              'backend_version':'cable_collision_backend/0.1.0',
              'base_frame':init['model']['base_frame'],'ee_frame':init['model']['ee_frame'],
              'joint_order':init['model']['joint_order'],'units':{'length':'m','angle':'rad'}}
    if any(identity.get(k)!=v for k,v in expected.items()):raise CollisionError('backend/model/scene identity mismatch')
    for key in ['robot_geometry_sha256','policy_sha256','srdf_sha256']:
        if not isinstance(identity.get(key),str) or len(identity[key])!=64:raise CollisionError('missing identity: '+key)
    audit=reply.get('audit',{})
    xml=ET.fromstring(init['model']['urdf_xml'])
    if audit.get('urdf_link_count')!=len(xml.findall('link')) or audit.get('urdf_joint_count')!=len(xml.findall('joint')):
        raise CollisionError('robot tree counts mismatch')
    actual={j['name']:j for j in audit['joints']}
    for j in xml.findall('joint'):
        a=actual[j.get('name')]
        if (a['parent'],a['child'])!=(j.find('parent').get('link'),j.find('child').get('link')):
            raise CollisionError('joint tree mismatch')
        if j.get('type')=='revolute':
            limit=j.find('limit')
            if a['limits']!=[float(limit.get('lower')),float(limit.get('upper'))]:raise CollisionError('joint limits mismatch')
        mimic=j.find('mimic')
        if mimic is not None:
            if a['mimic']!={'joint':mimic.get('joint'),'multiplier':float(mimic.get('multiplier','1')),
                            'offset':float(mimic.get('offset','0'))}:raise CollisionError('mimic audit mismatch')
    expected_meshes=[]
    for link in xml.findall('link'):
        for collision in link.findall('collision'):
            mesh=collision.find('geometry/mesh')
            if mesh is not None:
                uri=mesh.get('filename');expected_meshes.append((link.get('name'),uri,
                    [float(x) for x in mesh.get('scale','1 1 1').split()],
                    sha256(mesh_resource_path(uri).read_bytes()).hexdigest()))
    meshes=[(g['link'],g['uri'],g['scale'],g['resource_sha256']) for g in audit['geometry'] if g['type']=='mesh']
    if sorted(meshes)!=sorted(expected_meshes):raise CollisionError('robot collision resource mismatch')
    if audit['moveit_shape_count']!=sum(len(l.findall('collision')) for l in xml.findall('link')):
        raise CollisionError('lost collision geometry')
    if audit['detector']!='FCL' or audit['acm_size']!=0:raise CollisionError('unexpected collision policy/backend')
    if audit['objects_base']!=init['objects']:raise CollisionError('scene object audit mismatch')
    if 'geometry_profile_canonical_json' in init:
        text=init['geometry_profile_canonical_json'];profile=strict_json(text)
        if sha256(text.encode()).hexdigest()!=init['geometry_profile_sha256'] or reply.get('geometry_profile')!=profile:raise CollisionError('geometry profile acknowledgement mismatch')
        if identity.get('source_urdf_sha256')!=init['model']['urdf_sha256'] or identity.get('geometry_profile_sha256')!=init['geometry_profile_sha256'] or identity.get('geometry_profile')!=profile['id'] or identity.get('geometry_review_status')!=profile['review_status']:raise CollisionError('geometry profile identity mismatch')
        if 'effective_model' in init:
            child=reply.get('effective_ready',{})
            nested={k:v for k,v in init.items() if k not in ['geometry_profile_canonical_json','geometry_profile_sha256','effective_model']}
            nested['model']={**init['model'],**init['effective_model']}
            child_identity=check_ready(child,nested,executable)
            if identity.get('effective_urdf_sha256')!=init['effective_model']['urdf_sha256'] or identity.get('effective_geometry_sha256')!=child_identity['robot_geometry_sha256']:raise CollisionError('effective model identity mismatch')
            expected_geometry=sha256((identity['source_geometry_sha256']+':'+identity['effective_geometry_sha256']+':'+init['geometry_profile_sha256']).encode()).hexdigest()
            if identity['robot_geometry_sha256']!=expected_geometry:raise CollisionError('composite geometry identity mismatch')
        elif identity.get('source_geometry_sha256')!=identity['robot_geometry_sha256'] or identity.get('effective_geometry_sha256')!=identity['robot_geometry_sha256']:raise CollisionError('official geometry identity mismatch')
    loaded={o['id']:o for o in audit['loaded_objects_base']}
    if set(loaded)!={o['id'] for o in init['objects']}:raise CollisionError('loaded object IDs mismatch')
    for o in init['objects']:
        actual=loaded[o['id']]
        if not np.allclose(actual['transform'],o['transform'],atol=1e-10,rtol=0):raise CollisionError('loaded object pose mismatch')
        for key in ['type','size_m','radius_m','length_m']:
            if key in o and actual.get(key)!=o[key]:raise CollisionError('loaded object dimensions mismatch')
    return identity


def check_response(reply, request_id, identity=None):
    """Validate envelope and lock model/scene/backend identity for every response."""
    if type(reply.get('protocol_version')) is not int or reply['protocol_version']!=VERSION:
        raise CollisionError('protocol version mismatch')
    if reply.get('id')!=request_id:raise CollisionError('request/response ID mismatch')
    if identity is not None and reply.get('identity')!=identity:raise CollisionError('response identity changed')
    if reply.get('status')!='OK':raise CollisionError('backend error: '+str(reply.get('error','unknown')))
    return reply


def check_query(reply):
    """Reject inconsistent or optimistic collision classifications."""
    for key in ['collision_detected','self_collision','environment_collision','pairs_may_be_truncated']:
        if type(reply.get(key)) is not bool:raise CollisionError('invalid collision boolean')
    collision=reply['self_collision'] or reply['environment_collision']
    if reply['collision_detected']!=collision:raise CollisionError('inconsistent collision union')
    if reply.get('backend_status')!='OK' or reply.get('decision')!=('COLLIDING' if collision else 'UNVERIFIED'):
        raise CollisionError('invalid C1 decision; no CLEAR allowed for unreviewed coverage')
    validate_transform(reply['T_base_to_ee'])
    if not isinstance(reply.get('colliding_pairs'),list):raise CollisionError('invalid contacts')
    for pair in reply['colliding_pairs']:
        if (not isinstance(pair,dict) or pair.get('kind') not in {'self','environment'}
                or any(not isinstance(pair.get(k),str) or not pair[k] for k in ['first','second'])):
            raise CollisionError('invalid collision pair')
        if not reply['self_collision' if pair['kind']=='self' else 'environment_collision']:
            raise CollisionError('contact classification disagrees with collision boolean')
    if isinstance(reply.get('query_elapsed_s'),bool) or not isinstance(reply.get('query_elapsed_s'),(int,float)) or not np.isfinite(reply['query_elapsed_s']) or reply['query_elapsed_s']<0:
        raise CollisionError('invalid timing')
    return reply
