"""Explicit, hash-checked pair-local diagnostic collision geometry. No FK.

Only whole triangles inside a declared joint-axis cylinder are excluded.
Boundary/outside triangles are retained unchanged; environment and non-target
self pairs always use the complete official geometry in the native compositor.
"""
from dataclasses import dataclass,replace
from hashlib import sha256
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from .assets import read_stl
from .protocol import CollisionError,mesh_resource_path


def canonical(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False)


@dataclass(frozen=True)
class GeometryProfile:
    canonical_json:str
    sha256:str
    effective_xml:str|None

    def wire(self):
        data={'geometry_profile_canonical_json':self.canonical_json,'geometry_profile_sha256':self.sha256}
        if self.effective_xml is not None:data['effective_model']={'urdf_xml':self.effective_xml,'urdf_sha256':sha256(self.effective_xml.encode()).hexdigest()}
        return data


def triangle_mask(vertices,faces,axis,radius,axial):
    """A convex cylinder contains a triangle iff all its vertices are inside.

    We remove only such triangles. No vertex moves, no new surface/cap is added.
    This is a surface diagnostic, not a validated closed solid approximation.
    """
    axis=np.asarray(axis,dtype=float);axis=axis/np.linalg.norm(axis)
    radius=float(radius);axial=np.asarray(axial,dtype=float)
    if not np.isfinite(radius) or radius<=0 or axial.shape!=(2,) or not np.isfinite(axial).all() or axial[0]>=axial[1]:raise CollisionError('invalid cylinder')
    along=np.asarray(vertices)@axis;radial=np.linalg.norm(vertices-np.outer(along,axis),axis=1)
    inside=(radial<radius)&(along>axial[0])&(along<axial[1])
    return ~inside[faces].all(axis=1)


def _stl_bytes(vertices,faces):
    result=bytearray(b'C27 diagnostic retained official triangles'.ljust(80,b'\0')+struct.pack('<I',len(faces)))
    for face in faces:
        points=vertices[face];normal=np.cross(points[1]-points[0],points[2]-points[0]);length=np.linalg.norm(normal)
        if length:normal/=length
        result.extend(struct.pack('<12fH',*normal,*points.ravel(),0))
    return bytes(result)


def load_geometry_profile(path,model,*,asset_dir=None):
    """Materialize derived local meshes into a NEW directory; official is read-only."""
    config=yaml.safe_load(Path(path).read_text())
    if set(config)!={'version','id','method','review_status','scope','edits','target_pairs'} or config['version']!=1:raise CollisionError('invalid geometry profile schema')
    if config['method'] not in ['official_full','pair_local_triangle_exclusion']:raise CollisionError('unknown geometry method')
    data={**config,'source_urdf_sha256':model.metadata()['expanded_urdf_sha256']}
    xml=ET.fromstring(model.urdf_xml)
    if config['method']=='official_full':
        if config['edits'] or config['target_pairs'] or config['review_status']!='official_unvalidated':raise CollisionError('official profile cannot edit geometry')
        text=canonical(data);return GeometryProfile(text,sha256(text.encode()).hexdigest(),None)
    if config['review_status']!='diagnostic_only' or not config['edits'] or not config['target_pairs']:raise CollisionError('refined profile must be explicitly diagnostic')
    if asset_dir is None:raise CollisionError('new asset_dir required for refinement')
    folder=Path(asset_dir).absolute();folder.mkdir(parents=True,exist_ok=False)
    data['edits']=[];edited=set()
    for edit in config['edits']:
        if set(edit)!={'link','joint','radius_m','axial_m','reason'}:raise CollisionError('invalid edit fields')
        name=edit['link'];joint=model.joints[edit['joint']]
        if name in edited or joint.child!=name or joint.kind!='revolute':raise CollisionError('edit must be unique child at its own revolute axis')
        edited.add(name);link=next(l for l in xml.findall('link') if l.get('name')==name)
        collisions=link.findall('collision')
        if len(collisions)!=1:raise CollisionError('one collision mesh per edited link required')
        collision=collisions[0];mesh=collision.find('geometry/mesh')
        if mesh is None:raise CollisionError('mesh edit required')
        vertices,faces,digest=read_stl(mesh_resource_path(mesh.get('filename')))
        scale=np.array([float(x) for x in mesh.get('scale','1 1 1').split()]);origin=collision.find('origin')
        xyz=np.zeros(3) if origin is None else np.array([float(x) for x in origin.get('xyz','0 0 0').split()])
        rpy=np.zeros(3) if origin is None else np.array([float(x) for x in origin.get('rpy','0 0 0').split()])
        points=(vertices*scale)@Rotation.from_euler('xyz',rpy).as_matrix().T+xyz
        keep=triangle_mask(points,faces,joint.axis,edit['radius_m'],edit['axial_m'])
        if not keep.any() or keep.all():raise CollisionError('refinement must retain and exclude triangles')
        payload=_stl_bytes(vertices,faces[keep]);target=folder/(name+'.stl')
        with target.open('xb') as f:f.write(payload)
        data['edits'].append({**edit,'axis_link':list(joint.axis),'source_uri':mesh.get('filename'),'source_sha256':digest,'effective_uri':target.as_uri(),'effective_sha256':sha256(payload).hexdigest(),'source_triangles':len(faces),'retained_triangles':int(keep.sum()),'excluded_triangle_indices':np.flatnonzero(~keep).tolist(),'outside_boundary_triangles_unchanged':True})
        mesh.set('filename',target.as_uri())
    for pair in config['target_pairs']:
        if len(pair)!=2 or pair[0]==pair[1] or not (set(pair)&edited):raise CollisionError('target pair must contain an edited link')
    text=canonical(data);return GeometryProfile(text,sha256(text.encode()).hexdigest(),ET.tostring(xml,encoding='unicode'))


def effective_render_data(model,ready,row):
    """Adapt validated native effective audit to the existing C2.5 renderer."""
    from .assets import robot_surfaces
    if 'effective_ready' not in ready:return robot_surfaces(model,ready,row)
    derived=replace(model,urdf_xml=ready['effective_model_xml'])
    native_ready=ready['effective_ready'];native_row={**row,'identity':native_ready['identity']}
    return robot_surfaces(derived,native_ready,native_row)


def materialized_profile(data,model):
    """Reload a recorded profile without rewriting assets or changing its identity."""
    if data['source_urdf_sha256']!=model.metadata()['expanded_urdf_sha256']:raise CollisionError('recorded profile source mismatch')
    xml=ET.fromstring(model.urdf_xml)
    for edit in data['edits']:
        link=next(l for l in xml.findall('link') if l.get('name')==edit['link']);mesh=link.find('collision/geometry/mesh')
        if mesh.get('filename')!=edit['source_uri'] or sha256(mesh_resource_path(edit['source_uri']).read_bytes()).hexdigest()!=edit['source_sha256'] or sha256(mesh_resource_path(edit['effective_uri']).read_bytes()).hexdigest()!=edit['effective_sha256']:raise CollisionError('recorded profile resource mismatch')
        mesh.set('filename',edit['effective_uri'])
    text=canonical(data)
    return GeometryProfile(text,sha256(text.encode()).hexdigest(),ET.tostring(xml,encoding='unicode') if data['method']!='official_full' else None)
