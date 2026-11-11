"""Hash-checked URDF collision surfaces transformed by native RobotState; no FK."""
from hashlib import sha256
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from ..geometry.se3 import pose_to_matrix, validate_transform
from .protocol import CollisionError,mesh_resource_path


def read_stl(path):
    """Read binary STL triangles exactly; reject unsupported format instead of guessing."""
    data=path.read_bytes()
    if len(data)<84:raise CollisionError('truncated STL')
    n=struct.unpack_from('<I',data,80)[0]
    if not n or len(data)!=84+50*n:raise CollisionError('expected nonempty binary STL')
    vertices=np.array([struct.unpack_from('<9f',data,84+50*i+12) for i in range(n)],dtype=np.float64).reshape(-1,3)
    if not np.isfinite(vertices).all():raise CollisionError('nonfinite STL vertices')
    return vertices,np.arange(3*n,dtype=np.int64).reshape(-1,3),sha256(data).hexdigest()


def robot_surfaces(model,ready,row):
    """Return every physical collision Mesh in base_link, including auxiliary links."""
    expected=model.metadata()['expanded_urdf_sha256']
    if ready['identity']['urdf_sha256']!=expected or row['identity']['urdf_sha256']!=expected:
        raise CollisionError('render model identity mismatch')
    for key in ['scene_sha256','robot_geometry_sha256','backend_sha256']:
        if ready['identity'][key]!=row['identity'][key]:raise CollisionError('render/query identity mismatch')
    geometry=ready['audit']['geometry'];loaded=ready['audit']['loaded_shapes'];out=[]
    for link in ET.fromstring(model.urdf_xml).findall('link'):
        name=link.get('name')
        for index,collision in enumerate(link.findall('collision')):
            mesh=collision.find('geometry/mesh')
            if mesh is None:raise CollisionError('renderer currently requires Mesh collision geometry')
            uri=mesh.get('filename');path=mesh_resource_path(uri)
            vertices,faces,digest=read_stl(path)
            audit=[x for x in geometry if x['link']==name][index]
            scale=np.array([float(v) for v in mesh.get('scale','1 1 1').split()])
            if digest!=audit['resource_sha256'] or uri!=audit['uri'] or not np.array_equal(scale,audit['scale']):
                raise CollisionError('render mesh resource mismatch')
            origin=collision.find('origin')
            xyz=[0,0,0] if origin is None else [float(v) for v in origin.get('xyz','0 0 0').split()]
            rpy=[0,0,0] if origin is None else [float(v) for v in origin.get('rpy','0 0 0').split()]
            local=pose_to_matrix(xyz,Rotation.from_euler('xyz',rpy).as_quat())
            native_origin=[x for x in loaded if x['link']==name][index]['origin']
            if not np.allclose(local,native_origin,atol=1e-12,rtol=0):raise CollisionError('collision origin mismatch')
            transform=validate_transform(row['link_transforms_base'][name])@local
            points=(vertices*scale)@transform[:3,:3].T+transform[:3,3]
            out.append({'id':name,'geometry_index':index,'vertices':points,'faces':faces,
                        'uri':uri,'sha256':digest,'scale':scale.tolist(),'collision_origin':local.tolist(),
                        'T_base_collision':transform.tolist()})
    if len(out)!=ready['audit']['moveit_shape_count']:raise CollisionError('missing robot surfaces')
    return out
