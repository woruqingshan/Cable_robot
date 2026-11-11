"""Model-only coverage inventory; visual envelopes are evidence, not hardware certification."""
from hashlib import sha256
import struct
import xml.etree.ElementTree as ET
import numpy as np
from .protocol import mesh_resource_path


def coverage_report(model):
    root=ET.fromstring(model.urdf_xml)
    links=[];lidar=None
    for link in root.findall('link'):
        collisions=link.findall('collision');visuals=link.findall('visual')
        links.append({'link':link.get('name'),'collision_geometry_count':len(collisions),
                      'classification':'modeled_geometry' if collisions else 'missing_physical_geometry' if visuals else 'reference_frame'})
        if link.get('name')=='lidar_link':
            visual=link.find('visual');mesh=visual.find('geometry/mesh');path=mesh_resource_path(mesh.get('filename'))
            data=path.read_bytes();n=struct.unpack_from('<I',data,80)[0]
            if len(data)!=84+50*n:raise ValueError('LiDAR STL is not expected binary triangle format')
            points=np.array([struct.unpack_from('<9f',data,84+50*i+12) for i in range(n)]).reshape(-1,3)
            scale=np.array([float(x) for x in mesh.get('scale','1 1 1').split()]);points*=scale
            lidar={'resource':str(path),'resource_sha256':sha256(data).hexdigest(),'triangles':n,
                   'visual_origin_xml':ET.tostring(visual.find('origin'),encoding='unicode') if visual.find('origin') is not None else 'identity',
                   'mesh_local_bounds_m':[points.min(axis=0).tolist(),points.max(axis=0).tolist()],
                   'overlay_implemented':False,'physical_envelope_validated':False,
                   'reason':'visual mesh bounds establish a model-only candidate; hardware envelope/mount tolerance unmeasured'}
    return {'scope':'bare_gripper_model','links':links,'lidar_visual_evidence':lidar,
            'modeled_links':sum(x['collision_geometry_count']>0 for x in links),
            'missing_task_geometry':['RJ45 plug','calibrated TCP','flexible cable'],
            'gripper_state_model_verified':True,'physical_gripper_calibrated':False,'coverage_validated':False}
