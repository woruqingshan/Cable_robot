"""Mesh topology and joint-axis evidence; no collision decision from contact samples."""
import numpy as np
from scipy.spatial.transform import Rotation
FOCUS=[('back_shell_green_link','link1','joint1'),('link2','servo_link1','joint2'),('gripper_link','l_in_link','l_in_joint'),('gripper_link','r_in_link','r_in_joint'),('l_in_link','l_out_link','l_out_joint'),('l_link','l_out_link','l_joint'),('l_link','r_link','l_joint'),('r_in_link','r_out_link','r_out_joint'),('r_link','r_out_link','r_joint'),('link2','link3','joint3')]


def joint_frame(model,row,name):
    joint=model.joints[name];local=np.eye(4);local[:3,:3]=Rotation.from_euler('xyz',joint.rpy).as_matrix();local[:3,3]=joint.xyz
    return np.asarray(row['link_transforms_base'][joint.parent])@local


def topology(model,first,second):
    parent={j.child:j for j in model.joints.values()}
    def ancestors(link):
        result=[]
        while link in parent:result.append(parent[link]);link=parent[link].parent
        return result
    a=ancestors(first);b=ancestors(second);common={first}|{j.parent for j in a};lca=second
    for j in [None,*b]:
        if j is not None:lca=j.parent
        if lca in common:break
    path=[]
    for side in [a,b]:
        for j in side:
            if j.child==lca:break
            path.append(j)
            if j.parent==lca:break
    return {'lca':lca,'joint_path':[j.name for j in path],'moving_joints':[{'name':j.name,'mimic_joint':j.mimic_joint,'axis':j.axis} for j in path if j.kind!='fixed'],'interpretation':'URDF tree path only; not a measured physical closed-loop constraint'}


def surface_topology(vertices,faces):
    unique,inverse=np.unique(vertices,axis=0,return_inverse=True);indices=inverse[faces]
    edges=np.sort(np.vstack([indices[:,[0,1]],indices[:,[1,2]],indices[:,[2,0]]]),axis=1)
    _,counts=np.unique(edges,axis=0,return_counts=True)
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    graph=coo_matrix((np.ones(len(edges)),(edges[:,0],edges[:,1])),shape=(len(unique),len(unique)))
    n,labels=connected_components(graph,directed=False)
    components=[]
    for i in range(n):
        points=unique[labels==i];components.append({'vertex_count':len(points),'bounds_link_m':[points.min(0).tolist(),points.max(0).tolist()]})
    return {'triangle_count':len(faces),'unique_vertex_count':len(unique),'vertex_connected_components':components,'boundary_edges':int((counts==1).sum()),'nonmanifold_edges':int((counts>2).sum()),'degenerate_triangles':int((np.linalg.norm(np.cross(vertices[faces[:,1]]-vertices[faces[:,0]],vertices[faces[:,2]]-vertices[faces[:,0]]),axis=1)<1e-15).sum()),'physical_volume_validated':False}


def contact_evidence(model,row,first,second,joint_name):
    contacts=[c for c in row['contacts_base'] if c['kind']=='self' and {c['first'],c['second']}=={first,second}]
    transform=joint_frame(model,row,joint_name);inverse=np.linalg.inv(transform);axis=np.asarray(model.joints[joint_name].axis);axis/=np.linalg.norm(axis)
    points=np.asarray([c['position_base_m'] for c in contacts]).reshape(-1,3)
    local=points@inverse[:3,:3].T+inverse[:3,3]
    axial=local@axis;radial=np.linalg.norm(local-np.outer(axial,axis),axis=1)
    distance=next((p for p in row.get('pair_geometry_distances',[]) if set(p['links'])=={first,second}),None)
    return {'pair_distance':distance,'contact_count':len(contacts),'joint':joint_name,'axis_joint_frame':axis.tolist(),'T_base_joint_zero':transform.tolist(),'points_joint_m':local.tolist(),'radial_range_m':[float(radial.min()),float(radial.max())] if len(radial) else None,'axial_range_m':[float(axial.min()),float(axial.max())] if len(axial) else None,'depth_range_m':[min(c['depth_m'] for c in contacts),max(c['depth_m'] for c in contacts)] if contacts else None,'contact_surface_complete':False}
