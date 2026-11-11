"""Pure SceneSnapshot -> marker descriptions; ROS message conversion is separate."""
from dataclasses import dataclass
from typing import Any
import numpy as np


@dataclass(frozen=True)
class MarkerSpec:
    namespace: str
    marker_id: int
    kind: str
    position: tuple[float, ...]
    orientation: tuple[float, ...]
    scale: tuple[float, ...]
    color: tuple[float, ...]
    points: tuple[tuple[float, ...], ...] = ()
    text: str = ''


def scene_specs(scene: Any) -> tuple[MarkerSpec, ...]:
    from workspace_analysis.geometry.se3 import matrix_to_quaternion
    result = []
    for i,obj in enumerate(scene.collision_objects):
        t = scene.object_transform(obj['id'], scene.reference_frame)
        scale = tuple(obj['size_m']) if obj['type'] == 'box' else (2*obj['radius_m'],2*obj['radius_m'],obj['length_m'])
        color = (.53,.60,.66,.32) if obj['id'].startswith('cab_') else (.22,.42,.57,.52)
        if obj['id'].startswith('restricted_'):
            color=(1.,.52,.05,.85)
        result.append(MarkerSpec('idc_geometry',i,'CUBE' if obj['type'] == 'box' else 'CYLINDER',
            tuple(t[:3,3]),tuple(matrix_to_quaternion(t[:3,:3])),scale,color))
    return tuple(result)


def restriction_specs(info,goal_id=None):
    """World boundary markers converted into base coordinates; finite display extent."""
    config=info.get('workspace_config')
    if config is None:
        return ()
    world_from_base=np.asarray(info['world_from_base'])
    rotation=world_from_base[:3,:3]
    translation=world_from_base[:3,3]
    def local(point):
        return tuple((np.asarray(point)-translation)@rotation)
    specs=[]
    if config['enabled']:
        x=config['x_trigger_m'];far=x+.11
        y0,y1=config['y_min_m'],config['y_max_m'];z0,z1=config['z_min_m'],config['z_max_m']
        corners=[(x,y0,z0),(x,y1,z0),(x,y1,z1),(x,y0,z1)]
        edges=[]
        for i in range(4):
            a,b=corners[i],corners[(i+1)%4]
            edges.extend([a,b,a,(far,a[1],a[2]),(far,a[1],a[2]),(far,b[1],b[2])])
        specs.append(MarkerSpec('workspace_boundary',0,'LINE_LIST',(0.,0.,0.),(0.,0.,0.,1.),(.0015,0.,0.),
            (.9,.25,.9,.8),tuple(local(p) for p in edges)))
        label=('TCP LOCAL-ONLY; must stay x >= 250 mm' if config.get('mode')=='local_only'
               else 'TCP boundary ON; x >= 250 mm (display ends at 360 mm)')
        specs.append(MarkerSpec('workspace_boundary',1,'TEXT_VIEW_FACING',local((x,0,z1+.025)),(0.,0.,0.,1.),(0.,0.,.011),
            (1.,.5,1.,1.),text=label))
    if 'start_world_tcp_xyz_m' in info:
        start=local(info['start_world_tcp_xyz_m'])
        specs.append(MarkerSpec('ready_pose',0,'SPHERE',start,(0.,0.,0.,1.),(.012,)*3,(1.,.75,.1,1.)))
        specs.append(MarkerSpec('ready_pose',1,'TEXT_VIEW_FACING',tuple(np.asarray(start)+[0,0,.02]),
            (0.,0.,0.,1.),(0.,0.,.012),(1.,.85,.2,1.),text='Start '+info['start_id']+' / '+info['workspace_mode']))
    if 'preparation_goal_world_tcp_xyz_m' in info:
        goal=local(info['preparation_goal_world_tcp_xyz_m'])
        specs.append(MarkerSpec('preparation_goal',0,'SPHERE',goal,(0.,0.,0.,1.),(.012,)*3,(.1,1.,1.,1.)))
        specs.append(MarkerSpec('preparation_goal',1,'TEXT_VIEW_FACING',tuple(np.asarray(goal)+[0,0,.02]),
            (0.,0.,0.,1.),(0.,0.,.012),(.1,1.,1.,1.),text='Ready '+info['ready_goal_id']+' (reference motion)'))
    failures=info.get('failure_diagnostics',{}).get(str(goal_id),[])
    for i,failure in enumerate(failures):
        specs.append(MarkerSpec('restriction_failure',i,'SPHERE',local(failure['world_tcp_xyz_m']),
            (0.,0.,0.,1.),(.013,)*3,(1.,.1,.1,.95)))
    return tuple(specs)


def port_specs(scene: Any, selected_port: str, targets: Any, *, tcp: Any = None,
               trajectory: Any = None) -> tuple[MarkerSpec, ...]:
    result = []
    def add(kind, position, scale, color, *, points=(), text=''):
        result.append(MarkerSpec('idc_ports',len(result),kind,tuple(position),(0.,0.,0.,1.),tuple(scale),tuple(color),tuple(tuple(p) for p in points),text))
    for port in scene.ports:
        t = scene.frame_graph.resolve(scene.reference_frame,port['frame']); p = t[:3,3]
        selected = port['id'] == selected_port
        add('SPHERE',p,(.010,)*3,(.0,.9,1.,1.) if selected else (.75,.75,.75,1.))
        add('TEXT_VIEW_FACING',p+[0,0,.015],(0.,0.,.013),(1.,1.,1.,1.),text=port['id'])
        for axis, length, color in [(port['insertion_axis_local'],.045,(1.,.15,.1,1.)),
                                     (port['key_axis_local'],.030,(.15,.4,1.,1.))]:
            direction = t[:3,:3] @ np.asarray(axis)
            add('ARROW',(0.,0.,0.),(.0018,.0045,.009),color,points=(p,p+length*direction))
    pre = targets.T_base_pre_tcp[:3,3]
    add('SPHERE',pre,(.008,)*3,(.1,1.,.2,1.))
    add('LINE_STRIP',(0.,0.,0.),(.0018,0.,0.),(.1,1.,.2,.85),points=(pre,targets.port_tcp.matrix()[:3,3]))
    if trajectory is not None:
        add('LINE_STRIP',(0.,0.,0.),(.0015,0.,0.),(.0,.9,.9,.8),points=trajectory.tcp[:,:3,3])
    if tcp is not None:
        t = np.asarray(tcp); p = t[:3,3]
        add('SPHERE',p,(.009,)*3,(1.,.1,1.,1.))
        for i,color in enumerate(((1.,0.,0.,1.),(0.,1.,0.,1.),(0.,.3,1.,1.))):
            add('ARROW',(0.,0.,0.),(.0015,.004,.008),color,points=(p,p+.025*t[:3,i]))
    return tuple(result)


def marker_array(specs: tuple[MarkerSpec, ...], frame: str, stamp: Any, *, clear: bool = False):
    """ROS imports only at the transport boundary; no alternative geometry model."""
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    array = MarkerArray()
    if clear:
        deletion = Marker(); deletion.action = Marker.DELETEALL
        deletion.header.frame_id = frame; deletion.header.stamp = stamp
        array.markers.append(deletion)
    for item in specs:
        m = Marker(); m.header.frame_id = frame; m.header.stamp = stamp
        m.ns = item.namespace; m.id = item.marker_id; m.type = getattr(Marker,item.kind); m.action = Marker.ADD
        m.pose.position.x,m.pose.position.y,m.pose.position.z = map(float,item.position)
        m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w = map(float,item.orientation)
        m.scale.x,m.scale.y,m.scale.z = map(float,item.scale)
        m.color.r,m.color.g,m.color.b,m.color.a = map(float,item.color)
        m.text = item.text
        for point in item.points:
            p = Point(); p.x,p.y,p.z = map(float,point); m.points.append(p)
        array.markers.append(m)
    return array
