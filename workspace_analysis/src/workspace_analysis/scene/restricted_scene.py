"""Deterministic World S1 obstacle variants, using existing Schema v1."""
import json
import numpy as np
from .schema import parse_scene
from .model import SceneError


def restricted_scene(world, label, config):
    if config.get('schema_version')!=1 or config.get('usage')!='RESEARCH_ONLY':
        raise SceneError('research restricted scene configuration required')
    if label not in config['scenes']:
        raise SceneError('unknown restricted scene')
    size=np.asarray(config['obstacle_size_m'])
    if size.shape!=(3,) or size.dtype.kind not in 'fiu' or not np.isfinite(size).all() or np.any(size<=0):
        raise SceneError('three finite positive Box dimensions required')
    variant=config['scenes'][label]
    if len(set(variant['obstacle_ports']))!=len(variant['obstacle_ports']) or variant['target'] in variant['obstacle_ports']:
        raise SceneError('duplicate or target-blocking obstacle declaration')
    data=world.to_dict()
    scene=data['scene']
    new_id=world.scene_id+'__restricted_'+label
    mapping={f.id:new_id+f.id[len(world.scene_id):] for f in world.frames}
    mapping['base_link']='base_link'
    scene['id']=new_id
    for frame in scene['frames']:
        frame['id']=mapping[frame['id']]
        frame['parent']=mapping[frame['parent']]
    for obj in scene['collision_objects']:
        obj['parent_frame']=mapping[obj['parent_frame']]
    for item in scene['entities']+scene['ports']:
        item['frame']=mapping[item['frame']]
    for port_id in variant['obstacle_ports']:
        port=next((p for p in world.ports if p['id']==port_id),None)
        if port is None:
            raise SceneError('unknown obstacle port')
        center=world.frame_graph.resolve('base_link',port['frame'])[:3,3].copy()
        center[0]-=size[0]/2
        scene['collision_objects'].append({'id':'restricted_'+label+'_'+port_id,'type':'box','parent_frame':'base_link',
            'pose':{'position_m':center.tolist(),'quaternion_xyzw':[0.,0.,0.,1.]},'size_m':size.tolist()})
    generated=parse_scene(json.dumps(data,allow_nan=False))
    if len({o['id'] for o in generated.collision_objects})!=len(generated.collision_objects):
        raise SceneError('duplicate obstacle identity')
    return generated


def geometry_audit(original, generated, target_id):
    def bounds(scene,obj):
        points=np.array([[x,y,z] for x in (-.5,.5) for y in (-.5,.5) for z in (-.5,.5)])*np.asarray(obj['size_m'])
        transform=scene.object_transform(obj['id'])
        points=points@transform[:3,:3].T+transform[:3,3]
        return points.min(0),points.max(0)
    originals={o['id']:o for o in original.collision_objects}
    for obj in generated.collision_objects:
        if obj['id'] in originals:
            np.testing.assert_allclose(generated.object_transform(obj['id']),original.object_transform(obj['id']),atol=1e-12)
            if list(obj['size_m'])!=list(originals[obj['id']]['size_m']):
                raise SceneError('original geometry changed')
    for port in original.ports:
        other=next(p for p in generated.ports if p['id']==port['id'])
        np.testing.assert_allclose(original.frame_graph.resolve('base_link',port['frame']),generated.frame_graph.resolve('base_link',other['frame']),atol=1e-12)
    target=next(p for p in generated.ports if p['id']==target_id)
    point=generated.frame_graph.resolve('base_link',target['frame'])[:3,3]
    evidence=[]
    for obj in generated.collision_objects:
        if obj['id'] in originals:
            continue
        lo,hi=bounds(generated,obj)
        contacts=[]
        for other in original.collision_objects:
            a,b=bounds(original,other)
            overlap=np.minimum(hi,b)-np.maximum(lo,a)
            if np.all(overlap>=-1e-12):
                contacts.append({'object':other['id'],'relationship':'VOLUME_OVERLAP' if np.all(overlap>1e-12) else 'SURFACE_CONTACT',
                    'aabb_overlap_m':np.maximum(overlap,0).tolist()})
        inside=bool(np.all(point>=lo) and np.all(point<=hi))
        if inside:
            raise SceneError('target origin inside local obstacle')
        evidence.append({'object':obj['id'],'bounds_world_m':[lo.tolist(),hi.tolist()],
            'target_origin_inside':inside,'original_geometry_contacts':contacts})
    return {'status':'PASS','original_objects_checked':len(originals),'ports_checked':len(original.ports),
            'new_objects':evidence,'relation_method':'axis-aligned S1 Box AABBs; touching at front plane is recorded'}
