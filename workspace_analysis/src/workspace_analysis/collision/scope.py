"""Explicit research scope, separate from ACM permissions and geometry profiles."""
from hashlib import sha256
import json
from pathlib import Path
import yaml
from .assembly import assembly_topology
from .geometry_audit import topology
from .protocol import CollisionError,strict_json

FIELDS={'version','id','source_model_sha256','existing_acm_policy_sha256','assembly_root','assembly_members','internal_diagnostic_pairs','blocking_rule','environment_rule','geometry_rule','missing_coverage','verification_status','evidence_references'}
MECHANISMS={'gear_meshing_candidate','joint_bearing_candidate','linkage_assembly_candidate','unresolved_internal_contact'}


def canonical_scope(data,model,policy):
    if not isinstance(data,dict) or set(data)!=FIELDS or type(data['version']) is not int or data['version']!=1:raise CollisionError('task scope fields/version mismatch')
    if data['source_model_sha256']!=model.metadata()['expanded_urdf_sha256'] or data['existing_acm_policy_sha256']!=policy[1]:raise CollisionError('task scope model/ACM mismatch')
    if data['assembly_root']!='gripper_link' or data['assembly_members']!=assembly_topology(model)['members']:raise CollisionError('task scope gripper membership mismatch')
    if data['blocking_rule']!='all_other_self_pairs_with_existing_acm' or data['environment_rule']!='official_full_all_scene_objects' or data['geometry_rule']!='official_full_only' or data['verification_status']!='task_scoped_unverified':raise CollisionError('unsupported/optimistic task scope rule')
    if not data['missing_coverage'] or not data['evidence_references'] or not isinstance(data['id'],str) or not data['id']:raise CollisionError('scope evidence/coverage required')
    members=set(data['assembly_members']);pairs=[];seen=set();allowed={tuple(p['links']) for p in json.loads(policy[0])['allowed_pairs']}
    for p in data['internal_diagnostic_pairs']:
        if set(p)!={'links','mechanism','urdf_joint_path','evidence','unverified_factors'}:raise CollisionError('internal diagnostic fields')
        pair=tuple(sorted(p['links']))
        if len(pair)!=2 or pair[0]==pair[1] or not set(pair)<=members or pair in seen or pair in allowed:raise CollisionError('illegal/duplicate internal pair')
        if p['mechanism'] not in MECHANISMS or not p['evidence'] or not p['unverified_factors'] or sorted(p['urdf_joint_path'])!=sorted(topology(model,*pair)['joint_path']):raise CollisionError('internal topology/evidence mismatch')
        seen.add(pair);pairs.append({**p,'links':list(pair),'urdf_joint_path':sorted(p['urdf_joint_path'])})
    if not pairs:raise CollisionError('explicit internal pairs required')
    text=json.dumps({**data,'internal_diagnostic_pairs':sorted(pairs,key=lambda p:p['links'])},sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False)
    return text,sha256(text.encode()).hexdigest()


def load_scope(path,model,policy):
    class Loader(yaml.SafeLoader):pass
    def mapping(loader,node):
        result={}
        for k,v in node.value:
            key=loader.construct_object(k)
            if key in result:raise CollisionError('duplicate scope key')
            result[key]=loader.construct_object(v)
        return result
    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,mapping)
    return canonical_scope(yaml.load(Path(path).read_text(),Loader=Loader),model,policy)


def check_scope_result(reply,scope,identity):
    result=reply.get('task_scope',{})
    if result.get('scope_sha256')!=scope[1] or result.get('binding')!=identity:raise CollisionError('task scope response identity mismatch')
    for key in ['official_raw_self_collision','official_effective_self_collision','task_scope_blocking_self_collision','gripper_internal_collision','environment_collision','zero_clearance_candidate']:
        if type(result.get(key)) is not bool:raise CollisionError('invalid task scope boolean')
    if result['official_raw_self_collision']!=reply['raw_self_collision'] or result['official_effective_self_collision']!=reply['effective_self_collision'] or result['environment_collision']!=reply['environment_collision']:raise CollisionError('official/task facts disagree')
    if result['zero_clearance_candidate']!=(not result['task_scope_blocking_self_collision'] and not result['environment_collision']):raise CollisionError('inconsistent task candidate')
    return reply
