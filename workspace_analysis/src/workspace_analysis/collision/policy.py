"""Explicit, provisional self-ACM policy; never infer permissions from frequency."""
from hashlib import sha256
import json
from pathlib import Path
import yaml
from .protocol import CollisionError


def canonical_policy(data, model):
    """Normalize reversed/duplicate pairs; reject unknown links and unsupported margins."""
    keys={'version','model_sha256','review_status','scope','allowed_pairs','unresolved_pairs','clearance'}
    if not isinstance(data,dict) or set(data)!=keys:raise CollisionError('policy fields mismatch')
    if data['version']!=1 or type(data['version']) is not int:raise CollisionError('policy version')
    if data['model_sha256']!=model.metadata()['expanded_urdf_sha256']:raise CollisionError('policy model mismatch')
    if data['review_status'] not in ['baseline','provisional_diagnostic']:raise CollisionError('policy not validated; unsupported review claim')
    if data['scope']!='bare_gripper_model':raise CollisionError('tool/cable geometry unverified')
    c=data['clearance']
    if not isinstance(c,dict) or set(c)!={'environment_m','self_m','contact_mode'}:raise CollisionError('clearance fields')
    if any(type(c[k]) not in (int,float) or c[k]!=0 for k in ['environment_m','self_m']):
        raise CollisionError('positive/invalid clearance unsupported')
    if c['contact_mode']!='FCL_raw_boolean':raise CollisionError('epsilon/distance policy unsupported')
    links={j.parent for j in model.joints.values()}|{j.child for j in model.joints.values()}
    unique={}
    for entry in data['allowed_pairs']:
        if set(entry)!={'links','reason','evidence','review_status'}:raise CollisionError('pair fields')
        pair=tuple(sorted(entry['links']))
        if len(pair)!=2 or pair[0]==pair[1] or not set(pair)<=links:raise CollisionError('invalid self pair')
        if not entry['reason'] or not entry['evidence'] or entry['review_status']!='provisional_model_only':
            raise CollisionError('pair evidence/review missing')
        normalized={**entry,'links':list(pair)}
        if pair in unique and unique[pair]!=normalized:raise CollisionError('conflicting duplicate pair')
        unique[pair]=normalized
    for pair in data['unresolved_pairs']:
        if len(pair)!=2 or not set(pair)<=links:raise CollisionError('invalid unresolved pair')
        if tuple(sorted(pair)) in unique:raise CollisionError('unresolved pair cannot be allowed')
    result={**data,'allowed_pairs':[unique[k] for k in sorted(unique)],
            'unresolved_pairs':sorted({tuple(sorted(p)) for p in data['unresolved_pairs']})}
    text=json.dumps(result,sort_keys=True,separators=(',',':'),ensure_ascii=False,allow_nan=False)
    return text,sha256(text.encode()).hexdigest()


def load_policy(path,model):
    """Load explicit configuration; duplicate JSON/YAML keys are rejected."""
    class Loader(yaml.SafeLoader):pass
    def mapping(loader,node):
        result={}
        for k,v in node.value:
            key=loader.construct_object(k)
            if key in result:raise CollisionError('duplicate policy key')
            result[key]=loader.construct_object(v)
        return result
    Loader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,mapping)
    return canonical_policy(yaml.load(Path(path).read_text(),Loader=Loader),model)
