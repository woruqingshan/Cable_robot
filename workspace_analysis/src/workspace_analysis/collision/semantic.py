"""Explicit SRDF source, strict supported subset and independently verified ACM identity.

Sampling classifications are review candidates only. No candidate becomes a permission.
"""
from dataclasses import dataclass
from hashlib import sha256
from itertools import combinations
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import math
from .errors import CollisionError


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False,ensure_ascii=False)


def parse_srdf(xml,model):
    """Validate expanded SRDF; unsupported semantics fail instead of being ignored.

    Supported: groups (joints, links, chains, subgroups), scalar named states,
    explicit disabled collisions. Virtual joints/end effectors require future support.
    """
    try:
        if '<!DOCTYPE' in xml or '<!ENTITY' in xml:raise ValueError('DTD/entities forbidden')
        root=ET.fromstring(xml);urdf=ET.fromstring(model.urdf_xml)
        if root.tag!='robot' or root.attrib!={'name':urdf.get('name')}:raise ValueError('SRDF robot name mismatch')
        links={e.get('name') for e in urdf.findall('link')};groups={};states={};disabled={}
        for e in root:
            if e.tag=='group':
                name=e.get('name')
                if not name or set(e.attrib)!={'name'} or name in groups or not len(e):raise ValueError('invalid/duplicate group')
                groups[name]=e
            elif e.tag not in {'group_state','disable_collisions'}:raise ValueError('unsupported SRDF element: '+e.tag)
        resolved={}
        def members(name,visiting):
            if name in visiting or name not in groups:raise ValueError('cyclic/unknown subgroup')
            if name in resolved:return resolved[name]
            found=set()
            for e in groups[name]:
                if len(e):raise ValueError('nested group member')
                if e.tag in {'joint','link','group'}:
                    if set(e.attrib)!={'name'}:raise ValueError('group attributes')
                    v=e.get('name')
                    if e.tag=='joint':
                        if v not in model.joints:raise ValueError('unknown joint')
                        found.add(v)
                    elif e.tag=='link':
                        if v not in links:raise ValueError('unknown link')
                        found.update(j.name for j in model.joints.values() if j.child==v)
                    else:found.update(members(v,visiting|{name}))
                elif e.tag=='chain':
                    if set(e.attrib)!={'base_link','tip_link'}:raise ValueError('chain attributes')
                    base=e.get('base_link');tip=e.get('tip_link');parent={j.child:j for j in model.joints.values()}
                    if base not in links or tip not in links or base==tip:raise ValueError('invalid chain')
                    if base in parent:found.add(parent[base].name)
                    while tip!=base:
                        if tip not in parent:raise ValueError('chain base not ancestor')
                        j=parent[tip];found.add(j.name);tip=j.parent
                else:raise ValueError('unknown group member')
            resolved[name]=found;return found
        for name in groups:members(name,set())
        for e in root:
            if e.tag=='disable_collisions':
                if len(e) or set(e.attrib)!={'link1','link2','reason'} or not e.get('reason','').strip():raise ValueError('disabled pair fields')
                a,b=e.get('link1'),e.get('link2');pair=tuple(sorted([a,b]))
                if a==b or not {a,b}<=links or pair in disabled:raise ValueError('invalid/duplicate disabled pair')
                disabled[pair]=e.get('reason')
            elif e.tag=='group_state':
                if set(e.attrib)!={'name','group'} or not e.get('name') or e.get('group') not in groups:raise ValueError('invalid named state')
                key=(e.get('group'),e.get('name'))
                if key in states or not len(e):raise ValueError('duplicate/empty named state')
                values={}
                for j in e:
                    if j.tag!='joint' or set(j.attrib)!={'name','value'} or len(j):raise ValueError('invalid state joint')
                    name=j.get('name');spec=model.joints.get(name)
                    if name in values or name not in resolved[key[0]] or spec is None or spec.kind=='fixed' or spec.mimic_joint:raise ValueError('state joint not independent group member')
                    v=float(j.get('value'))
                    if not math.isfinite(v) or not spec.lower<=v<=spec.upper:raise ValueError('named state out of bounds')
                    values[name]=v
                states[key]=values
        return {'robot_name':root.get('name'),'groups':{n:sorted(v) for n,v in resolved.items()},
                'named_states':[{'group':g,'name':n,'positions':v} for (g,n),v in sorted(states.items())],
                'disabled_pairs':[list(p) for p in sorted(disabled)],'reasons':[{'links':list(p),'reason':disabled[p]} for p in sorted(disabled)]}
    except (ET.ParseError,ValueError,TypeError,KeyError) as exc:raise CollisionError('invalid SRDF: '+str(exc)) from exc


def candidate_pairs(source_text,model):
    """Select ONLY Adjacent/Default from one complete, frozen native proposal.

    This validates provenance/selection, never certifies mechanical assembly safety.
    """
    try:
        source=json.loads(source_text);result=source['result']
        if result.get('status')!='OK' or result.get('backend_status')!='CANDIDATES' or result.get('method')!='MoveIt Setup Assistant computeDefaultCollisions':raise ValueError('not a standard MoveIt candidate result')
        if result.get('semantic_acm_modified') is not False or result.get('finite_sampling_is_safety_proof') is not False:raise ValueError('candidate source made unsupported claims')
        if result['identity']['urdf_sha256']!=model.metadata()['expanded_urdf_sha256']:raise ValueError('candidate source model mismatch')
        geometry=result['identity']['robot_geometry_sha256']
        if not isinstance(geometry,str) or len(geometry)!=64:raise ValueError('missing candidate geometry identity')
        robot=ET.fromstring(model.urdf_xml);links={l.get('name') for l in robot.findall('link') if l.find('collision') is not None}
        all_pairs=set(combinations(sorted(links),2));seen=set();selected={}
        for row in result['candidate_pairs']:
            pair=tuple(row['links']);kind=row['candidate_class']
            if pair not in all_pairs or pair in seen or kind not in {'Adjacent','Default','Always','Never','Not Disabled'}:raise ValueError('invalid/duplicate candidate pair or class')
            if row.get('applied') is not False or row.get('review_status')!='UNREVIEWED' or type(row.get('generator_disable_recommendation')) is not bool or row['generator_disable_recommendation']!=(kind!='Not Disabled'):raise ValueError('candidate source flags')
            seen.add(pair)
            if kind in {'Adjacent','Default'}:selected[pair]=kind
        if seen!=all_pairs:raise ValueError('incomplete candidate source matrix')
        return selected,geometry
    except (ValueError,TypeError,KeyError) as exc:raise CollisionError('invalid candidate source: '+str(exc)) from exc


@dataclass(frozen=True)
class SemanticConfig:
    """Immutable wire payload bound to reviewed SRDF and source URDF bytes."""
    payload_json:str

    def wire(self):return {'semantic':json.loads(self.payload_json)}

    @classmethod
    def from_xml(cls,xml,model,review,*,allow_candidate=False,candidate_source_text=None):
        parsed=parse_srdf(xml,model)
        expected={'urdf_sha256':model.metadata()['expanded_urdf_sha256'],'srdf_sha256':sha256(xml.encode()).hexdigest(),
                  'disabled_pairs':parsed['disabled_pairs']}
        status=review.get('status')
        if any(review.get(k)!=v for k,v in expected.items()) or status not in {'model_only_partial','candidate_baseline_unverified'}:raise CollisionError('semantic review/model/SRDF binding mismatch')
        candidate=status=='candidate_baseline_unverified'
        if candidate:
            if allow_candidate is not True:raise CollisionError('candidate baseline requires explicit allow_candidate=True; not approved semantics')
            if not isinstance(candidate_source_text,str):raise CollisionError('candidate source bytes required')
            selected,geometry=candidate_pairs(candidate_source_text,model)
            if parsed['disabled_pairs']!=[list(p) for p in sorted(selected)] or review.get('selected_categories')!=['Adjacent','Default'] or review.get('usage')!='research_only' or review.get('hardware_validated') is not False:raise CollisionError('candidate selection/usage mismatch')
            if review.get('candidate_source_sha256')!=sha256(candidate_source_text.encode()).hexdigest() or review.get('source_geometry_sha256')!=geometry:raise CollisionError('candidate source identity mismatch')
        elif allow_candidate or candidate_source_text is not None:raise CollisionError('candidate opt-in supplied to strict semantics')
        entries=review.get('pair_reviews',[])
        if len(entries)!=len(expected['disabled_pairs']) or sorted(e.get('links',[]) for e in entries)!=expected['disabled_pairs']:raise CollisionError('each disabled pair needs one review')
        decision='candidate_disabled_unverified' if candidate else 'disabled_model_only'
        if any(e.get('decision')!=decision or not e.get('evidence') for e in entries):raise CollisionError('pair review evidence missing')
        if candidate and any(e.get('candidate_class')!=selected[tuple(e['links'])] for e in entries):raise CollisionError('candidate pair class mismatch')
        text=canonical(review)
        payload={**expected,'srdf_xml':xml,'review_canonical_json':text,'review_sha256':sha256(text.encode()).hexdigest()}
        if candidate:payload.update(allow_candidate_baseline=True,candidate_source_json=candidate_source_text)
        return cls(canonical(payload))

    @classmethod
    def load(cls,path,model,*,allow_candidate=False):
        path=Path(path);review=json.loads((path.parent/'semantic_manifest.json').read_text())
        source=None
        if review.get('status')=='candidate_baseline_unverified':source=(path.parent/'candidate_source.json').read_bytes().decode('utf-8')
        return cls.from_xml(path.read_bytes().decode('utf-8'),model,review,allow_candidate=allow_candidate,candidate_source_text=source)

    @property
    def review_status(self):
        return json.loads(json.loads(self.payload_json)['review_canonical_json'])['status']


def check_semantic_ready(reply,wire):
    sem=wire['semantic'];identity=reply['identity'];audit=reply['audit'].get('semantic',{})
    pairs=sem['disabled_pairs'];digest=sha256(canonical(pairs).encode()).hexdigest()
    if identity.get('srdf_sha256')!=sem['srdf_sha256'] or identity.get('semantic_acm_sha256')!=digest or identity.get('semantic_review_sha256')!=sem['review_sha256']:raise CollisionError('semantic identity acknowledgement mismatch')
    if audit.get('disabled_pairs')!=pairs or audit.get('actual_disabled_pairs')!=pairs or audit.get('acm_sha256')!=digest or audit.get('raw_acm_size')!=0 or audit.get('initialization_verified') is not True or audit.get('overlay_applied') is not False:raise CollisionError('actual native ACM differs from expected SRDF')
    review=json.loads(sem['review_canonical_json'])
    if audit.get('review_status')!=review['status']:raise CollisionError('semantic review status acknowledgement mismatch')
    if review['status']=='candidate_baseline_unverified':
        if identity.get('semantic_review_status')!=review['status'] or identity.get('semantic_usage')!='research_only' or identity.get('robot_geometry_sha256')!=review['source_geometry_sha256']:raise CollisionError('candidate status/geometry acknowledgement mismatch')
    if reply['audit']['acm_size']!=len({n for p in pairs for n in p}):raise CollisionError('unexpected ACM names/defaults')


def check_semantic_query(reply,pairs,review_status=None):
    if type(reply.get('semantic_self_collision')) is not bool or reply.get('semantic_query_method')!='independent_MoveIt_FCL_contacts_and_boolean' or reply.get('hardware_validated') is not False:raise CollisionError('invalid semantic FCL result')
    if reply.get('disabled_collision_pairs')!=pairs or type(reply.get('semantic_pair_list_may_be_truncated')) is not bool:raise CollisionError('semantic pair identity mismatch')
    if review_status is not None and reply.get('semantic_review_status','model_only_partial')!=review_status:raise CollisionError('query semantic review status changed')
    blocking=reply.get('semantic_blocking_pairs')
    known={tuple(p) for p in pairs}
    if not isinstance(blocking,list) or any(len(p)!=2 or tuple(sorted(p)) in known for p in blocking):raise CollisionError('invalid semantic blocking pairs')
    if blocking and not reply['semantic_self_collision'] or reply['semantic_self_collision'] and not reply['raw_self_collision']:raise CollisionError('semantic boolean/pair contradiction')
    for pair in blocking:
        if any(not isinstance(n,str) or n not in reply['link_transforms_base'] for n in pair) or pair[0]==pair[1]:raise CollisionError('unknown semantic blocking link')
    clearance=reply.get('semantic_clearance')
    if not isinstance(clearance,dict) or set(clearance)!={'self','environment'}:raise CollisionError('missing semantic clearance')
    for value in clearance.values():
        if type(value.get('available')) is not bool:raise CollisionError('invalid semantic distance availability')
        d=value.get('separation_m')
        if value['available']:
            if type(d) not in (int,float) or not math.isfinite(d) or d<0:raise CollisionError('invalid semantic distance')
        elif d is not None:raise CollisionError('unavailable semantic distance must be null')
    if clearance['environment']!=reply['clearance']['environment']:raise CollisionError('semantic environment query changed')


def generate_candidates(model,rows,default_row):
    """MoveIt-style classification on real Raw FCL samples; never an ACM writer.

    Never means only never observed in this finite input. Classification flags are
    independent (Adjacent may also be Default/Always); Remaining Checked is explicit.
    """
    xml=ET.fromstring(model.urdf_xml);links=sorted(l.get('name') for l in xml.findall('link') if l.find('collision') is not None)
    adjacent={tuple(sorted([j.parent,j.child])) for j in model.joints.values()}
    def pairs(row):return {tuple(sorted([p['first'],p['second']])) for p in row['colliding_pairs'] if p['kind']=='self'}
    sets=[pairs(r) for r in rows];default=pairs(default_row);result=[]
    if not rows or any(r.get('pairs_may_be_truncated') for r in [*rows,default_row]):raise CollisionError('candidate classification requires nontruncated raw pairs')
    for p in combinations(links,2):
        n=sum(p in v for v in sets);flags=[]
        if p in adjacent:flags.append('Adjacent')
        if p in default:flags.append('Default')
        if n==len(sets):flags.append('Always')
        if n==0:flags.append('Never')
        result.append({'links':list(p),'candidate_classes':flags or ['Remaining_Checked'],'observed_collisions':n,'sample_count':len(sets),'disabled':False,'review_status':'UNREVIEWED','finite_sampling_is_safety_proof':False})
    return result
