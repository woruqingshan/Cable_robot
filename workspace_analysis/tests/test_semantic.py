"""C3 strict semantics and real native regressions. No weakened historical checks."""
from copy import deepcopy
from hashlib import sha256
import json,os
from pathlib import Path
import numpy as np
import pytest
from workspace_analysis.collision.semantic import SemanticConfig,parse_srdf,canonical,check_semantic_ready,generate_candidates
from workspace_analysis.collision.protocol import CollisionError,make_init
from workspace_analysis.collision.client import CollisionClient
from workspace_analysis.collision.evaluator import Evaluator
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.workspace.results import load_results
from test_task_scope import fixture
ROOT=Path(__file__).resolve().parents[1]


def config(model,disabled=()):
    from xml.etree import ElementTree as E
    root=E.Element('robot',name=E.fromstring(model.urdf_xml).get('name'))
    for a,b in disabled:E.SubElement(root,'disable_collisions',link1=a,link2=b,reason='synthetic reviewed fixture')
    xml=E.tostring(root,encoding='unicode');pairs=sorted([sorted(p) for p in disabled])
    review={'urdf_sha256':model.metadata()['expanded_urdf_sha256'],'srdf_sha256':sha256(xml.encode()).hexdigest(),'disabled_pairs':pairs,'status':'model_only_partial','pair_reviews':[{'links':p,'decision':'disabled_model_only','evidence':'synthetic FCL fixture'} for p in pairs]}
    return SemanticConfig.from_xml(xml,model,review)


@pytest.mark.parametrize('bad',['name','pair','duplicate','joint','chain','cycle','state','tag','nan','attribute'])
def test_invalid_srdf_strict(bad):
    m,_,_,_=fixture();base='<robot name="assembly_fixture">';end='</robot>'
    cases={'name':'<robot name="other"/>','pair':base+'<disable_collisions link1="absent" link2="base_link" reason="x"/>'+end,'duplicate':base+2*'<disable_collisions link1="base_link" link2="gripper_link" reason="x"/>'+end,
      'joint':base+'<group name="arm"><joint name="absent"/></group>'+end,
      'chain':base+'<group name="arm"><chain base_link="gripper_link" tip_link="base_link"/></group>'+end,
      'cycle':base+'<group name="a"><group name="b"/></group><group name="b"><group name="a"/></group>'+end,
      'state':base+'<group name="a"><joint name="joint1"/></group><group_state name="home" group="a"><joint name="joint2" value="0"/></group_state>'+end,
      'nan':base+'<group name="a"><joint name="joint1"/></group><group_state name="home" group="a"><joint name="joint1" value="nan"/></group_state>'+end,
      'tag':base+'<unknown/>'+end,'attribute':base+'<disable_collisions link1="base_link" link2="gripper_link" reason="x" default="true"/>'+end}
    with pytest.raises(CollisionError):parse_srdf(cases[bad],m)


def test_named_states_groups_and_review_binding():
    m,_,_,_=fixture()
    xml='<robot name="assembly_fixture"><group name="arm"><chain base_link="base_link" tip_link="arm_link"/></group><group_state name="fixture_only" group="arm"><joint name="joint1" value=".2"/></group_state></robot>'
    d=parse_srdf(xml,m);assert d['groups']['arm']==list(m.joint_order) and d['named_states'][0]['positions']=={'joint1':.2}
    c=config(m,[['gripper_link','r_in_link']]);review=json.loads(c.wire()['semantic']['review_canonical_json']);review['pair_reviews']=[]
    with pytest.raises(CollisionError):SemanticConfig.from_xml(c.wire()['semantic']['srdf_xml'],m,review)
    review['urdf_sha256']='0'*64
    with pytest.raises(CollisionError):SemanticConfig.from_xml(xml,m,review)


@pytest.mark.parametrize('external',[None,'base','arm','environment','undeclared_gripper'])
def test_real_fcl_semantic_vs_raw_external_positive_and_state_reset(external):
    m,s,p,_=fixture(external);sem=config(m,[['gripper_link','r_in_link']])
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        ready=c.initialize(m,s,semantic=sem);assert ready['audit']['semantic']['initialization_verified']
        for angle in [0,.4,0]:
            r=c.query(m,[angle,0,0,0,0],auxiliary_positions={'r_joint':.3})
            assert r['raw_self_collision']
            if angle==0:
                assert r['environment_collision'] if external=='environment' else r['semantic_self_collision']==(external is not None)
                assert r['semantic_self_collision'] if external in ['base','arm','undeclared_gripper'] else not r['semantic_self_collision']
            assert r['disabled_collision_pairs']==[['gripper_link','r_in_link']]
    with Evaluator(m,s,None,os.environ['C1_BACKEND'],semantic=sem) as e:
        row=e.evaluate([[0]*5])[0]
    assert row['model_configuration_valid']==(external is None) and not row['hardware_validated']
    assert row['raw_self_collision'] and row['effective_self_collision'] # legacy fields remain full/empty ACM
    with pytest.raises(CollisionError,match='independent'):Evaluator(m,s,p,os.environ['C1_BACKEND'],semantic=sem)


@pytest.mark.parametrize('bad',['srdf','model','review','pairs','xml'])
def test_native_rejects_bypassed_python_invalid_semantic_wire(bad):
    m,s,_,_=fixture();sem=config(m,[['gripper_link','r_in_link']]);wire=sem.wire()
    if bad=='srdf':wire['semantic']['srdf_sha256']='0'*64
    if bad=='model':wire['semantic']['urdf_sha256']='0'*64
    if bad=='review':wire['semantic']['review_sha256']='0'*64
    if bad=='pairs':wire['semantic']['disabled_pairs']=[]
    if bad=='xml':
        wire['semantic']['srdf_xml']='<robot name="wrong"/>';wire['semantic']['srdf_sha256']=sha256(wire['semantic']['srdf_xml'].encode()).hexdigest()
        review=json.loads(wire['semantic']['review_canonical_json']);review['srdf_sha256']=wire['semantic']['srdf_sha256'];text=canonical(review);wire['semantic'].update(review_canonical_json=text,review_sha256=sha256(text.encode()).hexdigest())
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        with pytest.raises(CollisionError):c._exchange({'op':'INIT',**make_init(m,s),**wire})


def test_actual_acm_acknowledgement_not_assumed():
    m,s,_,_=fixture();sem=config(m,[['gripper_link','r_in_link']])
    with CollisionClient(os.environ['C1_BACKEND']) as c:r=c.initialize(m,s,semantic=sem)
    for key in ['actual_disabled_pairs','disabled_pairs','initialization_verified','overlay_applied']:
        bad=deepcopy(r);bad['audit']['semantic'][key]=None
        with pytest.raises(CollisionError):check_semantic_ready(bad,sem.wire())


def test_jetrover_semantics_preserves_raw_state_fk_mimic_and_known_rejections():
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');sem=SemanticConfig.load(ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf',m)
    arrays,_=load_results(ROOT/'results/workspace_n100_seed42',model=m)
    from workspace_analysis.collision.policy import load_policy
    with CollisionClient(os.environ['C1_BACKEND']) as old,CollisionClient(os.environ['C1_BACKEND']) as new:
        old.initialize(m,s);old.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m));ready=new.initialize(m,s,semantic=sem)
        for opening in [0,.3,1.]:
            for i in [8,19,73]:
                a=old.query(m,arrays['q'][i],auxiliary_positions={'r_joint':opening});b=new.query(m,arrays['q'][i],auxiliary_positions={'r_joint':opening})
                for key in ['colliding_pairs','raw_self_collision','environment_collision','T_base_to_ee','link_transforms_base','joint_positions_used']:assert a[key]==b[key]
                assert b['semantic_self_collision']==a['effective_self_collision'] and b['semantic_blocking_pairs']==a['effective_self_pairs']
                assert b['semantic_clearance']=={k:a['clearance'][k] for k in ['self','environment']}
                if i==19:assert ['back_shell_black_link','link3'] in b['semantic_blocking_pairs']
                if i in [8,73]:assert b['environment_collision']
        with pytest.raises(CollisionError):new.set_policy(*load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m))
    assert len(ready['audit']['semantic']['actual_disabled_pairs'])==5
    assert ['link2','link3'] not in ready['audit']['semantic']['actual_disabled_pairs']


def test_candidates_are_not_permissions():
    m,_,_,_=fixture();row={'colliding_pairs':[{'kind':'self','first':'gripper_link','second':'r_in_link'}],'pairs_may_be_truncated':False}
    candidates=generate_candidates(m,[row,row],row)
    assert any('Always' in p['candidate_classes'] for p in candidates)
    assert any('Never' in p['candidate_classes'] for p in candidates)
    assert all(p['disabled'] is False and not p['finite_sampling_is_safety_proof'] for p in candidates)
    with pytest.raises(CollisionError):generate_candidates(m,[],row)


def test_standard_moveit_candidates_are_isolated_not_installed_permissions():
    m,s,_,_=fixture();sem=config(m,[['gripper_link','r_in_link']])
    with CollisionClient(os.environ['C1_BACKEND'],timeout=60) as c:
        c.initialize(m,s,semantic=sem);a=c.query(m,[0]*5)
        proposal=c.candidates(trials=0)
        assert proposal['method']=='MoveIt Setup Assistant computeDefaultCollisions'
        assert not proposal['semantic_acm_modified'] and not proposal['finite_sampling_is_safety_proof']
        assert all(not p['applied'] and p['review_status']=='UNREVIEWED' for p in proposal['candidate_pairs'])
        b=c.query(m,[0]*5)
        for key in ['colliding_pairs','semantic_blocking_pairs','disabled_collision_pairs','semantic_self_collision','link_transforms_base','semantic_clearance']:assert a[key]==b[key]


def test_semantic_scene_hash_and_unknown_query_result_fail_closed():
    from workspace_analysis.collision.semantic import check_semantic_query
    m,s,_,_=fixture();sem=config(m,[['gripper_link','r_in_link']])
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        init=make_init(m,s);init['scene_sha256']='0'*64
        with pytest.raises(CollisionError):c._exchange({'op':'INIT',**init,**sem.wire()})
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,s,semantic=sem);r=c.query(m,[0]*5)
    for update in [{'semantic_self_collision':None},{'disabled_collision_pairs':[]},{'semantic_blocking_pairs':[['base_link','missing']]},{'semantic_clearance':{}}]:
        bad={**r,**update}
        with pytest.raises(CollisionError):check_semantic_query(bad,[['gripper_link','r_in_link']])


def test_semantic_viewer_contains_full_mesh_and_explicit_semantic_identity(tmp_path):
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');sem=SemanticConfig.load(ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf',m)
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        ready=c.initialize(m,s,semantic=sem);row=c.query(m,[0]*5);row.update(q=[0]*5,sample_index='diagnostic_not_safe')
        fig=build_collision_figure(s,robot_surfaces(m,ready,row),row)
    assert len([t for t in fig.data if (t.meta or {}).get('role')=='robot_collision_mesh'])==24
    assert fig.layout.meta['query_identity']['semantic_acm_sha256']==ready['identity']['semantic_acm_sha256']
    path=export_collision_viewer(fig,row,tmp_path/'semantic.html');html=path.read_text()
    assert 'Semantic blocking' in html and 'SRDF disabled (contact not guaranteed)' in html
    assert 'semantic_self_collision' in html and 'diagnostic_not_safe' in html
    with pytest.raises(ValueError):export_collision_viewer(fig,row,path)
