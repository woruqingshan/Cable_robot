"""Opt-in research semantics must not masquerade as reviewed assembly permissions."""

from test_resources import SYSTEM_SOURCE

from test_resources import ARTIFACTS
from copy import deepcopy
from hashlib import sha256
import json,os
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from workspace_analysis.collision.semantic import SemanticConfig,candidate_pairs,canonical
from workspace_analysis.collision.decision import semantic_configuration_decision
from workspace_analysis.collision.client import CollisionClient
from workspace_analysis.collision.evaluator import Evaluator
from workspace_analysis.collision.protocol import CollisionError,make_init
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.workspace.results import load_results
ROOT=Path(__file__).resolve().parents[1]
CANDIDATE=ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf'


@pytest.fixture(scope='module')
def robot():return load_model(ROOT/'config/workspace.yaml')


def test_frozen_selection_is_exactly_adjacent_default_not_never(robot):
    c=SemanticConfig.load(CANDIDATE,robot,allow_candidate=True);wire=c.wire()['semantic'];selected,_=candidate_pairs(wire['candidate_source_json'],robot)
    assert len(selected)==29 and list(selected.values()).count('Adjacent')==23 and list(selected.values()).count('Default')==6
    assert wire['disabled_pairs']==[list(p) for p in sorted(selected)]
    assert ['link2','link3'] in wire['disabled_pairs'] and ['back_shell_black_link','link3'] not in wire['disabled_pairs']
    with pytest.raises(CollisionError,match='explicit'):SemanticConfig.load(CANDIDATE,robot)


@pytest.mark.parametrize('change',['source_hash','source_pair','source_model','source_geometry','category','missing_pair','review_claim'])
def test_candidate_provenance_and_review_claims_reject_tampering(robot,change):
    c=SemanticConfig.load(CANDIDATE,robot,allow_candidate=True);w=c.wire()['semantic'];review=json.loads(w['review_canonical_json']);source=json.loads(w['candidate_source_json'])
    if change=='source_hash':review['candidate_source_sha256']='0'*64
    if change=='source_pair':source['result']['candidate_pairs'][0]['links']=['bad','link1']
    if change=='source_model':source['result']['identity']['urdf_sha256']='0'*64
    if change=='source_geometry':review['source_geometry_sha256']='0'*64
    if change=='category':review['selected_categories']=['Adjacent','Default','Never']
    if change=='missing_pair':source['result']['candidate_pairs'].pop()
    if change=='review_claim':review['pair_reviews'][0]['decision']='disabled_model_only'
    text=canonical(source) if change in ['source_pair','source_model','missing_pair'] else w['candidate_source_json']
    if change in ['source_pair','source_model','missing_pair']:review['candidate_source_sha256']=sha256(text.encode()).hexdigest()
    with pytest.raises(CollisionError):SemanticConfig.from_xml(w['srdf_xml'],robot,review,allow_candidate=True,candidate_source_text=text)


@pytest.mark.parametrize('valid,status,expected',[(True,'candidate_baseline_unverified','RESEARCH_CANDIDATE'),(False,'candidate_baseline_unverified','REJECTED'),(None,'candidate_baseline_unverified','UNVERIFIED'),(True,'model_only_partial','VALID_MODEL_ONLY')])
def test_explicit_decision_cannot_approve_unverified_model(valid,status,expected):
    r=semantic_configuration_decision(valid,status)
    assert r['status']==expected and not r['planning_approved'] and not r['hardware_validated']
    assert r['legacy_fields_do_not_drive_this_decision']==['final_decision','model_scope_collision_free']


def test_real_native_three_semantics_known_cases_and_geometry_unchanged(robot):
    strict=SemanticConfig.load(ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf',robot)
    candidate=SemanticConfig.load(CANDIDATE,robot,allow_candidate=True)
    s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');arrays,_=load_results(ARTIFACTS / 'workspace_n100_seed42',model=robot)
    with Evaluator(robot,s,None,os.environ['C1_BACKEND'],semantic=strict) as a,Evaluator(robot,s,None,os.environ['C1_BACKEND'],semantic=candidate) as b:
        for index,q in [('zero',[0]*5),(8,arrays['q'][8]),(19,arrays['q'][19]),(73,arrays['q'][73])]:
            old=a.evaluate(np.asarray(q).reshape(1,5))[0];old_native=deepcopy(a.last_native)
            new=b.evaluate(np.asarray(q).reshape(1,5))[0];new_native=b.last_native
            assert old['raw_self_collision']==new['raw_self_collision'] and old['environment_collision']==new['environment_collision']
            for name in ['link_transforms_base','T_base_to_ee','joint_positions_used','colliding_pairs']:assert old_native[name]==new_native[name]
            assert a.ready['identity']['robot_geometry_sha256']==b.ready['identity']['robot_geometry_sha256']
            assert a.ready['identity']['srdf_sha256']!=b.ready['identity']['srdf_sha256']
            assert a.ready['identity']['semantic_acm_sha256']!=b.ready['identity']['semantic_acm_sha256']
            assert new['configuration_decision']['usage']=='research_only' and not new['configuration_decision']['planning_approved']
            if index=='zero':
                assert old['semantic_self_collision'] and not new['semantic_self_collision']
                assert new['model_configuration_valid'] is True and new['semantic_decision']=='RESEARCH_CANDIDATE'
                assert new['final_decision']=='COLLIDING' # explicit legacy meaning retained
            if index in [8,73]:assert new['environment_collision'] and new['semantic_decision']=='REJECTED'
            if index==19:assert ['back_shell_black_link','link3'] in new['semantic_blocking_pairs']
        # Query order does not install a global matrix or change the strict session.
        assert a.evaluate([[0]*5])[0]['semantic_self_collision']


@pytest.mark.parametrize('change',['opt_in','status','never','source_geometry','source_hash'])
def test_native_enforces_candidate_rules_when_python_is_bypassed(robot,change):
    c=SemanticConfig.load(CANDIDATE,robot,allow_candidate=True);w=c.wire();review=json.loads(w['semantic']['review_canonical_json'])
    if change=='opt_in':w['semantic'].pop('allow_candidate_baseline')
    if change=='status':review['status']='model_only_partial'
    if change=='never':
        source=json.loads(w['semantic']['candidate_source_json'])['result'];pair=next(p['links'] for p in source['candidate_pairs'] if p['candidate_class']=='Never')
        root=ET.fromstring(w['semantic']['srdf_xml']);ET.SubElement(root,'disable_collisions',link1=pair[0],link2=pair[1],reason='forbidden Never')
        xml=ET.tostring(root,encoding='unicode');w['semantic']['srdf_xml']=xml;w['semantic']['srdf_sha256']=sha256(xml.encode()).hexdigest();w['semantic']['disabled_pairs']=sorted(w['semantic']['disabled_pairs']+[pair]);review.update(srdf_sha256=w['semantic']['srdf_sha256'],disabled_pairs=w['semantic']['disabled_pairs']);review['pair_reviews'].append({'links':pair,'candidate_class':'Never','decision':'candidate_disabled_unverified','evidence':'must be rejected'})
    if change=='source_geometry':review['source_geometry_sha256']='0'*64
    if change=='source_hash':review['candidate_source_sha256']='0'*64
    text=canonical(review);w['semantic']['review_canonical_json']=text;w['semantic']['review_sha256']=sha256(text.encode()).hexdigest()
    with CollisionClient(os.environ['C1_BACKEND']) as native:
        with pytest.raises(CollisionError):native._exchange({'op':'INIT',**make_init(robot,load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')),**w})


def test_candidate_builder_refuses_overwrite_and_is_reproducible(robot,tmp_path):
    from workspace_analysis.collision.candidate_baseline import build_candidate_baseline
    strict=ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf';source=CANDIDATE.parent/'candidate_source.json';out=tmp_path/'candidate'
    config=build_candidate_baseline(robot,strict,source,out)
    assert config.wire()['semantic']['srdf_sha256']==SemanticConfig.load(CANDIDATE,robot,allow_candidate=True).wire()['semantic']['srdf_sha256']
    with pytest.raises(FileExistsError):build_candidate_baseline(robot,strict,source,out)


def test_invalid_configuration_keeps_explicit_error_decision(robot):
    candidate=SemanticConfig.load(CANDIDATE,robot,allow_candidate=True)
    scene=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    with Evaluator(robot,scene,None,os.environ['C1_BACKEND'],semantic=candidate) as evaluator:
        r=evaluator.evaluate([[10,0,0,0,0]])[0]
    assert r['configuration_decision']['status']=='ERROR' and r['model_configuration_valid'] is None
    assert not r['configuration_decision']['planning_approved'] and r['semantic_decision']=='ERROR'


def test_vendor_source_conventions_are_evidence_not_new_limits(robot):
    from workspace_analysis.experiments.assembly_semantics import audit_system_source
    result=audit_system_source(robot,SYSTEM_SOURCE)
    rows={r['joint']:r for r in result['angle_conventions']}
    assert rows['joint2']['source_mapping'][1]=={'pulse':500,'dh_degrees':-90.,'controller_jointstate_degrees':0.}
    assert rows['joint1']['source_mapping'][0]=={'pulse':0,'dh_degrees':-120.,'controller_jointstate_degrees':120.}
    assert result['normal_reference_status']=='PENDING' and all(r['limits_replaced'] is False for r in rows.values())
    meshes={r['link']:r for r in result['mesh_topology']}
    assert {k:r['triangles'] for k,r in meshes.items()}=={'back_shell_green_link':1969,'link1':1716,'link2':1144,'servo_link1':405}
    assert all(r['volume_validated'] is False for r in meshes.values())


def test_strict_old_reply_status_compatibility_does_not_enable_candidates(robot):
    from workspace_analysis.collision.semantic import check_semantic_query
    strict=SemanticConfig.load(ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf',robot)
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        ready=c.initialize(robot,load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml'),semantic=strict)
        row=c.query(robot,[0]*5)
    row.pop('semantic_review_status') # archived C3 response shape
    pairs=ready['audit']['semantic']['actual_disabled_pairs']
    check_semantic_query(row,pairs,'model_only_partial')
    with pytest.raises(CollisionError):check_semantic_query(row,pairs,'candidate_baseline_unverified')
