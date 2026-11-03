"""Real native scope positives/negatives; no contact-list inference or subtree blanket."""
from copy import deepcopy
from hashlib import sha256
import json,os
from pathlib import Path
import numpy as np
import pytest,yaml
from workspace_analysis.robot.model import load_model,model_from_xml
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.collision.assembly import assembly_topology
from workspace_analysis.collision.geometry_audit import topology
from workspace_analysis.collision.policy import load_policy,canonical_policy
from workspace_analysis.collision.scope import load_scope,canonical_scope
from workspace_analysis.collision.client import CollisionClient
from workspace_analysis.collision.evaluator import Evaluator
from workspace_analysis.collision.protocol import CollisionError
from test_geometry_profile import scene_fixture
ROOT=Path(__file__).resolve().parents[1]


def fixture(external=None):
    xml='<robot name="assembly_fixture"><link name="base_link"><collision>'
    if external=='base':xml+='<origin xyz="1 0 .5"/>'
    xml+='<geometry><box size=".2 .2 .2"/></geometry></collision></link>'
    parent='base_link'
    for i in range(5):
        name='arm_link' if i==4 else 'segment'+str(i)
        xml+=f'<link name="{name}">'
        if i==4:xml+='<collision>'+('<origin xyz="0 0 .5"/>' if external=='arm' else '')+'<geometry><box size=".1 .1 .1"/></geometry></collision>'
        xml+=f'</link><joint name="joint{i+1}" type="revolute"><parent link="{parent}"/><child link="{name}"/><origin xyz="'+('1 0 0' if i==0 else '0 0 0')+'"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>';parent=name
    xml+='''<link name="gripper_link"><collision><geometry><box size=".2 .2 .2"/></geometry></collision></link><joint name="gripper_joint" type="fixed"><parent link="arm_link"/><child link="gripper_link"/><origin xyz="0 0 .5"/></joint><link name="r_link">'''
    if external=='undeclared_gripper':xml+='<collision><geometry><box size=".2 .2 .2"/></geometry></collision>'
    xml+='''</link><joint name="r_joint" type="revolute"><parent link="gripper_link"/><child link="r_link"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint><link name="r_in_link"><collision><geometry><box size=".2 .2 .2"/></geometry></collision></link><joint name="r_in_joint" type="revolute"><parent link="gripper_link"/><child link="r_in_link"/><origin xyz=".05 0 0"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/><mimic joint="r_joint" multiplier="-1" offset="0"/></joint></robot>'''
    m=model_from_xml(xml,ee_frame='arm_link',auxiliary_positions={'r_joint':0})
    data=yaml.safe_load((ROOT/'config/collision/baseline.yaml').read_text());data.update(model_sha256=m.metadata()['expanded_urdf_sha256'],allowed_pairs=[],unresolved_pairs=[]);policy=canonical_policy(data,m)
    data=yaml.safe_load((ROOT/'config/collision/task_scope_v0.yaml').read_text());data.update(source_model_sha256=m.metadata()['expanded_urdf_sha256'],existing_acm_policy_sha256=policy[1],assembly_members=assembly_topology(m)['members'],internal_diagnostic_pairs=[{'links':['gripper_link','r_in_link'],'mechanism':'joint_bearing_candidate','urdf_joint_path':['r_in_joint'],'evidence':'synthetic intersecting boxes, explicit internal pair','unverified_factors':['synthetic scope does not prove physical safety']}])
    scope=canonical_scope(data,m,policy)
    objects=[]
    if external=='environment':objects=[{'id':'test_obstacle','type':'box','parent_frame':'base_link','pose':{'position_m':[1,0,.5],'quaternion_xyzw':[0,0,0,1]},'size_m':[.2,.2,.2]}]
    return m,scene_fixture(objects),policy,scope


def test_native_candidate_retains_full_collision_and_internal_diagnostic_and_reset():
    m,s,p,scope=fixture()
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,s);c.set_policy(*p);first=c.query(m,[0]*5)
        result=c.query(m,[0]*5,task_scope=scope)['task_scope']
        assert result['official_effective_self_collision'] and result['gripper_internal_collision']
        assert not result['task_scope_blocking_self_collision'] and result['zero_clearance_candidate']
        assert result['gripper_internal_pairs']==[['gripper_link','r_in_link']] and result['gripper_internal_contacts']
        assert not result['contact_surface_complete']
        last=c.query(m,[0]*5);assert 'task_scope' not in last
        for key in ['effective_self_collision','colliding_pairs','clearance','joint_positions_used','T_base_to_ee','link_transforms_base']:assert first[key]==last[key]
        changed=json.loads(scope[0]);changed['id']+='review';other=canonical_scope(changed,m,p)
        assert other[1]!=scope[1] and c.query(m,[0]*5,task_scope=other)['task_scope']['scope_sha256']==other[1]
    with Evaluator(m,s,p,os.environ['C1_BACKEND'],task_scope=scope,gripper=.3) as e:
        row=e.evaluate([[0]*5])[0]
    assert row['task_scope_candidate'] is True and row['final_decision']=='COLLIDING'
    assert row['model_scope_collision_free'] is False and row['hardware_safe'] is None


@pytest.mark.parametrize('external',['base','arm','environment','undeclared_gripper'])
def test_external_collisions_and_unlisted_internal_pairs_remain_rejected(external):
    m,s,p,scope=fixture(external)
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,s);c.set_policy(*p)
        r=c.query(m,[0]*5,task_scope=scope,geometry_audit={'max_contacts_per_pair':1,'max_contacts':1})['task_scope']
        assert not r['zero_clearance_candidate'] and r['gripper_internal_collision']
        assert r['environment_collision'] if external=='environment' else r['task_scope_blocking_self_collision']
        assert r['candidate_computed_from_fresh_boolean_queries']


@pytest.mark.parametrize('change',['model','policy','pair','path','members','verified'])
def test_illegal_scope_rejected_in_python_and_native(change):
    m,s,p,scope=fixture();d=json.loads(scope[0])
    if change=='model':d['source_model_sha256']='0'*64
    if change=='policy':d['existing_acm_policy_sha256']='0'*64
    if change=='pair':d['internal_diagnostic_pairs'][0]['links']=['arm_link','gripper_link']
    if change=='path':d['internal_diagnostic_pairs'][0]['urdf_joint_path']=['joint1']
    if change=='members':d['assembly_members'].append('arm_link')
    if change=='verified':d['verification_status']='validated'
    with pytest.raises(CollisionError):canonical_scope(d,m,p)
    text=json.dumps(d,sort_keys=True,separators=(',',':'))
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,s);c.set_policy(*p)
        with pytest.raises(CollisionError):c.query(m,[0]*5,task_scope=(text,sha256(text.encode()).hexdigest()))


def test_scene_binding_mismatch_rejected():
    m,s,p,scope=fixture()
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,s);c.set_policy(*p);context={**c.identity,'scene_sha256':'0'*64}
        with pytest.raises(CollisionError):c._exchange({'op':'QUERY','joint_positions':m.joint_positions([0]*5),'task_scope_canonical_json':scope[0],'task_scope_sha256':scope[1],'task_scope_context':context})


def test_jetrover_scope_topology_gripper_scans_and_real_rejections():
    m=load_model(ROOT/'config/workspace.yaml');p=load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m);scope=load_scope(ROOT/'config/collision/task_scope_v0.yaml',m,p)
    data=json.loads(scope[0]);assert len(data['internal_diagnostic_pairs'])==7 and len(data['assembly_members'])==7
    s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    with np.load(ROOT/'results/workspace_n100_seed42.npz') as f:qs=f['q'].copy()
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,s);c.set_policy(*p)
        for opening in [-1.5,0,.3,.9,1.5]:
            for index in [8,19,73]:
                plain=c.query(m,qs[index],auxiliary_positions={'r_joint':opening});scoped=c.query(m,qs[index],auxiliary_positions={'r_joint':opening},task_scope=scope)
                for key in ['colliding_pairs','effective_self_pairs','T_base_to_ee','environment_collision','clearance']:assert plain[key]==scoped[key]
                r=scoped['task_scope'];assert r['gripper_internal_collision'] and r['gripper_internal_verification']=='UNVERIFIED_MECHANISM'
                if index in [8,73]:assert r['environment_collision']
                if index==19:assert ['back_shell_black_link','link3'] in r['task_scope_blocking_pairs']
                for j in m.joints.values():
                    if j.mimic_joint:assert scoped['joint_positions_used'][j.name]==pytest.approx(scoped['joint_positions_used'][j.mimic_joint]*j.multiplier+j.offset)


def test_diagnostic_geometry_not_silently_used_for_task_scope(tmp_path):
    from workspace_analysis.collision.geometry_profile import load_geometry_profile
    m=load_model(ROOT/'config/workspace.yaml');p=load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m);scope=load_scope(ROOT/'config/collision/task_scope_v0.yaml',m,p)
    profile=load_geometry_profile(ROOT/'config/collision/geometry_profiles/refined_assembly_v0.yaml',m,asset_dir=tmp_path/'assets')
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml'),geometry_profile=profile);c.set_policy(*p)
        with pytest.raises(CollisionError,match='official full'):c.query(m,[0]*5,task_scope=scope)


def test_scope_viewer_real_contacts_have_distinct_diagnostic_and_blocking_layers(tmp_path):
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.collision import build_collision_figure,export_collision_viewer
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');p=load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m);scope=load_scope(ROOT/'config/collision/task_scope_v0.yaml',m,p)
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        ready=c.initialize(m,s);c.set_policy(*p);r=c.query(m,[0]*5,task_scope=scope,auxiliary_positions={'r_joint':.3});r.update(q=[0]*5,sample_index=0)
        fig=build_collision_figure(s,robot_surfaces(m,ready,r),r)
    meshes=[t for t in fig.data if (t.meta or {}).get('role')=='robot_collision_mesh'];assert len(meshes)==24
    internal=next(t for t in fig.data if (t.meta or {}).get('role')=='scope_internal_contacts')
    np.testing.assert_array_equal(np.column_stack([internal.x,internal.y,internal.z]),[p['position_base_m'] for p in r['task_scope']['gripper_internal_contacts']])
    output=tmp_path/'scope.html';export_collision_viewer(fig,r,output)
    assert 'Internal Diagnostic (UNVERIFIED)' in output.read_text() and 'Task Blocking Self' in output.read_text()
    with pytest.raises(ValueError):export_collision_viewer(fig,r,output)


def test_d2_original_arrays_statistical_conservation_and_no_overwrite(tmp_path):
    from workspace_analysis.experiments.task_scoped_workspace import run_task_pilot
    out=tmp_path/'d2';s=run_task_pilot(ROOT,os.environ['C1_BACKEND'],ROOT/'results/workspace_n100_seed42',out)
    with np.load(ROOT/'results/workspace_n100_seed42.npz',allow_pickle=False) as a,np.load(out/'workspace_results.npz',allow_pickle=False) as b:
        for key in a.files:np.testing.assert_array_equal(a[key],b[key])
    b=s['B_task_scope'];assert b['candidates']+b['rejected']+b['unknown']+b['errors']==100
    assert b['full_effective_self']==100 and b['gripper_internal']==100 and b['environment']==2
    with pytest.raises(ValueError):run_task_pilot(ROOT,os.environ['C1_BACKEND'],ROOT/'results/workspace_n100_seed42',out)
