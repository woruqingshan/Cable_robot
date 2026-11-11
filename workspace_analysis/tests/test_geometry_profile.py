"""Native mesh fixtures exercise local changes and preserved dangerous collisions."""

from test_resources import ARTIFACTS
from dataclasses import replace
import json,os
from pathlib import Path
import numpy as np
import pytest,yaml
from workspace_analysis.robot.model import load_model,model_from_xml
from workspace_analysis.scene.schema import load_scene,parse_scene
from workspace_analysis.collision.geometry_profile import load_geometry_profile,triangle_mask,_stl_bytes,GeometryProfile,canonical
from workspace_analysis.collision.assets import read_stl,robot_surfaces
from workspace_analysis.collision.client import CollisionClient
from workspace_analysis.collision.policy import load_policy
from workspace_analysis.collision.protocol import CollisionError
ROOT=Path(__file__).resolve().parents[1]


def scene_fixture(objects=()):
    d=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml').to_dict()
    for key in ['frames','entities','ports']:d['scene'][key]=[]
    d['scene']['collision_objects']=list(objects)
    return parse_scene(json.dumps(d))


def cube(center,size):
    v=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]],float)*size/2+center
    f=np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    return v,f


def fixture(tmp_path,third=False):
    files=[]
    for name,size in [('base',.2),('tip',.1)]:
        shift=.075 if name=='base' else 0
        a,fa=cube([shift,0,0],size);b,fb=cube([1+shift,0,0],size)
        p=tmp_path/(name+'.stl');p.write_bytes(_stl_bytes(np.vstack([a,b]),np.vstack([fa,fb+8])));files.append(p.as_uri())
    xml=f'''<robot name="local_fixture"><link name="base_link"><collision><geometry><mesh filename="{files[0]}"/></geometry></collision></link><link name="tip"><collision><geometry><mesh filename="{files[1]}"/></geometry></collision></link><joint name="j" type="revolute"><parent link="base_link"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint>'''
    if third:xml+='<link name="third"><collision><geometry><box size="0.12 0.12 0.12"/></geometry></collision></link><joint name="f" type="fixed"><parent link="base_link"/><child link="third"/><origin xyz="0 1 0"/></joint>'
    xml+='</robot>';m=model_from_xml(xml,ee_frame='tip',joint_order=('j',))
    config={'version':1,'id':'synthetic_local','method':'pair_local_triangle_exclusion','review_status':'diagnostic_only','scope':'synthetic numeric truth only','edits':[{'link':'tip','joint':'j','radius_m':.2,'axial_m':[-.2,.2],'reason':'Exclude separate proximal cube; preserve distal cube'}],'target_pairs':[['base_link','tip']]}
    path=tmp_path/'profile.yaml';path.write_text(yaml.safe_dump(config));return m,load_geometry_profile(path,m,asset_dir=tmp_path/'assets')


def test_triangle_cylinder_keeps_boundary_and_outside_faces():
    v=np.array([[0,0,0],[.1,0,0],[0,.1,0],[2,0,0]])
    f=np.array([[0,1,2],[0,1,3]])
    np.testing.assert_array_equal(triangle_mask(v,f,[0,0,1],.2,[-1,1]),[False,True])
    np.testing.assert_array_equal(triangle_mask(v,f,[0,0,1],.1,[-1,1]),[True,True])


def test_local_model_removes_only_proximal_interference_and_keeps_distal_positive(tmp_path):
    m,p=fixture(tmp_path);binary=os.environ['C1_BACKEND'];s=scene_fixture()
    with CollisionClient(binary) as official,CollisionClient(binary) as refined:
        a=official.initialize(m,s);b=refined.initialize(m,s,geometry_profile=p)
        assert b['identity']['source_geometry_sha256']==a['identity']['robot_geometry_sha256']
        assert b['identity']['robot_geometry_sha256']!=a['identity']['robot_geometry_sha256']
        for angle in [.9,1.2,np.pi/2]:
            x=official.query(m,[angle]);y=refined.query(m,[angle])
            assert x['raw_self_collision'] and not y['raw_self_collision']
            np.testing.assert_array_equal(x['T_base_to_ee'],y['T_base_to_ee'])
        y=refined.query(m,[np.pi/2],geometry_audit={'max_contacts_per_pair':8,'max_contacts':100,'distance_pairs':[['base_link','tip']]});assert y['clearance']['self']['separation_m']==pytest.approx(.85,abs=1e-6)
        assert y['pair_geometry_distances'][0]['separation_m']==pytest.approx(.85,abs=1e-6)
        assert y['official_pair_geometry_distances'][0]['collision'] and y['official_pair_geometry_distances'][0]['separation_m']==0
        assert refined.query(m,[0])['raw_self_collision'] # same target pair, genuine distal overlap retained
    with pytest.raises(FileExistsError):load_geometry_profile(tmp_path/'profile.yaml',m,asset_dir=tmp_path/'assets')


def test_nonadjacent_positive_and_removed_region_environment_coverage(tmp_path):
    m,p=fixture(tmp_path,third=True);binary=os.environ['C1_BACKEND']
    obstacle={'id':'world','type':'box','parent_frame':'base_link','pose':{'position_m':[0,0,0],'quaternion_xyzw':[0,0,0,1]},'size_m':[.12,.12,.12]}
    with CollisionClient(binary) as c:
        c.initialize(m,scene_fixture([obstacle]),geometry_profile=p);r=c.query(m,[np.pi/2])
        assert any(set([x['first'],x['second']])=={'third','tip'} for x in r['colliding_pairs'])
        assert any(x['kind']=='environment' and 'tip' in [x['first'],x['second']] for x in r['colliding_pairs'])
        assert r['effective_self_collision'] and r['environment_collision']


def test_geometry_audit_bounds_locals_and_partial_surface(tmp_path):
    m,p=fixture(tmp_path);binary=os.environ['C1_BACKEND']
    with CollisionClient(binary) as c:
        c.initialize(m,scene_fixture());r=c.query(m,[.7],geometry_audit={'max_contacts_per_pair':4,'max_contacts':20})
        assert r['contact_collection']['complete_contact_surface'] is False
        assert r['contact_collection']['pairs_at_limit']
        for contact in r['contacts_base']:
            for side in ['first','second']:
                t=np.array(r['link_transforms_base'][contact[side]]);local=contact['position_'+side+'_link_m']
                np.testing.assert_allclose(t[:3,:3]@local+t[:3,3],contact['position_base_m'],atol=1e-12)
    with CollisionClient(binary) as c:
        c.initialize(m,scene_fixture())
        with pytest.raises(CollisionError):c.query(m,[0],geometry_audit={'max_contacts_per_pair':513,'max_contacts':1000})


def test_native_rejects_changed_kinematics_and_profile_hash(tmp_path):
    m,p=fixture(tmp_path);binary=os.environ['C1_BACKEND']
    changed=replace(p,effective_xml=p.effective_xml.replace('lower="-3.14"','lower="-2"'))
    with CollisionClient(binary) as c:
        with pytest.raises(CollisionError,match='limits'):c.initialize(m,scene_fixture(),geometry_profile=changed)
    changed=replace(p,sha256='0'*64)
    with CollisionClient(binary) as c:
        with pytest.raises(CollisionError,match='hash'):c.initialize(m,scene_fixture(),geometry_profile=changed)


def test_jetrover_profiles_fk_mimic_environment_and_non_target_invariance(tmp_path):
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    p=load_geometry_profile(ROOT/'config/collision/geometry_profiles/refined_assembly_v0.yaml',m,asset_dir=tmp_path/'assets')
    o=load_geometry_profile(ROOT/'config/collision/geometry_profiles/official_full.yaml',m)
    targets={tuple(sorted(x)) for x in json.loads(p.canonical_json)['target_pairs']}
    with CollisionClient(os.environ['C1_BACKEND']) as a,CollisionClient(os.environ['C1_BACKEND']) as b:
        ra=a.initialize(m,s,geometry_profile=o);rb=b.initialize(m,s,geometry_profile=p)
        policy=load_policy(ROOT/'config/collision/assembly_policy_v1.yaml',m);a.set_policy(*policy);b.set_policy(*policy)
        with np.load(ARTIFACTS / 'workspace_n100_seed42.npz') as data:qs=[data['q'][8],data['q'][73],[.35,-.6,.8,-.4,.2]]
        for q in qs:
            for opening in [-.4,.3,.9]:
                x=a.query(m,q,auxiliary_positions={'r_joint':opening});y=b.query(m,q,auxiliary_positions={'r_joint':opening})
                assert x['joint_positions_used']==y['joint_positions_used']
                for link in x['link_transforms_base']:np.testing.assert_allclose(x['link_transforms_base'][link],y['link_transforms_base'][link],atol=1e-12,rtol=0)
                assert x['environment_collision']==y['environment_collision']
                assert x['clearance']['environment']==y['clearance']['environment']
                non_target=lambda r:sorted((z['kind'],*sorted([z['first'],z['second']])) for z in r['colliding_pairs'] if tuple(sorted([z['first'],z['second']])) not in targets or z['kind']=='environment')
                assert non_target(x)==non_target(y)
                assert len(robot_surfaces(m,rb,y))==24


def test_nonzero_origin_scale_retained_vertices_and_native_render(tmp_path):
    from workspace_analysis.collision.geometry_profile import effective_render_data
    m,_=fixture(tmp_path)
    # Distinct nonzero collision origin and nonuniform scale, preserved by profile.
    xml=m.urdf_xml.replace('<link name="tip"><collision>','<link name="tip"><collision><origin xyz="0 0 .01" rpy="0 0 .3"/>').replace(f'filename="{(tmp_path/"tip.stl").as_uri()}"',f'filename="{(tmp_path/"tip.stl").as_uri()}" scale="1.2 .9 1.1"')
    m=model_from_xml(xml,ee_frame='tip',joint_order=('j',))
    p=load_geometry_profile(tmp_path/'profile.yaml',m,asset_dir=tmp_path/'transformed_assets')
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        ready=c.initialize(m,scene_fixture(),geometry_profile=p);row=c.query(m,[.4])
        surfaces=effective_render_data(m,ready,row);original=robot_surfaces(m,ready,row)
        derived=next(x for x in surfaces if x['id']=='tip');source=next(x for x in original if x['id']=='tip')
        # The retained distal cube is the last 12 source triangles, byte-exact vertices.
        np.testing.assert_allclose(derived['vertices'],source['vertices'][36:],atol=1e-12,rtol=0)
        assert len(derived['faces'])==12


def test_undeclared_mesh_change_and_incomplete_composition_rejected(tmp_path):
    m,p=fixture(tmp_path);base_uri=(tmp_path/'base.stl').as_uri();tip_uri=(tmp_path/'tip.stl').as_uri()
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        bad=replace(p,effective_xml=p.effective_xml.replace(base_uri,tip_uri))
        with pytest.raises(CollisionError,match='undeclared'):c.initialize(m,scene_fixture(),geometry_profile=bad)
    with CollisionClient(os.environ['C1_BACKEND']) as c:
        c.initialize(m,scene_fixture(),geometry_profile=p)
        with pytest.raises(CollisionError,match='nontruncated'):c.query(m,[0],geometry_audit={'max_contacts_per_pair':1,'max_contacts':1})


@pytest.mark.parametrize('script',['audit_geometry.py','evaluate_geometry_workspace.py'])
def test_new_cli_help(script):
    import subprocess,sys
    p=subprocess.run([sys.executable,'-B',str(ROOT/'scripts'/script),'--help'],capture_output=True,text=True)
    assert p.returncode==0 and '--backend' in p.stdout


def test_d1_paired_experiment_preserves_arrays_identity_and_robot_only(tmp_path):
    from workspace_analysis.experiments.geometry_workspace import run_comparison
    out=tmp_path/'comparison';summary=run_comparison(ROOT,os.environ['C1_BACKEND'],ARTIFACTS / 'workspace_n100_seed42',out)
    with np.load(ARTIFACTS / 'workspace_n100_seed42.npz',allow_pickle=False) as source,np.load(out/'workspace_results.npz',allow_pickle=False) as result:
        for key in source.files:np.testing.assert_array_equal(source[key],result[key])
    manifest=json.loads((out/'manifest.json').read_text())
    assert manifest['identities']['idc']['official']['geometry_profile']=='official_full'
    assert manifest['identities']['idc']['refined']['geometry_review_status']=='diagnostic_only'
    assert summary['idc']['official']['environment']==summary['idc']['refined']['environment']==2
    assert summary['robot_only']['official']['environment']==summary['robot_only']['refined']['environment']==0
    paired=summary['idc']['paired']
    assert len(paired['recovered_indices'])+len(paired['new_rejected_indices'])+paired['both_rejected_count']+paired['both_candidate_count']==100
    with pytest.raises(ValueError):run_comparison(ROOT,os.environ['C1_BACKEND'],ARTIFACTS / 'workspace_n100_seed42',out)
