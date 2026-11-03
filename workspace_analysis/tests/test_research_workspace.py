"""Research decision statistics, frozen indices and actual FCL review integration."""
from copy import deepcopy
import json,os
from pathlib import Path
import numpy as np
import pytest
from workspace_analysis.experiments.research_workspace import semantic_masks,summarize_semantic,compare_rows,check_input_rows,representative_indices,run_research_workspace,verify_research_workspace
from workspace_analysis.experiments.assembly_validation import _number_expression,recorded_pose_candidates,pair_review
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.collision.semantic import SemanticConfig
from workspace_analysis.collision.evaluator import Evaluator
from workspace_analysis.workspace.results import load_results
ROOT=Path(__file__).resolve().parents[1]
CANDIDATE=ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf'


def row(index=0,passed=False,self_collision=True,environment=False,error=False):
    return {'sample_index':index,'q':[0.]*5,'ee_position':[0.]*3,'ee_orientation':np.eye(3).tolist(),
        'raw_self_collision':True,'semantic_self_collision':self_collision,'environment_collision':environment,
        'raw_collision_pairs':[{'kind':'self','first':'a','second':'b'}],
        'semantic_blocking_pairs':[['a','b']] if self_collision else [],'model_configuration_valid':passed,
        'configuration_decision':{'status':'ERROR' if error else 'RESEARCH_CANDIDATE' if passed else 'REJECTED','planning_approved':False,'hardware_validated':False},
        'semantic_decision':'ERROR' if error else 'RESEARCH_CANDIDATE' if passed else 'REJECTED','query_time':.01,
        'pairs_may_be_truncated':False,'semantic_pairs_may_be_truncated':False}


def test_disjoint_decisions_do_not_double_count_overlapping_collision_causes():
    rows=[row(0,True,False),row(1,False,True,True),row(2,False,False,True),row(3,False,False,False),row(4,None,False),row(5,None,False,error=True)]
    s=summarize_semantic(rows)
    assert (s['model_pass'],s['rejected'],s['unknown'],s['error'])==(1,3,1,1)
    assert s['semantic_self_collision']==1 and s['environment_collision']==2 and s['both']==1
    assert s['clearance_only_rejected']==1 and s['model_pass']+s['rejected']+s['unknown']+s['error']==6
    assert s['usage']=='RESEARCH_ONLY' and not s['planning_approved']


@pytest.mark.parametrize('change',['index','q','environment','raw_pair','new_pair','truncated','unknown'])
def test_paired_comparison_fails_on_unexplained_or_incomplete_changes(change):
    a=row();b=row(passed=True,self_collision=False)
    if change=='index':b['sample_index']=1
    if change=='q':b['q'][0]=.1
    if change=='environment':b['environment_collision']=True
    if change=='raw_pair':b['raw_collision_pairs']=[]
    if change=='new_pair':b['semantic_blocking_pairs']=[['c','d']]
    if change=='truncated':b['semantic_pairs_may_be_truncated']=True
    if change=='unknown':b['model_configuration_valid']=None
    with pytest.raises(ValueError):compare_rows([a],[b],[],[['a','b']])


def test_pair_recovery_records_exact_indices_and_changed_pairs():
    result=compare_rows([row(19)],[row(19,True,False)],[],[['a','b']])[0]
    assert result['sample_index']==19 and result['recovered_research_candidate'] and result['removed_pairs']==[['a','b']]
    assert result['remaining_pairs']==[] and result['usage']=='RESEARCH_ONLY'


def test_candidate_selection_is_deterministic_without_resampling():
    rows=[row(0,True,False),row(1,True,False),row(2,False,True,True)]
    for r in rows:r['semantic_clearance']={'native':{'self':{'separation_m':float(r['sample_index'])}}}
    a={'position':np.array([[0.,0,0],[1.,0,0],[2.,0,0]])}
    assert representative_indices(a,rows)==[('candidate_first',0),('candidate_largest_self_separation',1),('environment_rejected',2)]


def test_read_only_reference_records_are_not_upgraded_to_normal_truth():
    m=load_model(ROOT/'config/workspace.yaml');result=recorded_pose_candidates(ROOT,m,ROOT.parents[1]/'system_source')
    assert result['candidates'] and result['selected_trusted_normal_pose'] is None and result['status']=='PENDING'
    ready=next(r for r in result['candidates'] if r['name']=='CABLE_HOLD_READY')
    assert ready['urdf_joint_limits_valid'] and ready['q_controller_jointstate_rad'][1]<0
    assert ready['normal_reference_status']=='PENDING' and not ready['physical_calibration_verified']
    import ast
    with pytest.raises(ValueError):_number_expression(ast.parse('__import__("os")',mode='eval').body)


def test_all29_relations_have_separate_kinematic_and_mechanical_status():
    m=load_model(ROOT/'config/workspace.yaml');s=SemanticConfig.load(ROOT/'config/collision/semantic/jetrover_semantic_v0.srdf',m);c=SemanticConfig.load(CANDIDATE,m,allow_candidate=True)
    review=pair_review(m,s,c)
    assert len(review['entries'])==29 and all(r['whole_pair_physical_approval']=='PENDING' for r in review['entries'])
    assert sum(r['strict_disabled'] for r in review['entries'])==5
    joint3=next(r for r in review['entries'] if r['links']==['link2','link3'])
    assert joint3['risk']=='HIGH_PRIORITY_STATE_DEPENDENT_JOINT_INTERFACE' and joint3['kinematic_relation_confirmed']


def test_real_evaluator_semantic_stats_use_new_fields_not_legacy_raw():
    m=load_model(ROOT/'config/workspace.yaml');s=load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml');c=SemanticConfig.load(CANDIDATE,m,allow_candidate=True)
    arrays,_=load_results(ROOT/'results/workspace_n100_seed42',model=m)
    with Evaluator(m,s,None,os.environ['C1_BACKEND'],semantic=c) as e:
        values=e.evaluate(arrays['q'][:4]);check_input_rows({k:v[:4] for k,v in arrays.items()},values)
    summary=summarize_semantic(values)
    assert summary['total']==4 and all(r['model_scope_collision_free'] is False for r in values)
    assert summary['model_pass']==sum(r['model_configuration_valid'] is True for r in values)
    assert all('semantic_pairs_may_be_truncated' in r for r in values)
    bad=deepcopy(values);bad[0]['q'][0]+=.01
    with pytest.raises(ValueError):check_input_rows({k:v[:4] for k,v in arrays.items()},bad)


def test_existing_output_and_large_dataset_are_rejected_before_evaluation(tmp_path):
    m=load_model(ROOT/'config/workspace.yaml');out=tmp_path/'exists';out.mkdir()
    with pytest.raises(ValueError,match='existing'):run_research_workspace(ROOT,os.environ['C1_BACKEND'],out,[],source=ROOT.parents[1]/'system_source',candidate_srdf=CANDIDATE,allow_candidate=True)
    with pytest.raises(ValueError,match='1,000'):run_research_workspace(ROOT,os.environ['C1_BACKEND'],tmp_path/'large',[ROOT/'results/workspace_n10000_seed42'],source=ROOT.parents[1]/'system_source',candidate_srdf=CANDIDATE,allow_candidate=True)
    assert not (tmp_path/'large').exists()
