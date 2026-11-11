"""Failures stay in denominators; paired optimization does not imply TCP smoothing."""
import pytest
from workspace_analysis.experiments.robustness_reporting import distribution,robustness_groups,paired_optimization


def row(path,success):
    return {'trial':path,'experiment':'main','case':'C1','start':'S0','seed':1,'mode':'conditional',
        'status':'VALID_DISCRETE_PATH' if success else 'PLANNER_TIMEOUT','termination':'SUCCESS' if success else 'TIME_LIMIT',
        'success':success,'search_time_s':6. if success else 20.,'joint_length_rad':2. if success else None,
        'tcp_length_m':.1 if success else None,'min_environment_clearance_m':.01 if success else None}


def test_descriptive_distributions_reject_nonfinite_and_keep_missing_counts():
    assert distribution([None,None])['n']==0
    assert distribution([1.,3.,None])['mean']==2.
    with pytest.raises(ValueError):distribution([float('nan')])


def test_failed_search_is_not_removed_from_seed_denominator():
    g=robustness_groups([row('a',True),row('b',False)])[0]
    assert g['scheduled']==2 and g['executed']==2 and g['accepted']==1 and g['fraction']==.5
    assert g['termination_counts']=={'SUCCESS':1,'TIME_LIMIT':1}
    assert g['observed_search_time_s']['n']==2 and g['successful_search_time_s']['n']==1
    assert g['successful_joint_length_rad']['n']==1


def test_paired_comparison_preserves_failed_raw_and_reports_tcp_clearance_regression():
    raw=[row('a',True),row('b',False)]
    optimized=[{**row('oa',True),'joint_length_rad':1.5,'tcp_length_m':.12,'min_environment_clearance_m':.005}]
    index=[{'raw_trial':'a','optimized_trial':'oa','status':'SHORTCUT_VALIDATED'},
        {'raw_trial':'b','optimized_trial':None,'status':'NOT_RUN_NO_VALID_RAW_PATH'}]
    p=paired_optimization(raw,optimized,index)
    assert len(p)==2 and p[0]['joint_reduction_fraction']==.25
    assert p[0]['tcp_reduction_fraction']<0 and p[0]['environment_clearance_change_m']<0
    assert p[1]['raw_search_success'] is False and p[1]['optimized_joint_length_rad'] is None
    optimized[0]['joint_length_rad']=2.1
    with pytest.raises(ValueError):paired_optimization(raw,optimized,index)


def test_home_audit_does_not_execute_vendor_code_or_invent_joint_home(tmp_path):
    import yaml
    from workspace_analysis.experiments.preparation_benchmark import audit_home
    vendor=tmp_path/'vendor';project=tmp_path/'project'
    folder=vendor/'driver/controller/config';folder.mkdir(parents=True)
    settings={'type':'action','action':{'action_name':'init'},'servo':{**{'id'+str(i):500 for i in range(1,6)},'duration':1}}
    (folder/'init_pose.yaml').write_text(yaml.safe_dump({'/**':{'ros__parameters':settings}}))
    script=vendor/'driver/controller/controller/init_pose.py';script.parent.mkdir(parents=True)
    script.write_text("raise AssertionError('vendor hardware code must not execute')")
    servo=vendor/'driver/servo_controller/config/servo_controller.yaml';servo.parent.mkdir(parents=True)
    servo.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'joint'+str(i):{'id':i,'init':500,'min':1000,'max':0} for i in range(1,6)}}}))
    report=audit_home(vendor,project)
    assert report['authoritative_home_q'] is None and not report['physical_home_verified']
    assert report['planning_reference']=='REFERENCE_START_TO_READY' and not report['vendor_code_executed']
    assert report['vendor_action_files']==[] and len(report['inactive_pulse_block'])==5


def test_new_cli_help_and_existing_output_guards_do_not_start_native(tmp_path):
    import subprocess,sys
    from workspace_analysis.experiments.robustness_benchmark import run_search
    from workspace_analysis.experiments.shortcut_benchmark import run_shortcuts
    from workspace_analysis.experiments.preparation_benchmark import run_preparation
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    for script in ('evaluate_robustness_benchmark.py','shortcut_robustness_paths.py','evaluate_preparation_motion.py'):
        result=subprocess.run([sys.executable,'-B',str(root/'scripts'/script),'--help'],cwd=tmp_path,text=True,capture_output=True)
        assert result.returncode==0 and '--allow-candidate-baseline' in result.stdout
    out=tmp_path/'existing';out.mkdir()
    with pytest.raises(FileExistsError):run_search(root,tmp_path,sys.executable,out,expected_source_manifest_sha256='x')
    with pytest.raises(FileExistsError):run_shortcuts(root,tmp_path,sys.executable,out,expected_search_manifest_sha256='x')
    with pytest.raises(FileExistsError):run_preparation(root,tmp_path,sys.executable,out,expected_source_manifest_sha256='x',system_source=tmp_path)
