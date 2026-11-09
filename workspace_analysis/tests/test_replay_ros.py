"""Actual DDS/RSP/TF/Qt and full installed launch; no robot driver entry points."""
import os
from pathlib import Path
import subprocess
import sys
import json
from test_resources import ARTIFACTS

ROOT=Path(__file__).resolve().parents[1]


def probe(mode,tmp_path,domain,*,benchmark=None,anchor=None):
    env=dict(os.environ);env.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(domain),PYTHONDONTWRITEBYTECODE='1',
        QT_QPA_PLATFORM='offscreen',ROS_LOG_DIR=str(tmp_path/'ros_logs'),ROS_HOME=str(tmp_path/'ros_home'),
        WORKSPACE_ARTIFACT_ROOT=str(ARTIFACTS))
    env['PYTHONPATH']=os.pathsep.join([str(ROOT/'ros2/jetrover_idc_visualization'),str(ROOT/'src'),str(ROOT/'tests'),str(ROOT),env.get('PYTHONPATH','')])
    if benchmark is not None:env['IDC_REPLAY_BENCHMARK']=str(benchmark)
    if anchor is not None:env['IDC_REPLAY_MANIFEST_SHA256']=anchor
    result=subprocess.run([sys.executable,'-B',str(ROOT/'tests/replay_ros_probe.py'),'--mode',mode,'--output',str(tmp_path/'probe')],
        cwd=ROOT,env=env,text=True,capture_output=True,timeout=120)
    assert result.returncode==0,result.stdout+'\n'+result.stderr+'\n'+(tmp_path/'probe/process.log').read_text()
    report=json.loads((tmp_path/'probe/report.json').read_text())
    assert report['status']=='PASS' and report['cleanup']['exit_code']==0 and not report['cleanup']['forced']
    return report


def test_real_jointstate_rsp_tf_scene_markers_and_clean_shutdown(tmp_path):
    report=probe('transport',tmp_path,91)
    assert len(report['cases'])==2 and report['max_ee_tf_error']<=1e-9


def test_real_qt_panel_selection_pause_resume_reset_and_protocol(tmp_path):
    report=probe('panel',tmp_path,92)
    assert report['qt_timer_responsive'] and (tmp_path/'probe/actual_qt_panel.png').is_file()
    assert report['failed_path_goal_visible_and_play_disabled']


def test_replay_explicitly_refuses_default_domain_and_nonloopback(monkeypatch):
    import pytest
    from jetrover_idc_visualization.node import require_isolated_ros
    from jetrover_idc_visualization.errors import ReplayError
    for domain,localhost in [('0','1'),('83','0'),('invalid','1')]:
        monkeypatch.setenv('ROS_DOMAIN_ID',domain);monkeypatch.setenv('ROS_LOCALHOST_ONLY',localhost)
        with pytest.raises(ReplayError):require_isolated_ros()
    monkeypatch.setenv('ROS_DOMAIN_ID','83');monkeypatch.setenv('ROS_LOCALHOST_ONLY','1');require_isolated_ros()


def test_installed_launch_actual_p12_p22_samples_mimic_tf_and_shutdown(tmp_path):
    report=probe('artifact',tmp_path,93)
    assert len(report['cases'])==4 and all(c['all_original_indices_seen'] for c in report['cases'])


def test_new_station_world_tf_fixed_cabinet_and_original_saved_samples(tmp_path):
    report=probe('world',tmp_path,94,
        benchmark=ARTIFACTS/'base_pose_benchmark/t52_r2_s1_20261104_01/stations/xm30/benchmark',
        anchor='d39077e52aef45339dd0b167ce1f12650f07301ef9dee34d100c58f58c39919f')
    assert report['world_base_tf_verified'] and report['base_link_single_parent']
    assert report['world_fixed_scene_markers_checked']==38 and report['maximum_world_marker_error']<1e-9
    assert report['base_pose']['x_m']==-.03 and all(c['all_original_indices_seen'] for c in report['cases'])


def test_restricted_d4_real_markers_workspace_and_replanned_npz(tmp_path):
    report=probe('restricted',tmp_path,95,
        benchmark=ARTIFACTS/'restricted_benchmark/t53_a_20261109_02/b0_D4_workspace_on',
        anchor='b0ee3d3c078f64852396187618f46c5f6151b9748249c3d9fb3c3a70e61ca423')
    assert report['scene_object_markers']==42
    assert report['workspace_boundary_marker_verified'] and report['base_link_single_parent']
    assert report['maximum_world_marker_error']<=1e-9 and report['max_ee_tf_error']<=1e-9
    assert report['cases'][0]['all_original_indices_seen'] and report['cases'][0]['samples_observed']==426
