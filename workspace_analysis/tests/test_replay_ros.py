"""Actual DDS/RSP/TF/Qt and full installed launch; no robot driver entry points."""
import os
from pathlib import Path
import subprocess
import sys
import json
from test_resources import ARTIFACTS

ROOT=Path(__file__).resolve().parents[1]


def probe(mode,tmp_path,domain):
    env=dict(os.environ);env.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(domain),PYTHONDONTWRITEBYTECODE='1',
        QT_QPA_PLATFORM='offscreen',ROS_LOG_DIR=str(tmp_path/'ros_logs'),ROS_HOME=str(tmp_path/'ros_home'),
        WORKSPACE_ARTIFACT_ROOT=str(ARTIFACTS))
    env['PYTHONPATH']=os.pathsep.join([str(ROOT/'ros2/jetrover_idc_visualization'),str(ROOT/'src'),str(ROOT/'tests'),str(ROOT),env.get('PYTHONPATH','')])
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
