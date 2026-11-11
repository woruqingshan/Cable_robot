"""Artifact-only real T3/T4 replay; original frame assertions retained."""

from test_resources import ARTIFACTS
import pytest
from test_resources import PROJECT_ROOT as ROOT
from test_viewer_replay import parse_saved_html,runtime

@pytest.mark.parametrize('name',['q0','q1'])
def test_saved_t4_native_data_all_26_frames_slider_and_play(name):
    data,traces,layout=parse_saved_html(ARTIFACTS / f'task_approach/p22_20261104_03/figures/{name}_approach.html')
    assert len(data['states'])==26
    assert any('name' not in t for t in traces),'regression fixture must retain actual nameless traces'
    with runtime(data,traces,layout) as js:
        for i in range(26):
            js.evaluate(f'selectFrame({i});');assert js.json(f'checkFrame({i})')['meshes']==24
        js.evaluate('getPlay().onclick();');js.json('checkFrame(0)')
        for i in range(1,26):
            js.evaluate('tick();');result=js.json(f'checkFrame({i})');assert not result['only_endpoint_is_task_constrained']
        assert js.json('intervals.size')==0


def test_saved_t3_replay_keeps_endpoint_only_semantics_and_mesh_updates():
    data,traces,layout=parse_saved_html(ARTIFACTS / 'task_planning/p22_20261104_02/figures/q0_linear.html')
    assert data.get('replay_context') is None
    with runtime(data,traces,layout) as js:
        for i in [0,len(data['states'])//2,len(data['states'])-1]:
            js.evaluate(f'selectFrame({i});');result=js.json(f'checkFrame({i})')
            assert result['meshes']==24 and result['only_endpoint_is_task_constrained']

