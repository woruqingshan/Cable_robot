"""Execute exported replay JS, including nameless traces and async UI races."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from viewer_js_runtime import JavaScriptRuntime,HARNESS

ROOT=Path(__file__).resolve().parents[1]


def production_script():
    tree=ast.parse((ROOT/'src/workspace_analysis/viz/joint_path.py').read_text())
    scripts=[n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str) and 'const root=document.getElementById' in n.value]
    assert len(scripts)==1
    return scripts[0]


def runtime(data,traces,layout=None,script=None):
    engine=JavaScriptRuntime()
    harness=HARNESS.replace('INPUT_TRACES',json.dumps(traces,allow_nan=False)).replace('INPUT_LAYOUT',json.dumps(layout or {'scene':{},'meta':{}},allow_nan=False))
    engine.evaluate(harness)
    engine.evaluate((script or production_script()).replace('PAYLOAD',json.dumps(data,allow_nan=False)).replace('{plot_id}','test'))
    return engine


def toy_replay():
    states=[]
    for i in range(26):
        t=np.eye(4);t[0,3]=i*.001
        states.append({'q':[i*.01,0,0,0,0],'valid':True,'status':'PASS','reasons':[],
            'T_base_to_ee':t.tolist(),'expected_T_base_tcp':t.tolist(),'link_transforms_base':{'moving':t.tolist()},
            'semantic_blocking_pairs':[],'environment_pairs':[],'contacts_base':None,'query_id':i,
            'semantic_self_collision':False,'semantic_clearance':{},'task_errors':{},'approach_check':{'progress_m':i*.001}})
    data={'label':'test','surfaces':[{'link':'moving','vertices':[[0,0,0],[.01,0,0],[0,.01,0]]}],
          'states':states,'T_ee_tcp':np.eye(4).tolist(),'tcp':{'insertion_axis_local':[1,0,0],'key_axis_local':[0,0,1]},
          'pre_tcp_position':[0,0,0],'replay_context':{'all_samples_task_constrained':True}}
    traces=[{'type':'mesh3d','meta':{'role':'robot_collision_mesh'},'x':[0,.01,0],'y':[0,0,.01],'z':[0,0,0]},
        {'type':'scatter3d','name':'Actual synthetic TCP'},{'type':'scatter3d','name':'Approach expected TCP'},
        {'type':'scatter3d'}, {'type':'scatter3d','name':None}, {'type':'scatter3d','name':42},
        {'type':'scatter3d','name':'Target TCP invalid'},
        {'type':'scatter3d','meta':{'role':'robot_link_frame','object_id':'moving'},'line':None},
        {'type':'scatter3d','meta':{'role':'fcl_contacts'}}]
    return data,traces


def test_missing_null_nonstring_names_and_line_do_not_stop_any_of_26_frames():
    data,traces=toy_replay()
    with runtime(data,traces) as js:
        assert js.json('root.dataset').get('replayError') is None
        for i in range(26):
            js.evaluate(f'selectFrame({i});');assert js.json(f'checkFrame({i})')['index']==i
        assert js.json('root.data[7].visible') is False


def test_play_pause_end_restart_and_slider_stay_synchronized():
    data,traces=toy_replay()
    with runtime(data,traces) as js:
        js.evaluate('selectFrame(25);');js.json('checkFrame(25)')
        js.evaluate('renderHistory.length=0;getPlay().onclick();');js.json('checkFrame(0)')
        for i in range(1,26):
            js.evaluate('tick();');js.json(f'checkFrame({i})')
        assert js.json('renderHistory')==list(range(26))
        assert js.json('({playing:intervals.size,text:getPlay().textContent})')=={'playing':0,'text':'Play'}
        js.evaluate('selectFrame(3);');js.evaluate('getPlay().onclick();tick();');js.json('checkFrame(4)')
        js.evaluate('getPlay().onclick();tick();');js.json('checkFrame(4)')


def test_latest_slider_request_wins_while_plotly_render_is_pending():
    data,traces=toy_replay()
    with runtime(data,traces) as js:
        js.evaluate('Plotly.hold=true;selectFrame(1);selectFrame(5);selectFrame(10);')
        assert js.json('+root.dataset.pathIndex')==0
        js.evaluate('flushReact();');assert js.json('+root.dataset.pathIndex')==1
        js.evaluate('flushReact();');js.json('checkFrame(10)')
        assert js.json('renderHistory')==[0,1,10]


def test_render_error_pauses_playback_and_can_recover():
    data,traces=toy_replay()
    with runtime(data,traces) as js:
        js.evaluate('getPlay().onclick();Plotly.failNext=true;tick();')
        assert 'test Plotly failure' in js.json('root.dataset.replayError')
        assert js.json('intervals.size')==0 and js.json('getPlay().textContent')=='Play'
        js.evaluate('selectFrame(0);');js.json('checkFrame(0)')


def parse_saved_html(path):
    text=Path(path).read_text();payload=json.loads(text.split('\nconst model=',1)[1].split(';\nconst panel=',1)[0])
    call=text.rsplit('Plotly.newPlot(',1)[1];decoder=json.JSONDecoder();values=[]
    for _ in range(4):
        call=call.lstrip();value,end=decoder.raw_decode(call);values.append(value);call=call[end:].lstrip()
        if call.startswith(','):call=call[1:]
    return payload,values[1],values[2]


def test_newly_exported_html_executes_fixed_replay_not_only_source_ast(tmp_path):
    import plotly.graph_objects as go
    from workspace_analysis.viz.joint_path import export_path_viewer
    data,traces=toy_replay()
    targets=SimpleNamespace(spec=SimpleNamespace(T_ee_tcp=np.eye(4),data={'tcp':data['tcp']}),pre_tcp=SimpleNamespace(position_m=[0,0,0]))
    path=export_path_viewer(go.Figure(data=traces),data['surfaces'],data['states'],targets,tmp_path/'replay.html',label='test',replay_context=data['replay_context'])
    payload,saved_traces,layout=parse_saved_html(path);html=path.read_text()
    start=html.rfind('const root=document.getElementById(')
    marker='window.pathViewer={root,model,draw};draw(0);';end=html.index(marker,start)+len(marker)
    with runtime(payload,saved_traces,layout,script=html[start:end]) as js:
        for i in range(26):
            js.evaluate(f'selectFrame({i});');js.json(f'checkFrame({i})')


@pytest.mark.parametrize('name',['q0','q1'])
def test_saved_t4_native_data_all_26_frames_slider_and_play(name):
    data,traces,layout=parse_saved_html(ROOT/f'results/task_approach/p22_20261104_03/figures/{name}_approach.html')
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
    data,traces,layout=parse_saved_html(ROOT/'results/task_planning/p22_20261104_02/figures/q0_linear.html')
    assert data.get('replay_context') is None
    with runtime(data,traces,layout) as js:
        for i in [0,len(data['states'])//2,len(data['states'])-1]:
            js.evaluate(f'selectFrame({i});');result=js.json(f'checkFrame({i})')
            assert result['meshes']==24 and result['only_endpoint_is_task_constrained']
