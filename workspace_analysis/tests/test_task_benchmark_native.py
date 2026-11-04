"""Small generated integration inputs, independent of all historical results."""
from hashlib import sha256
import json
import os
from pathlib import Path
import pytest
import yaml
from workspace_analysis.robot.model import load_model
from workspace_analysis.workspace.sampling import sample_workspace
from workspace_analysis.workspace.results import save_results
from workspace_analysis.task.spec import load_task
from workspace_analysis.experiments.task_benchmark import run_benchmark
from workspace_analysis.experiments.task_benchmark_validation import verify_benchmark

ROOT = Path(__file__).resolve().parents[1]


def test_fresh_native_generic_roundtrip_and_same_seed_reproducibility(tmp_path):
    model = load_model(ROOT/'config/workspace.yaml'); config = ROOT/'config/workspace.yaml'
    prefix = tmp_path/'small_fixture'
    save_results(prefix, sample_workspace(model, samples=8, seed=42), model,
                 config_path=config, config_sha256=sha256(config.read_bytes()).hexdigest())
    data = load_task(ROOT/'config/tasks/p22_research_v0.yaml').data
    data['solver'].update(workspace_nearest_seeds=1, extra_joint_seeds=0, include_midpoint_seed=False, max_nfev=1)
    task = tmp_path/'limited_task.yaml'; task.write_text(yaml.safe_dump(data))
    summaries = []
    for i in range(2):
        output = tmp_path/str(i)
        result = run_benchmark(ROOT, Path(os.environ['C1_BACKEND']), ROOT/'config/scenes/idc_prototype_v0.yaml', task,
            prefix, output, port_ids=['P22'], allow_candidate=True)
        assert verify_benchmark(output, model)['status'] == 'PASS'
        summaries.append(json.loads((output/'ports/P22/result.json').read_text()))
        assert result['rows'][0]['ik_attempts'] == 3
        assert result['rows'][0]['linear_status'] == 'NOT_RUN'
    a,b = summaries
    assert a['start_q'] == b['start_q'] and a['status'] == b['status']
    assert a['reachability']['seeds'] == b['reachability']['seeds']
    assert [s['q'] for s in a['reachability']['ik']['attempts']] == [s['q'] for s in b['reachability']['ik']['attempts']]


@pytest.mark.parametrize('port', ['P11','P22','P33'])
def test_actual_geometry_task_viewer_uses_selected_port_label(port):
    from workspace_analysis.scene.schema import load_scene
    from workspace_analysis.task.template import tasks_for_ports
    from workspace_analysis.task.targets import build_targets, tcp_errors
    from workspace_analysis.planning.validity import assess_task
    from workspace_analysis.collision.client import CollisionClient
    from workspace_analysis.collision.semantic import SemanticConfig
    from workspace_analysis.collision.assets import robot_surfaces
    from workspace_analysis.viz.task import build_task_figure
    import numpy as np
    model = load_model(ROOT/'config/workspace.yaml'); scene = load_scene(ROOT/'config/scenes/idc_prototype_v0.yaml')
    spec = tasks_for_ports(load_task(ROOT/'config/tasks/p22_research_v0.yaml'), scene, [port])[0]
    targets = build_targets(spec, scene)
    semantic = SemanticConfig.load(ROOT/'config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf', model, allow_candidate=True)
    with CollisionClient(os.environ['C1_BACKEND']) as client:
        ready = client.initialize(model, scene, semantic=semantic)
        row = client.query(model, [0]*5, auxiliary_positions={'r_joint':.3}); row.update(q=[0]*5, sample_index='label_test')
        check = assess_task(tcp_errors(np.asarray(row['T_base_to_ee']) @ spec.T_ee_tcp, targets), targets)
        figure = build_task_figure(scene, robot_surfaces(model, ready, row), row, targets, check)
    assert figure.layout.title.text.startswith(port+' ')
    assert port+' port' in [t.name for t in figure.data]
    if port != 'P22': assert 'P22 port' not in [t.name for t in figure.data]
