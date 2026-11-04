"""Real new T5.1 saved evidence and historical equivalence; explicit artifacts."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from test_resources import ARTIFACTS
from workspace_analysis.robot.model import load_model
from workspace_analysis.task.spec import TaskError
from workspace_analysis.experiments.task_benchmark_validation import verify_benchmark
from workspace_analysis.experiments.task_benchmark_regression import compare_p22

ROOT = Path(__file__).resolve().parents[2]


def test_generic_p22_full_precision_historical_equivalence():
    model = load_model(ROOT/'config/workspace.yaml')
    report = compare_p22(ARTIFACTS/'task_benchmark/t5_1_p22_equivalence_20261104_02',
        ARTIFACTS/'task_reachability/p22_20261104_02', ARTIFACTS/'task_planning/p22_20261104_02',
        ARTIFACTS/'task_approach/p22_20261104_03', model)
    assert report['status'] == 'PASS'
    assert report['paths_compared'] == 4 and report['approach_samples_compared'] == 52
    assert max(report['maximum_absolute_differences'].values()) <= 1e-10


def test_nine_ports_evidence_hashes_counts_and_identity_tamper(tmp_path):
    out = ARTIFACTS/'task_benchmark/t5_1_s1_20261104_03'
    model = load_model(ROOT/'config/workspace.yaml')
    report = verify_benchmark(out, model)
    assert report['status'] == 'PASS' and report['ports_checked'] == 9
    for port in ('P11','P33'):
        text = (out/'ports'/port/'best_approximation.html').read_text()
        assert '"'+port+' port"' in text and '"P22 port"' not in text
    with pytest.raises(TaskError, match='model identity'):
        verify_benchmark(out, SimpleNamespace(metadata=lambda: {'expanded_urdf_sha256':'wrong'}))
    manifest = json.loads((out/'manifest.json').read_text())
    name = 'ports/P22/result.json'; manifest['files'] = {name: manifest['files'][name]}
    (tmp_path/name).parent.mkdir(parents=True)
    (tmp_path/name).write_text((out/name).read_text()+'\n')
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(TaskError, match='artifact hash/path'):
        verify_benchmark(tmp_path, model)
    escaped = tmp_path/'escape'; escaped.mkdir()
    manifest['files'] = {'../outside': '0'*64}
    (escaped/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(TaskError, match='artifact hash/path'):
        verify_benchmark(escaped, model)


def test_missing_historical_equivalence_is_blocked(tmp_path):
    report = compare_p22(tmp_path, tmp_path/'ik', tmp_path/'path', tmp_path/'approach', None)
    assert report['status'] == 'BLOCKED' and report['reason'] == 'MISSING_ARTIFACT'
