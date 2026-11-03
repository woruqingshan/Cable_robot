"""G4 small offline tests; never run the formal nine-experiment matrix."""

from dataclasses import replace
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest

from workspace_analysis.robot.model import load_model
from workspace_analysis.workspace.results import file_sha256, save_results
from workspace_analysis.workspace.sampling import sample_workspace
from workspace_analysis.experiments import workspace_validation as validation
from workspace_analysis.experiments import reporting as reporting


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / 'config/workspace.yaml'
SOURCE = Path(os.environ.get('G4_TEST_SOURCE_ROOT', ROOT))


@pytest.fixture(scope='module')
def model():
    return load_model(CONFIG)


@pytest.fixture
def small(model):
    return sample_workspace(model, samples=7, seed=42)


@pytest.fixture
def pair(model, small, tmp_path):
    prefix = tmp_path / 'input' / 'workspace_n7_seed42'
    metadata = save_results(prefix, small, model, config_path=CONFIG, config_sha256=file_sha256(CONFIG))
    return prefix, small.arrays(), metadata


def test_same_seed_exact_all_arrays_and_different_seed_changes(model, small):
    same = sample_workspace(model, samples=7, seed=42)
    other = sample_workspace(model, samples=7, seed=43)
    result = validation.compare_arrays(small.arrays(), same.arrays(), model.limits)
    assert result['status'] == 'PASS'
    assert result['max_position_error_m'] == 0
    assert result['max_rotation_error_rad'] <= 1e-14
    for key, item in result['arrays'].items():
        assert item['array_equal'] and item['first_dtype'] == 'float64'
        assert item['first_shape'] == list(small.arrays()[key].shape)
    assert validation.compare_arrays(small.arrays(), other.arrays(), model.limits)['status'] == 'FAIL'
    assert not np.array_equal(small.q, other.q)


def test_quaternion_sign_equivalence_does_not_relax_determinism(model, small):
    altered = {key: value.copy() for key, value in small.arrays().items()}
    altered['quaternion_xyzw'] *= -1
    result = validation.compare_arrays(small.arrays(), altered, model.limits)
    assert result['status'] == 'FAIL'
    assert result['max_quaternion_rotation_error_rad'] <= 1e-14
    assert result['max_rotation_error_rad'] <= 1e-14


@pytest.mark.parametrize('damage', ['dtype', 'shape', 'nan'])
def test_invalid_comparison_arrays_are_rejected(model, small, damage):
    arrays = {key: value.copy() for key, value in small.arrays().items()}
    if damage == 'dtype':
        arrays['position'] = arrays['position'].astype(np.float32)
    elif damage == 'shape':
        arrays['position'] = arrays['position'][:, :2]
    else:
        arrays['q'][1, 0] = np.nan
    with pytest.raises(ValueError):
        validation.compare_arrays(small.arrays(), arrays, model.limits)


def test_different_valid_sample_shapes_fail_with_no_invented_error(model, small):
    other = sample_workspace(model, samples=3, seed=42)
    result = validation.compare_arrays(small.arrays(), other.arrays(), model.limits)
    assert result['status'] == 'FAIL' and result['max_position_error_m'] is None


def test_fixed_grid_duplicate_hits_exact_edges_and_half_open_upper():
    grid = validation.VoxelGrid((0, 0, 0), (1, 1, 1), 0.25)
    points = np.array([[0, 0, 0], [0.1, 0.1, 0.1], [.25, .25, .25],
                       [np.nextafter(1., 0), .5, .75]])
    hits = validation.voxel_hits(points, grid)
    np.testing.assert_array_equal(hits['indices'], [[0, 0, 0], [0, 0, 0], [1, 1, 1], [3, 2, 3]])
    assert hits['hit_voxels'] == 3
    assert sorted(hits['counts']) == [1, 1, 2]
    assert grid.metadata()['upper_exclusive_m'] == [1, 1, 1]
    with pytest.raises(validation.ValidationError, match='sample_index=1'):
        validation.voxel_hits(np.array([[0., 0, 0], [1., 0, 0]]), grid)


def test_default_decimal_edges_and_shared_grid_are_not_fitted_to_data():
    grid = validation.VoxelGrid()
    before = grid.metadata()
    boundary = grid.edges[0][15]
    values = np.array([[boundary, 0, 0], [np.nextafter(boundary, -np.inf), 0, 0]])
    hits = validation.voxel_hits(values, grid)
    np.testing.assert_array_equal(hits['indices'][:, 0], [15, 14])
    validation.voxel_hits(np.array([[.01, .01, .01]]), grid)
    assert grid.metadata() == before
    assert grid.shape == (60, 60, 60)


@pytest.mark.parametrize('position', [np.zeros((0, 3)), np.zeros((3, 2)), [[float('nan'), 0, 0]],
                                      [[-.7, 0, 0]], [[0, 0, .9]], [[True, False, True]]])
def test_grid_rejects_invalid_or_outside_points(position):
    with pytest.raises(validation.ValidationError):
        validation.voxel_hits(position, validation.VoxelGrid())


@pytest.mark.parametrize('size', [0, -.1, float('nan'), float('inf'), True, np.bool_(True), '0.02'])
def test_invalid_voxel_size_rejected(size):
    with pytest.raises(validation.ValidationError):
        validation.VoxelGrid(voxel_size=size)


def test_grid_requires_integer_span_and_valid_range():
    with pytest.raises(validation.ValidationError):
        validation.VoxelGrid((0, 0, 0), (1, 1, 1), .3)
    with pytest.raises(validation.ValidationError):
        validation.VoxelGrid((0, 0, 0), (-1, 1, 1), .1)


def test_flat_statistics_and_voxel_count_mapping(pair):
    prefix, arrays, meta = pair
    # Explicit unit data tests the statistic mapping, not a saved FK dataset.
    synthetic = {'position': np.array([[0., 0, 0], [2., 4., 6.]])}
    row = validation.experiment_row(prefix, synthetic, meta,
                                    validation.VoxelGrid((0, 0, 0), (8, 8, 8), 1), reused=True)
    assert row['x_min_m'] == 0 and row['z_max_m'] == 6
    assert row['y_mean_m'] == row['y_std_m'] == 2
    assert row['z_aabb_edge_m'] == 6 and row['aabb_volume_m3'] == 48
    assert row['hit_voxels'] == 2 and row['reused'] is True
    assert row['fk_elapsed_s'] == meta['experiment']['fk_elapsed_s']


def test_seed_aggregates_have_predeclared_ddof_and_full_matrix_check():
    rows = [{'samples': 3, 'seed': seed, **{metric: value for metric in validation.METRICS}}
            for seed, value in [(42, 1), (43, 3), (44, 5)]]
    result = validation.aggregate_seeds(rows, [3], [42, 43, 44])
    assert len(result) == len(validation.METRICS)
    assert all(r['mean'] == 3 and r['std'] == 2 and r['std_ddof'] == 1 for r in result)
    for bad in (rows[:-1], rows + [rows[0]], [rows[0]] * 3):
        with pytest.raises(validation.ValidationError):
            validation.aggregate_seeds(bad, [3], [42, 43, 44])


def test_reuse_checks_model_config_versions_parameters_and_hash(pair, model):
    prefix, _, _ = pair
    versions, digest = validation.software_versions(), file_sha256(CONFIG)
    reused = validation.load_reuse([prefix], model, digest, versions, {(7, 42)})
    assert set(reused) == {(7, 42)}
    with pytest.raises(validation.ValidationError, match='Configuration hash mismatch'):
        validation.load_reuse([prefix], model, '0' * 64, versions, {(7, 42)})
    with pytest.raises(ValueError, match='model identity mismatch'):
        validation.load_reuse([prefix], replace(model, base_frame='base_footprint'), digest, versions, {(7, 42)})
    with pytest.raises(validation.ValidationError, match='Software version mismatch'):
        validation.load_reuse([prefix], model, digest, {**versions, 'numpy': 'different'}, {(7, 42)})
    with pytest.raises(validation.ValidationError, match='Unexpected or duplicate'):
        validation.load_reuse([prefix], model, digest, versions, {(7, 43)})
    with pytest.raises(validation.ValidationError, match='Duplicate experiment input'):
        validation.load_reuse([prefix, prefix], model, digest, versions, {(7, 42)})


def test_duplicate_experiment_in_two_distinct_directories_rejected(pair, model, tmp_path):
    prefix, _, _ = pair
    second = tmp_path / 'second' / prefix.name
    second.parent.mkdir()
    for suffix in ('.npz', '.json'):
        shutil.copyfile(Path(str(prefix) + suffix), Path(str(second) + suffix))
    with pytest.raises(validation.ValidationError, match='Unexpected or duplicate'):
        validation.load_reuse([prefix, second], model, file_sha256(CONFIG), validation.software_versions(), {(7, 42)})


@pytest.mark.parametrize('damage', ['missing', 'npz', 'json', 'lock'])
def test_incomplete_corrupt_reused_input_rejected(pair, model, damage):
    prefix, _, _ = pair
    if damage == 'missing':
        Path(str(prefix) + '.npz').unlink()
    elif damage == 'npz':
        with Path(str(prefix) + '.npz').open('ab') as stream:
            stream.write(b'corruption')
    elif damage == 'json':
        Path(str(prefix) + '.json').write_text('invalid json')
    else:
        Path(str(prefix) + '.lock').touch()
    with pytest.raises(ValueError):
        validation.load_reuse([prefix], model, file_sha256(CONFIG), validation.software_versions(), {(7, 42)})


def test_small_pipeline_reuses_data_roundtrips_reports_and_does_not_overwrite(pair, model, tmp_path, monkeypatch):
    prefix, _, _ = pair
    inputs = [Path(str(prefix) + suffix) for suffix in ('.npz', '.json')]
    before = [file_sha256(p) for p in inputs]
    actual, calls = validation.sample_workspace, []
    def recorded(model, *, samples, seed):
        calls.append((samples, seed))
        return actual(model, samples=samples, seed=seed)
    monkeypatch.setattr(validation, 'sample_workspace', recorded)
    report, paths = validation.run_validation(CONFIG, tmp_path / 'run', reuse_prefixes=[prefix],
                                              sample_counts=[7], seeds=[42, 43], repro_samples=5, progress=lambda _: None)
    assert calls == [(5, 42), (5, 42), (7, 43)]
    assert report['status'] == 'PASS' and report['experiment_profile'] == 'diagnostic'
    assert report['tf_rerun_in_g4'] is False
    assert report['stage_acceptance'] == 'pending_user_review'
    assert [r['reused'] for r in report['experiments']] == [True, False]
    assert report == reporting.load_validation_bundle(tmp_path / 'run')
    assert before == [file_sha256(p) for p in inputs]
    text = Path(paths['markdown']).read_text()
    assert 'did not launch RSP' in text and 'not reachable workspace volume' in text
    json_path = Path(paths['json'])
    original_json = json_path.read_text()
    tampered = json.loads(original_json)
    tampered['status'] = 'FAIL'
    json_path.write_text(json.dumps(tampered))
    with pytest.raises(validation.ValidationError, match='Inconsistent reproducibility status'):
        reporting.load_validation_bundle(tmp_path / 'run')
    json_path.write_text(original_json)
    with pytest.raises(validation.ValidationError, match='Refusing to overwrite'):
        validation.run_validation(CONFIG, tmp_path / 'run', sample_counts=[2], seeds=[42, 43], repro_samples=2)
    # Even a freshly recomputed artifact hash cannot hide CSV/JSON disagreement.
    csv_path = Path(paths['experiments_csv'])
    changed = csv_path.read_text().replace('7,42,True', '8,42,True', 1)
    assert changed != csv_path.read_text()
    csv_path.write_text(changed)
    with pytest.raises(validation.ValidationError, match='artifact hash mismatch'):
        reporting.load_validation_bundle(tmp_path / 'run')
    stored = json.loads(Path(paths['json']).read_text())
    stored['artifacts']['experiments_csv']['sha256'] = file_sha256(csv_path)
    Path(paths['json']).write_text(json.dumps(stored))
    with pytest.raises(validation.ValidationError, match='CSV/JSON results disagree'):
        reporting.load_validation_bundle(tmp_path / 'run')


def test_csv_comparison_preserves_float_precision_and_rejects_tampering():
    rows = [{'value': 1.2345678901234567, 'flag': True}]
    text = reporting.csv_text(rows, ['value', 'flag'])
    reporting.check_csv_json(text, rows, ['value', 'flag'])
    with pytest.raises(validation.ValidationError):
        reporting.check_csv_json(text.replace('True', 'False'), rows, ['value', 'flag'])


def test_output_files_refuse_overwrite_and_publication_rolls_back(tmp_path, monkeypatch):
    first, second = tmp_path / 'first.csv', tmp_path / 'validation.json'
    second.write_text('user data')
    with pytest.raises(validation.ValidationError, match='Refusing to overwrite'):
        reporting._publish_texts({first: 'unit A', second: 'unit B'})
    assert not first.exists() and second.read_text() == 'user data'
    second.unlink()
    actual = os.link
    def failure(source, target):
        if target == second:
            raise OSError('injected second publication failure')
        actual(source, target)
    monkeypatch.setattr(os, 'link', failure)
    with pytest.raises(OSError, match='second publication failure'):
        reporting._publish_texts({first: 'unit A', second: 'unit B'})
    assert list(tmp_path.iterdir()) == []


def test_sampling_exception_keeps_failed_run_marker_and_no_pass_report(tmp_path, monkeypatch):
    def failure(*args, **kwargs):
        raise RuntimeError('injected FK failure')
    monkeypatch.setattr(validation, 'sample_workspace', failure)
    output = tmp_path / 'failed'
    with pytest.raises(RuntimeError, match='FK failure'):
        validation.run_validation(CONFIG, output, sample_counts=[2], seeds=[42, 43], repro_samples=2)
    assert (output / '.g4_incomplete').exists()
    assert json.loads((output / 'failure.json').read_text())['status'] == 'ERROR'
    assert not (output / 'validation.json').exists()
    with pytest.raises(validation.ValidationError, match='Incomplete/failed'):
        reporting.load_validation_bundle(output)


def test_reproducibility_failure_is_a_real_fail_report(tmp_path, monkeypatch):
    actual, calls = validation.sample_workspace, []
    def flip_second(model, *, samples, seed):
        calls.append((samples, seed))
        run = actual(model, samples=samples, seed=seed)
        return replace(run, quaternion_xyzw=-run.quaternion_xyzw) if len(calls) == 2 else run
    monkeypatch.setattr(validation, 'sample_workspace', flip_second)
    report, _ = validation.run_validation(CONFIG, tmp_path / 'failed_repro', sample_counts=[2],
                                          seeds=[42, 43], repro_samples=3, progress=lambda _: None)
    assert report['status'] == 'FAIL' and report['reproducibility']['status'] == 'FAIL'
    assert reporting.load_validation_bundle(tmp_path / 'failed_repro')['status'] == 'FAIL'


def test_reused_points_outside_shared_region_stop_before_sampling(pair, tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('must not sample after failed preflight')
    monkeypatch.setattr(validation, 'sample_workspace', forbidden)
    with pytest.raises(validation.ValidationError, match='sample_index=0'):
        validation.run_validation(CONFIG, tmp_path / 'outside', reuse_prefixes=[pair[0]],
                                  sample_counts=[7], seeds=[42, 43], repro_samples=3,
                                  grid=validation.VoxelGrid((99, 99, 99), (100, 100, 100), .1))
    assert not (tmp_path / 'outside').exists()


def test_existing_markdown_template_is_never_overwritten(tmp_path):
    markdown = tmp_path / 'template.md'
    markdown.write_text('pending experiment template')
    with pytest.raises(validation.ValidationError, match='Refusing to overwrite'):
        validation.run_validation(CONFIG, tmp_path / 'new_run', sample_counts=[2], seeds=[42, 43],
                                  repro_samples=2, report_path=markdown)
    assert markdown.read_text() == 'pending experiment template'
    assert not (tmp_path / 'new_run').exists()


def test_prior_evidence_is_cited_without_tf_rerun(model):
    digest = model.metadata()['expanded_urdf_sha256']
    for name, stage in [('g1a_fk.json', 'G1-A'), ('g1b_tf_v3.json', 'G1-B')]:
        result = validation.prior_evidence(SOURCE / 'results' / name, stage, digest)
        assert result['available'] and not result['rerun_in_g4'] and len(result['cases']) == 3


def test_cli_small_diagnostic_from_other_cwd_and_error_exit(tmp_path):
    script = ROOT / 'scripts/validate_workspace.py'
    output = tmp_path / 'cli_run'
    command = [sys.executable, str(script), '--config', str(CONFIG), '--output-dir', str(output),
               '--sample-counts', '3', '--seeds', '42', '43', '--repro-samples', '4',
               '--g1a-report', str(SOURCE / 'results/g1a_fk.json'),
               '--g1b-report', str(SOURCE / 'results/g1b_tf_v3.json')]
    result = subprocess.run(command, cwd=tmp_path, text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert 'profile: diagnostic' in result.stdout and 'status: PASS' in result.stdout
    assert reporting.load_validation_bundle(output)['experiment_profile'] == 'diagnostic'
    again = subprocess.run(command, cwd=tmp_path, text=True, capture_output=True, timeout=30)
    assert again.returncode != 0 and 'status: ERROR' in again.stderr
    assert 'status: PASS' not in again.stdout


def test_report_does_not_assume_stability_improves():
    aggregates = [{'samples': n, 'metric': metric, 'mean': 1., 'std': float(n)}
                  for n in [2, 4] for metric in validation.METRICS]
    text = reporting.stability_text({'design': {'sample_counts': [2, 4]}, 'across_seed': aggregates})
    assert 'Only 0/6' in text and 'increased' in text and 'no universal convergence' in text
