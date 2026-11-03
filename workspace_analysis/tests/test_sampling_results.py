"""Small CPU-only G2 tests. No ROS Context, RSP, hardware or 10k experiment."""

from dataclasses import replace
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from workspace_analysis.robot.fk import ForwardKinematics
from workspace_analysis.robot.model import load_model
from workspace_analysis.workspace.sampling import (SamplingError, evaluate_joint_samples, recheck_fk,
                                         sample_joint_positions, sample_workspace, validate_arrays)
from workspace_analysis.workspace.results import (ResultsError, file_sha256, load_results, result_paths,
                                        save_results, workspace_statistics)
from reference_fk import pose_errors, reference_fk


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/workspace.yaml"


@pytest.fixture(scope="module")
def model():
    return load_model(CONFIG)


@pytest.fixture(scope="module")
def run(model):
    return sample_workspace(model, 12, 42)


def save(prefix, run, model):
    return save_results(prefix, run, model, config_path=CONFIG, config_sha256=file_sha256(CONFIG))


def test_sampling_shape_dtype_finite_limits_and_dynamic_joint_count():
    limits = np.array([[-0.7, 1.3], [0, 0], [0.2, 0.5]])
    q = sample_joint_positions(limits, 32, 42)
    assert q.shape == (32, 3) and q.dtype == np.float64
    assert np.isfinite(q).all()
    assert (q >= limits[:, 0]).all() and (q <= limits[:, 1]).all()
    assert (q[:, 1] == 0).all()


@pytest.mark.parametrize("count", [0, -1, 1.2, True, "10", None, np.nan])
def test_invalid_sample_counts(count):
    with pytest.raises(SamplingError, match="samples"):
        sample_joint_positions([[-1, 1]], count, 42)


@pytest.mark.parametrize("seed", [-1, 0.3, True, "42", None, np.inf])
def test_invalid_seed(seed):
    with pytest.raises(SamplingError, match="seed"):
        sample_joint_positions([[-1, 1]], 3, seed)


@pytest.mark.parametrize("limits", [[], [1, 2], [[1, -1]], [[np.nan, 1]],
                                     [[-1, np.inf]], [["-1", "1"]], [[False, True]], [[-1e308, 1e308]]])
def test_invalid_limits(limits):
    with pytest.raises(SamplingError):
        sample_joint_positions(limits, 3, 42)


def test_fixed_seed_reproduces_all_arrays_and_specific_other_seed_differs(model, run):
    repeated = sample_workspace(model, 12, 42)
    for name, value in run.arrays().items():
        np.testing.assert_array_equal(value, repeated.arrays()[name])
    # A fixed concrete comparison, not an assertion about every pair of seeds.
    other = sample_joint_positions(model.limits, 12, 43)
    assert not np.array_equal(run.q, other)
    assert run.bit_generator == type(np.random.default_rng(42).bit_generator).__name__


def test_all_poses_valid_and_reference_checks(model, run):
    validate_arrays(run.arrays(), model.limits)
    assert run.q.shape == (12, 5)
    np.testing.assert_allclose(np.linalg.norm(run.quaternion_xyzw, axis=1), 1, atol=1e-12, rtol=0)
    np.testing.assert_allclose(Rotation.from_quat(run.quaternion_xyzw).as_matrix(), run.rotation, atol=1e-12, rtol=0)
    verification = recheck_fk(model, run.arrays())
    assert verification["indices"] == [0, 6, 11]
    for index in verification["indices"]:
        expected = reference_fk(model.urdf_xml, run.q[index], base_frame=model.base_frame,
                                ee_frame=model.ee_frame, joint_order=model.joint_order)
        actual = np.eye(4)
        actual[:3, :3], actual[:3, 3] = run.rotation[index], run.position[index]
        position_error, angle_error = pose_errors(actual, expected)
        assert position_error <= 1e-5 and angle_error <= 1e-4


def test_zero_and_joint5_geometry_use_explicit_inputs(model):
    zero = evaluate_joint_samples(model, np.zeros((1, 5)))
    pose = ForwardKinematics(model).pose(np.zeros(5))
    np.testing.assert_array_equal(zero.position[0], pose.position)
    np.testing.assert_array_equal(zero.rotation[0], pose.rotation)
    q = np.array([[0.35, -0.6, 0.8, -0.4, -0.3], [0.35, -0.6, 0.8, -0.4, 0.7]])
    explicit = evaluate_joint_samples(model, q)
    np.testing.assert_allclose(explicit.position[0], explicit.position[1], atol=1e-12, rtol=0)
    assert Rotation.from_matrix(explicit.rotation[0].T @ explicit.rotation[1]).magnitude() > 0.1
    assert explicit.q.shape == (2, 5)  # joint5 is retained.


def test_fk_exception_identifies_sample_and_does_not_return_partial_run(model):
    solver = ForwardKinematics(model)
    class FailsOnThird:
        count = 0
        def pose(self, q):
            self.count += 1
            if self.count == 3:
                raise RuntimeError("simulated FK failure")
            return solver.pose(q)
    q = sample_joint_positions(model.limits, 4, 42)
    with pytest.raises(SamplingError, match="sample_index=2.*simulated FK failure"):
        evaluate_joint_samples(model, q, fk=FailsOnThird())


@pytest.mark.parametrize("bad", ["nonfinite", "rotation", "transform"])
def test_invalid_fk_pose_identifies_sample(model, bad):
    real = ForwardKinematics(model).pose(np.zeros(5))
    position, rotation, transform = real.position.copy(), real.rotation.copy(), real.transform.copy()
    if bad == "nonfinite":
        position[0] = np.nan
    elif bad == "rotation":
        rotation[0, 0] = 2
    else:
        transform[3, 3] = 0
    fake = SimpleNamespace(pose=lambda q: SimpleNamespace(position=position, rotation=rotation, transform=transform))
    with pytest.raises(SamplingError, match="sample_index=0"):
        evaluate_joint_samples(model, np.zeros((1, 5)), fk=fake)


@pytest.mark.parametrize("field", ["q", "position", "rotation", "quaternion_xyzw"])
def test_nonfinite_array_row_is_rejected(model, run, field):
    changed = run.arrays().copy()
    changed[field] = changed[field].copy()
    changed[field][2].flat[0] = np.nan
    with pytest.raises(SamplingError, match="sample_index=2"):
        validate_arrays(changed, model.limits)


def test_invalid_quaternion_dtype_shape_and_row_correspondence(model, run):
    for field, value in [("quaternion_xyzw", run.quaternion_xyzw * 2),
                         ("q", run.q.astype(np.float32)), ("position", run.position[:-1]),
                         ("quaternion_xyzw", np.roll(run.quaternion_xyzw, 1, axis=0))]:
        with pytest.raises(SamplingError):
            validate_arrays({**run.arrays(), field: value}, model.limits)


def test_npz_json_roundtrip_metadata_statistics_and_hash(model, run, tmp_path):
    prefix = tmp_path / "workspace.v1"
    metadata = save(prefix, run, model)
    arrays, loaded = load_results(prefix, model=model)
    assert loaded == metadata
    for name, value in run.arrays().items():
        np.testing.assert_array_equal(arrays[name], value)
        assert metadata["dataset"]["arrays"][name] == {"shape": list(value.shape), "dtype": "float64"}
    npz, json_path, lock = result_paths(prefix)
    expected_hash = sha256(npz.read_bytes()).hexdigest()
    assert metadata["dataset"]["sha256"] == expected_hash
    assert metadata["reproducibility"]["npz_sha256"] == expected_hash
    assert metadata["configuration"]["sha256"] == sha256(CONFIG.read_bytes()).hexdigest()
    assert metadata["robot_model"]["joint_limits_rad"] == model.limits.tolist()
    assert metadata["experiment"]["completed_samples"] == 12
    stats = metadata["workspace_statistics"]
    np.testing.assert_array_equal(stats["min_m"], run.position.min(axis=0))
    np.testing.assert_array_equal(stats["max_m"], run.position.max(axis=0))
    np.testing.assert_array_equal(stats["mean_m"], run.position.mean(axis=0))
    np.testing.assert_array_equal(stats["std_m"], run.position.std(axis=0, ddof=0))
    np.testing.assert_array_equal(stats["aabb_edge_lengths_m"], np.ptp(run.position, axis=0))
    assert stats["aabb_volume_m3"] == np.prod(np.ptp(run.position, axis=0))
    assert "NOT reachable workspace volume" in stats["volume_meaning"]
    assert not lock.exists() and json_path.exists()
    assert metadata["stage_acceptance"] == "pending_user_review"


@pytest.mark.parametrize("extension", [".npz", ".json", ".lock"])
def test_existing_targets_not_overwritten(model, run, tmp_path, extension):
    prefix = tmp_path / "protected"
    existing = Path(str(prefix) + extension)
    existing.write_bytes(b"keep-me")
    with pytest.raises(ResultsError, match="Refusing to overwrite"):
        save(prefix, run, model)
    assert existing.read_bytes() == b"keep-me"
    assert sorted(p.name for p in tmp_path.iterdir()) == [existing.name]


def test_second_commit_failure_rolls_back_npz_and_temporaries(model, run, tmp_path, monkeypatch):
    actual_link = os.link
    def fail_json(source, destination):
        if Path(destination).suffix == ".json":
            raise OSError("simulated second commit failure")
        actual_link(source, destination)
    monkeypatch.setattr(os, "link", fail_json)
    with pytest.raises(ResultsError, match="second commit failure"):
        save(tmp_path / "failed", run, model)
    assert list(tmp_path.iterdir()) == []


def test_both_staging_paths_checked_before_any_publication(model, run, tmp_path, monkeypatch):
    import workspace_analysis.workspace.results as results
    original = results.tempfile.mkstemp
    calls = [0]
    def second_path_denied(*args, **kwargs):
        calls[0] += 1
        if calls[0] == 2:
            raise PermissionError("second staging path not writable")
        return original(*args, **kwargs)
    monkeypatch.setattr(results.tempfile, "mkstemp", second_path_denied)
    monkeypatch.setattr(os, "link", lambda *a, **k: pytest.fail("publication before staging preflight"))
    with pytest.raises(ResultsError, match="not writable"):
        save(tmp_path / "failed", run, model)
    assert list(tmp_path.iterdir()) == []


def test_target_appearing_during_commit_is_not_overwritten(model, run, tmp_path, monkeypatch):
    original = os.link
    existing_json = tmp_path / "race.json"
    def race(source, destination):
        if Path(destination) == existing_json:
            existing_json.write_text("external-writer", encoding="utf-8")
        original(source, destination)
    monkeypatch.setattr(os, "link", race)
    with pytest.raises(ResultsError):
        save(tmp_path / "race", run, model)
    assert existing_json.read_text() == "external-writer"
    assert sorted(p.name for p in tmp_path.iterdir()) == [existing_json.name]


def test_configuration_change_or_bad_fk_data_refuses_publication(model, run, tmp_path):
    with pytest.raises(ResultsError, match="Configuration changed"):
        save_results(tmp_path / "badconfig", run, model, config_path=CONFIG, config_sha256="0" * 64)
    with pytest.raises(ResultsError, match="sample_index"):
        save(tmp_path / "badpose", replace(run, position=run.position + 0.01), model)
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("remove", ["npz", "json"])
def test_incomplete_pair_rejected(model, run, tmp_path, remove):
    prefix = tmp_path / "incomplete"
    save(prefix, run, model)
    Path(str(prefix) + "." + remove).unlink()
    with pytest.raises(ResultsError, match="Incomplete"):
        load_results(prefix)


def test_lock_and_corrupt_npz_rejected(model, run, tmp_path):
    prefix = tmp_path / "valid"
    save(prefix, run, model)
    npz, _, lock = result_paths(prefix)
    lock.write_text("interrupted transaction")
    with pytest.raises(ResultsError, match="transaction lock"):
        load_results(prefix)
    lock.unlink()
    with npz.open("ab") as stream:
        stream.write(b"corrupt")
    with pytest.raises(ResultsError, match="hash mismatch"):
        load_results(prefix)


@pytest.mark.parametrize("tamper", ["shape", "count", "stats", "scope", "missing", "invalid_json"])
def test_inconsistent_json_rejected(model, run, tmp_path, tamper):
    prefix = tmp_path / "valid"
    metadata = save(prefix, run, model)
    _, path, _ = result_paths(prefix)
    if tamper == "shape":
        metadata["dataset"]["arrays"]["q"]["shape"][0] = 999
    elif tamper == "count":
        metadata["experiment"]["completed_samples"] -= 1
    elif tamper == "stats":
        metadata["workspace_statistics"]["mean_m"][0] += 0.01
    elif tamper == "scope":
        metadata["scope"]["collision_checked"] = True
    elif tamper == "missing":
        del metadata["experiment"]["completed_utc"]
    if tamper == "invalid_json":
        path.write_text("{invalid")
    else:
        path.write_text(json.dumps(metadata))
    with pytest.raises(ResultsError):
        load_results(prefix)


@pytest.mark.parametrize("malformed", ["missing_array", "float32", "object"])
def test_invalid_npz_even_with_updated_file_hash_is_rejected(model, run, tmp_path, malformed):
    prefix = tmp_path / "badnpz"
    metadata = save(prefix, run, model)
    npz, path, _ = result_paths(prefix)
    arrays = run.arrays().copy()
    if malformed == "missing_array":
        del arrays["position"]
    elif malformed == "float32":
        arrays["q"] = arrays["q"].astype(np.float32)
    else:
        arrays["q"] = arrays["q"].astype(object)
    with npz.open("wb") as stream:
        np.savez(stream, **arrays)
    metadata["dataset"]["sha256"] = metadata["reproducibility"]["npz_sha256"] = file_sha256(npz)
    path.write_text(json.dumps(metadata))
    with pytest.raises(ResultsError):
        load_results(prefix)


def test_wrong_model_identity_rejected(model, run, tmp_path):
    prefix = tmp_path / "valid"
    save(prefix, run, model)
    changed = replace(model, ee_frame="different")
    with pytest.raises(ResultsError, match="model identity mismatch"):
        load_results(prefix, model=changed)


def test_cli_from_other_cwd_small_dataset_only(tmp_path):
    prefix = tmp_path / "cli_7"
    script = ROOT / "scripts/sample_workspace.py"
    command = [sys.executable, str(script), "--config", str(CONFIG), "--samples", "7", "--seed", "42",
               "--output-prefix", str(prefix)]
    result = subprocess.run(command, cwd=tmp_path, text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert "FK completed: 7/7" in result.stdout and "status: PASS" in result.stdout
    arrays, _ = load_results(prefix)
    assert arrays["q"].shape == (7, 5)
    protected = [file_sha256(path) for path in result_paths(prefix)[:2]]
    again = subprocess.run(command, cwd=tmp_path, text=True, capture_output=True, timeout=30)
    assert again.returncode != 0 and "Refusing to overwrite" in again.stderr
    assert protected == [file_sha256(path) for path in result_paths(prefix)[:2]]


def test_statistics_overflow_is_not_silently_saved():
    with pytest.raises(ResultsError):
        workspace_statistics(np.array([[1e308, 0, 0], [-1e308, 0, 0]]))


def test_invalid_prefixes_do_not_resolve_to_unintended_sibling_files(tmp_path):
    for prefix in ["", ".", "..", "/", tmp_path, tmp_path / "data.npz", tmp_path / "data.json"]:
        with pytest.raises(ResultsError):
            result_paths(prefix)


def test_keyboard_interrupt_during_second_commit_also_rolls_back(model, run, tmp_path, monkeypatch):
    original = os.link
    def interrupt(source, destination):
        if Path(destination).suffix == ".json":
            raise KeyboardInterrupt()
        original(source, destination)
    monkeypatch.setattr(os, "link", interrupt)
    with pytest.raises(KeyboardInterrupt):
        save(tmp_path / "interrupted", run, model)
    assert list(tmp_path.iterdir()) == []


def test_incomplete_fk_evidence_and_rng_provenance_rejected(model, run, tmp_path):
    prefix = tmp_path / "valid"
    metadata = save(prefix, run, model)
    _, path, _ = result_paths(prefix)
    del metadata["validation"]["fk_recheck"]["max_rotation_error_rad"]
    path.write_text(json.dumps(metadata))
    with pytest.raises(ResultsError):
        load_results(prefix)
    metadata["validation"]["fk_recheck"]["max_rotation_error_rad"] = 0
    del metadata["reproducibility"]["bit_generator"]
    path.write_text(json.dumps(metadata))
    with pytest.raises(ResultsError):
        load_results(prefix)


def test_bad_completion_metadata_never_gets_committed(model, run, tmp_path):
    with pytest.raises(ResultsError):
        save(tmp_path / "badtime", replace(run, completed_utc="not-a-time"), model)
    assert list(tmp_path.iterdir()) == []
