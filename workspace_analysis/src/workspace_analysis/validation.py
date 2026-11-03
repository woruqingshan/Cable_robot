"""G4 reproducibility, descriptive stability and fixed-grid hit statistics.

Reuse G1/G2 unchanged. Formal experiments are run only through the user's CLI;
small diagnostic designs use the same path, explicitly labelled diagnostic.
"""

from dataclasses import dataclass
from datetime import datetime, timezone
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform

import numpy as np
import PyKDL
import scipy
from scipy.spatial.transform import Rotation

from .model import JOINT_ORDER, load_model
from .results import file_sha256, load_results, result_paths, save_results, workspace_statistics
from .sampling import ALGORITHM, SCOPE_LIMITATIONS, sample_workspace, validate_arrays, validate_integer


FORMAL_COUNTS = (100, 1000, 10000)
FORMAL_SEEDS = (42, 43, 44)
METRICS = tuple(f"{axis}_{stat}_m" for axis in "xyz" for stat in ("min", "max", "mean", "std", "aabb_edge")) + (
    "aabb_volume_m3", "fk_elapsed_s", "hit_voxels")
ROW_FIELDS = ("samples", "seed", "reused", "source_prefix", "npz_sha256", "json_sha256",
              "model_sha256", "config_sha256") + METRICS


class ValidationError(ValueError):
    """Invalid evidence, an incomparable experiment, or failed G4 execution."""


def software_versions():
    versions = {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
                "PyKDL": getattr(PyKDL, "__version__", "unknown")}
    for name in ("urdfdom_py", "xacro", "PyYAML"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = "unknown"
    return versions


def _positive_design(values, label, minimum):
    result = tuple(validate_integer(v, label, minimum=minimum) for v in values)
    if not result or len(set(result)) != len(result):
        raise ValidationError(f"{label} must be nonempty and contain no duplicates")
    return tuple(sorted(result))


def _xyz(value, label):
    raw = np.asarray(value)
    if raw.shape != (3,) or raw.dtype.kind not in "iuf" or not np.isfinite(raw).all():
        raise ValidationError(f"{label} must be three finite real numbers")
    return raw.astype(np.float64)


@dataclass(frozen=True)
class VoxelGrid:
    """One fixed analysis grid shared by every experiment; never fit per dataset."""

    origin: tuple = (-0.6, -0.6, -0.4)
    upper: tuple = (0.6, 0.6, 0.8)
    voxel_size: float = 0.02

    def __post_init__(self):
        origin, requested = _xyz(self.origin, "grid origin"), _xyz(self.upper, "grid upper")
        if (isinstance(self.voxel_size, (bool, np.bool_, str)) or not np.isscalar(self.voxel_size)
                or not math.isfinite(float(self.voxel_size)) or float(self.voxel_size) <= 0):
            raise ValidationError("voxel_size must be finite and positive, in metres")
        size = float(self.voxel_size)
        with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
            cells = (requested - origin) / size
        if (not np.isfinite(cells).all() or np.any(cells < 1) or np.any(cells > 1_000_000)
                or not np.allclose(cells, np.rint(cells), atol=1e-8, rtol=0)):
            raise ValidationError("Grid spans must be positive integer multiples of voxel_size (<=1e6 cells/axis)")
        shape = tuple(int(v) for v in np.rint(cells))
        upper = origin + np.array(shape) * size
        edges = tuple(origin[i] + np.arange(shape[i] + 1) * size for i in range(3))
        if any(np.any(np.diff(axis) <= 0) for axis in edges):
            raise ValidationError("Grid edges are not distinguishable at float64 precision")
        object.__setattr__(self, "origin", tuple(origin.tolist()))
        object.__setattr__(self, "requested_upper", tuple(requested.tolist()))
        object.__setattr__(self, "upper", tuple(upper.tolist()))
        object.__setattr__(self, "voxel_size", size)
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "edges", edges)

    def metadata(self):
        return {"origin_m": list(self.origin), "upper_exclusive_m": list(self.upper),
                "requested_upper_m": list(self.requested_upper), "voxel_size_m": self.voxel_size,
                "shape": list(self.shape), "axis_order": ["x", "y", "z"],
                "index_rule": "edge[k] = origin + k*size in float64; cell [edge[k],edge[k+1]); "
                              "index = searchsorted(edges, point, side='right') - 1 on each axis",
                "out_of_bounds": "ERROR with first original row index; never clip or discard",
                "meaning": "sample-hit cells only; empty is not unreachable; hits are not collision-safe",
                "resolution_meaning": "analysis resolution, not robot positioning precision"}


def voxel_hits(position, grid):
    values = np.asarray(position)
    if (values.ndim != 2 or values.shape[0] < 1 or values.shape[1] != 3
            or values.dtype.kind not in "iuf" or not np.isfinite(values).all()):
        raise ValidationError("position must be a nonempty finite real (N,3) array")
    outside = ((values < grid.origin) | (values >= grid.upper)).any(axis=1)
    if outside.any():
        index = int(np.flatnonzero(outside)[0])
        raise ValidationError(f"Outside fixed voxel region: sample_index={index}, position={values[index].tolist()}")
    indices = np.column_stack([np.searchsorted(grid.edges[i], values[:, i], side="right") - 1
                               for i in range(3)]).astype(np.int64)
    unique, counts = np.unique(indices, axis=0, return_counts=True)
    return {"indices": indices, "unique_indices": unique, "counts": counts, "hit_voxels": len(unique)}


def compare_arrays(first, second, limits):
    """Predeclared criterion: same float64 shapes and np.array_equal for ALL arrays.

    Pose-equivalence errors are diagnostics, not a relaxed deterministic pass.
    In particular a quaternion sign flip fails array equality but has zero
    orientation-equivalence error.
    """
    validate_arrays(first, limits)
    validate_arrays(second, limits)
    comparisons = {}
    same_shapes = True
    for name in ("q", "position", "rotation", "quaternion_xyzw"):
        a, b = first[name], second[name]
        same = a.shape == b.shape
        same_shapes &= same
        comparisons[name] = {"first_shape": list(a.shape), "second_shape": list(b.shape),
                             "first_dtype": str(a.dtype), "second_dtype": str(b.dtype),
                             "array_equal": bool(same and np.array_equal(a, b)),
                             "max_absolute_difference": float(np.max(np.abs(a - b))) if same else None}
    position_error = rotation_error = quaternion_error = None
    if same_shapes:
        position_error = float(np.max(np.linalg.norm(first["position"] - second["position"], axis=1)))
        rotation_error = float(np.max((Rotation.from_matrix(first["rotation"]).inv()
                                       * Rotation.from_matrix(second["rotation"])).magnitude()))
        quaternion_error = float(np.max((Rotation.from_quat(first["quaternion_xyzw"]).inv()
                                         * Rotation.from_quat(second["quaternion_xyzw"])).magnitude()))
    return {"criterion": "all four arrays: identical shape/dtype and np.array_equal; no tolerance relaxation",
            "status": "PASS" if all(row["array_equal"] for row in comparisons.values()) else "FAIL",
            "arrays": comparisons, "max_position_error_m": position_error,
            "max_rotation_error_rad": rotation_error, "max_quaternion_rotation_error_rad": quaternion_error,
            "scope": "same software environment only; no cross-platform or cross-version bitwise guarantee"}


def load_experiment(prefix, model, config_sha256, versions):
    """G2 validation plus the configuration/software provenance needed by G4."""
    arrays, metadata = load_results(prefix, model=model)
    if metadata["configuration"]["sha256"] != config_sha256:
        raise ValidationError(f"Configuration hash mismatch: {prefix}")
    expected = json.loads(json.dumps(model.metadata()))
    if any(metadata["robot_model"].get(key) != value for key, value in expected.items()):
        raise ValidationError(f"Full model/source identity mismatch: {prefix}")
    if metadata["experiment"]["software_versions"] != versions:
        raise ValidationError(f"Software version mismatch: {prefix}; use one environment or fresh experiments")
    return arrays, metadata


def load_reuse(prefixes, model, config_sha256, versions, design):
    result, seen = {}, set()
    for prefix in prefixes:
        path = Path(prefix).expanduser().resolve()
        if path in seen:
            raise ValidationError(f"Duplicate experiment input path: {path}")
        seen.add(path)
        arrays, metadata = load_experiment(path, model, config_sha256, versions)
        key = (metadata["experiment"]["completed_samples"], metadata["experiment"]["seed"])
        if key not in design or key in result:
            raise ValidationError(f"Unexpected or duplicate experiment (N,seed): {key}")
        result[key] = (path, arrays, metadata)
    return result


def experiment_row(prefix, arrays, metadata, grid, *, reused):
    stats = workspace_statistics(arrays["position"])
    row = {"samples": len(arrays["position"]), "seed": metadata["experiment"]["seed"],
           "reused": bool(reused), "source_prefix": str(Path(prefix).resolve()),
           "npz_sha256": metadata["dataset"]["sha256"], "json_sha256": file_sha256(result_paths(prefix)[1]),
           "model_sha256": metadata["robot_model"]["expanded_urdf_sha256"],
           "config_sha256": metadata["configuration"]["sha256"],
           "aabb_volume_m3": stats["aabb_volume_m3"],
           "fk_elapsed_s": metadata["experiment"]["fk_elapsed_s"],
           "hit_voxels": voxel_hits(arrays["position"], grid)["hit_voxels"]}
    for index, axis in enumerate("xyz"):
        for label, field in (("min", "min_m"), ("max", "max_m"), ("mean", "mean_m"),
                             ("std", "std_m"), ("aabb_edge", "aabb_edge_lengths_m")):
            row[f"{axis}_{label}_m"] = stats[field][index]
    return row


def aggregate_seeds(rows, sample_counts, seeds):
    expected = {(n, seed) for n in sample_counts for seed in seeds}
    keys = [(row["samples"], row["seed"]) for row in rows]
    if len(seeds) < 2 or len(keys) != len(expected) or set(keys) != expected:
        raise ValidationError("Incomplete/duplicate experiment matrix, or fewer than two seeds")
    result = []
    for n in sample_counts:
        selected = [r for r in rows if r["samples"] == n]
        for metric in METRICS:
            values = np.array([r[metric] for r in selected], dtype=np.float64)
            if not np.isfinite(values).all():
                raise ValidationError(f"Nonfinite statistic: {metric}")
            result.append({"samples": n, "metric": metric, "seed_count": len(seeds),
                           "mean": float(values.mean()), "std": float(values.std(ddof=1)), "std_ddof": 1})
    return result


def prior_evidence(path, stage, model_sha256):
    """Cite prior files only; importing/running tf_validation is unnecessary."""
    if path is None:
        return {"stage": stage, "available": False, "rerun_in_g4": False}
    path = Path(path).expanduser().resolve()
    data = json.loads(path.read_text(encoding="utf-8"))
    if data["stage"] != stage or data["model"]["expanded_urdf_sha256"] != model_sha256:
        raise ValidationError(f"Prior evidence stage/model mismatch: {path}")
    rows = data["poses"] if stage == "G1-A" else data["cases"]
    accepted = data["status"] in ("PASS", "SOFTWARE_CHECKS_PASSED")
    accepted &= (len(rows) == 3 and {r["name"] for r in rows} == {"Zero", "Nonzero", "Near-limit"})
    accepted &= all(isinstance(r[key], (int, float)) and not isinstance(r[key], bool)
                    and math.isfinite(r[key]) and 0 <= r[key] <= threshold
                    for r in rows for key, threshold in (("position_error_m", 1e-5), ("rotation_error_rad", 1e-4)))
    if stage == "G1-A":
        accepted &= data.get("reference_checked") is True
    if stage == "G1-B":
        accepted &= (data.get("tf_checked") is True and data.get("strict_endpoint_identity_verified") is True
                     and all(r.get("passed") is True and r.get("strict_endpoint_identity_verified") is True
                             and r.get("sync_complete") is True and r["cleanup"]["rsp_exit_normal"] is True
                             and r["cleanup"]["rsp_exit_code"] == 0 for r in rows))
    if not accepted or len(rows) != 3:
        raise ValidationError(f"Prior evidence is not a completed three-case success: {path}")
    return {"stage": stage, "available": True, "path": str(path), "sha256": file_sha256(path),
            "reported_status": data["status"], "rerun_in_g4": False,
            "strict_endpoint_identity_verified": data.get("strict_endpoint_identity_verified"),
            "rsp_exit_codes": [r['cleanup']['rsp_exit_code'] for r in rows] if stage == 'G1-B' else [],
            "cases": [{"name": r["name"], "position_error_m": r["position_error_m"],
                       "rotation_error_rad": r["rotation_error_rad"]} for r in rows],
            "note": "Previously user-executed evidence; no TF integration rerun in G4"}


def _code_hashes():
    return {str(p): file_sha256(p) for p in Path(__file__).parent.glob("*.py")}


def run_validation(config_path, output_dir, *, reuse_prefixes=(), sample_counts=FORMAL_COUNTS,
                   seeds=FORMAL_SEEDS, repro_samples=1000, repro_seed=42, grid=None,
                   report_path=None, g1a_report=None, g1b_report=None, visualization_prefix=None, progress=print):
    """User-run experiment orchestration; output_dir MUST NOT already exist.

    Failure leaves an explicit .g4_incomplete marker and diagnostics; valid
    generated G2 pairs are preserved for inspection. Never mutate reused input.
    """
    from .validation_report import write_bundle

    counts = _positive_design(sample_counts, "sample_counts", 1)
    seeds = _positive_design(seeds, "seeds", 0)
    if len(seeds) < 2:
        raise ValidationError("At least two distinct seeds are needed for across-seed sample std")
    repro_samples = validate_integer(repro_samples, "repro_samples", minimum=1)
    repro_seed = validate_integer(repro_seed, "repro_seed", minimum=0)
    grid = VoxelGrid() if grid is None else grid
    config = Path(config_path).expanduser().resolve()
    config_hash, versions, code_hashes = file_sha256(config), software_versions(), _code_hashes()
    model = load_model(config)
    if config_hash != file_sha256(config):
        raise ValidationError("Configuration changed during model loading")
    if model.joint_order != JOINT_ORDER or model.base_frame != "base_link" or model.ee_frame != "end_effector_link":
        raise ValidationError("G4 requires the approved base_link -> end_effector_link five-joint model")
    model_hash = model.metadata()["expanded_urdf_sha256"]
    evidence = [prior_evidence(g1a_report, "G1-A", model_hash), prior_evidence(g1b_report, "G1-B", model_hash)]
    visualization_files = []
    if visualization_prefix is not None:
        for suffix in ('_3d.html', '_projections.html'):
            path = Path(str(visualization_prefix) + suffix).expanduser().absolute()
            visualization_files.append({'path': str(path), 'exists': path.is_file(),
                                        **({'sha256': file_sha256(path)} if path.is_file() else {})})
    reused = load_reuse(reuse_prefixes, model, config_hash, versions, {(n, s) for n in counts for s in seeds})
    # Validate reused points against the SAME grid before starting new sampling.
    for _, arrays, _ in reused.values():
        voxel_hits(arrays["position"], grid)
    output = Path(output_dir).expanduser().absolute()
    markdown = Path(report_path).expanduser().absolute() if report_path is not None else output / "g4_validation.md"
    if os.path.lexists(output) or os.path.lexists(markdown):
        raise ValidationError("Refusing to overwrite output directory or Markdown report; choose a new run name")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.mkdir()  # exclusive reservation, including an already-existing empty directory
    marker = output / ".g4_incomplete"
    marker.write_text("IN_PROGRESS; accept validation.json only after this marker is absent\n")

    def generated(prefix, n, seed):
        progress(f"Sampling N={n}, seed={seed}: {prefix}")
        run = sample_workspace(model, samples=n, seed=seed)
        save_results(prefix, run, model, config_path=config, config_sha256=config_hash)
        arrays, metadata = load_experiment(prefix, model, config_hash, versions)
        if any(not np.array_equal(run.arrays()[key], arrays[key]) for key in arrays):
            raise ValidationError("Generated data readback changed array values")
        return arrays, metadata

    try:
        independent, run_info = [], []
        for index in (1, 2):
            prefix = output / "reproducibility" / f"run{index}_n{repro_samples}_seed{repro_seed}"
            arrays, metadata = generated(prefix, repro_samples, repro_seed)
            independent.append(arrays)
            run_info.append({"source_prefix": str(prefix), "samples": repro_samples, "seed": repro_seed,
                             "model_sha256": model_hash, "config_sha256": config_hash,
                             "software_versions": metadata["experiment"]["software_versions"],
                             "npz_sha256": metadata["dataset"]["sha256"],
                             "json_sha256": file_sha256(result_paths(prefix)[1])})
        reproduction = compare_arrays(*independent, model.limits)
        reproduction["runs"] = run_info
        rows, provenance = [], []
        for n in counts:
            for seed in seeds:
                existing = (n, seed) in reused
                if existing:
                    prefix, arrays, metadata = reused[n, seed]
                    progress(f"Reuse verified N={n}, seed={seed}: {prefix}")
                else:
                    prefix = output / "experiments" / f"workspace_n{n}_seed{seed}"
                    arrays, metadata = generated(prefix, n, seed)
                rows.append(experiment_row(prefix, arrays, metadata, grid, reused=existing))
                provenance.append({"source_prefix": str(prefix), "experiment": metadata["experiment"],
                                   "array_schema": metadata["dataset"]["arrays"],
                                   "configuration": metadata["configuration"]})
        aggregates = aggregate_seeds(rows, counts, seeds)
        # Detect changed config, model sources, software or code before final reports.
        if (file_sha256(config) != config_hash or software_versions() != versions or _code_hashes() != code_hashes
                or load_model(config).metadata() != model.metadata()):
            raise ValidationError("Model/configuration/software/code changed during G4")
        for row in rows + run_info:
            prefix = row["source_prefix"]
            if (file_sha256(result_paths(prefix)[0]) != row["npz_sha256"]
                    or file_sha256(result_paths(prefix)[1]) != row["json_sha256"]):
                raise ValidationError(f"Input changed during validation: {prefix}")
        for item in evidence:
            if item["available"] and file_sha256(item["path"]) != item["sha256"]:
                raise ValidationError("Prior evidence changed during G4")
        report = {"schema_version": 1, "stage": "G4", "status": reproduction["status"],
                  "stage_acceptance": "pending_user_review", "hardware_checked": False, "tf_rerun_in_g4": False,
                  "completed_utc": datetime.now(timezone.utc).isoformat(),
                  "experiment_profile": "formal" if (counts == FORMAL_COUNTS and seeds == FORMAL_SEEDS
                                                         and repro_samples == 1000 and repro_seed == 42) else "diagnostic",
                  "design": {"sample_counts": list(counts), "seeds": list(seeds), "algorithm": ALGORITHM,
                             "within_dataset_std_ddof": 0, "across_seed_std_ddof": 1,
                             "nested_samples_note": "Same seed across N uses RNG prefixes; runs are not independent across N",
                             "timing_note": "G2 FK loop scope; reused timings are historical, not a controlled benchmark"},
                  "robot_model": json.loads(json.dumps({**model.metadata(), "joint_limits_rad": model.limits.tolist()})),
                  "configuration": {"path": str(config), "sha256": config_hash},
                  "environment": {"executable": os.sys.executable, "platform": platform.platform(),
                                  "software_versions": versions, "code_sha256": code_hashes},
                  "prior_evidence": evidence, "visualization_files": visualization_files,
                  "reproducibility": reproduction, "grid": grid.metadata(),
                  "experiments": rows, "experiment_provenance": provenance, "across_seed": aggregates,
                  "scope": list(SCOPE_LIMITATIONS) + ["Only three formal seeds: preliminary evidence, no confidence guarantee.",
                                                      "Hit voxels are neither a collision map nor a strict reachable volume."]}
        paths = write_bundle(output, report, markdown)
        marker.unlink()
        return report, paths
    except BaseException as exc:
        # Retain generated evidence; never remove other runs or reused datasets.
        try:
            with (output / "failure.json").open("x", encoding="utf-8") as stream:
                json.dump({"status": "ERROR", "type": type(exc).__name__, "message": str(exc),
                           "stage_acceptance": "pending_user_review"}, stream, ensure_ascii=False, indent=2)
        except OSError:
            pass  # Preserve the original exception even on a full/unwritable disk.
        raise
