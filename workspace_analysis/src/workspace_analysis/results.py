"""Validated NPZ/JSON pairs; JSON is the commit marker, lock blocks readers.

Publication uses same-directory hard links for atomic NO-REPLACE semantics on
Ubuntu. Two filenames cannot be committed in one filesystem operation. Normal
failures roll back only owned files; interrupted transactions fail closed.
"""

from hashlib import sha256
from datetime import datetime
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import tempfile

import numpy as np

from .sampling import (ALGORITHM, SCOPE_LIMITATIONS,
                       checked_limits, recheck_fk, validate_arrays, validate_integer)


class ResultsError(ValueError):
    """A failed transaction, malformed dataset or inconsistent metadata."""


def file_sha256(path):
    digest = sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def result_paths(prefix):
    prefix = Path(prefix).expanduser()
    if prefix.name in ("", ".", "..") or prefix.suffix in (".npz", ".json") or prefix.is_dir():
        raise ResultsError("output-prefix must be a filename prefix, without .npz/.json")
    prefix = prefix.absolute()
    return Path(str(prefix) + ".npz"), Path(str(prefix) + ".json"), Path(str(prefix) + ".lock")


def check_output_available(prefix):
    paths = result_paths(prefix)
    for path in paths:
        if os.path.lexists(path):
            raise ResultsError(f"Refusing to overwrite existing file/transaction: {path}")
    return paths


def workspace_statistics(position):
    try:
        with np.errstate(over="raise", invalid="raise"):
            minimum, maximum = position.min(axis=0), position.max(axis=0)
            edge = maximum - minimum
            result = {"axis_order": ["x", "y", "z"], "min_m": minimum.tolist(),
                      "max_m": maximum.tolist(), "mean_m": position.mean(axis=0).tolist(),
                      "std_m": position.std(axis=0, ddof=0).tolist(), "std_ddof": 0,
                      "aabb_edge_lengths_m": edge.tolist(), "aabb_volume_m3": float(np.prod(edge)),
                      "volume_meaning": "axis-aligned bounding box only; NOT reachable workspace volume"}
        json.dumps(result, allow_nan=False)
        return result
    except Exception as exc:
        raise ResultsError(f"Cannot compute finite workspace statistics: {exc}") from exc


def _metadata(run, model, config_path, expected_config_sha256, npz_path, npz_hash, verification):
    seed = validate_integer(run.seed, "seed", minimum=0)
    if not isinstance(run.bit_generator, str) or not run.bit_generator:
        raise ResultsError("Only runs with recorded random-generator provenance can be saved")
    if isinstance(run.fk_elapsed_s, bool) or not math.isfinite(run.fk_elapsed_s) or run.fk_elapsed_s < 0:
        raise ResultsError("FK elapsed time must be finite and nonnegative")
    if datetime.fromisoformat(run.completed_utc).tzinfo is None:
        raise ResultsError("Completion timestamp requires a timezone")
    config = Path(config_path).expanduser().resolve()
    config_hash = file_sha256(config)
    if config_hash != expected_config_sha256:
        raise ResultsError("Configuration changed since model loading; refusing mismatched provenance")
    # G1 metadata contains tuples in joint geometry. Normalize only the G2
    # representation so the returned metadata exactly matches its JSON readback.
    identity = json.loads(json.dumps(model.metadata(), allow_nan=False))
    identity["joint_limits_rad"] = checked_limits(model.limits).tolist()
    versions = {"python": platform.python_version(), "numpy": np.__version__}
    import PyKDL
    versions["PyKDL"] = getattr(PyKDL, "__version__", "unknown")
    for name in ("scipy", "urdfdom_py", "xacro", "PyYAML"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = "unknown"
    arrays = run.arrays()
    return {
        "schema_version": 1, "stage": "G2", "status": "PASS",
        "stage_acceptance": "pending_user_review", "hardware_checked": False,
        "experiment": {"sampling_algorithm": ALGORITHM, "requested_samples": len(run.q),
                       "completed_samples": len(run.q), "seed": seed, "completed_utc": run.completed_utc,
                       "fk_elapsed_s": run.fk_elapsed_s,
                       "timing_scope": "sequential FK, pose validation, quaternion conversion and progress callbacks",
                       "software_versions": versions},
        "robot_model": identity,
        "configuration": {"source": str(config), "sha256": config_hash},
        "dataset": {"npz_file": npz_path.name, "sha256": npz_hash,
                    "arrays": {name: {"shape": list(value.shape), "dtype": str(value.dtype)}
                               for name, value in arrays.items()},
                    "units": {"length": "m", "angle": "rad"},
                    "quaternion_order": "xyzw", "quaternion_sign": "q and -q represent the same rotation"},
        "workspace_statistics": workspace_statistics(run.position),
        "reproducibility": {"generator": "numpy.random.Generator", "constructor": "numpy.random.default_rng",
                            "bit_generator": run.bit_generator, "seed": seed, "numpy_version": np.__version__,
                            "expanded_urdf_sha256": identity["expanded_urdf_sha256"],
                            "config_sha256": config_hash, "npz_sha256": npz_hash,
                            "comparison": "array values in the same software environment; NPZ byte equality not required"},
        "validation": {"all_rows_validated": True, "all_joint_limits_checked": True,
                       "quaternion_rotation_consistency_checked": True, "fk_recheck": verification},
        "scope": {"definition": "joint-limit-constrained geometric reachable workspace sampling",
                  "collision_checked": False, "rj45_task_constraints_checked": False,
                  "strict_boundary_proven": False, "calibrated_tcp": False,
                  "limitations": list(SCOPE_LIMITATIONS)},
    }


def _read_npz(path):
    with np.load(path, allow_pickle=False) as archive:
        expected = {"q", "position", "rotation", "quaternion_xyzw"}
        if len(archive.files) != len(expected) or set(archive.files) != expected:
            raise ResultsError("NPZ has missing, duplicate or unexpected arrays")
        return {name: archive[name].copy() for name in expected}


def _owned_unlink(path, identity):
    try:
        info = path.lstat()
        if (info.st_dev, info.st_ino) == identity:
            path.unlink()
    except FileNotFoundError:
        pass


def _identity(path):
    info = path.stat()
    return info.st_dev, info.st_ino


def _sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _reject_json_constant(value):
    raise ResultsError(f"Invalid JSON number {value}")


def save_results(prefix, run, model, *, config_path, config_sha256):
    """Validate, recheck FK, stage and publish a pair without overwriting.

    Caller captures config_sha256 around model loading. No FK errors are hidden;
    JSON status PASS means software checks, never user stage acceptance.
    """
    temporary, published = [], []
    lock_identity = None
    lock_path = None
    try:
        validate_arrays(run.arrays(), model.limits)
        verification = recheck_fk(model, run.arrays())
        npz_path, json_path, lock_path = check_output_available(prefix)
        npz_path.parent.mkdir(parents=True, exist_ok=True)
        # The two actual staging files test writability before publication.
        with lock_path.open("xb") as stream:
            lock_identity = _identity(lock_path)
            stream.write(f"G2 WRITING pid={os.getpid()}\n".encode())
            stream.flush()
            os.fsync(stream.fileno())
        for destination in (npz_path, json_path):
            fd, name = tempfile.mkstemp(prefix=f".{destination.name}.", suffix=".tmp", dir=destination.parent)
            path = Path(name)
            temporary.append(path)
            os.close(fd)
        # Recheck both final names under the cooperating-writer lock.
        if any(os.path.lexists(p) for p in (npz_path, json_path)):
            raise ResultsError("A target file appeared while preparing the output transaction")
        with temporary[0].open("wb") as stream:
            np.savez(stream, **run.arrays())
            stream.flush()
            os.fsync(stream.fileno())
        stored = _read_npz(temporary[0])
        validate_arrays(stored, model.limits)
        if any(not np.array_equal(stored[name], run.arrays()[name]) for name in stored):
            raise ResultsError("NPZ staging readback differs from the source arrays")
        metadata = _metadata(run, model, config_path, config_sha256, npz_path,
                             file_sha256(temporary[0]), verification)
        with temporary[1].open("w", encoding="utf-8") as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        # link() is atomic and refuses an existing destination (unlike replace).
        # JSON is the last-published commit marker; readers also reject the lock.
        for staged, destination in zip(temporary, (npz_path, json_path)):
            identity = _identity(staged)
            os.link(staged, destination)
            published.append((destination, identity))
        _sync_directory(npz_path.parent)
        # Keep the lock until staging cleanup succeeds. A cleanup error still
        # enters rollback, rather than returning an apparently successful pair.
        for path in temporary:
            path.unlink()
        temporary.clear()
        _owned_unlink(lock_path, lock_identity)
        lock_identity = None
        return metadata
    except BaseException as exc:
        for path, identity in reversed(published):
            _owned_unlink(path, identity)
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        raise ResultsError(f"Cannot save result pair: {exc}") from exc
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)
        if lock_path is not None and lock_identity is not None:
            _owned_unlink(lock_path, lock_identity)


def load_results(prefix, *, model=None):
    """Reject incomplete/corrupt pairs; optionally recheck identity and FK."""
    try:
        npz_path, json_path, lock_path = result_paths(prefix)
        if os.path.lexists(lock_path):
            raise ResultsError(f"Incomplete/in-progress transaction lock: {lock_path}")
        if not npz_path.is_file() or not json_path.is_file():
            raise ResultsError("Incomplete result pair: both NPZ and JSON are required")
        metadata = json.loads(json_path.read_text(encoding="utf-8"), parse_constant=_reject_json_constant)
        if metadata["schema_version"] != 1 or metadata["stage"] != "G2" or metadata["status"] != "PASS":
            raise ResultsError("Unsupported or incomplete G2 metadata")
        if metadata["stage_acceptance"] != "pending_user_review" or metadata["hardware_checked"] is not False:
            raise ResultsError("Invalid stage/hardware claims in metadata")
        dataset, experiment = metadata["dataset"], metadata["experiment"]
        for block, keys in (
            (experiment, ("sampling_algorithm", "requested_samples", "completed_samples", "seed",
                          "completed_utc", "fk_elapsed_s", "software_versions", "timing_scope")),
            (metadata["robot_model"], ("base_frame", "ee_frame", "joint_order", "joint_limits_rad", "source_path",
                                       "source_hashes", "expanded_urdf_sha256", "xacro_environment", "auxiliary_positions")),
        ):
            if not isinstance(block, dict) or any(key not in block for key in keys):
                raise ResultsError("Incomplete experiment or robot-model metadata")
        for key in ("requested_samples", "completed_samples"):
            validate_integer(experiment[key], key, minimum=1)
        if (isinstance(experiment["fk_elapsed_s"], bool) or not math.isfinite(experiment["fk_elapsed_s"])
                or experiment["fk_elapsed_s"] < 0):
            raise ResultsError("Invalid FK elapsed time")
        if datetime.fromisoformat(experiment["completed_utc"]).tzinfo is None:
            raise ResultsError("Completion timestamp requires a timezone")
        frames = metadata["robot_model"]
        if any(not isinstance(frames[k], str) or not frames[k] for k in ("base_frame", "ee_frame")):
            raise ResultsError("Missing Base/EE frame identity")
        if dataset["npz_file"] != npz_path.name or dataset["sha256"] != file_sha256(npz_path):
            raise ResultsError("NPZ filename/hash mismatch")
        arrays = _read_npz(npz_path)
        limits = checked_limits(metadata["robot_model"]["joint_limits_rad"])
        shapes = validate_arrays(arrays, limits)
        count = len(arrays["q"])
        if (experiment["requested_samples"] != count or experiment["completed_samples"] != count
                or experiment["sampling_algorithm"] != ALGORITHM):
            raise ResultsError("Metadata sample count/algorithm mismatch")
        validate_integer(experiment["seed"], "seed", minimum=0)
        if (len(metadata["robot_model"]["joint_order"]) != len(limits)
                or len(set(metadata["robot_model"]["joint_order"])) != len(limits)):
            raise ResultsError("Metadata joint order/limit dimension mismatch")
        if dataset["units"] != {"length": "m", "angle": "rad"} or dataset["quaternion_order"] != "xyzw":
            raise ResultsError("Invalid dataset units or quaternion convention")
        expected_arrays = {name: {"shape": list(shape), "dtype": "float64"} for name, shape in shapes.items()}
        if dataset["arrays"] != expected_arrays:
            raise ResultsError("Metadata array shapes/dtypes mismatch")
        if metadata["workspace_statistics"] != workspace_statistics(arrays["position"]):
            raise ResultsError("Workspace statistics disagree with stored positions")
        scope = metadata["scope"]
        if (any(scope[k] is not False for k in ("collision_checked", "rj45_task_constraints_checked",
                                                "strict_boundary_proven", "calibrated_tcp"))
                or scope["limitations"] != SCOPE_LIMITATIONS):
            raise ResultsError("Invalid geometric-workspace scope claims")
        verification = metadata["validation"]
        if any(verification[k] is not True for k in ("all_rows_validated", "all_joint_limits_checked",
                                                      "quaternion_rotation_consistency_checked")):
            raise ResultsError("Incomplete validation claims")
        if verification["fk_recheck"]["indices"] != sorted({0, count // 2, count - 1}):
            raise ResultsError("Unexpected FK verification indices")
        check = verification["fk_recheck"]
        for error_key, tolerance_key, tolerance in (
            ("max_position_error_m", "position_tolerance_m", 1e-5),
            ("max_rotation_error_rad", "rotation_tolerance_rad", 1e-4),
        ):
            error = check[error_key]
            if (check[tolerance_key] != tolerance or isinstance(error, bool)
                    or not math.isfinite(error) or not 0 <= error <= tolerance):
                raise ResultsError("Invalid FK verification evidence")
        provenance = metadata["reproducibility"]
        if (provenance["generator"] != "numpy.random.Generator"
                or provenance["constructor"] != "numpy.random.default_rng"
                or not isinstance(provenance["bit_generator"], str) or not provenance["bit_generator"]
                or provenance["numpy_version"] != experiment["software_versions"]["numpy"]):
            raise ResultsError("Invalid random-generator provenance")
        if (provenance["seed"] != experiment["seed"] or provenance["npz_sha256"] != dataset["sha256"]
                or provenance["config_sha256"] != metadata["configuration"]["sha256"]
                or provenance["expanded_urdf_sha256"] != metadata["robot_model"]["expanded_urdf_sha256"]):
            raise ResultsError("Inconsistent reproducibility metadata")
        if model is not None:
            expected = model.metadata()
            for key in ("base_frame", "ee_frame", "joint_order", "auxiliary_positions", "expanded_urdf_sha256"):
                if metadata["robot_model"][key] != expected[key]:
                    raise ResultsError(f"Loaded model identity mismatch: {key}")
            if not np.array_equal(limits, model.limits):
                raise ResultsError("Loaded model joint limits differ from the dataset")
            recheck_fk(model, arrays)
        # Detect a cooperating writer that began during reading too.
        if os.path.lexists(lock_path):
            raise ResultsError("Result pair changed while reading")
        return arrays, metadata
    except ResultsError:
        raise
    except Exception as exc:
        raise ResultsError(f"Cannot read result pair: {type(exc).__name__}: {exc}") from exc
