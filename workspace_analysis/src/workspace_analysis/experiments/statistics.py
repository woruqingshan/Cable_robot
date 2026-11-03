"""G4 row schema, shared errors and across-seed statistics; NumPy only."""

import numpy as np


METRICS = tuple(f"{axis}_{stat}_m" for axis in "xyz" for stat in ("min", "max", "mean", "std", "aabb_edge")) + (
    "aabb_volume_m3", "fk_elapsed_s", "hit_voxels")
ROW_FIELDS = ("samples", "seed", "reused", "source_prefix", "npz_sha256", "json_sha256",
              "model_sha256", "config_sha256") + METRICS


class ValidationError(ValueError):
    """Invalid evidence, an incomparable experiment, or failed G4 execution."""


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


