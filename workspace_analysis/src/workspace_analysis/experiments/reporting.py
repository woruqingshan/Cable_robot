"""G4 CSV/JSON/Markdown rendering and no-overwrite publication, without ROS."""

import csv
from hashlib import sha256
import io
import json
import os
from pathlib import Path
import tempfile

from .statistics import ROW_FIELDS, ValidationError, aggregate_seeds


AGGREGATE_FIELDS = ("samples", "metric", "seed_count", "mean", "std", "std_ddof")


def csv_text(rows, fields):
    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return buffer.getvalue()


def check_csv_json(text, rows, fields):
    reader = csv.DictReader(io.StringIO(text))
    if reader.fieldnames != list(fields) or list(reader) != [{key: str(row[key]) for key in fields} for row in rows]:
        raise ValidationError("CSV/JSON results disagree")


def _table(headers, rows):
    # Quote paths/text safely as single Markdown cells; numerics retain float precision in JSON/CSV.
    def cell(value):
        return str(value).replace("|", "\\|").replace("\n", " ")
    return "\n".join(["| " + " | ".join(map(cell, headers)) + " |",
                      "| " + " | ".join(["---"] * len(headers)) + " |"] +
                     ["| " + " | ".join(map(cell, row)) + " |" for row in rows])


def stability_text(report):
    counts = report["design"]["sample_counts"]
    if len(counts) < 2:
        return "Diagnostic design has only one N; no sample-count stability comparison was performed."
    lookup = {(r["samples"], r["metric"]): r for r in report["across_seed"]}
    small, large = min(counts), max(counts)
    lines = [f"Observed comparisons for N={small} versus N={large} (descriptive, not a convergence test):"]
    for axis in "xyz":
        lo0, lo1 = (lookup[n, f"{axis}_min_m"]["mean"] for n in (small, large))
        hi0, hi1 = (lookup[n, f"{axis}_max_m"]["mean"] for n in (small, large))
        lines.append(f"- {axis.upper()} across-seed average sampled extrema: [{lo0:.8g}, {hi0:.8g}] -> "
                     f"[{lo1:.8g}, {hi1:.8g}] m. These are sampled extrema, not strict boundaries.")
    decreased = 0
    for axis in "xyz":
        for stat in ("mean", "std"):
            a, b = (lookup[n, f"{axis}_{stat}_m"]["std"] for n in (small, large))
            decreased += b < a
            direction = "decreased" if b < a else "increased" if b > a else "was unchanged"
            lines.append(f"- Across-seed sample SD of {axis.upper()} {stat}: {a:.8g} -> {b:.8g} m; {direction}.")
    lines.append(f"Only {decreased}/6 mean/std dispersion measures decreased in this comparison. "
                 "The table retains residual fluctuations; no universal convergence, strict boundary or confidence guarantee follows.")
    return "\n".join(lines)


def markdown_report(report):
    model, reproduction = report["robot_model"], report["reproducibility"]
    sections = ["# G4 Workspace Validation", "",
                f"Software status: **{report['status']}**; experiment profile: **{report['experiment_profile']}**.",
                "Stage acceptance: **pending_user_review**. Hardware checked: false. P2.2 acceptance is not declared here.",
                f"Completed UTC: {report['completed_utc']}", "", "## 1. Robot Model", "",
                f"Model: {model['source_path']}; {len(model['joint_order'])} independent arm joints.",
                f"Base: `{model['base_frame']}`; EE: `{model['ee_frame']}`; units m/rad.",
                f"Expanded URDF SHA256: `{model['expanded_urdf_sha256']}`.",
                f"Configuration SHA256: `{report['configuration']['sha256']}`.",
                _table(["Joint", "Lower [rad]", "Upper [rad]"],
                       [[name, *limits] for name, limits in zip(model['joint_order'], model['joint_limits_rad'])]),
                "", "## 2. FK Validation — Prior Evidence", "",
                "This section cites previously user-executed files. G4 did not launch RSP or rerun ROS2 TF integration."]
    for evidence in report["prior_evidence"]:
        if not evidence["available"]:
            sections.append(f"{evidence['stage']}: prior file not supplied; no new result is inferred.")
        else:
            sections.extend([f"{evidence['stage']}: {evidence['path']} (SHA256 `{evidence['sha256']}`), "
                             f"reported status {evidence['reported_status']}.",
                             _table(["Case", "Position error [m]", "Rotation error [rad]"],
                                    [[r['name'], r['position_error_m'], r['rotation_error_rad']]
                                     for r in evidence['cases']])])
            if evidence['stage'] == 'G1-B':
                sections.append(f"Prior strict endpoint identity: {evidence['strict_endpoint_identity_verified']}; "
                                f"prior RSP exit codes: {evidence['rsp_exit_codes']}. No new TF run.")
    sections.extend(["", "## 3. Workspace Sampling", "",
                     f"Algorithm: `{report['design']['algorithm']}`. N={report['design']['sample_counts']}; "
                     f"seeds={report['design']['seeds']}.",
                     "The approved G2 sampler and G1-A PyKDL FK are reused unchanged. q is uniform independently "
                     "within URDF limits; Cartesian density is not uniform. All negative-Z samples are retained.",
                     "G2 pairs contain float64 q(N,5), position(N,3), rotation(N,3,3), quaternion_xyzw(N,4). "
                     "Each CSV row identifies its source pair and both file hashes. Full provenance is in validation.json.",
                     report['design']['nested_samples_note'] + ".",
                     "", "## 4. Reproducibility", "", reproduction['criterion'],
                     f"Two independent calls: N={reproduction['runs'][0]['samples']}, "
                     f"seed={reproduction['runs'][0]['seed']}. Result: **{reproduction['status']}**.",
                     _table(["Array", "First shape/dtype", "Second shape/dtype", "Exactly equal", "Max absolute difference"],
                            [[name, f"{r['first_shape']} / {r['first_dtype']}", f"{r['second_shape']} / {r['second_dtype']}",
                              r['array_equal'], r['max_absolute_difference']] for name, r in reproduction['arrays'].items()]),
                     f"Max position error: {reproduction['max_position_error_m']} m; max rotation error: "
                     f"{reproduction['max_rotation_error_rad']} rad; quaternion orientation error: "
                     f"{reproduction['max_quaternion_rotation_error_rad']} rad.",
                     "Quaternion q/-q are orientation-equivalent but do not satisfy deterministic array equality. "
                     "Timestamps, elapsed times and NPZ byte hashes are not required to be equal.",
                     f"Software versions: `{json.dumps(report['environment']['software_versions'], sort_keys=True)}`.",
                     reproduction['scope'] + ".", "", "## 5. Sampling Stability", "",
                     "Within each dataset, position std uses ddof=0. Across seeds, every scalar metric has mean "
                     "and sample std (ddof=1); only three seeds in the formal design, with no confidence guarantee."])
    for axis in "xyz":
        sections.extend([f"### {axis.upper()} statistics [m]", "",
                         _table(["N", "Seed", "Min", "Max", "Mean", "Std (ddof=0)", "AABB edge"],
                                [[r['samples'], r['seed']] + [f"{r[f'{axis}_{s}_m']:.9g}" for s in
                                  ('min', 'max', 'mean', 'std', 'aabb_edge')] for r in report['experiments']])])
    sections.extend(["", _table(["N", "Seed", "Reused", "AABB volume [m3]", "FK loop [s]", "Hit voxels"],
                               [[r['samples'], r['seed'], r['reused'], r['aabb_volume_m3'],
                                 r['fk_elapsed_s'], r['hit_voxels']] for r in report['experiments']]),
                     "AABB volume is only a bounding-box statistic, not reachable workspace volume. "
                     + report['design']['timing_note'] + ".", "",
                     _table(["N", "Metric", "Across-seed mean", "Across-seed SD (ddof=1)"],
                            [[r['samples'], r['metric'], f"{r['mean']:.9g}", f"{r['std']:.9g}"]
                             for r in report['across_seed']]), "", stability_text(report),
                     "", "## 6. Voxel Hit Statistics", "", "```json", json.dumps(report['grid'], indent=2), "```",
                     "One fixed grid is shared across all runs. Out-of-region points cause an error; there is no "
                     "clipping or per-run AABB grid fitting. Unique hit counts are in the table/CSV. "
                     "Empty cells are not proven unreachable; hit cells are not wholly reachable or collision-safe. "
                     "No sum of hit-cell volumes is claimed as true reachable volume.",
                     "", "## 7. Visualization", "",
                     "G3 was previously accepted by the user. It exports offline 3D and XY/XZ/YZ HTML using "
                     "scripts/plot_workspace.py --config config/workspace.yaml --input-prefix <G2-prefix> "
                     "--output-prefix <new-prefix>. G4 does not regenerate these HTML files."])
    for item in report.get('visualization_files', []):
        sections.append(f"- {item['path']} (exists={item['exists']}, SHA256={item.get('sha256', 'not available')}). "
                        "File inventory only; no new visual acceptance.")
    sections.extend(["", "## 8. Limitations", ""] + [f"- {text}" for text in report['scope']] + [
        "- Hardware positioning accuracy is not validated; URDF limits are not measured motor safety limits.",
        "- Base origin is not ground. EE has no calibrated RJ45 TCP or insertion-direction/contact constraints.",
        "", "## 9. Conclusion", "",
        f"This run has software status {report['status']} and profile {report['experiment_profile']}. "
        "It provides model-based sampling, deterministic reproduction checks and descriptive statistics at a stated "
        "voxel resolution. It does not establish a complete planning system or a strict workspace boundary.",
        "User review and formal P2.2 acceptance remain pending."])
    return "\n\n".join(sections) + "\n"


def _publish_texts(documents):
    temporary, published = [], []
    try:
        for path in documents:
            if os.path.lexists(path):
                raise ValidationError(f"Refusing to overwrite: {path}")
        for target, text in documents.items():
            target.parent.mkdir(parents=True, exist_ok=True)
            fd, name = tempfile.mkstemp(prefix=f".{target.name}.", suffix=".tmp", dir=target.parent)
            temporary.append(Path(name))
            with os.fdopen(fd, 'w', encoding='utf-8', newline='') as stream:
                stream.write(text)
                stream.flush()
                os.fsync(stream.fileno())
        # Validation JSON is the last commit marker. Each link refuses replacement.
        for staged, target in zip(temporary, documents):
            info = staged.stat()
            os.link(staged, target)
            published.append((target, (info.st_dev, info.st_ino)))
    except BaseException:
        for path, identity in reversed(published):
            try:
                info = path.lstat()
                if (info.st_dev, info.st_ino) == identity:
                    path.unlink()
            except FileNotFoundError:
                pass
        raise
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)


def write_bundle(output_dir, report, markdown_path):
    output = Path(output_dir)
    paths = {'experiments_csv': output / 'experiments.csv', 'across_seed_csv': output / 'across_seed.csv',
             'markdown': Path(markdown_path), 'json': output / 'validation.json'}
    if len({p.absolute() for p in paths.values()}) != len(paths):
        raise ValidationError("Output paths must be distinct")
    primary = csv_text(report['experiments'], ROW_FIELDS)
    aggregate = csv_text(report['across_seed'], AGGREGATE_FIELDS)
    check_csv_json(primary, report['experiments'], ROW_FIELDS)
    check_csv_json(aggregate, report['across_seed'], AGGREGATE_FIELDS)
    markdown = markdown_report(report)
    content = {paths['experiments_csv']: primary, paths['across_seed_csv']: aggregate, paths['markdown']: markdown}
    report['artifacts'] = {key: {'path': str(paths[key].absolute()),
                               'sha256': sha256(text.encode('utf-8')).hexdigest()}
                           for key, text in [('experiments_csv', primary), ('across_seed_csv', aggregate), ('markdown', markdown)]}
    content[paths['json']] = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    _publish_texts(content)
    stored = json.loads(paths['json'].read_text(encoding='utf-8'))
    if stored != report:
        raise ValidationError("Validation JSON readback differs")
    for key, rows, fields in [('experiments_csv', report['experiments'], ROW_FIELDS),
                              ('across_seed_csv', report['across_seed'], AGGREGATE_FIELDS)]:
        check_csv_json(paths[key].read_text(encoding='utf-8'), rows, fields)
    return {key: str(path.absolute()) for key, path in paths.items()}


def load_validation_bundle(output_dir):
    """Accept only completed bundles, verifying CSV/JSON/Markdown file hashes."""
    from ..workspace.artifacts import file_sha256
    output = Path(output_dir).expanduser().resolve()
    if (output / '.g4_incomplete').exists() or (output / 'failure.json').exists():
        raise ValidationError("Incomplete/failed G4 run")
    report = json.loads((output / 'validation.json').read_text(encoding='utf-8'))
    if (report['schema_version'] != 1 or report['stage'] != 'G4' or report['status'] not in ('PASS', 'FAIL')
            or report['stage_acceptance'] != 'pending_user_review' or report['hardware_checked'] is not False
            or report['tf_rerun_in_g4'] is not False):
        raise ValidationError("Invalid G4 status/schema")
    expected = 'PASS' if all(item['array_equal'] is True for item in report['reproducibility']['arrays'].values()) else 'FAIL'
    if (set(report['reproducibility']['arrays']) != {'q', 'position', 'rotation', 'quaternion_xyzw'}
            or report['status'] != expected or report['reproducibility']['status'] != expected):
        raise ValidationError('Inconsistent reproducibility status')
    if aggregate_seeds(report['experiments'], report['design']['sample_counts'], report['design']['seeds']) != report['across_seed']:
        raise ValidationError('Across-seed aggregates disagree with experiment rows')
    for artifact in report['artifacts'].values():
        if file_sha256(artifact['path']) != artifact['sha256']:
            raise ValidationError("G4 artifact hash mismatch")
    for key, rows, fields in [('experiments_csv', report['experiments'], ROW_FIELDS),
                              ('across_seed_csv', report['across_seed'], AGGREGATE_FIELDS)]:
        check_csv_json(Path(report['artifacts'][key]['path']).read_text(encoding='utf-8'), rows, fields)
    return report
