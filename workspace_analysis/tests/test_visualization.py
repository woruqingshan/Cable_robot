"""G3 small-data tests; Plotly-dependent cases are explicit SKIPs if absent.

File tests COPY an existing G2 100-row pair, never generate replacement data.
G3_TEST_INPUT_PREFIX may point to that pair when testing a staging checkout.
"""

import builtins
from hashlib import sha256
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

import numpy as np
import pytest
import yaml

from workspace_analysis.results import ResultsError
from workspace_analysis import visualization as viz


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config/workspace.yaml"


@pytest.fixture
def synthetic():
    position = np.array([[-0.2, 0.1, -0.1], [0.1, -0.3, 0.2], [0.4, 0.2, 0.3],
                         [-0.1, -0.2, -0.2], [0.3, 0.4, 0.1]], dtype=np.float64)
    q = np.arange(25, dtype=np.float64).reshape(5, 5) / 50
    arrays = {"q": q, "position": position, "rotation": np.repeat(np.eye(3)[None], 5, axis=0),
              "quaternion_xyzw": np.tile([0., 0., 0., 1.], (5, 1))}
    metadata = {"robot_model": {"joint_order": [f"joint{i}" for i in range(1, 6)],
                                "joint_limits_rad": [[-1., 1.]] * 5,
                                "base_frame": "base_link", "ee_frame": "end_effector_link",
                                "xacro_environment": {"MACHINE_TYPE": "SyntheticModel"},
                                "source_path": "/test/synthetic.xacro"},
                "experiment": {"completed_samples": 5, "seed": 42},
                "dataset": {"sha256": "a" * 64}}
    return arrays, metadata


@pytest.fixture
def source_prefix():
    prefix = Path(os.environ.get("G3_TEST_INPUT_PREFIX", ROOT / "results/workspace_n100_seed42"))
    if not all(Path(str(prefix) + ext).is_file() for ext in (".npz", ".json")):
        pytest.fail(f"G3 needs the existing validated G2 100-row pair: {prefix}. "
                    "Supply G3_TEST_INPUT_PREFIX; tests do not silently resample missing inputs.")
    return prefix


@pytest.fixture
def copied_pair(source_prefix, tmp_path):
    prefix = tmp_path / source_prefix.name
    for extension in (".npz", ".json"):
        shutil.copyfile(Path(str(source_prefix) + extension), Path(str(prefix) + extension))
    return prefix


@pytest.fixture
def plotly_ready():
    return pytest.importorskip("plotly", reason="Plotly missing: figure/HTML generation has NOT been validated")


def test_full_view_preserves_all_rows_negative_z_and_source_arrays(synthetic):
    arrays, metadata = synthetic
    before = {key: value.copy() for key, value in arrays.items()}
    view = viz.prepare_view(arrays, metadata)
    np.testing.assert_array_equal(view.sample_indices, np.arange(5))
    np.testing.assert_array_equal(view.position, arrays["position"])
    np.testing.assert_array_equal(view.customdata[:, 0], np.arange(5))
    np.testing.assert_array_equal(view.customdata[:, 1:4], arrays["position"])
    np.testing.assert_array_equal(view.customdata[:, 4:], arrays["q"])
    assert (view.position[:, 2] < 0).sum() == 2
    for key in arrays:
        np.testing.assert_array_equal(before[key], arrays[key])


def test_deterministic_display_selection_keeps_original_npz_indices(synthetic):
    arrays, metadata = synthetic
    first = viz.prepare_view(arrays, metadata, max_points=3)
    second = viz.prepare_view(arrays, metadata, max_points=3)
    np.testing.assert_array_equal(first.sample_indices, [0, 2, 4])
    np.testing.assert_array_equal(first.sample_indices, second.sample_indices)
    np.testing.assert_array_equal(first.position, arrays["position"][[0, 2, 4]])
    np.testing.assert_array_equal(first.q, arrays["q"][[0, 2, 4]])
    assert first.total_samples == 5


@pytest.mark.parametrize("maximum", [0, -1, True, 1.5, "3"])
def test_invalid_display_limit_rejected(synthetic, maximum):
    with pytest.raises(viz.VisualizationError):
        viz.prepare_view(*synthetic, max_points=maximum)


@pytest.mark.parametrize("change", ["shape", "nan", "metadata_count", "joint_names"])
def test_invalid_data_is_not_ignored(synthetic, change):
    arrays, metadata = synthetic
    if change == "shape":
        arrays["position"] = arrays["position"][:, :2]
    elif change == "nan":
        arrays["position"][0, 0] = np.nan
    elif change == "metadata_count":
        metadata["experiment"]["completed_samples"] = 9
    else:
        metadata["robot_model"]["joint_order"] = ["joint1"]
    with pytest.raises(viz.VisualizationError):
        viz.prepare_view(arrays, metadata)


def test_g2_loader_is_reused_with_model_and_real_100_row_pair(copied_pair, monkeypatch):
    actual = viz.load_results
    calls = []
    def observed(prefix, *, model):
        calls.append((prefix, model))
        return actual(prefix, model=model)
    monkeypatch.setattr(viz, "load_results", observed)
    view = viz.load_workspace(copied_pair, CONFIG)
    assert len(calls) == 1 and calls[0][1].base_frame == "base_link"
    assert len(view.position) == view.total_samples == 100
    np.testing.assert_array_equal(view.sample_indices, np.arange(100))


def test_loading_does_not_change_source_npz_json(copied_pair):
    paths = [Path(str(copied_pair) + suffix) for suffix in (".npz", ".json")]
    before = [sha256(path.read_bytes()).hexdigest() for path in paths]
    viz.load_workspace(copied_pair, CONFIG)
    assert before == [sha256(path.read_bytes()).hexdigest() for path in paths]


@pytest.mark.parametrize("damage", ["missing", "hash", "npz", "json"])
def test_g2_file_validation_errors_stop_before_plotting(copied_pair, damage):
    npz, meta = Path(str(copied_pair) + ".npz"), Path(str(copied_pair) + ".json")
    if damage == "missing":
        npz.unlink()
    elif damage in ("hash", "npz"):
        npz.write_bytes(b"not-an-npz" if damage == "npz" else npz.read_bytes() + b"changed")
    else:
        meta.write_text("invalid json")
    with pytest.raises(ResultsError):
        viz.load_workspace(copied_pair, CONFIG)


def test_model_mismatch_is_rejected(copied_pair, tmp_path):
    config = yaml.safe_load(CONFIG.read_text())
    config["model"]["base_frame"] = "base_footprint"
    changed = tmp_path / "different.yaml"
    changed.write_text(yaml.safe_dump(config))
    with pytest.raises(ResultsError, match="model identity mismatch"):
        viz.load_workspace(copied_pair, changed)


def test_missing_plotly_has_explicit_error_and_no_installation(monkeypatch):
    actual = builtins.__import__
    def missing(name, *args, **kwargs):
        if name == "plotly" or name.startswith("plotly."):
            raise ModuleNotFoundError("simulated absent Plotly")
        return actual(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", missing)
    with pytest.raises(viz.MissingPlotlyError, match="No dependencies were installed"):
        viz._plotly_modules()


def test_output_paths_and_conflicts_preserve_existing_files(tmp_path):
    prefix = tmp_path / "workspace"
    paths = viz.html_paths(prefix)
    assert [path.name for path in paths[:2]] == ["workspace_3d.html", "workspace_projections.html"]
    for conflict in paths:
        conflict.write_bytes(b"keep")
        with pytest.raises(viz.VisualizationError, match="Refusing to overwrite"):
            viz.check_html_available(prefix)
        assert conflict.read_bytes() == b"keep"
        conflict.unlink()


def test_publication_transaction_failure_rolls_back_only_its_files(tmp_path, monkeypatch):
    # Filesystem-only test strings are NOT renderer/HTML acceptance evidence.
    original = os.link
    def fail_second(source, destination):
        if str(destination).endswith("_projections.html"):
            raise OSError("second publication failed")
        original(source, destination)
    monkeypatch.setattr(os, "link", fail_second)
    with pytest.raises(viz.VisualizationError, match="second publication failed"):
        viz._publish_documents(tmp_path / "failed", ("filesystem unit payload A", "filesystem unit payload B"))
    assert list(tmp_path.iterdir()) == []


def test_cli_bad_input_returns_nonzero_without_creating_html(tmp_path):
    result = subprocess.run([sys.executable, str(ROOT / "scripts/plot_workspace.py"),
                             "--config", str(CONFIG), "--input-prefix", str(tmp_path / "missing"),
                             "--output-prefix", str(tmp_path / "output")],
                            cwd=tmp_path, capture_output=True, text=True, timeout=30)
    assert result.returncode != 0 and "status: ERROR" in result.stderr
    assert not list(tmp_path.glob("*.html"))


def test_cli_missing_plotly_does_not_claim_pass(copied_pair, tmp_path, monkeypatch, capsys):
    path = ROOT / "scripts/plot_workspace.py"
    spec = importlib.util.spec_from_file_location("g3_cli_unit", path)
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    def missing():
        raise viz.MissingPlotlyError("unit test: Plotly unavailable")
    monkeypatch.setattr(viz, "_plotly_modules", missing)
    monkeypatch.setattr(sys, "argv", [str(path), "--config", str(CONFIG), "--input-prefix", str(copied_pair),
                                     "--output-prefix", str(tmp_path / "no_plotly")])
    assert cli.main() == 1
    captured = capsys.readouterr()
    assert "status: ERROR" in captured.err and "status: PASS" not in captured.out
    assert not list(tmp_path.glob("*.html"))


def test_render_failure_does_not_publish_either_html(synthetic, tmp_path, monkeypatch):
    # Inject a renderer failure, without constructing a mock Plotly figure.
    def broken(view):
        raise viz.VisualizationError("renderer failed")
    monkeypatch.setattr(viz, "build_3d_figure", broken)
    with pytest.raises(viz.VisualizationError, match="renderer failed"):
        viz.export_workspace(viz.prepare_view(*synthetic), tmp_path / "render_error")
    assert list(tmp_path.iterdir()) == []


def test_3d_coordinates_hover_color_origin_and_scale(plotly_ready, synthetic):
    view = viz.prepare_view(*synthetic)
    figure = viz.build_3d_figure(view)
    samples, origin = figure.data
    assert samples.type == "scatter3d" and len(samples.x) == 5
    for axis in range(3):
        np.testing.assert_array_equal((samples.x, samples.y, samples.z)[axis], view.position[:, axis])
    np.testing.assert_array_equal(samples.marker.color, view.position[:, 2])
    np.testing.assert_array_equal(samples.customdata[:, 4:], view.q)
    assert "Sample Index" in samples.hovertemplate
    for name in view.joint_order:
        assert name in samples.hovertemplate
    assert "rad" in samples.hovertemplate
    assert tuple(origin.x) == tuple(origin.y) == tuple(origin.z) == (0,)
    assert origin.name != samples.name and origin.marker.symbol != samples.marker.symbol
    assert figure.layout.scene.aspectmode == "data"
    assert figure.layout.scene.xaxis.title.text == "X [m]"
    assert figure.layout.scene.yaxis.title.text == "Y [m]"
    assert figure.layout.scene.zaxis.title.text == "Z [m]"
    assert samples.marker.showscale


def test_display_subset_figure_labels_and_hover_use_original_rows(plotly_ready, synthetic):
    view = viz.prepare_view(*synthetic, max_points=3)
    figure = viz.build_3d_figure(view)
    np.testing.assert_array_equal(figure.data[0].customdata[:, 0], [0, 2, 4])
    np.testing.assert_array_equal(figure.data[0].customdata[:, 4:], synthetic[0]["q"][[0, 2, 4]])
    assert "3 shown / 5 total" in figure.layout.title.text
    assert figure.layout.meta["selection"] == "deterministic linspace indices; no resampling"


def test_projection_axes_same_rows_and_equal_metric_scales(plotly_ready, synthetic):
    view = viz.prepare_view(*synthetic)
    figure = viz.build_projections_figure(view)
    spans = []
    for index, (a, b) in enumerate(((0, 1), (0, 2), (1, 2)), 1):
        samples, origin = figure.data[2 * (index - 1):2 * index]
        np.testing.assert_array_equal(samples.x, view.position[:, a])
        np.testing.assert_array_equal(samples.y, view.position[:, b])
        np.testing.assert_array_equal(samples.customdata[:, 0], view.sample_indices)
        np.testing.assert_array_equal(samples.marker.color, view.position[:, 2])
        assert tuple(origin.x) == tuple(origin.y) == (0,)
        suffix = "" if index == 1 else str(index)
        xaxis, yaxis = figure.layout[f"xaxis{suffix}"], figure.layout[f"yaxis{suffix}"]
        assert yaxis.scaleanchor == f"x{suffix}" and yaxis.scaleratio == 1
        assert xaxis.title.text == f"{'XYZ'[a]} [m]" and yaxis.title.text == f"{'XYZ'[b]} [m]"
        spans.extend([xaxis.range[1] - xaxis.range[0], yaxis.range[1] - yaxis.range[0]])
    np.testing.assert_allclose(spans, spans[0], atol=1e-12)
    captions = " ".join(annotation.text for annotation in figure.layout.annotations)
    assert "not a strict boundary" in captions and "Negative Z is retained" in captions


def test_real_html_export_embeds_plotly_and_has_no_external_script(plotly_ready, synthetic, tmp_path):
    view = viz.prepare_view(*synthetic)
    outputs = viz.export_workspace(view, tmp_path / "html")
    for path in outputs:
        text = path.read_text(encoding="utf-8")
        parser = viz._ScriptLinks()
        parser.feed(text)
        assert path.stat().st_size > 10000 and "Plotly.newPlot" in text and "plotly.js" in text
        assert not parser.sources
    before = [path.read_bytes() for path in outputs]
    with pytest.raises(viz.VisualizationError, match="Refusing to overwrite"):
        viz.export_workspace(view, tmp_path / "html")
    assert before == [path.read_bytes() for path in outputs]


def test_real_cli_keeps_source_npz_json_unchanged(plotly_ready, copied_pair, tmp_path):
    inputs = [Path(str(copied_pair) + suffix) for suffix in (".npz", ".json")]
    before = [sha256(path.read_bytes()).hexdigest() for path in inputs]
    result = subprocess.run([sys.executable, str(ROOT / "scripts/plot_workspace.py"),
                             "--config", str(CONFIG), "--input-prefix", str(copied_pair),
                             "--output-prefix", str(tmp_path / "figure")],
                            cwd=tmp_path, text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr
    assert "status: PASS" in result.stdout and "100 displayed / 100 total" in result.stdout
    assert before == [sha256(path.read_bytes()).hexdigest() for path in inputs]
