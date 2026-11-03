"""G3 views of G2-validated data. No sampling, batch FK or ROS nodes.

Plotly is imported lazily: data validation/indexing tests still run when the
optional plotting dependency is absent. Missing Plotly never becomes a PASS.
"""

from dataclasses import dataclass
from html import escape
from html.parser import HTMLParser
import os
from pathlib import Path
import tempfile

import numpy as np

from ..workspace.results import load_results
from ..workspace.sampling import validate_arrays, validate_integer


class VisualizationError(ValueError):
    """Invalid input, missing plotting support or failed HTML publication."""


class MissingPlotlyError(VisualizationError):
    """Plotly must be provided by the user; never install automatically."""


def _plotly_modules():
    try:
        import plotly.graph_objects as go
        import plotly.io as pio
        from plotly.subplots import make_subplots
        return go, pio, make_subplots
    except ImportError as exc:
        raise MissingPlotlyError(
            "Plotly is unavailable in this Python environment. No dependencies were installed. "
            "Provide a compatible Plotly environment, then rerun G3 tests and visualization.") from exc


@dataclass(frozen=True)
class WorkspaceView:
    position: np.ndarray
    q: np.ndarray
    sample_indices: np.ndarray
    total_samples: int
    metadata: dict
    source_prefix: str
    full_min: np.ndarray
    full_max: np.ndarray

    @property
    def joint_order(self):
        return self.metadata["robot_model"]["joint_order"]

    @property
    def customdata(self):
        # Columns: original NPZ row index, X, Y, Z, then all joint angles.
        return np.column_stack((self.sample_indices, self.position, self.q))

    @property
    def model_name(self):
        model = self.metadata["robot_model"]
        return (model["xacro_environment"].get("MACHINE_TYPE")
                or (Path(model["source_path"]).stem if model.get("source_path") else "URDF model"))


def prepare_view(arrays, metadata, *, source_prefix="", max_points=None):
    """Use G2's array validator; select original rows deterministically."""
    try:
        validate_arrays(arrays, metadata["robot_model"]["joint_limits_rad"])
        count = len(arrays["position"])
        if metadata["experiment"]["completed_samples"] != count:
            raise VisualizationError("Metadata sample count differs from positions")
        if len(metadata["robot_model"]["joint_order"]) != arrays["q"].shape[1]:
            raise VisualizationError("Joint names differ from q columns")
        selected = count if max_points is None else min(validate_integer(max_points, "max_points", minimum=1), count)
        indices = (np.arange(count, dtype=np.int64) if selected == count else
                   np.linspace(0, count - 1, selected, dtype=np.int64))
        # No filtering by Z, collision assumptions or random reordering.
        return WorkspaceView(arrays["position"][indices].copy(), arrays["q"][indices].copy(),
                             indices, count, metadata, str(source_prefix),
                             arrays["position"].min(axis=0), arrays["position"].max(axis=0))
    except VisualizationError:
        raise
    except Exception as exc:
        raise VisualizationError(f"Invalid visualization data: {exc}") from exc


def load_workspace(input_prefix, config_path, *, max_points=None):
    """G2 loader verifies hash/schema/model and internally rechecks <=3 FK rows.

    This reuses the existing validation API, not a new sampling/FK pipeline.
    """
    from ..robot.model import load_model

    model = load_model(config_path)
    arrays, metadata = load_results(input_prefix, model=model)
    return prepare_view(arrays, metadata, source_prefix=str(Path(input_prefix).expanduser().absolute()),
                        max_points=max_points)


def _title(view, kind):
    model = view.metadata["robot_model"]
    return (f"{escape(view.model_name)} — {kind}<br>"
            f"<sup>{len(view.position):,} shown / {view.total_samples:,} total · seed {view.metadata['experiment']['seed']} · "
            f"Base: {escape(model['base_frame'])} · EE: {escape(model['ee_frame'])}</sup>")


def _hover(view):
    text = ("Sample Index: %{customdata[0]:.0f}<br>X: %{customdata[1]:.6f} m"
            "<br>Y: %{customdata[2]:.6f} m<br>Z: %{customdata[3]:.6f} m")
    for column, name in enumerate(view.joint_order, 4):
        text += f"<br>{escape(name)}: %{{customdata[{column}]:.6f}} rad"
    return text + "<extra>Workspace sample</extra>"


def _layout(view, kind, height):
    selection = "all original rows" if len(view.position) == view.total_samples else "deterministic linspace indices; no resampling"
    return {
        "title": {"text": _title(view, kind), "x": 0.03}, "template": "plotly_white", "height": height,
        "margin": {"l": 65, "r": 90, "t": 125, "b": 155},
        "legend": {"orientation": "h", "x": 0, "y": 1.08},
        "meta": {"stage": "G3", "stage_acceptance": "pending_user_review", "hardware_checked": False,
                 "total_samples": view.total_samples, "displayed_samples": len(view.position),
                 "selection": selection, "source_prefix": view.source_prefix,
                 "npz_sha256": view.metadata["dataset"]["sha256"],
                 "base_frame": view.metadata["robot_model"]["base_frame"],
                 "ee_frame": view.metadata["robot_model"]["ee_frame"], "units": {"length": "m", "angle": "rad"}},
    }


def _add_caption(figure, view):
    selection = "All original rows" if len(view.position) == view.total_samples else "Deterministic index selection"
    figure.add_annotation(
        x=0, y=-0.16, xref="paper", yref="paper", xanchor="left", yanchor="top", showarrow=False,
        align="left", font={"size": 11, "color": "#475569"},
        text=(f"{selection}. Joint-limit-constrained geometric reachable workspace samples; color = Z [m].<br>"
              "No collision, physical or RJ45 task feasibility checked; sampled extrema are not a strict boundary.<br>"
              "Negative Z is retained. Base origin is not ground; EE is not a calibrated RJ45 TCP.<br>"
              "Density depends on joint sampling; no true workspace volume is inferred from a convex hull."))


def _color_limits(view):
    low, high = float(view.full_min[2]), float(view.full_max[2])
    if low == high:
        half = max(1e-6, abs(low) * 1e-6)
        return low - half, high + half
    return low, high


def build_3d_figure(view):
    go, _, _ = _plotly_modules()
    low, high = _color_limits(view)
    figure = go.Figure()
    figure.add_trace(go.Scatter3d(
        x=view.position[:, 0], y=view.position[:, 1], z=view.position[:, 2], mode="markers",
        name="Workspace samples", customdata=view.customdata, hovertemplate=_hover(view),
        marker={"size": 2.5, "opacity": 0.65, "color": view.position[:, 2], "colorscale": "Viridis",
                "cmin": low, "cmax": high, "showscale": True, "colorbar": {"title": {"text": "Z [m]"}}}))
    figure.add_trace(go.Scatter3d(
        x=[0], y=[0], z=[0], mode="markers", name="Base origin",
        marker={"size": 7, "color": "#ea580c", "symbol": "diamond"},
        hovertemplate="Base origin: (0, 0, 0) m<extra></extra>"))
    figure.update_layout(**_layout(view, "3D workspace point cloud", 850),
                         scene={"xaxis": {"title": {"text": "X [m]"}},
                                "yaxis": {"title": {"text": "Y [m]"}},
                                "zaxis": {"title": {"text": "Z [m]"}},
                                "aspectmode": "data", "dragmode": "orbit"})
    _add_caption(figure, view)
    return figure


def build_projections_figure(view):
    go, _, make_subplots = _plotly_modules()
    figure = make_subplots(rows=1, cols=3, subplot_titles=("XY projection", "XZ projection", "YZ projection"),
                           horizontal_spacing=0.09)
    minimum = np.minimum(view.full_min, 0)
    maximum = np.maximum(view.full_max, 0)
    span = max(float(np.max(maximum - minimum)) * 1.1, 1e-6)
    centers = (minimum + maximum) / 2
    ranges = [[float(center - span / 2), float(center + span / 2)] for center in centers]
    for column, (a, b) in enumerate(((0, 1), (0, 2), (1, 2)), 1):
        figure.add_trace(go.Scattergl(
            x=view.position[:, a], y=view.position[:, b], mode="markers", name="Workspace samples",
            legendgroup="samples", showlegend=column == 1, customdata=view.customdata, hovertemplate=_hover(view),
            marker={"size": 3, "opacity": 0.6, "color": view.position[:, 2], "coloraxis": "coloraxis"}), row=1, col=column)
        figure.add_trace(go.Scatter(
            x=[0], y=[0], mode="markers", name="Base origin", legendgroup="base", showlegend=column == 1,
            marker={"size": 10, "color": "#ea580c", "symbol": "diamond"},
            hovertemplate="Base origin: (0, 0) m<extra></extra>"), row=1, col=column)
        axis_id = "x" if column == 1 else f"x{column}"
        figure.update_xaxes(title_text=f"{'XYZ'[a]} [m]", range=ranges[a], constrain="domain", row=1, col=column)
        figure.update_yaxes(title_text=f"{'XYZ'[b]} [m]", range=ranges[b], scaleanchor=axis_id,
                            scaleratio=1, constrain="domain", row=1, col=column)
    low, high = _color_limits(view)
    figure.update_layout(**_layout(view, "Orthogonal workspace projections", 650),
                         coloraxis={"colorscale": "Viridis", "cmin": low, "cmax": high,
                                    "colorbar": {"title": {"text": "Z [m]"}}})
    _add_caption(figure, view)
    return figure


def html_paths(output_prefix):
    prefix = Path(output_prefix).expanduser()
    if prefix.name in ("", ".", "..") or prefix.is_dir() or prefix.suffix in (".html", ".npz", ".json"):
        raise VisualizationError("output-prefix must be a filename prefix without a data/HTML suffix")
    prefix = prefix.absolute()
    return (Path(str(prefix) + "_3d.html"), Path(str(prefix) + "_projections.html"),
            Path(str(prefix) + "_visualization.lock"))


def check_html_available(output_prefix):
    paths = html_paths(output_prefix)
    for path in paths:
        if os.path.lexists(path):
            raise VisualizationError(f"Refusing to overwrite existing HTML/transaction: {path}")
    return paths


class _ScriptLinks(HTMLParser):
    def __init__(self):
        super().__init__()
        self.sources = []
    def handle_starttag(self, tag, attributes):
        if tag.lower() == "script":
            self.sources.extend(value for key, value in attributes if key.lower() == "src" and value)


def standalone_html(figure):
    _, pio, _ = _plotly_modules()
    document = pio.to_html(figure, full_html=True, include_plotlyjs=True, include_mathjax=False,
                           config={"responsive": True, "scrollZoom": True, "displaylogo": False})
    parser = _ScriptLinks()
    parser.feed(document)
    if (parser.sources or "Plotly.newPlot" not in document or "plotly.js" not in document
            or "</html>" not in document.lower()):
        raise VisualizationError("Export is not a complete standalone Plotly HTML document")
    return document


def _publish_documents(output_prefix, documents):
    """Filesystem publication only; callers supply already-rendered documents."""
    paths = check_html_available(output_prefix)
    targets, lock = paths[:2], paths[2]
    temporary, owned = [], []
    lock_identity = None
    def identity(path):
        stat = path.stat()
        return stat.st_dev, stat.st_ino
    def remove_owned(path, inode):
        try:
            stat = path.lstat()
            if (stat.st_dev, stat.st_ino) == inode:
                path.unlink()
        except FileNotFoundError:
            pass
    try:
        if len(documents) != 2 or any(not isinstance(text, str) or not text for text in documents):
            raise VisualizationError("Two nonempty rendered documents are required")
        lock.parent.mkdir(parents=True, exist_ok=True)
        with lock.open("xb") as stream:
            lock_identity = identity(lock)
            stream.write(f"G3 WRITING pid={os.getpid()}\n".encode())
        for target, document in zip(targets, documents):
            fd, name = tempfile.mkstemp(prefix=f".{target.name}.", suffix=".tmp", dir=target.parent)
            temporary.append(Path(name))
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                stream.write(document)
                stream.flush()
                os.fsync(stream.fileno())
        for path, target in zip(temporary, targets):
            inode = identity(path)
            os.link(path, target)  # atomic no-replace, same filesystem
            owned.append((target, inode))
        for path in temporary:
            path.unlink()
        temporary.clear()
        remove_owned(lock, lock_identity)
        lock_identity = None
        return targets
    except BaseException as exc:
        for path, inode in reversed(owned):
            remove_owned(path, inode)
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        raise VisualizationError(f"HTML publication failed: {exc}") from exc
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)
        if lock_identity is not None:
            remove_owned(lock, lock_identity)


def export_workspace(view, output_prefix):
    check_html_available(output_prefix)
    # Render BOTH in memory before publishing either filename.
    documents = (standalone_html(build_3d_figure(view)), standalone_html(build_projections_figure(view)))
    return _publish_documents(output_prefix, documents)
