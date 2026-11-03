"""B2 Plotly geometry display in base_link; no collision/distance/IK logic.

Scene-only imports are independent of ROS/URDF. Workspace mode lazily reuses
G3 load_workspace and build_3d_figure, preserving G2 validation and customdata.
"""

from html import escape
from html.parser import HTMLParser
import os
from pathlib import Path
import tempfile
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from ..scene.model import SceneSnapshot
from ..scene.schema import load_scene
from .meshes import SceneVisualizationError, _positive, _segments, _wire_edges, box_mesh, cylinder_mesh
from ..scene.frames import resolve_transform
from ..geometry.se3 import transform_point, transform_direction

if TYPE_CHECKING:
    from plotly.graph_objects import Figure
    from .workspace import WorkspaceView


def _plotly():
    try:
        import plotly.graph_objects as go
        import plotly.io as pio
        return go, pio
    except ImportError as exc:
        raise SceneVisualizationError('Plotly is unavailable; no dependencies were installed') from exc


def object_mesh(scene: SceneSnapshot, object_id: str, cylinder_segments: int = 48
                ) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
    """Build an object's mesh in base_link using B1's full rigid transform."""
    _segments(cylinder_segments)
    for obj in scene.collision_objects:
        if obj['id'] == object_id:
            local, faces = (box_mesh(obj['size_m']) if obj['type'] == 'box' else
                            cylinder_mesh(obj['radius_m'], obj['length_m'], cylinder_segments))
            transform = scene.object_transform(object_id, target_frame='base_link')
            vertices = np.array([transform_point(transform, point) for point in local])
            return vertices, faces
    raise SceneVisualizationError(f'Unknown scene object: {object_id!r}')


def frame_axes(scene: SceneSnapshot, frame_id: str, axis_length: float = .05
               ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Return base-frame origin and three axis endpoints computed by B1."""
    length = _positive(axis_length, 'axis_length_m')
    transform = resolve_transform(scene, 'base_link', frame_id)
    origin = transform_point(transform, [0,0,0])
    ends = np.array([transform_point(transform, length * axis) for axis in np.eye(3)])
    return origin, ends


def port_directions(scene: SceneSnapshot, port_id: str) -> tuple[NDArray[np.float64], dict[str, NDArray[np.float64]]]:
    """Return port origin and outward/insertion/key vectors in base_link."""
    for port in scene.ports:
        if port['id'] == port_id:
            transform = resolve_transform(scene, 'base_link', port['frame'])
            return transform_point(transform, [0,0,0]), {
                name: transform_direction(transform, port[name + '_axis_local'])
                for name in ('outward', 'insertion', 'key')}
    raise SceneVisualizationError(f'Unknown semantic port: {port_id!r}')


def _add_geometry(figure: 'Figure', scene: SceneSnapshot, segments: int) -> None:
    go, _ = _plotly()
    palette = {'cabinet': '#94a3b8', 'patch_panel': '#8b5cf6', 'independent_obstacle': '#f59e0b'}
    for obj in scene.collision_objects:  # Semantic assemblies never create extra solids.
        memberships = [e for e in scene.entities if obj['id'] in e['components']]
        kinds = {e['kind'] for e in memberships}
        category = 'patch_panel' if 'patch_panel' in kinds else 'cabinet' if 'cabinet' in kinds else 'independent_obstacle'
        vertices, faces = object_mesh(scene, obj['id'], segments)
        center = scene.object_transform(obj['id'])[:3, 3].tolist()
        dimensions = ({'size_m': list(obj['size_m'])} if obj['type'] == 'box' else
                      {'radius_m': obj['radius_m'], 'length_m': obj['length_m']})
        group = 'object:' + obj['id']
        hover = (f"Object ID: {escape(obj['id'])}<br>Type: {obj['type']}<br>Category: {category}"
                 f"<br>Parent: {escape(obj['parent_frame'])}<br>Center XYZ [m]: {center}"
                 f"<br>Geometry [m]: {dimensions}<br>Base XYZ: (%{{x:.5f}}, %{{y:.5f}}, %{{z:.5f}}) m"
                 '<extra>Geometry display only</extra>')
        figure.add_trace(go.Mesh3d(
            x=vertices[:,0], y=vertices[:,1], z=vertices[:,2], i=faces[:,0], j=faces[:,1], k=faces[:,2],
            color=palette[category], opacity=.30 if category == 'cabinet' else .50, flatshading=True,
            name=f"{category}: {obj['id']}", showlegend=True, legendgroup=group, hovertemplate=hover,
            meta={'role': 'scene_object', 'object_id': obj['id'], 'type': obj['type'], 'category': category,
                  'entity_ids': [e['id'] for e in memberships], 'dimensions': dimensions, 'center_m': center}))
        lines = [[], [], []]
        for a, b in _wire_edges(obj, segments):
            for axis in range(3):
                lines[axis].extend([vertices[a,axis], vertices[b,axis], None])
        figure.add_trace(go.Scatter3d(
            x=lines[0], y=lines[1], z=lines[2], mode='lines', line={'color': palette[category], 'width': 3},
            legendgroup=group, showlegend=False, hoverinfo='skip', name=obj['id'] + ' edges',
            meta={'role': 'object_edges', 'object_id': obj['id']}))


def _add_frames_and_ports(figure: 'Figure', scene: SceneSnapshot, length: float) -> None:
    go, _ = _plotly()
    for frame_id in ['base_link', *(f.id for f in scene.frames)]:
        origin, ends = frame_axes(scene, frame_id, length)
        group = 'frame:' + frame_id
        for i, (axis, color) in enumerate(zip('XYZ', ['#dc2626', '#16a34a', '#2563eb'])):
            figure.add_trace(go.Scatter3d(
                x=[origin[0],ends[i,0]], y=[origin[1],ends[i,1]], z=[origin[2],ends[i,2]],
                mode='lines+text', text=['',axis], textposition='top center', line={'color': color, 'width': 5},
                name='Frame: ' + frame_id, legendgroup=group, showlegend=i == 0,
                hovertemplate=f'{escape(frame_id)} / {axis}<br>(%{{x:.5f}}, %{{y:.5f}}, %{{z:.5f}}) m<extra></extra>',
                meta={'role': 'frame_axis', 'frame_id': frame_id, 'axis': axis}))
        figure.add_trace(go.Scatter3d(
            x=[origin[0]], y=[origin[1]], z=[origin[2]], mode='markers+text', text=[frame_id],
            textposition='bottom center' if frame_id in {p['frame'] for p in scene.ports} else 'top center',
            marker={'size': 3, 'color': '#334155'}, legendgroup=group, showlegend=False,
            hovertemplate=f'Frame origin: {escape(frame_id)}<extra></extra>',
            meta={'role': 'frame_label', 'frame_id': frame_id}))
    for port in scene.ports:
        origin, directions = port_directions(scene, port['id'])
        for name, direction in directions.items():
            end = origin + length * 1.6 * direction
            color = {'outward': '#0891b2', 'insertion': '#db2777', 'key': '#0f766e'}[name]
            group = f"port:{port['id']}:{name}"
            label = f"Port {port['id']}: {name.title()} Direction"
            figure.add_trace(go.Scatter3d(
                x=[origin[0],end[0]], y=[origin[1],end[1]], z=[origin[2],end[2]],
                mode='lines+text', text=['', name.title()], textposition='bottom center',
                line={'color': color, 'width': 7, 'dash': 'dash'}, name=label, legendgroup=group,
                hovertemplate=f'{escape(label)}<br>Unit direction: {direction.tolist()}<extra>Uncalibrated semantic port</extra>',
                meta={'role': 'port_direction', 'port_id': port['id'], 'direction': name, 'unit_vector': direction.tolist()}))
            figure.add_trace(go.Cone(
                x=[end[0]], y=[end[1]], z=[end[2]], u=[direction[0]], v=[direction[1]], w=[direction[2]],
                anchor='tip', sizemode='absolute', sizeref=length * .20,
                colorscale=[[0,color],[1,color]], showscale=False, showlegend=False, legendgroup=group,
                hoverinfo='skip', meta={'role': 'port_arrowhead', 'port_id': port['id'], 'direction': name}))


def build_scene_figure(scene: SceneSnapshot, workspace: 'WorkspaceView | None' = None, *,
                       axis_length: float = .05, cylinder_segments: int = 48) -> 'Figure':
    """Draw validated B1/G3 inputs; file callers should use load_scene_figure.

    No sampling, collision labels, point filtering or inter-object distances.
    """
    length, n = _positive(axis_length, 'axis_length_m'), _segments(cylinder_segments)
    if scene.reference_frame != 'base_link':
        raise SceneVisualizationError('Scene reference frame must be base_link')
    if workspace is not None and workspace.metadata['robot_model']['base_frame'] != scene.reference_frame:
        raise SceneVisualizationError('Workspace and Scene Base Frames must both be base_link')
    go, _ = _plotly()
    if workspace is None:
        figure = go.Figure(go.Scatter3d(x=[0], y=[0], z=[0], mode='markers', name='Base origin',
                          marker={'size': 7, 'color': '#ea580c', 'symbol': 'diamond'},
                          hovertemplate='Base origin (0,0,0) m<extra></extra>', meta={'role': 'base_origin'}))
        workspace_meta = None
    else:
        from .workspace import build_3d_figure
        figure = build_3d_figure(workspace)
        figure.data[0].update(meta={'role': 'workspace_samples'}, legendgroup='workspace_samples')
        figure.data[1].update(meta={'role': 'base_origin'})
        workspace_meta = {**dict(figure.layout.meta), 'seed': workspace.metadata['experiment']['seed'],
                          'model_name': workspace.model_name,
                          'model_sha256': workspace.metadata['robot_model']['expanded_urdf_sha256']}
    figure.layout.annotations = ()
    _add_geometry(figure, scene, n)
    _add_frames_and_ports(figure, scene, length)
    label = 'Synthetic Scene' if scene.provenance['kind'] == 'synthetic' else 'Scene'
    samples = ('Scene Only' if workspace is None else
               f"{len(workspace.position):,} shown / {workspace.total_samples:,} total workspace samples")
    figure.update_layout(
        title={'text': f'{escape(scene.scene_id)} — {label} / Not Hardware Verified<br><sup>{samples} · Base: base_link</sup>', 'x': .03},
        template='plotly_white', height=950, margin={'l': 35, 'r': 25, 't': 115, 'b': 145},
        scene={'xaxis': {'title': {'text': 'X [m]'}}, 'yaxis': {'title': {'text': 'Y [m]'}},
               'zaxis': {'title': {'text': 'Z [m]'}}, 'aspectmode': 'data', 'dragmode': 'orbit',
               'domain': {'x': [0, .73]}, 'camera': {'eye': {'x': -1.7, 'y': -1.4, 'z': 1.2}}},
        legend={'x': .76, 'y': 1, 'orientation': 'v', 'groupclick': 'togglegroup', 'font': {'size': 10}},
        meta={'stage': 'P2.3-B2', 'stage_acceptance': 'pending_user_visualization_review', 'hardware_checked': False,
              'collision_checked': False, 'scene_id': scene.scene_id, 'scene_path': scene.source_path,
              'raw_yaml_sha256': scene.raw_yaml_sha256, 'canonical_scene_sha256': scene.canonical_scene_sha256,
              'reference_frame': scene.reference_frame, 'units': dict(scene.units),
              'scene_provenance': dict(scene.provenance), 'coverage': scene.coverage,
              'geometry_object_count': len(scene.collision_objects), 'axis_length_m': length,
              'cylinder_segments': n, 'workspace': workspace_meta})
    figure.add_annotation(x=0, y=-.09, xref='paper', yref='paper', xanchor='left', yanchor='top', showarrow=False,
        align='left', font={'size': 11}, text=(
        'Synthetic fixtures are not measured IDC/RJ45 geometry. Object colors = semantic categories; workspace color = Z [m].<br>'
        'Visual overlap does not prove collision; visual separation does not prove whole-arm clearance. No collision or distance checked.<br>'
        'Frame axes: X red, Y green, Z blue. Port is semantic; TCP uncalibrated. Base origin is not ground.<br>'
        'Negative Z retained. Density depends on joint sampling; finite samples do not prove a strict boundary, volume or task feasibility.'))
    return figure


def load_scene_figure(scene_path: str | Path, *, input_prefix: str | Path | None = None,
                      config_path: str | Path | None = None, max_points: int | None = None,
                      axis_length: float = .05, cylinder_segments: int = 48) -> 'Figure':
    """Load Scene Only or reuse G3's verified G2 loader for a workspace overlay."""
    scene = load_scene(scene_path)
    workspace = None
    if input_prefix is not None:
        from .workspace import load_workspace
        config = config_path or Path(__file__).resolve().parents[3] / 'config/workspace.yaml'
        workspace = load_workspace(input_prefix, config, max_points=max_points)
    elif max_points is not None:
        raise SceneVisualizationError('max_points requires input_prefix; Scene Only has no workspace rows')
    return build_scene_figure(scene, workspace, axis_length=axis_length, cylinder_segments=cylinder_segments)


def check_html_available(output: str | Path) -> Path:
    """Validate a single .html destination and refuse existing files/symlinks."""
    path = Path(output).expanduser().absolute()
    if path.suffix.lower() != '.html':
        raise SceneVisualizationError('output must end in .html')
    if os.path.lexists(path):
        raise SceneVisualizationError(f'Refusing to overwrite existing output: {path}')
    return path


class _ScriptSources(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.sources = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag == 'script':
            self.sources.extend(value for key, value in attrs if key == 'src' and value)


def export_scene_html(figure: 'Figure', output: str | Path) -> Path:
    """Embed Plotly.js and atomically publish one HTML without replacement.

    G3's publisher requires a pair. This single-file variant retains its
    same-directory temp, fsync, hard-link NO-REPLACE and owned-file rollback.
    """
    target = check_html_available(output)
    _, pio = _plotly()
    document = pio.to_html(figure, full_html=True, include_plotlyjs=True, include_mathjax=False,
                           config={'responsive': True, 'scrollZoom': True, 'displaylogo': False})
    parser = _ScriptSources()
    parser.feed(document)
    if parser.sources or 'Plotly.newPlot' not in document or 'plotly.js' not in document or '</html>' not in document.lower():
        raise SceneVisualizationError('Export is not a complete offline Plotly HTML')
    temporary = None
    owned = None
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix='.' + target.name + '.', suffix='.tmp', dir=target.parent)
        temporary = Path(name)
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            stream.write(document)
            stream.flush()
            os.fsync(stream.fileno())
        info = temporary.stat()
        os.link(temporary, target)
        owned = info.st_dev, info.st_ino
        temporary.unlink()
        temporary = None
        return target
    except BaseException as exc:
        if owned is not None and os.path.lexists(target):
            info = target.lstat()
            if (info.st_dev, info.st_ino) == owned:
                target.unlink()
        if isinstance(exc, (KeyboardInterrupt, SystemExit)):
            raise
        raise SceneVisualizationError(f'HTML publication failed: {exc}') from exc
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
