#!/usr/bin/env python3
"""Export hash-bound World scene geometry using the existing Plotly scene renderer."""
import argparse
from pathlib import Path
from hashlib import sha256
import json
import sys
import numpy as np
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'ros2/jetrover_idc_visualization')]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    from workspace_analysis.scene.schema import parse_scene
    from workspace_analysis.viz.scene import build_scene_figure
    from jetrover_idc_visualization.markers import restriction_specs
    import plotly.graph_objects as go
    args.output_dir.mkdir(parents=True,exist_ok=False)
    selected={'D0':'b0_D0_workspace_on','D1':'ym70_D1_workspace_on','D2':'ym70_D2_workspace_on',
              'D3':'b0_D3_workspace_on','D4':'b0_D4_workspace_on'}
    evidence={}
    for label,name in selected.items():
        condition=args.run/name
        manifest=json.loads((condition/'manifest.json').read_text())
        raw=(condition/'world_scene.json').read_bytes()
        if sha256(raw).hexdigest()!=manifest['files']['world_scene.json']:
            raise ValueError('World scene hash differs')
        scene=parse_scene(raw)
        result=json.loads((condition/'result.json').read_text())
        figure=build_scene_figure(scene,axis_length=.015)
        for trace in figure.data:
            meta=trace.meta or {}
            if meta.get('role') in ('frame_axis','frame_label'):
                trace.visible=False
            if meta.get('role')=='scene_object' and meta.get('object_id','').startswith('restricted_'):
                trace.color='#f59e0b';trace.opacity=.9
        specs=restriction_specs({'workspace_config':result['workspace_config'],'world_from_base':np.eye(4).tolist()})
        for spec in specs:
            if spec.kind!='LINE_LIST':
                continue
            points=[]
            for index in range(0,len(spec.points),2):
                points.extend([spec.points[index],spec.points[index+1],(None,None,None)])
            figure.add_trace(go.Scatter3d(x=[p[0] for p in points],y=[p[1] for p in points],z=[p[2] for p in points],
                mode='lines',line={'color':'magenta','width':4},name='Conditional TCP boundary (finite display extent)'))
        figure.update_layout(title=f'{label} — World geometry / target {result["target"]} — RESEARCH_ONLY',
            scene={'aspectmode':'data'},meta={'world_scene_sha256':scene.canonical_scene_sha256,
                'source_condition_manifest_sha256':sha256((condition/'manifest.json').read_bytes()).hexdigest(),
                'scope':'recorded scene geometry and workspace markers'})
        target=args.output_dir/(label+'.html')
        with target.open('x') as stream:stream.write(figure.to_html(include_plotlyjs=True,full_html=True))
        evidence[label]={'file':target.name,'sha256':sha256(target.read_bytes()).hexdigest(),
            'world_scene_sha256':scene.canonical_scene_sha256}
    with (args.output_dir/'manifest.json').open('x') as stream:json.dump(evidence,stream,indent=2)


if __name__=='__main__':main()
