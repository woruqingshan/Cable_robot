"""Lazy Plotly presentation of recorded station results; no task recomputation."""
import os
from pathlib import Path


def export_base_pose_summary(summary: dict,output: Path) -> None:
    import plotly.graph_objects as go
    if summary.get('stage')!='T5.2-R2' or summary.get('usage')!='RESEARCH_ONLY':
        raise ValueError('T5.2-R2 research summary required')
    output=Path(output)
    if os.path.lexists(output):raise FileExistsError('refusing existing figure: '+str(output))
    rows=summary['rows'];stations=list(summary['stations']);ports=sorted({r['port_id'] for r in rows})
    lookup={(r['base_pose_id'],r['port_id']):r for r in rows}
    if len(lookup)!=len(rows):raise ValueError('duplicate station/port result')
    z=[];text=[];hover=[]
    for station in stations:
        zs=[];ts=[];hs=[]
        for port in ports:
            row=lookup.get((station,port))
            if row is None:zs.append(None);ts.append('Not run');hs.append('No experiment');continue
            status=row['status'];zs.append(1 if status=='SUCCESS_RESEARCH_ONLY' else 0 if status=='IK_NO_SOLUTION_WITHIN_BUDGET' else .5)
            ts.append('PASS' if zs[-1]==1 else 'IK budget' if zs[-1]==0 else status)
            hs.append(f"{status}<br>IK {row['ik_solutions']}; collision-valid {row['collision_candidates']}"
                f"<br>Linear {row['linear_successes']}/{row['linear_trials']}; RRT {row['rrt_connect_successes']}/{row['rrt_connect_trials']}"
                f"<br>Approach {row['approach_successes']}/{row['approach_trials']}<br>Start valid: {row['start_valid']}"
                f"<br>Coverage: {row['coverage']}")
        z.append(zs);text.append(ts);hover.append(hs)
    figure=go.Figure(go.Heatmap(x=ports,y=stations,z=z,text=text,customdata=hover,
        hovertemplate='%{y} / %{x}<br>%{customdata}<extra></extra>',zmin=0,zmax=1,
        colorscale=[[0,'#b54747'],[.5,'#e8b44f'],[1,'#36876c']],showscale=False,hoverongaps=False))
    for i,station in enumerate(stations):
        for j,port in enumerate(ports):figure.add_annotation(x=port,y=station,text=text[i][j],showarrow=False,font={'color':'white' if z[i][j] is not None else '#777'})
    figure.update_layout(title='T5.2-R2 bounded station pilot — RESEARCH_ONLY',xaxis_title='Port (unrun cells are empty)',
        yaxis_title='Base Pose relative to B0',template='plotly_white',height=470)
    with output.open('x') as f:f.write(figure.to_html(include_plotlyjs=True,full_html=True))
