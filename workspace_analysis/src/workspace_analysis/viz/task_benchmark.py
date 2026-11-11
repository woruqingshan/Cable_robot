"""Compact overview from benchmark summary only; no FK or collision decisions."""
from pathlib import Path


def export_benchmark_summary(summary: dict, output: Path) -> Path:
    """Render counts with their explicit levels and actual per-trial timing sums."""
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
    rows = summary['rows']; ports = [r['port_id'] for r in rows]
    figure = make_subplots(rows=3, cols=1, specs=[[{'type':'xy'}],[{'type':'xy'}],[{'type':'table'}]],
        row_heights=[.3,.3,.4], vertical_spacing=.09,
        subplot_titles=['Unique IK / configuration / goal counts (not port success rates)',
                        'Sum of search and final validation time per port [s]', 'Port outcomes'])
    for key, label in [('ik_solutions','Task-pose IK solutions'),('collision_candidates','Collision-valid configurations'),
                       ('approach_successes','Goals with full approach')]:
        figure.add_trace(go.Bar(x=ports,y=[r[key] for r in rows],name=label),row=1,col=1)
    for method in ('linear','rrt_connect'):
        for suffix in ('search_s','validation_s'):
            figure.add_trace(go.Bar(x=ports,y=[r[method+'_'+suffix] for r in rows],name=method+' '+suffix),row=2,col=1)
    figure.add_trace(go.Table(header={'values':['Port','Task status','Linear','RRT','Approach']},
        cells={'values':[[r[k] for r in rows] for k in ('port_id','status','linear_status','rrt_connect_status','approach_status')]}),row=3,col=1)
    figure.update_layout(height=1150,barmode='group',title='RESEARCH_ONLY — one scene / common simulation start / declared seed; discrete validation only',
        meta={'usage':'RESEARCH_ONLY','port_denominator':summary['port_denominator'],'hardware_validated':False})
    path = Path(output)
    if path.suffix != '.html': raise ValueError('HTML output required')
    with path.open('x') as f: f.write(figure.to_html(include_plotlyjs=True,full_html=True))
    return path
