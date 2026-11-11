#!/usr/bin/env python3
"""Regenerate contact-region and ACM comparison plots from saved actual queries."""
import argparse,json,sys,uuid
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from workspace_analysis.viz.workspace import standalone_html
import plotly.graph_objects as go
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--refinement-dir',type=Path,required=True)
parser.add_argument('--pilot-dir',type=Path,required=True)
parser.add_argument('--output-dir',type=Path)
a=parser.parse_args();source=a.refinement_dir;pilot=a.pilot_dir
out=a.output_dir or source/('analysis_'+uuid.uuid4().hex)
out.mkdir(parents=True,exist_ok=False);(out/'figures').mkdir()
probes=json.loads((source/'joint_probes.json').read_text())
for joint,pair in [('joint1',{'back_shell_green_link','link1'}),('joint2',{'link2','servo_link1'}),('joint3',{'link2','link3'})]:
 contacts=[(p['angle'],c) for p in probes if p['joint']==joint for c in p['query']['contacts_base'] if {c['first'],c['second']}==pair]
 fig=go.Figure()
 xyz=np.array([c['position_joint_frame_m'] for _,c in contacts]);angles=[a for a,_ in contacts]
 fig.add_trace(go.Scatter3d(x=xyz[:,0],y=xyz[:,1],z=xyz[:,2],mode='markers',marker={'color':angles,'colorscale':'Viridis','size':5,'colorbar':{'title':'angle rad'}},customdata=[[a,c['depth_m']] for a,c in contacts],hovertemplate='angle=%{customdata[0]} rad<br>FCL depth=%{customdata[1]} m<extra></extra>',name=str(sorted(pair))))
 fig.update_layout(title=f'{joint}: actual representative FCL contacts in joint zero-axis frame',scene={'aspectmode':'data','xaxis_title':'X [m]','yaxis_title':'Y [m]','zaxis_title':'Z [m]'})
 with (out/f'figures/{joint}_contact_regions.html').open('x') as f:f.write(standalone_html(fig))
rows=json.loads((pilot/'evaluations.json').read_text())
focus=[('back_shell_green_link','base_link'),('link4','servo_link2'),('back_shell_green_link','link1'),('link2','servo_link1'),('link2','link3'),('gripper_link','l_in_link')]
fig=go.Figure()
for label,data in rows.items():
 counts=[sum(any(p['kind']=='self' and set(p['links'])==set(pair) for p in r['remaining_collision_pairs']) for r in data) for pair in focus]
 fig.add_trace(go.Bar(x=[' / '.join(p) for p in focus],y=counts,name=label))
fig.update_layout(title='Actual input configuration self-pair incidence after each explicit ACM',yaxis_title='Configuration count (overlapping pairs)',barmode='group')
with (out/'figures/policy_comparison.html').open('x') as f:f.write(standalone_html(fig))

print(out)
