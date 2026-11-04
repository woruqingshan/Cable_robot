"""Offline full-robot replay of checked native RobotState samples (no FK here)."""
import json
import os
from pathlib import Path
import tempfile
import numpy as np
from .task import build_task_figure


def build_path_figure(scene, surfaces, native_start, targets, assessment, states, label):
    """Reuse task/collision/scene rendering. Transforms must come from native checks."""
    import plotly.graph_objects as go
    if not states or any('link_transforms_base' not in s or 'T_base_to_ee' not in s for s in states):
        raise ValueError('replay requires real native transforms at every displayed sample')
    figure = build_task_figure(scene, surfaces, native_start, targets, assessment)
    tcp = np.array([(np.asarray(s['T_base_to_ee'])@targets.spec.T_ee_tcp)[:3, 3] for s in states])
    figure.add_trace(go.Scatter3d(x=tcp[:, 0], y=tcp[:, 1], z=tcp[:, 2], mode='lines+markers',
        line={'width': 4, 'color': '#64748b'}, marker={'size': 2, 'color': ['#16a34a' if s['valid'] else '#ef4444' for s in states]},
        customdata=list(range(len(states))), hovertemplate='Checked path sample %{customdata}<extra></extra>',
        name='Sampled TCP trajectory (not insertion)', meta={'role': 'checked_tcp_path'}))
    for i, name, color in [(0, 'Simulation start (not hardware Home)', '#0284c7'), (-1, 'Goal / end of diagnostic path', '#ea580c')]:
        p = tcp[i]
        figure.add_trace(go.Scatter3d(x=[p[0]], y=[p[1]], z=[p[2]], mode='markers', marker={'size': 8, 'color': color}, name=name))
    # Store geometry once in link coordinates; replay applies ONLY native transforms.
    local_surfaces = []
    points = [tcp]
    for surface in surfaces:
        t = np.asarray(native_start['link_transforms_base'][surface['id']])
        v = (surface['vertices']-t[:3, 3])@t[:3, :3]
        local_surfaces.append({'link': surface['id'], 'geometry_index': surface['geometry_index'], 'vertices': v.tolist()})
        lo, hi = v.min(axis=0), v.max(axis=0)
        corners = np.array([[x, y, z] for x in [lo[0], hi[0]] for y in [lo[1], hi[1]] for z in [lo[2], hi[2]]])
        for state in states:
            t = np.asarray(state['link_transforms_base'][surface['id']])
            points.append(corners@t[:3, :3].T+t[:3, 3])
    bounds = np.vstack(points)
    axis_ranges = {}
    for i, name in enumerate(['xaxis', 'yaxis', 'zaxis']):
        old = getattr(figure.layout.scene, name).range
        axis_ranges[name] = {'range': [min(old[0], float(bounds[:, i].min())-.03), max(old[1], float(bounds[:, i].max())+.03)]}
    figure.layout.annotations = ()
    figure.add_annotation(text='RESEARCH_ONLY • 29 SRDF exemptions physically unreviewed • discrete checks only • uncalibrated virtual TCP, no RJ45/cable body • illustrative playback, not timed execution',
                          x=0, y=1.04, xref='paper', yref='paper', showarrow=False)
    figure.update_layout(title=label, scene={**axis_ranges, 'aspectmode': 'data'},
        meta={**dict(figure.layout.meta), 'stage': 'T3', 'viewer_stage': 'T3', 'path_label': label,
              'path_sample_count': len(states), 'all_robot_meshes_replayed': len(surfaces),
              'replay_transform_source': 'native MoveIt RobotState at every checked q',
              'continuous_collision_proven': False, 'time_parameterized': False})
    # Task pose is constrained ONLY at the goal, not along the transit path.
    meta = dict(figure.layout.meta)
    meta['initial_task_check'] = meta.pop('task_check')
    figure.layout.meta = meta
    return figure, local_surfaces


def export_path_viewer(figure, local_surfaces, states, targets, output, *, label):
    """Keep one immutable mesh + per-state transforms; avoid enormous mesh frames."""
    import plotly.io as pio
    path = Path(output).absolute()
    if path.suffix != '.html' or os.path.lexists(path): raise ValueError('new .html output required')
    data = {'usage': 'RESEARCH_ONLY', 'label': label, 'surfaces': local_surfaces, 'states': states,
            'T_ee_tcp': targets.spec.T_ee_tcp.tolist(), 'tcp': targets.spec.data['tcp'],
            'pre_tcp_position': list(targets.pre_tcp.position_m)}
    payload = json.dumps(data, allow_nan=False).replace('<', '\\u003c')
    script = r'''
const root=document.getElementById('{plot_id}');
const model=PAYLOAD;
const panel=document.createElement('section');panel.style='padding:14px;background:#f1f5f9;font:14px system-ui';root.parentNode.insertBefore(panel,root);
const heading=document.createElement('b');heading.textContent=model.label+' · RESEARCH_ONLY · no insertion / no timed execution';panel.append(heading,document.createElement('br'));
const play=document.createElement('button');play.textContent='Play';const slider=document.createElement('input');slider.type='range';slider.min=0;slider.max=model.states.length-1;slider.step=1;slider.value=0;slider.style='width:55%';panel.append(play,slider);
const controls={robot:true,scene:true,contact:false,frame:false};
for(const [key,name] of Object.entries({robot:'All robot meshes',scene:'IDC geometry',contact:'Actual raw FCL contacts (may be SRDF-disabled)',frame:'Link frames'})){
 const label=document.createElement('label'),c=document.createElement('input');c.type='checkbox';c.checked=controls[key];c.onchange=()=>{controls[key]=c.checked;draw(+slider.value)};label.append(c,name+' ');panel.append(label);
}
const detail=document.createElement('pre');detail.style='white-space:pre-wrap;max-height:230px;overflow:auto';panel.append(detail);
const meshIndices=root.data.map((t,i)=>(t.meta||{}).role==='robot_collision_mesh'?i:-1).filter(i=>i>=0);
const mul=(t,p)=>t.slice(0,3).map(r=>r[0]*p[0]+r[1]*p[1]+r[2]*p[2]+r[3]);
const rot=(t,p)=>t.slice(0,3).map(r=>r[0]*p[0]+r[1]*p[1]+r[2]*p[2]);
const matmul=(a,b)=>a.map(r=>b[0].map((_,j)=>r.reduce((s,x,k)=>s+x*b[k][j],0)));
let timer=null,busy=false,pending=null;
async function draw(index){
 if(busy){pending=index;return;}busy=true;
 try{
 const s=model.states[index],tc=matmul(s.T_base_to_ee,model.T_ee_tcp),p=tc.slice(0,3).map(r=>r[3]);
 const pairs=[...s.semantic_blocking_pairs,...s.environment_pairs.map(p=>[p.first,p.second])],blocked=new Set(pairs.flat());
 const data=root.data.map(t=>({...t}));
 model.surfaces.forEach((surface,j)=>{const points=surface.vertices.map(v=>mul(s.link_transforms_base[surface.link],v)),t=data[meshIndices[j]];
   t.x=points.map(v=>v[0]);t.y=points.map(v=>v[1]);t.z=points.map(v=>v[2]);t.color=blocked.has(surface.link)?'#ef4444':'#6096ba';t.visible=controls.robot;});
 data.forEach(t=>{const m=t.meta||{};
 if(m.role==='scene_object'||m.role==='object_edges'){t.visible=controls.scene;if(t.type==='mesh3d')t.color=blocked.has(m.object_id)?'#f97316':'#94a3b8';}
 if(m.role==='robot_link_frame'){
   const tr=s.link_transforms_base[m.object_id],o=tr.slice(0,3).map(r=>r[3]);let axis=t.line.color==='red'?0:t.line.color==='green'?1:2;
   const e=o.map((v,j)=>v+.015*tr[j][axis]);t.x=[o[0],e[0]];t.y=[o[1],e[1]];t.z=[o[2],e[2]];t.visible=controls.frame;
 }
 if(m.role==='fcl_contacts'){const c=s.contacts_base||[];t.x=c.map(c=>c.position_base_m[0]);t.y=c.map(c=>c.position_base_m[1]);t.z=c.map(c=>c.position_base_m[2]);t.text=c.map(c=>c.first+' / '+c.second);t.visible=controls.contact;}
 if(t.name==='Actual synthetic TCP'){t.x=[p[0]];t.y=[p[1]];t.z=[p[2]];}
 for(const [name,axis] of [['Actual tool insertion axis',model.tcp.insertion_axis_local],['Actual tool key direction',model.tcp.key_axis_local]]){
   const d=rot(tc,axis),e=p.map((v,j)=>v+.035*d[j]);
   if(t.name===name&&t.type==='scatter3d'){t.x=[p[0],e[0]];t.y=[p[1],e[1]];t.z=[p[2],e[2]];}
   if(t.name===name&&t.type==='cone'){t.x=[e[0]];t.y=[e[1]];t.z=[e[2]];t.u=[d[0]];t.v=[d[1]];t.w=[d[2]];}
 }
 if(t.name==='Synthetic EE to TCP offset'||t.name==='TCP position residual'){
   const a=t.name==='Synthetic EE to TCP offset'?s.T_base_to_ee.slice(0,3).map(r=>r[3]):model.pre_tcp_position;
   t.x=[a[0],p[0]];t.y=[a[1],p[1]];t.z=[a[2],p[2]];
 }
 });
 const layout={...root.layout,title:{text:model.label+' — sample '+index+'/'+(model.states.length-1)+' — '+s.status},scene:{...root.layout.scene,uirevision:'T3-fixed-camera'},
   meta:{...root.layout.meta,path_sample_index:index,q:s.q,actual_T_base_tcp:tc,current_configuration_valid:s.valid,current_task_errors:s.task_errors,
     semantic_self_collision:s.semantic_self_collision,semantic_blocking_pairs:s.semantic_blocking_pairs,semantic_clearance:s.semantic_clearance}};
 await Plotly.react(root,data,layout,{responsive:true,displaylogo:false,scrollZoom:true});
 slider.value=index;root.dataset.pathIndex=index;root.dataset.pathValid=s.valid;
 detail.textContent=JSON.stringify({sample_index:index,q:s.q,configuration_status:s.status,reasons:s.reasons,semantic_blocking_pairs:s.semantic_blocking_pairs,environment_pairs:s.environment_pairs,semantic_clearance:s.semantic_clearance,task_errors:s.task_errors,only_endpoint_is_task_constrained:true,query_id:s.query_id,usage:'RESEARCH_ONLY'},null,2);
 }catch(error){root.dataset.replayError=String(error);throw error;}finally{busy=false;if(pending!==null){const next=pending;pending=null;draw(next);}}
}
slider.oninput=()=>draw(+slider.value);
play.onclick=()=>{if(timer){clearInterval(timer);timer=null;play.textContent='Play';return;}if(+slider.value===model.states.length-1)slider.value=0;play.textContent='Pause';timer=setInterval(()=>{if(busy)return;const i=+slider.value;if(i>=model.states.length-1){clearInterval(timer);timer=null;play.textContent='Play';}else draw(i+1);},200);};
window.pathViewer={root,model,draw};draw(0);
'''.replace('PAYLOAD', payload)
    html = pio.to_html(figure, full_html=True, include_plotlyjs=True, post_script=script,
                       config={'responsive': True, 'displaylogo': False, 'scrollZoom': True})
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix='.'+path.name, dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as f: f.write(html); f.flush(); os.fsync(f.fileno())
        os.link(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)
    return path
