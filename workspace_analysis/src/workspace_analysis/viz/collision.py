"""Complete collision-surface viewer; consumes native query data, never computes FK/collisions."""
from html import escape
import json
import os
from pathlib import Path
import tempfile
import numpy as np
from .scene import build_scene_figure

CAMERA={'eye':{'x':-1.25,'y':-1.15,'z':.9},'up':{'x':0,'y':0,'z':1}}


def build_collision_figure(scene,surfaces,row):
    """Reuse actual scene solids; add all verified robot collision triangles."""
    import plotly.graph_objects as go
    figure=build_scene_figure(scene,axis_length=.015)
    for trace in figure.data:
        role=(trace.meta or {}).get('role')
        if role in {'frame_axis','frame_label','port_direction','port_arrowhead'}:trace.visible=False
        elif role=='scene_object':trace.opacity=.24
    for surface in surfaces:
        v=surface['vertices'];f=surface['faces'];name=surface['id']
        figure.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],
            color='#6096ba',opacity=.78,flatshading=True,name='Robot: '+name,showlegend=True,
            legendgroup='robot:'+name,hovertemplate=escape(name)+'<br>Collision Mesh XYZ [m]: (%{x:.5f}, %{y:.5f}, %{z:.5f})<extra></extra>',
            meta={'role':'robot_collision_mesh','object_id':name,'geometry_index':surface['geometry_index'],
                  'resource_sha256':surface['sha256'],'triangle_count':len(f)}))
    for name,t in row['link_transforms_base'].items():
        t=np.array(t);o=t[:3,3]
        for axis,color in zip(range(3),['red','green','blue']):
            end=o+.015*t[:3,axis]
            figure.add_trace(go.Scatter3d(x=[o[0],end[0]],y=[o[1],end[1]],z=[o[2],end[2]],
                mode='lines',line={'color':color,'width':3},name='Link frame: '+name,visible=False,showlegend=False,
                meta={'role':'robot_link_frame','object_id':name}))
    contacts=row.get('contacts_base',[])
    points=np.array([x['position_base_m'] for x in contacts]).reshape(-1,3)
    figure.add_trace(go.Scatter3d(x=points[:,0],y=points[:,1],z=points[:,2],mode='markers',
        marker={'size':5,'color':'#facc15','symbol':'diamond'},name='FCL contacts (base_link)',
        text=[x['first']+' / '+x['second'] for x in contacts],hovertemplate='%{text}<br>FCL contact (%{x:.5f}, %{y:.5f}, %{z:.5f}) m<extra></extra>',
        meta={'role':'fcl_contacts','contact_source':'actual Native CollisionResult','pairs':contacts}))
    if 'task_scope' in row:
        scoped=row['task_scope'];internal={tuple(sorted(p)) for p in scoped['gripper_internal_pairs']};blocking={tuple(sorted(p)) for p in scoped['task_scope_blocking_pairs']}
        figure.data[-1].visible=False;figure.data[-1].meta={**figure.data[-1].meta,'suppressed_for_task_scope':True}
        groups=[('Internal Diagnostic — UNVERIFIED',scoped['gripper_internal_contacts'],'#a855f7','scope_internal_contacts'),('Blocking Self Contacts',[c for c in contacts if c['kind']=='self' and tuple(sorted([c['first'],c['second']])) in blocking],'#ef4444','scope_blocking_contacts'),('Environment Contacts',[c for c in contacts if c['kind']=='environment'],'#f97316','scope_environment_contacts')]
        for name,items,color,role in groups:
            pts=np.array([c['position_base_m'] for c in items]).reshape(-1,3)
            figure.add_trace(go.Scatter3d(x=pts[:,0],y=pts[:,1],z=pts[:,2],mode='markers',marker={'color':color,'size':5},name=name,text=[c['first']+' / '+c['second'] for c in items],meta={'role':role,'pairs':items,'surface_complete':False}))
        figure.add_annotation(text='Purple: internal mechanism diagnostics UNVERIFIED. Red: blocking self. Orange: environment. Task candidate never means full robot/hardware safe.',xref='paper',yref='paper',x=0,y=1.03,showarrow=False)
    # Fixed metric ranges encompass every mesh and scene solid; identical camera
    # and a generous common range for JetRover samples, expanded if necessary.
    points=np.vstack([s['vertices'] for s in surfaces]+[np.column_stack([t.x,t.y,t.z]) for t in figure.data if (t.meta or {}).get('role')=='scene_object'])
    bounds=[[-.30,.60],[-.30,.30],[-.20,.60]]
    for axis in range(3):bounds[axis]=[min(bounds[axis][0],float(points[:,axis].min())-.03),max(bounds[axis][1],float(points[:,axis].max())+.03)]
    figure.update_layout(height=850,margin={'l':20,'r':10,'t':70,'b':90},
        title=f"Collision geometry — sample {row['sample_index']} — {row['effective_decision']}",
        scene={'camera':CAMERA,'aspectmode':'data','xaxis':{'range':bounds[0]},'yaxis':{'range':bounds[1]},'zaxis':{'range':bounds[2]}},
        meta={**dict(figure.layout.meta),'stage':'C2.5','collision_checked':True,'viewer_stage':'C2.5','sample_index':row['sample_index'],
              'q':row['q'],'query_identity':row['identity'],'robot_mesh_count':len(surfaces),
              'contacts_available':bool(contacts),'missing_collision_link':'lidar_link'})
    figure.layout.annotations=()
    figure.add_annotation(x=0,y=-.08,xref='paper',yref='paper',showarrow=False,xanchor='left',align='left',
        text='Geometry: blue robot collision Mesh; translucent IDC solids. Pair: red / orange; actual FCL contacts: yellow.<br>'
             'All 24 modeled collision Mesh shown. LiDAR / RJ45 / cable coverage incomplete; physical policy validation PARTIAL.')
    if 'task_scope' in row:
        scoped=row['task_scope'];figure.update_layout(title=f'Task scope {"CANDIDATE (not full safe)" if scoped["zero_clearance_candidate"] else "REJECTED"} — official full {row["effective_decision"]} — {row["sample_index"]}',meta={**dict(figure.layout.meta),'task_scope':scoped})
        figure.add_annotation(text='Purple: internal mechanism diagnostics UNVERIFIED. Red: blocking self. Orange: environment. Task candidate never means full robot/hardware safe.',xref='paper',yref='paper',x=0,y=1.03,showarrow=False)
    return figure


def export_collision_viewer(figure,row,output):
    """Single offline HTML with pair selection and independent layer switches."""
    import plotly.io as pio
    path=Path(output).absolute()
    if path.suffix!='.html' or os.path.lexists(path):raise ValueError('new .html output required')
    pairs=[];filtered={tuple(sorted(p)) for p in row.get('filtered_pairs',[])}
    effective={tuple(sorted(p)) for p in row.get('effective_self_pairs',[])}
    for pair in row['colliding_pairs']:
        names=tuple(sorted([pair['first'],pair['second']]))
        mode='environment' if pair['kind']=='environment' else 'filtered' if names in filtered else 'effective'
        pairs.append({**pair,'mode':mode,'effective':pair['kind']=='environment' or names in effective})
    scoped=row.get('task_scope')
    if scoped:
        internals={tuple(sorted(p)) for p in scoped['gripper_internal_pairs']};blockers={tuple(sorted(p)) for p in scoped['task_scope_blocking_pairs']}
        seen={tuple(sorted([p['first'],p['second']])) for p in pairs if p['kind']=='self'}
        for names in sorted((internals|blockers)-seen):pairs.append({'kind':'self','first':names[0],'second':names[1],'mode':'internal' if names in internals else 'blocking','effective':True})
        for p in pairs:
            key=tuple(sorted([p['first'],p['second']]))
            if p['kind']=='self' and key in internals:p['mode']='internal'
            elif p['kind']=='self' and key in blockers:p['mode']='blocking'
    payload=json.dumps({'pairs':pairs,'q':row['q'],'index':row['sample_index'],'decision':row['effective_decision'],
                         'policy':row.get('applied_policy'),'identity':row['identity'],'task_scope':scoped},allow_nan=False).replace('<','\\u003c')
    script=r'''
const root=document.getElementById('{plot_id}');
const model=PAYLOAD;
const panel=document.createElement('section'); panel.style='padding:16px;font:14px system-ui;background:#f1f5f9;max-height:260px;overflow:auto';
root.parentNode.insertBefore(panel,root);
panel.innerHTML='<b>Robot Collision Mesh · base_link · units m/rad · LiDAR/tool coverage incomplete</b><br><label>Pair filter <select id="pair-filter"><option value="raw">Raw (all)</option><option value="effective">Effective</option><option value="filtered">ACM filtered</option><option value="environment">Environment</option></select></label> <select id="pair-select" style="max-width:75%"></select><br>';
const layers={robot:true,scene:true,contact:true,frame:false};
for(const [key,label] of Object.entries({robot:'Robot Mesh (all 24)',scene:'Scene Geometry',contact:'Actual FCL Contacts',frame:'Link Frames'})){
 const l=document.createElement('label');const c=document.createElement('input');c.type='checkbox';c.checked=layers[key];c.id='layer-'+key;c.onchange=()=>{layers[key]=c.checked;render()};l.append(c,label+'  ');panel.append(l);
}
const detail=document.createElement('pre');detail.style='white-space:pre-wrap';panel.append(detail);
const select=panel.querySelector('#pair-select'),filter=panel.querySelector('#pair-filter');
if(model.task_scope){filter.add(new Option('Task Blocking Self','blocking'));filter.add(new Option('Internal Diagnostic (UNVERIFIED)','internal'));const note=document.createElement('b');note.textContent=' Task zero-clearance candidate: '+model.task_scope.zero_clearance_candidate+'; official full decision: '+model.decision+'; internal mechanics UNVERIFIED';panel.prepend(note);}
function choices(){select.replaceChildren();let first=new Option('All objects; no pair selection','-1');select.add(first);
model.pairs.forEach((p,i)=>{if(filter.value==='raw'||(filter.value==='effective'&&p.effective)||filter.value===p.mode)select.add(new Option(p.kind+' / '+p.mode+': '+p.first+' ↔ '+p.second,''+i));}); select.value=select.options.length>1?select.options[1].value:'-1';render();}
const defaults=root.data.map(t=>({color:t.color,opacity:t.opacity,visible:t.visible!==false}));
function render(){const pair=model.pairs[+select.value];const selected=pair?[pair.first,pair.second]:[];
 const updates={visible:[],color:[],opacity:[]};
 root.data.forEach((t,i)=>{const m=t.meta||{}, role=m.role;let visible=defaults[i].visible;
 if(role==='robot_collision_mesh')visible=layers.robot;
 if(role==='scene_object'||role==='object_edges')visible=layers.scene;
 if(role==='robot_link_frame')visible=layers.frame;
 if(['fcl_contacts','scope_internal_contacts','scope_blocking_contacts','scope_environment_contacts'].includes(role))visible=layers.contact&&!m.suppressed_for_task_scope;
 let color=defaults[i].color,opacity=defaults[i].opacity;
 if(role==='robot_collision_mesh'||role==='scene_object'){
 const which=selected.indexOf(m.object_id);color=which===0?(pair&&pair.mode==='internal'?'#a855f7':'#ef4444'):which===1?(pair&&pair.mode==='internal'?'#38bdf8':'#f59e0b'):defaults[i].color;
 opacity=which>=0?.98:selected.length?(role==='robot_collision_mesh'?.48:.12):defaults[i].opacity;
 }
 if(['fcl_contacts','scope_internal_contacts','scope_blocking_contacts','scope_environment_contacts'].includes(role)&&pair){const points=m.pairs.filter(p=>(p.kind===pair.kind||p.kind==='internal_diagnostic'&&pair.mode==='internal')&&((p.first===pair.first&&p.second===pair.second)||(p.first===pair.second&&p.second===pair.first)));
 Plotly.restyle(root,{x:[points.map(p=>p.position_base_m[0])],y:[points.map(p=>p.position_base_m[1])],z:[points.map(p=>p.position_base_m[2])],text:[points.map(p=>p.first+' / '+p.second)]},[i]);}
 if(['fcl_contacts','scope_internal_contacts','scope_blocking_contacts','scope_environment_contacts'].includes(role)&&!pair){const points=m.pairs;Plotly.restyle(root,{x:[points.map(p=>p.position_base_m[0])],y:[points.map(p=>p.position_base_m[1])],z:[points.map(p=>p.position_base_m[2])],text:[points.map(p=>p.first+' / '+p.second)]},[i]);}
 updates.visible.push(visible);updates.color.push(color);updates.opacity.push(opacity);
 });
 // Only Mesh3d supports color as a top-level property.
 const meshes=root.data.map((t,i)=>t.type==='mesh3d'?i:-1).filter(i=>i>=0);
 Plotly.restyle(root,{visible:updates.visible});
 Plotly.restyle(root,{color:meshes.map(i=>updates.color[i]),opacity:meshes.map(i=>updates.opacity[i])},meshes);
 detail.textContent=JSON.stringify({sample_index:model.index,q:model.q,decision:model.decision,task_scope:model.task_scope,selected_pair:pair||null,policy:model.policy,identity:model.identity},null,2);
 root.dataset.selectedPair=pair?pair.first+'|'+pair.second:'none';
}
select.onchange=render;filter.onchange=choices;choices();
window.collisionViewer={root,model,render,choices};
'''.replace('PAYLOAD',payload)
    html=pio.to_html(figure,full_html=True,include_plotlyjs=True,include_mathjax=False,post_script=script,
                     config={'responsive':True,'displaylogo':False,'scrollZoom':True})
    path.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(prefix='.'+path.name,dir=path.parent)
    try:
        with os.fdopen(fd,'w') as f:f.write(html);f.flush();os.fsync(f.fileno())
        os.link(tmp,path)
    finally:Path(tmp).unlink(missing_ok=True)
    return path
