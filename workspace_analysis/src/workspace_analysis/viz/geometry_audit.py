"""Local original/derived surfaces alongside actual bounded native contacts."""
import numpy as np


def local_pair_figure(row,official_surfaces,derived_surfaces,first,second,joint_name,joint_transform,joint_axis):
    import plotly.graph_objects as go
    transform=np.asarray(joint_transform);inverse=np.linalg.inv(transform)
    fig=go.Figure();variants=[]
    for label,surfaces in [('Official',official_surfaces),('Derived target-pair diagnostic',derived_surfaces)]:
        for name,color in [(first,'#ef4444'),(second,'#3b82f6')]:
            mesh=next(s for s in surfaces if s['id']==name);v=mesh['vertices']@inverse[:3,:3].T+inverse[:3,3];f=mesh['faces']
            fig.add_trace(go.Mesh3d(x=v[:,0],y=v[:,1],z=v[:,2],i=f[:,0],j=f[:,1],k=f[:,2],name=label+': '+name,color=color,opacity=.55,flatshading=True,visible=label=='Official',meta={'role':'local_collision_mesh','variant':label,'object_id':name,'sha256':mesh['sha256'],'triangles':len(f)}));variants.append(label)
    for label,contacts,color in [('Official native contacts',row.get('official_contacts_base',row['contacts_base']),'#f59e0b'),('Profile native contacts',row['contacts_base'],'#16a34a')]:
        points=np.array([c['position_base_m'] for c in contacts if c['kind']=='self' and {c['first'],c['second']}=={first,second}]).reshape(-1,3);p=points@inverse[:3,:3].T+inverse[:3,3]
        fig.add_trace(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode='markers',name=label,marker={'size':3,'color':color},meta={'role':'actual_native_contact_points','surface_complete':False}));variants.append('always')
    axis=np.asarray(joint_axis,dtype=float);axis/=np.linalg.norm(axis);p=np.vstack([-.025*axis,.025*axis]);fig.add_trace(go.Scatter3d(x=p[:,0],y=p[:,1],z=p[:,2],mode='lines',line={'color':'black','width':7},name='Joint axis: '+joint_name));variants.append('always')
    fig.update_layout(title=f'{first} / {second}: joint zero-axis frame; sample {row["sample_index"]}',scene={'aspectmode':'data','xaxis_title':'X [m]','yaxis_title':'Y [m]','zaxis_title':'Z [m]'},height=850,meta={'q':row['q'],'identity':row['identity'],'contact_collection':row.get('contact_collection'),'derived_contact_collection':row.get('derived_contact_collection'),'joint':joint_name},updatemenus=[{'buttons':[{'label':label,'method':'restyle','args':[{'visible':[v in [label,'always'] for v in variants]}]} for label in ['Official','Derived target-pair diagnostic']]}])
    fig.add_annotation(text='Pair-local diagnostic surfaces only. Environment / non-target self use complete official Mesh. Finite FCL contacts are not a complete contact surface.',xref='paper',yref='paper',x=0,y=-.1,showarrow=False)
    return fig
