"""Task target/TCP overlays on the existing full robot collision figure."""
import numpy as np
from .collision import build_collision_figure


def build_task_figure(scene,surfaces,row,targets,assessment):
    """Consume real native geometry plus declared task data; no FK/IK/collision here."""
    import plotly.graph_objects as go
    fig=build_collision_figure(scene,surfaces,row)
    payload=targets.to_dict();actual=np.asarray(row['T_base_to_ee'])@targets.spec.T_ee_tcp
    def point(name,p,color,symbol='diamond'):
        fig.add_trace(go.Scatter3d(x=[p[0]],y=[p[1]],z=[p[2]],mode='markers+text',text=[name],textposition='top center',
            marker={'size':6,'color':color,'symbol':symbol},name=name,meta={'role':'task_reference','source':'Scene FrameGraph / declared synthetic task / actual native state'}))
    def axis(name,p,d,color,length=.045):
        p=np.array(p);d=np.array(d);end=p+length*d
        fig.add_trace(go.Scatter3d(x=[p[0],end[0]],y=[p[1],end[1]],z=[p[2],end[2]],mode='lines',name=name,line={'width':7,'color':color},meta={'role':'task_axis'}))
        fig.add_trace(go.Cone(x=[end[0]],y=[end[1]],z=[end[2]],u=[d[0]],v=[d[1]],w=[d[2]],anchor='tip',sizemode='absolute',sizeref=.006,
            colorscale=[[0,color],[1,color]],showscale=False,showlegend=False,name=name,meta={'role':'task_axis_arrow'}))
    port=np.array(targets.base_to_port.position_m);pre=np.array(targets.pre_tcp.position_m)
    port_id=targets.spec.data['target']['port_id']
    point(port_id+' port',port,'#0891b2');point('Pre-insertion target TCP',pre,'#16a34a');point('Actual synthetic TCP',actual[:3,3],'#c026d3','cross')
    axis('Port insertion direction',port,targets.insertion_axis_base,'#ef4444')
    axis('Port key reference',port,targets.key_axis_base,'#2563eb')
    data=targets.spec.data['tcp']
    axis('Actual tool insertion axis',actual[:3,3],actual[:3,:3]@data['insertion_axis_local'],'#db2777',.035)
    axis('Actual tool key direction',actual[:3,3],actual[:3,:3]@data['key_axis_local'],'#7c3aed',.035)
    for i,color in enumerate(['#ef4444','#22c55e','#3b82f6']):axis('Target TCP '+['X','Y','Z'][i],pre,targets.T_base_pre_tcp[:3,i],color,.018)
    for name,ends,color in [('Geometric approach reference (not a planned path)',np.array([pre,port]),'#64748b'),('Synthetic EE to TCP offset',np.array([np.array(row['T_base_to_ee'])[:3,3],actual[:3,3]]),'#c026d3'),('TCP position residual',np.array([pre,actual[:3,3]]),'#f59e0b')]:
        fig.add_trace(go.Scatter3d(x=ends[:,0],y=ends[:,1],z=ends[:,2],mode='lines',line={'color':color,'width':4,'dash':'dash'},name=name,meta={'role':'task_reference_line'}))
    e=assessment['errors'];fig.update_layout(title=f"{port_id} {assessment['status']} — position {e['position_mm']:.4f}mm / axis {e['insertion_axis_deg']:.4f}° / key {e['key_axis_deg']:.4f}°",
        meta={**dict(fig.layout.meta),'task_targets':payload,'task_check':assessment,'actual_T_base_tcp':actual.tolist(),'usage':'RESEARCH_ONLY','stage':'T1_T2','viewer_stage':'T1_T2','tcp_geometry_modeled':False})
    fig.add_annotation(text='Synthetic, uncalibrated TCP / approach / tolerances. Bare-gripper collision only; no RJ45/cable body. Dashed approach is a target reference, not a planned or checked path.',xref='paper',yref='paper',x=0,y=1.04,showarrow=False)
    return fig
