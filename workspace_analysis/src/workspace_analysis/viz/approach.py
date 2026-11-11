"""T4 overlays on the existing native-state path replay; no FK/IK/FCL here."""
import numpy as np
from .joint_path import build_path_figure, export_path_viewer


def export_approach_viewer(scene, surfaces, native_start, targets, initial_assessment, result, output, *, label):
    import plotly.graph_objects as go
    # Include first failed sample as a red diagnostic, never later invalid motion.
    stop = result['first_failed_sample_index']
    states = result['samples'] if stop is None else result['samples'][:stop+1]
    if not states: raise ValueError('no native sample available for approach replay')
    if any('link_transforms_base' not in s for s in states): raise ValueError('native state missing for approach replay')
    figure, local = build_path_figure(scene, surfaces, native_start, targets, initial_assessment, states, label)
    requested = np.array([np.asarray(w['target']['T_base_pre_tcp'])[:3, 3] for w in result['waypoints']])
    end = targets.port_tcp.matrix()[:3, 3]-result['settings']['end_distance_m']*np.asarray(targets.insertion_axis_base)
    start = targets.pre_tcp.matrix()[:3, 3]
    for trace in figure.data:
        if trace.name == 'Geometric approach reference (not a planned path)':
            trace.x=[start[0],end[0]]; trace.y=[start[1],end[1]]; trace.z=[start[2],end[2]]
            trace.name='Requested Cartesian approach (not insertion)'
        if trace.name == 'Simulation start (not hardware Home)': trace.name='T2 pre-insertion start (not hardware Home)'
    figure.add_trace(go.Scatter3d(x=requested[:,0],y=requested[:,1],z=requested[:,2],mode='markers+text',
        text=[f"{1000*w['distance_to_port_m']:.1f} mm" for w in result['waypoints']],marker={'size':5,'color':'#06b6d4'},
        textposition='top center',name='Attempted Cartesian waypoint targets',meta={'role':'approach_waypoint_targets'}))
    figure.add_trace(go.Scatter3d(x=[start[0]],y=[start[1]],z=[start[2]],mode='markers',marker={'size':7,'symbol':'diamond','color':'#06b6d4'},name='Approach expected TCP'))
    last=result['last_valid_sample'];failure=result['first_failure']
    if last is not None:
        p=np.asarray(last['actual_T_base_tcp'])[:3,3]
        figure.add_trace(go.Scatter3d(x=[p[0]],y=[p[1]],z=[p[2]],mode='markers',marker={'size':8,'color':'#f97316'},name='Last validated approach TCP'))
    if failure is not None:
        p=np.asarray(failure['expected_T_base_tcp'])[:3,3]
        figure.add_trace(go.Scatter3d(x=[p[0]],y=[p[1]],z=[p[2]],mode='markers',marker={'size':9,'symbol':'x','color':'#ef4444'},name='First failed expected TCP'))
    context={k:result[k] for k in ['status','accepted_waypoints','requested_waypoints','last_accepted_waypoint_distance_m',
        'last_valid_expected_distance_m','validated_progress_m','first_failure','settings']}
    context.update(all_samples_task_constrained=True,stage='T4',insertion_success=False,usage='RESEARCH_ONLY')
    figure.update_layout(meta={**dict(figure.layout.meta),'stage':'T4','viewer_stage':'T4','all_samples_task_constrained':True,'approach_result':context})
    return export_path_viewer(figure,local,states,targets,output,label=label,replay_context=context)
