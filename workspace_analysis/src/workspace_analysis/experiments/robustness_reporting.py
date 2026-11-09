"""Descriptive paired summaries; failures remain in declared search denominators."""
from collections import Counter
import numpy as np


def distribution(values):
    data=np.asarray([v for v in values if v is not None],dtype=float)
    if not len(data):return {'n':0,'minimum':None,'median':None,'maximum':None,'mean':None,'std_population':None,'q25':None,'q75':None}
    if not np.isfinite(data).all():raise ValueError('nonfinite observed metric')
    return {'n':len(data),'minimum':float(data.min()),'median':float(np.median(data)),
        'maximum':float(data.max()),'mean':float(data.mean()),'std_population':float(data.std()),
        'q25':float(np.quantile(data,.25)),'q75':float(np.quantile(data,.75))}


def robustness_groups(rows):
    output=[]
    for key in sorted(set((r['experiment'],r['case'],r['start']) for r in rows)):
        group=[r for r in rows if (r['experiment'],r['case'],r['start'])==key]
        accepted=[r for r in group if r['success']]
        output.append({'experiment':key[0],'case':key[1],'start':key[2],'scheduled':len(group),
            'executed':sum(r['status']!='NOT_RUN' for r in group),'accepted':len(accepted),
            'fraction':len(accepted)/len(group),'termination_counts':dict(Counter(r['termination'] for r in group)),
            'observed_search_time_s':distribution([r['search_time_s'] for r in group]),
            'successful_search_time_s':distribution([r['search_time_s'] for r in accepted]),
            'successful_joint_length_rad':distribution([r['joint_length_rad'] for r in accepted]),
            'successful_tcp_length_m':distribution([r['tcp_length_m'] for r in accepted]),
            'successful_environment_clearance_m':distribution([r['min_environment_clearance_m'] for r in accepted]),
            'note':'timeout times are censored observations, not completion times; length distributions only use accepted paths'})
    return output


def paired_optimization(raw_rows,optimized_rows,index):
    by_path={r['trial']:r for r in optimized_rows};pairs=[]
    for record in index:
        raw=next(r for r in raw_rows if r['trial']==record['raw_trial'])
        optimized=None if record['optimized_trial'] is None else by_path[record['optimized_trial']]
        row={'raw_trial':raw['trial'],'case':raw['case'],'start':raw['start'],'seed':raw['seed'],'mode':raw['mode'],
            'raw_search_success':raw['success'],'optimization_status':record['status'],
            'raw_joint_length_rad':raw['joint_length_rad'],'raw_tcp_length_m':raw['tcp_length_m'],
            'raw_waypoint_count':raw.get('waypoint_count'),'raw_total_time_s':raw.get('total_time_s'),
            'optimized_waypoint_count':None,'optimized_total_time_s':None,'shortcut_time_s':None,
            'raw_environment_clearance_m':raw['min_environment_clearance_m'],'optimized_environment_clearance_m':None,
            'optimized_joint_length_rad':None,'optimized_tcp_length_m':None,'joint_reduction_fraction':None,
            'tcp_reduction_fraction':None,'environment_clearance_change_m':None,'optimized_trial':record['optimized_trial']}
        if optimized is not None and optimized['success']:
            old=raw['joint_length_rad'];new=optimized['joint_length_rad']
            if new>old+1e-12:raise ValueError('accepted optimization increased joint length')
            row.update(optimized_joint_length_rad=new,optimized_tcp_length_m=optimized['tcp_length_m'],
                optimized_waypoint_count=optimized.get('waypoint_count'),optimized_total_time_s=optimized.get('total_time_s'),
                shortcut_time_s=optimized.get('shortcut_time_s'),optimized_environment_clearance_m=optimized['min_environment_clearance_m'],
                joint_reduction_fraction=None if old<=1e-12 else (old-new)/old,
                tcp_reduction_fraction=None if raw['tcp_length_m']<=1e-12 else (raw['tcp_length_m']-optimized['tcp_length_m'])/raw['tcp_length_m'],
                environment_clearance_change_m=optimized['min_environment_clearance_m']-raw['min_environment_clearance_m'])
        pairs.append(row)
    return pairs
