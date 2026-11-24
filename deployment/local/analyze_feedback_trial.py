"""Measured host statistics and research plots; never opens a robot device."""
import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import statistics

IDS=(1,2,3,4,5,10)

def percentile(values,p):
    return sorted(values)[max(0,math.ceil(len(values)*p)-1)] if values else None

def metrics(rows,manifest):
    groups={}
    valid=[]
    for row in rows:
        groups.setdefault(int(row['sequence']),[]).append(row)
        if row.get('outer_frame_validated')=='True' and not row.get('error'):
            valid.append(row)
    full=[g for g in groups.values() if len(g)==6 and {int(r['servo_id']) for r in g}==set(IDS)
          and all(r.get('outer_frame_validated')=='True' and not r.get('error') for r in g)]
    starts=sorted(min(int(r['read_started_at_ns']) for r in g) for g in full)
    durations=[float(r['read_latency_ns'])/1e6 for r in valid if r.get('read_latency_ns')]
    spans=[(max(int(r['response_received_at_ns']) for r in g)-min(int(r['read_started_at_ns']) for r in g))/1e6 for g in full]
    seconds=(manifest['stopped_monotonic_ns']-manifest['started_monotonic_ns'])/1e9
    result=dict(status='MEASURED',recording_complete=manifest.get('complete'),recording_seconds=seconds,
        complete_snapshots=len(full),queries_attempted=len(rows),valid_outer_replies=len(valid),
        outer_query_success_fraction=len(valid)/len(rows) if rows else None,
        actual_update_hz=(len(starts)-1)*1e9/(starts[-1]-starts[0]) if len(starts)>1 else None,
        complete_snapshots_per_recording_second=len(full)/seconds if seconds else None,
        mean_latency_ms=statistics.mean(durations) if durations else None,
        p95_latency_ms=percentile(durations,.95),p99_latency_ms=percentile(durations,.99),
        max_latency_ms=max(durations) if durations else None,
        mean_host_span_ms=statistics.mean(spans) if spans else None,
        p95_host_span_ms=percentile(spans,.95),max_host_span_ms=max(spans) if spans else None,
        incomplete_snapshots=len(groups)-len(full),skipped_periods=sum(int(g[0]['skipped_periods'] or 0) for g in groups.values()),
        source_type='unknown',device_sample_time_ns=None,inner_association_verified=False,
        physical_calibrated=False,canonical_observation_eligible=False,p95_p99_method='nearest rank')
    result['per_id']={}
    for i in IDS:
        data=[int(r['raw_pulse']) for r in valid if int(r['servo_id'])==i]
        lat=[float(r['read_latency_ns'])/1e6 for r in valid if int(r['servo_id'])==i and r.get('read_latency_ns')]
        result['per_id'][str(i)]=dict(valid_outer_replies=len(data),min_pulse=min(data) if data else None,
            max_pulse=max(data) if data else None,pulse_population_stddev=statistics.pstdev(data) if data else None,
            mean_latency_ms=statistics.mean(lat) if lat else None,p95_latency_ms=percentile(lat,.95),
            p99_latency_ms=percentile(lat,.99),max_latency_ms=max(lat) if lat else None)
    return result


def plot(rows,out,label,manifest):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t0=manifest['started_monotonic_ns']
    fig,axes=plt.subplots(3,2,figsize=(11,8),sharex=True)
    for ax,i in zip(axes.flat,IDS):
        d=[r for r in rows if int(r['servo_id'])==i and r.get('raw_pulse') and r.get('response_received_at_ns')]
        ax.plot([(int(r['response_received_at_ns'])-t0)/1e9 for r in d],[int(r['raw_pulse']) for r in d],lw=.8)
        ax.set_title('Servo '+str(i));ax.set_ylabel('Pulse');ax.grid(alpha=.25)
    fig.suptitle(label+' feedback: host receive time; source/physical calibration unverified')
    for ax in axes[-1]:ax.set_xlabel('Host monotonic seconds since recording start')
    fig.tight_layout();fig.savefig(out/(label+'_pulses.png'),dpi=160);plt.close(fig)
    lat=[float(r['read_latency_ns'])/1e6 for r in rows if r.get('read_latency_ns')]
    fig,ax=plt.subplots(figsize=(8,4));ax.hist(lat,bins=50)
    ax.set(xlabel='Host transaction latency (ms)',ylabel='Queries',title=label+' read latency')
    ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/(label+'_latency.png'),dpi=160);plt.close(fig)
    groups={}
    for r in rows:groups.setdefault(int(r['sequence']),[]).append(r)
    g=[v for v in groups.values() if len(v)==6 and all(x.get('read_started_at_ns') and x.get('response_received_at_ns') for x in v)]
    starts=[min(int(r['read_started_at_ns']) for r in v) for v in g]
    spans=[(max(int(r['response_received_at_ns']) for r in v)-s)/1e6 for s,v in zip(starts,g)]
    x=[(v-t0)/1e9 for v in starts]
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    axes[0].plot(x,spans,lw=.8);axes[0].set_ylabel('Six-host span (ms)')
    axes[1].plot(x[1:],[1e9/(b-a) for a,b in zip(starts,starts[1:])],lw=.8)
    axes[1].axhline(20,color='grey',ls='--',label='Requested 20 Hz');axes[1].legend()
    axes[1].set(xlabel='Host monotonic seconds',ylabel='Adjacent-cycle Hz')
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle(label+' host timing; not device acquisition timing')
    fig.tight_layout();fig.savefig(out/(label+'_span_frequency.png'),dpi=160);plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trial',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--label',choices=('static','dynamic'),required=True)
    a=p.parse_args();trial=json.loads(a.trial.read_text());manifest=trial['stop_recording']
    a.output.mkdir(parents=True,exist_ok=True)
    source=Path(manifest['directory'])/'feedback.csv'
    rows=list(csv.DictReader(source.open()))
    target=a.output/(a.label+'_feedback.csv')
    if target.exists():raise FileExistsError(target)
    shutil.copyfile(source,target)
    result=metrics(rows,manifest)
    (a.output/(a.label+'_metrics.json')).write_text(json.dumps(result,indent=2)+'\n')
    plot(rows,a.output,a.label,manifest);print(json.dumps(result,indent=2))

if __name__=='__main__':main()
