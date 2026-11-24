"""Compare measured feedback archives; missing dynamic data stays visibly NOT RUN."""
import argparse
import csv
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--results',type=Path,required=True)
    p.add_argument('--dynamic-trial',type=Path);a=p.parse_args();root=a.results
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    datasets={label:list(csv.DictReader((root/(label+'_feedback.csv')).open())) for label in ('static','dynamic')}
    def missing(ax):
        ax.text(.5,.5,'NOT RUN\nFeedback fault before motion\nNo measured dynamic data',ha='center',va='center',transform=ax.transAxes)
        ax.set_xticks([]);ax.set_yticks([])
    def save(fig,name):
        fig.tight_layout();fig.savefig(root/name,dpi=160);plt.close(fig)
    dynamic=datasets['dynamic']
    if not dynamic:
        for name,title in [('dynamic_pulses.png','Dynamic six-servo feedback'),('target_feedback.png','Dynamic target / feedback comparison')]:
            fig,ax=plt.subplots(figsize=(9,5));missing(ax);ax.set_title(title);save(fig,name)
    else:
        trial=json.loads(a.dynamic_trial.read_text());manifest=trial['stop_recording']
        from deployment.local.analyze_feedback_trial import plot
        plot(dynamic,root,'dynamic',manifest)
        t0=manifest['started_monotonic_ns']
        commands=[c for c in trial['final_status']['commands'] if c.get('kind')=='position' and c['status']=='SENT']
        fig,axes=plt.subplots(3,2,figsize=(11,8))
        for ax,i in zip(axes.flat,(1,2,3,4,5,10)):
            data=[r for r in dynamic if int(r['servo_id'])==i]
            ax.plot([(int(r['response_received_at_ns'])-t0)/1e9 for r in data],[int(r['raw_pulse']) for r in data],label='RRC outer feedback')
            if str(i) in trial['plan']['A']:
                points=[(0,trial['plan']['A'][str(i)])]+[((c['send_finished_monotonic_ns']-t0)/1e9,c['target_pulses'][str(i)]) for c in commands]
                points.append(((manifest['stopped_monotonic_ns']-t0)/1e9,points[-1][1]))
                ax.step([v[0] for v in points],[v[1] for v in points],where='post',ls='--',label='A reference / sent endpoints')
            ax.set(title='Servo '+str(i),xlabel='Host seconds',ylabel='Pulse');ax.legend();ax.grid(alpha=.2)
        fig.suptitle('Endpoint steps are not device trajectories or communication latency');save(fig,'target_feedback.png')
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for ax,(label,rows) in zip(axes,datasets.items()):
        if not rows:missing(ax)
        else:
            ax.hist([float(r['read_latency_ns'])/1e6 for r in rows if r.get('read_latency_ns')],bins=50)
            ax.set(xlabel='Host transaction latency (ms)',ylabel='Queries');ax.grid(alpha=.2)
        ax.set_title(label)
    save(fig,'latency_comparison.png')
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    for column,(label,rows) in enumerate(datasets.items()):
        if not rows:
            for ax in axes[:,column]:missing(ax)
            continue
        groups={}
        for row in rows:groups.setdefault(int(row['sequence']),[]).append(row)
        groups=[v for v in groups.values() if len(v)==6]
        starts=[min(int(r['read_started_at_ns']) for r in v) for v in groups];t0=starts[0]
        spans=[(max(int(r['response_received_at_ns']) for r in v)-s)/1e6 for s,v in zip(starts,groups)]
        x=[(v-t0)/1e9 for v in starts]
        axes[0,column].plot(x,spans,lw=.8);axes[0,column].set(title=label+' six-host span',ylabel='ms')
        axes[1,column].plot(x[1:],[1e9/(b-a) for a,b in zip(starts,starts[1:])],lw=.8)
        axes[1,column].axhline(20,color='grey',ls='--');axes[1,column].set(xlabel='Host seconds',ylabel='Adjacent-cycle Hz')
        for ax in axes[:,column]:ax.grid(alpha=.2)
    fig.suptitle('Host intervals; device sample times unverified');save(fig,'span_frequency_comparison.png')

if __name__=='__main__':main()
