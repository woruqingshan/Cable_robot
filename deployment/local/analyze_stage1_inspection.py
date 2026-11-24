"""Analyze real Stage1 runs; never labels native actions as robot trajectories."""
import argparse,json,math,statistics
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('case',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 cloud={r['request_id']:r for r in json.loads((a.case/'cloud_summary.json').read_text())};all_rows=[];runs={};requests=[]
 def stats(v):
  v=sorted(v)
  return dict(mean=statistics.mean(v),p50=v[max(0,math.ceil(len(v)*.5)-1)],p95=v[max(0,math.ceil(len(v)*.95)-1)],max=max(v)) if v else None
 for kind in ['live_single','live_continuous']:
  root=next((a.case/kind).iterdir());m=json.loads((root/'metrics.json').read_text());rows=[json.loads(l) for l in (root/'inference_responses.jsonl').read_text().splitlines()]
  for row in rows:
   c=cloud[row['request_id']];assert c['finite'] and row['shape']==[50,6];assert all(math.isfinite(v) for step in row['values'] for v in step);row['cloud_model_s']=c['runtime']['last_inference']['model_s'];row['cloud_preprocess_s']=c['runtime']['last_inference']['preprocess_s'];row['cloud_postprocess_s']=c['runtime']['last_inference']['postprocess_s'];all_rows.append(row)
  captured=[json.loads(f.read_text()) for f in sorted(root.glob('observation_*.json'))]
  poll_hz=(captured[-1]['state_revision']-captured[0]['state_revision'])/((captured[-1]['feedback_report']['channels'][0]['read_started_at_ns']-captured[0]['feedback_report']['channels'][0]['read_started_at_ns'])/1e9)
  requests.extend(m['observations']);runs[kind]=dict(feedback_polling_hz_from_cycle_counter=poll_hz,duration_s=m['duration_s'],observations=len(m['observations']),successful_requests=len(rows),failed_http_requests=sum('inference_error' in e for e in m['errors']),construction_rejections=len([e for e in m['errors'] if 'observation_error' in e]),rtt_s=stats([r['round_trip_s'] for r in rows]),gpu_forward_s=stats([r['cloud_model_s'] for r in rows]),json_encoding_s=stats([r['encoding_s'] for r in rows]),camera_rgb_receive_hz=m['camera']['streams']['rgb']['receive_hz'],camera_depth_receive_hz=m['camera']['streams']['depth']['receive_hz'],feedback_status_publication_hz=m['feedback_status_hz'],actual_inference_hz=len(rows)/m['duration_s'],camera=m['camera'])
 metrics=dict(runs=runs,total_successful_requests=len(all_rows),matched_cloud_ids=len(all_rows),action_shape=[50,6],action_finite=True,execution_enabled=False,physical_calibrated=False,cloud_gpu='RTX 4080',rtt_s=stats([r['round_trip_s'] for r in all_rows]),gpu_forward_s=stats([r['cloud_model_s'] for r in all_rows]))
 (a.output/'inference_metrics.json').write_text(json.dumps(metrics,indent=2));(a.output/'inference_requests.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in requests));(a.output/'inference_responses.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in all_rows))
 import matplotlib;matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axs=plt.subplots(1,2,figsize=(10,4));axs[0].hist([r['round_trip_s'] for r in all_rows],bins=15);axs[0].set(xlabel='HTTP RTT seconds',ylabel='Requests');axs[1].plot([r['cloud_model_s'] for r in all_rows]);axs[1].set(xlabel='Request index',ylabel='CUDA synchronized model forward seconds');fig.tight_layout();fig.savefig(a.output/'latency.png');plt.close(fig)
 fig,axs=plt.subplots(1,2,figsize=(10,4));axs[0].plot([1]*len(all_rows),'.');axs[0].set(xlabel='Successful request index',ylabel='Complete finite response');keys=list(runs);axs[1].bar([0,1],[runs[k]['camera_rgb_receive_hz'] for k in keys],width=.25,label='RGB subscriber Hz');axs[1].bar([.3,1.3],[runs[k]['feedback_status_publication_hz'] for k in keys],width=.25,label='Feedback status Hz (not 20Hz polling)');axs[1].bar([.6,1.6],[runs[k]['feedback_polling_hz_from_cycle_counter'] for k in keys],width=.25,label='Feedback polling Hz from real cycle counter');axs[1].set_xticks([.3,1.3],keys);axs[1].legend(fontsize=7);fig.tight_layout();fig.savefig(a.output/'success_rates.png');plt.close(fig)
 fig,ax=plt.subplots(figsize=(9,5))
 for i in range(6):ax.plot([r[i] for r in all_rows[0]['values']],label='native_'+str(i))
 ax.set(xlabel='Native output step index (not verified control time)',ylabel='model native unknown units',title='Genuine SmolVLA output; NEVER a JetRover command');ax.legend();fig.tight_layout();fig.savefig(a.output/'native_actions.png');plt.close(fig)
 print(json.dumps({k:metrics[k] for k in ['total_successful_requests','rtt_s','gpu_forward_s']}))
if __name__=='__main__':main()
