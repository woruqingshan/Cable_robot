"""Explicit recorded REAL research inputs. Never claims current/live acquisition."""
import argparse,json,time
from pathlib import Path
from cable_robot_vla.policy_client import PolicyClient
from vla_protocol.research import ResearchObservation
from vla_protocol.profiles import RuntimeLimits

def main():
 p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True);p.add_argument('--server-url',default='http://127.0.0.1:18000');a=p.parse_args()
 out=Path(a.output);out.mkdir(parents=True,exist_ok=False);client=PolicyClient(a.server_url,research_inspection=True,limits=RuntimeLimits(request_timeout_s=30.,max_horizon=64));rows=[]
 try:
  for f in sorted(Path(a.input).glob('observation_*.json')):
   data=json.loads(f.read_text());data.update(capture_mode='recorded_replay',replay_reference=str(f.resolve()));o=ResearchObservation.model_validate(data)
   (out/f.name).write_text(o.model_dump_json());started=time.time();client.submit(o)
   while client.busy:
    result=client.poll()
    if result:
     row=dict(request_id=o.request_id,recorded_input=str(f),capture_mode='recorded_replay',sent_wall_time=started,received_wall_time=time.time(),encoding_s=result.encoding_s,round_trip_s=result.round_trip_s,action=result.candidate.model_dump());rows.append(row)
     with (out/'inference_responses.jsonl').open('a') as s:s.write(json.dumps(row)+'\n')
    time.sleep(.01)
 finally:
  client.close();(out/'metrics.json').write_text(json.dumps({'capture_mode':'recorded_replay','responses':len(rows)},indent=2))
 print('recorded real input inference count',len(rows))
if __name__=='__main__':main()
