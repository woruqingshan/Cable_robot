"""Record a finite static window; the collector continues afterward. No motion API."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

PREFIX='/idc_vla/real/feedback'

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds',type=float,default=30.)
    p.add_argument('--expected-source-sha',required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not 0<a.seconds<=60 or len(a.expected_source_sha)!=40:raise ValueError('INVALID_TRIAL')
    with a.output.open('x') as result:
        rclpy.init();n=rclpy.create_node('record_feedback_trial',namespace='/idc_vla/real')
        latest={};statuses=[]
        def status(m):
            latest.clear();latest.update(json.loads(m.data));latest['_received_ns']=time.monotonic_ns()
            statuses.append(dict(latest))
        sub=n.create_subscription(String,PREFIX+'/status',status,10)
        def call(name):
            c=n.create_client(Trigger,PREFIX+'/'+name)
            if not c.wait_for_service(timeout_sec=5):raise RuntimeError('SERVICE_UNAVAILABLE:'+name)
            f=c.call_async(Trigger.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
            if not f.done():raise RuntimeError('RPC_OUTCOME_UNKNOWN_NO_RETRY:'+name)
            response=f.result()
            if not response.success:raise RuntimeError(response.message)
            return json.loads(response.message)
        outcome=dict(requested_seconds=a.seconds,expected_source_sha=a.expected_source_sha,status='NOT_STARTED')
        try:
            deadline=time.monotonic()+8
            while time.monotonic()<deadline:
                rclpy.spin_once(n,timeout_sec=.02)
                if latest.get('feedback',{}).get('valid') and latest.get('source_sha')==a.expected_source_sha:break
            else:raise RuntimeError('FRESH_EXPECTED_DRIVER_NOT_DISCOVERED')
            outcome['start_recording']=call('start_recording')
            end=outcome['start_recording']['started_monotonic_ns']+int(a.seconds*1e9)
            while time.monotonic_ns()<end:
                rclpy.spin_once(n,timeout_sec=.01)
                if not latest.get('feedback',{}).get('valid'):raise RuntimeError('FEEDBACK_INVALID_STOP_TRIAL')
                if time.monotonic_ns()-latest['_received_ns']>500_000_000:raise RuntimeError('STATUS_STALE')
            outcome['stop_recording']=call('stop_recording')
            before=latest['collector']['cycles'];deadline=time.monotonic()+.35
            while time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
            outcome.update(status='RECORDED',collector_continued=latest['collector']['cycles']>before,
                final_status=latest,driver_statuses=statuses)
            if not outcome['collector_continued']:raise RuntimeError('COLLECTOR_DID_NOT_CONTINUE')
        except Exception as e:
            outcome.update(status='FAILED',error=str(e))
            if 'start_recording' in outcome and 'stop_recording' not in outcome:
                try:outcome['stop_recording']=call('stop_recording')
                except Exception as stop_error:outcome['recording_stop_error']=str(stop_error)
        finally:
            result.write(json.dumps(outcome,indent=2)+'\n')
            n.destroy_node();rclpy.shutdown()
        print(json.dumps({k:v for k,v in outcome.items() if k not in ('driver_statuses','final_status')},indent=2))
        return 0 if outcome['status']=='RECORDED' else 2

if __name__=='__main__':raise SystemExit(main())
