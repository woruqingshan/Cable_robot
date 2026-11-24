"""D405 + actual diagnostic feedback to optional PolicyClient; no actuator imports."""
import argparse,base64,json,time,uuid
from pathlib import Path
from hashlib import sha256
import rclpy
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from cable_robot_vla.real_camera import CameraRecordAdapter,stamp_ns
from cable_robot_vla.real_recording import RealRecorder
from cable_robot_vla.robot_state_provider import RobotStateProvider
from cable_robot_vla.policy_client import PolicyClient
from vla_protocol.research import ResearchObservation
from vla_protocol import codec
from vla_protocol.profiles import RuntimeLimits

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--seconds',type=float,default=30);p.add_argument('--server-url');p.add_argument('--instruction',default='Inspect the RJ45 cable and network port.');a=p.parse_args()
 r=RealRecorder(a.output,{'mode':'research_inspection_only','execution':False});manifest=json.loads((r.path/'manifest.json').read_text());manifest['profile']='real_research_inspection_v1';(r.path/'manifest.json').write_text(json.dumps(manifest,indent=2))
 cam=CameraRecordAdapter(r,sample_hz=2);state=RobotStateProvider();state.configure_raw_conversion('/home/ubuntu/ros2_ws/src/driver/servo_controller/config/servo_controller.yaml')
 latest={};session=uuid.uuid4().hex;seq=0;errors=[];results=[];observations=[];feedback_times=[]
 client=PolicyClient(a.server_url,research_inspection=True,limits=RuntimeLimits(request_timeout_s=30.,max_horizon=64)) if a.server_url else None
 rclpy.init();node=rclpy.create_node('stage1_research_inspection')
 def status(m):
  d=json.loads(m.data);latest['feedback']=d;latest['feedback_received']=time.monotonic();feedback_times.append(time.monotonic())
 def rgb(m):
  now=time.monotonic();saved=cam.rgb(m)
  if saved:latest['rgb']=(m,dict(cam.provenance),now)
 def depth(m):cam.depth(m)
 def protect(fn):
  def wrapped(m):
   try:fn(m)
   except Exception as e:errors.append({'at':time.time(),'sensor_error':str(e)})
  return wrapped
 node.create_subscription(String,'/idc_vla/real/feedback/status',protect(status),10)
 node.create_subscription(Image,'/d405/wrist/color/image_rect_raw',protect(rgb),qos_profile_sensor_data)
 node.create_subscription(Image,'/d405/wrist/depth/image_rect_raw',protect(depth),qos_profile_sensor_data)
 for kind in ['color','depth']:
  node.create_subscription(CameraInfo,f'/d405/wrist/{kind}/camera_info',protect(lambda m,k=kind:cam.camera_info(k,m)),qos_profile_sensor_data)
 start=time.monotonic();next_sample=start+2
 try:
  while time.monotonic()-start<a.seconds or (client and client.busy):
   rclpy.spin_once(node,timeout_sec=.005);now=time.monotonic()
   if client and client.busy:
    try:
     result=client.poll()
     if result:
      data=result.candidate.model_dump();data.update(round_trip_s=result.round_trip_s,encoding_s=result.encoding_s);results.append(data)
      with (r.path/'inference_responses.jsonl').open('a') as f:f.write(json.dumps(data)+'\n')
    except Exception as e:errors.append({'at':time.time(),'inference_error':str(e)})
   if now>=next_sample and now-start<a.seconds and not (client and client.busy):
    next_sample=now+.5
    try:
     d=latest['feedback']
     if not d.get('feedback',{}).get('valid'):raise ValueError('FEEDBACK_INVALID_NO_QUERY_OR_INFERENCE')
     report=d['feedback']['report']
     if not d['feedback']['valid'] or d.get('motion_permission'):raise ValueError('FEEDBACK_INVALID_OR_MOTION_ENABLED')
     conv=state.offer_diagnostic_report(report)
     if 'positions' not in conv:raise ValueError('INVALID_NOMINAL_CONVERSION')
     m,im,received=latest['rgb'];raw=(r.path/im['png_path']).read_bytes();channels=report['channels']
     joint_stamp={'nanoseconds':max(x['response_received_at_ns'] for x in channels[:5]),'clock_domain':'host_monotonic_group_end_not_sample'}
     grip_stamp={'nanoseconds':channels[5]['response_received_at_ns'],'clock_domain':'host_monotonic_receive_not_sample'}
     values=dict(schema_version=1,request_id=uuid.uuid4().hex,session_id=session,observation_seq=seq,task_revision=0,robot_id='jetrover_real_inspection',robot_profile_id='jetrover_joint_abs_rad_v1',state_revision=report['sequence'],scene_id='unregistered_static_scene',scene_sha256='0'*64,model_id='smolvla_base_research_inspection_v1',instruction=a.instruction,joints=dict(names=conv['joint_names'][:5],positions=conv['positions'][:5],unit='rad',stamp=joint_stamp),gripper=dict(name='r_joint',position=conv['positions'][5],unit='rad',stamp=grip_stamp,simulated=False,calibrated=False),images=[dict(camera_id='real_rgb',frame_id=m.header.frame_id,stamp=dict(nanoseconds=stamp_ns(m),clock_domain='driver_ros_header_unverified_acquisition'),width=m.width,height=m.height,codec='png',pixel_encoding='rgb8',sha256=sha256(raw).hexdigest(),data_base64=base64.b64encode(raw).decode())],feedback_report=report,conversion_evidence=conv,timing_evidence=dict(feedback_receive_age_s=now-latest['feedback_received'],image_receive_age_s=now-received,receive_pair_difference_s=abs(latest['feedback_received']-received),physical_synchronization_verified=False))
     o=ResearchObservation.model_validate(values);body=codec.dumps(o);(r.path/f'observation_{seq:06d}.json').write_bytes(body);observations.append({'request_id':o.request_id,'bytes':len(body),'seq':seq,'timing':values['timing_evidence']});seq+=1
     if client:client.submit(o)
    except Exception as e:errors.append({'at':time.time(),'observation_error':str(e)})
 finally:
  if client:client.close()
  node.destroy_node();rclpy.shutdown()
  metrics=dict(duration_s=time.monotonic()-start,observations=observations,responses=len(results),camera=cam.status(),feedback_status_hz=(len(feedback_times)-1)/(feedback_times[-1]-feedback_times[0]) if len(feedback_times)>1 else None,errors=errors,final_feedback=latest.get('feedback'))
  (r.path/'metrics.json').write_text(json.dumps(metrics,indent=2));r.observations.close();r.events.close();print(str(r.path),json.dumps({'observations':len(observations),'responses':len(results),'errors':len(errors)}))
if __name__=='__main__':main()
