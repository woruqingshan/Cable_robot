"""Real ROS executor and original packet parser, simulated transport only."""
import os
from pathlib import Path
import subprocess
import sys
import pytest
from patches.jetrover_readback.prepare_feedback_stage import stage


def test_frozen_driver_one_mock_port_record_stop_keeps_feedback_running(tmp_path):
    pytest.importorskip('rclpy')
    vendor=Path('/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller')
    if not vendor.exists():pytest.skip('audited vendor reference unavailable')
    output=tmp_path/'package';stage(output,vendor,'a'*40,tmp_path/'recordings')
    script=tmp_path/'exercise.py'
    script.write_text(r'''
import sys,json,time,queue,struct
from pathlib import Path
root=Path(sys.argv[1]);sys.path.insert(0,str(root))
from patches.jetrover_readback import guarded_readback as guard
assert Path(guard.__file__).is_relative_to(root)
sys.modules['ros_robot_controller.guarded_readback']=guard
guard.claim_linux_exclusive=lambda port:None  # MOCK FD, never a device ioctl.
import serial
ports=[]
class Port:
 def __init__(self,*args,**kwargs):
  assert not ports
  ports.append(self);self.write_timeout=kwargs['write_timeout'];self.is_open=False;self.fd=None
  self.q=queue.Queue();self.requests=0;self.closed=0;self.moves=0;self.stops=0
 def setPort(self,p):self.port=p
 def open(self):self.fd=123;self.is_open=True
 def close(self):self.closed+=1;self.fd=None;self.is_open=False
 def fileno(self):return self.fd
 def cancel_read(self):self.q.put(b'')
 def read(self,*args):
  try:return self.q.get(timeout=.02)
  except queue.Empty:return b''
 def write(self,b):
  from ros_robot_controller.ros_robot_controller_sdk import checksum_crc8
  b=bytes(b);assert b[2]==5, b.hex()
  if b[4]==1:
   assert set(b[8:-1:3])<={1,2,3,4};self.moves+=1;return len(b)
  if b[4]==3:
   assert list(b[6:-1])==[1,2,3,4];self.stops+=1;return len(b)
  assert b[4]==5 and len(b)==7, b.hex()
  i=b[5];self.requests+=1
  data=bytes([0xaa,0x55,5,5])+struct.pack('<BBbh',i,5,0,500+i)
  data+=bytes([checksum_crc8(data[2:])])
  for v in data:self.q.put(bytes([v]))
  return len(b)
serial.Serial=Port
sys.argv=[__file__,'--ros-args','--params-file',str(root/'feedback.json')]
from cable_robot_control.unified_feedback_node import UnifiedFeedbackNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
node=UnifiedFeedbackNode();executor=SingleThreadedExecutor();executor.add_node(node)
monitor=rclpy.create_node('feedback_test_monitor');executor.add_node(monitor)
received=[]
subscription=monitor.create_subscription(JointState,'/idc_vla/real/feedback_joint_states',received.append,10)
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:executor.spin_once(timeout_sec=.01)
try:
 spin(.5);assert node.collector.snapshot()['valid']
 response=node._record_start(Trigger.Request(),Trigger.Response());assert response.success,response.message
 spin(.4);response=node._record_stop(Trigger.Request(),Trigger.Response());assert response.success,response.message
 manifest=json.loads(response.message);assert manifest['written_batches']>0
 before=ports[0].requests;spin(.2);assert ports[0].requests>before
 assert received and all(m.header.stamp.sec==m.header.stamp.nanosec==0 for m in received)
 assert all(len(m.name)==6 for m in received)
 assert node.board._maintenance_writer.policy is None
 import hashlib
 from rclpy.parameter import Parameter
 from servo_controller_msgs.msg import ServosPosition,ServoPosition
 from ros_robot_controller_msgs.msg import SetBusServoState,BusServoState
 plan=dict(schema_version=1,A={'1':501,'2':502,'3':503,'4':504},
  B={'1':451,'2':552,'3':553,'4':524},duration_s=4.,
  reviewed_path_reference='MOCK_ONLY',motion_authorization_reference='MOCK_ONLY')
 path=root/'mock_motion_plan.json';path.write_text(json.dumps(plan));digest=hashlib.sha256(path.read_bytes()).hexdigest()
 node.set_parameters([Parameter('motion_plan_path',value=str(path)),Parameter('motion_plan_sha256',value=digest)])
 response=node._arm_motion(Trigger.Request(),Trigger.Response());assert response.success,response.message
 m=ServosPosition();m.duration=4.;m.position_unit='pulse'
 for i,v in plan['B'].items():
  p=ServoPosition();p.id=int(i);p.position=float(v);m.position.append(p)
 before=ports[0].requests;node._motion(m);spin(.15)
 assert ports[0].moves==1 and ports[0].requests>before
 stop=SetBusServoState();stop.duration=.02
 for i in (1,2,3,4):
  row=BusServoState();row.present_id=[1,i];row.stop=[1,0];stop.state.append(row)
 node._stop_message(stop);spin(.1);assert ports[0].stops==1
 assert node.board._maintenance_writer.policy.stopped
 assert node.provider.read_state_snapshot()['raw_values']==[None]*6
 assert node.board._bounded_bus_feedback.status()['trusted_continuous_feedback'] is False
finally:
 executor.shutdown(timeout_sec=1);report=node.close_feedback();assert report['port_closed']
 monitor.destroy_node();node.destroy_node();rclpy.shutdown()
assert len(ports)==1 and ports[0].closed==1
print('MOCK_TRANSPORT_ROS_PASS')
''')
    env=dict(os.environ,ROS_DOMAIN_ID='117',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',PYTHONDONTWRITEBYTECODE='1')
    result=subprocess.run([sys.executable,str(script),str(output)],env=env,text=True,capture_output=True,timeout=20)
    assert result.returncode==0,result.stdout+result.stderr
    assert 'MOCK_TRANSPORT_ROS_PASS' in result.stdout
