"""Explicit experimental ROS runtime using the audited single SDK owner.

Maintenance remains the default transport mode. Motion needs a separate reviewed,
hash-bound A/B capability and never accepts controller, torque or VLA channels.
"""
import hashlib
import json
from pathlib import Path
import queue
import signal
import threading
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import SetBusServoState
from ros_robot_controller import ros_robot_controller_sdk as sdk
from ros_robot_controller.ros_robot_controller_node import RosRobotController
from patches.jetrover_readback.continuous_feedback import ExperimentalReadbackGate, ContinuousFeedbackCollector
from .feedback_motion import MotionPolicy, UnifiedFrameWriter, MotionFeedbackMonitor
from .feedback_recording import FeedbackRecorder
from cable_robot_vla.robot_state_provider import RobotStateProvider

PREFIX='/idc_vla/real/feedback'

class UnifiedFeedbackNode(RosRobotController):
    def __init__(self, name='ros_robot_controller'):
        sdk.ReadbackGate=ExperimentalReadbackGate
        sdk.MaintenanceFrameWriter=UnifiedFrameWriter
        super().__init__(name)
        if not self.readback_maintenance or self.readback_diagnostic_plan or self.readback_probe_reference:
            raise RuntimeError('EXCLUSIVE_EXPERIMENT_MODE_REQUIRED')
        def param(k,v):return self.declare_parameter(k,v).value
        self.source_sha=param('feedback_source_sha','')
        self.root=Path(param('feedback_output_directory','feedback_recordings'))
        self.root.mkdir(parents=True,exist_ok=True)
        self.recorder=None;self.last_recording=None;self.commands=[];self.motion_error=None
        self.motion_monitor=MotionFeedbackMonitor()
        self.declare_parameter('motion_plan_path','')
        self.declare_parameter('motion_plan_sha256','')
        cfg=param('feedback_configuration_path','')
        self.provider=RobotStateProvider();self.provider.configure_raw_conversion(cfg)
        self.archive_provider=RobotStateProvider();self.archive_provider.configure_raw_conversion(cfg)
        self.board._bounded_bus_feedback.arm_experiment('R1 experimental continuous '+self.source_sha)
        self.raw_pub=self.create_publisher(String,'/idc_vla/real/raw_feedback',10)
        self.joint_pub=self.create_publisher(JointState,'/idc_vla/real/feedback_joint_states',10)
        self.status_pub=self.create_publisher(String,PREFIX+'/status',10)
        self.collector=ContinuousFeedbackCollector(self.board,hz=param('feedback_hz',20.))
        self.collector.on_record=self._archive
        self.collector.on_fault=self._fault_stop
        self.create_service(Trigger,PREFIX+'/start_recording',self._record_start)
        self.create_service(Trigger,PREFIX+'/stop_recording',self._record_stop)
        self.create_service(Trigger,PREFIX+'/arm_reviewed_motion',self._arm_motion)
        self.create_service(Trigger,PREFIX+'/stop_motion',self._stop_service)
        self.create_subscription(ServosPosition,PREFIX+'/reviewed_servo_command',self._motion,10)
        self.create_subscription(SetBusServoState,PREFIX+'/reviewed_stop_command',self._stop_message,10)
        self.timer=self.create_timer(.005,self._publish_feedback)
        self.last_status=0.
        self.collector.start()

    def _archive(self,report):
        if report['complete']:
            try:self.motion_monitor.check(report,self.board._maintenance_writer)
            except Exception as error:
                self.board._bounded_bus_feedback.fail(str(error))
                report['motion_monitor_error']=str(error)
        if report['complete']:
            report['conversion_candidate']=self.archive_provider.offer_diagnostic_report(report)
        recorder=self.recorder
        if recorder:recorder.submit(report)

    def _record_start(self,request,response):
        try:
            if self.recorder and self.recorder.thread.is_alive():raise RuntimeError('RECORDING_ALREADY_OPEN')
            if not self.collector.snapshot()['valid']:raise RuntimeError('FEEDBACK_NOT_VALID')
            self.recorder=FeedbackRecorder(self.root/('run_'+uuid.uuid4().hex))
            response.success=True;response.message=json.dumps(self.recorder.status())
        except Exception as e:response.success=False;response.message=str(e)
        return response

    def _record_stop(self,request,response):
        try:
            if not self.recorder:raise RuntimeError('NO_RECORDING')
            self.last_recording=self.recorder.stop()
            response.success=self.last_recording['complete']
            response.message=json.dumps(self.last_recording)
        except Exception as e:response.success=False;response.message=str(e)
        return response

    def _publish_feedback(self):
        try:report=self.collector.output.get_nowait()
        except queue.Empty:report=None
        if report is not None:
            report.update(runtime_source_sha=self.source_sha,ros_publication_time_ns=self.get_clock().now().nanoseconds)
            self.raw_pub.publish(String(data=json.dumps(report)))
            if report['complete'] and self.collector.snapshot()['valid']:
                converted=self.provider.offer_diagnostic_report(report)
                if 'positions' in converted:
                    msg=JointState();msg.name=converted['joint_names'];msg.position=converted['positions']
                    self.joint_pub.publish(msg)  # No invented device acquisition stamp.
        now=time.monotonic()
        writer=self.board._maintenance_writer
        if writer.policy and time.monotonic_ns()>getattr(self,'motion_until_ns',2**63):
            writer.motion_active=False
            if writer.policy.index>=2:writer.policy.enabled=False
        if now-self.last_status>=.1:
            self.last_status=now
            status=dict(source_sha=self.source_sha,collector=self.collector.stats.copy(),
                gate=self.board._bounded_bus_feedback.status(),feedback=self.collector.snapshot(),
                recording=self.recorder.status() if self.recorder else self.last_recording,
                commands=list(self.commands),motion_error=self.motion_error,
                source_type='unknown',physical_calibrated=False,canonical_observation_eligible=False)
            self.status_pub.publish(String(data=json.dumps(status)))

    def _arm_motion(self,request,response):
        try:
            if getattr(self,'motion_ever_armed',False):raise RuntimeError('MOTION_CAPABILITY_ALREADY_USED')
            snap=self.collector.snapshot()
            if not snap['valid']:raise RuntimeError('FRESH_FEEDBACK_REQUIRED')
            path=Path(self.get_parameter('motion_plan_path').value)
            digest=self.get_parameter('motion_plan_sha256').value
            raw=path.read_bytes()
            if len(raw)>65536 or len(digest)!=64 or hashlib.sha256(raw).hexdigest()!=digest:
                raise RuntimeError('MOTION_PLAN_HASH_MISMATCH')
            current={r['servo_id']:r['raw_pulse'] for r in snap['report']['channels']}
            self.board._maintenance_writer.policy=MotionPolicy(json.loads(raw),current)
            self.motion_ever_armed=True
            response.success=True;response.message='Reviewed B then A only; no command sent'
        except Exception as e:response.success=False;response.message=str(e)
        return response

    def _motion(self,msg):
        try:
            policy=self.board._maintenance_writer.policy
            if not policy or not self.collector.snapshot()['valid']:raise RuntimeError('MOTION_NOT_ARMED_OR_FEEDBACK_INVALID')
            if msg.position_unit!='pulse' or not msg.position:raise RuntimeError('PULSE_COMMAND_REQUIRED')
            pulses={}
            for p in msg.position:
                if p.id in pulses or not float(p.position).is_integer():raise RuntimeError('INVALID_TARGET')
                pulses[p.id]=int(p.position)
            policy.validate(pulses,float(msg.duration))
            if policy.index==0:
                actual={r['servo_id']:r['raw_pulse'] for r in self.collector.snapshot()['report']['channels']}
                if any(abs(actual[i]-policy.A[i])>3 for i in policy.A):raise RuntimeError('START_POSE_CHANGED')
            event=dict(status='QUEUED',target_pulses=pulses,duration_s=float(msg.duration),
                       received_monotonic_ns=time.monotonic_ns(),kind='position')
            def execute():
                if policy.stopped:
                    event['status']='CANCELLED';return
                if not self.collector.snapshot()['valid']:raise RuntimeError('STALE_BEFORE_MOTION')
                event['send_started_monotonic_ns']=time.monotonic_ns()
                self.board.bus_servo_set_position(msg.duration,sorted(pulses.items()))
                event.update(status='SENT',send_finished_monotonic_ns=time.monotonic_ns(),
                    tx_frame_hex=self.board._maintenance_writer.last_motion_frame)
                self.motion_until_ns=event['send_finished_monotonic_ns']+int(msg.duration*1e9)
            self.collector.submit(execute);self.commands.append(event)
        except Exception as e:self.motion_error=str(e)

    def _send_stop(self):
        writer=self.board._maintenance_writer
        if not writer.policy or not writer.policy.enabled:return
        self.board.bus_servo_stop(sorted(writer.policy.A))
        self.commands.append(dict(kind='stop',status='SENT',monotonic_ns=time.monotonic_ns(),
                                  tx_frame_hex=writer.last_motion_frame))

    def _stop_service(self,request,response):
        try:
            if not self.board._maintenance_writer.policy:raise RuntimeError('NO_MOTION_CAPABILITY')
            self.collector.submit(self._send_stop,stop=True)
            response.success=True;response.message='Stop queued at next transaction boundary'
        except Exception as e:response.success=False;response.message=str(e)
        return response

    def _stop_message(self,msg):
        try:
            policy=self.board._maintenance_writer.policy
            if not policy:raise RuntimeError('NO_MOTION_CAPABILITY')
            ids=[]
            for row in msg.state:
                if list(row.present_id[:1])!=[1] or list(row.stop)!=[1,0]:raise RuntimeError('STOP_ONLY')
                for field in row.get_fields_and_field_types():
                    if field not in ('present_id','stop') and getattr(row,field):raise RuntimeError('TORQUE_OR_OTHER_FIELD_FORBIDDEN')
                ids.append(row.present_id[1])
            if ids!=sorted(policy.A):raise RuntimeError('STOP_IDS_MISMATCH')
            self.collector.submit(self._send_stop,stop=True)
        except Exception as e:self.motion_error=str(e)

    def _fault_stop(self,report):
        writer=self.board._maintenance_writer
        if writer.motion_active:
            try:
                if not self.board._bounded_bus_feedback.lock.acquire(timeout=.05):raise RuntimeError('STOP_LOCK_BUSY')
                try:self._send_stop()
                finally:self.board._bounded_bus_feedback.lock.release()
            except Exception as e:self.motion_error='STOP_NOT_CONFIRMED_OPERATOR_REQUIRED:'+str(e)

    def close_feedback(self):
        self.collector.stop()
        if self.recorder and self.recorder.thread.is_alive():self.last_recording=self.recorder.stop()
        self.running=False
        self._publisher_thread.join(timeout=.5)
        return self.board.close_maintenance_transport()


def main():
    stopped=threading.Event();node=None;executor=None
    def stop(*_):stopped.set()
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        node=UnifiedFeedbackNode()
        executor=SingleThreadedExecutor(context=node.context);executor.add_node(node)
        while not stopped.is_set() and rclpy.ok():executor.spin_once(timeout_sec=.01)
    finally:
        if executor:executor.shutdown(timeout_sec=.5)
        if node:
            print('feedback close',node.close_feedback(),flush=True)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
