"""One explicitly authorized A->B->A using existing ArmInterface/trajectory executor.
No port open, torque, gripper, base, IK or VLA path. Dry-run is default.
"""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import signal
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True);p.add_argument('--plan-sha256',required=True)
    p.add_argument('--expected-source-sha',required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute-reviewed-motion',action='store_true')
    a=p.parse_args();raw=a.plan.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=a.plan_sha256:raise ValueError('PLAN_HASH_MISMATCH')
    plan=json.loads(raw)
    A={int(k):v for k,v in plan['A'].items()};B={int(k):v for k,v in plan['B'].items()}
    duration=float(plan['duration_s'])
    trajectory=dict(waypoints={'A':{'joint_pulse':A,'duration':duration},'B':{'joint_pulse':B,'duration':duration}},
        sequence=['WAIT 4','MOVE B','WAIT 4','MOVE A','WAIT 4'])
    from cable_robot_control.safety import ArmLimits
    from cable_robot_control.trajectory_executor import FixedTrajectoryExecutor
    limits=ArmLimits(sorted(A),10)
    if not a.execute_reviewed_motion:
        print('\n'.join(FixedTrajectoryExecutor(None,limits,trajectory,frozen_cartesian_servo_ids=[],include_gripper=False).dry_run()));return 0
    if 'PENDING' in plan.get('motion_authorization_reference','').upper():raise ValueError('MOTION_AUTHORIZATION_PENDING')
    # Reserve evidence before any ROS request or motion command.
    with a.output.open('x') as result:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rcl_interfaces.srv import SetParametersAtomically
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from cable_robot_control.arm_interface import ArmInterface
        from cable_robot_control.experiment_logger import ExperimentLogger
        from rclpy.signals import SignalHandlerOptions
        abort_requested=threading.Event()
        previous={sig:signal.signal(sig,lambda *_:abort_requested.set()) for sig in (signal.SIGINT,signal.SIGTERM)}
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('reviewed_feedback_motion',namespace='/idc_vla/real')
        prefix='/idc_vla/real/feedback';latest={};guard=threading.Lock();cancelled=threading.Event()
        statuses=[];outcome=dict(status='NOT_STARTED',plan=plan,plan_sha256=a.plan_sha256,
            expected_source_sha=a.expected_source_sha,execute_reviewed_motion=True)
        logger=ExperimentLogger(str(a.output.parent),'dynamic_motion_'+a.output.stem)
        def status(m):
            d=json.loads(m.data);d['_received_ns']=time.monotonic_ns()
            with guard:latest.clear();latest.update(d);statuses.append(d)
        sub=node.create_subscription(String,prefix+'/status',status,10)
        executor=SingleThreadedExecutor();executor.add_node(node)
        def spin():
            while not cancelled.is_set():executor.spin_once(timeout_sec=.02)
        worker=threading.Thread(target=spin,daemon=True);worker.start()
        def snapshot():
            with guard:return dict(latest)
        def check():
            if abort_requested.is_set():raise RuntimeError('OPERATOR_CANCELLED')
            d=snapshot()
            if d.get('gate',{}).get('error'):raise RuntimeError('DRIVER_FAULT:'+d['gate']['error'])
            if (d.get('source_sha')!=a.expected_source_sha or not d.get('feedback',{}).get('valid')
                    or time.monotonic_ns()-d.get('_received_ns',0)>500_000_000):
                raise RuntimeError('FEEDBACK_OR_SOURCE_UNAVAILABLE')
            if d.get('motion_error'):raise RuntimeError(d['motion_error'])
            return d
        def wait(seconds):
            logger.record(stage='phase_start',duration=seconds,monotonic_ns=time.monotonic_ns())
            end=time.monotonic()+seconds
            try:
                while time.monotonic()<end:check();time.sleep(min(.02,max(0,end-time.monotonic())))
            finally:logger.record(stage='phase_end',duration=seconds,monotonic_ns=time.monotonic_ns())
        def call(name,request=None,kind=Trigger):
            c=node.create_client(kind,name);assert c.wait_for_service(timeout_sec=5),name
            f=c.call_async(request or kind.Request());done=threading.Event()
            f.add_done_callback(lambda _:done.set())
            if not done.wait(5):raise RuntimeError('RPC_OUTCOME_UNKNOWN_NO_RETRY:'+name)
            response=f.result()
            if hasattr(response,'success') and not response.success:raise RuntimeError(response.message)
            return response
        arm=ArmInterface(node,limits,servo_controller_topic=prefix+'/reviewed_servo_command',
            bus_servo_set_state_topic=prefix+'/reviewed_stop_command')
        motion_attempted=False
        class GuardedArm:
            def move_joint(self,pulses,seconds):
                check()
                if arm.servo_position_pub.get_subscription_count()!=1:raise RuntimeError('EXPECTED_ONE_COMMAND_OWNER')
                nonlocal motion_attempted
                motion_attempted=True
                sent_after=time.monotonic_ns();arm.move_joint(pulses,seconds)
                deadline=time.monotonic()+1
                while time.monotonic()<deadline:
                    d=check()
                    for command in d.get('commands',[]):
                        target={int(k):v for k,v in command.get('target_pulses',{}).items()}
                        if target==pulses and command.get('send_started_monotonic_ns',0)>=sent_after and command.get('status')=='SENT':return
                    time.sleep(.01)
                raise RuntimeError('COMMAND_SEND_NOT_CONFIRMED_NO_RETRY')
        try:
            end=time.monotonic()+8
            while time.monotonic()<end:
                try:check();break
                except RuntimeError as error:
                    if str(error).startswith('DRIVER_FAULT:'):raise
                    time.sleep(.05)
            else:raise RuntimeError('DRIVER_NOT_READY')
            outcome['initial_feedback']=check()['feedback']
            request=SetParametersAtomically.Request()
            request.parameters=[Parameter('motion_plan_path',value=str(a.plan.resolve())).to_parameter_msg(),
                Parameter('motion_plan_sha256',value=a.plan_sha256).to_parameter_msg()]
            response=call('/ros_robot_controller/set_parameters_atomically',request,SetParametersAtomically)
            if not response.result.successful:raise RuntimeError(response.result.reason)
            call(prefix+'/arm_reviewed_motion')
            outcome['motion_capability_armed']=True
            outcome['start_recording']=json.loads(call(prefix+'/start_recording').message)
            outcome['sequence_started_monotonic_ns']=time.monotonic_ns()
            logger.record(stage='dynamic_start',monotonic_ns=outcome['sequence_started_monotonic_ns'],plan_sha256=a.plan_sha256)
            sequence=FixedTrajectoryExecutor(GuardedArm(),limits,trajectory,logger=logger,wait=wait,
                frozen_cartesian_servo_ids=[],include_gripper=False)
            sequence.execute();check()
            outcome['sequence_finished_monotonic_ns']=time.monotonic_ns()
            outcome['stop_recording']=json.loads(call(prefix+'/stop_recording').message)
            wait(.25)
            outcome.update(status='RECORDED',final_status=snapshot(),driver_statuses=statuses)
        except BaseException as e:
            outcome.update(status='FAILED',error=repr(e))
            try:
                d=snapshot()
                if motion_attempted and not any(c.get('kind')=='stop' for c in d.get('commands',[])):
                    arm.stop_arm();time.sleep(.2)
            except Exception as stop_error:outcome['stop_error']=str(stop_error)
            if 'start_recording' in outcome and 'stop_recording' not in outcome:
                try:outcome['stop_recording']=json.loads(call(prefix+'/stop_recording').message)
                except Exception as stop_error:outcome['recording_stop_error']=str(stop_error)
            outcome['final_status']=snapshot();outcome['driver_statuses']=statuses
        finally:
            outcome['motion_command_attempted']=motion_attempted
            outcome.setdefault('motion_capability_armed',False)
            try:logger.record(stage='dynamic_end',success=outcome['status']=='RECORDED',monotonic_ns=time.monotonic_ns())
            except Exception as error:outcome['logger_error']=str(error)
            result.write(json.dumps(outcome,indent=2)+'\n')
            cancelled.set();worker.join(timeout=1);executor.shutdown(timeout_sec=1)
            node.destroy_node();rclpy.shutdown()
            for sig,handler in previous.items():signal.signal(sig,handler)
        print(json.dumps({k:v for k,v in outcome.items() if k not in ('initial_feedback','final_status','driver_statuses')},indent=2))
        return 0 if outcome['status']=='RECORDED' else 2

if __name__=='__main__':raise SystemExit(main())
