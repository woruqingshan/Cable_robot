"""Owned real ROS processes and optional real Qt widgets. Not a planning experiment."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def run(mode, output):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from rclpy.time import Time
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from visualization_msgs.msg import MarkerArray
    from std_msgs.msg import String
    from tf2_ros import Buffer,TransformListener
    from ament_index_python.packages import get_package_prefix
    from workspace_analysis.robot.model import load_model
    from workspace_analysis.robot.fk import ForwardKinematics
    from scipy.spatial.transform import Rotation
    from jetrover_idc_visualization.node import ReplayNode
    from jetrover_idc_visualization.data import joint_values
    from replay_fixtures import catalog_fixture
    output=Path(output);output.mkdir(exist_ok=False)
    model=load_model(ROOT/'config/workspace.yaml') if mode!='panel' else None
    fixture=None if mode=='artifact' else catalog_fixture(model)
    if model is None:model=fixture.model
    context=rclpy.context.Context();rclpy.init(context=context)
    observer=Node('replay_test_observer',namespace='/idc_replay_test',context=context,
        cli_args=['--ros-args','-r','/tf:=/idc_replay/tf','-r','/tf_static:=/idc_replay/tf_static'])
    latest={'status':None,'joint':None,'scene':None,'ports':None};history=[]
    # A test observer retains every state; the interactive panel deliberately
    # keeps only the latest status to remain responsive.
    qos=QoSProfile(depth=1000,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def status(msg):
        value=json.loads(msg.data);latest['status']=value;history.append(value)
    subscriptions=[observer.create_subscription(String,'/idc_replay/status',status,qos),
        observer.create_subscription(JointState,'/idc_replay/joint_states',lambda m:latest.update(joint=m),100),
        observer.create_subscription(MarkerArray,'/idc_replay/scene_markers',lambda m:latest.update(scene=m),qos),
        observer.create_subscription(MarkerArray,'/idc_replay/port_markers',lambda m:latest.update(ports=m),qos)]
    pub=observer.create_publisher(String,'/idc_replay/command',10)
    buffer=Buffer(node=observer);listener=TransformListener(buffer,observer)
    observer_executor=SingleThreadedExecutor(context=context);observer_executor.add_node(observer)
    replay_executor=SingleThreadedExecutor(context=context)
    replay=None;panel=None;app=None;process=None;log=None;cleanup={};report={}
    if mode=='artifact':
        installed=Path(os.environ['IDC_REPLAY_INSTALL'])
        command=['ros2','launch','jetrover_idc_visualization','replay.launch.py',
            'workspace_root:='+str(ROOT),'benchmark_dir:='+str(Path(os.environ['WORKSPACE_ARTIFACT_ROOT'])/'task_benchmark/t5_1_s1_20261104_03'),
            'gui:=false','domain_id:='+os.environ['ROS_DOMAIN_ID'],'replay_hz:=20.0',
            'expected_manifest_sha256:=3e3a0d27123c197c8ef9a41c8e1a111e2181f0998740e0fa0ca95903d58cfbcd']
    else:
        binary=Path(get_package_prefix('robot_state_publisher'))/'lib/robot_state_publisher/robot_state_publisher'
        parameters=output/'rsp.yaml';parameters.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':model.urdf_xml,'publish_frequency':100.0}}}))
        command=[str(binary),'--ros-args','--params-file',str(parameters),'-r','__ns:=/idc_replay',
            '-r','/tf:=/idc_replay/tf','-r','/tf_static:=/idc_replay/tf_static',
            '-r','joint_states:=/idc_replay/joint_states']
        replay=ReplayNode(catalog_loader=lambda:fixture,context=context)
        replay_executor.add_node(replay)
        if mode=='panel':
            from PyQt5.QtWidgets import QApplication
            from PyQt5.QtCore import QTimer
            from jetrover_idc_visualization.panel import ReplayPanel
            observer_executor.remove_node(observer);observer_executor.shutdown()
            app=QApplication([sys.argv[0]]);panel=ReplayPanel(observer);panel.show()
            observer_executor=panel.executor
    log=(output/'process.log').open('wb')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def pump():
        if replay is not None:replay_executor.spin_once(timeout_sec=0.)
        if app is not None:app.processEvents()
        for _ in range(8): observer_executor.spin_once(timeout_sec=0.)
        time.sleep(.002)
    def wait(condition,timeout=12):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            pump()
            if condition():return
            if process.poll() is not None:raise RuntimeError('Owned ROS process exited early: '+str(process.returncode))
        raise TimeoutError('ROS replay condition timeout; status='+str(latest['status']))
    def send(action,**fields):
        value=latest['status']; request=uuid.uuid4().hex
        message=String();message.data=json.dumps({'version':1,'session_id':value['session_id'],'request_id':request,
            'selection_revision':value['selection_revision'],'action':action,**fields});pub.publish(message)
        wait(lambda:latest['status'].get('last_request_id')==request)
        return latest['status']
    tf_errors=[];cases=[]
    def check_tf():
        q=latest['status']['q'];expected=ForwardKinematics(model)(q)
        def matches():
            try:
                transform=buffer.lookup_transform('base_link',model.ee_frame,Time()).transform
                matrix=np.eye(4);matrix[:3,3]=[transform.translation.x,transform.translation.y,transform.translation.z]
                rot=transform.rotation;matrix[:3,:3]=Rotation.from_quat([rot.x,rot.y,rot.z,rot.w]).as_matrix()
                error=float(np.max(np.abs(matrix-expected)))
                if error<=1e-9:tf_errors.append(error);return True
            except Exception:pass
            return False
        wait(matches)
        names,values=joint_values(model,q,.3)
        wait(lambda:latest['joint'] is not None and latest['joint'].name==list(names) and list(latest['joint'].position)==list(values))
    try:
        wait(lambda:latest['status'] is not None and latest['status']['stage']=='READY',timeout=65)
        wait(lambda:latest['scene'] is not None and latest['ports'] is not None and pub.get_subscription_count()>0)
        assert sum(m.action==m.ADD for m in latest['scene'].markers)==38
        assert sum(m.type==m.TEXT_VIEW_FACING for m in latest['ports'].markers)==9
        if mode=='panel':
            def select_widget(widget,value):
                widget.setCurrentIndex(widget.findData(value));wait(lambda:panel.pending is None)
            select_widget(panel.ports,'P12');select_widget(panel.phases,'Path + Approach')
            panel.play.click();wait(lambda:latest['status']['state']=='PLAYING' and latest['status']['sample_index']>=2)
            panel.pause.click();wait(lambda:latest['status']['state']=='PAUSED')
            index=latest['status']['sample_index'];begin=time.monotonic()
            while time.monotonic()-begin<.15:pump()
            assert latest['status']['sample_index']==index
            panel.pause.click();wait(lambda:latest['status']['state'] in ('PLAYING','FINISHED'))
            panel.reset.click();wait(lambda:latest['status']['state']=='STOPPED' and latest['status']['sample_index']==0)
            select_widget(panel.ports,'P22');select_widget(panel.goals,1);select_widget(panel.planners,'rrt_connect')
            panel.play.click();wait(lambda:latest['status']['state']=='PLAYING')
            panel.pause.click();wait(lambda:latest['status']['state']=='PAUSED')
            responsive=[];QTimer.singleShot(20,lambda:responsive.append(True));wait(lambda:bool(responsive))
            assert panel.grab().save(str(output/'actual_qt_panel.png'))
            select_widget(panel.ports,'P11');assert not panel.play.isEnabled() and panel.goals.count()==0
            report['qt_offscreen_widget_test']=True;report['qt_timer_responsive']=True
        else:
            cases_to_run=[('P12',0,'linear'),('P12',1,'rrt_connect'),('P22',0,'linear'),('P22',1,'rrt_connect')] if mode=='artifact' else [('P22',0,'linear'),('P22',1,'rrt_connect')]
            for port,goal,method in cases_to_run:
                selected=send('select',port_id=port,goal_id=goal,planner=method,phase='Path + Approach')
                assert selected['last_command_ok'];revision=selected['selection_revision'];total=selected['total_samples']
                start_history=len(history);assert send('play')['last_command_ok']
                wait(lambda:latest['status']['state']=='FINISHED',timeout=35)
                played=[s for s in history[start_history:] if s.get('selection_revision')==revision and s.get('state') in ('PLAYING','FINISHED')]
                assert set(s['sample_index'] for s in played)==set(range(total)),(port,goal,total,len(played))
                if mode=='artifact':
                    base=Path(os.environ['WORKSPACE_ARTIFACT_ROOT'])/'task_benchmark/t5_1_s1_20261104_03'
                    for state in played:
                        source,index=state['source_frame']
                        with np.load(base/source,allow_pickle=False) as arrays:
                            np.testing.assert_array_equal(state['q'],arrays['q'][index])
                    # Native saved gripper/mimic transforms must also match RSP TF.
                    record=json.loads((base/'ports'/port/'result.json').read_text())
                    g=next(g for g in record['goals'] if g['solution_id']==goal)
                    saved=g['approach']['samples'][-1]['link_transforms_base']
                    for link in ('l_link','r_link','l_in_link','r_out_link'):
                        expected=np.array(saved[link])
                        def mimic_matches(link=link,expected=expected):
                            try:
                                t=buffer.lookup_transform('base_link',link,Time()).transform;r=t.rotation
                                actual=np.eye(4);actual[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
                                actual[:3,:3]=Rotation.from_quat([r.x,r.y,r.z,r.w]).as_matrix()
                                return np.allclose(actual,expected,atol=1e-9,rtol=0)
                            except Exception:return False
                        wait(mimic_matches)
                check_tf();cases.append({'port':port,'goal':goal,'planner':method,'samples_observed':total,'all_original_indices_seen':True})
            failed=send('select',port_id='P11',goal_id=None,planner=None,phase='Path')
            assert not failed['playable'] and not send('play')['last_command_ok']
            assert latest['status']['state']=='STOPPED'
        pubs=observer.get_topic_names_and_types()
        assert not any(name in ('/cmd_vel','/servo_controller','/ros_robot_controller/bus_servo/set_state') for name,_ in pubs)
        report.update(status='PASS',mode=mode,cases=cases,max_ee_tf_error= max(tf_errors,default=0.),
            scene_object_markers=38,port_labels=9,private_domain=os.environ['ROS_DOMAIN_ID'],
            hardware_control=False,usage='RESEARCH_ONLY')
    finally:
        if panel is not None:panel.timer.stop();panel.close();app.processEvents()
        replay_executor.shutdown()
        if replay is not None:replay.destroy_node()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=15);cleanup['forced']=False
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5);cleanup['forced']=True
        if process is not None:cleanup['exit_code']=process.returncode
        # Parent collection is not enough: no owned names may remain publishing.
        end=time.monotonic()+8
        owned={'replay','robot_state_publisher','panel','rviz2'}
        residual=[]
        while time.monotonic()<end:
            observer_executor.spin_once(timeout_sec=.02)
            residual=[n for n,ns in observer.get_node_names_and_namespaces() if ns=='/idc_replay' and n in owned]
            if not residual:break
        cleanup['residual_owned_nodes']=residual
        observer_executor.shutdown();observer.destroy_node();context.shutdown()
        if log is not None:log.close()
        report['cleanup']=cleanup
        (output/'report.json').write_text(json.dumps(report,indent=2))
    assert cleanup.get('exit_code')==0 and not cleanup.get('forced') and not cleanup['residual_owned_nodes'],cleanup
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--mode',choices=['transport','panel','artifact'],required=True);p.add_argument('--output',required=True)
    args=p.parse_args()
    try:print(json.dumps(run(args.mode,args.output)))
    except Exception:
        import traceback;traceback.print_exc();raise SystemExit(2)
