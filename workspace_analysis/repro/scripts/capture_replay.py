#!/usr/bin/env python3
"""Capture actual owned RViz/Qt replay windows and status; never synthesize images."""
import argparse
import ctypes
from hashlib import sha256
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
sys.dont_write_bytecode=True
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'ros2/jetrover_idc_visualization')]


def image_has_content(image):
    colors={image.pixelColor(x,y).getRgb()[:3]
            for x in range(0,image.width(),max(1,image.width()//12))
            for y in range(0,image.height(),max(1,image.height()//12))}
    return len(colors)>=3


def capture(a):
    if not re.fullmatch('[0-9a-f]{64}',a.expected_manifest_sha256):raise ValueError('trusted manifest SHA256 required')
    if not 1<=a.domain_id<=101:raise ValueError('private loopback domain required')
    out=a.output_dir.absolute();out.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(a.domain_id),PYTHONDONTWRITEBYTECODE='1',
        ROS_LOG_DIR=str(out/'ros_logs'),ROS_HOME=str(out/'ros_home'))
    import rclpy
    from rclpy.node import Node
    from PyQt5.QtWidgets import QApplication
    from jetrover_idc_visualization.panel import ReplayPanel
    command=['ros2','launch','jetrover_idc_visualization','replay.launch.py','workspace_root:='+str(ROOT),
        'benchmark_dir:='+str(a.benchmark_dir.absolute()),'expected_manifest_sha256:='+a.expected_manifest_sha256,
        'domain_id:='+str(a.domain_id),'gui:='+('false' if a.render_capture_binary else 'true')]
    app=QApplication([sys.argv[0]]);rclpy.init();node=Node('evidence_panel',namespace='/idc_replay')
    panel=ReplayPanel(node);panel.show();process=None;report={'status':'ERROR','command':command,'usage':'RESEARCH_ONLY'}
    def wait(condition,timeout=65):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            app.processEvents();time.sleep(.01)
            if condition():return
            if process is not None and process.poll() is not None:raise RuntimeError('owned launch exited')
        raise TimeoutError(str(panel.latest))
    with (out/'launch.log').open('wb') as log:
        try:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            wait(lambda:panel.latest is not None and panel.latest.get('stage')=='READY')
            panel.send('select',port_id=a.port,goal_id=a.goal,planner=a.planner,phase=a.phase)
            wait(lambda:panel.pending is None and panel.latest.get('port_id')==a.port and panel.latest.get('goal_id')==a.goal)
            if not a.inspection_only:
                assert panel.play.isEnabled(),'selected saved trajectory is not playable'
                panel.play.click();wait(lambda:panel.latest.get('state')=='FINISHED')
            report.update(selection=panel.latest,domain_id=a.domain_id)
            assert panel.grab().save(str(out/'qt_panel_actual.png'))
            if a.render_capture_binary:
                from PyQt5.QtGui import QImage
                binary=a.render_capture_binary.resolve(strict=True)
                config=ROOT/'ros2/jetrover_idc_visualization/config/replay.rviz'
                render_command=[str(binary),str(config),str(out/'rviz_actual.png'),'--ros-args',
                    '-r','__ns:=/idc_replay','-r','/tf:=/idc_replay/tf','-r','/tf_static:=/idc_replay/tf_static']
                with (out/'native_render.log').open('wb') as render_log:
                    renderer=subprocess.Popen(render_command,stdout=render_log,stderr=subprocess.STDOUT,start_new_session=True)
                    try:
                        wait(lambda:renderer.poll() is not None,timeout=60)
                        if renderer.returncode!=0:
                            raise RuntimeError('native RViz screenshot process failed: '+str(renderer.returncode))
                    finally:
                        if renderer.poll() is None:
                            renderer.send_signal(signal.SIGINT)
                            try:renderer.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                os.killpg(renderer.pid,signal.SIGTERM);renderer.wait(timeout=5)
                if not image_has_content(QImage(str(out/'rviz_actual.png'))):
                    raise RuntimeError('native RViz render image is uniform')
                report.update(rviz_capture_method='RViz RenderWindow.captureScreenShot',
                    renderer_sha256=sha256(binary.read_bytes()).hexdigest(),renderer_exit_code=renderer.returncode)
            else:
                # Only a window owned by THIS saved launch process group is eligible.
                tree=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
                windows=[]
                for line in tree.splitlines():
                    match=re.match(r'\s*(0x[0-9a-f]+)\s',line)
                    if not match:continue
                    wid=match.group(1)
                    props=subprocess.run(['xprop','-id',wid,'_NET_WM_PID','WM_CLASS'],capture_output=True,text=True).stdout
                    pid=re.search(r'_NET_WM_PID\(CARDINAL\) = (\d+)',props)
                    if pid and 'rviz' in props.lower():
                        try:owned=os.getpgid(int(pid.group(1)))==process.pid
                        except ProcessLookupError:owned=False
                        if owned:
                            info=subprocess.check_output(['xwininfo','-id',wid],text=True)
                            width=re.search(r'Width: (\d+)',info);height=re.search(r'Height: (\d+)',info)
                            if width and height and 'Map State: IsViewable' in info and int(width.group(1))>500 and int(height.group(1))>400:
                                windows.append((int(width.group(1))*int(height.group(1)),int(wid,16)))
                if not windows:raise RuntimeError('no owned visible actual RViz main window')
                windows.sort(reverse=True);window=windows[0][1]
                xlib=ctypes.CDLL('libX11.so.6');xlib.XOpenDisplay.restype=ctypes.c_void_p
                xlib.XOpenDisplay.argtypes=[ctypes.c_char_p];display=xlib.XOpenDisplay(None)
                if not display:raise RuntimeError('no actual X display')
                xlib.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong]
                xlib.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
                xlib.XSync.argtypes=[ctypes.c_void_p,ctypes.c_bool];xlib.XCloseDisplay.argtypes=[ctypes.c_void_p]
                try:
                    xlib.XMoveResizeWindow(display,window,20,30,1250,850);xlib.XRaiseWindow(display,window);xlib.XSync(display,False)
                    until=time.monotonic()+2
                    while time.monotonic()<until:app.processEvents();time.sleep(.02)
                    pixmap=app.primaryScreen().grabWindow(window)
                    report['owned_rviz_window_id']=hex(window)
                    report['active_window_property']=subprocess.run(['xprop','-root','_NET_ACTIVE_WINDOW'],capture_output=True,text=True).stdout.strip()
                    captured=pixmap.toImage()
                    capture_method='QScreen.grabWindow'
                    if not image_has_content(captured):
                        raise RuntimeError('RViz screenshot is uniform/black; display capture unavailable')
                    if captured.isNull() or not captured.save(str(out/'rviz_actual.png')):raise RuntimeError('actual RViz capture failed')
                    report['rviz_capture_method']=capture_method
                finally:xlib.XCloseDisplay(display)
            report.update(status='PASS',selection=panel.latest,domain_id=a.domain_id,
                screenshot_method=report['rviz_capture_method']+' + production QWidget.grab; no generated imagery',
                user_visual_acceptance='PENDING',hardware_control=False)
        except Exception as exc:
            report['error']=str(exc)
            raise
        finally:
            panel.timer.stop();panel.close();panel.executor.shutdown();node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:process.wait(timeout=20);report['forced_shutdown']=False
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5);report['forced_shutdown']=True
            report['launch_exit_code']=None if process is None else process.returncode
            report['images']={p.name:sha256(p.read_bytes()).hexdigest() for p in out.glob('*.png')}
            (out/'evidence.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    assert report['launch_exit_code']==0 and not report.get('forced_shutdown')
    print(json.dumps({'status':report['status'],'output':str(out)}))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--benchmark-dir',type=Path,required=True);p.add_argument('--expected-manifest-sha256',required=True)
    p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--domain-id',type=int,default=84)
    p.add_argument('--port',default='P22');p.add_argument('--goal',type=int,default=0)
    p.add_argument('--planner',choices=['linear','rrt_connect'],default='linear')
    p.add_argument('--phase',choices=['Path','Approach','Path + Approach'],default='Approach')
    p.add_argument('--render-capture-binary',type=Path,help='Optional built native RViz render screenshot helper')
    p.add_argument('--inspection-only',action='store_true',help='Capture selected stored frame or no-path goal without playback')
    capture(p.parse_args())
