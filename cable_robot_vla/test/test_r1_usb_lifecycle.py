"""Offline USB lifecycle tests. Physical tty/USB devices must never be opened."""
import errno
import json
from pathlib import Path
import os
import struct
import subprocess
import sys

import pytest
import serial as actual_serial
from patches.jetrover_readback.guarded_readback import (
    initialize_existing_serial, close_existing_serial_once)
from test_r1_vendor_patch import audited_sources, patched_sdk, constructor_sdk


class SignalPort:
    def __init__(self, error=None, fail_after_open=False):
        self.events=[]
        self.error=error
        self.fail_after_open=fail_after_open
        self.is_open=False
        self.fd=None
        self.close_count=0
    @property
    def rts(self):return False
    @rts.setter
    def rts(self,value):self.events.append(("rts",value))
    @property
    def dtr(self):return False
    @dtr.setter
    def dtr(self,value):self.events.append(("dtr",value))
    def setPort(self,path):self.events.append(("device",path))
    def open(self):
        self.events.append(("open",))
        if self.fail_after_open:self.is_open=True;self.fd=object()
        if self.error:raise self.error
        self.is_open=True;self.fd=object()
    def close(self):
        self.close_count+=1
        self.is_open=False
        self.fd=None


def test_signal_order_is_unchanged_and_close_is_idempotent():
    port=SignalPort()
    report=initialize_existing_serial(port,"/fixture/no-device")
    assert port.events==[("rts",False),("dtr",False),("device","/fixture/no-device"),("open",)]
    assert report["success"] and report["open_attempts"]==1
    assert close_existing_serial_once(port)["port_closed"]
    assert close_existing_serial_once(port)["status"]=="already_closed"
    assert port.close_count==1


def test_initializer_preserves_original_timeout_without_retry(capsys):
    error=TimeoutError(errno.ETIMEDOUT,"fixture")
    port=SignalPort(error)
    with pytest.raises(TimeoutError) as caught:initialize_existing_serial(port,"/fixture/no-device")
    assert caught.value is error
    report=error.serial_initialization_report
    assert report["error_code"]=="SERIAL_OPEN_CONTROL_TIMEOUT"
    assert report["open_attempts"]==1 and report["cleanup"]=="not_open_no_duplicate_close"
    assert port.close_count==0 and len([e for e in port.events if e[0]=="open"])==1
    assert json.loads(capsys.readouterr().err)["report"]["errno"]==110


def test_partial_open_failure_closes_owned_resource_once():
    port=SignalPort(OSError(errno.EIO,"fixture"),fail_after_open=True)
    with pytest.raises(OSError):initialize_existing_serial(port,"/fixture/no-device")
    assert port.close_count==1 and port.fd is None
    close_existing_serial_once(port)
    assert port.close_count==1


@pytest.mark.parametrize("failed_signal,code",[
    ("dtr","USB_DTR_CONTROL_TIMEOUT"),("rts","USB_RTS_CONTROL_TIMEOUT")])
def test_actual_pyserial_open_closes_fake_fd_once_on_control_timeout(
        monkeypatch,failed_signal,code):
    import serial
    import serial.serialposix as posix
    # Execute the actual installed open()/signal/cleanup code, mock only OS I/O.
    port=serial.Serial(None,baudrate=1000000,timeout=5,write_timeout=.05)
    fake_fd=-8947
    opened=[];closed=[];calls=[]
    real_open,real_close,real_ioctl=posix.os.open,posix.os.close,posix.fcntl.ioctl
    def fake_open(path,*args,**kwargs):
        if path=="/fixture/rrc_usb":opened.append(path);return fake_fd
        return real_open(path,*args,**kwargs)
    def fake_close(fd):
        if fd==fake_fd:closed.append(fd)
        else:return real_close(fd)
    def fake_ioctl(fd,command,argument):
        if fd!=fake_fd:return real_ioctl(fd,command,argument)
        calls.append(argument)
        if argument==(posix.TIOCM_DTR_str if failed_signal=="dtr" else posix.TIOCM_RTS_str):
            raise TimeoutError(errno.ETIMEDOUT,"fake control transfer")
        return 0
    monkeypatch.setattr(posix.os,"open",fake_open)
    monkeypatch.setattr(posix.os,"close",fake_close)
    monkeypatch.setattr(posix.fcntl,"ioctl",fake_ioctl)
    monkeypatch.setattr(port,"_reconfigure_port",lambda **_:None)
    with pytest.raises(TimeoutError) as error:initialize_existing_serial(port,"/fixture/rrc_usb")
    assert error.value.serial_initialization_report["error_code"]==code
    assert opened==["/fixture/rrc_usb"] and closed==[fake_fd]
    assert port.fd is None and not port.is_open
    port.close()
    assert closed==[fake_fd]


def test_board_close_cancels_existing_receiver_then_closes_once(patched_sdk,monkeypatch):
    sdk,_,ports,_=constructor_sdk(patched_sdk,monkeypatch)
    board=sdk.Board(maintenance_tx_timeout_s=.05)
    calls=[];alive=[True]
    ports[0].cancel_read=lambda:calls.append("cancel")
    def join(timeout):
        calls.append(("join",timeout));alive[0]=False
    from types import SimpleNamespace as S
    board._receiver_thread=S(is_alive=lambda:alive[0],join=join)
    first=board.close_maintenance_transport()
    second=board.close_maintenance_transport()
    assert first==second and first["port_closed"]
    assert calls==["cancel",("join",.5)] and ports[0].close_calls==1
    assert ports[0].frames==[]


def test_receiver_that_does_not_exit_is_not_closed_or_force_killed(patched_sdk,monkeypatch):
    from types import SimpleNamespace as S
    sdk,_,ports,_=constructor_sdk(patched_sdk,monkeypatch)
    board=sdk.Board(maintenance_tx_timeout_s=.05)
    calls=[]
    ports[0].cancel_read=lambda:calls.append("cancel")
    board._receiver_thread=S(is_alive=lambda:True,join=lambda **_:calls.append("join"))
    result=board.close_maintenance_transport()
    assert not result["port_closed"] and result["error"]=="RECEIVER_STILL_RUNNING_NO_CLOSE"
    assert ports[0].close_calls==0 and ports[0].frames==[]
    board.close_maintenance_transport()
    assert calls==["cancel","join"]


@pytest.mark.parametrize("delay",[0,.1])
def test_candidate_sigterm_exits_zero_and_closes_once_offline(tmp_path,audited_sources,delay):
    pytest.importorskip("rclpy")
    import ast,select,signal,time
    from patches.jetrover_readback.build_candidate_patch import candidate_sources
    source=candidate_sources(*audited_sources)[1]
    main=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=="main")
    script=tmp_path/"offline_candidate_shutdown.py"
    prelude='''import rclpy
import signal
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from types import SimpleNamespace as S
class RosRobotController(Node):
    def __init__(self,name):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        super().__init__(name)
        self.readback_maintenance=True
        self.running=True
        def close():
            print("CLOSE_CALLED",flush=True)
            return {"port_closed":True}
        self.board=S(close_maintenance_transport=close,
            set_motor_speed=lambda *_:print("MOTOR_CALLED",flush=True))
        print("OFFLINE_READY",flush=True)
'''
    script.write_text(prelude+ast.unparse(main)+"\nmain()\n")
    child=subprocess.Popen([sys.executable,"-u",str(script)],stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,text=True,
        env=dict(os.environ,ROS_DOMAIN_ID="117",RMW_IMPLEMENTATION="rmw_fastrtps_cpp"))
    output=""
    try:
        end=time.monotonic()+8
        while time.monotonic()<end:
            ready,_,_=select.select([child.stdout],[],[],.2)
            if ready:
                line=child.stdout.readline();output+=line
                if "OFFLINE_READY" in line:break
            if child.poll() is not None:break
        assert "OFFLINE_READY" in output,output
        if delay:time.sleep(delay)
        child.send_signal(signal.SIGTERM)
        rest,_=child.communicate(timeout=5);output+=rest
        assert child.returncode==0,output
        assert output.count("CLOSE_CALLED")==1 and "MOTOR_CALLED" not in output
        assert "Traceback" not in output
    finally:
        if child.poll() is None:child.kill();child.communicate()


def test_initializer_refuses_an_already_open_owner_without_toggling_signals():
    port=SignalPort();port.is_open=True;port.fd=object()
    with pytest.raises(ValueError):initialize_existing_serial(port,"/fixture/no-device")
    assert port.events==[] and port.close_count==0


@pytest.mark.parametrize('execute,error',[ (False,None),(True,None),(True,TimeoutError(110,'fixture')) ])
def test_open_probe_is_opt_in_and_has_no_retry(tmp_path,monkeypatch,execute,error):
    import patches.jetrover_readback.probe_serial_open_close as probe
    from types import SimpleNamespace as S
    monkeypatch.setattr(probe.subprocess,'check_output',
        lambda cmd,**_:('a'*40+'\n') if 'rev-parse' in cmd else '')
    monkeypatch.setattr(probe,'inventory',lambda:{'serial_ownership':{
        'visible_openers':[],'inaccessible_process_fd_directories':0,
        'resolved_path':'/dev/ttyACM0'}})
    monkeypatch.setattr(probe,'cached_identity',lambda:{'vid':'1a86','pid':'55d4','driver':'/sys/bus/usb/drivers/cdc_acm'})
    created=[]
    def factory(*args,**kwargs):
        assert execute,'inspect must not construct Serial'
        port=SignalPort(error);created.append(port);return port
    monkeypatch.setitem(sys.modules,'serial',S(Serial=factory))
    output=tmp_path/'probe.json'
    args=['--output',str(output)]
    if execute:args+=['--execute-once','--expected-source-sha','a'*40]
    status=probe.main(args)
    report=json.loads(output.read_text())
    assert report['physical_open_attempts']==int(execute)
    assert report['serial_writes']==0 and report['servo_queries']==0 and not report['retry']
    assert len(created)==int(execute)
    if error:
        assert status==2 and report['errno']==110 and created[0].close_count==0
    elif execute:
        assert status==0 and created[0].close_count==1
    else:assert status==0 and report['status']=='INSPECTED_NO_OPEN'


@pytest.mark.skipif(sys.platform!="linux",reason="virtual Linux PTY")
def test_existing_receiver_cancel_and_fd_cleanup_on_virtual_pty(patched_sdk,monkeypatch):
    import pty,threading
    master,slave=pty.openpty()
    sdk,_,_,_=patched_sdk
    entered=threading.Event();board=None
    def factory(*args,**kwargs):
        port=actual_serial.Serial(*args,**kwargs)
        original=port.read
        def read(*a,**k):
            entered.set();return original(*a,**k)
        port.read=read
        def forbidden_write(*a,**k):raise AssertionError("no serial data write allowed")
        port.write=forbidden_write
        return port
    monkeypatch.setattr(sdk.serial,"Serial",factory)
    try:
        device=os.ttyname(slave);assert device.startswith("/dev/pts/")
        board=sdk.Board(device=device,baudrate=115200,maintenance_tx_timeout_s=.05)
        board.enable_reception()
        assert entered.wait(2)
        first=board.close_maintenance_transport()
        assert first["port_closed"] and not board._receiver_thread.is_alive()
        assert board.port.fd is None and not board.port.is_open
        assert board.close_maintenance_transport()==first
    finally:
        if board is not None and board.port.is_open:board.port.close()
        os.close(slave);os.close(master)
