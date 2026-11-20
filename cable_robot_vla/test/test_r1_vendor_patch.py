"""Apply the candidate ONLY to pytest temporary copies; real serial is forbidden."""
import ast
from hashlib import sha256
import importlib.util
from pathlib import Path
import struct
import subprocess
import sys
import threading
from types import ModuleType, SimpleNamespace as S

import pytest
from patches.jetrover_readback.build_candidate_patch import (
    SDK_SHA, NODE_SHA, candidate_sources, build_patch, HERE)


@pytest.fixture
def audited_sources():
    root=Path("/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller")
    if not root.exists():
        pytest.skip("exact vendor sources are required for offline patch compatibility")
    sdk=(root/"ros_robot_controller_sdk.py").read_text()
    node=(root/"ros_robot_controller_node.py").read_text()
    assert sha256(sdk.encode()).hexdigest()==SDK_SHA
    assert sha256(node.encode()).hexdigest()==NODE_SHA
    return sdk,node


def test_patch_applies_and_reverses_only_in_temporary_copy(tmp_path,audited_sources):
    sdk,node=audited_sources
    root=tmp_path/"ros_robot_controller";root.mkdir()
    (root/"ros_robot_controller_sdk.py").write_text(sdk)
    (root/"ros_robot_controller_node.py").write_text(node)
    assert (HERE/"sdk_bounded_readback_candidate.patch").read_text()==build_patch(sdk,node)
    patch=tmp_path/"candidate.patch";patch.write_text(build_patch(sdk,node))
    subprocess.run(["git","apply","--check",str(patch)],cwd=tmp_path,check=True)
    subprocess.run(["git","apply",str(patch)],cwd=tmp_path,check=True)
    expected_sdk,expected_node=candidate_sources(sdk,node)
    assert (root/"ros_robot_controller_sdk.py").read_text()==expected_sdk
    assert (root/"ros_robot_controller_node.py").read_text()==expected_node
    assert (root/"guarded_readback.py").read_text()==(HERE/"guarded_readback.py").read_text()
    subprocess.run(["git","apply","--reverse","--check",str(patch)],cwd=tmp_path,check=True)
    subprocess.run(["git","apply","--reverse",str(patch)],cwd=tmp_path,check=True)
    assert (root/"ros_robot_controller_sdk.py").read_text()==sdk
    assert (root/"ros_robot_controller_node.py").read_text()==node


def test_unreviewed_vendor_revision_rejected(audited_sources):
    sdk,node=audited_sources
    with pytest.raises(ValueError):candidate_sources(sdk+"\n",node)


class StopFixture(Exception):
    pass


class FakePort:
    write_timeout=.005
    def __init__(self):
        self.frames=[];self.on_write=None;self.incoming=[]
    def write(self,frame):
        self.frames.append(bytes(frame))
        if self.on_write:self.on_write(bytes(frame))
        return len(frame)
    def read(self):
        if not self.incoming:raise StopFixture()
        return bytes([self.incoming.pop(0)])


@pytest.fixture
def patched_sdk(monkeypatch,audited_sources):
    original,node=audited_sources
    sdk_source,node_source=candidate_sources(original,node)
    package=ModuleType("offline_vendor");package.__path__=[]
    monkeypatch.setitem(sys.modules,"offline_vendor",package)
    helper=ModuleType("offline_vendor.guarded_readback")
    helper.__file__=str(HERE/"guarded_readback.py")
    monkeypatch.setitem(sys.modules,helper.__name__,helper)
    exec(compile((HERE/"guarded_readback.py").read_text(),helper.__file__,"exec"),helper.__dict__)
    def forbidden(*args,**kwargs):
        raise AssertionError("real Serial construction is forbidden in offline tests")
    monkeypatch.setitem(sys.modules,"serial",S(Serial=forbidden))
    sdk=ModuleType("offline_vendor.sdk");sdk.__package__="offline_vendor"
    monkeypatch.setitem(sys.modules,sdk.__name__,sdk)
    exec(compile(sdk_source,"offline_sdk.py","exec"),sdk.__dict__)
    return sdk,helper,node_source,original


def board_fixture(sdk,helper,*,arm=True):
    board=sdk.Board.__new__(sdk.Board)
    board.port=FakePort();board.servo_read_lock=threading.Lock();board.enable_recv=True
    board._bounded_bus_feedback=helper.ReadbackGate(board.servo_read_lock)
    board.last_bus_position_sample=None
    if arm:board._bounded_bus_feedback.arm("OFFLINE_FIXTURE_ONLY",clean_epoch_verified=True,
        exclusive_owner_verified=True,echo_contract_verified=True,write_bound_verified=True,inner_response_match_verified=True)
    board.frame=[];board.recv_count=0
    board.state=sdk.PacketControllerState.PACKET_CONTROLLER_STATE_STARTBYTE1
    board.parsers={sdk.PacketFunction.PACKET_FUNC_BUS_SERVO:board.packet_report_serial_servo}
    return board


def wire_response(sdk,servo_id=1,*,bad_crc=False):
    body=struct.pack("<BBbh",servo_id,5,0,600+servo_id)
    data=bytes([int(sdk.PacketFunction.PACKET_FUNC_BUS_SERVO),len(body)])+body
    return bytes([0xAA,0x55])+data+bytes([sdk.checksum_crc8(data)^(1 if bad_crc else 0)])


def test_existing_crc_receiver_to_guard_to_position_path(patched_sdk):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper)
    def send(frame):
        board.port.incoming=list(wire_response(sdk))
        try:board.recv_task()
        except StopFixture:pass
    board.port.on_write=send
    assert board.bus_servo_read_position(1)==[601]
    sample=board.last_bus_position_sample
    assert sample.raw_pulse==601 and sample.servo_id==1 and sample.device_sample_time_ns is None


def test_bad_crc_times_out_without_returning_position(patched_sdk):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper)
    def send(frame):
        board.port.incoming=list(wire_response(sdk,bad_crc=True))
        try:board.recv_task()
        except StopFixture:pass
    board.port.on_write=send
    with pytest.raises(helper.ReadbackFault) as error:board.bus_servo_read_position(1)
    assert error.value.code=="READ_TIMEOUT"
    assert board.last_bus_position_sample is None
    assert board.servo_read_lock.acquire(blocking=False);board.servo_read_lock.release()


def test_default_gate_refuses_before_any_write(patched_sdk):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper,arm=False)
    with pytest.raises(helper.ReadbackFault) as error:board.bus_servo_read_position(1)
    assert error.value.code=="DISARMED" and board.port.frames==[]


def test_current_unbounded_serial_write_is_refused(patched_sdk):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper);board.port.write_timeout=None
    with pytest.raises(helper.ReadbackFault) as error:board.bus_servo_read_position(1)
    assert error.value.code=="BOUNDED_WRITE_UNVERIFIED" and board.port.frames==[]


def test_receiver_exception_latches_without_restart(patched_sdk):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper)
    board.recv_task=lambda:(_ for _ in ()).throw(OSError("offline RX failure"))
    board._recv_task_guarded()
    assert board._bounded_bus_feedback.status()["error"]=="RECEIVER_FAILED"


def test_control_packet_generation_unchanged_even_after_read_fault(patched_sdk,audited_sources):
    sdk,helper,_,original=patched_sdk;board=board_fixture(sdk,helper)
    baseline=ModuleType("offline_original")
    exec(compile(original,"offline_original.py","exec"),baseline.__dict__)
    before=baseline.Board.__new__(baseline.Board);before.port=FakePort()
    for target in (before,board):
        target.bus_servo_set_position(.5,[[1,500],[10,600]])
    assert board.port.frames==before.port.frames
    board._bounded_bus_feedback.fail("READ_TIMEOUT")
    for target in (before,board):
        target.bus_servo_set_position(.2,[[2,700]])
    assert board.port.frames==before.port.frames
    def methods(text):
        return {n.name:ast.dump(n,include_attributes=False) for n in ast.walk(ast.parse(text))
            if isinstance(n,ast.FunctionDef)}
    old=methods(original);new=methods(candidate_sources(original,audited_sources[1])[0])
    for name in old:
        if name not in ("__init__","bus_servo_read_and_unpack","packet_report_serial_servo","buf_write"):
            assert new[name]==old[name],name


def service_wrapper(source,fault_type):
    tree=ast.parse(source)
    wrapper=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=="get_bus_servo_state")
    namespace={"ReadbackFault":fault_type}
    exec(compile(ast.Module(body=[wrapper],type_ignores=[]),"offline_service.py","exec"),namespace)
    return namespace["get_bus_servo_state"]


def command(servo_id=1,**flags):
    data=dict(id=servo_id,get_position=1,get_id=0,get_offset=0,get_voltage=0,get_temperature=0,
        get_position_limit=0,get_voltage_limit=0,get_max_temperature_limit=0,get_torque_state=0)
    data.update(flags)
    return S(**data)


def test_service_fault_clears_partial_snapshot_and_returns_failure(patched_sdk):
    _,helper,source,_=patched_sdk;messages=[]
    def partial(request,response):
        response.state=[S(position=[500])]
        raise helper.ReadbackFault("READ_TIMEOUT")
    owner=S(_get_bus_servo_state_legacy=partial,get_logger=lambda:S(error=messages.append))
    response=S(state=[],success=False)
    result=service_wrapper(source,helper.ReadbackFault)(owner,S(cmd=[command(1),command(2)]),response)
    assert result.success is False and result.state==[] and messages==["Readback refused: READ_TIMEOUT"]


@pytest.mark.parametrize("commands",[
    [],[command(6)],[command(1),command(1)],[command(1,get_torque_state=1)],[command(1,get_position=0)]])
def test_service_rejects_bad_or_out_of_scope_request_before_read(patched_sdk,commands):
    _,helper,source,_=patched_sdk
    def forbidden(*args):raise AssertionError("legacy read must not run")
    owner=S(_get_bus_servo_state_legacy=forbidden)
    result=service_wrapper(source,helper.ReadbackFault)(owner,S(cmd=commands),S(state=[],success=False))
    assert result.success is False and result.state==[]


@pytest.mark.parametrize('ids',[[1],[1,2,3,4,5,10]])
def test_service_success_returns_complete_position_only_snapshot(patched_sdk,ids):
    _,helper,source,_=patched_sdk
    def complete(request,response):
        response.state=[S(position=[400+item.id]) for item in request.cmd]
        response.success=True
        return response
    owner=S(_get_bus_servo_state_legacy=complete)
    result=service_wrapper(source,helper.ReadbackFault)(owner,S(cmd=[command(i) for i in ids]),S(state=[],success=False))
    assert result.success is True
    assert [s.position for s in result.state]==[[400+i] for i in ids]


def test_service_short_snapshot_cannot_report_success(patched_sdk):
    _,helper,source,_=patched_sdk
    owner=S(_get_bus_servo_state_legacy=lambda request,response:S(state=[],success=True))
    result=service_wrapper(source,helper.ReadbackFault)(owner,S(cmd=[command(1)]),S(state=[],success=False))
    assert result.success is False and result.state==[]


def test_failed_new_sdk_read_clears_previous_diagnostic_sample(patched_sdk):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper)
    board.port.on_write=lambda frame:board.packet_report_serial_servo(struct.pack('<BBbh',1,5,0,500))
    assert board.bus_servo_read_position(1)==[500]
    assert board.last_bus_position_sample is not None
    board.port.on_write=None
    with pytest.raises(helper.ReadbackFault):board.bus_servo_read_position(2)
    assert board.last_bus_position_sample is None


@pytest.mark.parametrize("written", [0, 1, None, True, 8])
def test_incomplete_write_cannot_return_even_with_a_reply(patched_sdk, written):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper)
    def partial_write(frame):
        board.packet_report_serial_servo(struct.pack("<BBbh",1,5,0,500))
        return written
    board.port.write=partial_write
    with pytest.raises(helper.ReadbackFault) as error:board.bus_servo_read_position(1)
    assert error.value.code=="SEND_FAILED"
    assert str(error.value.__cause__)=="READBACK_TX_INCOMPLETE"
    assert board.last_bus_position_sample is None
    assert board._bounded_bus_feedback.status()["state"]=="FAULT"
    assert board.servo_read_lock.acquire(blocking=False);board.servo_read_lock.release()


def test_legacy_write_default_return_semantics_unchanged(patched_sdk):
    sdk,helper,_,_=patched_sdk;board=board_fixture(sdk,helper)
    board.port.write=lambda frame:0
    assert board.buf_write(sdk.PacketFunction.PACKET_FUNC_BUS_SERVO,[1,1]) is None
