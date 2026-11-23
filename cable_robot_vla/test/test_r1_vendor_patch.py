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
        if name not in ("__init__","bus_servo_read_and_unpack","packet_report_serial_servo","buf_write","recv_task"):
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


class ConstructorPort(FakePort):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.arguments = (args, kwargs)
        self.write_timeout = kwargs.get("write_timeout")
        self.closed = False
        self.opened = False

    def setPort(self, device):
        self.device = device

    def open(self):
        self.opened = True

    def close(self):
        self.closed = True


def constructor_sdk(patched_sdk, monkeypatch):
    sdk, helper, node_source, original = patched_sdk
    ports, exclusive = [], []
    def make(*args, **kwargs):
        port = ConstructorPort(*args, **kwargs)
        ports.append(port)
        return port
    monkeypatch.setattr(sdk.serial, "Serial", make)
    monkeypatch.setattr(sdk.time, "sleep", lambda _: None)
    monkeypatch.setattr(sdk.threading, "Thread", lambda **_: S(start=lambda: None))
    monkeypatch.setattr(sdk, "claim_linux_exclusive", exclusive.append)
    return sdk, helper, ports, exclusive


@pytest.mark.parametrize("mode", [None, .05])
def test_board_constructor_maintenance_is_explicit_and_does_not_transmit(
        patched_sdk, monkeypatch, mode):
    sdk, helper, ports, exclusive = constructor_sdk(patched_sdk, monkeypatch)
    board = sdk.Board(maintenance_tx_timeout_s=mode)
    assert len(ports) == 1 and ports[0].frames == [] and ports[0].opened
    assert board._bounded_bus_feedback.status()["state"] == "DISARMED"
    if mode is None:
        assert ports[0].arguments[1] == {"timeout": 5}
        assert board._maintenance_writer is None and exclusive == []
    else:
        assert ports[0].arguments[1] == {"timeout": 5, "write_timeout": .05}
        assert exclusive == [ports[0]]
        assert board._maintenance_writer.port is ports[0]


@pytest.mark.parametrize("mode", [True, 0, .1, float("nan"), float("inf")])
def test_invalid_maintenance_timeout_rejected_before_serial_open(
        patched_sdk, monkeypatch, mode):
    sdk, _, ports, _ = constructor_sdk(patched_sdk, monkeypatch)
    with pytest.raises(ValueError):
        sdk.Board(maintenance_tx_timeout_s=mode)
    assert ports == []


def test_exclusive_claim_failure_closes_port_without_receiver_start(
        patched_sdk, monkeypatch):
    sdk, _, ports, _ = constructor_sdk(patched_sdk, monkeypatch)
    def fail(port):
        raise OSError("offline exclusive failure")
    monkeypatch.setattr(sdk, "claim_linux_exclusive", fail)
    with pytest.raises(OSError):
        sdk.Board(maintenance_tx_timeout_s=.05)
    assert ports[0].closed and ports[0].frames == []


def test_maintenance_motion_blocked_and_fault_disables_read_gate(
        patched_sdk, monkeypatch):
    sdk, helper, ports, _ = constructor_sdk(patched_sdk, monkeypatch)
    board = sdk.Board(maintenance_tx_timeout_s=.05)
    with pytest.raises(helper.ReadbackFault) as error:
        board.bus_servo_set_position(.5, [[1, 500]])
    assert error.value.code == "MAINTENANCE_COMMAND_FORBIDDEN"
    assert ports[0].frames == []
    assert board._bounded_bus_feedback.status()["state"] == "FAULT"


def test_maintenance_buzzer_allowed_but_read_remains_disarmed(
        patched_sdk, monkeypatch):
    sdk, helper, ports, _ = constructor_sdk(patched_sdk, monkeypatch)
    board = sdk.Board(maintenance_tx_timeout_s=.05)
    board.set_buzzer(1900, .2, .01, 1)
    assert len(ports[0].frames) == 1 and ports[0].frames[0][2] == 2
    with pytest.raises(helper.ReadbackFault) as error:
        board.bus_servo_read_position(1)
    assert error.value.code == "DISARMED" and len(ports[0].frames) == 1


def fake_node_class(source, mode, reference=""):
    """Execute exact candidate constructor against fake Node/Board, not ROS."""
    calls, subscriptions, services = [], [], []
    class BoardStub:
        def __init__(self, **kwargs):
            calls.append(("Board", kwargs))
        def enable_reception(self):
            calls.append(("enable", None))
        def pwm_servo_set_offset(self, *args):
            calls.append(("pwm", args))
        def set_motor_speed(self, *args):
            calls.append(("motor", args))
    class Base:
        def __init__(self, name):
            self.name = name
            self.parameters = {"readback_maintenance": mode, "readback_probe_reference": reference}
        def declare_parameter(self, name, value):
            self.parameters.setdefault(name, value)
        def get_parameter(self, name):
            return S(value=self.parameters[name])
        def create_publisher(self, *args):
            return S()
        def create_subscription(self, message, topic, *args):
            subscriptions.append(topic)
        def create_service(self, message, name, *args):
            services.append(name)
        def get_clock(self):
            return S()
        def get_logger(self):
            return S(info=lambda *_: None)
    namespace = dict(Node=Base, Board=BoardStub,
        rclpy=S(init=lambda: None),
        threading=S(Thread=lambda **_: S(start=lambda: None)))
    for name in ("Imu Joy Sbus ButtonState UInt16 LedState BuzzerState OLEDState "
                 "MotorsState Bool SetBusServoState ServosPosition "
                 "SetPWMServoState GetBusServoState GetPWMServoState Trigger").split():
        namespace[name] = type(name, (), {})
    tree = ast.parse(source)
    class_node = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    exec(compile(ast.Module(body=[class_node], type_ignores=[]),
                 "offline_maintenance_node.py", "exec"), namespace)
    return namespace["RosRobotController"], calls, subscriptions, services


@pytest.mark.parametrize("mode", [False, True])
def test_exact_node_constructor_has_no_maintenance_actuator_interfaces(
        patched_sdk, mode):
    _, _, source, _ = patched_sdk
    cls, calls, topics, services = fake_node_class(source, mode)
    node = cls("offline")
    if mode:
        assert calls == [("Board", {"maintenance_tx_timeout_s": .05}), ("enable", None)]
        assert topics == ["~/set_buzzer"]
        assert services == ["~/bus_servo/get_state", "~/init_finish"]
    else:
        assert calls == [("Board", {}), ("enable", None),
            ("pwm", (1, 0)), ("motor", ([[1, 0], [2, 0], [3, 0], [4, 0]],))]
        assert len(topics) == 8 and len(services) == 3


@pytest.mark.parametrize("mode", [False, True])
def test_exact_main_interrupt_does_not_write_motor_in_maintenance(
        patched_sdk, mode):
    _, _, source, _ = patched_sdk
    calls = []
    node = S(readback_maintenance=mode,
        board=S(set_motor_speed=lambda *_: calls.append("motor")),
        destroy_node=lambda: calls.append("destroy"))
    def interrupt(_):
        raise KeyboardInterrupt()
    namespace = dict(RosRobotController=lambda _: node,
        rclpy=S(spin=interrupt, shutdown=lambda: calls.append("shutdown")))
    main = next(n for n in ast.parse(source).body
                if isinstance(n, ast.FunctionDef) and n.name == "main")
    exec(compile(ast.Module(body=[main], type_ignores=[]),
                 "offline_main.py", "exec"), namespace)
    namespace["main"]()
    assert calls == (["destroy", "shutdown"] if mode
                     else ["motor", "destroy", "shutdown"])



def test_maintenance_writer_existing_crc_path_and_key_reuse_guard(
        patched_sdk, monkeypatch):
    sdk, helper, ports, _ = constructor_sdk(patched_sdk, monkeypatch)
    board = sdk.Board(maintenance_tx_timeout_s=.05)
    board.enable_reception()
    board._bounded_bus_feedback.arm("OFFLINE_ONLY_NOT_HARDWARE_PROOF",
        clean_epoch_verified=True, exclusive_owner_verified=True,
        echo_contract_verified=True, write_bound_verified=True,
        inner_response_match_verified=True)
    def respond(raw):
        servo_id = raw[5]
        board.port.incoming = list(wire_response(sdk, servo_id))
        try:
            board.recv_task()
        except StopFixture:
            pass
    board.port.on_write = respond
    assert [board.bus_servo_read_position(i) for i in [1, 2, 3, 4, 5, 10]] == [
        [601], [602], [603], [604], [605], [610]]
    with pytest.raises(helper.ReadbackFault) as error:
        board.bus_servo_read_position(1)
    assert error.value.code == "KEY_REUSE_UNSAFE"
    assert len(ports[0].frames) == 6


def test_maintenance_short_write_buzzer_error_latches_position_path(
        patched_sdk, monkeypatch):
    sdk, helper, ports, _ = constructor_sdk(patched_sdk, monkeypatch)
    board = sdk.Board(maintenance_tx_timeout_s=.05)
    ports[0].write = lambda _: 0
    with pytest.raises(helper.ReadbackFault) as error:
        board.set_buzzer(1900, .2, .01, 1)
    assert error.value.code == "MAINTENANCE_TX_SHORT_WRITE"
    assert board._bounded_bus_feedback.status()["error"] == error.value.code
    assert board.servo_read_lock.acquire(blocking=False)
    board.servo_read_lock.release()


def test_probe_success_preserves_actual_outer_bytes_and_stays_unverified(patched_sdk):
    sdk, helper, _, _ = patched_sdk
    board = board_fixture(sdk, helper, arm=False)
    board._maintenance_writer = helper.MaintenanceFrameWriter(board.port, sdk.checksum_crc8)
    def reply(raw):
        board.port.incoming = list(wire_response(sdk))
        try:
            board.recv_task()
        except StopFixture:
            pass
    board.port.on_write = reply
    report = board.probe_bus_servo_id1_once("review-id1")
    assert report["raw_pulse"] == 601 and report["outer_frame_validated"]
    assert report["rx_frame_hex"] == wire_response(sdk).hex()
    assert report["tx_frame_hex"] == board.port.frames[0].hex()
    assert report["source_type"] == "unknown"
    assert not report["inner_association_verified"]
    assert not report["canonical_observation_eligible"]
    assert report["device_sample_time_ns"] is None
    assert report["read_latency_ns"] >= 0
    assert report["capture_elapsed_ns"] >= report["read_latency_ns"]
    assert report["gate_status"]["state"] == "COMPLETE_UNVERIFIED"
    second = board.probe_bus_servo_id1_once("do-not-retry")
    assert second["error"] == "REARM_FORBIDDEN"
    assert second["tx_frame_hex"] is None and second["rx_frame_hex"] is None
    assert not second["outer_frame_validated"] and second["raw_pulse"] is None
    assert len(board.port.frames) == 1
    with pytest.raises(helper.ReadbackFault):
        board.bus_servo_read_position(1)
    assert len(board.port.frames) == 1


@pytest.mark.parametrize("kind,error", [
    ("none", "READ_TIMEOUT"), ("bad_crc", "READ_TIMEOUT"),
    ("wrong_id", "SERVO_ID_MISMATCH"), ("wrong_cmd", "COMMAND_MISMATCH"),
    ("bad_length", "INVALID_PAYLOAD_OR_CRC_PROVENANCE"),
    ("device_error", "DEVICE_ERROR"), ("duplicate", "DUPLICATE_REPLY"),
    ("io", "SEND_FAILED")])
def test_probe_fault_never_returns_a_position_or_retries(patched_sdk, kind, error):
    sdk, helper, _, _ = patched_sdk
    board = board_fixture(sdk, helper, arm=False)
    board._maintenance_writer = helper.MaintenanceFrameWriter(board.port, sdk.checksum_crc8)
    def reply(raw):
        if kind == "io":
            raise OSError("offline")
        payload = struct.pack("<BBbh", 2 if kind == "wrong_id" else 1,
            6 if kind == "wrong_cmd" else 5, -1 if kind == "device_error" else 0, 500)
        if kind == "none":
            return
        if kind == "bad_length":
            payload = payload[:-1]
        body = bytes([5, len(payload)])+payload
        wire = b"\xaa\x55"+body+bytes([sdk.checksum_crc8(body) ^ (kind == "bad_crc")])
        board.port.incoming = list(wire*(2 if kind == "duplicate" else 1))
        try:
            board.recv_task()
        except StopFixture:
            pass
    board.port.on_write = reply
    report = board.probe_bus_servo_id1_once("offline-case")
    assert report["error"] == error and report["raw_pulse"] is None
    assert not report["outer_frame_validated"]
    assert report["gate_status"]["state"] == "FAULT"
    assert board.probe_bus_servo_id1_once("no-retry")["error"] == "REARM_FORBIDDEN"
    assert len(board.port.frames) == 1
    assert board.servo_read_lock.acquire(blocking=False)
    board.servo_read_lock.release()


def test_probe_requires_existing_maintenance_writer(patched_sdk):
    sdk, helper, _, _ = patched_sdk
    board = board_fixture(sdk, helper, arm=False)
    assert board.probe_bus_servo_id1_once("case")["error"] == "MAINTENANCE_REQUIRED"
    assert board.port.frames == []


def test_parser_trace_insertion_preserves_every_original_parser_statement(audited_sources):
    old, node = audited_sources
    new = candidate_sources(old, node)[0]
    def method(text):
        return next(n for n in ast.walk(ast.parse(text))
                    if isinstance(n, ast.FunctionDef) and n.name == "recv_task")
    current = method(new)
    class RemoveTrace(ast.NodeTransformer):
        def visit_If(self, item):
            if ("_last_bus_rx_frame" in ast.unparse(item)
                    and isinstance(item.test, ast.Compare)
                    and ast.unparse(item.test) == "func == PacketFunction.PACKET_FUNC_BUS_SERVO"):
                return None
            return self.generic_visit(item)
    assert ast.dump(RemoveTrace().visit(current)) == ast.dump(method(old))


def test_service_unexpected_failure_clears_partial_data_and_latches(patched_sdk):
    _, helper, source, _ = patched_sdk
    gate = helper.ReadbackGate()
    def broken(request, response):
        response.state = [S(position=[500])]
        raise OSError("offline")
    owner = S(_get_bus_servo_state_legacy=broken,
        board=S(_bounded_bus_feedback=gate),
        get_logger=lambda: S(error=lambda _: None))
    result = service_wrapper(source, helper.ReadbackFault)(
        owner, S(cmd=[command(1)]), S(state=[], success=False))
    assert result.state == [] and not result.success
    assert gate.status()["error"] == "SERVICE_READ_FAILED"


def test_staging_package_copies_only_reviewed_sources_and_refuses_overwrite(
        tmp_path, audited_sources):
    from patches.jetrover_readback.prepare_sdk_stage import prepare
    vendor=tmp_path/"vendor";vendor.mkdir()
    sdk,node=audited_sources
    (vendor/"ros_robot_controller_sdk.py").write_text(sdk)
    (vendor/"ros_robot_controller_node.py").write_text(node)
    output=tmp_path/"staging"
    report=prepare(output,vendor,"a"*40,"r1-id1-review")
    assert report["prepared_only"] and not report["hardware_validated"]
    assert report["diagnostic_id"]==1 and not report["continuous_readback_supported"]
    import json
    params=json.loads((output/"maintenance.json").read_text())
    assert params["ros_robot_controller"]["ros__parameters"]=={
        "readback_maintenance":True,"readback_probe_reference":"r1-id1-review"}
    for name, expected in report["file_sha256"].items():
        assert sha256((output/name).read_bytes()).hexdigest()==expected
    assert (vendor/"ros_robot_controller_sdk.py").read_text()==sdk
    with pytest.raises(FileExistsError):prepare(output,vendor,"a"*40)
    (vendor/"ros_robot_controller_sdk.py").write_text(sdk+"\n")
    with pytest.raises(ValueError):prepare(tmp_path/"rejected",vendor,"a"*40)
    assert not (tmp_path/"rejected").exists()


def test_probe_ros_wrapper_serializes_diagnostics_not_measured_state(patched_sdk):
    import json
    _,helper,source,_=patched_sdk
    method=next(n for n in ast.walk(ast.parse(source))
        if isinstance(n,ast.FunctionDef) and n.name=="probe_id1_once")
    namespace={"json":json}
    exec(compile(ast.Module(body=[method],type_ignores=[]),"probe_wrapper.py","exec"),namespace)
    calls=[]
    owner=S(readback_probe_reference="approved-case",
        board=S(probe_bus_servo_id1_once=lambda ref:(calls.append(ref) or
            {"outer_frame_validated":True,"source_type":"unknown",
             "canonical_observation_eligible":False,"raw_pulse":500})))
    result=namespace["probe_id1_once"](owner,S(),S())
    assert result.success and calls==["approved-case"]
    assert not json.loads(result.message)["canonical_observation_eligible"]


def test_maintenance_readiness_cannot_release_a_motion_initializer(patched_sdk):
    _,_,source,_=patched_sdk
    method=next(n for n in ast.walk(ast.parse(source))
        if isinstance(n,ast.FunctionDef) and n.name=="get_node_state")
    namespace={}
    exec(compile(ast.Module(body=[method],type_ignores=[]),"readiness.py","exec"),namespace)
    assert not namespace["get_node_state"](S(readback_maintenance=True),S(),S()).success
    assert namespace["get_node_state"](S(readback_maintenance=False),S(),S()).success


def test_real_ros_sigterm_on_offline_main_has_no_motor_calls(
        tmp_path, audited_sources):
    pytest.importorskip("rclpy")
    import os,select,signal
    tree=ast.parse(audited_sources[1])
    main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=="main")
    script=tmp_path/"offline_signal.py"
    prelude='''import rclpy
from rclpy.node import Node
from types import SimpleNamespace as S
class RosRobotController(Node):
    def __init__(self,name):
        rclpy.init()
        super().__init__(name)
        self.board=S(set_motor_speed=lambda *_:print("MOTOR_CALLED",flush=True))
        print("OFFLINE_READY",flush=True)
'''
    script.write_text(prelude+ast.unparse(main)+"\nmain()\n")
    env=dict(os.environ,ROS_DOMAIN_ID="117",RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    child=subprocess.Popen([sys.executable,"-u",str(script)],stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,text=True,env=env)
    output=""
    try:
        import time
        end=time.monotonic()+8
        while time.monotonic()<end:
            ready,_,_=select.select([child.stdout],[],[],.2)
            if ready:
                line=child.stdout.readline();output+=line
                if "OFFLINE_READY" in line:break
            if child.poll() is not None:break
        assert "OFFLINE_READY" in output,output
        # Signal only the Popen child created by this test, never the real driver.
        child.send_signal(signal.SIGTERM)
        rest,_=child.communicate(timeout=5);output+=rest
        assert "MOTOR_CALLED" not in output,output
        assert "shutdown finish" in output,output
    finally:
        if child.poll() is None:
            child.kill();child.communicate()



def test_prepared_json_is_accepted_by_real_ros_parameter_parser(tmp_path):
    rclpy=pytest.importorskip('rclpy')
    from rclpy.context import Context
    from rclpy.node import Node
    import json
    config=tmp_path/'maintenance.json'
    config.write_text(json.dumps({'ros_robot_controller':{'ros__parameters':{
        'readback_maintenance':True,'readback_probe_reference':'offline-id1'}}}))
    context=Context();node=None
    try:
        rclpy.init(args=['--ros-args','--params-file',str(config)],context=context)
        node=Node('ros_robot_controller',context=context)
        node.declare_parameter('readback_maintenance',False)
        node.declare_parameter('readback_probe_reference','')
        assert node.get_parameter('readback_maintenance').value is True
        assert node.get_parameter('readback_probe_reference').value=='offline-id1'
    finally:
        if node is not None:node.destroy_node()
        if context.ok():context.shutdown()


@pytest.mark.parametrize("mode",[False,True])
def test_probe_reference_requires_maintenance_before_any_board_open(patched_sdk,mode):
    _,_,source,_=patched_sdk
    cls,calls,topics,services=fake_node_class(source,mode,"id1-review")
    if not mode:
        with pytest.raises(ValueError):cls("offline")
        assert calls==[]
    else:
        cls("offline")
        assert "~/bus_servo/probe_id1_once" in services
        assert topics==["~/set_buzzer"]
        assert all(kind not in ("pwm","motor") for kind,_ in calls)
