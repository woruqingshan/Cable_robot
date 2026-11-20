"""Offline fixtures only: no serial device, ROS graph, SDK process or hardware RPC."""
import struct
import threading
import time

import pytest
from patches.jetrover_readback.guarded_readback import ReadbackGate, ReadbackFault


def armed():
    gate = ReadbackGate()
    gate.arm("OFFLINE_FIXTURE_ONLY", clean_epoch_verified=True, exclusive_owner_verified=True,
        echo_contract_verified=True, write_bound_verified=True,inner_response_match_verified=True)
    return gate


def packet(servo_id=1, command=5, result=0, pulse=500):
    return struct.pack("<BBbh", servo_id, command, result, pulse)


def read(gate, servo_id=1, send=None, **kwargs):
    return gate.read_position(servo_id,5,"<BBbh",send or (lambda:None),
        timeout_s=kwargs.pop("timeout_s",.03),write_timeout_s=kwargs.pop("write_timeout_s",.005),
        reception_enabled=kwargs.pop("reception_enabled",True),**kwargs)


def assert_fault(code, function):
    with pytest.raises(ReadbackFault) as error:
        function()
    assert error.value.code == code


def test_disarmed_and_review_assertions_do_not_send():
    gate=ReadbackGate();sent=[]
    assert_fault("DISARMED",lambda:read(gate,send=lambda:sent.append(1)))
    assert_fault("ARMING_EVIDENCE_REQUIRED",lambda:gate.arm("not evidence"))
    assert sent==[]


def test_normal_reply_preserves_raw_position_and_host_time_only():
    gate=armed()
    sample=read(gate,send=lambda:gate.receive(packet(),crc_checked=True))
    assert sample.servo_id==1 and sample.raw_pulse==500
    assert sample.read_started_at_ns<=sample.response_received_at_ns
    assert sample.device_sample_time_ns is None and sample.clock_domain=="host_monotonic"
    assert sample.source_type=="unknown"
    assert sample.raw_payload==packet()
    assert gate.lock.acquire(blocking=False);gate.lock.release()


def test_timeout_releases_lock_and_latches_no_retry_or_stale_consumption():
    gate=armed();sent=[];start=time.monotonic()
    assert_fault("READ_TIMEOUT",lambda:read(gate,send=lambda:sent.append(1)))
    assert time.monotonic()-start < .4
    assert gate.lock.acquire(blocking=False);gate.lock.release()
    gate.receive(packet(),crc_checked=True)
    assert_fault("READ_TIMEOUT",lambda:read(gate,servo_id=2,send=lambda:sent.append(2)))
    assert sent==[1] and gate.status()["discarded_packets"]==1
    assert_fault("REARM_FORBIDDEN",lambda:gate.arm("new assertion"))


@pytest.mark.parametrize("payload,code",[
    (b"bad","INVALID_PAYLOAD_OR_CRC_PROVENANCE"),
    (packet(servo_id=2),"SERVO_ID_MISMATCH"),
    (packet(command=6),"COMMAND_MISMATCH"),
    (packet(result=1),"DEVICE_ERROR"),
    (packet(pulse=-1),"PULSE_OUT_OF_RANGE"),
    (packet(pulse=1001),"PULSE_OUT_OF_RANGE")])
def test_wrong_reply_never_returns_state(payload,code):
    gate=armed()
    assert_fault(code,lambda:read(gate,send=lambda:gate.receive(payload,crc_checked=True)))
    assert gate.status()["state"]=="FAULT"


def test_crc_provenance_required():
    gate=armed()
    assert_fault("INVALID_PAYLOAD_OR_CRC_PROVENANCE",lambda:read(gate,send=lambda:gate.receive(packet())))


def test_duplicate_during_pending_request_faults():
    gate=armed()
    def send():
        gate.receive(packet(),crc_checked=True);gate.receive(packet(),crc_checked=True)
    assert_fault("DUPLICATE_REPLY",lambda:read(gate,send=send))


def test_same_key_is_never_reused_even_after_success():
    gate=armed();sent=[]
    read(gate,send=lambda:gate.receive(packet(),crc_checked=True))
    assert_fault("KEY_REUSE_UNSAFE",lambda:read(gate,send=lambda:sent.append(1)))
    assert sent==[] and gate.status()["state"]=="FAULT"


def test_late_old_reply_cannot_satisfy_next_different_id():
    gate=armed()
    read(gate,send=lambda:gate.receive(packet(),crc_checked=True))
    assert_fault("SERVO_ID_MISMATCH",lambda:read(gate,servo_id=2,
        send=lambda:gate.receive(packet(1),crc_checked=True)))


def test_unsolicited_packet_invalidates_epoch():
    gate=armed();gate.receive(packet(),crc_checked=True)
    assert_fault("UNSOLICITED_REPLY",lambda:read(gate))


def test_all_five_arm_ids_plus_independent_gripper_sequentially_once():
    gate=armed();samples=[]
    for servo_id in [1,2,3,4,5,10]:
        samples.append(read(gate,servo_id=servo_id,send=lambda i=servo_id:
            gate.receive(packet(i,pulse=400+i),crc_checked=True)))
    assert [s.servo_id for s in samples]==[1,2,3,4,5,10]
    assert [s.raw_pulse for s in samples]==[401,402,403,404,405,410]
    assert len(gate.used)==6


@pytest.mark.parametrize("write_timeout",[None,0.,float("inf"),1.])
def test_unbounded_or_excessive_write_timeout_refuses_before_send(write_timeout):
    gate=armed();sent=[]
    assert_fault("BOUNDED_WRITE_UNVERIFIED",lambda:read(gate,send=lambda:sent.append(1),write_timeout_s=write_timeout))
    assert sent==[]


def test_transport_failure_latches_without_background_sender():
    gate=armed()
    def send():raise TimeoutError("offline serial write timeout")
    assert_fault("SEND_FAILED",lambda:read(gate,send=send))
    assert gate.lock.acquire(blocking=False);gate.lock.release()


def test_lock_wait_is_bounded_without_disturbing_owner():
    gate=armed();gate.lock.acquire()
    try:assert_fault("READ_LOCK_BUSY",lambda:read(gate))
    finally:gate.lock.release()
    assert gate.status()["state"]=="READY"


def test_receiver_failure_wakes_pending_wait_and_releases_lock():
    gate=armed();started=threading.Event();errors=[]
    def worker():
        try:read(gate,send=started.set,timeout_s=.5)
        except ReadbackFault as error:errors.append(error.code)
    thread=threading.Thread(target=worker);thread.start()
    assert started.wait(.5)
    gate.fail("RECEIVER_FAILED");thread.join(.5)
    assert not thread.is_alive() and errors==["RECEIVER_FAILED"]
    assert gate.lock.acquire(blocking=False);gate.lock.release()


def test_reply_after_deadline_never_returns_even_if_send_completes():
    gate=armed();ticks=[0];gate.clock_ns=lambda:ticks[0]
    def send():
        ticks[0]=40_000_000;gate.receive(packet(),crc_checked=True)
    assert_fault("LATE_REPLY",lambda:read(gate,send=send))


@pytest.mark.parametrize("servo_id",[0,6,9,254,True])
def test_no_sixth_arm_or_broadcast_id(servo_id):
    assert_fault("POSITION_ONLY_SCOPE",lambda:read(armed(),servo_id=servo_id))


def test_outer_request_echo_is_insufficient_without_inner_response_matching_evidence():
    gate=ReadbackGate()
    assert_fault("ARMING_EVIDENCE_REQUIRED",lambda:gate.arm("OFFLINE_OUTER_ECHO_ONLY",
        clean_epoch_verified=True,exclusive_owner_verified=True,
        echo_contract_verified=True,write_bound_verified=True))
    assert gate.status()["state"]=="DISARMED"
