"""Offline only: fake serial instances and Linux PTYs, never /dev/rrc."""
import os
from types import SimpleNamespace as S
import sys

import pytest

from patches.jetrover_readback.guarded_readback import (
    MaintenanceFrameWriter, ReadbackFault, claim_linux_exclusive)


def crc(data):
    return sum(data) % 256


def frame(function=5, payload=b"\x05\x01"):
    body = bytes([function, len(payload)]) + payload
    return b"\xaa\x55" + body + bytes([crc(body)])


class Port:
    write_timeout = .005

    def __init__(self):
        self.frames = []
        self.result = None
        self.error = None

    def write(self, raw):
        self.frames.append(bytes(raw))
        if self.error:
            raise self.error
        return len(raw) if self.result is None else self.result


def writer():
    port = Port()
    return port, MaintenanceFrameWriter(port, crc)


def fault(code, call):
    with pytest.raises(ReadbackFault) as error:
        call()
    assert error.value.code == code


def test_position_and_button_buzzer_use_same_complete_writer():
    port, tx = writer()
    for raw in (frame(), frame(2, bytes(8))):
        assert tx.write(raw) == len(raw)
    assert port.frames == [frame(), frame(2, bytes(8))]


@pytest.mark.parametrize("function,payload", [
    (3, bytes(8)), (4, bytes(8)), (5, b"\x01\x01"),
    (5, b"\x0b\x01"), (5, b"\x05\x06"), (1, bytes(8)), (10, bytes(8))])
def test_actuator_torque_and_other_commands_rejected_before_write(function, payload):
    port, tx = writer()
    fault("MAINTENANCE_COMMAND_FORBIDDEN",
          lambda: tx.write(frame(function, payload)))
    assert port.frames == []


@pytest.mark.parametrize("raw", [
    b"", b"\xaa\x55\x05\x02\x05\x01\x00",
    b"\xaa\x55\x05\x03\x05\x01\x0d",
    b"\x00\x55\x05\x02\x05\x01\x0d"])
def test_bad_frame_length_header_and_crc_never_reach_port(raw):
    port, tx = writer()
    fault("MAINTENANCE_INVALID_TX_FRAME", lambda: tx.write(raw))
    assert port.frames == []


@pytest.mark.parametrize("timeout", [None, 0, True, float("nan"), float("inf"), .2])
def test_finite_timeout_is_required_without_changing_port(timeout):
    port, tx = writer()
    port.write_timeout = timeout
    fault("MAINTENANCE_FINITE_WRITE_REQUIRED", lambda: tx.write(frame()))
    assert port.frames == []
    assert port.write_timeout is timeout


@pytest.mark.parametrize("count", [0, 1, True, 100])
def test_short_or_invalid_write_latches_and_releases_lock(count):
    port, tx = writer()
    port.result = count
    fault("MAINTENANCE_TX_SHORT_WRITE", lambda: tx.write(frame()))
    fault("MAINTENANCE_TX_SHORT_WRITE", lambda: tx.write(frame()))
    assert len(port.frames) == 1
    assert tx.lock.acquire(blocking=False)
    tx.lock.release()


def test_io_exception_latches_without_retry():
    port, tx = writer()
    port.error = OSError("offline")
    fault("MAINTENANCE_TX_IO_FAILED", lambda: tx.write(frame()))
    fault("MAINTENANCE_TX_IO_FAILED", lambda: tx.write(frame(2, bytes(8))))
    assert len(port.frames) == 1
    assert tx.lock.acquire(blocking=False)
    tx.lock.release()


def test_lock_wait_and_insufficient_budget_are_bounded():
    port, tx = writer()
    tx.lock.acquire()
    try:
        fault("MAINTENANCE_TX_LOCK_BUDGET", lambda: tx.write(frame()))
    finally:
        tx.lock.release()
    fault("MAINTENANCE_TX_LOCK_BUDGET",
          lambda: tx.write(frame(), deadline_ns=tx.clock_ns()+1))
    assert port.frames == []


def test_write_completion_after_deadline_is_not_accepted():
    port, tx = writer()
    ticks = [0]
    tx.clock_ns = lambda: ticks[0]
    def late(raw):
        ticks[0] = 100_000_001
        port.frames.append(raw)
        return len(raw)
    port.write = late
    fault("MAINTENANCE_TX_DEADLINE_OVERRUN", lambda: tx.write(frame()))
    assert len(port.frames) == 1


def test_position_gate_forwards_its_original_deadline():
    from patches.jetrover_readback.guarded_readback import ReadbackGate
    import struct
    gate = ReadbackGate()
    gate.arm("OFFLINE_FIXTURE", clean_epoch_verified=True,
        exclusive_owner_verified=True, echo_contract_verified=True,
        write_bound_verified=True, inner_response_match_verified=True)
    seen = []
    def send(deadline):
        seen.append(deadline)
        gate.receive(struct.pack("<BBbh", 1, 5, 0, 500), crc_checked=True)
    sample = gate.read_position(1, 5, "<BBbh", send, timeout_s=.1,
        write_timeout_s=.005, reception_enabled=True, send_with_deadline=True)
    assert seen == [sample.read_started_at_ns+100_000_000]


@pytest.mark.skipif(sys.platform != "linux", reason="Linux PTY ioctl test")
def test_linux_exclusive_on_virtual_pty_only():
    import errno
    import pty
    master, slave = pty.openpty()
    extra = None
    try:
        path = os.ttyname(slave)
        assert path.startswith("/dev/pts/")
        claim_linux_exclusive(S(fileno=lambda: slave))
        try:
            extra = os.open(path, os.O_RDWR | os.O_NOCTTY)
        except OSError as error:
            assert error.errno == errno.EBUSY
        else:
            # CAP_SYS_ADMIN can bypass TIOCEXCL; do not claim enforcement then.
            pytest.fail("this process bypassed TIOCEXCL on the virtual PTY")
    finally:
        if extra is not None:
            os.close(extra)
        os.close(slave)
        os.close(master)
