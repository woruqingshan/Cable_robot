"""OFFLINE CANDIDATE. No serial driver, ROS client, automatic arming or recovery."""
from dataclasses import dataclass
import math
import struct
import threading
import time

SERVO_IDS = (1, 2, 3, 4, 5, 10)


class ReadbackFault(RuntimeError):
    def __init__(self, code):
        self.code = code
        super().__init__(code)


@dataclass(frozen=True)
class PositionSample:
    servo_id: int
    raw_pulse: int
    read_started_at_ns: int
    response_received_at_ns: int
    raw_payload: bytes
    clock_domain: str = "host_monotonic"
    source_type: str = "unknown"
    device_sample_time_ns: None = None


class ReadbackGate:
    """One verified clean epoch, each (ID,position command) at most once.

    No request nonce is present in the audited protocol. Repeated keys cannot
    distinguish a fresh reply from a delayed duplicate. This intentionally is
    NOT a continuous readback implementation. Outer request echo alone does not
    prove inner servo reply association. Arming arguments are assertions
    supplied by an external reviewed procedure, not proof created by this code.
    """
    def __init__(self, transaction_lock=None, clock_ns=time.monotonic_ns):
        self.lock = transaction_lock if transaction_lock is not None else threading.Lock()
        self.condition = threading.Condition()
        self.clock_ns = clock_ns
        self.armed = False
        self.fault = None
        self.pending = None
        self.reply = None
        self.used = set()
        self.discarded = 0
        self.evidence_reference = None

    def arm(self, reference, *, clean_epoch_verified=False, exclusive_owner_verified=False,
            echo_contract_verified=False, write_bound_verified=False,
            inner_response_match_verified=False):
        with self.condition:
            if self.armed or self.fault or self.used:
                raise ReadbackFault("REARM_FORBIDDEN")
            assertions = (clean_epoch_verified, exclusive_owner_verified,
                          echo_contract_verified, write_bound_verified, inner_response_match_verified)
            if (not isinstance(reference, str) or not reference.strip() or len(reference) > 512
                    or any(value is not True for value in assertions)):
                raise ReadbackFault("ARMING_EVIDENCE_REQUIRED")
            self.evidence_reference = reference
            self.armed = True

    def fail(self, code):
        with self.condition:
            self.fault = self.fault or code
            self.condition.notify_all()

    def status(self):
        with self.condition:
            return {"state": "FAULT" if self.fault else "DISARMED" if not self.armed
                    else "PENDING" if self.pending else "READY",
                "error": self.fault, "used_keys": sorted(self.used),
                "discarded_packets": self.discarded, "evidence_reference": self.evidence_reference,
                "continuous_readback_supported": False, "hardware_validated": False}

    def receive(self, data, *, crc_checked=False):
        """Called only by the existing CRC-checked BUS_SERVO parser."""
        with self.condition:
            if self.fault:
                self.discarded += 1
                return
            if not self.armed or self.pending is None:
                self.fail("UNSOLICITED_REPLY")
                return
            now = self.clock_ns()
            servo_id, command, started, deadline = self.pending
            if now >= deadline:
                self.fail("LATE_REPLY")
                return
            if self.reply is not None:
                self.fail("DUPLICATE_REPLY")
                return
            if crc_checked is not True or not isinstance(data, (bytes, bytearray)) or len(data) != 5:
                self.fail("INVALID_PAYLOAD_OR_CRC_PROVENANCE")
                return
            actual_id, actual_command, result, pulse = struct.unpack("<BBbh", data)
            if actual_id != servo_id:
                self.fail("SERVO_ID_MISMATCH")
            elif actual_command != command:
                self.fail("COMMAND_MISMATCH")
            elif result != 0:
                self.fail("DEVICE_ERROR")
            elif not 0 <= pulse <= 1000:
                self.fail("PULSE_OUT_OF_RANGE")
            else:
                self.reply = PositionSample(actual_id, pulse, started, now, bytes(data))
                self.condition.notify_all()

    def read_position(self, servo_id, command, unpack, send, *, timeout_s=0.1,
                      write_timeout_s=None, reception_enabled=False, send_with_deadline=False):
        if type(servo_id) is not int or servo_id not in SERVO_IDS or command != 0x05 or unpack != "<BBbh":
            raise ReadbackFault("POSITION_ONLY_SCOPE")
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 5:
            raise ReadbackFault("INVALID_DEADLINE")
        started = self.clock_ns()
        deadline = started + int(timeout_s*1e9)
        # Bound lock waiting too. A contending caller must not disturb the owner.
        if not self.lock.acquire(timeout=max(0., (deadline-self.clock_ns())/1e9)):
            raise ReadbackFault("READ_LOCK_BUSY")
        try:
            with self.condition:
                if self.fault:
                    raise ReadbackFault(self.fault)
                if not self.armed:
                    raise ReadbackFault("DISARMED")
                if not reception_enabled:
                    raise ReadbackFault("RECEPTION_DISABLED")
                if (write_timeout_s is None or not math.isfinite(write_timeout_s)
                        or not 0 < write_timeout_s <= (deadline-self.clock_ns())/1e9):
                    raise ReadbackFault("BOUNDED_WRITE_UNVERIFIED")
                key = (servo_id, command)
                if key in self.used:
                    self.fail("KEY_REUSE_UNSAFE")
                    raise ReadbackFault(self.fault)
                self.used.add(key)
                self.pending = (servo_id, command, started, deadline)
                self.reply = None
            try:
                # Existing SDK buf_write only. No extra serial port or worker.
                # A finite, independently verified serial write bound is required.
                if send_with_deadline:
                    send(deadline)
                else:
                    send()
            except Exception as error:
                self.fail("SEND_FAILED")
                raise ReadbackFault("SEND_FAILED") from error
            with self.condition:
                while True:
                    if self.fault:
                        raise ReadbackFault(self.fault)
                    remaining = (deadline-self.clock_ns())/1e9
                    if remaining <= 0:
                        self.fail("READ_TIMEOUT")
                        raise ReadbackFault(self.fault)
                    if self.reply is not None:
                        return self.reply
                    self.condition.wait(timeout=remaining)
        finally:
            with self.condition:
                self.pending = None
                self.reply = None
            self.lock.release()


def claim_linux_exclusive(port):
    """Claim the EXISTING fd only; never opens a second serial device.

    TIOCEXCL rejects later ordinary opens, but does not revoke older descriptors
    and privileged callers can bypass it. Owner preflight is still mandatory.
    """
    import fcntl
    import termios
    fcntl.ioctl(port.fileno(), termios.TIOCEXCL)


class MaintenanceFrameWriter:
    """Bounded opt-in maintenance TX using the one existing PySerial instance.

    Only buzzer and position-query frames are permitted. No reset/retry exists.
    Software deadlines are checked; this is not a hard-real-time guarantee.
    """
    def __init__(self, port, crc_function, *, clock_ns=time.monotonic_ns):
        self.port, self.crc_function, self.clock_ns = port, crc_function, clock_ns
        self.lock = threading.Lock()
        self.fault = None

    def write(self, frame, *, deadline_ns=None):
        raw = bytes(frame)
        if (len(raw) < 5 or raw[:2] != b"\xaa\x55"
                or raw[3] != len(raw)-5 or self.crc_function(raw[2:-1]) != raw[-1]):
            raise ReadbackFault("MAINTENANCE_INVALID_TX_FRAME")
        function, payload = raw[2], raw[4:-1]
        allowed = (function == 2 and len(payload) == 8) or (
            function == 5 and len(payload) == 2 and payload[0] == 5 and payload[1] in SERVO_IDS)
        if not allowed:
            raise ReadbackFault("MAINTENANCE_COMMAND_FORBIDDEN")
        timeout = self.port.write_timeout
        if (type(timeout) not in (int, float) or not math.isfinite(timeout)
                or not 0 < timeout <= .1):
            raise ReadbackFault("MAINTENANCE_FINITE_WRITE_REQUIRED")
        now = self.clock_ns()
        if deadline_ns is None:
            deadline_ns = now + 100_000_000
        if type(deadline_ns) is not int:
            raise ReadbackFault("MAINTENANCE_INVALID_TX_DEADLINE")
        if self.fault:
            raise ReadbackFault(self.fault)
        lock_budget = (deadline_ns-now)/1e9-timeout
        if lock_budget <= 0 or not self.lock.acquire(timeout=lock_budget):
            raise ReadbackFault("MAINTENANCE_TX_LOCK_BUDGET")
        try:
            if self.fault:
                raise ReadbackFault(self.fault)
            if (deadline_ns-self.clock_ns())/1e9 < timeout:
                raise ReadbackFault("MAINTENANCE_TX_BUDGET_EXHAUSTED")
            try:
                written = self.port.write(raw)
            except Exception as error:
                self.fault = "MAINTENANCE_TX_IO_FAILED"
                raise ReadbackFault(self.fault) from error
            if type(written) is not int or written != len(raw):
                self.fault = "MAINTENANCE_TX_SHORT_WRITE"
                raise ReadbackFault(self.fault)
            if self.clock_ns() >= deadline_ns:
                self.fault = "MAINTENANCE_TX_DEADLINE_OVERRUN"
                raise ReadbackFault(self.fault)
            return written
        finally:
            self.lock.release()
