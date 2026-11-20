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
    NOT a continuous readback implementation. Arming arguments are assertions
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
            echo_contract_verified=False, write_bound_verified=False):
        with self.condition:
            if self.armed or self.fault or self.used:
                raise ReadbackFault("REARM_FORBIDDEN")
            assertions = (clean_epoch_verified, exclusive_owner_verified,
                          echo_contract_verified, write_bound_verified)
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
                      write_timeout_s=None, reception_enabled=False):
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
