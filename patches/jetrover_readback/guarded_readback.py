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
        self.diagnostic_only = False
        self.diagnostic_ids = (1,)
        self.diagnostic_attempt_limit = None
        self.diagnostic_attempts = 0
        self.diagnostic_repeat_budget = 0

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

    def arm_diagnostic_once(self, reference):
        """Operator-approved ID1 capture, NOT proof of a fresh hardware sample."""
        with self.condition:
            if self.armed or self.fault or self.used:
                raise ReadbackFault("REARM_FORBIDDEN")
            if not isinstance(reference, str) or not reference.strip() or len(reference) > 512:
                raise ReadbackFault("DIAGNOSTIC_REFERENCE_REQUIRED")
            self.evidence_reference = reference
            self.diagnostic_only = True
            self.armed = True

    def fail(self, code):
        with self.condition:
            self.fault = self.fault or code
            self.condition.notify_all()

    def status(self):
        with self.condition:
            return {"state": "FAULT" if self.fault else "DISARMED" if not self.armed
                    else "PENDING" if self.pending else
                    "COMPLETE_UNVERIFIED" if self.diagnostic_only and self.used else "READY",
                "error": self.fault, "used_keys": sorted(self.used),
                "discarded_packets": self.discarded, "evidence_reference": self.evidence_reference,
                "continuous_readback_supported": False, "hardware_validated": False,
                "diagnostic_only": self.diagnostic_only,
                "canonical_observation_eligible": False}

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

    def _repeat_diagnostic_allowed(self):
        return self.diagnostic_repeat_budget > 0

    def read_position(self, servo_id, command, unpack, send, *, timeout_s=0.1,
                      write_timeout_s=None, reception_enabled=False, send_with_deadline=False, diagnostic=False):
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
                if self.diagnostic_only != diagnostic:
                    raise ReadbackFault("DIAGNOSTIC_PATH_MISMATCH")
                if diagnostic and servo_id not in self.diagnostic_ids:
                    raise ReadbackFault("DIAGNOSTIC_ID1_ONLY")
                if not reception_enabled:
                    raise ReadbackFault("RECEPTION_DISABLED")
                if (write_timeout_s is None or not math.isfinite(write_timeout_s)
                        or not 0 < write_timeout_s <= (deadline-self.clock_ns())/1e9):
                    raise ReadbackFault("BOUNDED_WRITE_UNVERIFIED")
                key = (servo_id, command)
                if diagnostic and self.diagnostic_attempt_limit is not None:
                    if self.diagnostic_attempts >= self.diagnostic_attempt_limit:
                        self.fail("DIAGNOSTIC_BUDGET_EXHAUSTED")
                        raise ReadbackFault(self.fault)
                if key in self.used and not (diagnostic and self._repeat_diagnostic_allowed()):
                    self.fail("KEY_REUSE_UNSAFE")
                    raise ReadbackFault(self.fault)
                if diagnostic:
                    self.diagnostic_attempts += 1
                    if key in self.used and self.diagnostic_repeat_budget > 0:
                        self.diagnostic_repeat_budget -= 1
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


class PositionCapture:
    """Shared single-SDK capture; no scheduling or permission policy."""
    def _capture(self, servo_id):
        self.board._last_bus_tx_frame = self.board._last_bus_rx_frame = None
        row = dict(servo_id=servo_id, raw_pulse=None, source_type="unknown",
            inner_association_verified=False, canonical_observation_eligible=False,
            device_sample_time_ns=None, error=None)
        started = self.clock_ns()
        try:
            self.board.bus_servo_read_and_unpack(servo_id, 5, "<BBbh", diagnostic=True)
            sample = self.board.last_bus_position_sample
            row.update(raw_pulse=sample.raw_pulse,
                read_started_at_ns=sample.read_started_at_ns,
                response_received_at_ns=sample.response_received_at_ns,
                read_latency_ns=sample.response_received_at_ns-sample.read_started_at_ns,
                raw_payload_hex=sample.raw_payload.hex(), clock_domain=sample.clock_domain,
                outer_frame_validated=True)
        except Exception as error:
            self.board._bounded_bus_feedback.fail(
                error.code if isinstance(error, ReadbackFault) else "DIAGNOSTIC_CAPTURE_FAILED")
            row.update(error=self.board._bounded_bus_feedback.fault, outer_frame_validated=False)
        row["capture_elapsed_ns"] = self.clock_ns()-started
        for key, attr in (("tx_frame_hex", "_last_bus_tx_frame"), ("rx_frame_hex", "_last_bus_rx_frame")):
            value = getattr(self.board, attr, None)
            row[key] = value.hex() if value is not None else None
        return row

    def _batch(self, ids, sequence):
        started = self.clock_ns()
        rows = []
        for servo_id in ids:
            row = self._capture(servo_id)
            rows.append(row)
            if row["error"]:
                break
        report = dict(sequence=sequence, channels=rows,
            complete=len(rows)==len(ids) and all(r["error"] is None for r in rows),
            batch_elapsed_ns=self.clock_ns()-started, source_type="unknown",
            inner_association_verified=False, canonical_observation_eligible=False,
            physical_calibrated=False, device_sample_span_ns=None)
        if report["complete"]:
            report["host_transaction_span_ns"] = (
                rows[-1]["response_received_at_ns"]-rows[0]["read_started_at_ns"])
            report["host_span_exceeds_20ms"] = report["host_transaction_span_ns"] > 20_000_000
            if report["host_transaction_span_ns"] > 500_000_000:
                self.board._bounded_bus_feedback.fail("DIAGNOSTIC_BATCH_HOST_STALE")
                report["complete"] = False
        else:
            report["host_transaction_span_ns"] = None
        if self.on_cycle:
            try:
                self.on_cycle(report)
            except Exception:
                self.board._bounded_bus_feedback.fail("DIAGNOSTIC_RECORD_CALLBACK_FAILED")
                report["complete"] = False
        if self.board._bounded_bus_feedback.fault:
            report["complete"] = False
        return report


class BoundedDiagnosticSession(PositionCapture):
    """Finite unverified captures. No recovery, autonomous polling or new port.

    Repeated diagnostic replies have UNPROVEN transaction identity. They MUST NOT
    be promoted to trusted feedback, regardless of apparent stability or rate.
    """
    def __init__(self, board, plan, *, clock_ns=time.monotonic_ns, sleep=time.sleep, on_cycle=None):
        self.validate_plan(plan)
        self.board, self.plan = board, dict(plan)
        self.clock_ns, self.sleep, self.on_cycle = clock_ns, sleep, on_cycle
        self.phase = "PREPARED"
        self.operation_lock = threading.Lock()

    @staticmethod
    def validate_plan(plan):
        if (not isinstance(plan, dict) or plan.get("schema_version") != 1
                or plan.get("prior_used_ids") != [1]
                or plan.get("d2_ids") != [2, 3, 4, 5, 10]
                or type(plan.get("d3_cycles")) is not int or not 1 <= plan["d3_cycles"] <= 3
                or type(plan.get("cycle_interval_s")) not in (int, float)
                or not 1 <= plan["cycle_interval_s"] <= 5
                or not isinstance(plan.get("reference"), str) or not plan["reference"].strip()
                or len(plan["reference"]) > 512
                or not isinstance(plan.get("prior_evidence_sha256"), str)
                or len(plan["prior_evidence_sha256"]) != 64
                or any(c not in "0123456789abcdef" for c in plan["prior_evidence_sha256"])
                or not isinstance(plan.get("source_sha"), str) or len(plan["source_sha"]) != 40
                or any(c not in "0123456789abcdef" for c in plan["source_sha"])):
            raise ValueError("INVALID_FINITE_DIAGNOSTIC_PLAN")


    def run(self, stage):
        if not self.operation_lock.acquire(blocking=False):
            return {"complete":False,"error":"DIAGNOSTIC_OPERATION_BUSY","batches":[]}
        try:
            gate = self.board._bounded_bus_feedback
            if stage == "D2":
                if self.phase != "PREPARED":
                    return {"complete":False,"error":"D2_ALREADY_CONSUMED","batches":[]}
                self.phase = "D2_RUNNING"
                gate.arm_diagnostic_once(self.plan["reference"])
                # History is carried forward explicitly; restart is NOT a clean epoch.
                gate.diagnostic_ids = SERVO_IDS
                gate.used.update((i,5) for i in self.plan["prior_used_ids"])
                gate.diagnostic_attempt_limit = 5
                batches = [self._batch(self.plan["d2_ids"], 0)]
                self.phase = "D2_COMPLETE" if batches[0]["complete"] else "FAULT"
            elif stage == "D3":
                if self.phase != "D2_COMPLETE" or gate.fault:
                    return {"complete":False,"error":"D2_NOT_CLEANLY_COMPLETE","batches":[]}
                self.phase = "D3_RUNNING"
                # Only explicitly untrusted finite experiments may reuse keys.
                gate.diagnostic_attempt_limit = 5+6*self.plan["d3_cycles"]
                gate.diagnostic_repeat_budget = 6*self.plan["d3_cycles"]
                batches = []
                began = self.clock_ns()
                for cycle in range(self.plan["d3_cycles"]):
                    target = began+int(cycle*self.plan["cycle_interval_s"]*1e9)
                    if target > self.clock_ns():
                        self.sleep((target-self.clock_ns())/1e9)
                    batch = self._batch(SERVO_IDS, cycle+1)
                    batches.append(batch)
                    if not batch["complete"]:
                        break
                self.phase = "COMPLETE_UNVERIFIED" if all(b["complete"] for b in batches) else "FAULT"
            else:
                return {"complete":False,"error":"UNKNOWN_DIAGNOSTIC_STAGE","batches":[]}
            complete = all(b["complete"] for b in batches) and gate.fault is None
            if not complete:
                self.phase = "FAULT"
            return dict(stage=stage, complete=complete,
                phase=self.phase, error=gate.fault, batches=batches,
                source_type="unknown", canonical_observation_eligible=False,
                association_guarantee="UNPROVEN_REPEATED_RESPONSES",
                prior_used_ids=self.plan["prior_used_ids"], gate_status=gate.status())
        except Exception as error:
            self.phase = "FAULT"
            self.board._bounded_bus_feedback.fail(
                error.code if isinstance(error, ReadbackFault) else "DIAGNOSTIC_SESSION_FAILED")
            return dict(complete=False,error=self.board._bounded_bus_feedback.fault,batches=[])
        finally:
            self.operation_lock.release()


def initialize_existing_serial(port, device, *, clock_ns=time.monotonic_ns):
    """Configure ONE caller-owned Serial instance; keep DTR/RTS order unchanged.

    Serial read/write timeouts do not bound open/modem-control ioctls. No worker,
    abandoned operation, retry, reset or ignored ioctl exception is introduced.
    """
    import errno
    import json
    import sys
    if getattr(port, "is_open", False) or getattr(port, "fd", None) is not None:
        raise ValueError("SERIAL_INITIALIZATION_REQUIRES_CLOSED_INSTANCE")
    started = clock_ns()
    report = dict(device=str(device), stage="configure_rts", dtr=False, rts=False,
                  open_attempts=0, success=False)
    try:
        port.rts = False
        report["stage"] = "configure_dtr"
        port.dtr = False
        report["stage"] = "set_device"
        port.setPort(device)
        report["stage"] = "open"
        report["open_attempts"] = 1
        port.open()
        report.update(stage="open_complete", success=True)
        return report
    except BaseException as error:
        names = []
        tb = error.__traceback__
        while tb is not None:
            names.append(tb.tb_frame.f_code.co_name)
            tb = tb.tb_next
        code = "SERIAL_INITIALIZATION_FAILED"
        if isinstance(error, OSError) and error.errno == errno.ETIMEDOUT:
            code = ("USB_DTR_CONTROL_TIMEOUT" if "_update_dtr_state" in names
                    else "USB_RTS_CONTROL_TIMEOUT" if "_update_rts_state" in names
                    else "SERIAL_OPEN_CONTROL_TIMEOUT")
        elif isinstance(error, (KeyboardInterrupt, SystemExit)):
            code = "SERIAL_INITIALIZATION_INTERRUPTED"
        report.update(error_code=code, errno=getattr(error,"errno",None),
                      exception_type=type(error).__name__, traceback_functions=names)
        # PySerial.open already closes its raw FD on failure. Avoid a second close
        # when is_open=False; never close an FD number independently of its owner.
        if getattr(port, "is_open", False):
            try:
                port.close()
                report["cleanup"] = "closed_open_port_once"
            except Exception as cleanup_error:
                report["cleanup"] = "close_failed_no_retry"
                report["cleanup_exception_type"] = type(cleanup_error).__name__
        else:
            report["cleanup"] = "not_open_no_duplicate_close"
        report["fd_cleared"] = getattr(port, "fd", None) is None
        report["elapsed_ns"] = clock_ns()-started
        error.serial_initialization_report = report
        print(json.dumps({"event":"serial_initialization_failed","report":report}),
              file=sys.stderr, flush=True)
        raise
    finally:
        report["elapsed_ns"] = clock_ns()-started


def close_existing_serial_once(port):
    """Close only an open caller-owned port. Does not flush, reset, or toggle."""
    if not getattr(port, "is_open", False):
        return {"port_closed":getattr(port,"fd",None) is None,
                "close_called":False,"status":"already_closed"}
    port.close()
    return {"port_closed":not port.is_open and getattr(port,"fd",None) is None,
            "close_called":True,"status":"closed"}
