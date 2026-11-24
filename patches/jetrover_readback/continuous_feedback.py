"""Uncalibrated experimental repetition on the existing SDK, never a trusted epoch."""
import copy
import math
import queue
import threading
import time
import struct
from .guarded_readback import ReadbackGate, ReadbackFault, PositionCapture, SERVO_IDS


class CompletedReplyError(ReadbackFault):
    completed_transaction = True


class ExperimentalReadbackGate(ReadbackGate):
    """Explicit independent permission path. Any ambiguous communication error latches."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.terminal_statuses = set()
        self.contract_reference = None
        self.last_rejection = None

    def configure_terminal_statuses(self, statuses, contract_reference):
        if self.armed or any(type(x) is not int or x==0 or not -128<=x<=127 for x in statuses):
            raise ValueError('INVALID_TERMINAL_STATUS_POLICY')
        if statuses and not contract_reference:raise ValueError('TERMINAL_CONTRACT_REQUIRED')
        self.terminal_statuses=set(statuses);self.contract_reference=contract_reference

    def _reject_completed(self,code,data):
        i,cmd,status,pulse=struct.unpack('<BBbh',data)
        recoverable=(code=='PULSE_OUT_OF_RANGE' or status in self.terminal_statuses)
        self.last_rejection=dict(servo_id=i,command=cmd,status=status,raw_pulse=pulse,
            payload_hex=bytes(data).hex(),received_monotonic_ns=self.clock_ns(),crc_checked=True,
            completed_outer_reply=True,terminal_contract=self.contract_reference,
            decision='INVALID_GROUP_CONTINUE' if recoverable else 'PAUSE_UNKNOWN_TERMINATION')
        if recoverable:
            self.transaction_error=CompletedReplyError(code);self.condition.notify_all()
        else:self.fail(code)

    def arm_experiment(self, reference):
        with self.condition:
            if self.armed or self.fault or self.used:
                raise ReadbackFault('REARM_FORBIDDEN')
            if not isinstance(reference, str) or not reference.strip() or len(reference)>512:
                raise ReadbackFault('EXPERIMENT_REFERENCE_REQUIRED')
            self.evidence_reference = reference
            self.diagnostic_only = True
            self.diagnostic_ids = SERVO_IDS
            self.armed = True

    def arm(self, *args, **kwargs):
        raise ReadbackFault('EXPERIMENT_CANNOT_BECOME_TRUSTED')

    def _repeat_diagnostic_allowed(self):
        return self.armed and self.diagnostic_only and not self.fault

    def status(self):
        result = super().status()
        result.update(experimental_continuous_configured=self.armed,
            experimental_continuous_enabled=self.armed and not self.fault,
            continuous_readback_supported=True, trusted_continuous_feedback=False,
            inner_association_verified=False)
        return result


class ContinuousFeedbackCollector(PositionCapture):
    """Single sequential bus worker; fixed-rate schedule drops backlog, never bursts.

    Recording and ROS consumers receive nonblocking copies. Their failures do not
    recover/rearm the communication gate. No total query/runtime cap is imposed.
    """
    def __init__(self, board, *, hz=20., clock_ns=time.monotonic_ns):
        if type(hz) not in (int,float) or not math.isfinite(hz) or not 0<hz<=50:
            raise ValueError('INVALID_FEEDBACK_RATE')
        if not isinstance(board._bounded_bus_feedback, ExperimentalReadbackGate):
            raise ValueError('EXPERIMENT_GATE_REQUIRED')
        self.board, self.clock_ns = board, clock_ns
        self.on_cycle = None
        self.period_ns = int(1e9/hz)
        self.hz = hz
        self.stop_event = threading.Event()
        self.thread = None
        self.commands = queue.PriorityQueue(maxsize=16)
        self.output = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.next_tick = None
        self.on_record = None
        self.on_fault = None
        self.paused = False
        self.consecutive_errors = 0
        self.max_consecutive_errors = 3
        self.stats = dict(cycles=0, skipped_periods=0, observer_drops=0,
            observer_errors=0, record_errors=0, command_errors=0)
        self._command_sequence = 0
        self.last_stop_result = None

    def submit(self, callback, *, stop=False):
        """Only the reviewed driver supplies callbacks; bus I/O never runs in ROS callbacks."""
        if stop:
            original=callback
            def callback():
                try:
                    result=original()
                    self.last_stop_result=dict(status='WRITE_COMPLETED_NOT_ACKNOWLEDGED',delivery_guaranteed=False)
                    return result
                except Exception as error:
                    self.last_stop_result=dict(status='FAILED_DELIVERY_NOT_GUARANTEED',error=str(error),delivery_guaranteed=False)
                    raise
        if stop and self.stop_event.is_set():
            lock=self.board._bounded_bus_feedback.lock
            if not lock.acquire(timeout=.1):raise ReadbackFault('STOP_LOCK_TIMEOUT')
            try:return callback()
            finally:lock.release()
        if self.stop_event.is_set() or (self.board._bounded_bus_feedback.fault and not stop):
            raise ReadbackFault('FEEDBACK_UNAVAILABLE')
        with self.lock:
            self._command_sequence += 1
            ticket = self._command_sequence
        self.commands.put_nowait((0 if stop else 1, ticket, callback))
        return ticket

    def _capture(self, servo_id):
        self._dispatch()
        return super()._capture(servo_id)

    def _dispatch(self):
        # Bound service time: at most one command at each transaction boundary.
        try:
            _, _, callback = self.commands.get_nowait()
        except queue.Empty:
            return
        try:
            with self.board._bounded_bus_feedback.lock:
                callback()
        except Exception:
            self.stats['command_errors'] += 1
            self.board._bounded_bus_feedback.fail('COMMAND_DISPATCH_FAILED')
        finally:
            self.commands.task_done()

    def snapshot(self):
        with self.lock:
            sample = copy.deepcopy(self.latest)
        fault = self.board._bounded_bus_feedback.fault
        now = self.clock_ns()
        if (fault or not sample or not sample['complete']
                or now-sample['channels'][0]['read_started_at_ns']>500_000_000
                or now<sample['channels'][-1]['response_received_at_ns']):
            return dict(valid=False, error=fault or 'STALE_OR_MISSING_FEEDBACK',
                        last_fault_report=sample if fault else None,
                        source_type='unknown', canonical_observation_eligible=False)
        return dict(valid=True, report=sample, source_type='unknown',
                    canonical_observation_eligible=False)

    def tick(self):
        if self.paused:
            self._dispatch()
            return None
        now = self.clock_ns()
        if self.next_tick is None:
            self.next_tick = now
        if now < self.next_tick or self.stop_event.is_set():
            return None
        report = self._batch(SERVO_IDS, self.sequence)
        report.update(schedule_started_ns=now, scheduled_ns=self.next_tick,
            schedule_lateness_ns=max(0,now-self.next_tick), requested_hz=self.hz)
        self.sequence += 1
        self.stats['cycles'] += 1
        self.next_tick += self.period_ns
        finished = self.clock_ns()
        skipped = 0
        if finished > self.next_tick:
            skipped = (finished-self.next_tick+self.period_ns-1)//self.period_ns
            self.next_tick += skipped*self.period_ns
        self.stats['skipped_periods'] += skipped
        report['skipped_periods'] = skipped
        with self.lock:
            self.latest = copy.deepcopy(report)
        try:
            self.output.put_nowait(report)
        except queue.Full:
            try:self.output.get_nowait()
            except queue.Empty:pass
            self.output.put_nowait(report)
            self.stats['observer_drops'] += 1
        if self.on_record:
            try:
                self.on_record(copy.deepcopy(report))
            except Exception:
                self.stats['record_errors'] += 1
        if not report['complete'] or self.board._bounded_bus_feedback.fault:
            self.consecutive_errors += 1
            if self.consecutive_errors >= self.max_consecutive_errors:
                self.board._bounded_bus_feedback.fail('REPEATED_COMPLETED_ERRORS')
            self.paused = bool(self.board._bounded_bus_feedback.fault)
            report['recovery_decision'] = 'PAUSED_NO_QUERY' if self.paused else 'NEXT_SCHEDULED_CYCLE'
            report['consecutive_errors'] = self.consecutive_errors
            if self.on_fault:
                try:self.on_fault(report)
                except Exception:self.stats['command_errors'] += 1
        else:self.consecutive_errors = 0
        return report

    def start(self):
        if self.thread is not None:
            raise RuntimeError('COLLECTOR_ALREADY_STARTED')
        self.thread = threading.Thread(target=self._run, name='feedback_bus', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                self.tick()
                delay = .05 if self.paused else max(0, (self.next_tick-self.clock_ns())/1e9)
                self.stop_event.wait(delay)
        except Exception:
            self.board._bounded_bus_feedback.fail('COLLECTOR_WORKER_FAILED')
            self.stop_event.set()
            if self.on_fault:
                try:self.on_fault(None)
                except Exception:self.stats['command_errors'] += 1

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=.5)
            if self.thread.is_alive():
                raise RuntimeError('COLLECTOR_STILL_RUNNING')
