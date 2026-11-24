import struct
from types import SimpleNamespace as S
import pytest
from patches.jetrover_readback.guarded_readback import ReadbackGate, ReadbackFault
from patches.jetrover_readback.continuous_feedback import ExperimentalReadbackGate, ContinuousFeedbackCollector

class Board:
    def __init__(self, step=2_000_000):
        self.now=1_000_000_000; self.step=step;self.sent=[];self.bad=False
        self._bounded_bus_feedback=ExperimentalReadbackGate(clock_ns=lambda:self.now)
        self._bounded_bus_feedback.arm_experiment('offline test')
        self._last_bus_tx_frame=self._last_bus_rx_frame=None
    def bus_servo_read_and_unpack(self,i,cmd,unpack,diagnostic):
        def send(deadline):
            self.sent.append(i);self.now+=self.step
            payload=struct.pack('<BBbh',99 if self.bad else i,5,0,500+i)
            self._last_bus_tx_frame=bytes([5,i]);self._last_bus_rx_frame=payload
            self._bounded_bus_feedback.receive(payload,crc_checked=True)
        self.last_bus_position_sample=self._bounded_bus_feedback.read_position(i,cmd,unpack,send,
            write_timeout_s=.001,reception_enabled=True,diagnostic=diagnostic,send_with_deadline=True)

def collector(board):return ContinuousFeedbackCollector(board,clock_ns=lambda:board.now)

def test_continuous_600_cycles_has_no_diagnostic_total_budget():
    b=Board();c=collector(b)
    for _ in range(600):
        r=c.tick();assert r['complete'];b.now=c.next_tick
    assert len(b.sent)==3600 and c.stats['cycles']==600
    assert c.snapshot()['valid'] and not r['canonical_observation_eligible']
    assert b._bounded_bus_feedback.diagnostic_attempt_limit is None
    assert b._bounded_bus_feedback.diagnostic_attempts==3600

def test_legacy_gate_reuse_policy_is_not_changed():
    gate=ReadbackGate();gate.arm_diagnostic_once('legacy')
    gate.used.add((1,5))
    with pytest.raises(ReadbackFault,match='KEY_REUSE_UNSAFE'):
        gate.read_position(1,5,'<BBbh',lambda:None,write_timeout_s=.001,
                          reception_enabled=True,diagnostic=True)

def test_experiment_cannot_arm_a_trusted_epoch_or_rearm():
    b=Board()
    with pytest.raises(ReadbackFault,match='EXPERIMENT_CANNOT_BECOME_TRUSTED'):
        b._bounded_bus_feedback.arm('evidence')
    with pytest.raises(ReadbackFault,match='REARM_FORBIDDEN'):
        b._bounded_bus_feedback.arm_experiment('again')

def test_wrong_id_latches_and_no_next_query_is_sent():
    b=Board();c=collector(b);b.bad=True
    assert not c.tick()['complete'] and c.paused and not c.stop_event.is_set()
    b.now+=1_000_000_000;assert c.tick() is None and b.sent==[1]
    assert not c.snapshot()['valid']

def test_scheduler_skips_overrun_without_backlog_burst():
    b=Board(step=12_000_000);c=collector(b)
    r=c.tick();assert r['complete'] and r['skipped_periods']==1
    assert c.tick() is None and len(b.sent)==6
    b.now=c.next_tick;assert c.tick()['sequence']==1

def test_recording_failure_does_not_kill_collection():
    b=Board();c=collector(b)
    def fail(_):raise OSError('disk failed')
    c.on_record=fail
    assert c.tick()['complete'] and not c.stop_event.is_set()
    b.now=c.next_tick;assert c.tick()['complete']
    assert c.stats['record_errors']==2

def test_commands_are_prioritized_and_interleaved_only_at_query_boundaries():
    b=Board();c=collector(b);seen=[]
    c.submit(lambda:seen.append(('move',len(b.sent))))
    c.submit(lambda:seen.append(('stop',len(b.sent))),stop=True)
    c.tick()
    assert seen==[('stop',0),('move',1)]

def test_stale_snapshot_is_invalid_even_without_another_cycle():
    b=Board();c=collector(b);c.tick();b.now+=600_000_000
    assert not c.snapshot()['valid']

def test_ambiguous_timeout_is_not_automatically_recovered():
    gate=ExperimentalReadbackGate();gate.arm_experiment('offline')
    with pytest.raises(ReadbackFault,match='READ_TIMEOUT'):
        gate.read_position(1,5,'<BBbh',lambda:None,timeout_s=.01,
                          write_timeout_s=.001,reception_enabled=True,diagnostic=True)
    with pytest.raises(ReadbackFault,match='READ_TIMEOUT'):
        gate.read_position(1,5,'<BBbh',lambda:pytest.fail('retry'),
                          write_timeout_s=.001,reception_enabled=True,diagnostic=True)

def test_duplicate_reply_latches_fault_and_old_key_is_not_trusted():
    gate=ExperimentalReadbackGate();gate.arm_experiment('offline')
    def send():
        data=struct.pack('<BBbh',1,5,0,502)
        gate.receive(data,crc_checked=True);gate.receive(data,crc_checked=True)
    with pytest.raises(ReadbackFault,match='DUPLICATE_REPLY'):
        gate.read_position(1,5,'<BBbh',send,write_timeout_s=.001,reception_enabled=True,diagnostic=True)

def test_same_key_delayed_duplicate_is_explicitly_unprovable_in_experiment():
    b=Board();c=collector(b);c.tick();b.now=c.next_tick;c.tick()
    # Success never upgrades missing wire identity to measured/calibrated evidence.
    assert not b._bounded_bus_feedback.status()['inner_association_verified']
    assert not b._bounded_bus_feedback.status()['hardware_validated']
