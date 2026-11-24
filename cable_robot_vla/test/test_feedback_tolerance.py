import struct
import pytest
from test_continuous_feedback import Board,collector
from patches.jetrover_readback.continuous_feedback import ExperimentalReadbackGate,CompletedReplyError
from patches.jetrover_readback.guarded_readback import ReadbackFault

def run(gate,status=1,pulse=500):
 def send():gate.receive(struct.pack('<BBbh',1,5,status,pulse),crc_checked=True)
 return gate.read_position(1,5,'<BBbh',send,write_timeout_s=.001,reception_enabled=True,diagnostic=True)

def test_unknown_status_remains_isolated_with_identity_evidence():
 g=ExperimentalReadbackGate();g.arm_experiment('test')
 with pytest.raises(ReadbackFault):run(g)
 assert g.fault=='DEVICE_ERROR' and g.last_rejection['servo_id']==1
 assert g.last_rejection['status']==1 and g.last_rejection['decision'].startswith('PAUSE')

def test_terminal_status_requires_explicit_contract_and_does_not_rearm_motion():
 g=ExperimentalReadbackGate()
 with pytest.raises(ValueError):g.configure_terminal_statuses([1],'')
 g.configure_terminal_statuses([1],'MOCK proven terminal reply')
 g.arm_experiment('test')
 with pytest.raises(CompletedReplyError):run(g)
 assert not g.fault and g.pending is None
 assert run(g,status=0).raw_pulse==500

def test_complete_invalid_pulse_rejects_data_but_not_next_transaction():
 g=ExperimentalReadbackGate();g.arm_experiment('test')
 with pytest.raises(CompletedReplyError):run(g,status=0,pulse=1001)
 assert run(g,status=0,pulse=502).raw_pulse==502

def test_paused_worker_and_manually_stopped_worker_preserve_stop_path():
 b=Board();c=collector(b);b.bad=True;c.tick()
 assert c.paused and not c.stop_event.is_set()
 sent=len(b.sent);events=[];c.submit(lambda:events.append('stop'),stop=True)
 c.tick();assert events==['stop'] and len(b.sent)==sent
 c.stop();c.submit(lambda:events.append('final stop'),stop=True)
 assert len(events)==2

def test_repeated_terminal_errors_exhaust_budget_without_repeating_failed_request():
 b=Board();g=ExperimentalReadbackGate(clock_ns=lambda:b.now)
 g.configure_terminal_statuses([1],'MOCK_TERMINAL');g.arm_experiment('mock');b._bounded_bus_feedback=g
 calls=[]
 def reject(i,cmd,unpack,diagnostic):calls.append(i);return run(g,status=1)
 b.bus_servo_read_and_unpack=reject;c=collector(b)
 for _ in range(3):
  assert not c.tick()['complete'];b.now=c.next_tick
 assert len(calls)==3 and c.paused and g.fault=='REPEATED_COMPLETED_ERRORS'
 c.tick();assert len(calls)==3

def test_stop_write_failure_is_reported_after_communication_pause():
 b=Board();c=collector(b);b.bad=True;c.tick()
 def broken():raise OSError('port unavailable')
 c.submit(broken,stop=True);c.tick()
 assert c.last_stop_result['status']=='FAILED_DELIVERY_NOT_GUARANTEED'
 assert c.last_stop_result['delivery_guaranteed'] is False
 assert c.paused and not c.stop_event.is_set()

def test_permission_publication_lag_is_not_revocation_before_confirmation():
 from deployment.local.run_reviewed_feedback_motion import motion_permission_revoked
 assert not motion_permission_revoked(dict(motion_permission=False,commands=[]),False)
 assert motion_permission_revoked(dict(motion_permission=False,commands=[]),True)

def test_completed_roundtrip_and_fault_stop_are_distinct_permissions():
 from deployment.local.run_reviewed_feedback_motion import motion_permission_revoked
 commands=[dict(kind='position',status='SENT')]*2
 assert not motion_permission_revoked(dict(motion_permission=False,commands=commands),True)
 assert motion_permission_revoked(dict(motion_permission=False,commands=commands+[dict(kind='stop')]),True)
