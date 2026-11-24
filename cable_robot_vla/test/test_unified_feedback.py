import json
import struct
import time
from types import SimpleNamespace as S
import pytest
from cable_robot_control.feedback_motion import MotionPolicy, UnifiedFrameWriter
from cable_robot_control.feedback_recording import FeedbackRecorder
from patches.jetrover_readback.guarded_readback import ReadbackFault

def plan():
    return dict(schema_version=1,A={'1':500,'2':765,'3':15,'4':220},
        B={'1':300,'2':850,'3':50,'4':250},duration_s=4.,
        reviewed_path_reference='offline fixture only',motion_authorization_reference='mock')

def policy():return MotionPolicy(plan(),{1:500,2:765,3:15,4:220,5:500,10:500})

def crc(b):return sum(b)%256

def frame(payload):
    b=bytes([0xaa,0x55,5,len(payload)])+payload
    return b+bytes([crc(b[2:])])

def move(pulses,duration=4.):
    return frame(bytes([1])+struct.pack('<HB',int(duration*1000),len(pulses))+
                 b''.join(struct.pack('<BH',i,p) for i,p in sorted(pulses.items())))

class Port:
    write_timeout=.01
    def __init__(self):self.frames=[]
    def write(self,b):self.frames.append(bytes(b));return len(b)

@pytest.mark.parametrize('change',[
 lambda p:p['B'].update({'10':500}),
 lambda p:p['B'].update({'1':1001}),
 lambda p:p.update(duration_s=float('nan')),
 lambda p:p.update(reviewed_path_reference=''),
 lambda p:p['B'].update({'1':0}),
])
def test_unreviewed_or_unsafe_plan_refused(change):
    p=plan();change(p)
    with pytest.raises((ValueError,KeyError)):MotionPolicy(p,{1:500,2:765,3:15,4:220})

def test_start_pose_must_match_fresh_feedback():
    with pytest.raises(ValueError,match='START_POSE_CHANGED'):
        MotionPolicy(plan(),{1:520,2:765,3:15,4:220})

def test_static_writer_refuses_motion_torque_and_gripper():
    port=Port();w=UnifiedFrameWriter(port,crc)
    with pytest.raises(ReadbackFault):w.write(move(policy().B))
    assert not port.frames
    w.policy=policy()
    for payload in (bytes([0xb,1,1]),bytes([3,1,10])):
        with pytest.raises(ReadbackFault):w.write(frame(payload))
    assert not port.frames

def test_exact_B_then_A_only_and_no_third_move():
    port=Port();w=UnifiedFrameWriter(port,crc);w.policy=policy()
    w.write(move(w.policy.B));w.write(move(w.policy.A))
    with pytest.raises(ReadbackFault):w.write(move(w.policy.B))
    assert len(port.frames)==2

def test_stop_uses_only_participating_arm_ids_and_no_torque():
    port=Port();w=UnifiedFrameWriter(port,crc);w.policy=policy()
    w.write(move(w.policy.B));w.write(frame(bytes([3,4,1,2,3,4])))
    assert w.policy.stopped and not w.motion_active
    with pytest.raises(ReadbackFault):w.write(move(w.policy.A))

def batch(seq=1):
    return dict(sequence=seq,channels=[dict(servo_id=1,raw_pulse=502,source_type='unknown',
        read_started_at_ns=1,response_received_at_ns=2,read_latency_ns=1,outer_frame_validated=True,error=None)],
        conversion_candidate={'joint_names':['joint1'],'positions':[-.008]},host_transaction_span_ns=1)

def test_recording_drain_preserves_raw_frames_and_stopping_is_independent(tmp_path):
    r=FeedbackRecorder(tmp_path/'run')
    for i in range(20):r.submit(batch(i))
    result=r.stop()
    assert result['complete'] and result['written_batches']==20
    r.submit(batch(99));assert r.accepted==20
    rows=[json.loads(x) for x in (r.path/'raw_feedback.jsonl').read_text().splitlines()]
    assert len(rows)==20 and rows[-1]['sequence']==19
    assert 'nominal_rad' in (r.path/'feedback.csv').read_text()

def test_recording_backpressure_marks_incomplete_without_raising_in_collector(tmp_path):
    r=FeedbackRecorder(tmp_path/'run',capacity=1)
    # Deterministic queue saturation while the worker is stalled by a held GIL interval.
    for i in range(5000):r.submit(batch(i))
    result=r.stop()
    assert result['rejected_batches']>0 and result['error']=='RECORDING_QUEUE_FULL'
    assert not result['complete']

def test_recording_will_not_overwrite_existing_run(tmp_path):
    p=tmp_path/'run';p.mkdir()
    with pytest.raises(FileExistsError):FeedbackRecorder(p)

def test_motion_monitor_accepts_changes_and_detects_frozen_or_uncommanded_feedback():
    from cable_robot_control.feedback_motion import MotionFeedbackMonitor
    w=S(policy=policy(),motion_active=True);w.policy.index=1
    def report(t,delta=0,gripper=500):
        return {'channels':[dict(servo_id=i,raw_pulse=(500+delta if i<5 else gripper if i==10 else 500),response_received_at_ns=t) for i in (1,2,3,4,5,10)]}
    m=MotionFeedbackMonitor();m.check(report(1_000_000_000),w)
    m.check(report(1_050_000_000,delta=3),w)
    with pytest.raises(ReadbackFault,match='UNCOMMANDED_ACTUATOR_CHANGED'):
        m.check(report(1_100_000_000,delta=4,gripper=511),w)
    m=MotionFeedbackMonitor();m.check(report(1_000_000_000),w)
    with pytest.raises(ReadbackFault,match='MOTION_FEEDBACK_FROZEN'):
        m.check(report(4_000_000_000),w)
