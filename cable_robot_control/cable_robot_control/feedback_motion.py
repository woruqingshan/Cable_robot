"""Reviewed A/B capability on one existing SDK writer, no planner or torque path."""
import math
import struct
from patches.jetrover_readback.guarded_readback import MaintenanceFrameWriter, ReadbackFault
from .safety import ArmLimits, validate_pulses, validate_duration, soft_delta_warnings

class MotionPolicy:
    def __init__(self, plan, current):
        self.limits=ArmLimits([1,2,3,4,5],10)
        required={'schema_version','A','B','duration_s','reviewed_path_reference','motion_authorization_reference'}
        if not isinstance(plan,dict) or set(plan)!=required or plan['schema_version']!=1:
            raise ValueError('INVALID_MOTION_PLAN')
        for field in ('reviewed_path_reference','motion_authorization_reference'):
            if not isinstance(plan[field],str) or not plan[field].strip():
                raise ValueError('REVIEW_AND_AUTHORIZATION_REQUIRED')
        self.A={int(k):v for k,v in plan['A'].items()}
        self.B={int(k):v for k,v in plan['B'].items()}
        if (set(self.A)!=set(self.B) or not 3<=len(self.A)<=5
                or any(type(v) is not int for v in [*self.A.values(),*self.B.values()])):
            raise ValueError('THREE_TO_FIVE_ARM_JOINTS_REQUIRED')
        validate_pulses(self.A,self.limits,include_gripper=False)
        validate_pulses(self.B,self.limits,include_gripper=False)
        self.duration=plan['duration_s']
        if type(self.duration) not in (int,float) or not math.isfinite(self.duration) or not 2<=self.duration<=8:
            raise ValueError('INVALID_EXPERIMENT_MOVE_DURATION')
        validate_duration(self.duration,self.limits)
        if soft_delta_warnings(self.A,self.B,self.limits):
            raise ValueError('SOFT_DELTA_REVIEW_REQUIRED')
        if any(abs(self.B[i]-self.A[i])/self.duration>100 for i in self.A):
            raise ValueError('EXPERIMENT_VELOCITY_LIMIT')
        if any(i not in current or abs(current[i]-self.A[i])>3 for i in self.A):
            raise ValueError('START_POSE_CHANGED')
        if sum(self.A[i]!=self.B[i] for i in self.A)<3:
            raise ValueError('THREE_MOVING_JOINTS_REQUIRED')
        self.index=0;self.enabled=True;self.stopped=False

    def validate(self, pulses, duration):
        expected=self.B if self.index==0 else self.A
        if (not self.enabled or self.index>=2 or self.stopped or pulses!=expected
                or abs(duration-self.duration)>.0005):
            raise ReadbackFault('UNREVIEWED_MOTION_REJECTED')

class UnifiedFrameWriter(MaintenanceFrameWriter):
    """Adds only exact A/B frames after explicit reviewed capability activation."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.policy=None
        self.last_motion_frame=None
        self.motion_active=False

    def _allowed(self,function,payload):
        if super()._allowed(function,payload):return True
        if function!=5 or not self.policy:return False
        if len(payload)>=2 and payload[0]==3:
            ids=list(payload[2:])
            return (payload[1]==len(ids) and ids==sorted(self.policy.A)
                    and self.policy is not None)
        if len(payload)<7 or payload[0]!=1:return False
        duration=int.from_bytes(payload[1:3],'little')/1000
        count=payload[3]
        if len(payload)!=4+3*count:return False
        rows=[struct.unpack('<BH',payload[i:i+3]) for i in range(4,len(payload),3)]
        if len({i for i,_ in rows})!=len(rows):return False
        self.policy.validate(dict(rows),duration)
        return True

    def write(self,frame,**kwargs):
        raw=bytes(frame)
        motion=len(raw)>5 and raw[2]==5 and raw[4] in (1,3)
        written=super().write(raw,**kwargs)
        if motion:
            self.last_motion_frame=raw.hex()
            if raw[4]==1:
                self.policy.index+=1
                self.motion_active=True
            else:
                self.motion_active=False
                self.policy.stopped=True
        return written

class MotionFeedbackMonitor:
    """Experimental anomaly bounds, not calibrated tracking or collision certification."""
    def __init__(self):self.previous=None;self.index=None;self.started=None;self.initial=None

    def check(self,report,writer):
        if not writer.motion_active or not writer.policy:
            self.previous=None;self.index=None;return
        rows={r['servo_id']:r for r in report['channels']}
        now=max(r['response_received_at_ns'] for r in rows.values())
        if self.index!=writer.policy.index:
            self.index=writer.policy.index;self.started=now
            self.initial={i:r['raw_pulse'] for i,r in rows.items()};self.previous=rows;return
        for i,r in rows.items():
            before=self.previous[i];dt=(r['response_received_at_ns']-before['response_received_at_ns'])/1e9
            delta=abs(r['raw_pulse']-before['raw_pulse'])
            if dt<=0 or delta>max(20,200*dt):raise ReadbackFault('MOTION_FEEDBACK_JUMP')
            if i not in writer.policy.A and delta>10:raise ReadbackFault('UNCOMMANDED_ACTUATOR_CHANGED')
        target=writer.policy.B if self.index==1 else writer.policy.A
        if (now-self.started)/1e9>writer.policy.duration*.6:
            for i in target:
                if abs(target[i]-self.initial[i])>25 and abs(rows[i]['raw_pulse']-self.initial[i])<3:
                    raise ReadbackFault('MOTION_FEEDBACK_FROZEN')
        self.previous=rows
