from copy import deepcopy
import pytest
from deployment.local.test_support import observation
from vla_protocol.observation import Observation
from vla_protocol.research import ResearchObservation
from vla_protocol import codec

def sample():
 d=observation().model_dump()
 for k in ['simulated','calibration_status','source_profile']:d.pop(k,None)
 d['gripper'].update(simulated=False,calibrated=False)
 vals=d['joints']['positions']+[d['gripper']['position']]
 d['feedback_report']={'complete':True,'source_type':'unknown','canonical_observation_eligible':False,'channels':[dict(servo_id=i,raw_pulse=500,source_type='unknown',outer_frame_validated=True,error=None) for i in [1,2,3,4,5,10]]}
 d['conversion_evidence']={'positions':vals,'raw_pulses':[500]*6,'physical_calibrated':False}
 d['timing_evidence']={'feedback_receive_age_s':.1,'image_receive_age_s':.1}
 return d

def test_research_roundtrip_and_canonical_rejection():
 o=ResearchObservation.model_validate(sample());assert codec.loads(codec.dumps(o),ResearchObservation)==o
 with pytest.raises(Exception):Observation.model_validate(o.model_dump())

@pytest.mark.parametrize('change',[
 lambda d:d.update(execution_enabled=True),
 lambda d:d.update(physical_calibrated=True),
 lambda d:d['feedback_report'].update(complete=False),
 lambda d:d['feedback_report']['channels'].pop(),
 lambda d:d['feedback_report']['channels'][0].update(source_type='controller_cached'),
 lambda d:d['feedback_report']['channels'][0].update(error='TIMEOUT'),
 lambda d:d['timing_evidence'].update(image_receive_age_s=.6),
 lambda d:d['timing_evidence'].update(feedback_receive_age_s=-.1),
 lambda d:d['conversion_evidence']['positions'].__setitem__(0,99.),
 lambda d:d['images'][0].update(sha256='0'*64),
])
def test_invalid_research_input(change):
 d=sample();change(d)
 with pytest.raises(Exception):ResearchObservation.model_validate(d)
