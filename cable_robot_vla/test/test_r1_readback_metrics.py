import pytest
from cable_robot_vla.readback_metrics import diagnostic_statistics


def fixture():
    batches=[]
    for cycle in range(3):
        start=1_000_000_000+cycle*1_000_000_000
        channels=[dict(servo_id=i,error=None,outer_frame_validated=True,
            read_started_at_ns=start+j*2_000_000,read_latency_ns=(j+1)*100_000)
            for j,i in enumerate([1,2,3,4,5,10])]
        batches.append(dict(complete=True,channels=channels,host_transaction_span_ns=11_000_000))
    return dict(batches=batches)


def test_metrics_describe_outer_replies_not_device_sampling():
    result=diagnostic_statistics(fixture())
    assert result['attempts']==18 and result['valid_outer_replies']==18
    assert result['empirical_cycle_hz']==1
    assert result['host_latency_mean_ms']==pytest.approx(.35)
    assert result['host_latency_p95_ms']==pytest.approx(.6)
    assert result['host_six_channel_spans_ms']==[11,11,11]
    assert not result['canonical_observation_eligible']
    assert result['device_sample_span_ns'] is None


def test_failure_counts_are_preserved():
    value=fixture();value['batches']=value['batches'][:1]
    value['batches'][0]['complete']=False
    value['batches'][0]['channels'][-1]=dict(error='READ_TIMEOUT',outer_frame_validated=False)
    result=diagnostic_statistics(value)
    assert result['attempts']==6 and result['valid_outer_replies']==5
    assert result['outer_reply_success_fraction']==pytest.approx(5/6)
    assert result['host_six_channel_spans_ms']==[]
    assert result['empirical_cycle_hz'] is None


def test_empty_metrics_never_invent_a_rate():
    result=diagnostic_statistics({})
    assert result['attempts']==0
    assert result['outer_reply_success_fraction'] is None
    assert result['empirical_cycle_hz'] is None
    assert result['host_latency_p95_ms'] is None
