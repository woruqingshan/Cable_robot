from deployment.local.analyze_feedback_trial import metrics,percentile

def row(sequence,i,start,valid=True):
    return dict(sequence=str(sequence),servo_id=str(i),raw_pulse=str(500+i),
        read_started_at_ns=str(start),response_received_at_ns=str(start+2_000_000),
        read_latency_ns='2000000',outer_frame_validated=str(valid),error='' if valid else 'READ_TIMEOUT',skipped_periods='0')

def test_measured_frequency_and_recording_duration_use_separate_denominators():
    rows=[row(s,i,1_000_000_000+s*50_000_000+j*2_000_000) for s in range(3) for j,i in enumerate((1,2,3,4,5,10))]
    m=metrics(rows,dict(started_monotonic_ns=1_000_000_000,stopped_monotonic_ns=1_200_000_000,complete=True))
    assert m['complete_snapshots']==3 and m['actual_update_hz']==20
    assert m['complete_snapshots_per_recording_second']==15
    assert m['queries_attempted']==18 and m['p99_latency_ms']==2
    assert m['mean_host_span_ms']==12 and not m['canonical_observation_eligible']

def test_partial_failed_snapshot_does_not_count_unsent_channels_as_queries():
    rows=[row(0,1,1_000_000_000,False)]
    m=metrics(rows,dict(started_monotonic_ns=1_000_000_000,stopped_monotonic_ns=2_000_000_000,complete=False))
    assert m['queries_attempted']==1 and m['complete_snapshots']==0
    assert m['outer_query_success_fraction']==0 and m['mean_latency_ms'] is None

def test_no_data_is_unknown_not_zero_latency_or_success():
    m=metrics([],dict(started_monotonic_ns=0,stopped_monotonic_ns=1_000_000_000,complete=True))
    assert m['actual_update_hz'] is None and m['outer_query_success_fraction'] is None
    assert m['mean_latency_ms'] is None

def test_percentiles_are_explicit_nearest_rank():
    assert percentile([1,2,3,4,5],.99)==5 and percentile([], .95) is None
