"""Finite readback metrics; no hardware or controller interfaces."""
def diagnostic_statistics(report):
    """Empirical finite-run metrics; no fresh-sample or inner-association claim."""
    import math
    batches = report.get("batches", [])
    rows = [row for batch in batches for row in batch["channels"]]
    valid = [r for r in rows if not r.get("error") and r.get("outer_frame_validated")]
    latency = sorted(r["read_latency_ns"]/1e6 for r in valid)
    starts = [b["channels"][0]["read_started_at_ns"] for b in batches
              if b.get("complete") and b.get("channels")]
    cadence = None if len(starts) < 2 else (len(starts)-1)*1e9/(starts[-1]-starts[0])
    return dict(attempts=len(rows), valid_outer_replies=len(valid),
        outer_reply_success_fraction=None if not rows else len(valid)/len(rows),
        host_latency_mean_ms=None if not latency else sum(latency)/len(latency),
        host_latency_p95_ms=None if not latency else latency[math.ceil(.95*len(latency))-1],
        host_latency_max_ms=None if not latency else max(latency),
        empirical_cycle_hz=cadence,
        host_six_channel_spans_ms=[b["host_transaction_span_ns"]/1e6 for b in batches
                                  if b.get("complete") and len(b["channels"])==6],
        metric_scope="finite unverified outer replies; NOT validated device sample rate",
        p95_method="nearest rank", device_sample_span_ns=None,
        canonical_observation_eligible=False)
