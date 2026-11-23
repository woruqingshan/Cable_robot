# R1 D2-D4 integration checkpoints (2026-11-23)

Baseline: 384959d. Runtime remains b020afb until a coordinated driver switch is approved.

D2 implementation: one coordinated serial owner; explicitly carry prior ID1 use;
query only IDs 2,3,4,5,10 in order; preserve complete outer TX/RX, host intervals,
raw pulses, failures and batch span. No retries or fabricated measured provenance.

Finite D3 infrastructure: up to three 6-ID cycles, target 1 Hz, no automatic loop.
Key reuse is permitted ONLY by an explicit bounded diagnostic policy. Trusted
KEY_REUSE_UNSAFE behavior is unchanged. Identical late replies inside a later
request window remain indistinguishable under this protocol; every such result
stays unknown/uncalibrated and ineligible for Observation. Sleep is scheduling,
not a response-association mechanism or proof of recovery.

Tests: 109 passed with isolated ROS environment; focused finite-plan tests pass.
Actual D2/D3 requests: NOT_RUN pending coordinated driver substitution approval.
Current service is ID1-only with its one opportunity consumed; process restart
must not be interpreted as a fresh protocol epoch.

D3 software checkpoint: bounded repeat experiment and metric analysis available;
3 metric tests pass. Metrics report empirical outer-reply success, mean/P95/max
host latency, observed cycle cadence and six-channel host span. Device sample
rate/time and transaction association are explicitly unknown. No claim of
reliable closed-loop feedback is made without firmware-level evidence.
