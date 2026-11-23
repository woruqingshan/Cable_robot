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

D4 software checkpoint: RobotStateProvider accepts complete unverified raw
batches separately from controller cache. Pulse conversion follows runtime
controller id/init/min/max parameters and 240deg/1000 pulse scale, with SHA
identity. Physical zero/direction/internal offsets remain unverified.
RealObservationNode optionally publishes candidate JointState on its private
physical diagnostics topic with unset header acquisition time; raw unknown
feedback never enters canonical Observation. Isolated ROS callback test passes.
Existing URDF/KDL FK interface is wired, but urdf_parser_py and PyKDL are missing
on this robot. No replacement kinematic engine or package install was performed.
Selected regression: 364 passed, 2 skipped (PyKDL, FastAPI), 18.25s.

## 最新执行终点：USB控制面打开失败

切换已单独获批。旧维护进程退出后，新ede2020驱动在open期间的DTR ioctl Errno110失败。
未运行D2/D3，未自动重试或恢复。只读观察器已停止，当前无串口驱动/正常控制；
整车父服务仍运行且Restart=no。D4实机反馈/FK和R2不声称通过。
代码与证据保留，详见R1_D2_D4_RESULT_20261123.md。
