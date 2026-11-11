# Stage 1 local VLA integration

This implements a repeated ROS 2 observation → HTTP inference → explicit action
adaptation → local validation → mock execution loop. It has **no hardware execution
mode**. The default policy holds position. No VLA weights, LeRobot checkout, GPU
adapter, real perception, calibration, insertion/removal controller or research
algorithm is included.

## Components and boundaries

| Component | Implemented responsibility |
| --- | --- |
| `vla_protocol` | Python 3.10/Pydantic v2 contracts, bounded PNG/JSON, stable error envelopes |
| `vla_server` | FastAPI `/health`, `/v1/infer`, replaceable `Policy`, deterministic hold policy |
| `cable_robot_vla` | ROS RGB/state mock, snapshot collector, worker HTTP client, synthetic mapping, validator adapter, recorder and mock state machine |
| `deployment/local` | Locked Python dependencies, declared test groups, owned-process smoke probe |
| `deployment/cloud` | Stage 2 boundary documentation only |

`ObservationAdapter` is hosted inside the VLA node, rather than introducing a
third ROS process and a second serialized observation topic. Network and P2 work
run in separate single-worker executors. ROS callbacks collect/publish messages
and poll completed work. One inference is outstanding at a time; local validation
also completes before another request is submitted.

Neither shared protocol nor server imports ROS, P2, Torch or LeRobot. The ROS
package has no vendor control imports or command clients. P2 is explicitly
configured and lazily imported; its existing files and algorithms are unchanged.

## Setup (Ubuntu 22.04, ROS 2 Humble, Python 3.10)

Run from the Cable_robot Git root. Commands create only the new environment and
build outputs; do not recreate or modify the vendor workspace.

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
export VLA_PYTHON="$PWD/.venv/bin/python"
"$VLA_PYTHON" -m pip install -r deployment/local/requirements.lock
"$VLA_PYTHON" -m pip install --no-build-isolation --no-deps ./vla_protocol ./vla_server

"$VLA_PYTHON" -m colcon --log-base log/vla_colcon build \
  --base-paths cable_robot_vla --packages-select cable_robot_vla \
  --build-base build/vla --install-base install/vla --symlink-install
source install/vla/setup.bash
```

The ROS/APT base supplies `rclpy`, messages, `std_srvs`, launch, colcon and their
system libraries. Pure Python application dependencies are pinned in
`deployment/local/requirements.lock`. P2 uses its existing separately documented
ROS/APT environment; it is not installed by this setup. The root `.venv/` is
already ignored by Git. Alternatively create the venv outside the checkout.

## Start the complete mock loop

Terminal 1, using the environment above:

```bash
"$VLA_PYTHON" -m vla_server.app --host 127.0.0.1 --port 8000
```

Terminal 2, source ROS and `install/vla/setup.bash`, then set `VLA_PYTHON` to the
same absolute interpreter path:

```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=71
ros2 launch cable_robot_vla local_mock.launch.py \
  python_executable:="$VLA_PYTHON" \
  advance_simulation:=true allow_unvalidated_simulation:=true \
  log_dir:="$PWD/log/vla/demo_01"
```

The two explicit simulation options permit unchecked simulated advancement.
Every such result remains `NOT_CHECKED` / `SIMULATED_UNVALIDATED`. Omitting them
records candidates without advancing state. The hold policy preserves joint
values while execution increments the simulated state revision. It does not
claim cable-manipulation success.

```bash
ros2 topic echo /idc_vla/mock/outcomes
ros2 topic echo /idc_vla/mock/joint_states
ros2 topic pub --once /idc_vla/mock/task_instruction std_msgs/msg/String \
  '{data: "Hold position for the next observation"}'
ros2 service call /idc_vla/mock/reset std_srvs/srv/Trigger '{}'
```

Reset creates a new session and cancels an active mock trajectory. A changed
instruction increments task revision and invalidates old requests/trajectories.
Stop each terminal with Ctrl-C. Only owned worker threads, nodes and services
are closed. The server never publishes ROS messages.

Direct module startup also works after installing the packages:
`"$VLA_PYTHON" -m cable_robot_vla.mock_robot_node` and
`"$VLA_PYTHON" -m cable_robot_vla.vla_node --ros-args -p server_url:=http://127.0.0.1:8000`.

## ROS observation and execution semantics

Namespace is fixed to `/idc_vla/mock`; topic leaf names are configurable using
`rgb_topic`, `joints_topic`, `gripper_topic`, `task_topic`, `execute_topic`,
`outcomes_topic`, `execution_status_topic` and `instruction_topic`. Remaps outside
that namespace are rejected before application endpoints are created. Node
startup requires localhost-only DDS and a nonzero private domain in 1..101.
Stage 1 rejects `use_sim_time`; source stamps use the local ROS system clock.

The mock publishes a deterministic 64×48 RGB fixture at 10 Hz (or an explicitly
configured `image_path` PNG asset), five named arm joints, separate `r_joint`
gripper state and task/status metadata. Images are consumed through ordinary ROS
`Image` subscriptions and encoded to PNG by the observation adapter. No HTTP
test shortcut is used in the running loop. All source timestamps are preserved.

The default scene ID is only a synthetic label: the all-zero scene hash is an
explicit sentinel for **no geometric scene loaded**, and the model ID labels
synthetic joint state. These defaults cannot pass required P2 identity checks.
Supply actual scene/model identities when enabling P2; no RGB-D reconstruction
or calibrated TCP is implied by an observation.

Mock execution checks session, state revision, task revision, scene/model,
starting joints/gripper, candidate age and duplicate action identity. It advances
the simulated state at the declared waypoint offsets, publishing those updates
through the mock robot. It is a waypoint simulation, not a physics simulation or
a servo trajectory controller. A dropped execution acknowledgement is logged.

## Wire contract v1

All schemas forbid unexpected fields and implicit string-to-number coercion.
JSON rejects duplicate keys and NaN/Infinity. Numeric values must be finite.

* `Observation`: schema/request/session identity, observation sequence, task and
  state revisions, robot/profile, instruction, scene ID/canonical SHA256,
  model ID, named radians, separate gripper, RGB PNG descriptors/payloads, source
  timestamps/clock domain and explicit synthetic/uncalibrated status.
* `PolicyActionChunk`: full correlated identity, native space, named channels,
  units, absolute/delta semantics, `[H,D]` tensor, positive increasing offsets,
  policy/framework/checkpoint identity, processor/normalization identity,
  inference duration and synthetic flag. Native dimensions are not fixed to five.
* `RobotActionChunk`: `jetrover_joint_abs_rad_v1`, exact order
  `joint1,joint2,joint3,joint4,joint5`, absolute radians, `[H,5]`, separate timing
  and source policy action ID. Gripper is excluded.
* `ValidatedTrajectory`: source action and starting state, verdict, accepted
  candidate or rejection reasons, scene/model identity, validation configuration
  and provenance. Physical validation and continuous-collision proof remain false.

Only an explicitly synthetic identity mapping is registered. It requires the
native JetRover profile, exact channels/units, absolute semantics,
`mock_identity_v1` processor and `none` normalization. Matching dimensions alone
are insufficient. Real-model, delta, unknown-unit and unknown-normalization
outputs are rejected by the adapter even if their tensors are well formed.

Health advertises the protocol, policy, limits and supported spaces, with
`gpu_model_ready:false`. The hold policy's action ID and values are deterministic
for one session/request. Measured inference duration may differ across requests.
`Policy.start/predict/close` is the future model integration boundary.
The server serializes that lifecycle on one owned policy worker, including after
an HTTP timeout. Thread inference cannot be forcibly killed; shutdown waits for
an already-running call before closing policy resources. A future model needing
hard inference termination will require a process-based adapter.

## Exact defaults and limits

| Setting | Default / wire ceiling |
| --- | --- |
| Request/response JSON | 6,000,000 bytes |
| PNG payload | 1,048,576 decoded bytes per image; 1–4 images |
| Image geometry | At most 1024 per dimension and 1,048,576 pixels |
| PNG format | Non-interlaced RGB8; signature, chunks/CRC, hash and bounded inflation checked |
| Native channels | 1–256; robot profile exactly 5 |
| Action horizon | 1–32 |
| Timing | Positive, strictly increasing offsets; final offset ≤10 s |
| HTTP deadline | 2 s, measured on client-local monotonic clock |
| Source freshness | 0.5 s receive age and ROS source age |
| Source synchronization | Maximum 0.02 s skew, same declared clock domain |
| Candidate freshness | 5 s from request submission, rechecked after validation |
| Inference interval | 0.25 s |
| P2 path resolution | 0.01 rad, configurable up to 0.02 rad |
| P2 coarse path sample budget | 4096 before optional existing workspace refinement |
| P2 validation budget | 2 s cooperative between Native queries |
| Native call timeout | 5 s |

Runtime limits can tighten wire payload/horizon ceilings. Client timeouts and age
settings are ROS parameters; server byte/horizon/deadline settings are CLI options.
Never compare monotonic timestamps across machines. HTTPX phase timeouts and a
total elapsed-time check protect the worker; a late result is ignored. Invalidated
in-flight I/O occupies the one worker until it finishes/times out. No async action
buffering or automatic retry of a consumed request is implemented.

The logged HTTP round-trip duration includes server inference and response
decoding/validation; it is not a measurement of network propagation alone.

Errors use `{schema_version, code, message, request_id}`. Examples:
`UNSUPPORTED_VERSION`, `INVALID_SCHEMA`, `INVALID_JSON`, `NONFINITE_NUMBER`,
`PAYLOAD_TOO_LARGE`, `INVALID_IMAGE`, `ACTION_SHAPE_MISMATCH`,
`INVALID_ACTION_TIMING`, `IDENTITY_MISMATCH`, `DUPLICATE_RESPONSE`,
`STALE_RESPONSE`, `REQUEST_INVALIDATED`, `NETWORK_ERROR`, `TIMEOUT`,
`UNSUPPORTED_ACTION_PROFILE`, `UNSUPPORTED_NORMALIZATION`, `P2_RESOURCES_MISSING`,
`P2_IDENTITY_MISMATCH`, `STALE_ACTION` and `STATE_INVALIDATED`.

## Optional P2 mode

`disabled` returns `NOT_CHECKED` and never imports P2. `required` blocks on missing
resources, joint-limit violations, backend errors, unknown results, identity
mismatches or rejected segments. Copy and fill
`cable_robot_vla/config/p2_required.example.yaml` outside the source tree. Supply
absolute paths to the existing workspace_analysis root, model configuration,
scene, SRDF and actual Native executable.

The adapter calls `KinematicModel.validate_q` and the existing
`ConfigurationValidity.path([current_q, *targets], resolution, fresh=True, ...)`.
Thus the initial transition and all later segments use P2's intermediate-state
checks. It creates no FK/IK/FCL/RRT replacement and performs no path optimization.
Unknowns never become accepted motion. The same immutable Native context is
used serially and closed at shutdown.

Configure the mock robot's `scene_id`/`scene_sha256` to the loaded SceneSnapshot,
and `model_id` to `urdf:<expanded_urdf_sha256>`. The robot's starting joints must
be explicitly configured to an appropriate simulated state. Zero is not a
physical home or an assumed collision-free pose. `gripper_rad` must match the
simulated gripper used for the P2 session.

Strict SRDF is the default. Candidate semantics require explicit
`allow_candidate_semantics:true` and retain their unverified research identity.
The real Native integration test uses this opt-in explicitly. It does not alter
the SRDF or authorize hardware execution.

`workspace_enabled:true` additionally requires an explicit task file and
`world_from_base` transform; it borrows P2's existing TCP workspace constraint.
General joint-space screening does not enforce port pose/approach constraints at
every state. Provenance explicitly reports `task_pose_checked:false` and whether
workspace checking was enabled. Full task-feasibility-aware VLA planning remains
future research.

Launch with `validation_mode:=required p2_config_path:=/absolute/p2.yaml`, matching
scene/model launch arguments, and `advance_simulation:=true`. No unvalidated
simulation permission is needed for a passing P2 research candidate. Configure
the robot's initial joints/gripper via a ROS parameter file supplied as
`config_file:=/absolute/local_mock.yaml` when required.

## Recording and acceptance commands

Each run creates a new JSONL file without overwriting another. Records include
observation identity and image SHA256/descriptor, raw policy action metadata and
values, encoding/round-trip/inference duration, adaptation/validation evidence,
mock execution events and errors. Images are not repeatedly embedded in JSONL;
the default deterministic fixture can be regenerated, while custom assets must
be retained separately by the experiment owner.

Run from the repository root with ROS sourced and `VLA_PYTHON` selected:

```bash
"$VLA_PYTHON" deployment/local/run_tests.py unit
"$VLA_PYTHON" deployment/local/run_tests.py ros

# Additional real P2 integration, with the existing model overlay sourced:
export IDC_P2_ROOT="$PWD/workspace_analysis"
export C1_BACKEND=/absolute/path/to/collision_backend
"$VLA_PYTHON" deployment/local/run_tests.py p2

# Separate-process manual acceptance probe; output directory must be NEW:
"$VLA_PYTHON" deployment/local/smoke.py --output-dir /tmp/idc-vla-smoke-01 --domain 72
# Verify the installed launch entry point as well:
"$VLA_PYTHON" deployment/local/smoke.py --launch --output-dir /tmp/idc-vla-launch-01 --domain 72
```

Unit transport tests use named HTTP fakes. ROS tests use real DDS and actual HTTP
sockets, including the default hold policy and an explicitly test-only 0.01-rad
movement fixture. P2 tests distinguish real Native model queries from the real
P2 interpolator with a fake collision oracle. Missing native resources fail the
explicit integration command; they are never silently skipped or replaced.

The smoke probe starts three owned processes, observes repeated completed cycles,
captures the private graph and JSONL, and verifies clean process exit. Its passing
status covers software integration only. See `docs/vla_stage1_verification.md`
for actual development verification results.

Existing P2 groups remain separate: run `core`, `integration` and `artifact`
using `workspace_analysis/repro/README.md`; its default pytest selects only core.

## Stage 2 handoff

Keep the local ROS mock stack unchanged and set
`server_url:=http://REMOTE_HOST:8000` (or an HTTPS endpoint) at launch. DDS can stay
localhost-only; HTTP is independent. The Stage 1 service is unauthenticated and
defaults to loopback. Decide the transport/access configuration before exposing
the GPU service beyond the development host.

Implement a real model `Policy` in `vla_server/policies`, with a separate,
version-pinned LeRobot environment and checkpoint directory. Use official
pre/postprocessors and report native semantics honestly. Add a robot mapping
only after dimensions, channels, units, normalization and calibration are
explicitly established. Until then, log genuine outputs and reject execution.

References: [Pydantic configuration](https://docs.pydantic.dev/latest/api/config/),
[FastAPI error handling](https://fastapi.tiangolo.com/tutorial/handling-errors/),
[HTTPX timeout semantics](https://www.python-httpx.org/advanced/timeouts/).
