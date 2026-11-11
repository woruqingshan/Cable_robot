# Stage 1.5 — VLA compatibility, cloud reproducibility and read-only RViz

Stage 1.5 provides three modes: **Mock**, **model-native inspection**, and
**read-only trajectory preview**. All performed inference/execution tests are
synthetic. Genuine SmolVLA/GPU inference, physical motion, RGB-D reconstruction,
calibration, constraint-aware research algorithms and Gazebo physics are not
implemented or claimed.

## Architecture and source map

```text
ROS Mock Robot → canonical Observation → HTTP Policy Client
    → configured Policy Factory → model feature binding → Policy
    → PolicyActionChunk → explicit Action Adapter Registry
    → RobotActionChunk → optional unchanged P2 checker → Mock Executor

Recorded accepted RobotActionChunk / explicit synthetic fixture
    → read-only preview → private JointState / TF → RSP → RViz
```

* `vla_protocol`: optional v1 capability/source/horizon metadata; strict legacy
  defaults remain. Native and robot actions are still different types.
* `vla_server/config.py`, `policy_factory.py`: strict YAML configuration selects
  `mock`, `fake` or optional `smolvla`; no GPU imports in Mock mode.
* `observation_adapter.py`: named camera/state feature binding and validation.
* `policies/smolvla.py`: offline official loader/processors/chunk prediction
  boundary, with explicit dependency/checkpoint errors and no fallback policy.
* `session.py`: one owned policy thread, reset/invalidation, idempotency and
  discarded late outputs. No asynchronous execution buffer or RTC was added.
* `cable_robot_vla/action_adapter.py`: explicit registry and local prefix selection.
* `visualization/`: bounded offline record selection, timed read-only preview,
  official model binding, private JointState/markers and live preview input.
* `deployment/cloud`: pinned base HTTP installer, policy configs, GPU manifest,
  non-destructive guarded framework/download scripts and actual HTTP smoke tool.

Existing P2/hardware files and Stage 1 test assertions remain unchanged. The
preview uses P2's existing model/joint/FK APIs without changing historical replay
or duplicating planning/FK algorithms. The ROS launch description loader avoids
Pydantic imports in ROS's system interpreter; the preview node runs under the
explicit protocol-aware Python interpreter.

## Protocol v1 compatibility and calibration

Default `codec.dumps` requests/responses omit inactive Stage 1.5 fields. Required
nullable framework/checkpoint fields remain present. The default wire was tested
against the strict protocol implementation from Stage 1 commit `b48c99e`.

Clients explicitly declare optional capabilities:

| Capability | Purpose |
| --- | --- |
| `native_actions_v1` | Explicit native horizon metadata, including long chunks |
| `source_profiles_v1` | Explicit synthetic/real/calibration source profiles |
| `session_state_v1` | Model-side session generation in responses |

Health advertises supported extensions. The server refuses to emit extended
native metadata to a client that did not opt in. This is capability declaration,
not automatic downgrade of a model output. Unknown versions/fields/capabilities
remain errors. New requests using extensions require a supporting server.

Source profiles distinguish `synthetic_uncalibrated_v1`, `real_uncalibrated_v1`
and `real_calibrated_v1`. Real observations require explicit matching legacy
status/gripper fields and source capability. Calibrated declarations require a
calibration ID and SHA256; those are provenance attestations, not proof that a
physical calibration was performed. The Mock executor remains synthetic-only.

## Native actions, local mapping and horizons

Legacy default receive horizon stays 32. Explicit native metadata plus configured
limits allow up to 256 native steps and 256 channels; robot execution stays at
most 32 five-joint steps. Relative offsets stay positive, increasing and ≤10 s.

`PolicyActionChunk.horizon` records model-generated/transmitted horizons. Stage
1.5 supports full transmission only: no server truncation, padding or resampling.
`RobotActionChunk.horizon` records generated, transmitted and locally selected
execution horizons. Full selection never truncates. `execution_selection:=prefix`
requires explicit `execution_steps` within both transmitted and robot limits;
selected values/timestamps are unchanged and provenance is recorded.

The registry requires native space, exact channel order, units, absolute/delta
semantics, processor, normalization, target profile, calibration requirement and
mapping version. The shipped mapping remains `synthetic_identity_v1`, requiring
five absolute-radian joints and Mock identity processing. Higher-dimensional,
real-model, unknown normalization, delta or otherwise unmapped outputs can be
recorded but cannot execute. Dimension matching alone grants no mapping.

## Model input and official APIs

`ModelFeatureSpec` explicitly binds camera IDs to feature names and image shapes,
state channel order/units/dimension, accepted source profiles and processor/model
identity. There is no guessed padding, state scaling or camera substitution.
Exact shapes are checked, or image resizing is explicitly delegated to a declared
official checkpoint policy. Fake feature specs test success and failure locally.

The SmolVLA adapter delegates tensor preparation, tokenization, normalization,
image processing and output processing to official LeRobot APIs. It invokes
`predict_action_chunk`, not the single-action queue API. Actual checkpoint feature
shapes and chunk size must match the supplied contract. No real checkpoint was
loaded. Binding-spy tests validate call structure only, not model inference.

Reviewed upstream reference:
[`ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2`](https://github.com/huggingface/lerobot/tree/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2),
including [model](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/policies/smolvla/modeling_smolvla.py),
[configuration](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/policies/smolvla/configuration_smolvla.py),
[processors](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/policies/smolvla/processor_smolvla.py)
and [observation utility](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/policies/utils.py).
Its default chunk size 50 and Python 3.12 requirement are API-review facts,
not selected deployment pins or evidence about a downloaded checkpoint.

## Lifecycle and timing policy

One server policy worker owns start, warmup, reset, prediction and close. A timed-out
thread cannot be forcibly killed: its result is discarded, later calls remain
serialized, and teardown waits for it. Hard termination of a hung model would
require process isolation, which is not implemented.

New/reset robot sessions retire the previous session. Task, scene/model or source
profile changes reset temporal state; older revisions/retired sessions are rejected.
Same request ID and identical input are idempotent; changed input with a reused ID
is rejected. Late, disconnected or failed inference dirties state and forces reset
before later prediction. Model action queues/processors are reset together.
Warmup is explicit, validates its output and resets before serving a robot session.

Mock settings remain HTTP/inference 2 s, observation age 0.5 s, synchronization
0.02 s, action age 5 s and inference interval 0.25 s. Service inference/startup/
warmup deadlines and client request/response/horizon settings are separate.
The cloud template's 30 s transport / 20 s inference values are unverified examples;
they do not relax the 5 s action-age gate. No remote monotonic clocks are compared.

## Operating commands

Use Stage 1 setup in `docs/vla_integration.md`, rebuild only the updated ROS package,
and source its install. `VLA_PYTHON` is the absolute Python 3.10 protocol environment.

### Mode A — unchanged Mock loop

```bash
export VLA_PYTHON=/absolute/path/to/local/venv/bin/python
bash deployment/cloud/scripts/start_server.sh --config deployment/cloud/config/mock.yaml
# In the ROS terminal:
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=71
ros2 launch cable_robot_vla local_mock.launch.py python_executable:="$VLA_PYTHON" \
  advance_simulation:=true allow_unvalidated_simulation:=true
```

### Mode B — native model inspection, without execution

```bash
bash deployment/cloud/scripts/start_server.sh --config deployment/cloud/config/fake.yaml
# In the ROS terminal, source ROS and the new install first:
ros2 launch cable_robot_vla local_mock.launch.py python_executable:="$VLA_PYTHON" \
  config_file:="$PWD/cable_robot_vla/config/model_inspection.yaml" \
  server_url:=http://127.0.0.1:8000 log_dir:="$PWD/log/vla/native_inspection"
```

The Fake policy produces synthetic `[50,7]` actions with explicit native channels,
units, normalization and horizon metadata. Observe `candidate_received` followed
by request-correlated `candidate_rejected`. The state does not advance and later
requests continue. The same inspection path will accept genuine structurally valid
model output later, without assuming a JetRover mapping.

### Mode C — read-only offline or live RViz preview

```bash
# Creates only a small explicit synthetic fixture; refuses overwrite:
PYTHONPATH="$PWD:$PYTHONPATH" "$VLA_PYTHON" deployment/local/make_preview_fixture.py /tmp/p3_preview.json
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=73
ros2 launch cable_robot_vla trajectory_preview.launch.py \
  workspace_root:="$PWD/workspace_analysis" input_path:=/tmp/p3_preview.json \
  allow_synthetic:=true loop:=true python_executable:="$VLA_PYTHON"
```

The external `jetrover_description` overlay must already be sourced. Neither URDF
nor meshes are copied into the repository. RSP/RViz use private `/idc_vla/preview`
JointState, robot-description and TF topics. Labels distinguish unchecked synthetic
preview from research/discrete validation and always state read-only/no execution.
The cyan line depicts EE waypoint positions from existing P2 FK, not a calibrated
RJ45 TCP or a collision-free path proof.

For saved JSONL, set `input_path` to the run and `source_action_id` to one accepted
trajectory; ambiguous selection is rejected. For live read-only preview, omit
`input_path` and publish a full `ValidatedTrajectory` JSON String on
`/idc_vla/preview/candidate`. Raw PolicyActionChunks are never interpreted as joints.
Rejected/error trajectories are not playable. `allow_synthetic` is required for
`NOT_CHECKED` playback; this never grants execution authorization. `gui:=false`
supports headless transport/TF tests.

There is no Gazebo controller, physics simulation, `JointTrajectory` command
publisher or `FollowJointTrajectory` client. Future simulation execution must
consume the already interpreted RobotActionChunk behind a separate controller
boundary; it must not reuse this read-only preview as authorization.

## Cloud recreation and Stage 2 acceptance

See `deployment/cloud/README.md` for clone/pin/venv/bootstrap/model-download/
Docker/tunnel commands and their verified versus unverified status. The new
branch must be published through a later authorized action before another host
can fetch it. All GPU source/checkpoint/environment pins remain unresolved here.

Use an SSH-forwarded localhost endpoint, for example
`server_url:=http://127.0.0.1:18000`; never expose the unauthenticated service on a
public interface. No credentials or host addresses are stored in source.

Before claiming real GPU integration: select and restore a compatible pinned
environment/checkpoint, verify its inventory and official processors, load the
real model, execute genuine inference, identify actual action semantics, pass
HTTP/ROS inspection and session/failure tests, measure timing and verify cleanup.
Robot mappings, physical calibration, real scene/perception and RJ45 operation
remain later work. Collision filtering/Mock/RViz integration are engineering
infrastructure, not completed research contributions.

For exact fresh results and evidence, see `docs/stage15_verification.md`.
