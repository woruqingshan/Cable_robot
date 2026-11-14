# Stage 2.3 — private Gazebo, ROS, RViz and genuine SmolVLA

The local integration preserves the Stage 2.1/2.2 HTTP API and native `[50,6]`
actions. No model-to-JetRover mapping is invented. The two independent paths are:

```text
Gazebo RGB + JointState + sim clock + language
  -> SensorObservationSource -> existing PolicyClient -> SSH -> genuine SmolVLA
  -> native JSON + channel array + RViz plots -> unsupported-mapping rejection

Explicit rule-generated joint trajectory (no model action values)
  -> registered synthetic ActionInterpreter -> existing native P2/FCL validator
  -> enabled Gazebo-only ExecutionBackend -> controller setpoints
  -> observed JointState + rendered image -> next cloud observation
```

## Stable local boundaries

| Interface | Current adapters | Future hardware boundary |
|---|---|---|
| `ObservationSource.snapshot(sequence, now_ros_ns)` | Synthetic, Gazebo and uncalibrated real sensor collection through `SensorObservationSource` | Replace sensor/topic adapter; preserve timestamps/frame/profile and validity checks |
| `PolicyClient.submit/poll/invalidate/close` | Existing bounded HTTP worker, unchanged | Same wire `Observation` / `PolicyActionChunk` |
| `ActionInterpreter.interpret(candidate)` | Existing explicit mapping registry | Register only a reviewed calibrated policy/robot mapping; unknown actions remain rejected |
| `TrajectoryValidator.validate(action, observation)` | Existing `P2Adapter` and original URDF/FCL/workspace checker | Same identity binding, scene/model/calibration provenance |
| `ExecutionBackend.submit/tick/cancel/close` | Read-only inspection or explicitly enabled Gazebo | A separately implemented controller adapter; hardware factory currently always rejects |

Interpretation and validation finish before any setpoint is published. Validation
cancellation carries a generation number so a late P2 result cannot revive a
cancelled trajectory. Runtime identity, sensor freshness, context and tracking
error are checked before commands continue. Position controllers retain their
last setpoint on a fault; queued waypoints are discarded and never auto-resume.

The Gazebo node subscribes to the private clock and uses a **steady-clock watchdog**
so pausing simulation does not disable stale-stream detection. Joint samples are
buffered and paired to actual camera timestamps. Original reception times, source
stamps and image frame IDs are retained. Invalid images/joints clear their stream;
clock rewind creates a new policy session and requires a new image. No fabricated
synchronization timestamps are introduced.

Gazebo observations are synthetic, uncalibrated sensor data. They use the existing
`synthetic_uncalibrated_v1` wire profile, with explicit local `source_kind: gazebo`
provenance. The generic sensor adapter can produce `real_uncalibrated_v1` with the
same schema, but a real JetRover sensor deployment and calibration remain unverified.
The current cloud configuration intentionally accepts only synthetic observations.
No shared protocol or cloud-server change is necessary for backend selection.

## Model and validation scope

Use the original installed description overlay:

```bash
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH="$HOME/Desktop/jetrover_sim_ws/install/jetrover_description:$AMENT_PREFIX_PATH"
export REPO="$HOME/Desktop/code/cable_robot/cable_vla_robot"
export P2="$REPO/workspace_analysis"
export C1_BACKEND="$P2/results/c3/assembly_build_20261104/build/collision_backend"
export LOCAL_PY=REPLACE_WITH_EXISTING_LOCAL_PYTHON310_VENV/bin/python
```

The verified local Python is the existing Stage 1/2 test environment at
`~/Documents/Codex/2026-11-11/idc-vla-project-research-context-p3/work/venv/bin/python`.
Do not install cloud LeRobot/Torch into ROS Python 3.10.

P2's semantic review is bound to the exact expanded URDF hash
`5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`.
A different ament resource prefix can change one absolute visual mesh URI and
therefore the hash. The integration preserves the existing binding; it does not
rewrite the semantic manifest, relax collision rules or silently accept a new
model. All robot geometry remains external and unmodified.

The generator loads the existing IDC scene, exports its exact primitives into
Gazebo, and records model/scene/backend/semantic identities. It probes native P2
for a valid starting pose and a self-collision rejection fixture, then validates
both a known trajectory and a joint-limit rejection fixture. Workspace checks
are enabled. Accepted paths are **PASS_RESEARCH_ONLY / DISCRETE_ONLY**, using the
existing candidate ACM baseline; they are not physical safety certificates or
continuous-collision proofs. Existing coverage limitations, including missing
lidar collision geometry and unverified gripper/ACM semantics, remain in effect.

Gazebo Fortress 6.16 uses the original converted robot model with a fixed base,
zero gravity and bounded ideal position/velocity controllers. A small Gazebo-only
plugin initializes joints directly at the P2-validated starting pose before the
first physics iteration. Mimic gripper joints are initialized and held at their
URDF-derived states. This is a kinematic integration fixture, not a calibrated
actuator/dynamics benchmark. The scene is synthetic. No fixture is a robot demo
for training or evidence of successful cable manipulation.

## Genuine cloud service and SSH

Start the previously verified service on AutoDL, without installation/downloads:

```bash
ssh -t autodl-vla '
export VLA_PYTHON=/root/autodl-tmp/idc-vla/venvs/vla-model/bin/python
export HF_HOME=/root/autodl-tmp/idc-vla/cache/huggingface
export HF_HUB_CACHE=$HF_HOME/hub
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 TOKENIZERS_PARALLELISM=false
bash /root/autodl-tmp/idc-vla/runs/stage2/checkouts/68d97f7/deployment/cloud/scripts/start_server.sh \
 --manifest /root/autodl-tmp/idc-vla/runs/stage2/deployment-68d97f7/manifest.yaml \
 --config /root/autodl-tmp/idc-vla/runs/stage2/deployment-68d97f7/server.yaml'
```

In another Ubuntu terminal:

```bash
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:18000:127.0.0.1:8000 autodl-vla
curl --fail http://127.0.0.1:18000/health
```

The service remains bound to remote `127.0.0.1:8000`; the tunnel is local-only.

## Automated inspection and simulation acceptance

Use a new external output directory for every run. An active local display is
needed for camera rendering and the optional native RViz screenshot helper.
The following commands reuse the existing RViz helper. The initialization plugin
is built locally by the runner from installed Gazebo headers; no downloads occur.
Pass `--plugin-dir /existing/build` to reuse an existing compiled plugin.

```bash
export RENDER="$P2/results/rviz_replay/rviz_snapshot_build_20261109/rviz_snapshot"
export RUN="$HOME/Documents/Codex/stage23-NEW"

# Real Gazebo sensors + real GPU actions + RViz; zero command publishers in the node.
"$LOCAL_PY" "$REPO/deployment/local/stage23_demo.py" \
 --backend "$C1_BACKEND" --output-dir "$RUN-inspection" \
 --render-binary "$RENDER" --fault-tests

# Separate rule-generated, P2-validated simulation trajectory and feedback loop.
"$LOCAL_PY" "$REPO/deployment/local/stage23_demo.py" \
 --backend "$C1_BACKEND" --output-dir "$RUN-execution" \
 --render-binary "$RENDER" --enable-simulation --fault-tests
```

The runner owns and cleans up Gazebo, bridge, robot_state_publisher, integration
node, fault-test node and RViz processes. It refuses existing output directories.
Network fault testing uses an unused localhost port. Sensor fault testing stops
only its own bridge and confirms the steady watchdog rejects stale inputs without
new commands. Reports distinguish genuine inference, native rejection, synthetic
scenario control, actual simulator feedback and native P2 results. With no
`--render-binary`, marker/data exposure is tested but actual visual rendering is
explicitly unverified. Full visual acceptance uses the screenshot helper.

## Interactive ROS launch

Prepare a fresh validated runtime/plugin without starting the GPU or simulator:

```bash
"$LOCAL_PY" "$REPO/deployment/local/stage23_demo.py" --prepare-only \
 --backend "$C1_BACKEND" --output-dir "$RUN-live"

# Build only this ROS package into isolated directories; no vendor drivers.
colcon --log-base "$RUN-build-logs" build --base-paths "$REPO/cable_robot_vla" \
 --build-base "$RUN-build" --install-base "$RUN-install" --symlink-install
source "$RUN-install/local_setup.bash"
export PYTHONPATH="$REPO/vla_protocol/src:$PYTHONPATH"
ros2 launch cable_robot_vla stage23.launch.py \
 runtime_path:="$RUN-live/runtime/runtime.json" plugin_dir:="$RUN-live/plugin-build" \
 python_executable:="$LOCAL_PY" log_dir:="$RUN-live/live-loop"
```

Default launch is inspection-only. `enable_simulation:=true` explicitly enables
the independent known-semantic scenario, after two genuine native chunks have
been inspected. It never maps those native values into joints. Ctrl-C shuts down
all owned local components. No hardware execution backend is implemented.

## Inspectable topics

All endpoints are under `/idc_vla/sim` on a private localhost DDS domain. Each
world has its own Ignition partition. Topic remaps are checked against exact names.

| Topic | Data |
|---|---|
| `camera/image`, `joint_states`, `clock` | Actual simulator sensors and time |
| `instruction` | Bounded language text, changes task revision |
| `observation_state` | Submitted joint state, image hash/frame, original stamp/revision |
| `native_actions` | Full genuine native ActionChunk JSON |
| `native_values` | Float64MultiArray `[horizon, native_channel]` |
| `markers` | Clearly labeled native-channel plots and latency/status; no inferred robot pose |
| `validation` | Native `NOT_INTERPRETABLE` or actual P2 research validation |
| `status` | Request, rejection, independent scenario and feedback status |
| `tf`, `tf_static`, `robot_description` | Actual Gazebo feedback displayed with original URDF |
| `control/joint1..joint5/position` | Simulation-only bridge topics, published by execution mode only |

These are not vendor servo, base velocity or FollowJointTrajectory endpoints.
The simulation module creates no physical controller clients. Switching observation
or execution adapters preserves the shared protocol. Future hardware work must
supply a reviewed native-to-robot mapping, calibrated sensor/state/timing data,
updated P2 context and a separately reviewed executor before any physical action.
