# Stage 1 development verification — 2026-11-11

## Git baseline and scope

Repository: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot`.
Original `main` and feature-branch base:
`2aea40e1d7b73bffe7a0cd00c31224a4d613c0ea`.
Branch: `feat/vla-integration`. No pull, merge, rebase, push or PR was performed.

Implementation commits:

* `d303f4b` — versioned protocol and deterministic Mock HTTP Policy Server.
* `3783c4c` — ROS mock loop, optional P2 validation, test/deployment tools.
* `88e403b` — serialized policy lifecycle after HTTP timeouts, plus real TCP
  workspace binding coverage.
* Documentation is committed separately after verification.

All 313 pre-existing tracked files were SHA256 checked against the initial
checkout and remained byte-for-byte unchanged. Existing untracked
`docs/develop/` and `workspace_analysis/results.zip` were neither staged nor
modified. No vendor description, P2 algorithm, historical result or control code
was changed. New generated logs/builds/environments live outside the repository
in this chat's `work/` directory. Package build metadata is ignored.

## Implemented flow and source map

`MockRobotNode` publishes real ROS `Image`, five-joint `JointState`, separate
gripper state and stamped task metadata. `ObservationAdapter` enforces source
freshness and synchronization. `PolicyClient` sends real HTTP requests on one
worker. FastAPI calls `Policy.predict`; the default `HoldPositionPolicy` returns
deterministic candidate values with explicit semantics and identity.

`action_adapter` permits only the declared synthetic absolute-radian mapping.
`MockExecutor` invokes optional `P2Adapter`, then rechecks current state/context
and age. The mock robot consumes the validated simulation message, rechecks its
own current state and advances timed waypoints. New state is observed in the
next inference cycle. JSONL records correlation, timing, adaptation, validation,
execution and failure evidence without repeated image payloads.

* `vla_protocol/src/vla_protocol`: observation/action/profile/error/codec models
  and a bounded standard-library RGB8 PNG validator/encoder.
* `vla_server/src/vla_server`: FastAPI app, Policy lifecycle/interface and hold
  policy. GPU and ROS dependencies are absent.
* `cable_robot_vla/cable_robot_vla`: ROS nodes, snapshot collection, HTTP worker,
  action adapter, recorder, simulation state machine, isolation guard and P2 adapter.
* `cable_robot_vla/config`, `launch`: conservative defaults and a configurable
  two-node launch; default behavior records without advancement.
* `deployment/local`: pinned dependencies, explicit test-group runner and
  separate-process acceptance probe.
* `deployment/cloud`: Stage 2 integration boundary documentation only.

P2 reuse is limited to existing model/scene/SRDF loading, joint-limit validation,
sampled path checking and optional TCP workspace checking. No FK, collision,
planning or optimization algorithm was duplicated. The observation adapter is
hosted in the VLA node rather than introducing a third ROS process.

## Fresh automated results

The following counts were executed during this development, not copied from
historical reports. Tests were not weakened or skipped to obtain these results.

| Verification | Actual result |
| --- | --- |
| Stage 1 unit/server/failure/isolation group | **83 passed**, 1 dependency deprecation warning, 0.78 s |
| Stage 1 real ROS + HTTP group | **3 passed**, 3.70 s |
| Stage 1 P2 adapter integration group | **3 passed**, 1 existing parser warning, 1.03 s |
| Existing P2 core | **568 passed**, 273 deselected, 25.38 s |
| Existing P2 integration | **194 passed**, 647 deselected, 1 existing parser warning, 26.83 s |
| Existing P2 artifact | **79 passed**, 762 deselected, 1 existing parser warning, 896.68 s |
| New ROS package build | **1 package finished**, 1.12 s |
| Protocol/server wheel builds | Both built and installed successfully |
| Application dependency lock | All pinned distribution versions matched the tested environment |

Totals: **89 new Stage 1 tests passed** and **841 existing P2 tests passed**
across the three separately selected P2 groups. There were no skipped tests.

The ROS tests include actual localhost HTTP sockets and DDS for the hold policy,
a separate controlled test-only 0.01-rad movement fixture, repeated observation/
execution cycles, observable changed joint positions and task revision changes.
They also reject a command-topic remap outside the mock namespace.

The P2 group includes real official-model/Native FCL hold-path and joint-limit
rejection cases both with and without the task-TCP workspace constraint. Another
test uses the real P2 interpolator with an
explicit fake collision oracle to verify rejection inside both the initial
transition and a later segment. Unit fakes separately cover backend errors,
unknown results, missing Native executable and scene/model mismatch. Fakes are
not described as physical or real Native evidence.

The unit warning comes from Starlette's deprecated AnyIO BlockingPortal alias.
P2 warnings come from its existing URDF parser/private-variable compatibility
warning. They were reported, not suppressed.

## Commands executed

The Stage 1 interpreter was an isolated Python 3.10.12 environment under the
chat workspace, with `--system-site-packages` for ROS. The dependency lock is in
`deployment/local/requirements.lock`; `packaging==24.2` is needed with the pinned
setuptools version. No system Python, P2 venv or GPU environment was modified.

From the feature-branch root, with ROS Humble sourced and `VLA_PYTHON` pointing to
that isolated interpreter:

```bash
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py unit
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py ros

source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
export IDC_P2_ROOT="$PWD/workspace_analysis"
export C1_BACKEND="$IDC_P2_ROOT/results/c3/assembly_build_20261104/build/collision_backend"
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py p2
```

The final package was built from the actual repository's `cable_robot_vla`
source using `python -B -m colcon`, selecting only this package and placing build,
install and logs under the chat's `work/final_build`, `work/final_install` and
`work/final_build_logs`. Both protocol/server packages were installed with
`pip install --no-build-isolation --no-deps` in the isolated environment.

Existing P2 was tested in its own unchanged `.venv`, from `workspace_analysis`:

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m core tests
.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m integration tests \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend

source results/rviz_replay/build_t53_c_20261109/install/setup.bash
.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m artifact tests \
  --artifact-root "$PWD/results" \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source \
  --replay-install results/rviz_replay/build_t53_c_20261109/install
```

The first artifact attempt was blocked by the missing explicit replay install.
A subsequent partial run reported package-discovery failures because that install
had not been sourced. The final command above supplies and sources it; P2 source
and assertions were left unchanged.

## Separate-process acceptance and hardware isolation

Two acceptance probes were executed from the actual feature-branch checkout:

```bash
"$VLA_PYTHON" -B deployment/local/smoke.py --domain 72 \
  --output-dir "$EVIDENCE_ROOT/process_smoke"
source "$EVIDENCE_ROOT/final_install/setup.bash"
"$VLA_PYTHON" -B deployment/local/smoke.py --launch --domain 72 \
  --output-dir "$EVIDENCE_ROOT/launch_smoke_final"
```

`EVIDENCE_ROOT` was
`/home/siyuen/Documents/Codex/2026-11-11/idc-vla-project-research-context-p3/work`.

* Direct startup: **PASS**, three completed cycles; robot, pipeline and server
  all exited 0.
* Installed launch: **PASS**, three completed cycles; launch and server exited
  0, and both ROS child nodes explicitly finished cleanly.
* Hold-position values remained `[0,0,0,0,0]`, as intended, while execution/state
  revisions advanced. The separate ROS movement fixture verified changed values.
* Default disabled screening was recorded as `NOT_CHECKED`; advancement was
  explicitly opted into and labelled `SIMULATED_UNVALIDATED`.

The launch probe sends SIGINT to the launch parent once, allowing launch to
forward it to its nodes. An earlier group-signal probe delivered duplicate
interrupts; the probe was corrected to check child-node cleanup as well as the
launch parent's exit code. The final result above uses that corrected check.

Observed application topics were only:

```text
/idc_vla/mock/execute
/idc_vla/mock/execution_status
/idc_vla/mock/gripper_state
/idc_vla/mock/joint_states
/idc_vla/mock/outcomes
/idc_vla/mock/rgb
/idc_vla/mock/task
/idc_vla/mock/task_instruction
```

Standard `/parameter_events` and `/rosout` were also present. No servo, bus-servo
or kinematics command topic was present. Runtime tests observed zero command
clients on mock nodes, and AST tests rejected hardware/GPU imports. No vendor
controller nodes or hardware-facing control classes were started.

Raw evidence is under `EVIDENCE_ROOT`: `stage1_unit.log`, `stage1_ros.log`,
`stage1_p2.log`, `p2_core.log`, `p2_integration.log`, `p2_artifact_final.log`,
`process_smoke/` and `launch_smoke_final/`. Generated evidence is not committed.

## Limitations and Stage 2 handoff

This is a cloud-edge **software simulation foundation**. It does not establish
physical safety, continuous collision freedom or successful RJ45 manipulation.
The default scene/model labels are synthetic; the all-zero scene hash denotes
no loaded geometric scene. Required P2 mode needs actual matching identities.

P2 validation is `DISCRETE_ONLY`, with its existing uncalibrated TCP and research
collision semantics. General path screening does not prove port alignment or
task feasibility; provenance explicitly reports whether workspace checking was
enabled and that task-pose checking was not performed. Mock trajectories use
timed waypoint updates, with no physics, calibrated servo mapping or contact model.

No real SmolVLA inference, GPU deployment, data collection, training, RGB-D scene
reconstruction or hardware execution was implemented. No weights were downloaded.
The remaining GPU-stage work is environment/model pinning, official model
pre/postprocessing, a real `Policy` adapter, native-action identification and an
explicit compatible robot mapping. Unmapped real-model actions remain rejected.

Use `docs/vla_integration.md` to start the local server/client. For Stage 2, set
the same client's `server_url` launch argument to the remote HTTP(S) endpoint;
keep LeRobot/CUDA/Torch in an independent server environment. Decide remote
transport/access settings before exposing the currently loopback-only,
unauthenticated development service.
