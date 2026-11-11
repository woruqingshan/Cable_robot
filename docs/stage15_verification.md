# Stage 1.5 implementation and verification

Verified locally on 2026-11-11, Ubuntu 22.04, Python 3.10.12 and ROS 2 Humble.
All inference performed here used explicitly synthetic Mock/Fake policies.
No physical robot, model weights, LeRobot installation or GPU inference was used.

## A. Git and preservation

Started from `feat/vla-integration` at
`b48c99e5bc1affcbddd4cfaee9f5fe4e966c0964`. Created
`feat/vla-cloud-readiness` from that exact commit. Implementation commits:

| Commit | Change |
| --- | --- |
| `85bf0f7` | Native action/source profiles, explicit registry, receive/selection limits |
| `19ae910` | Policy factory, optional official binding, configuration and lifecycle |
| `aec2e2e` | Cloud tooling, tests and read-only ROS/RViz preview |
| `40d705a` | Actual-render-verified RViz text spacing correction |

Documentation is committed separately after verification. No push, merge, PR,
reset or rebase was performed. Stable `feat/vla-integration` remains at `b48c99e`;
`main` remains at `2aea40e1d7b73bffe7a0cd00c31224a4d613c0ea`.
The unrelated untracked `docs/develop/` and `workspace_analysis/results.zip`
remain untracked and preserved. Original tests, P2 source/results, vendor assets
and hardware package files were checked against the initial SHA256 snapshot.
Runtime/build/log evidence is outside the repository. Final tracked source is
clean after the documentation commit.

## B. Architecture

The existing ROS client sends canonical observations to a configuration-selected
policy. `vla_server` alone owns the optional model loader and official processors.
The response is a model-native `PolicyActionChunk`. The local explicit registry
can produce a `RobotActionChunk` only for a known semantic/calibration mapping;
the unchanged optional P2 bridge produces a `ValidatedTrajectory` for Mock use.
Native records are retained even when adaptation fails. The server has no ROS or
motor-command dependency.

The optional SmolVLA adapter delegates loading, observation preparation,
preprocessing, chunk prediction and postprocessing to reviewed official APIs.
Lazy dependencies and offline-only artifacts keep Mock startup independent of
Torch/LeRobot. Startup failure keeps HTTP diagnostics available with an unready
policy; there is no Mock fallback. One policy thread owns lifecycle/inference;
session transitions, cancellation, stale work and temporal reset are explicit.

Read-only preview independently consumes an accepted robot-space trajectory or
explicit synthetic fixture. It uses unchanged P2 model/FK APIs, the external
official `jetrover_description`, private JointState/TF topics and RViz. It never
interprets raw native outputs or authorizes execution.

See `stage15_cloud_readiness.md` for the source map, wire extension capabilities,
source/calibration boundaries, horizon provenance, timing and operating commands.
See `../deployment/cloud/README.md` for the independent cloud environment,
artifact pins, bootstrap, checkpoint receipt, SSH tunnel and Docker workflow.

## C. Code and compatibility

* Protocol v1 retains strict default Stage 1 wire behavior. A new test loads the
  actual strict old protocol from `b48c99e` and checks default requests/responses.
  Optional capabilities enable native horizon/session/source metadata.
* Native horizon supports up to 256 steps, while default receive and robot
  execution horizons remain 32. Full transmission is required. Local prefix
  selection is explicit and recorded; no resampling/padding/reordering is added.
* Source profiles distinguish synthetic, real uncalibrated and declared real
  calibrated observations. Calibration ID/hash is provenance, not physical proof.
  Mock execution continues to require synthetic uncalibrated state.
* `action_adapter.py` registers exact channels, units, semantics, processing,
  normalization, target and calibration. Only the previous synthetic identity
  mapping ships. Equal dimensions cannot establish compatibility.
* New `config.py`, `policy_factory.py`, `observation_adapter.py`, `session.py`,
  `artifacts.py`, `deployment.py` and `policies/{fake,smolvla}.py` isolate model and
  deployment logic. Fake is explicitly synthetic and is not SmolVLA inference.
* New cloud configs/scripts/tests provide a standard-library CPU installer,
  exact HTTP dependency lock, guarded GPU manifest/source/model operations and
  HTTP smoke entry point. GPU pins stay unresolved rather than fabricated.
* New `visualization/`, launch/config and local fixture/preview probes add timed
  JointState playback, source provenance, bounded offline selection, live
  read-only preview and native RViz capture. No Gazebo or controller was added.

Two small justified additions beyond the suggested filenames are a Fake policy
configuration and a pure description helper. The latter avoids importing
Pydantic into ROS's system launch interpreter. No system package was installed
to solve that interpreter boundary.

## D. Fresh verification

Every result below was rerun for Stage 1.5; none is copied as a new result from
the historical Stage 1 report. No test was weakened, removed or skipped.

| Group | Passed | Time |
| --- | ---: | ---: |
| Original Stage 1 unit/server/edge/isolation/P2 adapter | 83 | 1.04 s |
| Original Stage 1 ROS/HTTP | 3 | 3.96 s |
| Original Stage 1 native P2 bridge | 3 | 1.04 s |
| New Stage 1.5 protocol/server/edge/cloud | 44 | 4.05 s |
| New Stage 1.5 ROS Fake/preview integration | 3 | 7.70 s |
| Existing P2 core | 568 | 24.59 s |
| Existing P2 integration | 194 | 25.81 s |
| Existing P2 artifact | 79 | 891.29 s |

Totals: **47 new tests, 89 Stage 1 regressions and 841 P2 regressions passed**.
P2 marker selection deselects other groups; it does not skip tests. Warnings were
the Starlette/AnyIO deprecated portal alias and the existing URDF-parser private
variable warning. No failing test remains. `git diff --check` also passed.

### Exact test/build command forms

Commands were run from the actual feature checkout. The following variables
name the absolute verification paths used on this workstation:

```bash
export REPO=/home/siyuen/Desktop/code/cable_robot/cable_vla_robot
export CHECK=/home/siyuen/Documents/Codex/2026-11-11/idc-vla-project-research-context-p3/work
export VLA_PYTHON="$CHECK/venv/bin/python"
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd "$REPO"
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B -m colcon \
  --log-base "$CHECK/stage15_build_logs" build \
  --base-paths "$REPO/cable_robot_vla" --packages-select cable_robot_vla \
  --build-base "$CHECK/stage15_build" --install-base "$CHECK/stage15_install" \
  --symlink-install
source "$CHECK/stage15_install/setup.bash"
export IDC_VLA_INSTALL="$CHECK/stage15_install"
export IDC_P2_ROOT="$REPO/workspace_analysis"
export C1_BACKEND="$IDC_P2_ROOT/results/c3/assembly_build_20261104/build/collision_backend"
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py unit
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py ros
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py p2
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py stage15-unit
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/run_tests.py stage15-ros

cd "$IDC_P2_ROOT"
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

Only `cable_robot_vla` was built: one package succeeded in 1.19 s. Existing P2
uses its unchanged environment/native backend/replay install. No ROS, driver or
CUDA environment was modified.

### End-to-end and hardware isolation evidence

1. **Mock success:** the original three ROS tests and a separate installed-launch
   probe completed three `SIMULATED_UNVALIDATED` cycles. Explicit simulation
   advancement is enabled; Hold Position naturally retains the same numerical
   joint positions while the simulated state revision advances. The launch and
   server exited 0; launch children finished cleanly.
2. **Native rejection:** two new actual HTTP/DDS tests use absolute and delta
   Fake `[50,7]` chunks. Three candidates per scenario are recorded then rejected
   with request correlation. Later requests continue; the simulated state
   revision does not advance even when advancement flags are deliberately enabled.
3. **Preview:** the installed launch test checks exact names/order/timing, all
   independent joints, private TF endpoints against existing P2 FK (1e-9
   tolerance), invalid live native-input rejection and clean shutdown.
4. **Real RViz rendering:** external JetRover assets and DISPLAY were available.
   The final native `RenderWindow.captureScreenShot` shows the green robot and
   clearly visible synthetic/read-only labels. The probe observed **263 distinct
   arm positions in 263 JointStates**, 262 statuses, private preview topics and
   clean exit 0. Loaded model SHA256:
   `5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`.
   Text spacing was corrected after inspecting actual captures, then recaptured.
5. Original AST isolation checks cover the expanded package. Preview graph checks
   found no servo/kinematics/execute endpoints; the existing isolated
   `/idc_vla/mock/execute` interface remains simulation-only. Only read-only
   JointState/TF/marker/status preview interfaces were introduced. No hardware
   was accessed.

Acceptance command forms, after sourcing the overlay above:

```bash
cd "$REPO"
export PYTHONPATH="$REPO:$REPO/vla_protocol/src:$REPO/vla_server/src:$REPO/cable_robot_vla:$PYTHONPATH"
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/smoke.py --launch \
  --output-dir "$CHECK/stage15_evidence/mock_launch" --domain 72
PYTHONDONTWRITEBYTECODE=1 "$VLA_PYTHON" -B deployment/local/preview_smoke.py \
  --workspace-root "$REPO/workspace_analysis" \
  --output-dir "$CHECK/stage15_evidence/rviz_preview_verified" \
  --render-binary "$REPO/workspace_analysis/results/rviz_replay/rviz_snapshot_build_20261109/rviz_snapshot"
```

### Cloud restoration evidence

The standard-library installer actually created a fresh isolated Python 3.10.12
venv and installed the exact base HTTP lock plus our two Python packages. A clean
local clone of the committed feature branch verified tracked-source completeness:

```bash
git clone --no-hardlinks --single-branch --branch feat/vla-cloud-readiness \
  "$REPO" "$CHECK/stage15_clean_clone"
cd "$CHECK/stage15_clean_clone"
env -u PYTHONPATH -u PYTHONHOME python3 deployment/cloud/scripts/install_server.py \
  --venv "$CHECK/stage15_fresh_env" --execute
export VLA_PYTHON="$CHECK/stage15_fresh_env/bin/python"
bash deployment/cloud/scripts/start_server.sh --config deployment/cloud/config/mock.yaml
# In another terminal, with the same VLA_PYTHON:
"$VLA_PYTHON" deployment/cloud/scripts/smoke_test.py --url http://127.0.0.1:8000
```

The owned acceptance probe used free local ports, tested both Mock and Fake
through these same entry points, queried actual health/inference/version errors,
then sent SIGINT and verified exit 0. The clean clone was fast-forwarded locally
to `40d705a` and the probe repeated. The installed modules resolve inside the
fresh venv; `torch`, `lerobot` and `rclpy` are absent. Initial environment creation
and repeat installer both passed. No GitHub clone of this unpublished branch was
claimed. GPU shell/config tests reject unresolved pins before network/install;
shell syntax, hashed lock validation and non-mutating installation plans passed.

Saved evidence outside Git includes `stage1_*.log`, `stage15_*.log`, `p2_*.log`,
`ros_build.log`, `server_bootstrap.log`, `clean_clone_install.log`,
`clean_clone_report.json`, `mock_launch/report.json`, and
`rviz_preview_verified/{report.json,rviz_actual.png}`. A compact copy accompanies
the delivered implementation report.

## E. Limitations

* No Python 3.12/CUDA/Torch/LeRobot environment, real checkpoint, real preprocessing
  execution or genuine GPU inference was verified. Official binding spies test
  delegation only. All real deployment pins remain explicitly unresolved.
* The Dockerfile is an unbuilt, GPU-unverified template requiring a base-image
  digest. An approved branch must first be published before a rented host can
  fetch it. Public network exposure, TLS/authentication and VPN infrastructure
  are not implemented; the supported access path is a localhost SSH tunnel.
* No real-model→JetRover mapping, servo calibration, physical TCP, camera
  calibration, perception-derived scene, physics simulation or manipulation
  capability is established. Synthetic fixtures and sampled P2 checks provide
  no continuous collision-freedom or hardware safety certification.
* A hung model thread cannot be hard-killed. Inference is serialized, invalid
  results are discarded and shutdown waits; future process isolation is needed
  for hard containment. No action buffer, RTC or multi-client concurrency policy
  for shared real hardware was added.
* The local vendor description and GUI were available and verified; they remain
  external runtime dependencies. No vendor resource was copied or modified.

## F. Stage 2 readiness — explicit answers

1. **Can a new GPU server reproduce the environment?** The committed source,
   CPU HTTP lock and documented bootstrap reproduce the tested Mock service.
   The unpublished branch must first be authorized/published. A real GPU/model
   environment cannot yet be called reproducible: select and verify all null
   source/checkpoint/Python/Torch/CUDA/image/lock pins first.
2. **Which commands were verified?** CPU `install_server.py --execute` (fresh and
   repeated), Mock/Fake `start_server.sh`, health/inference/error smoke, clean
   shutdown, GPU fail-before-install paths, shell syntax and installation plans.
   Model download/install, Docker build and genuine GPU startup were not run.
3. **Where is LeRobot installed?** Independently, for example
   `/workspace/third_party/lerobot`, at an exact clean revision, in a separate
   Python 3.12 model venv. Never inside Cable_robot or the ROS environment.
4. **Where is the checkpoint stored?** Separately, for example
   `/data/models/smolvla_base`, with exact Hub revision and content receipt/hash.
   Model/backbone/tokenizer artifacts must all be available for offline loading.
5. **Which adapter loads/runs it?**
   `vla_server.policies.smolvla.SmolVLAPolicyAdapter`, selected by the factory,
   using official loader, processors, observation utility and chunk prediction.
6. **How is native output returned/inspected?** Check actual features and `[H,D]`,
   preserve processing/channel/unit/semantics/timing/horizon/model identity, return
   `PolicyActionChunk`, and record it through the model-inspection client. Unknown
   mappings are explicit adaptation failures; no robot conversion is inferred.
7. **How does ROS connect?** SSH-forward remote `127.0.0.1:8000` to local 18000;
   set `server_url:=http://127.0.0.1:18000` and the explicit inspection profile.
   Tune transport/inference independently while preserving stale-action gates.
8. **What proves GPU integration?** Restore pinned dependencies/artifacts, load
   the real model and processors, execute genuine observation inference and
   reset/session tests, inspect actual action semantics, pass HTTP and local ROS
   inspection/failure probes, measure latency and verify resource cleanup.
9. **What follows loading?** Resolve demonstrated dataset/action semantics,
   calibrated mappings and task feasibility integration where valid; subsequently
   add real perception, physical calibration, task data/fine-tuning and the chosen
   constraint-aware research method. Real execution and RJ45 evaluation remain
   later work. Successful software integration alone proves no manipulation claim.
