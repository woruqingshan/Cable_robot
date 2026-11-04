# T5.2-R1 — Verified ROS2/RViz2 interactive replay

Status: software/automated checks and actual desktop smoke test; pending user acceptance.
Baseline HEAD: `00b4d768a680ad4add63572890752997e6854b37` on `feat/workspace-planning`.
All displayed motion remains RESEARCH_ONLY, fixed-base, discrete saved samples.

## Architecture and audit

Independent ament_python package: `ros2/jetrover_idc_visualization/`.
It installs only its visualization modules, launch and configs. The original
research package is explicitly imported from the `workspace_root` argument;
it is neither copied wholesale nor installed over the existing Python environment.
The original ROS workspace/official description are unchanged. All build output
is in a separate ignored results/rviz_replay directory.

```
saved T5.1 run + explicit input profile + existing Robot Model
   -> strict artifact reader + full existing verification
   -> immutable Catalog / original NPZ trajectories
   -> ReplayController (pure state machine)
   -> private ROS2 JointState + robot_state_publisher + TF
   -> SceneSnapshot markers + RViz2
Qt panel <-> versioned JSON command/status topics
```

Modules:

- `storage.py`: bounded no-follow local reads, relative-path constraints,
  strict JSON, hash/LFS-pointer checks. Artifact roots/components reject symlinks.
- `research.py`: explicit research root, rejects an already-imported other checkout.
- `reader.py`: hashes every listed file, checks the prior verification receipt,
  binds declared provenance inputs by content, invokes the FULL existing validator,
  checks R2 model/include/mesh identity, then loads immutable original NPZ arrays.
- `data.py`: Catalog/Port/Goal/Trajectory contracts, exact boundary join,
  URDF-derived independent joint mapping. RSP computes mimic states, not another FK.
- `controller.py`: STOPPED/PLAYING/PAUSED/FINISHED, selection/revision, play/pause/
  resume/reset and exact original frame/provenance index. No motion interpolation.
- `protocol.py`: fixed private topics, version/session/request/selection identities.
- `markers.py`: pure SceneSnapshot/FrameGraph geometry descriptors and a separate
  ROS MarkerArray conversion. Boxes/cylinders keep original transforms/dimensions.
- `node.py`: owned asynchronous verification, serial playback timer, status and
  private JointState/marker publishing. No hardware messages/services/actions.
- `panel.py`: Qt selectors/buttons, short nonblocking ROS executor turns; no
  artifact parsing, kinematics or trajectory generation in the GUI.
- `launch/replay.launch.py`: owns RSP, replay, Qt and RViz. Exit of an owned
  process shuts down the session. No vendor control launch is included.

The existing cable_robot_control ArmInterface publishes real servo/bus commands;
none of it is imported or started here. Official display.launch.py also starts a
Joint State Publisher GUI and has path/environment assumptions; it is not reused.
RSP receives the unchanged XML expanded by existing robot/model.py. No need_compile
parameter is required. Base/EE/order remain base_link/end_effector_link/joint1..5.
Auxiliary r_joint uses the saved evaluation's 0.3 rad, not config's default 0.
Independent joints are derived from URDF; mimic joints are not separately published.

## Full verification with explicit input relocation

The original T5.1 verifier followed absolute input names and settings.seed_prefix.
Its only change is optional keyword `input_bindings`: a COMPLETE original-name ->
explicit current-file mapping. Default behavior and every existing numeric/hash/
seed/FK/grid/query/summary check remain unchanged. No algorithm or historical file
is edited. Existing callers do not need to change.

`config/t51_inputs.json` declares five workspace-relative provenance files:
task YAML, scene YAML, original G4 NPZ/JSON and Native identity bytes. The reader
matches all original manifest digests against these explicit files, does not
follow legacy paths, and refuses missing/ambiguous/mismatched bindings. A Native
binary is hashed only as a provenance file: it is NEVER executed or loaded.
No MoveIt/FCL process, IK solve, RRT or Cartesian solve is needed for replay.
The verifier recomputes FK/task metrics and deterministic seed bookkeeping only.

This deliberately does not weaken validation to get an animation. Missing G4 or
backend identity bytes are BLOCKED; restore a matching archived file and explicitly
update the input profile. Preserve the G4 NPZ/JSON basename pair. The profile may
point to another workspace-relative location with the same contents. There is no
automatic archive download, data copying, path fallback or full experiment rerun.
Only the explicitly selected profile itself may resolve the expected colcon
symlink-install link; artifact and provenance-file symlinks remain rejected.

T5.1 `validation_manifest.json` binds its manifest and independent report. For
this run also pass the known manifest SHA256 below as an external trust anchor.
All input/trajectory bytes are rehashed after verification, then arrays have
immutable byte backing. Selection uses the frozen verified snapshot; switching
to another benchmark requires closing/restarting launch and revalidating it.

## Build and launch (actual tested environment)

Observed prerequisites: Ubuntu 22.04, ROS2 Humble, Python 3.10.12, RSP, RViz2,
PyQt5, NumPy, SciPy, PyKDL, Xacro/URDF parser and the existing official overlay.
No dependency was installed or upgraded. Plotly and a running Native backend
are not runtime requirements. Runtime uses existing APT/ROS Python modules.

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
export PYTHONDONTWRITEBYTECODE=1

# Reproduce the tested build configuration in a NEW user output directory.
colcon --log-base results/rviz_replay/build_user_01/log build \
  --base-paths ros2/jetrover_idc_visualization \
  --build-base results/rviz_replay/build_user_01/build \
  --install-base results/rviz_replay/build_user_01/install \
  --symlink-install --packages-select jetrover_idc_visualization

# Use the already verified installed build, or source your build_user_01 instead.
source results/rviz_replay/build_20261104_02/install/setup.bash

# Read-only full data check; no ROS nodes or Native process are started.
ros2 run jetrover_idc_visualization check_replay \
  --workspace-root "$PWD" \
  --benchmark-dir "$PWD/results/task_benchmark/t5_1_s1_20261104_03" \
  --input-profile "$PWD/ros2/jetrover_idc_visualization/config/t51_inputs.json" \
  --expected-manifest-sha256 3e3a0d27123c197c8ef9a41c8e1a111e2181f0998740e0fa0ca95903d58cfbcd

export ROS_LOG_DIR="$(mktemp -d /tmp/jetrover-idc-replay-log.XXXXXX)"
ros2 launch jetrover_idc_visualization replay.launch.py \
  workspace_root:="$PWD" \
  benchmark_dir:="$PWD/results/task_benchmark/t5_1_s1_20261104_03" \
  expected_manifest_sha256:=3e3a0d27123c197c8ef9a41c8e1a111e2181f0998740e0fa0ca95903d58cfbcd \
  domain_id:=83 replay_hz:=20.0
```

The launch forcibly sets ROS_LOCALHOST_ONLY=1 and a nondefault domain in 1..101.
Direct replay/panel entry points refuse default/non-loopback settings. All replay
traffic is under `/idc_replay/`: joint_states, tf, tf_static, robot_description,
scene_markers, port_markers, command and status. JointState publishers refuse
remaps away from their declared private output. There are no controller clients.
RViz fixed frame is base_link; no /clock is used (use_sim_time=false).
The selected private domain must not be intentionally reused for hardware nodes.

Normal main launch opens both Qt and RViz. Closing either, or Ctrl+C, closes the
owned session. `gui:=false` starts only RSP/replay for automated transport tests.
This does not launch Gazebo, move the chassis or perform insertion.

## Panel and time semantics

After VERIFYING_ARTIFACTS becomes READY, choose Port, qN, Linear/RRT-Connect and
Path/Approach/Path + Approach. Choices come from saved valid records. Seven failed
ports have no goal entries and Play is disabled; their exact saved failure and
best position/axis/key errors are displayed. P12 exposes four goals, P22 two.

Play restarts a finished sequence. Pause/Resume preserves the current original
index. Reset and selection stop old playback and instantly reposition to the
phase's saved FIRST sample; these jumps are not planned return motions or Home.
The final frame is held with fresh JointState timestamps. Current sample count,
phase, goal/planner, status and manifest digest remain visible.

Path starts at the saved common G4 sample, not physical Home. Approach starts at
the selected pre-insertion goal. A full sequence omits only the duplicate shared
boundary; all other q/TCP values and provenance are original saved samples.
Fixed 20 Hz is illustrative playback, not velocity/acceleration parameterization,
continuous collision certification or hardware timing. RViz may display fewer
frames at high rates; transport tests use the supported default 20 Hz and a
history-preserving observer. The panel intentionally subscribes to latest status.

Semantic port points/arrows are references, not manufactured RJ45 hole/tool collision
models. All 38 object poses/dimensions come from the snapshot, with transparent
display styling only. World coordinates and camera do not alter research data.

## Automated checks and manual acceptance

All new tests are declared in tests/dependencies.json. Extra requirements `ros`,
`qt`, `replay_install` BLOCK explicitly if absent; they never become a silent skip.

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source results/rviz_replay/build_20261104_02/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export C1_BACKEND="$PWD/results/c3/assembly_build_20261104/build/collision_backend"

.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m core tests
.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m integration tests \
  --native-backend "$C1_BACKEND"
.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m artifact tests \
  --native-backend "$C1_BACKEND" --artifact-root "$PWD/results" \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source \
  --replay-install "$PWD/results/rviz_replay/build_20261104_02/install"
.venv/bin/python -B repro/scripts/check_clean_checkout.py
```

The existing regression suite may exercise its existing algorithms on temporary
test fixtures; the new replay reader/node do not solve or plan. Artifact relocation
tests create/remove only unique temporary hardlink directories on the artifact
filesystem; they never copy large histories or mutate linked bytes.

ROS tests use private domains 91/92/93: synthetic fixtures only for transport/Qt
unit integration, and ACTUAL P12/P22 artifacts for installed multi-process launch.
The real artifact test checks every observed original index/q, endpoint EE TF and
saved Native gripper/mimic TF, marker counts, normal exit and no residual owned nodes.
Synthetic protocol fixtures are never presented as collision/IK research evidence.

Manual user acceptance:

1. Launch; check full robot, cabinet/devices, nine port labels and RViz Global Status.
2. P12-q0 / Linear / Path + Approach: Play, Pause, Resume, Reset.
3. P12-q1 / RRT-Connect / Path + Approach; inspect the changed posture.
4. P22-q0 and q1; inspect both methods and Approach-only playback.
5. P11: empty goals, saved IK failure, disabled Play; no old animation continues.
6. Change goals/planners during playback: STOPPED/new phase sample0, no mixed arrays.
7. Close Qt/RViz or Ctrl+C; confirm owned replay/RSP nodes stop.

Troubleshooting: package not found means source the NEW install setup; missing
jetrover_description means source the existing official overlay. BLOCKED inputs
require restoring exact hash-bound files/updating the explicit profile. Model or
Scene/SRDF mismatch is an error, not permission to rewrite snapshots. A display/
OpenGL failure does not invalidate saved plans; inspect logs and use transport-only
mode for diagnosis, then perform actual GUI acceptance on a working display.

## Boundaries

No FK/IK/FCL/RRT/Cartesian algorithm, SRDF, task tolerance, Scene Schema or historical
JSON/NPZ/HTML/Manifest is modified. No install/upgrade, hardware interface, control
launch, VLA, Gazebo, new benchmark/planning run or T5.2 four-scene work is performed.
The only existing research-file change is optional input relocation in the verifier.
All snapshots/identity checks retain research-only/uncalibrated/missing-coverage meaning.
An actual RViz desktop run and actual Qt captures are evidence of software rendering,
not physical geometry calibration, real insertion or user acceptance.

Input relocation does not bypass the existing raw-URDF/SRDF binding. On a machine
where resource-path serialization changes the expanded URDF hash, portable asset
identity may pass while the full saved-run identity fails; this is reported, not
silently rewritten. A separate reviewed model-binding solution is still needed.

## Actual validation record

The current baseline was actually rerun: 689 passed, one existing Xacro warning.
Final groups: Core 476, Integration 186, Artifact 60; total 722 (+33), zero failed,
blocked or skipped. Integration/Artifact each show the existing Xacro warning.
The isolated no-history copy passes 476 Core tests. Existing Native CTest passes
1/1; it is regression coverage, not a dependency started by the replay application.

The actual installed launch replays P12-q0 Linear/full, P12-q1 RRT/full, P22-q0
Linear/full and P22-q1 RRT/full at default 20 Hz: 270, 233, 196 and 393 frames,
respectively (1092 total). All original indices/q are observed; EE FK/TF and four
Native-saved gripper/mimic transforms match at 1e-9 or better. Owned processes
exit 0 without forced termination and no owned ROS nodes remain in the domain.

The actual desktop test opens all four processes in private Domain84 and completes
P12-q0 Approach (27/27). Selected evidence includes a real RViz render viewport,
an unobstructed real production Qt panel client displaying that same live artifact
state, and launch logs showing all four clean exits. Occluded desktop grabs are
not presented as RViz evidence. Full user mouse-driven visual acceptance is pending.

Machine reports, test logs, source/history integrity and selected screenshots are
saved under `results/rviz_replay/validation_20261104_01/`. The earlier generic-Python
build_01 is retained as development output; the validated ament_python install is
build_20261104_02. No historical experiment hashes or algorithms were changed.
