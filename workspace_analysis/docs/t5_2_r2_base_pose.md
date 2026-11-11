# T5.2-R2 — World/Base parameterization and bounded station pilot

RESEARCH_ONLY. Software evidence is prepared for user review; no hardware,
mobile-base execution, insertion or continuous collision safety claim.
Baseline HEAD: `f5afe5063f6866aef5ecc38feec8b81be5489ec4` on
`feat/workspace-planning`. Actual pre-change regression: 722 passed / 1 warning.
No stage/commit/push, dependency installation or official model edits.

## Coordinate and interface contract

World equals B0's original **base_link**. `World_T_Base(B0)=I`; World z=0 is
not the wheel-ground plane. BasePose defines base_link x/y/yaw in metres/radians,
with unchanged height/roll/pitch. Finite real values, explicit unique IDs and
yaw in [-pi,pi] are required. There is no implicit wrapping or height search.

`scene/base_pose.py` applies `inverse(World_T_Base)` exactly once to scene root
frame edges and collision objects parented directly to base_link. All child local
poses/dimensions, hierarchy and port/object IDs remain intact. Frame references
receive a consistent derived namespace. B0 returns the original immutable
snapshot, preserving canonical SHA256. Nonzero poses get separate scene identity.
Scene Schema v1 still has base_link as its root. FK and Native FCL consume the
same derived snapshot; this never moves only the target while leaving obstacles.

`verify_derivation` checks every frame, all 38 obstacles and nine ports, including
rotations and signed insertion/key axes, back in World. Tests include nonzero
yaw and a directly base-parented collision object. The nonzero-yaw pipeline is
geometrically tested; this Pilot deliberately runs yaw=0 only.

`robot/root_frame.py` derives the **fixed mount only**, not another arm FK:
the actual official root is base_footprint; its base_joint gives
`root_T_base.translation.z = 0.11609108215767461 m`.
The replay publishes
`World_T_root = World_T_base * inverse(root_T_base)`.
RSP retains sole ownership of root→base_link and all downstream transforms.
RViz fixed frame and camera target are World. Cabinet markers remain expressed
in the derived base_link coordinates, so they are fixed after TF into World.

Existing `run_benchmark` gains two optional keyword arguments:

* `start_sample_index`: fresh check of exactly that original G4 row, with no
  replacement. Invalid rows remain inspectable but all task stages are NOT_RUN.
  The default first-valid behavior for old callers is unchanged.
* `base_pose_context`: full hash-bound World source, pose, derived-scene identity,
  URDF mount and full geometry verification. Exact scene/context input bytes are
  embedded as listed benchmark artifacts for relocation and formal replay.

`base_pose_benchmark.py` calls existing T5.1 `run_benchmark` once per station.
No IK/FK/FCL/RRT/Approach algorithm, budgets, tolerances or SRDF rules were changed.
The full existing verifier checks each child, its input hashes and native ledger.
The new context verifier recomputes the World/Base derivation. Each child has its
own validation receipt and manifest; the parent hashes the entire bounded run.

## Pilot configuration and actual results

Config: `config/base_pose/s1_pilot_v0.json`. Five cross stations, 21 port tasks,
not a full Cartesian grid or global optimizer. Same G4 n1000/seed42 original
sample **0**, same 41 seeds / 123 optimizations per task, random seed 20261104,
RRT 20 s / 1500 iterations, 0.01 rad final checking, original task tolerances,
40→10 mm approach and explicit standard_moveit_candidate_v0 SRDF.
Every station freshly passed the same start; none was substituted or skipped.

Results: `results/base_pose_benchmark/t52_r2_s1_20261104_01/`.
Counts below are unique IK goals per task and accepted trials, not hardware rates.
`IK budget` means no full task-pose solution found within the declared solver
budget, not a mathematical proof of unreachability. Unlisted station/ports were
not run; B0's additional seven no-solution ports are all retained in JSON/CSV.

| Station (x,y mm; yaw=0) | Port | IK | FCL valid | Linear | RRT | Approach | Result |
| --- | --- | ---: | ---: | ---: | ---: | ---: | --- |
| B0 (0,0) | P12 | 4 | 4 | 4/4 | 4/4 | 4/4 | Research success |
| B0 (0,0) | P22 | 2 | 2 | 2/2 | 2/2 | 2/2 | Research success |
| B0 (0,0) | P32 | 0 | 0 | — | — | — | IK budget |
| xm30 (-30,0) | P12 | 4 | 4 | 4/4 | 4/4 | 1/4 | Research success; branch failures retained |
| xm30 (-30,0) | P22 | 2 | 2 | 2/2 | 2/2 | 2/2 | Research success |
| xm30 (-30,0) | P32 | 1 | 1 | 1/1 | 1/1 | 1/1 | Research success |
| xp30 (+30,0) | P12 | 2 | 2 | 2/2 | 2/2 | 2/2 | Research success |
| xp30 (+30,0) | P22 | 3 | 1 | 1/1 | 1/1 | 1/1 | Two IK goals collision-rejected |
| xp30 (+30,0) | P32 | 0 | 0 | — | — | — | IK budget |
| ym70 (0,-70) | P21 | 0 | 0 | — | — | — | IK budget |
| ym70 (0,-70) | P22 | 0 | 0 | — | — | — | IK budget |
| ym70 (0,-70) | P23 | 2 | 2 | 2/2 | 2/2 | 2/2 | Research success |
| yp70 (0,+70) | P21 | 2 | 2 | 2/2 | 2/2 | 2/2 | Research success |
| yp70 (0,+70) | P22 | 0 | 0 | — | — | — | IK budget |
| yp70 (0,+70) | P23 | 0 | 0 | — | — | — | IK budget |

B0 P11/P13/P21/P23/P31/P33 also have zero full IK goals. Across all 21 tasks:
2583 optimization attempts, 22 full task goals, 20 collision-valid goals,
20/20 Linear and 20/20 RRT trials accepted, 17/20 Approach trials accepted,
9 task successes and 12 IK-budget failures. Per-port elapsed time totals
589.169 s; 12680 main candidate Native queries (strict-control sessions and
initialization are separate). Detailed path lengths, search/validation times,
clearances and identity fields remain in summary.csv and per-port records.

At xp30/P22, two goals remain rejected by **link3/servo_link2** semantic self
collision. At xm30/P12, goals 0/1/3 stop at the existing joint-continuity limit
(first failure at 10/20/10 mm); goal 2 completes. These failures were retained,
without altering tolerances or semantic rules to improve success counts.

The original S1 has P21 y=+70 mm, P22 y=0, P23 y=-70 mm. Thus ym70 aligns P23,
while yp70 aligns P21. Targets **and** obstacles were transformed together;
their success is a measured result, not an assumed symmetry. The farther xm30
station enables this bounded P32 result; more IK solutions still do not imply
better approach continuity or quality.

All five station records classify the modeled rigid body as MODELED_BODY_CLEAR
at the chosen start. Its fixed component includes chassis, shells and four wheel
links. This is not whole-vehicle physical safety: lidar_link has no collision
mesh; tool/cable/contact, physical tolerances and calibration remain unverified;
ground support and the mobile trajectory into the station are not modeled.
Each station explicitly records PARTIAL_UNVERIFIED physical coverage.

## R1 fixes and World replay evidence

Goals are now separate from playable trajectories. Reader retains every saved
task goal (including collision-rejected inspection), individual planner status,
authoritative goal TCP and collision-candidate flag. Failed planners cannot
create a trajectory. The panel shows their actual status and disables Play when
none exists; inspection displays goal q instead of silently showing the start.
Actual Qt tests cover the IK-present/Linear-and-RRT-failed case.

Formal Reader and Launch require an **externally selected** Manifest SHA256.
Computing a SHA from arbitrary untrusted input is not a trust decision. The CLI
alone exposes `--allow-unanchored-development`; formal Launch has no unanchored
fallback. Each embedded input must match both its listed artifact SHA and original
input SHA; old absolute paths remain metadata, never fallback reads.
Strict JSON now recursively rejects exponent overflow (`1e999`), including
nested arrays/objects, as well as duplicate keys and NaN/Infinity tokens.

New actual desktop evidence (same World camera):

* `results/rviz_replay/validation_20261104_r2_01/desktop_b0/`
* `results/rviz_replay/validation_20261104_r2_01/desktop_xm30_02/`

Each contains actual RViz and production Qt PNGs, exact status, image SHA256,
launch log and clean exit status. `desktop_xm30/` is retained as a failed first
capture attempt: it matched hidden Qt windows; use `_02` for accepted evidence.
No image generation or synthetic rendering was used for the PNG evidence.
The headless actual new-station test replays 242 original saved samples, verifies
38 World-fixed scene markers (max position error 1.11e-16 m), World→root→base
TF without a second parent, mimic links and TCP against authoritative FK.
The historical P12/P22 launch still replays all 1092 original samples across
four Path+Approach cases. User interactive/visual acceptance remains pending.

Overview: `results/rviz_replay/validation_20261104_r2_01/station_overview.html`.
It plots only recorded Pilot values; empty cells mean not run.

## Data identity and preservation

Parent manifest SHA256:
`11c4f1eb77d0617ab281e047e25f6ebafc538336f4dd9ee89e70dcd2304252e6`.
Original/B0 canonical scene SHA256:
`d83e76d09f7e69fa0c01e753f0cefb3128dd520fb0292f1fd0b4838f66a5ab3b`.
xm30 derived scene SHA256:
`502c6a8c32705403b29eeb930041befc3875da748f08bdc66ee9e6e90ccacb42`.
Other full identities are in summary.json, station base_pose_context.json,
child manifest.json and the Native query ledger. Each child contains Git HEAD,
exact computation source hashes, URDF/geometry/SRDF/review/scene/backend identity,
gripper, clearance, task/TCP and fixed input hashes.

B0 P22 independent T2/T3/T4 comparison: 123 attempts, four paths and 52 approach
samples; maximum joint/transform/error/clearance differences all **zero**.
All nine B0 ports preserve IK goals, status, q/TCP/validity/clearance arrays.
Query IDs are per-run ledger addresses and can shift by one; both runs' IDs
are validated within their own ledger, not treated as portable sample identity.

No formal historical experiment bytes or official model resources were edited.
The integrity report separately lists the old colcon --symlink-install entries
that reflect intentionally changed live source/config, and two CTest runtime
log/cost files updated by running CTest. These build-state changes are not
reported as immutable historical experiment changes or hidden from the audit.
No original file was lost. Existing untracked ../docs/develop/ and results.zip
were left alone. New results remain ignored by Git.

## Reproduction and user acceptance

From a terminal with existing dependencies:

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash

# New-only output: use a fresh name on every rerun.
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_base_pose_benchmark.py \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --seed-prefix results/g4_run01/experiments/workspace_n1000_seed42 \
  --pilot-config config/base_pose/s1_pilot_v0.json \
  --allow-candidate-baseline \
  --output-dir results/base_pose_benchmark/t52_r2_user01

# Fresh build if the supplied R2 build is not present. No dependency install.
colcon --log-base results/rviz_replay/build_r2_user01/log build \
  --base-paths ros2/jetrover_idc_visualization \
  --build-base results/rviz_replay/build_r2_user01/build \
  --install-base results/rviz_replay/build_r2_user01/install --symlink-install
source results/rviz_replay/build_r2_user01/install/setup.bash

# Actual completed xm30 child (not the parent collection manifest).
ros2 launch jetrover_idc_visualization replay.launch.py \
  workspace_root:="$PWD" \
  benchmark_dir:="$PWD/results/base_pose_benchmark/t52_r2_s1_20261104_01/stations/xm30/benchmark" \
  expected_manifest_sha256:=d39077e52aef45339dd0b167ce1f12650f07301ef9dee34d100c58f58c39919f \
  domain_id:=83
```

In the panel select P12/P22/P32, goal and planner, then Path / Approach /
Path+Approach. Exercise Play/Pause/Resume/Reset. Confirm Base ID and x/y/yaw,
World cabinet, root tree and per-planner outcome. To compare B0, stop the owned
launch with Ctrl-C and relaunch with `stations/b0/benchmark` and SHA
`be5df487107a46b16937379881b9bcd5c463615567e885e13074aa8d00765e5d`.
Switching station is an offline reload, not a planned chassis movement.
No-path and collision-rejected goals are inspection only, with Play disabled.

```bash
# Full strict Reader; no IK/FCL process is run by replay.
ros2 run jetrover_idc_visualization check_replay \
  --workspace-root "$PWD" \
  --benchmark-dir results/base_pose_benchmark/t52_r2_s1_20261104_01/stations/xm30/benchmark \
  --input-profile ros2/jetrover_idc_visualization/config/t51_inputs.json \
  --expected-manifest-sha256 d39077e52aef45339dd0b167ce1f12650f07301ef9dee34d100c58f58c39919f

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/compare_task_p22.py \
  --benchmark results/base_pose_benchmark/t52_r2_s1_20261104_01/stations/b0/benchmark \
  --ik-run results/task_reachability/p22_20261104_02 \
  --path-run results/task_planning/p22_20261104_02 \
  --approach-run results/task_approach/p22_20261104_03

ctest --test-dir results/c3/assembly_build_20261104/build --output-on-failure
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B repro/scripts/doctor.py --mode full \
  --backend results/c3/assembly_build_20261104/build/collision_backend

source results/rviz_replay/build_20261104_r2_01/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider \
  -m 'core or integration or artifact' tests \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend \
  --artifact-root "$PWD/results" \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source \
  --replay-install "$PWD/results/rviz_replay/build_20261104_r2_01/install"
```

All new tests are declared in tests/dependencies.json; missing resources BLOCK
rather than skip. Actual final results and exact source/Git/integrity evidence
are collected under `results/rviz_replay/validation_20261104_r2_01/`.

Final combined run: **749 passed, 1 warning, 0 skipped**, 270.90 s.
The exclusive groups contain Core 496, Integration 189 and Artifact 64 cases.
Native CTest: 1/1 PASS. Environment Doctor: READY_FOR_RESEARCH_REPRODUCTION,
37 checks. Formal new-station Reader: PASS with the selected external anchor.
Both actual desktop launches returned 0 without forced shutdown; transport/Qt/
historical/new-World probes all passed and found no residual owned ROS nodes.
All-nine B0 comparison covers 18 trajectory NPZs / 3034 saved samples with
zero q/TCP/validity/clearance differences. The full regression includes the
new geometry, identity, invalid-start, no-path-goal and overflow cases.

Extending the study requires a new explicit pilot config and fresh output directory;
the same adapter accepts another fixed World scene without changing planning
algorithms. Station search, new hardware evidence, installation height, actual
mobile-base paths and T5.3 remain outside this delivery.
