# T4 — P22 task-constrained Cartesian approach

Software implementation completed; model-level discrete approach validation
completed; interactive Viewer acceptance and physical validation pending user review.
All results are **RESEARCH_ONLY**. No insertion, controller, time parameterization,
RRT detour, contact permission or hardware execution is implemented.

## Inputs and immutable assumptions

Source commit at development start: `02172cb25cd27113c1bafc132fc4dae83d6ca0d1`.
T1/T2 run: `results/task_reachability/p22_20261104_02/`.
T3 run: `results/task_planning/p22_20261104_02/`.
Both runs are verified before a T4 run. The full-precision original T2 q0/q1
are used unchanged, and must exactly equal the corresponding valid T3 linear
path endpoints. No IK is regenerated to select another starting branch.

Existing FrameGraph, Scene v1, official URDF/Mesh, joint limits, gripper/mimic,
TCP, task tolerances and 29 SRDF exclusions remain unchanged. T4 explicitly opts
into `standard_moveit_candidate_v0`; strict SRDF is not replaced or expanded.
Gripper remains `r_joint=0.3 rad`. Configuration clearance defaults are 0 m,
explicitly recorded and configurable. The session's model/Scene/gripper/clearance
must match the declared continuation inputs; mismatches are rejected.

P22 is at `(0.33,0,0.22) m`, with insertion Base +X and key Base +Z. The virtual
TCP is 30 mm along EE +Z with Ry(-90°), synthetic and uncalibrated. Original task
tolerances remain **1 mm / 3° insertion-axis / 5° key-direction**. There is no
RJ45/cable collision body or physical calibration, and the 29 exclusions have
not received complete mechanical approval.

## Target generation and continuation

For every port-derived distance `d`, use:

```text
p_target(d) = p_port - d * insertion_axis_base
R_target(d) = original task TCP target rotation
T_base_tcp(q) = existing PyKDL FK(q) @ original T_ee_tcp
```

The default distances are 40, 30, 20, 10 mm, derived from the original declared
40 mm approach distance, not new hardcoded world positions. The default P22
positions are `(0.29/0.30/0.31/0.32, 0, 0.22) m`. A configurable maximum Cartesian
step supports finer or nondivisible grids, with exact start/end inclusion.
The library is port-independent and tests rotated ports. The P22 CLI's main
experiment stops at >=10 mm; 5 mm/port-plane insertion/contact diagnostics are
outside this implementation and rejected by this CLI.

Each branch uses `NumericalIK` and its existing staged SciPy/PyKDL solver.
Every waypoint logs all optimizer attempts, seeds, evaluation counts, individual
task errors and candidate joint deltas. The first waypoint is solved as a
diagnostic while retaining the exact already-valid T2 source q. Later waypoints
use the previous accepted q. Pose-feasible candidates are rechecked with the
authoritative FK, continuity-filtered and ranked by joint L2 distance to the
previous q. Optimizer success alone is insufficient.

Maximum per-joint waypoint change defaults to **0.35 rad** (a declared research
continuity heuristic, not a physical speed/dynamic limit). On IK/continuity
failure, at most two local backup seeds are tried; each perturbation is bounded
by 0.03 rad and clipped to the unchanged URDF limits. RNG is fixed to 20261104.
All retries and suspected branch jumps are logged. Solver exceptions are
reported explicitly; budget exhaustion or local continuation failure is not a
proof of mathematical unreachability.

## Intermediate task and collision validation

Existing `ConfigurationValidity.edge` checks both endpoints and all joint-linear
samples, fresh without cache, at max per-joint increment **0.01 rad** by default.
For fraction `f` between waypoint distances `d0,d1`, the expected TCP is derived
from `d(f)=(1-f)d0+f*d1`. Every intermediate sample recomputes existing KDL FK/TCP
and checks the original full 3D position, insertion-axis and key tolerances.
Full position error is used, not just distance to the infinite approach line:
cross-track and signed along-track errors are retained as additional diagnostics.
Thus passing endpoint IKs do not imply a valid approach segment.

Every sampled native RobotState TCP is compared with KDL (absolute matrix
tolerance 1e-10). All states also require URDF limits, semantic self collision,
environment collision and declared clearance to pass. Legacy raw `final_decision`
is never used. Query errors, unknown required clearance, identity/FK mismatch,
branch discontinuity and task tolerance failure cannot become valid states.

An edge can be fully inspected diagnostically, but the next waypoint is not
attempted after its first failure. Only the prefix before the first failed
sample contributes to validated approach progress. Reports distinguish accepted
waypoints, task-pose-feasible waypoint candidates, last accepted waypoint, last
validated intermediate sample and first failure. The failure includes expected
distance/transform, q, TCP errors, query ID and specific reasons.

This is a discretely validated joint polyline whose TCP stays within the declared
Cartesian tolerances at checked states. It is not an exact Cartesian controller,
continuous sweep proof, dynamical trajectory or hardware-safe motion.

## Code/API

| File | Responsibility |
| --- | --- |
| `planning/cartesian_approach.py` | `approach_settings`, `target_at_distance`, `cartesian_waypoints`, `check_approach_sample`, `sequential_approach` |
| `experiments/p22_approach.py` | T2/T3 identity verification, both branch experiments, snapshots, JSON/CSV/NPZ, summary/manifest and `verify_approach` |
| `viz/approach.py` | Approach overlays and failure markers on existing T3 full-robot replay |
| `viz/joint_path.py` | Optional `replay_context`; existing T3 API/default semantics retained, expected moving target and all-sample constraints shown for T4 |
| `scripts/evaluate_p22_approach.py` | CWD-independent CLI, explicit candidate opt-in, no-overwrite output |
| `tests/test_cartesian_approach.py` | Coordinate/continuation/interior failures, retries, identity, native collision/clearance, complete failure run and Viewer |

The optional FK/solver injection arguments in the continuation API support
adversarial unit-test oracles; the production CLI always uses existing
`ForwardKinematics` and `NumericalIK`. No second production FK/IK/FCL is added.
Geometry/Scene/robot packages gain no planning dependency; Plotly remains lazy.

## Recorded results

Final run: `results/task_approach/p22_20261104_03/`. Preliminary `_01`/`_02` runs
are retained; no previous experiment is overwritten. The same q/TCP/validity and
clearance arrays are reproduced across the development runs; only diagnostics,
source hashes and timing differ.

| Metric | q0 | q1 |
| --- | ---: | ---: |
| Accepted / requested waypoints | 4 / 4 | 4 / 4 |
| Validated unique replay samples | 26 | 26 |
| Furthest requested distance | 10 mm | 10 mm |
| Actual final axial distance | 10.000000000067 mm | 10.000000000000 mm |
| Validated progress toward port | 30 mm | 30 mm |
| Maximum full position error (including interiors) | 0.102793 mm | 0.102765 mm |
| Maximum insertion-axis error | 0.231741° | <1e-9° |
| Maximum key-direction error | 0.231741° | <1e-9° |
| Maximum selected per-joint waypoint change | 0.080597 rad | 0.080576 rad |
| Minimum sampled self separation | 0.886201 mm | 0.936860 mm |
| Minimum sampled environment separation | 4.568033 mm | 4.553910 mm |
| IK optimization calls | 12 | 12 |
| Continuation + validation time, excluding HTML | 1.970 s | 2.196 s |
| Of which IK | 0.827 s | 0.896 s |
| Collision/clearance/task rejection | none observed | none observed |

Each branch issued 29 fresh FCL queries (shared edge endpoints rechecked) plus
one additional Viewer initialization query; total 60, cache hits 0, normal
backend exit 0. All 52 replay samples satisfy the unchanged original tolerances.
q0 deliberately uses the nearest pose-feasible continuation candidate at the
last waypoint, allowing a small 0.232° orientation change inside the existing
tolerances. It does not silently relax tolerances or claim constant rotation.
Position error peaks inside joint interpolation, not at solved endpoints.

Waypoints have no blocking semantic self/environment collision:

| Distance | q0 environment separation | q1 environment separation |
| --- | ---: | ---: |
| 40 mm | 34.536018 mm | 34.536018 mm |
| 30 mm | 24.537370 mm | 24.537370 mm |
| 20 mm | 14.540582 mm | 14.540582 mm |
| 10 mm | 4.568033 mm | 4.553910 mm |

Raw assembly contacts can remain present under the unreviewed candidate SRDF.
These results do not validate those exclusions or the real RJ45 mechanism.

Regression: **633 passed, 1 warning, 0 skipped**, preserving all 608 baseline
tests. Native CTest: **1/1 PASS** using the unchanged binary in an isolated log
directory. Independent artifact verification rechecked both branches and all
52 samples. **1,747 protected preexisting files** retained their SHA256, including
historical results and official model resources. T1/T2/T3 implementation files
are unchanged apart from the optional shared replay extension documented above;
the existing architecture test only adds a pure-import case for `viz.approach`.
Static full-robot snapshots, error/clearance plots, exact HTML/native payload
checks, reproducibility checks and logs are in
`results/task_approach/p22_validation_20261104_01/`.

## Artifacts and Viewer

`summary.json`, `settings.json`, `cartesian_targets.json`, `source_verification.json`
and snapshots identify source URDF/SRDF/Scene/task/TCP and unchanged inputs.
`manifest.json` hashes input manifests/paths, all output files, relevant source,
configuration, settings and actual backend binary identity. Query ledger and
graceful cleanup are preserved. Existing directories/symlinks are refused.

`branches/q0.json` and `q1.json` contain every IK attempt, candidate, endpoint
check, collision-edge check, all sampled task/FCL records and first failure.
Their NPZ files contain joint states, actual/expected TCP matrices, distances,
position/axis/key errors, cross/along-track errors, validity flags, segment
fractions, query IDs and sampled self/environment clearance. The CSV files give
per-waypoint q/errors/collision/clearance for quick inspection. Null unavailable
numeric measurements become NaN only in NPZ, never invented clearances.

The two HTML files replay all 24 official collision Meshes using saved native
transforms at every displayed q, with the IDC scene, P22 axes, moving expected
TCP, nominal straight approach, actual TCP trajectory, progress/errors/clearance
and joint state panel. Robot/Scene/contacts/link frames can be switched. Only
real native contacts are shown. Raw contacts can be SRDF-disabled assembly
contacts and are labelled. All samples are task-constrained in the T4 panel;
the inherited T3 "only endpoint constrained" mode remains for T3.

On failure, playback stops at the first failed sample, shows the last valid TCP
and first failed expected target, and retains the entire inspected-edge evidence
in JSON. An IK failure without a native pose is not filled with fake transforms.
Playback speed is illustrative, with no trajectory timing or hardware command.
Interactive acceptance is separate from numerical/static checks: the browser
policy blocks this agent from automatically opening local-file HTML.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_p22_approach.py \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --allow-candidate-baseline \
  --ik-run results/task_reachability/p22_20261104_02 \
  --path-run results/task_planning/p22_20261104_02 \
  --cartesian-step-mm 10 --end-distance-mm 10 \
  --edge-resolution-rad 0.01 --max-joint-delta-rad 0.35 \
  --backup-seeds 2 --backup-radius-rad 0.03 --random-seed 20261104 \
  --self-clearance-m 0 --environment-clearance-m 0 \
  --output-dir results/task_approach/p22_user_review_01

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_p22_approach.py \
  --verify-run results/task_approach/p22_user_review_01

PYTHONDONTWRITEBYTECODE=1 \
C1_BACKEND="$PWD/results/c3/assembly_build_20261104/build/collision_backend" \
  .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests

T4_CTEST_DIR="$(mktemp -d /tmp/jetrover-t4-ctest.XXXXXX)"
cp results/c3/assembly_build_20261104/build/CTestTestfile.cmake "$T4_CTEST_DIR/"
ctest --test-dir "$T4_CTEST_DIR" --output-on-failure
```

Choose a new output name for each run. For finer Cartesian waypoints, declare
`--cartesian-step-mm 5` or `2` in a separate new run. Finer joint sampling is
independently configured with `--edge-resolution-rad`. Source task tolerances
cannot be changed by this CLI. `--verify-run` hashes artifacts and original
inputs, checks exact T2 starts/interpolation/query provenance, and recomputes
all FK/task classifications; it makes no new FCL queries.

Native source/binary are unchanged; CTest above writes only to a fresh temporary
directory. See [T3 documentation](p22_path_planning.md) for an optional
new-directory C++ build.

## Before proceeding further

The software demonstrates two local free-space approach candidates under this
particular research model and virtual TCP. It does not prove all Cartesian
solutions have been searched or that the real robot can perform these motions.
Remaining requirements include mechanical approval of the 29 exemptions,
physical joint limits/Home, TCP/tool/cable geometry and calibration, appropriate
uncertainty/clearance margins, continuous motion validation, and any later
contact/force/latch model. No insertion/contact conclusion follows from reaching
10 mm, and no next development stage is entered automatically.
