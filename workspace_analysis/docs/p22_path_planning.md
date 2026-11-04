# T3 — P22 collision-aware joint-space planning

Status: software implemented; model-level discrete paths demonstrated;
interactive Viewer review and physical validation remain pending. All artifacts
are **RESEARCH_ONLY**. No robot controller, insertion or timed trajectory is used.

## Model and inputs

The unchanged T2 run `results/task_reachability/p22_20261104_02/` supplies the
two **full precision** IK goals, task YAML, virtual TCP and tolerances. T3 verifies
that run's manifest and recomputes its task errors before using the goals; it
does not re-solve IK or copy the rounded q values from a document. The unchanged
G4 prefix `results/g4_run01/experiments/workspace_n1000_seed42` supplies the start
pool, through the existing validated `load_results` reader.

Start selection checks G4 rows in ascending original zero-based index order and
chooses the first valid configuration. This rule is independent of planning
outcomes. `start_selection.json` retains every attempted configuration, original
index, collision result and native transforms. This is a simulation start,
**not a measured Home or a physically validated normal assembly posture**.

The candidate `standard_moveit_candidate_v0` must be explicitly enabled. Its 23
Adjacent + 6 Default SRDF exclusions are unchanged and physically unreviewed.
No strict SRDF, Mesh, URDF, S1 scene, joint limits or IK parameters are modified.
The T2 strict/candidate endpoint comparison remains applicable; T3 plans only
under the explicitly selected candidate SRDF. Native checks include gripper
`r_joint=0.3 rad` and existing mimic/fixed transforms. Virtual TCP is kinematic
only; RJ45 body, cable and LiDAR collision coverage remain incomplete.

The target is the **pre-insertion TCP** `(0.29, 0, 0.22) m`, 40 mm before P22.
The TCP is 30 mm along EE +Z with EE-to-TCP Ry(-90°), a synthetic uncalibrated
hypothesis. Only the endpoint must satisfy the original 1 mm / 3° / 5° position,
insertion-axis and key tolerances. Transit configurations are not task-pose
constrained. No insertion or withdrawal segment is planned.

## Implementation and APIs

| File | Responsibility |
| --- | --- |
| `planning/validity.py` | Existing task checks retained; new `ConfigurationValidity(evaluator)` uses semantic FCL results, exact-key cache, query ledger and fresh final checks |
| `planning/joint_path.py` | `interpolate(a,b,resolution_rad)`, `sample_path(waypoints,resolution_rad)`, `path_length(waypoints)` |
| `planning/rrt_connect.py` | Bounded bidirectional `rrt_connect(start,goal,limits,validity,...)`; returns an unverified polyline or explicit failure |
| `experiments/p22_path_planning.py` | Input identity checks, deterministic start, both goals/baselines, exhaustive revalidation, artifacts and independent verifier |
| `viz/joint_path.py` | Existing full robot/task/scene figure plus native-transform playback and checked TCP trajectory |
| `scripts/evaluate_p22_path.py` | CWD-independent CLI, explicit candidate opt-in and no-overwrite output |
| `tests/test_path_planning.py` | Geometry, collisions, cache/freshness, failures, deterministic detours, native regression and replay reconstruction |

`ConfigurationValidity.check(q, fresh=False, deadline=None)` returns explicit
`valid/status/reasons`, semantic collision flags/pairs, clearance, query ID and
actual native transforms. Only `model_configuration_valid is True` with both
semantic self/environment booleans false and research-only identity can pass.
Errors, invalid limits, unmet clearance and unknown required clearance fail
closed. Legacy raw `final_decision` is never used. Pair/contact truncation is
retained; contacts are not a complete surface or a mechanical safety proof.

`edge(a,b,resolution_rad,...)` and `path(waypoints,resolution_rad,...)` check both
exact endpoints and all selected intermediate configurations. Failed checks
retain first sample/segment/fraction/q/reasons; `stop_first=False` exhausts the
whole diagnostic path. All checked state data and minimum **sampled unsigned
FCL separation** are recorded. Interpolation uses finite-stop Euclidean radians,
without +/-pi wrapping; every per-joint increment is <= the declared resolution.

Search resolution defaults to 0.02 rad, final resolution to 0.01 rad, extension
range to 0.25 Euclidean rad, RNG to NumPy default_rng(20261104), goal bias 0.05,
iteration budget 1500 and search deadline 20 s. No positive clearance margin is
silently added: defaults inherit the research zero-margin condition (0 m),
explicitly configurable with CLI flags. Any stricter margin requires a new run.

The environment has no usable `ompl`, `moveit`, or `moveit_py` Python binding.
The small independent RRT-Connect implementation grows two alternating trees
using EXTEND/CONNECT and the **same** validity adapter. It adds no alternate FK
or collision calculation and no new dependency. See the
[OMPL algorithm reference](https://ompl.kavrakilab.org/core/classompl_1_1geometric_1_1RRTConnect.html).
The deadline is cooperative between native calls (a single in-flight query can
overrun); final exhaustive validation and rendering are outside the search
deadline and timed separately. Iteration/time exhaustion is not an unreachable
proof. Paths found by search are never approved until fresh full fine-resolution
FCL revalidation. No shortcut/smoothing is applied in this baseline.

Default `--rrt-mode on-failure` skips RRT after a valid linear path. The recorded
experiment uses `--rrt-mode always` to measure both algorithms for both goals;
this is an explicit comparison, not a claim that RRT was needed.

## Artifacts and replay

Each run is a new directory. `summary.json` gives the comparison and model
limitations; `manifest.json` records HEAD, all relevant source/input hashes,
URDF/geometry/backend/Scene/SRDF/ACM/task/TCP identities and parameter hash.
URDF, scene, task, SRDF and candidate provenance snapshots are included.
Native process exit status and total query ledger are retained. Existing output
directories and HTML paths are refused, including dangling symlinks.

Each `paths/q{0,1}_{linear,rrt_connect}.json` retains search and fresh verification
data. Its NPZ contains `q[N,5]`, `waypoints`, `T_base_tcp[N,4,4]`, `tcp_xyz[N,3]`,
`valid[N]`, `query_id[N]`, both clearances, original start index and goal ID.
NPZ bytes are hashed in the manifest; no-pickle reads are used. Missing distances
are NaN in numeric arrays and null/unavailable in JSON, never invented gaps.

Each self-contained HTML shows all 24 robot collision Meshes, 38 IDC objects,
P22/target axes, actual TCP and sampled trajectory. Slider/Play steps through
native RobotState transforms at the exact checked q, with current joint values,
semantic collision/clearance/task errors and query provenance. Intermediate
task pose failure is expected and does not mean the transit state is colliding.
Blocking objects are highlighted; optional raw contacts can include SRDF-disabled
assembly contacts and are labelled accordingly. Robot/scene/contact/link-frame
layers can be toggled. Plot bounds encompass every replayed robot state, with
equal metric aspect ratio and a fixed reusable camera.

Geometry is stored once in link coordinates and transformed in JavaScript by
native link matrices, rather than embedding hundreds of duplicate full meshes.
No client-side FK exists. Playback is illustrative, with no velocities,
accelerations, timing, continuous sweep or dynamic guarantees. A rejected linear
path, if present, is diagnostic and must not be interpreted as executable.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_p22_path.py \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --allow-candidate-baseline \
  --ik-run results/task_reachability/p22_20261104_02 \
  --seed-prefix results/g4_run01/experiments/workspace_n1000_seed42 \
  --rrt-mode always \
  --edge-resolution-rad 0.02 --verification-resolution-rad 0.01 \
  --step-rad 0.25 --random-seed 20261104 \
  --max-time-s 20 --max-iterations 1500 \
  --self-clearance-m 0 --environment-clearance-m 0 \
  --output-dir results/task_planning/p22_user_review_01

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_p22_path.py \
  --verify-run results/task_planning/p22_user_review_01

PYTHONDONTWRITEBYTECODE=1 \
C1_BACKEND="$PWD/results/c3/assembly_build_20261104/build/collision_backend" \
  .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests
```

Use a new output name each time. `--verify-run` checks all artifact and original
input bytes, G4 start selection, exact unchanged T2 goals, sample resolution,
NPZ/JSON/query correspondence, path lengths and KDL-vs-native TCP for every path
sample. It issues no new FCL queries and is not another collision proof.

Native regression without writing to historical build results:

```bash
T3_CTEST_DIR="$(mktemp -d /tmp/jetrover-t3-ctest.XXXXXX)"
cp results/c3/assembly_build_20261104/build/CTestTestfile.cmake "$T3_CTEST_DIR/"
ctest --test-dir "$T3_CTEST_DIR" --output-on-failure
```

The C++ source is unchanged. Optional fresh rebuild (separate directory):

```bash
T3_BUILD_ROOT="$(mktemp -d "$PWD/results/task_planning/native_review.XXXXXX")"
cmake -S cpp/collision_backend -B "$T3_BUILD_ROOT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$T3_BUILD_ROOT/build" -j2
ctest --test-dir "$T3_BUILD_ROOT/build" --output-on-failure
```

Use that new `collision_backend` path explicitly for a separate run if desired;
its actual binary hash is recorded. No historical build or experiment is
overwritten. The original 573-test baseline and existing pending T1/T2 changes
are preserved; no Git commit or push is performed.

## Recorded result (2026-11-04)

Final run: `results/task_planning/p22_20261104_02/`. The preliminary `_01` run is
retained and was not overwritten. Both use identical q paths, semantic results
and numeric TCP/clearance arrays; the final run also includes extra input
snapshots and current-frame Viewer metadata. No historical C1–T2 run is changed.

G4 original sample **0** passed as the first simulation start:

```text
[1.1451362829639269, -0.25548812183642133, 1.4989393052295785,
 0.8249983614681411, -1.6963386858296248]
```

| Goal / method | Result | Joint length rad | Waypoints / checked samples | Plan + validation s | Actual FCL queries | Min self / environment mm |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| q0 / linear | discrete pass | 2.301417 | 2 / 171 | 6.796 | 171 | 0.811 / 34.353 |
| q0 / RRT-Connect | discrete pass | 2.684915 | 12 / 198 | 11.636 | 298 | 0.811 / 33.611 |
| q1 / linear | discrete pass | 4.548611 | 2 / 344 | 14.387 | 344 | 0.799 / 8.665 |
| q1 / RRT-Connect | discrete pass | 4.794419 | 21 / 368 | 22.994 | 562 | 0.799 / 5.238 |

RRT search alone took 3.900 / 7.886 s; exhaustive fine revalidation took 7.685 /
15.019 s. Thus q1's overall 22.994 s does not exceed the **search-only** 20 s
budget. Counts exclude four additional replay-initialization queries. The full
session issued 1,382 real queries: 1 start + 2 goal + 1,375 planning/validation +
4 replay checks; 33 exact cache hits are recorded separately. Process cleanup
was graceful, exit code 0. All 1,081 final path samples passed, with native TCP
and independent existing KDL FK agreement (1e-10 absolute matrix tolerance).

Both methods passed 2/2 tested goals for **this one start and fixed seed**. This
is not a statistical success-rate estimate. Straight interpolation was adequate
for both; the RRT runs were optional comparison runs. The implementation's
blocked-interior and detour/budget tests cover the fallback logic independently.
Among these four paths q0 linear is shortest and has the largest sampled
environment clearance; no general optimum or hardware motion is asserted.

Python regression: **608 passed, 1 warning, 0 skipped** (baseline 573).
Native CTest: **1/1 PASS**, existing unchanged binary, isolated CTest log directory.
Full-robot replay transforms are numerically tested; JavaScript syntax is checked.
Interactive playback is **pending user review** because the local-file browser
policy blocks automated opening. Native-query/geometry/static verification must
not be substituted for that interactive acceptance.

## Remaining acceptance boundaries

These are a finite set of reproducible discrete model paths from one synthetic
start, not planner success probabilities, global completeness, physical Home
validation or hardware safety. The <1 mm self clearances, unreviewed assembly
exclusions, TCP/tool geometry/calibration, physical joint stops and absent timing
must be resolved before any hardware planning. A smaller sampling step alone
does not establish continuous collision safety. Browser security prevents this
agent from opening local HTML automatically; numerical replay reconstruction
and static rendering checks are separate from the user's interactive acceptance.
