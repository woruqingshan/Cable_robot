# T5.1 — Generic task planning and S1 nine-port benchmark

Status: implementation and model experiments; RESEARCH_ONLY, pending user review.
Reference HEAD: `8688e2ff2f9760d34b39189d35af2a924d39e5aa`.
The original untracked `../docs/develop/` and `results.zip` are untouched.

## Interface audit and implementation

The authoritative call chain is:

```
SceneSnapshot + validated TaskSpec
  -> task.targets.build_targets (FrameGraph, TCP and signed axes)
  -> planning.ik.build_seeds + NumericalIK.solve (existing PyKDL / SciPy)
  -> ConfigurationValidity.check (existing semantic Evaluator / Native MoveIt/FCL)
  -> linear endpoint polyline OR existing rrt_connect
  -> ConfigurationValidity.path(fresh=True, stop_first=False)
  -> existing sequential_approach (all interpolated samples task-constrained)
  -> versioned records, independent readback and optional existing mesh replay
```

The existing `p22_*` experiment entry points remain byte-for-byte unchanged.
They enforce P22, two saved IK branches, absolute upstream manifests and (for T4)
saved Linear paths. These are historical contracts, not generic algorithm limits.
Task targets, seed generation, IK, validity, RRT and Cartesian continuation were
already generic; none of those algorithms was modified or duplicated.

New files:

- `task/template.py`: validates unique existing ports; derives distinct task IDs.
  The template's original target retains its original identity for exact P22 comparison.
- `experiments/task_context.py`: explicit borrowed model/scene/targets/checker,
  session identity and settings validation, first-valid G4 start, system fault handling.
- `experiments/task_reachability.py`: unchanged multi-start IK, authoritative FK
  task recheck and fresh semantic screening of all unique full-task solutions.
- `experiments/task_path_planning.py`: explicit `linear`/`rrt_connect` dispatch,
  endpoint checks, recorded search timing and fresh exhaustive final validation.
- `experiments/task_approach.py`: adapter to unchanged sequential Cartesian IK.
- `experiments/task_benchmark.py`: serial session ownership, all-goal scheduling,
  per-port records, level-specific statistics, JSON/CSV/NPZ and opt-in viewers.
- `experiments/task_benchmark_validation.py`: file/input identity, seeds, original
  start index, FK, exact interpolation grids, query provenance and summary checks.
- `experiments/task_benchmark_regression.py`: explicitly supplied historical P22
  inputs, existing T2/T3/T4 verification, complete-precision numeric comparison.
- `viz/task_benchmark.py`: optional compact overview from authoritative summary.
- `scripts/evaluate_task_benchmark.py` and `scripts/compare_task_p22.py`: CWD-independent CLIs.

The caller owns a `TaskContext`'s evaluator. `run_benchmark` owns one persistent
candidate evaluator and closes it normally; strict controls use separate owned
sessions through existing `screen_solutions`. Model/scene/SRDF/backend identities
are verified by the existing Native protocol and checked again at the new boundary.
Unknown collision/clearance results are system faults, never ordinary rejection.
Ordinary no-solution/path/approach failures do not stop later ports. A shared system
fault is ERROR and subsequent task evaluation is BLOCKED, not counted as IK failure.

All collision-valid unique solutions are evaluated. Linear AND RRT run for each;
there is no on-failure shortcut. Approach runs once per goal with at least one
freshly validated arrival path, not once per successful planner. No RRT detour is
used for the Cartesian segment. Different valid SceneSnapshots and task files are
explicit inputs; current Scene Schema still requires base_link. Planner selection
is a small method dispatch; future strategies should implement the same candidate
polyline plus independent final validation contract, not replace FK/FCL.

## Experiment contract

S1 is unchanged. Ports P11–P33 resolve exclusively through its FrameGraph.
Pre-insertion 40 mm, approach end 10 mm, position 1 mm, insertion axis 3 degrees,
key 5 degrees, virtual EE+Z 30 mm / Ry(-90 degrees), r_joint 0.3 rad and zero
required model separation are inherited unchanged from the P22 research template.
The TCP is synthetic_uncalibrated. The 29 candidate SRDF permissions are unchanged
and physically unreviewed; strict controls remain independently identified.

Each task uses 16 nearest G4 virtual-TCP seeds, 24 deterministic extra joint seeds
and one diagnostic URDF midpoint, with the original 400-evaluation stage budget.
Seed 20261104, Linear final resolution 0.01 rad, RRT search resolution 0.02 rad,
step 0.25 rad, 1500 iterations / 20 seconds, goal bias 0.05; final RRT recheck
0.01 rad. Approach checks use 0.01 rad and unchanged continuation settings.
One common start is selected once by ascending original G4 row index, independent
of task outcomes. It is a simulation sample, never a verified physical Home.

`--random-seed` explicitly derives a new task identity and consistently changes
IK, RRT and approach seeds; changing only one stage's seed is rejected. Template
source hash, effective task hashes and settings are recorded separately. Wall-clock
budgets can affect results on another machine even with deterministic random draws.

Per-port JSON has schema_version=1, targets, complete IK attempts/errors, all goals,
per-planner search/validation, approach samples and first failures, identities,
latencies and query counts. Downstream phases not executed are NOT_RUN. No solution
within finite budget is IK_NO_SOLUTION_WITHIN_BUDGET, never mathematical unreachability.
Collision rejection, invalid endpoints, search budget/timeout, failed final recheck,
failed approach and system ERROR/BLOCKED have separate meanings.

The summary keeps seed attempts, unique IK solutions, planner trials, goal-level
approaches and port-level success denominators separate. Clearance minima include
inspected path/approach samples; they are unsigned separations, not safety margins.
The ledger includes additional start/diagnostic/viewer queries; per-task planning
query counts intentionally exclude later rendering and separate strict sessions.

Each run refuses any existing output path (including symlinks), stores complete
precision NPZ with no object arrays, source/input/file SHA256, semantic snapshots,
common start, task/scene identities, Git HEAD and an uncommitted-source-set digest.
The independent verifier makes zero new FCL queries: it checks hashed saved evidence
and FK/threshold/provenance/summary consistency. Fresh FCL validation is performed
during planning. Neither check is continuous collision or hardware certification.
Independent report and main Manifest are bound by validation_manifest.json.

## Reproduction commands

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
export PYTHONDONTWRITEBYTECODE=1
export C1_BACKEND="$PWD/results/c3/assembly_build_20261104/build/collision_backend"

.venv/bin/python -B repro/scripts/doctor.py --mode core
.venv/bin/python -B repro/scripts/doctor.py --mode full --backend "$C1_BACKEND"

# New output required; do not reuse the recorded run directory.
.venv/bin/python -B scripts/evaluate_task_benchmark.py \
  --backend "$C1_BACKEND" --allow-candidate-baseline \
  --seed-prefix results/g4_run01/experiments/workspace_n1000_seed42 \
  --scene config/scenes/idc_prototype_v0.yaml \
  --task-template config/tasks/p22_research_v0.yaml \
  --ports P11 P12 P13 P21 P22 P23 P31 P32 P33 \
  --viewer-ports P11 P22 P33 --summary-viewer \
  --output-dir results/task_benchmark/t5_1_s1_user_review_01

.venv/bin/python -B scripts/evaluate_task_benchmark.py \
  --verify-run results/task_benchmark/t5_1_s1_20261104_03

.venv/bin/python -B scripts/compare_task_p22.py \
  --benchmark results/task_benchmark/t5_1_p22_equivalence_20261104_02 \
  --ik-run results/task_reachability/p22_20261104_02 \
  --path-run results/task_planning/p22_20261104_02 \
  --approach-run results/task_approach/p22_20261104_03

.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m core tests
.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m integration tests \
  --native-backend "$C1_BACKEND"
.venv/bin/python -B -m pytest -q -rs -p no:cacheprovider -m artifact tests \
  --artifact-root "$PWD/results" --native-backend "$C1_BACKEND" \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source
.venv/bin/python -B repro/scripts/check_clean_checkout.py
```

Native CTest can be run from a new build directory using the unchanged build
commands in repro/README.md. This round reused the existing binary and copied its
CTestTestfile into a temporary directory, so even historical CTest logs were not
modified. Missing explicitly requested artifacts BLOCK with exit 4 via R2 preflight.
The benchmark CLI uses exit 3 for missing inputs/system-blocked runs, 2 for errors,
and 0 for a completed experiment even when some ports have ordinary task failures.
A completed experiment is not the same as every task succeeding.

## Review and next-stage boundary

The saved P22 equivalence report compares 123 attempts, four paths and 52 approach
samples with declared 1e-10 tolerances, excluding time/query IDs. Results and full
regression counts from this round are in the accompanying validation_report.json.
Historical data is never rewritten. Results are still ignored by Git; no upload
or archive recovery is added here. Saved absolute input paths retain the R2-C/D
relocation limitation; explicit artifact roots cannot silently use another checkout.

For T5.2, keep tasks, mounting assumptions, start and budgets fixed while changing
only declared SceneSnapshots. Distinguish posture/axis infeasibility from actual
obstacle-induced path rejection. The current S1 run cannot establish statistical
generalization or superiority of a planner. The 5-DoF/task constraint interaction,
real TCP calibration, candidate SRDF mechanical review, RJ45/cable collision coverage,
continuous collision checks and physical starting-state validity remain open.
T5.2, workspace-aware optimization, insertion/contact and hardware control are not implemented.

## Minimal changes to existing files

The shared `viz/task.py` had a hardcoded P22 point name/title. Both now derive the
selected Port ID; P22 text and all geometry/assessment behavior are preserved.
This is a display adapter, not a change to a planning algorithm. Native integration
tests cover P11/P22/P33 labels with real complete robot surfaces.

The R2 clean-copy helper now includes pending source/scripts/config files as well
as tests/repro. Otherwise uncommitted T5.1 modules would be absent from the isolated
copy. Discovery remains restricted to those directories and excludes results,
venv, caches and validation outputs. It does not copy results.zip or user notes.
Architecture tests add pure-import checks for task.template and viz.task_benchmark;
existing assertions are preserved. tests/dependencies.json declares every new test.

Early T5.1 development runs (_01/_02) are retained. The selected nine-port run is
`results/task_benchmark/t5_1_s1_20261104_03/`, generated after the shared-label fix.
The selected standalone P22 equivalence run is `_02`; its P22 labels are unchanged.
The final nine-port run is also compared against the historical P22 chain.

## Actual final validation

Baseline: 651 passed / 1 warning. Final: Core 450, Integration 183, Artifact 56; total 689 (+38), zero failed/blocked/skipped. Integration/Artifact each show the existing Xacro warning. Native CTest 1/1 PASS; isolated history-free copy 450 PASS. Core/Full Doctor ready.

P22 comparison against final nine-port output: 123 attempts, four paths and 52 approach samples; maximum joint/transform/error/clearance differences all zero (declared thresholds 1e-10).

|Port|IK solutions|Candidate|Linear|RRT|Approach|Task runtime s|Min self/env mm|Status|
|---|---:|---:|---|---|---|---:|---|---|
|P11|0|0|0/0|0/0|0/0|3.767|N/A|IK_NO_SOLUTION_WITHIN_BUDGET|
|P12|4|4|4/4|4/4|4/4|98.264|0.7990 / 4.5401|SUCCESS_RESEARCH_ONLY|
|P13|0|0|0/0|0/0|0/0|3.713|N/A|IK_NO_SOLUTION_WITHIN_BUDGET|
|P21|0|0|0/0|0/0|0/0|3.654|N/A|IK_NO_SOLUTION_WITHIN_BUDGET|
|P22|2|2|2/2|2/2|2/2|64.559|0.7990 / 4.5539|SUCCESS_RESEARCH_ONLY|
|P23|0|0|0/0|0/0|0/0|4.010|N/A|IK_NO_SOLUTION_WITHIN_BUDGET|
|P31|0|0|0/0|0/0|0/0|2.534|N/A|IK_NO_SOLUTION_WITHIN_BUDGET|
|P32|0|0|0/0|0/0|0/0|2.690|N/A|IK_NO_SOLUTION_WITHIN_BUDGET|
|P33|0|0|0/0|0/0|0/0|2.761|N/A|IK_NO_SOLUTION_WITHIN_BUDGET|

0/0 here means NOT_RUN, not a failed executed trial. Successful ports: 2/9 under one declared scene/start/seed only. Six pose solutions, 12 independently rechecked planner trials, six successful goal-level approaches, 1107 IK optimizations. Total benchmark runtime including strict controls/rendering/writing: 196.352 s.

Failed ports have POSITION_REACHABLE best approximations: P11/P13/P21/P23 position about 0.339 mm but insertion-axis about 14.733 degrees; P31/P33 axis about 22.066 and key about 16.611 degrees; P32 axis/key about 20.407 degrees. These are finite-budget findings, not a global impossibility proof. Downstream collision/path/approach was not attempted for these failed task poses.

Selected outputs: final run manifest.json / summary.json / summary.csv / ports/<ID>/result.json and NPZ. overview.html is compact; P22 has two path/approach replays; P11/P33 have explicitly failed best-approximation viewers. Additional P12 goal0 path/approach replays and their fresh Native geometry-query evidence are under ../t5_1_validation_20261104_01/figures/P12/. To generate more port viewers in a new run, include P12 (or any desired evaluated port) in --viewer-ports.

Actual integrity: 1705 original paths rehashed, all historical results/config/cpp/robot/planning/collision files unchanged; only existing src/viz/task.py has the authorized label adapter. All 12 official includes and 49 visual/collision mesh references match the prior observed asset baseline. No staging/commit/push, dependencies installed or physical control.
