# T1/T2 — P22 task-constrained reachability

Status: **RESEARCH_ONLY / TASK_CONFIGURATION_CANDIDATE** under the declared virtual TCP.
T1/T2 software and actual experiment completed; physical TCP/model validation remains PENDING.
No motion path, actual plug/unplug, hardware control or VLA integration was performed.

## Declared task and coordinate conventions

P22 is read from the existing S1 SceneSnapshot/FrameGraph, not copied into IK code.
Actual resolved values (base_link coordinates):

| Item | Value |
|---|---|
| P22 position | (0.33,0,0.22)m |
| Insertion direction | +X |
| Key reference direction | +Z |
| Pre-insertion distance | 0.040m, synthetic_uncalibrated |
| Pre-insertion TCP position | (0.29,0,0.22)m |
| TCP local insertion/key axes | +X / +Z |
| Position / insertion-axis / key tolerances | 1mm / 3deg / 5deg, research_assumption |
| Gripper / self/environment clearance | r_joint0.3rad / 0m / 0m |

`config/tasks/p22_research_v0.yaml` independently declares the virtual TCP:

```text
T_ee_tcp = [ R_y(-90deg), (0,0,0.030)m ]
TCP +X -> EE +Z; TCP +Z -> EE -X; TCP +Y -> EE +Y.
```

This is an explicit mounting hypothesis, not a measured RJ45/gripper dimension or orientation.
The target TCP rotation is identity for these local tool axes and resolved P22 axes.
The corresponding target EE position is (0.26,0,0.22)m, with R_y(+90deg).
Full matrices, including the distinct original P22 frame rotation, are in targets.json.

```text
T_base_tcp(q) = existing_PyKDL_FK(q) @ T_ee_tcp
p_pre = p_port - d_pre * insertion_axis_base
```

The target rotation maps both declared TCP directions to the scene-derived signed directions.
Key error is the 3D angle between actual and desired key vectors, not an unsigned axis or
projected roll. A180deg key flip fails. Task files reject unknown keys, duplicate keys,
nonunit/nonperpendicular axes, invalid SE(3), invalid units/numbers or unsupported calibration claims.
Changing the TCP, approach or tolerance changes task/parameter hashes; S1 and the official model remain unchanged.
The P22 experiment refuses silent substitution of another port ID.

## IK method and decision hierarchy

SciPy1.8 least_squares, bounded trf, linear loss and numerical2-point Jacobian.
All limits/axes/transforms come from the current URDF through the existing G1 model and PyKDL FK.
No DH angles or another FK were introduced.

Each initial seed is optimized in three stages: position, position+insertion-axis, and full task.
Position residual is divided by the declared position tolerance. Direction-vector differences
are divided by the corresponding angular-tolerance chord, with explicit configurable weights.
Direction differences avoid a zero cross-product residual at180deg misalignment.
Final acceptance independently checks position norm, insertion angular error and signed key-direction error.
Optimizer success does not override those thresholds.

| Highest satisfied cumulative level | Final task status before collision |
|---|---|
| Joint limits invalid or position outside tolerance | IK_FAILED |
| Position only | POSITION_REACHABLE |
| Position + insertion axis | AXIS_ALIGNED |
| Position + axis + key | TASK_POSE_FEASIBLE |
| Full task pose + experimental model collision/clearance pass | TASK_CONFIGURATION_CANDIDATE |

Atomic within_tolerance fields are separate from cumulative position_reachable/axis_aligned/task_pose_feasible flags.
Collision rejection retains TASK_POSE_FEASIBLE with collision_status=REJECTED; it does not become IK failure.
Only the new semantic collision fields and configuration_decision are used. Legacy final_decision is not used for task validity.
If no task pose passes, screen_solutions starts no task collision session. Native queries used solely to draw a failed
best approximation are marked render_diagnostics_only and cannot create a task candidate.

SciPy1.8 does not expose the accepted iteration count. attempts record iterations=null with this limitation,
reported nfev/njev and independently counted residual calls including finite-difference Jacobians.
The123 optimizations below are separate solves, not123 optimizer iterations.

## Actual P22 result

Final run: `results/task_reachability/p22_20261104_02/`.
Initial implementation run01 is retained; no prior results were overwritten.

| Metric | Actual result |
|---|---:|
| Initial seeds |41|
| Existing G4 nearest virtual-TCP seeds |16|
| Deterministic extra joint seeds |24, RNG seed20261104|
| URDF midpoint diagnostic seed |1, not a safe/rest reference|
| Optimization attempts |123 (3 stages per seed)|
| Optimizer success flags |114; not the task pass count|
| Position-within-tolerance attempts |85|
| Position+axis-within-tolerance attempts |51|
| Full-task-within-tolerance attempts |27|
| Unique full-task solutions |2, dedup max joint difference<=0.001rad|
| IK wall time |4.112s on this run|

The85/51/27 counts include staged/duplicate results, not distinct solutions or sampling success probabilities.
G4 seed indices, initial q, stage-specific initial q, optimizer status, evaluation counts,
errors, active bounds and times are recorded per attempt.

| Solution | q1..q5 [rad], rounded for presentation |
|---|---|
|0|[0,-0.2169061521,1.9244915420,-0.1367890631,0]|
|1|[0,1.7078979206,-1.9244915420,1.7873899482,0]|

Both solve the declared pre-insertion TCP target; numerical position error is below1e-6mm
and both direction errors below1e-6deg. These are floating-point model residuals, not hardware accuracy.
Full precision q/transforms/errors are in ik_results.json.

| Collision model | Task-pose solutions checked | Self rejected | Environment rejected | Task configuration candidates |
|---|---:|---:|---:|---:|
|strict SRDF v0|2|2|0|0|
|standard_moveit_candidate_v0|2|0|0|2|

Independent MoveIt/FCL sessions use exactly the same official geometry and S1 scene.
The29 candidate exemptions and strict five-pair configuration were not modified.
Strict rejection is from remaining assembly contacts, including green/link1, link2/servo1 and gripper pairs;
solution1 also has link2/link3. It is a strict semantic comparison, not proof of dangerous real interference.

Candidate-model self separations: solution0 about0.8862mm (link1/link2), solution1 about0.9369mm (link4/link5).
Environment separation is about34.536mm for both (l_out_link/router_2_front_bar_3).
Required clearances remain0m. No safety margin was relaxed after solving. Startup+screening wall times:
strict about0.183s, candidate about0.217s.

A separate partial diagnostic has position0.014608mm, axis0.352204deg, key5.293410deg.
Its status is AXIS_ALIGNED, not task feasible; task collision screening is not used to promote it.

## Output, Viewer and reproducibility

- task_config.yaml/task_spec.json/targets.json: exact input and derived transforms/axes.
- source_robot.urdf/model_metadata.json/scene_snapshot.json: expanded source robot and scene evidence.
- seeds.json/ik_results.json/ik_attempts.csv: all41 seeds,123 optimization attempts and2 solution clusters.
- strict_collision.json/candidate_collision.json: separate task checks, model decisions, identities and cleanups.
- viewers.json: full Native queries, q, task errors, TCP transforms and role/source information.
- summary.json/manifest.json: results, source/parameter/input hashes, software versions, robot/SRDF/ACM/scene/backend identities.
- figures/candidate_best.html: complete Robot+IDC with P22, target/actual TCP, insertion/key axes and errors.
- figures/strict_same_best.html: same task-feasible q rejected by strict SRDF.
- figures/partial_task_pose_diagnostic.html: position/axis pass but key tolerance fails.

All24 robot collision meshes and38 obstacles are present. Target references are not robot geometry or workspace points.
The displayed dashed approach is geometric target construction, not an evaluated trajectory.
Pair filter Raw shows actual contacts including disabled assembly pairs; Semantic blocking shows actual blockers.
HTML panel task status and collision-screen status are recorded independently from legacy raw collision labels.
A static geometry preview and Viewer data checks are in `results/task_reachability/p22_validation_20261104_01/`.
Interactive browser acceptance remains user review work.

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
export C1_BACKEND="$PWD/results/c3/assembly_build_20261104/build/collision_backend"

ctest --test-dir results/c3/assembly_build_20261104/build --output-on-failure
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests

# Read-only artifact/input/FK/error verification; no IK or collision re-run.
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_p22_task.py \
  --verify-run results/task_reachability/p22_20261104_02

# New experiment: output directory must not already exist.
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_p22_task.py \
  --backend "$C1_BACKEND" --allow-candidate-baseline \
  --task-config config/tasks/p22_research_v0.yaml \
  --seed-prefix results/g4_run01/experiments/workspace_n1000_seed42 \
  --output-dir results/task_reachability/p22_user_review_01

xdg-open results/task_reachability/p22_20261104_02/figures/candidate_best.html
xdg-open results/task_reachability/p22_20261104_02/figures/strict_same_best.html
xdg-open results/task_reachability/p22_20261104_02/figures/partial_task_pose_diagnostic.html
```

To change assumptions, create an explicit new task configuration and use --task-config with a fresh output directory.
Do not edit historical task snapshots or automatically adjust P22/tolerances/ACM to obtain success.
A finite failed multi-start budget would be reported as no solution found, not a proof of global unreachability.

## Code and testing

New task/spec.py, task/targets.py, planning/ik.py, planning/validity.py, viz/task.py,
experiments/p22_reachability.py, CLI, task YAML and two test modules.
Existing viz/collision.py adds optional task-check display; its legacy interface/behavior remains available.
Architecture tests explicitly register the new task/planning domains, keep old boundaries, add pure task/Viewer imports;
no historical acceptance assertions were removed or relaxed. No path-planning algorithm was added.

Baseline542 passed,1 warning,0 skipped. Final573 passed,1 warning,0 skipped. Native CTest1/1 PASS.
Tests include FrameGraph/TCP transforms, independent settings, key/axis flips, nonstandard axes,
reachable FK-generated target, bounded/failure IK, deterministic G4 seeding, dedup, optimizer-success false positives,
actual strict/candidate and test-only environment-obstacle screening, and a budget-limited failed end-to-end run
that still emits a best-approximation Viewer with0 task collision queries/candidates.

## Conditions before T3

1. Calibrate the actual grasped RJ45 TCP, forward/key axes and dimensions; current solution existence is conditional on a synthetic mounting orientation.
2. Confirm actual joint conventions/stops and a trusted current resting JointState. Solution1 joint2 is about97.86deg in URDF coordinates;
   vendor DH/controller range interpretation and hardware applicability remain unverified. No undeclared range restriction was added here.
3. Complete mechanical review of the29 exemptions; include LiDAR, RJ45 body and cable collision coverage and physical clearances.
   Bare-gripper collision pass is not full RJ45 collision verification; current self gaps are below1mm.
4. Select a measured start state and task/gripper constraints before T3; continuous joint motion and the insertion segment need separate collision/path checks.

Software can identify current model-level P22 pre-insertion candidates. Real insertion feasibility, calibrated accuracy,
physical safety and collision-free trajectory existence remain unverified.

Reference: [SciPy1.8 least_squares](https://docs.scipy.org/doc/scipy-1.8.0/reference/generated/scipy.optimize.least_squares.html).
