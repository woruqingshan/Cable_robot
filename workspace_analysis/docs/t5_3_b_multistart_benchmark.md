# T5.3-B Multi-start and local-only constrained planning

RESEARCH_ONLY. Baseline commit: 69d5a0991099cdbf57d6505b930c8b28cc88d5ec.
No physical Home, hardware motion, new IK, modified RRT, altered collision policy,
or continuous collision guarantee is introduced.

## Interfaces and preserved behavior

The existing conditional WorkspaceConstraint and ConfigurationValidity are unchanged.
LocalOnlyWorkspaceConstraint is an explicit separate subclass: when enabled it
requires World TCP x >= .250, y in [-.150,.150], z in [.040,.400], inclusive.
It uses the existing TCPWorkspaceCheck binding/refinement and the existing exact-q
cache and fresh final validation. The local-only mode has its own config/hash;
it participates in every search/configuration/edge query, not post-filtering.
Native FCL and the original World/Base adaptation are unchanged.

Source conditions come from the hash-anchored T5.3-A collection and pass its verifier.
The original task config, scene, base context, q goals and planning settings are
reused. No target multi-start IK or new Cartesian Approach is run. Approach is
outside this experiment's start-to-pre-insertion comparison.

## Start selection, frozen before planning

config/restricted_scenes/ready_pose_v0.json declares all criteria and finite pools.
First inspect the validated G4 1,000 seed42 rows. Only if no verified candidate
exists for a Start type, inspect the existing G2 10,000 seed42 rows, using exactly
the same criteria. Neither pool is regenerated. Both pass load_results(model=...).
There is no undeclared range relaxation or planner-informed selection.

S0 is exactly G4 Sample 0, compared against the saved historical start.
S1 requires World TCP X [180,240] mm, target-relative Y within 40 mm and Z within
70 mm. S2 uses X [255,280] mm, the same Y/Z offsets, World workspace inclusion,
25–70 mm TCP distance from the pre-insertion pose, and >= .15 rad Euclidean
joint distance from EACH saved goal. Both require >= .15 rad minimum absolute
joint-limit margin and |joint1| <= 1 rad. These are predeclared research filters,
not measured hardware Ready Pose specifications or task orientation constraints.

All region-eligible candidates receive Native semantic FCL/World workspace checks.
Within the first qualifying pool, rank by descending minimum joint margin, then
descending available environment separation, ascending |joint1|, then ascending
original sample index. Unavailable clearance sorts last within equal margin.
B0/P22 selects once in D0 and reuses the same q for D3 and both goals. ym70/P23
selects once in D1. New obstacles can invalidate a start; no substitute is selected.
No-qualified selections and invalid endpoints remain explicit NOT_RUN trials.

Every start has its own canonical SHA256, pool NPZ/JSON hashes, original index,
World TCP, rules, complete eligible-candidate checks and pool counts. S2 selection
does not prove a safe preparation motion from S0/Home to S2.

## Matrix and metrics

D0/B0/P22, D3/B0/P22 and D1/ym70/P23 each use S0/S1/S2, both original goals
and both planners in conditional mode (36 trials). S2 additionally uses local-only
in D0 and D1 (8 trials). Both planners retain the original 20 s RRT search budget,
seed 20261104, step .25 rad, edge .02 rad and final .01 rad sampling, including
existing .001 rad switch refinement. Search timeout is not a proof of infeasibility.

Each trial records raw returned waypoints, fresh final validation samples, separate
search/validation latency, query count, joint polyline length, checked World TCP
length, endpoint distance and detour ratio, sampled minimum unsigned FCL separation,
and whether any checked TCP has X < .250. For S0/S1 the exit flag may simply mean
the start is outside; only S2 can test retreat from an initially internal state.
Near-zero endpoint distance (<=1e-6 m) gives an undefined detour ratio, not infinity.
Rejected attempted paths retain diagnostics/metrics but cannot be replayed as valid.

## Storage and independent verification

Every condition has result.json, start.json, scene/task/context, path NPZs, a gzip
query ledger and a SHA256 manifest. Query records omit duplicate per-link matrices,
contact arrays and joint-name maps, retaining Native EE, q, semantic pair/clearance
results and World workspace evidence. Paths reference canonical query IDs rather
than duplicating the ledger. No raw collision field substitutes semantic validity.

verify_multistart checks source anchors, original goals/settings/scenes, start-to-pool
identity, Native/KDL EE consistency, every workspace decision, exact interpolation
and refinement grid, fresh final query IDs, NPZ states/TCP/clearance, and path quality.
It performs no collision query; the production planner's independent final check
already makes fresh real Native calls. This receipt is discrete evidence only.

The replay adapter verifies all evidence and returns the existing immutable Catalog.
Each condition directory fixes one real Start/mode. Only accepted paths are playable;
failed trial states remain inspectable as goals/diagnostic markers. The panel detail
and Ready Pose marker identify Start/mode, and local-only is labelled explicitly.

## Reproduction

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_multistart_benchmark.py \
  --source-run results/restricted_benchmark/t53_a_20261109_02 \
  --expected-source-manifest-sha256 a4b6c68c354bd2b9c6ddf51bcaa0ea566f1d62d9e4e55c2d21c0b91ba139d872 \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --allow-candidate-baseline \
  --output-dir results/multistart_benchmark/t53_b_user01

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B repro/scripts/report_multistart.py \
  --run results/multistart_benchmark/t53_b_user01 \
  --output-dir results/multistart_benchmark/t53_b_user01_analysis

colcon --log-base results/rviz_replay/build_t53_b_user/log build \
  --base-paths ros2/jetrover_idc_visualization \
  --build-base results/rviz_replay/build_t53_b_user/build \
  --install-base results/rviz_replay/build_t53_b_user/install --symlink-install
source results/rviz_replay/build_t53_b_user/install/setup.bash
condition="$PWD/results/multistart_benchmark/t53_b_user01/b0_D0_S2_local_only"
condition_sha=$(sha256sum "$condition/manifest.json" | cut -d ' ' -f 1)
ros2 launch jetrover_idc_visualization replay.launch.py \
  workspace_root:="$PWD" benchmark_dir:="$condition" \
  expected_manifest_sha256:="$condition_sha" domain_id:=83
```

Select Path (there is no new Approach in this run). Change the condition directory
for S0/S1/S2 and conditional/local-only. Expected-manifest SHA must refer to the
independently selected condition, not a different mode. Always use fresh output names.

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider \
  -m 'core or integration or artifact' tests \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend \
  --artifact-root "$PWD/results" \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source \
  --replay-install "$PWD/results/rviz_replay/build_t53_b_user/install"

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B repro/scripts/doctor.py --mode full \
  --backend results/c3/assembly_build_20261104/build/collision_backend
```

Actual results, regression and visual evidence are appended after execution.

## Actual fixed-seed experiment result

All 44 scheduled trials executed; every start and goal passed endpoint checks. No Goal IK or new Approach was generated.

| Condition | Goal 0 Linear / RRT | Goal 1 Linear / RRT |
| --- | --- | --- |
| b0_D0_S0_conditional | PASS / PASS | REJECTED / PASS |
| b0_D0_S1_conditional | PASS / PASS | PASS / PASS |
| b0_D0_S2_conditional | PASS / PASS | REJECTED / PASS |
| b0_D0_S2_local_only | PASS / PASS | REJECTED / PASS |
| b0_D3_S0_conditional | PASS / PASS | REJECTED / PASS |
| b0_D3_S1_conditional | PASS / PASS | PASS / PASS |
| b0_D3_S2_conditional | PASS / PASS | REJECTED / PASS |
| ym70_D1_S0_conditional | REJECTED / TIMEOUT | REJECTED / PASS |
| ym70_D1_S1_conditional | PASS / PASS | PASS / PASS |
| ym70_D1_S2_conditional | PASS / PASS | REJECTED / PASS |
| ym70_D1_S2_local_only | PASS / PASS | REJECTED / PASS |

S0 accepts Linear 2/6 and RRT 5/6; S1 accepts 6/6 for each; S2 conditional accepts Linear 3/6 and RRT 6/6. S2 local-only accepts Linear 2/4 and RRT 4/4. These are this matrix’s fractions, not population success probabilities.

S0/ym70/Goal 0 repeats the 20 s RRT timeout. S1 succeeds for both goals at every tested scene. S2 Goal 1’s direct interpolation collides with r_out_link / router_2_front_top in all matching conditions; RRT returns accepted alternatives. All accepted S2 conditional paths also stay inside x >= .250 at checked samples. Therefore this dataset does not demonstrate a required retreat outside the region; local-only validates that retreat is forbidden during search as well.

In B0/D0 RRT, Goal 0 checked TCP lengths are S0 417.93 mm, S1 113.49 mm and S2 136.95 mm; joint lengths are 2.6849, 1.9648 and 1.7807 rad. S2 has a larger detour ratio (2.907) than S0 (1.534) because its endpoint distance is much shorter; both absolute and normalized lengths must be interpreted together. For Goal 1 S2, the sampled environment separation is only about .796 mm in conditional mode. No hardware safety margin is established.

All native backend/model/SRDF/scene identity fields match each corresponding T5.3-A condition. The same B0 start records and hashes are reused for D0/D3. S1/S2 are position-region-selected research configurations with unconstrained initial tool orientation, not calibrated industrial Ready Poses.

Primary results: results/multistart_benchmark/t53_b_20261109_01/. Analysis, failure diagnostics and 22 XY/XZ projection images: sibling t53_b_20261109_01_analysis/. Each condition has an independent PASS receipt.

The comparison changes complete joint configurations, including initial tool orientation (for example S1 joint5 is +1.074 rad while S2 is -0.663 rad). It is not a controlled study of TCP X alone, and its effects must not be attributed solely to proximity to the cabinet. No preparation path to either Ready Pose or physical Ready Pose calibration has been validated.

### Forward extent of accepted local paths

Local-only intentionally has no upper X bound. Accepted S2 Goal 0 RRT samples
span approximately X=258.249–292.820 mm. Accepted S2 Goal 1 local-only samples
span X=260.420–334.227 mm at both B0/P22 and ym70/P23, exceeding the 330 mm
panel plane by about 4.227 mm. Conditional Goal 1 reaches about 333.349 mm.
These are actual checked TCP values, not plotting-axis limits. The current virtual
RJ45 TCP has no dedicated physical connector collision solid. Thus the local-only
result establishes the requested model-defined discrete condition, not a guarantee
that a real mounted connector remains in front of the panel or avoids its material.
No upper-X rule, tool mesh, task tolerance or SRDF was changed to adjust this result.
Tool geometry/calibration and any intended panel-side envelope require separate
explicit modelling before hardware interpretation.

### Native screenshot reproduction

After sourcing the selected visualization install, build the unchanged RViz helper
in a fresh directory and capture either the first real path frame or a completed
playback. The PNG is the official RViz render viewport, not a synthetic plot.

```bash
cmake -S repro/rviz_snapshot -B results/rviz_replay/t53_b_snapshot_user
cmake --build results/rviz_replay/t53_b_snapshot_user -j2
QT_QPA_PLATFORM=xcb PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B repro/scripts/capture_replay.py \
  --benchmark-dir "$condition" --expected-manifest-sha256 "$condition_sha" \
  --port P22 --goal 0 --planner rrt_connect --phase Path --inspection-only \
  --render-capture-binary results/rviz_replay/t53_b_snapshot_user/rviz_snapshot \
  --output-dir results/rviz_replay/t53_b_user_start_capture
```

Omit --inspection-only to actually play the accepted sequence to the last frame.
A failed planner can be inspected but must never be played as an accepted path.

## Executed regression and baseline equivalence

The full current run passed **807 tests, 1 warning, 0 skipped** in 629.42 s:
Core 541, Integration 193, Artifact 73. Native CTest passed 1/1, and Environment
Doctor reports READY_FOR_RESEARCH_REPRODUCTION. The unchanged Native test executable
was exercised through a temporary CTest directory to avoid writing historical logs.
The replay package built in the new isolated results/rviz_replay/build_t53_b_20261109.

All 379 protected historical bytes/algorithm configs passed SHA256 checks. S0's
start/goals/native identities/settings and returned waypoints match T5.3-A exactly
for all 12 corresponding trials; elapsed times are not required to match. Evidence
is under results/multistart_benchmark/validation_t53_b_20261109/.

The new actual ROS Local-only probe observed all 140 original Goal 0 RRT samples,
verified the selected real start q, World TCP minimum X=.2582494397779351 m,
38 fixed World objects, gripper/mimic TF and EE TF maximum error 1.94e-16.
The owned launch exited 0, without forced termination or residual nodes.

The initial screenshot attempt inside the sandbox failed before connecting to X11
and is retained as a rejected diagnostic. Authorized desktop access is used for
native RViz/Qt evidence; no screenshot from the failed attempt is accepted.

## Final native visual evidence and UI follow-up

Eight real native RViz + Qt capture sets (16 PNGs) are retained under
results/rviz_replay/validation_t53_b_20261109_native_capture/. They include the
three actual starts at frame 0, B0 S2 local-only Goal 1 at frame 339/339,
D3 S1 Goal 1 complete playback, ym70/P23 S2 local-only complete playback, and
rejected Linear goal inspections. All renderer/launch exits are 0, without forced
shutdown. Pixel QA passed and the images were manually inspected.

The last UI-only follow-up exposes the actual failed planner's reasons and
environment pair directly in the existing panel, without changing stored data or
raw planner status. Its final targeted unit/artifact/actual ROS regression passed
21 cases, 1 warning, in 114.65 s. The preceding 807-case full run predates this
last diagnostic-text addition; it is not presented as a second full current run.
S2_linear_rejected_reason is the accepted final diagnostic screenshot.

Core implementation/config/API files are local_workspace.py, path_quality.py,
ready_pose.py and multistart_benchmark.py. Existing FK, conditional workspace,
ConfigurationValidity, RRT and Native sources are unchanged. The original complete
selection/evidence SHA256 includes measured query timing; for a timing-independent
q/source/rule digest see ready_configuration_identities.json in the validation
directory. Neither receipt changes the original experiment's identity.

Final status: T5.3-B Completed — Pending User Review.
