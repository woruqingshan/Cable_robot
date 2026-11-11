# T5.3-A restricted workspace and neighboring-port obstacles

All results are RESEARCH_ONLY. Baseline HEAD is
2f308d8924695085093f5c14401658ba5751fea1. Actual pre-change regression was
749 passed, 1 warning. The first computation directory ending in _01 is a
retained diagnostic attempt. Its readback exposed a Native/KDL floating-point
representation mismatch. The _02 computation uses the existing PyKDL FK for
all TCP boundary decisions and keeps Native transforms as independent evidence.

## Coordinates and validity

World is the original B0 base_link. At World TCP x < 0.250 m the new constraint
is inactive. At x >= 0.250 m it requires y in [-0.150,0.150] and z in
[0.040,0.400], including their endpoints. The constraint applies to the virtual
TCP. The full robot continues to use Native FCL for collisions.

`planning/workspace_constraint.py` owns validated parameters, World TCP
transforms, boundary diagnostics and discrete switch refinement.
`ConfigurationValidity(..., workspace_check=...)` composes the constraint with
the existing semantic FCL result before caching or exposing a state to the
planners. It stores `fcl_valid` separately from the final conjunctive `valid`.
Fresh checks bypass the exact-q cache. Each checker belongs to one fixed
scene/base/TCP/constraint session. Existing callers omit the optional argument.

Linear, RRT and final validation use this same checker. Historical approaches
are rechecked through it and the existing Cartesian task-error checker. No new
multi-start IK or sequential Approach search is performed by this runner.

Edges retain their requested coarse resolution. For intervals whose endpoints
or midpoint indicate a crossing or proximity within 2 mm of the x=250 mm
switch, additional samples have at most .001 rad joint increments. Every
additional configuration is queried and retains interpolation provenance.
This finite adaptive sampling does not establish continuous safety or capture
every possible nonlinear excursion between samples.

## Scene and problem definitions

Config is `config/restricted_scenes/d0_d4_v0.json`. Each Box has World size
35 x 30 x 24 mm, x range [295,330] mm and y/z centered on its neighbor port.
The fixed World scene is generated first, then passed through the existing
`derive_scene` for the station. Schema v1 and all original geometry remain
the source of truth. Scene namespaces change consistently, while object IDs,
dimensions and all original World transforms remain equal.

| Scene | Target | New obstacle ports | Total objects |
| --- | --- | --- | ---: |
| D0 | P22 | none | 38 |
| D1 | P23 | none | 38 |
| D2 | P23 | P22 | 39 |
| D3 | P22 | P21, P23 | 40 |
| D4 | P22 | P12, P21, P23, P32 | 42 |

`geometry_audit.json` records new Box World bounds and contact with original
geometry using the axis-aligned S1 Box AABBs. Touching the front panel at
x=330 mm is recorded as surface contact. It checks original object transforms,
dimensions and all nine port transforms, and rejects target origins inside
new Boxes. Native integration separately demonstrates clear and new-Box
colliding robot configurations.

B0/P22 and xm30/P22 each run D0/D3/D4, while ym70/P23 runs D1/D2. All eight
use workspace ON. The three matching no-local-obstacle reference conditions
use workspace OFF. Each condition preserves both saved goals, original G4
sample 0, all task tolerances, the candidate SRDF and historical planner settings.
Workspace OFF references also replan to provide the same planner trial records.

The source collection requires its externally selected Manifest SHA256. Each
source child passes the existing T5.2 verifier before q is read. The new scene
targets are compared numerically to the historical Base-frame targets. Static
checks, historical trajectory revalidation and fresh planning have distinct
records. Invalid endpoints produce NOT_RUN planner trials. Approach validity
is reported independently of reaching the pre-insertion goal.

## Data and independent verification

Every condition saves result.json, scene_snapshot.json, world_scene.json,
base_pose_context.json, geometry_audit.json, task_spec.json, Native query ledger,
path NPZs, manifest.json and a full-verification receipt. The result contains all
historical and newly checked configurations, first failures, clearances and
source artifact references. Goal IDs retain their original source solution IDs.

`verify_restricted` verifies listed hashes, the historical source anchor,
original goals/start, Native/KDL pose consistency, recomputed World TCP boundary
decisions, combined validity, trajectory ledger provenance and Approach task
errors. It performs no new Native collision query.

The read-only RViz adapter invokes the verifier and existing source reader.
Accepted new paths use their NPZ samples. Accepted historical Approach samples
are loaded from the hash-bound result JSON into immutable replay arrays. All
goals remain inspectable. Failed paths are excluded from playable trajectories.
Orange solids are the same new Boxes queried by FCL. Magenta lines show the
conditional workspace boundary over a finite display interval, with red spheres
for first failed historical TCP positions. The visualization's x=360 mm end is
only the display extent. The mathematical workspace has no upper x limit.

## Reproduction

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_restricted_benchmark.py \
  --source-run results/base_pose_benchmark/t52_r2_s1_20261104_01 \
  --expected-source-manifest-sha256 11c4f1eb77d0617ab281e047e25f6ebafc538336f4dd9ee89e70dcd2304252e6 \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --allow-candidate-baseline \
  --output-dir results/restricted_benchmark/t53_a_user01

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B repro/scripts/report_restricted.py \
  --run results/restricted_benchmark/t53_a_user01 \
  --output-dir results/restricted_benchmark/t53_a_user01_analysis
```

Use fresh names on reruns. The reporting CLI generates failure_diagnosis.json
and analysis_summary.json. Its denominator is goals within each condition.
Planner trials count only valid endpoint pairs. Approach counts remain separate.
It maps the legacy URDF_JOINT_LIMIT reason to JOINT_LIMIT for diagnosis.

```bash
colcon --log-base results/rviz_replay/build_t53_user/log build \
  --base-paths ros2/jetrover_idc_visualization \
  --build-base results/rviz_replay/build_t53_user/build \
  --install-base results/rviz_replay/build_t53_user/install --symlink-install
source results/rviz_replay/build_t53_user/install/setup.bash

condition="$PWD/results/restricted_benchmark/t53_a_20261109_02/b0_D4_workspace_on"
condition_sha=$(sha256sum "$condition/manifest.json" | cut -d ' ' -f 1)
# This local hash is appropriate after independently selecting/verifying the run.
ros2 launch jetrover_idc_visualization replay.launch.py \
  workspace_root:="$PWD" benchmark_dir:="$condition" \
  expected_manifest_sha256:="$condition_sha" domain_id:=83
```

Select the goal and planner in the existing panel. Play is available only for
accepted stored trajectories. A successful Path with failed Approach remains a
separate outcome, shown in the goal status. Review first failures and the full
collision pairs in result.json or failure_diagnosis.json.

```bash
source results/rviz_replay/build_t53_a_20261109/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider \
  -m 'core or integration or artifact' tests \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend \
  --artifact-root "$PWD/results" \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source \
  --replay-install "$PWD/results/rviz_replay/build_t53_a_20261109/install"
```

## Actual experiment outcome

Primary collection is `results/restricted_benchmark/t53_a_20261109_02/`.
Detailed diagnostics and interactive World geometry are in the sibling
`t53_a_20261109_02_analysis/`. Each condition has two original saved goals.
In the table, P means an accepted discrete result, F means failed final
validation and T means the original 20-second RRT budget expired. Every
start and goal passed task pose, limits, FCL and its declared workspace rule.

| Scene | Base | Port | Workspace | Goal 0 Linear / RRT / Approach | Goal 1 Linear / RRT / Approach |
| --- | --- | --- | --- | --- | --- |
| D0 | B0 | P22 | OFF | P / P / P | P / P / P |
| D0 | B0 | P22 | ON | P / P / P | F / P / P |
| D3 | B0 | P22 | ON | P / P / P | F / P / P |
| D4 | B0 | P22 | ON | P / P / P | F / P / P |
| D0 | xm30 | P22 | OFF | P / P / P | P / P / P |
| D0 | xm30 | P22 | ON | P / P / P | F / P / P |
| D3 | xm30 | P22 | ON | P / P / P | F / P / P |
| D4 | xm30 | P22 | ON | P / P / P | F / P / P |
| D1 | ym70 | P23 | OFF | P / P / P | P / P / P |
| D1 | ym70 | P23 | ON | F / T / P | F / P / P |
| D2 | ym70 | P23 | ON | F / T / P | F / P / P |

The eight main conditions contain 16 goal evaluations. Linear accepted 6/16,
RRT 14/16 and historical Approach 16/16. The three OFF references accepted
6/6 for each stage. Across all 22 goal evaluations, Linear accepted 12/22,
RRT 20/22 and Approach 22/22. All eight main port conditions have at least one
branch with an accepted path and independently accepted historical approach.
These denominators describe this fixed set and seed, rather than randomized
planner success probabilities.

B0/P22 Goal 1's old Linear path first violates Y at World TCP approximately
[0.25000052,-0.19714468,0.27472106] m, 47.14 mm outside the lower Y bound.
The old RRT path first violates it by 71.60 mm. Fresh RRT succeeds with the
same endpoints. This is already observable in D0, where no local Box exists.

The added Boxes also have a distinct, actual FCL effect later in old RRT
paths. For Goal 1, B0 D3/D4 each has five old-RRT samples colliding between
r_out_link and the corresponding P23 Box. xm30 D3/D4 each has 27. Their D0
counterparts have no such environment collision. The new Goal-1 RRT waypoint
sequences differ between D0 and D3/D4 at both stations. First-failure reporting
alone hides these later environment contacts, so failure_diagnosis.json includes
all checked stage reason counts and actual pair occurrences as well.

ym70/P23 Goal 0 times out in D1 and D2 while Goal 1 succeeds. The evidence
therefore supports testing branch selection and constrained transit structure
before claiming a new search algorithm is necessary. A separate budget/seed
study would be needed to characterize timeout robustness. The final 40-to-10 mm
Approach is not the bottleneck in this fixed experiment. The extra D4 P12/P32
Boxes did not add historical contacts beyond the P23 blocker in these paths.
Compare D2 with D1 and D3/D4 with D0 within the same station/target.

## Validation and visual capture history

Final full regression is **780 passed, 1 warning, 0 skipped**, 479.69 seconds.
Core has 523 cases, Integration 189 and Artifact 68. Native CTest is 1/1 PASS
with its runtime log redirected to a temporary test directory. Environment
Doctor is READY_FOR_RESEARCH_REPRODUCTION. No dependency installation occurred.

The actual restricted ROS test loaded D4's 42 scene solids and boundary marker,
observed every one of the new RRT path's 426 saved samples, and checked TF,
gripper mimic and fixed World geometry. Maximum EE TF error was 1.67e-16 and
World object marker error was zero. Owned nodes exited normally with no residual
nodes. The same test is now part of the permanent regression.

D0-D4 were actually loaded into the production RViz/Qt launch, with logs and
real Qt panel PNGs in `results/rviz_replay/validation_t53_a_20261109/`.
However, the display capture returned uniform black RViz images. Native OpenGL
initialization and ROS data transport succeeded, but actual RViz render-image
acceptance is **BLOCKED by display capture**. The first D0 capture helper
reported PASS before pixel QA was added; its black PNG is explicitly rejected
as visual evidence. Subsequent captures fail explicitly when images are uniform.
The new guard prevents a uniform image being accepted automatically.

The World geometry HTMLs are generated from the hash-bound queried scene data
using the existing renderer, under `t53_a_20261109_02_analysis/world_scene_figures/`.
Live RViz can be reviewed using the launch command above. To retry capture on
an available desktop, run the following with a fresh output name.

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B repro/scripts/capture_replay.py \
  --benchmark-dir "$condition" --expected-manifest-sha256 "$condition_sha" \
  --goal 1 --planner rrt_connect --phase 'Path + Approach' \
  --output-dir results/rviz_replay/validation_t53_user_D4
```

The initial handoff retained the capture block above. A follow-up on the unlocked
desktop confirmed normal playback with black external-window captures under
Wayland/XWayland. Software OpenGL and compositor-pixmap attempts did not provide
readable window pixels. Those diagnostic attempts are retained separately.

The follow-up capture uses the installed official RViz VisualizationFrame and
its RenderWindow.captureScreenShot API. The small helper under
`repro/rviz_snapshot/` loads the same replay.rviz config and subscribes to the
same private production ROS replay topics. It saves the actual render viewport
without relying on operating-system window readback. The existing Qt panel is
captured separately. No new planning or geometry algorithm is used.

D0-D4 native render and Qt PNGs, per-scene selected q/frame, logs, source anchors
and image hashes are in
`results/rviz_replay/validation_t53_a_20261109_native_capture/`.
All five renderer and launch processes returned 0 with normal cleanup. All five
render images were checked for nonuniform pixels and manually inspected. D0
shows the accepted RRT Path+Approach at its final frame. D1/D2 inspect the
no-path Goal 0 with Play disabled. D3/D4 inspect the first saved frame of the
selected valid Path+Approach. The selected q and source frame are recorded in
each evidence.json.

The snapshot helper build and five actual captures passed. Screenshot pixel QA
and related existing replay/workspace tests passed **56 cases**, including three
new pixel-guard cases. The preceding 780-case full computational regression is
retained as the prior full run, rather than reporting a new full-suite count.

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
source results/rviz_replay/build_t53_a_20261109/install/setup.bash
cmake -S repro/rviz_snapshot -B results/rviz_replay/rviz_snapshot_user_build
cmake --build results/rviz_replay/rviz_snapshot_user_build -j2

QT_QPA_PLATFORM=xcb PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B repro/scripts/capture_replay.py \
  --benchmark-dir "$condition" --expected-manifest-sha256 "$condition_sha" \
  --goal 1 --planner rrt_connect --phase 'Path + Approach' \
  --render-capture-binary results/rviz_replay/rviz_snapshot_user_build/rviz_snapshot \
  --output-dir results/rviz_replay/validation_t53_user_native_D4
```

Current status is SOFTWARE_EXPERIMENTS_AND_NATIVE_CAPTURE_VERIFIED_PENDING_USER_REVIEW.
Physical tool dimensions, candidate assembly semantics, hardware calibration
and continuous execution remain research limitations. No T5.3-B/T6 work or Git
write operation was performed.
