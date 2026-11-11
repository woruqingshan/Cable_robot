# T5.3-C RRT robustness, standard shortcutting and preparation motion

RESEARCH_ONLY. Baseline Git HEAD is f2ae70fc41968e4a5dc05beeaa5f5f7d1521b7d0.
The baseline worktree contains only pre-existing untracked docs/develop and
workspace_analysis/results.zip. Neither is staged, deleted or packaged implicitly.

## Interface audit and frozen inputs

The original RRT-Connect remains unchanged: bounded five-joint Euclidean distance,
max_time_s=20, max_iterations=1500, step_rad=.25, goal_bias=.05, edge resolution
.02 rad and final resolution .01 rad. The existing World TCP switch refinement
adds .001-rad samples near x=.250 within the declared 2 mm proximity band.
Timeout is cooperative wall time between Native calls; final validation is separate.
Workspace rejection counts refer to real Native query records, excluding repeated
cache-hit requests; checker request/cache/query statistics are retained separately.

ConfigurationValidity remains the single fail-closed authority for joint limits,
semantic FCL and TCP Workspace. Fresh final checks bypass its exact-q cache. The
conditional rule and separate Local-only class are unchanged. No X upper bound is
introduced, and no tool geometry, mesh, SRDF, IK or collision exemption is altered.

B source collection is results/multistart_benchmark/t53_b_20261109_01, with
externally selected Manifest SHA256:
8fb6bf61298588b3d6912e5e6263c4570da243b8276fb162ae7fe5c0fb48c191.
The new runner verifies that collection and every selected B condition before
reading Start/Goal q. Scene/task/base context snapshots are copied byte-for-byte.
S0/S1/S2 selection is NOT rerun. No target IK or new Cartesian Approach is run.

## Preregistered experiment A

config/restricted_scenes/robustness_v0.json declares seeds 20261104 through
20261113, identical for all three starts. The main denominator is exactly 90:

| Case | Source scene / base / port | Saved goal | Starts | Seeds |
| --- | --- | --- | --- | --- |
| C1 | D0 / B0 / P22 | Goal 1 | S0,S1,S2 | 10 |
| C2 | D3 / B0 / P22 | Goal 1 | S0,S1,S2 | 10 |
| C3 | D1 / ym70 / P23 | Goal 0 | S0,S1,S2 | 10 |

A separate 10-trial supplement uses D0/B0/P22 Goal 1, S2, Local-only and the same
seed list. These ten trials are not added to the main 90-trial denominator.
Source q, base, scene, workspace, SRDF, gripper .3 rad and clearance settings stay
fixed. The only changed main parameter is the RRT seed. Invalid endpoints produce
NOT_RUN. TIME_LIMIT, ITERATION_LIMIT, PATH_VALIDATION_FAILURE and BACKEND_ERROR
retain separate meanings. Search-budget exhaustion is not a no-path proof.

After main/supplement search, select the FIRST failed main observation within each
case, ordered by S0/S1/S2 and ascending seed. At most three representatives receive
additional 5 s and 60 s trials. Their original 20 s observation is reused, not
counted as an additional independent result. max_iterations remains 1500 and all
other search settings remain unchanged. Any iteration termination is reported;
no simultaneous step/goal-bias adjustment or unreported retry is allowed.

Fixed seeds preserve random draws. A time-limited run can terminate at a different
iteration under different host load; bit-identical wall timing is not promised.

## Standard postprocessing B

planning/path_shortcut.py is an independent standard randomized VERTEX shortcut
postprocessor. It does not change search. A single fixed seed 20261114 and a
maximum of 20 attempts apply to every accepted main/local path. A uniformly
chosen nonadjacent pair of current vertices defines a candidate direct edge.
The edge is checked through the same ConfigurationValidity at the unchanged
FINAL resolution .01 rad, including switch refinement. It can replace its
subpath only if the complete edge is valid and joint polyline length decreases
strictly by more than 1e-12 rad.

The initial 100-attempt pilot is retained under t53_c_shortcut_20261109_01.
Its first completed path required about 117 s additional postprocessing, with
82 s in candidate attempts. The run was stopped on runtime grounds, without
selecting paths by improvement. Before any formal postprocessing, shortcut_v1.json
fixed a uniform 20-attempt cap for ALL accepted main/local paths in the independent
formal _03 output. Source search settings/results remain immutable. Seed, acceptance,
edge/final precision and fresh verification are unchanged. The 100-attempt profile
remains available explicitly; pilot results are not pooled into the formal report.
The _02 serial 20-attempt resource probe is also retained, without being pooled.
Formal _03 executes all eligible paths with at most four independent process
workers on the available 20-CPU affinity set. Each owns its own model/FK/Native
session/checker. Only untimed postprocessing is parallelized: all 90 main, ten
local and budget RRT observations remain serial and unchanged. Per-job wall
latencies explicitly record pool execution and are not advertised as standalone
single-worker algorithmic speedup. Query counts and path quality remain paired.

Exact-q cache reuse remains local to one immutable collision/constraint session.
Repeated edges still call the same checker; their query IDs/cache statistics are
retained. Early stopping is allowed only when no nonadjacent vertex pair remains.
The function freshly rechecks its input path and the entire final path, separately
from cached candidate checks. An unchanged path returns NO_IMPROVEMENT with fresh
validation. A failed final candidate can only fall back to a freshly valid original;
a failed/unverified result is never playable. Raw files are never overwritten.

Successful main/local paths all enter this uniform postprocessing; failed raw
searches remain in the optimization index as NOT_RUN_NO_VALID_RAW_PATH and remain
in their original search denominators. Budget-probe paths have a different purpose
and are not silently mixed into the main postprocessing comparison.

Joint length, TCP length/detour, waypoint count, sampled unsigned self/environment
clearance and separate search/input-check/shortcut/final-check times are reported.
Cumulative total is the measured original trial time plus postprocessor time,
including its fresh checks, excluding model initialization and artifact IO.
Shorter joint paths do not imply shorter TCP paths, smoother motion, greater
clearance or dynamically executable trajectories. Clearance regressions and TCP
length increases are retained in paired statistics.

## Home audit and experiment C

Official controller/config/init_pose.yaml selects type action and action init.
init_pose.py loads /home/ubuntu/software/arm_pc/ActionGroups, whose complete init
motion is not in the supplied source tree. The inactive servo pulse block is
source evidence, not a calibrated physical Home definition. Its controller-derived
joint3 angle is approximately 120 degrees, slightly beyond URDF 2.09 rad; no
clamping, vendor-DH-to-URDF substitution or zero-as-Home assumption is made.

The existing cable_robot_control insert/pull_out YAMLs also contain a named
CABLE_HOLD_READY pulse waypoint. It is retained as a candidate source, not ignored.
Under the controller's source convention its diagnostic q is approximately
[0,-1.1100,2.0316,1.1729,0] rad. Its unregistered vendor/URDF EE numeric coordinates
differ by about 98 mm, and its recorded gripper convention is about .838 rad
instead of this task's .3 rad. This is NOT a physical FK error estimate: the
vendor/URDF reference/tool frames have not been registered. It does not establish
a calibrated model-compatible Home and is not substituted for S0.

Static audit reads source files and YAML only. It never imports or executes the
vendor controller. Without a trustworthy physical Home, all preparation results
are labelled REFERENCE_START_TO_READY, with physical_home_verified=false.
Four fixed motions (B0 and ym70, each S0 -> S1/S2) run both existing Linear and
RRT (eight planner trials), independently of operation success rates. Preparation
uses CONDITIONAL workspace, allowing entry from x<.250 into S2; Local-only is not
incorrectly imposed on the external preparation start. Ready q is exactly the
saved B start. Its goal role is READY_POSE, not an RJ45 pre-insertion IK success.

## Evidence and replay

Each trial has inputs.json with canonical SHA256, result.json, scene/task/base
snapshots, a compressed query ledger, trajectory NPZ when a path exists, an
independent Manifest and a verification receipt. All real Native queries retain
q, semantic collision/clearance, Native EE and World TCP evidence; duplicate
full-link/contact matrices are not serialized. Path reports reference canonical
query IDs. Raw waypoints, final validation samples and display samples remain
separate quantities.

path_trial_artifacts.verify_path_trial recomputes input/source identity, fixed
Start/Goal bindings, Native/KDL EE agreement, workspace decisions, interpolation
and refinement grids, clearance/quality/NPZ consistency and accepted shortcut
reconstruction. It does not start Native or search. The producer's final validation
makes fresh actual Native calls. These are discrete checks, not continuous safety.

The existing read-only RViz Catalog adapts individual raw/shortcut/preparation
directories. Only accepted paths can play. The panel identifies variant, endpoints,
RRT seed, lengths and validation status. Preparation has an additional explicitly
labelled Ready goal marker, so it is not confused with the port's pre-insertion
marker. No hardware command topics are published.

## Reproduction

Always use fresh output names. The commands below reproduce the entire matrix;
they do not overwrite existing experiments.

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_robustness_benchmark.py \
  --source-run results/multistart_benchmark/t53_b_20261109_01 \
  --expected-source-manifest-sha256 8fb6bf61298588b3d6912e5e6263c4570da243b8276fb162ae7fe5c0fb48c191 \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --allow-candidate-baseline --output-dir results/robustness_benchmark/t53_c_search_user01

search_sha=$(sha256sum results/robustness_benchmark/t53_c_search_user01/manifest.json | cut -d ' ' -f 1)
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/shortcut_robustness_paths.py \
  --search-run results/robustness_benchmark/t53_c_search_user01 \
  --expected-search-manifest-sha256 "$search_sha" \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --shortcut-config config/restricted_scenes/shortcut_v1.json --workers 4 \
  --allow-candidate-baseline --output-dir results/robustness_benchmark/t53_c_shortcut_user01

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_preparation_motion.py \
  --source-run results/multistart_benchmark/t53_b_20261109_01 \
  --expected-source-manifest-sha256 8fb6bf61298588b3d6912e5e6263c4570da243b8276fb162ae7fe5c0fb48c191 \
  --backend results/c3/assembly_build_20261104/build/collision_backend \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source \
  --allow-candidate-baseline --output-dir results/robustness_benchmark/t53_c_preparation_user01

MPLCONFIGDIR=/tmp/t53_c_matplotlib PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B repro/scripts/report_robustness.py \
  --search-run results/robustness_benchmark/t53_c_search_user01 \
  --shortcut-run results/robustness_benchmark/t53_c_shortcut_user01 \
  --preparation-run results/robustness_benchmark/t53_c_preparation_user01 \
  --output-dir results/robustness_benchmark/t53_c_analysis_user01
```

Representative replay (change the directory to compare raw/shortcut/preparation):

```bash
colcon --log-base results/rviz_replay/build_t53_c_user/log build \
  --base-paths ros2/jetrover_idc_visualization \
  --build-base results/rviz_replay/build_t53_c_user/build \
  --install-base results/rviz_replay/build_t53_c_user/install --symlink-install
source results/rviz_replay/build_t53_c_user/install/setup.bash
condition="$PWD/results/robustness_benchmark/t53_c_shortcut_user01/main_C1_S0_seed20261104"
condition_sha=$(sha256sum "$condition/manifest.json" | cut -d ' ' -f 1)
ros2 launch jetrover_idc_visualization replay.launch.py \
  workspace_root:="$PWD" benchmark_dir:="$condition" \
  expected_manifest_sha256:="$condition_sha" domain_id:=83
```

Select P22 / q1 / RRT-Connect / Path for this example. Ready goals in preparation
have their own role and labels. There is no new Approach result in this experiment.

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider \
  -m 'core or integration or artifact' tests \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend \
  --artifact-root "$PWD/results" \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source \
  --replay-install "$PWD/results/rviz_replay/build_t53_c_user/install"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B repro/scripts/doctor.py --mode full \
  --backend results/c3/assembly_build_20261104/build/collision_backend
```

## Interpretation boundaries

The same 29 unverified candidate SRDF exemptions remain RESEARCH_ONLY. The virtual
TCP is uncalibrated and has no dedicated RJ45 collision solid. Local-only has no
upper X bound; passing it does not prove the physical connector stays in front
of the panel. No actual Home, controller execution, cable forces, continuous
collision proof, velocity/acceleration limits or time parameterization is added.
Changing the complete Ready q changes both posture and tool orientation, not just
TCP X. Ten fixed seeds support this controlled comparison, not universal optimality
or hardware safety. Final results, tests and visual evidence are appended below.

## Observed research results — 2026-11-09

The completed raw search is `results/robustness_benchmark/t53_c_search_20261109_01`.
Formal postprocessing is `_shortcut_20261109_03`; `_01` and `_02` are preserved
resource pilots. Preparation and read-only analysis use `_preparation_20261109_01`
and `_analysis_20261109_01`. None overwrites a B/A experiment.

| Case | S0 accepted / 10 | S1 accepted / 10 | S2 accepted / 10 |
| --- | --- | --- | --- |
| C1 D0/B0/P22 Goal 1 | 8 | 10 | 10 |
| C2 D3/B0/P22 Goal 1 | 8 | 10 | 10 |
| C3 D1/ym70/P23 Goal 0 | 4 | 10 | 10 |

All ten rejected main observations terminated at TIME_LIMIT. The additional
S2 Local-only group accepted 10/10. These are controlled counts over ten declared
seeds, not estimates of universal reliability. S1/S2 preserved success across
this set; full Ready q changes posture/orientation as well as TCP location.

Observed search-time medians across all scheduled seeds (including censored
timeouts) and accepted-path lengths are:

| Case / Start | Observed search median [s] | Accepted joint-length median [rad] | Accepted TCP-length median [m] |
| --- | --- | --- | --- |
| C1 / S0 | 16.915 | 5.145 | .5956 |
| C1 / S1 | 10.984 | 4.192 | .2272 |
| C1 / S2 | 11.076 | 4.200 | .3417 |
| C2 / S0 | 17.142 | 5.145 | .5956 |
| C2 / S1 | 11.266 | 4.192 | .2272 |
| C2 / S2 | 11.255 | 4.200 | .3417 |
| C3 / S0 | 20.008 | 2.981 | .5145 |
| C3 / S1 | 6.112 | 2.006 | .1045 |
| C3 / S2 | 2.713 | 1.724 | .0993 |

Successful lengths have different denominators (notably C3/S0 has only four).
`robustness_summary.json` includes n, quartiles, extrema and standard deviation;
`summary.csv` retains all individual observations and real query counts.

The first failed S0 seed is 20261108 for C1/C2 and 20261104 for C3. At 5 s all
three representatives timed out; their original 20 s results also timed out.
With only the time budget raised to 60 s, all three succeeded, searching for
21.613, 21.465 and 20.885 s respectively. max_iterations stayed 1500. This is
evidence of budget-sensitive difficulty for these three cases, not proof that
all other timeouts recover or that search found an optimal route.

All 80 accepted main paths and ten accepted Local-only paths received the same
formal 20-attempt profile. All 90 optimized paths passed fresh FCL/workspace
checks and independent artifact reconstruction. The failed raw ten remain in
the 90-main denominator and are NOT_RUN for shortcutting.

- Median paired joint-length reduction: 7.399% (range .510%–29.469%).
- Median paired TCP-length reduction: 27.460%; none increased in this dataset.
- Median vertex count: 18 raw, 3 optimized. Validation samples are a separate count.
- Environment minimum sampled separation decreased on 31/90 paths; every such
  path still met the unchanged configured validity rules. This is not a clearance
  objective, safety-margin improvement, dynamic smoothing or physical safety proof.
- Median additional complete postprocessing time: 82.807 s (range 21.787–130.832 s),
  measured under four independent workers. It includes fresh input/final checks;
  this is **not** a serial latency comparison or a claim that optimization is cheap.

All eight preparation trials succeeded: B0/ym70 × S1/S2 × Linear/RRT. The Linear
S0→S1 paths had joint length 3.012 rad and TCP length .2409 m; S0→S2 had 1.568 rad
and .2688 m. The corresponding RRT paths had 3.364 rad/.3556 m and 1.976 rad/.3833 m.
They are strictly REFERENCE_START_TO_READY, not verified physical Home motions.
Preparation success and operation success have independent denominators. Two
independently validated legs can motivate an explicit staged research baseline,
but this experiment does not count their unexecuted concatenation as a new trial.

The measured bottlenecks are long S0 search, unnecessary raw waypoints and costly
repeated Native checks during postprocessing. Further work should first compare
explicit Ready-stage routing and validation/query cost under matched budgets;
shortcutting alone does not eliminate the need for a better-defined physical
Home, tool geometry, calibrated frames or execution constraints.

For a read-only independent check of one saved optimized result:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B - <<'PY'
from pathlib import Path
import sys
sys.path.insert(0, str(Path('src').resolve()))
from workspace_analysis.robot.model import load_model
from workspace_analysis.experiments.path_trial_artifacts import verify_path_trial
print(verify_path_trial(
    Path('results/robustness_benchmark/t53_c_shortcut_20261109_03/main_C1_S0_seed20261104'),
    load_model('config/workspace.yaml')))
PY

# Run the existing Native test without changing the historical build's test logs.
native_test_dir=$(mktemp -d /tmp/t53_c_ctest_user.XXXXXXXX)
cp results/c3/assembly_build_20261104/build/CTestTestfile.cmake "$native_test_dir/CTestTestfile.cmake"
ctest --test-dir "$native_test_dir" --output-on-failure
```

## Completed engineering validation and actual visual evidence

The new complete run finished with **841 passed, 1 warning, 0 skipped** in
946.11 s: Core 568, Integration 194, Artifact 79. The single warning is the
existing parser/private-variable Python 3.11 compatibility warning. No test
assertion or numerical threshold was relaxed. `native_collision` CTest passed
1/1; the fresh Doctor returned READY_FOR_RESEARCH_REPRODUCTION after 37 checks.
The replay package was built into a new isolated `build_t53_c_20261109` directory.

Validation files are under
`results/robustness_benchmark/validation_t53_c_20261109/`:

- `full_regression.log`, `junit.xml`, `native_ctest.log`, `doctor.json/log` and
  `replay_build.log` hold the actual new execution results.
- `optimized_local_ros_probe/report.json` and its process log retain the actual
  DDS/JointState/RSP/TF test. All 335 original optimized Local-only samples were
  observed, including the exact S2 start; minimum World TCP X was
  .2604195673809405 m. Maximum EE TF error was 1.6653345369377348e-16, all 38
  World scene markers matched, and gripper/mimic transforms were checked against
  a real Native query. Cleanup was normal, with no residual owned nodes.
- `protected_before.json` / `protected_after.json` compare 374 protected historical
  data and original critical source/config files. No protected content changed.
- `all_proposed_changes.diff`, `proposed_source_snapshot.tar.gz` and
  `source_snapshot.json` include both tracked modifications and new source files;
  pre-existing docs/develop and results.zip are excluded and left untouched.
- `final_validation.json` and `validation_manifest.json` bind these checks and
  the separate experiment/capture manifests. No Stage/Commit/Push was performed.

Actual visual evidence is in
`results/rviz_replay/validation_t53_c_20261109_native_capture/`:

| Directory | Actual outcome |
| --- | --- |
| raw_C1_S0 | Raw path played through 423/423 samples |
| shortcut_C1_S0 | Optimized path played through 407/407 samples |
| shortcut_S2_local | Optimized Local-only path played through 335/335 samples |
| reference_b0_to_S1 | Reference preparation path played to S1 |
| reference_ym70_to_S2 | Reference preparation path played to S2 |
| C3_budget_60 | Successful larger-budget result played to Goal 0 |
| C3_budget_20_rejected | Timeout remains STOPPED, 0/0 samples, Play disabled |

Each directory contains `rviz_actual.png`, `qt_panel_actual.png`, `evidence.json`,
`launch.log` and `native_render.log`. These are actual native RViz render captures
and production QWidget captures, not synthetic pictures. All seven owned launch
processes exited 0 without forced cleanup. The evidence binds the exact selected
Manifest, port, goal and final q. Formal user visual acceptance remains PENDING.
The XY/XZ plots in the analysis directory make raw/shortcut geometric differences
easier to compare than the identical final robot poses in static screenshots.

Replay an existing validated result without repeating search or shortcutting:

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
source results/rviz_replay/build_t53_c_20261109/install/setup.bash
condition="$PWD/results/robustness_benchmark/t53_c_shortcut_20261109_03/local_LOCAL_S2_seed20261104"
condition_sha=$(sha256sum "$condition/manifest.json" | cut -d ' ' -f 1)
ros2 launch jetrover_idc_visualization replay.launch.py \
  workspace_root:="$PWD" benchmark_dir:="$condition" \
  expected_manifest_sha256:="$condition_sha" domain_id:=83
```

Select P22 / q1 / RRT-Connect / Path, then Play. For the exact programmatic capture
command used for any screenshot, inspect its `evidence.json` and the corresponding
`capture_*.log` in the validation directory. Model-level path validity and physical
hardware validity remain separate; the latter is NOT verified. Work stops at
T5.3-C, pending user review; no T6 or new planning algorithm was started.
