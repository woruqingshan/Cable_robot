# C3 validation and reproducible commands

Run from the workspace directory after sourcing ROS. Every generation command
requires a new output path. Old C1–D2 files and official models are read-only.

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis

cmake -S cpp/collision_backend -B results/c3/build_user_review/build -DCMAKE_BUILD_TYPE=Release
cmake --build results/c3/build_user_review/build -j2
ctest --test-dir results/c3/build_user_review/build --output-on-failure
export C1_BACKEND="$PWD/results/c3/build_user_review/build/collision_backend"

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests
```

The original baseline was479 passed,1 warning,0 skipped. Native1/1 passes with
new SRDF fixtures in addition to all old native checks. The final Python count
and logs are recorded in the run validation evidence, not assumed from code.
Existing assertions are unchanged. New tests exercise XML mismatches, groups,
limits/named states, invalid hashes, actual ACM acknowledgement, empty Raw vs
semantic FCL, gripper external positives, state restoration, candidates without
ACM mutation, sample8/19/73 at multiple gripper states and full mesh viewer data.

Audit + reference probes + Raw/historical/SRDF100/1,000 comparison + viewers:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/revalidate_semantic_workspace.py \
  --backend "$C1_BACKEND" \
  --output-dir results/c3/user_review_01 \
  --input-prefix results/workspace_n100_seed42 \
  --input-prefix results/g4_run01/experiments/workspace_n1000_seed42

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/revalidate_semantic_workspace.py \
  --verify-run results/c3/user_review_01

xdg-open results/c3/user_review_01/figures/19.html
xdg-open results/c3/user_review_01/figures/8.html
xdg-open results/c3/user_review_01/figures/73.html
xdg-open results/c3/user_review_01/figures/diagnostic_zero.html
xdg-open results/c3/user_review_01/figures/n100_workspace.html
xdg-open results/c3/user_review_01/figures/n1000_workspace.html
```

For SRDF audit/reference probes/viewers only, omit both --input-prefix arguments.
This still records true collision positives and a diagnostic zero query; it does
not invent a normal reference. --gripper is in radians, default0.3.
--candidate-trials0 uses a thread-aware standard generator trial count. A smaller
nonzero count is rejected before the known installed MoveIt integer-divide bug.

`manifest.json` binds HEAD, source SHA256, URDF/SRDF/ACM/scene/backend identities,
clearance, gripper, all study input hashes and output hashes. `verify-run` checks
output bytes and the existing strong G2 reader, then requires exact array equality
for all saved q/FK arrays and original indices. No input is resampled or filtered.

Each n100/n1000 folder holds evaluations.json, comparison.csv/json,
observed_pair_candidates.json, summary.json and workspace_results.npz.
The three actual native conditions are A_raw (baseline empty policy),
B_historical_v1 and C_semantic (no policy overlay). Native queries independently
compute the new matrix. B and C are expected identical because the SRDF contains
exactly the same five reviewed fixed pairs. This equivalence validates loading;
it does not solve the unresolved moving assemblies.

Required checks: sample19 black_shell/link3 remains in semantic blocking pairs;
sample8/73 retain environment collisions. Analytic FCL fixtures separately test
nonadjacent/gripper-base, gripper-arm and gripper-environment positives, plus
semantic-clear/raw-colliding synthetic internal assembly. Synthetic normal
fixtures are not JetRover hardware truth.

Normal reference: PENDING. No approved compatible vendor state/recording is
available. Therefore all100/1,000 results are DIAGNOSTIC_ONLY, not formal new
workspace validation. The CLI refuses datasets larger than1,000 before creating
an output directory. Do not run10,000 until a separately reviewed normal
reference passes, true positives remain rejected and all changes are explained.
There is deliberately no flag to bypass that unresolved gate.

Final acceptance includes opening HTML, selecting Semantic blocking/SRDF disabled,
checking all24 meshes and38 S1 obstacles, identical base/scene poses and visibly
different sample8/73 robot states. Automated data checks do not substitute for
this visual review or for hardware model validation.

## Actual run, 2026-11-04

Completed run: `results/c3/semantic_review_20261104_02/`.
Baseline479 passed,1 warning,0 skipped. Final506 passed,1 warning,0 skipped;
Native CTest1/1 PASS. No existing Python test assertion was modified.
Evidence/logs and historical-file integrity audit are in
`results/c3/validation_20261104_01/`.

| Input | Raw self | Historical effective self | SRDF semantic self | Environment | SRDF valid | Errors/unknown |
|---|---:|---:|---:|---:|---:|---:|
|100|100|100|100|2|0|0|
|1,000|1,000|1,000|1,000|21|0|0|

Both valid ratios are0%; historical→semantic changed indices: none.
Moving blockers green/link1 and link2/servo_link1 occur100/100 and1,000/1,000.
Link2/link3 occurs92/100 and897/1,000. All seven gripper focus pairs remain
checked. The independent semantic query reproduces the same reviewed matrix;
there is no evidence for adding further permissions.

For samples8/19/73 and diagnostic zero, the first64 returned contacts per pair
show green/link1 radial distances14.46–31.53mm from joint1, axial≈0–1.501mm;
link2/servo_link1 radial1.377–7.639mm from joint2, axial19.422–26.014mm.
These are **truncated finite contact subsets**, not full surface bounds. Both
pairs include moving joints. Their near-axis location suggests assembly contact
but does not prove whole-pair safety. They stay checked; evidence review PARTIAL.

Sample19 black_shell/link3 remains blocked. Sample8/73 both retain robot/IDC
collision. All24 collision meshes and38 scene objects are represented; sample8
and73 link3 transforms differ while base_link/scene stay equal.
`visualization_checks.json` records these checks. `robot_comparison.png` is a
static geometry preview; HTML interaction remains pending user review.

Standard candidate generation succeeded with2,000 trials, no applied permissions,
normal exit0. Its uncontrolled random recommendations are archived verbatim;
the100/1,000 observed candidate tables are deterministic from existing inputs.
The first failed attempt remains under semantic_review_20261104_01 and GDB
register/instruction logs identify the installed-library divisor problem.

Manifest identity for this run:

- URDF: `5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`
- Geometry: `4858b34b1ff62f8af791809701888028637650c2372dc5f3978a1d5e606b216d`
- SRDF: `5bf9ddcf8167da5f5e448b9f8f84ca7cd1353b161a45d96eece87e014fac5df7`
- Semantic ACM: `58be16b9ec5dd4b5d46e39be64c362ba3b34bb5d7371017c79b1ade6c468fc11`
- Scene: `d83e76d09f7e69fa0c01e753f0cefb3128dd520fb0292f1fd0b4838f66a5ab3b`
- Backend: `34cbcdedc89e60db72042651ebfc3c4c4e0ff45a189f7a0ec94d5add8b24ce9f`
- HEAD: `7f32d3cfde48dec46dc2a0740334f00e06d541d5` plus hashed uncommitted C3 sources.

No10,000 C3 workspace evaluation was performed. Software is implemented and
regression-tested. Semantic physical review remains PARTIAL, known-normal
reference PENDING, hardware NOT PERFORMED, research readiness DIAGNOSTIC_ONLY.
