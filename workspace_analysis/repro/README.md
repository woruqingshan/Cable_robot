# R2-A/B — Repository reproducibility foundation

Scope: test isolation, dependency declarations and read-only environment/model
diagnostics. No archive upload/download, full experiment rerunner or T5 work.
Git baseline: `0ae1655b225ea44d01c0364b858fd02abcca217c`. The real Git root is
`cable_vla_robot/`, with this directory under `workspace_analysis/repro/`.

## Audit and test contracts

The baseline is 641 passing cases / 305 test functions across 25 modules.
`results/` is ignored and `git ls-files results` returns zero: the commit title
alone does not establish that historical artifacts were uploaded. History
dependencies extend beyond Viewer HTML to G1 evidence, G2 pairs, G4 bundles,
collision probes and T2–T4 verification. Official description files live in an
external `jetrover_description` ROS package, not inside this Git root.

Every test function is declared in `tests/dependencies.json`; undeclared new
tests fail collection. Its entries identify an exclusive level, official-model /
Native / vendor-source requirements, and required relative artifact paths.
`tests/conftest.py` applies the markers before selection and performs preflight
checks only for selected tests. Missing resources raise `BLOCKED` with exit 4,
never a successful skip. Required Plotly coverage no longer uses `importorskip`.

| Level | Inputs / purpose | Current result |
| --- | --- | --- |
| `core` (default) | Source, checked-in small fixtures and declared Python/JS libraries; no history, official description or real Native process | 418 passed |
| `integration` | Official URDF/package index and/or explicit Native binary; synthetic/small generated test data, no history | 179 passed, 1 warning |
| `artifact` | Explicit saved experiment inputs; some cases additionally require model, Native or vendor sources | 54 passed, 1 warning |

The sum is 651 cases. All 641 original cases are retained; ten new cases cover
doctor/identity failures, synthetic T3 compatibility, missing/LFS-pointer inputs
and an external Viewer artifact root. The original 305 test-function ASTs,
including decorators/assertions, are unchanged after normalizing only the
declared artifact/vendor path substitutions. Historical tests were selected or
moved, not deleted or weakened. Warnings are the existing Xacro warning and may
occur once per separately launched group.

`test_viewer_replay.py` now contains only generated synthetic replay tests.
The original real q0/q1 26-frame and T3 key-frame tests are preserved in
`tests/integration/test_saved_viewer_artifacts.py`. Marker level, not directory
name, defines their artifact dependency. Both levels retain actual JavaScript
execution through the installed JavaScriptCoreGTK runtime with controlled
DOM/Plotly/timer doubles; this is not browser/WebGL acceptance.

Core/non-artifact execution has an audit hook rejecting implicit historical
reads; Core also rejects official-description reads. Artifact reads use one
declared `WORKSPACE_ARTIFACT_ROOT` / `--artifact-root`. The Native executable is
allowed as an explicitly selected executable even if currently located inside
an old results build. It is not treated as restored experiment data.

## Commands

Use the existing environment; nothing below installs dependencies automatically:

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis

# Default is also Core; explicit form is shown for clarity.
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest \
  -q -rs -p no:cacheprovider -m core tests

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest \
  -q -rs -p no:cacheprovider -m integration tests \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest \
  -q -rs -p no:cacheprovider -m artifact tests \
  --artifact-root "$PWD/results" \
  --native-backend results/c3/assembly_build_20261104/build/collision_backend \
  --system-source /home/siyuen/Desktop/code/cable_robot/system_source
```

To select all groups explicitly, use `-m 'core or integration or artifact'` with
the same required resource options. A normal `pytest tests` is deliberately
not a claim that all historical integration cases were executed.

The current Native path above is an example, not a diagnostic default. For a
new checkout, build the unchanged C++ sources in a separate directory:

```bash
R2_NATIVE_BUILD="$(mktemp -d /tmp/jetrover-r2-native.XXXXXX)"
cmake -S cpp/collision_backend -B "$R2_NATIVE_BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$R2_NATIVE_BUILD" -j2
ctest --test-dir "$R2_NATIVE_BUILD" --output-on-failure
export C1_BACKEND="$R2_NATIVE_BUILD/collision_backend"
```

Do not recreate the robot workspace or change the official description/config.
Source an existing overlay containing the official package. In a different
filesystem layout, supply `--system-source` to vendor-source integration tests.

## Clean-checkout simulation

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B repro/scripts/check_clean_checkout.py
```

This copies Git-tracked workspace sources plus current R2 test/infrastructure
changes into a new temporary tree, omits results/venv/build outputs, uses the
existing dependency interpreter, removes Native/model overlay discovery and
legacy input overrides, guards original-history reads and runs Core. It does
not hardlink original test/data files. The temporary tree and policy are removed
on completion. Optional `--json-output NEW_PATH` records copied source hashes,
logs, exit status and isolation facts. Current isolated result: 418 passed;
no results directory was copied or created.

## Environment Doctor

```bash
.venv/bin/python -B repro/scripts/doctor.py --mode core
.venv/bin/python -B repro/scripts/doctor.py --mode full \
  --backend results/c3/assembly_build_20261104/build/collision_backend

.venv/bin/python -B repro/scripts/doctor.py --mode full --backend "$C1_BACKEND" \
  --json --json-output /tmp/r2_doctor_new_report.json
```

Default execution is read-only and disables bytecode publication. Explicit
`--json-output` creates only a new file; existing output is refused. No software
installation, system modification, hardware node, motion or sampling is run.
Full Mode starts one owned Native process to verify initialization/identities
and normal shutdown, with **zero configuration queries**.

| Check status | Meaning |
| --- | --- |
| PASS | Check executed and declared condition met |
| FAIL | Executed check found a mismatch or incompatible source/environment |
| BLOCKED | Dependency/resource missing; check cannot complete |
| NOT_CHECKED | Full-only check not requested by Core Mode |

Process exit: 0 ready; 2 failure; 3 blocked (failure takes precedence).
Core returns `READY_FOR_CORE_TESTS`, Full returns
`READY_FOR_RESEARCH_REPRODUCTION`; neither means physical safety or artifact
availability. Example JSON shape:

```json
{"schema_version":1,"mode":"core","status":"READY_FOR_CORE_TESTS",
 "exit_code":0,"usage":"RESEARCH_ONLY","hardware_validated":false,
 "historical_artifacts_checked":false,
 "checks":[{"id":"robot.identity","status":"NOT_CHECKED",
            "summary":"Full mode only","details":{}}]}
```

The real report contains every check and observed value, not only this example.
Observed Core and Full environments pass. Missing-model and missing-backend CLI
probes return BLOCKED/3, and a copied mismatched identity manifest returns FAIL/2.
Negative unit tests use mocks/temporary fixtures, not damaged installations.

## Model and environment identity

`environment/model_assets.json` is the observed content baseline, not a model
copy. The doctor uses existing `robot/model.py`, with base `base_link`, EE
`end_effector_link`, joint1–joint5 and explicit Mecanum/A1 config. It hashes all
12 actual Xacro/includes and all referenced visual/collision Mesh bytes. A
canonical full-tree XML identity replaces only resource filenames by content
digests, retaining origins/scales/limits/mimic/kinematic structure. Raw URDF,
source paths and include hashes are reported independently. Relocated but
identical resource content passes; changed Mesh/include content fails.

Strict/Candidate SRDF bindings are separately verified by existing APIs. If raw
URDF path serialization changes but portable geometry matches, the geometry
check can pass while the existing raw-bound SRDF rejects the model. Doctor
reports both facts; it never rewrites or bypasses the historical binding.

Environment snapshots are split by actual provenance:

- `python_requirements.txt`: observed venv-provisioned packages only; not a
  command to replace APT/ROS modules with pip builds.
- `observed_environment.json`: import versions/origins and observed compiler,
  CMake and APT inventory. Null versions are explicitly UNRESOLVED.
- `ros_packages.txt`: actual `dpkg-query` package versions and unresolved errors.
- `native_backend.json`: observed executable hash and verified initialization;
  a new build's byte identity is recorded, not assumed portable.

Current environment uses Python 3.10.12/system-site-packages, APT NumPy 1.21.5,
SciPy 1.8.0, PyKDL 1.5.1 and PyYAML 5.4.1; Plotly 5.24.1 is in the venv.
URDF parser lacks a discoverable Python distribution version, so its module
version remains UNRESOLVED while its installed ROS/APT version is recorded.
Do not interpret the snapshot as a complete automatic installer or pip freeze.

## R2-C/D boundary and remaining limitations

No data archive/recovery or full experiment rerun is implemented here. Future
restoration should materialize actual files under an explicit artifact root,
verify stored hashes and invoke the artifact test group. Missing files and Git
LFS pointers are BLOCKED with exit 4. External-root saved Viewer tests already
pass after copying just the three required HTML files to an independent tree.

Some legacy T2/T3 manifests embed absolute upstream paths. Complete saved-run
tests report `BLOCKED / ARTIFACT_PATH_BINDING` when those paths leave the declared
root; they cannot silently read another checkout. Compatible original-layout
restoration works now. A future explicit relocation layer must preserve original
bytes/hashes and declare path mapping; this is R2-C/D interface work, not a
reason to alter historical manifests or weaken checks in R2-A/B.

Future orchestration can combine doctor exit status, explicit resource paths and
test selection. Doctor does not claim archives are present; Artifact preflight
does not claim a research collision model is hardware validated. The candidate
SRDF remains RESEARCH_ONLY, and physical model validation is unchanged.
