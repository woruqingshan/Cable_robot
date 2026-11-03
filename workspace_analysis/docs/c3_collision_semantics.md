# C3 collision semantics reconstruction

Status: software implemented; semantic model PARTIAL; normal reference PENDING;
hardware validation NOT PERFORMED. No new active-joint exemptions were inferred.

## Sources and applicability

The active robot is `jetrover`, official Mecanum/A1 expanded URDF SHA256
`5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`.
Official resources remain unchanged. The full 24 meshes / 21,568 triangles and
all 38 S1 obstacles are retained. No C3 mesh cutting or task-scope additions.

No matching SRDF was found in system_source, jetrover_sim_ws (including installed
symlinks), or `/opt/ros/humble/share`. Broader local search found:

- `/home/siyuen/Desktop/code/ros2_ws/src/hiwonder_moveit_config/config/rosmentor.srdf`
- `/home/siyuen/Desktop/ros2_ws/src/winterhack/config/rosmentor.srdf`

These files name robot `rosmentor`; hiwonder package.xml references
`landerpi_description`. They have a different gripper (left/right jaw,
gripper_base/linkage joints) and `back_shell_middle_link`. Exact missing names,
groups, named states, every disabled pair, paths and SHA256 are in
`srdf_source_audit.json`. They were REJECTED as incompatible, not silently renamed.
Their Home=zero/P1/open/close states are not trusted JetRover references.
Files in Trash were not restored or used as active model sources.

The official controller `init_pose.yaml` uses `type: action`, action `init`, whose
implementation loads `/home/ubuntu/software/arm_pc/ActionGroups`. That active path
is missing here. The alternate `servo` block is not the configured default;
controller pulse conversion gives joint3 pulse0 = +2.094395102 rad, outside
current URDF upper2.09. It must not be silently clamped into a safe reference.
Zero is queried only as a diagnostic default. Normal pose validation is PENDING.

## Semantic configuration

`config/collision/semantic/jetrover_semantic_v0.srdf` is a project-derived,
conservatively reviewed subset, NOT an adopted vendor SRDF. Its JSON manifest
binds URDF/SRDF and one evidence record per disabled pair. Current review scope
is `model_only_partial`. It independently declares the five existing fixed
assembly decisions, with no automatic historical-policy union:

| Disabled pair | Evidence scope |
|---|---|
| black shell / green shell | invariant fixed assembly; prior mesh/contact audit |
| green shell / base_link | invariant fixed assembly; C2.5/C2.6 |
| gripper_link / link5 | fixed mount, model-only |
| link1 / servo_link1 | fixed mount, model-only |
| link4 / servo_link2 | fixed mount, model-only |

Full names and historical evidence references are in semantic_manifest.json.
The two planning groups declare joint1..5 and independent r_joint. No fabricated
Home/Ready state is installed. FK, limits, all fixed transforms and mimic semantics
continue to be the official URDF's.

`back_shell_green_link/link1`, `link2/servo_link1`, `link2/link3`, and seven gripper
mechanism focus pairs remain CHECKED. The first two have moving joint paths and
persistent sampled contacts near assembly regions, but finite contacts do not
rule out other whole-pair interference. `link2/link3` is known state-dependent.
The gripper's physical closed-loop/linkage calibration remains unverified.
`collision_pair_review.json` contains paths and local-axis contact evidence;
`joint_gripper_contact_audit.json` holds 5-angle sweeps for joint1/2/3 and r_joint,
including per-query cap/complete-surface flags. No frequency is a permission.

## Native and Python implementation

Native INIT optionally accepts `semantic` with XML, SRDF/model SHA256, canonical
review JSON/hash and expected disabled pairs. XML validates name, group members,
chains, subgroup cycles, scalar named states and explicit pair names. Unsupported
SRDF elements fail explicitly; end_effectors/virtual/passive joints require future
reviewed support. This is an expanded SRDF loader, not an SRDF-Xacro expander.

MoveIt RobotModel/PlanningScene initialize from the supplied SRDF. All actual
ALWAYS entries are read back over the complete link set and compared with the
reviewed declaration. Python verifies this acknowledgement and independent ACM
SHA256 (canonical sorted array of sorted pairs), SRDF SHA256 and review SHA256.
Default entries are not used. No environment object can be a disabled robot pair.

Three matrices remain separate:

1. Raw: owned empty ACM for self AND environment queries.
2. Historical: explicit `policy_acm_`, unchanged legacy policy operation.
3. Semantic: actual MoveIt initialized SRDF ACM, independent contacts query and
   fresh contacts=false boolean query. Semantic distances use that same matrix.

Semantic sessions refuse POLICY overlays, Task Scope and geometry profiles.
There is no implicit union or geometrical replacement. Legacy INIT without
semantic still uses empty SRDF/empty base ACM and keeps C1–D2 behavior.

```python
from workspace_analysis.collision.semantic import SemanticConfig
from workspace_analysis.collision.evaluator import Evaluator
semantic = SemanticConfig.load(srdf_path, model)
with Evaluator(model, scene, None, backend, semantic=semantic, gripper=0.3) as e:
    rows = e.evaluate(q)  # existing q[N,5], radians
```

New fields: semantic_self_collision, raw_collision_pairs,
semantic_blocking_pairs, disabled_collision_pairs, semantic_clearance,
model_configuration_valid, semantic_decision, hardware_validated=false.

Validity requires limits, no semantic self/environment collision and available
requested clearance. The validity result is model-only and PARTIAL. Historical
`effective_self_collision`, `model_scope_collision_free`, `final_decision` and
`clearance` retain historical meaning; in a semantic-only session no legacy
POLICY is installed, so legacy effective fields use empty ACM. Consumers must
use the NEW semantic fields. Native identity has distinct semantic_acm_sha256
and semantic_review_sha256; legacy policy_sha256 is not an SRDF overlay hash.

Shared CollisionError moved to collision/errors.py to keep protocol/config
imports acyclic; protocol still reexports the same public exception.

## Candidate generation, not approval

The installed standard MoveIt Setup Assistant `computeDefaultCollisions` API
is used on a detached scene with empty ACM. It reports Adjacent/Default/Always/
Never/Not Disabled. All results remain UNREVIEWED, applied=false. Always uses
the standard95% threshold; Never is finite sampling, not proof. Project code
also saves deterministic classification flags over unchanged input samples,
including Remaining_Checked, independently of standard random recommendations.
Neither generator writes an SRDF or policy.

Installed Humble2.5.9 has a SIGFPE for undersized parallel trials. GDB confirmed
`divl 0x38(%rsp)` with zero divisor in libmoveit_setup_srdf_plugins.so, with
100 total trials and20 hardware threads. New calls require at least100 trials
per hardware thread (0 selects this count), upper bound100,000. On this host
0 selects2,000 **candidate diagnostic trials**, not a new workspace dataset.
Random seed is not controlled by this library API; the exact candidate output
is persisted and is not safety evidence. Candidates run in a separate child
session; errors/exit codes are recorded without blocking independent diagnostics.

No system dependency was installed or changed. Native now explicitly locates
already-installed TinyXML2/vendor and MoveIt setup framework/SRDF libraries.
This adds installed setup-tool build dependencies; no GUI or controller starts.

## Artifacts and limits

`results/c3/semantic_review_20261104_02/` is the completed diagnostic comparison.
`semantic_review_20261104_01/` retains the failed generator attempt and evidence.
`results/c3/build_c3_01/build/` contains the independent new Native build.
No historical C1–D2 files are replaced. All output directories must be new;
HTML export uses the established no-overwrite writer.

Viewers reuse C2.5 full robot and S1 solids, equal coordinates, actual Native
transforms and actual raw FCL contact positions. The Semantic filter highlights
blocking pairs; SRDF disabled selector also allows inspecting declared pairs
without an observed contact (explicitly labelled). No contact positions are
invented. Meta includes q/index/model/scene/SRDF/ACM identities. Zero viewer is
labelled NOT_SAFE_REFERENCE. Browser interaction remains user review work.

LiDAR collision coverage, RJ45 tool/TCP, cable, gripper calibration and physical
tolerances remain missing. A software SRDF loader does not validate those facts.
No claim of physical safety or positive workspace readiness is made.

## Changed files

New:

- collision/errors.py — shared exception, protocol keeps public reexport.
- collision/semantic.py — strict SRDF subset, review binding, identity checks and
  deterministic observed candidate classes.
- experiments/semantic_workspace.py — source/pose/pair audits, Evaluator comparison,
  original-array integrity verification and existing Viewer orchestration.
- scripts/revalidate_semantic_workspace.py — CLI generation and verification.
- config/collision/semantic/jetrover_semantic_v0.srdf, semantic_manifest.json —
  explicit project fixed-assembly semantics and evidence.
- cpp/collision_backend/src/candidates.cpp — installed standard MoveIt candidate
  API on detached scene with bounded/thread-aware trials.
- tests/test_semantic.py —27 new parameterized/regression cases.
- docs/c3_collision_semantics.md, docs/c3_semantic_validation.md — design/evidence/commands.

Modified:

- cpp/collision_backend/CMakeLists.txt, include/engine.hpp, src/engine.cpp,
  src/main.cpp — dependencies, optional semantic protocol, loaded ACM readback,
  separate Raw matrix, independent semantic FCL/clearance, candidate operation.
- cpp/collision_backend/tests/native_tests.cpp — added independent semantic/positive/
  invalid identity/restoration fixtures; prior checks retained.
- collision/client.py, protocol.py, evaluator.py — optional semantic API, validated
  acknowledgements/results and additional model-only validity fields.
- viz/collision.py — semantic metadata/filtering on the existing full Mesh viewer.
- README.md — links to C3 design and validation commands.

All Python paths above are under src/workspace_analysis unless prefixed otherwise.
Historical geometry-profile and task-scope modules/configuration remain available,
unchanged, and cannot be implicitly combined with the new semantic mode.
