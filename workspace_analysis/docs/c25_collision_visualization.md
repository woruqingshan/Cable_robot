# C2.5 Robot Collision Visualization & Geometry Audit

Visualization Implementation Status: IMPLEMENTED, Pending User Review.
Collision Geometry Validation Status: software transform/resource consistency VERIFIED; physical geometry/policy PARTIAL.

## Existing Viewer root cause (actual artifact audit)

Original C2 run 20261103T110913Z_diagnostic_dcddb9f7/sample_8.html has 188 traces, including one
Scatter3d named "Link origins ONLY (not collision surfaces)" with 29 points. sample_73.html has 187 traces,
no such trace. Neither has robot collision Mesh traces. Both queries.json rows have all 29 native transforms.
Thus 73 is missing a rendering trace, not missing RobotState data. Its historical separate generation path omitted
that addition. The current audit_collision.py single-sample path does append origins, so the old 73 artifact is not
an equivalent output of that common path. No evidence supports camera clipping or FK failure as the cause.
The marker has no explicit color; green is Plotly's automatic trace color. It is not G2 workspace data.

The old origin trace does use Native translations, but discards rotation, collision origin, scale and surfaces.
Changing its visibility cannot solve the lack of collision geometry. Old HTML and C1/C2 outputs are preserved.

## Implementation

New collision/assets.py reads exact binary STL triangles from the same URDF as G1/Native, resolves local resources
with the existing resolver, verifies SHA256/scale and Native loaded collision origin, then applies:

    p_base = T_base_link (Native RobotState) @ T_link_collision (URDF origin) @ Scale @ p_mesh

No Python FK implementation is introduced. Fixed links, arm, gripper and mimic use the same native full-tree
transforms already validated in C1. All 24 collision Mesh, 21,568 triangles are drawn, with no hull/sampling/EE approximation.
Binary STL triangle vertices are expanded for display rather than reusing Assimp vertex indices; the triangle surfaces
are the same resources. Unsupported/malformed resources fail explicitly. ASCII STL and non-Mesh robot geometry are
currently unsupported in this renderer and rejected, not silently omitted.

viz/collision.py consumes surface arrays and scene/query data; it has no robot FK or collision-engine dependency.
It reuses build_scene_figure and actual S1 solids, preserving B2 default behavior. Dense scene Frame/Port labels
are hidden in this new figure only. All modeled robot bodies are visible by default; unselected bodies remain
translucent when a pair is selected. LiDAR has no collision Mesh and is explicitly shown as incomplete coverage.

The self-contained offline HTML has Raw/Effective/ACM Filtered/Environment selectors, a specific-pair list,
red/orange body highlighting and yellow actual FCL contacts. Independent Robot/Scene/Contacts/Link Frames switches
are implemented. The numeric panel reports sample index, original q, selected pair, decision, policy and identity.
An equal-metric aspect and identical default camera are used for all samples; ranges include every surface.
The capped scrollable control panel leaves the robot visible without scrolling through the whole policy JSON.

Native protocol v1 is additively extended with contacts_base; existing booleans, pair lists and decisions are unchanged.
Contacts come only from actual raw self/environment CollisionResult contacts. Position is converted from model root
into base_link using the same root-to-base transform; normal is rotated, depth copied, pair names retained.
There is no invented centroid or fake nearest point. Raw contacts are also shown when selecting an ACM-filtered pair;
they are not misrepresented as forbidden Effective contacts. One contact per pair / total 10,000 remains the query limit.
A contact dot does not describe the full intersection surface or guarantee minimum clearance.
A Native primitive fixture with a fixed root offset of 1 m validates that contact positions return to base coordinates.

## Generated results and numeric conservation

Build: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2_5/build_293cca8e20`

Final viewers: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2_5/viewers_12a641cab9`

- [Sample 8](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2_5/viewers_12a641cab9/sample_8.html)
- [Sample 73](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2_5/viewers_12a641cab9/sample_73.html)
- [Sample 0](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2_5/viewers_12a641cab9/sample_0.html)

Sample 0 means original G2 row 0, not the all-zero configuration. Extra all-zero gripper/joint3 probes are separately
named in contact_region_probes.json. None of these probes change the ACM configuration.

The CLI re-reads G2 via load_results(model=model), queries both policies over all 100 original rows, and requires
all raw/environment booleans, Raw pairs, Effective pairs/decisions and filtered pairs to exactly match prior C2.
All 200 checks passed: 100 self/effective collisions, 2 environment collisions (8,73), 2 both, no ERROR/CLEAR.
No q/index/negative-Z filtering, no deduplication, no new sampling. Query timing is expected to vary and is not a
collision semantic comparison. Backend SHA changes because the executable now emits contact evidence.

All 24 Mesh are visible for 8/73/0. Sample 8 and 73 link4 surfaces differ, while base_link/shell surfaces and all
38 cabinet/object vertices are identical. In sample 8 selected cab_bottom/gripper_link are highlighted; in 73 selected
gripper_link/router_2_front_bottom are highlighted. Browser DOM checks validate those exact object IDs/colors.

## Geometry audit supplement (no new ACM allowances)

Below, relative-matrix change is max absolute element change over selected 0/8/73 plus gripper/joint3 probes,
not a distance metric. FCL depth is a reported contact diagnostic, not calibrated physical penetration or clearance.

| Pair | Relative matrix change | Example Base contact [m] | Reported depth [m] | Assessment |
|---|---:|---|---:|---|
| back_shell_green_link / base_link | 0 | [-0.11093302472276688, -0.0711149774986267, -0.03922981768846512] | 0.0019233382218084988 | Fixed relative geometry; additional model-only candidate, retain pending review |
| link4 / servo_link2 | 2.22045e-16 | [0.09049232004046401, -0.11765756302250259, 0.24778184165211486] | 0.00015411147365050528 | Fixed relative geometry; additional model-only candidate, retain pending review |
| link2 / servo_link1 | 1.23653 | [0.04756144609608225, 0.01327775103551312, 0.11221645725715251] | 0.0013400605493136403 | Articulated geometry; retain detection, inspect hinge/mimic and physical contact design |
| link2 / link3 | 1.86563 | [0.02981772374545381, 0.03997329291476616, 0.23266750481391968] | 2.59226792752483e-05 | Articulated geometry; retain detection, inspect hinge/mimic and physical contact design |
| gripper_link / l_in_link | 0.295511 | [0.10317054036730854, -0.16973960131437546, 0.23323837618745913] | 1.3194873652918465e-06 | Articulated geometry; retain detection, inspect hinge/mimic and physical contact design |
| l_link / r_link | 0.564622 | [0.08859246397651001, -0.14998821658120523, 0.23694564344899116] | 0.000913808611904824 | Articulated geometry; retain detection, inspect hinge/mimic and physical contact design |

back_shell_green_link/base_link and link4/servo_link2 have invariant relative transforms in all probes and FCL points
on their overlapping mesh regions; this supports fixed-assembly candidate status but not an automatic exemption.
link2/servo_link1 and link2/link3 are articulated. C2's joint3-endpoint evidence is consistent with the new contact
probe regions; changes in collision point/depth do not establish whether the hinge mesh interference is intended.
Gripper internal transforms change at r_joint=0.3; l_out_link/r_out_link contact appears at zero but not in that probe.
This demonstrates why the r_joint=0 100/100 counts cannot justify blanket gripper permissions.
All prior policies remain byte-identical. Physical geometry and intended assembly contacts still require CAD/hardware
review; LiDAR/plug/TCP/cable and clearance limitations remain. No D stage or 10,000-sample work was performed.

## Test and browser evidence

Python: **441 passed, 1 warning, 0 skipped** (16.33 s), original 437 retained. Four new tests cover nonzero collision
RPY/nonuniform scale, Native link transform composition, all meshes and distinct poses with fixed scene, actual
contact-to-object association, default label state, self-contained HTML and no-overwrite behavior.
Native CTest: 1/1 PASS, including new root-offset contact-position fixture and all prior C1/C2 assertions.
Chrome headless actually loaded all three full HTML pages and created WebGL canvases. JS actions verified 24 visible
robot bodies, pair highlighting, Robot hide/restore, Scene hide/restore, Contacts toggle, Link Frames toggle and Filtered
selection. Screenshots were inspected; this is additional evidence, not a substitute for your browser acceptance.

New files: collision/assets.py, viz/collision.py, scripts/view_collision.py, tests/test_collision_visualization.py,
and this report. Only Native engine.cpp/native_tests.cpp are extended; no official model, policy YAML, historical
query result or existing Python tests are changed. C2's pre-existing uncommitted work and S1 documentation are retained.

## Reproduce

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
C25_ID="$(.venv/bin/python -c 'import uuid;print(uuid.uuid4().hex)')"
C25_BUILD="$PWD/results/c2_5/build_$C25_ID"
cmake -S cpp/collision_backend -B "$C25_BUILD/build" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$C25_BUILD/build" -j2
ctest --test-dir "$C25_BUILD/build" --output-on-failure
export C1_BACKEND="$C25_BUILD/build/collision_backend"
.venv/bin/python -B scripts/view_collision.py --backend "$C1_BACKEND" --samples 8 73 0
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests
```

CLI output is a fresh results/c2_5/<run_id>/ directory; --output-dir can select another nonexistent run directory.
--reference-run selects the immutable C2 comparison run explicitly, never by modification time. The reference default
is the original C2 run documented above. Existing files are refused. Open sample_8.html / sample_73.html / sample_0.html;
select Environment for 8/73 or Effective/Filtered for self contacts; choose individual pairs and rotate/zoom as needed.
Browser harness screenshots are evidence only; the real output HTML has no injected test harness.

Visualization Implementation Status: Completed — Pending User Review.
Collision Geometry Validation Status: software consistency verified; physical interpretation PARTIAL.
Remaining Physical Model Limitations: LiDAR, RJ45/tool/TCP/cable, calibrated gripper, ACM approval and clearance.
