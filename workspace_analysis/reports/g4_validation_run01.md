# G4 Workspace Validation



Software status: **PASS**; experiment profile: **formal**.

Stage acceptance: **pending_user_review**. Hardware checked: false. P2.2 acceptance is not declared here.

Completed UTC: 2026-11-03T05:39:30.465472+00:00



## 1. Robot Model



Model: /home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description/urdf/jetrover.xacro; 5 independent arm joints.

Base: `base_link`; EE: `end_effector_link`; units m/rad.

Expanded URDF SHA256: `5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`.

Configuration SHA256: `0028f14160dc6bbf1dec0671c681961ebc46eedc0e0e7bd0c3d45c0ca5800b78`.

| Joint | Lower [rad] | Upper [rad] |
| --- | --- | --- |
| joint1 | -2.09 | 2.09 |
| joint2 | -2.09 | 2.09 |
| joint3 | -2.09 | 2.09 |
| joint4 | -2.09 | 2.09 |
| joint5 | -2.09 | 2.09 |



## 2. FK Validation — Prior Evidence



This section cites previously user-executed files. G4 did not launch RSP or rerun ROS2 TF integration.

G1-A: /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/g1a_fk.json (SHA256 `513371615f231cd27d802094361322245eabe50dc8a6e1e9f90976cb0fb3146c`), reported status SOFTWARE_CHECKS_PASSED.

| Case | Position error [m] | Rotation error [rad] |
| --- | --- | --- |
| Zero | 0.0 | 0.0 |
| Nonzero | 1.2197462367390768e-16 | 1.2888423398670574e-16 |
| Near-limit | 1.1816510524180758e-16 | 1.5027151625620989e-16 |

G1-B: /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/g1b_tf_v3.json (SHA256 `9cf0e8450ff0b484baa66b419f9797cb36169b9816f64ff1bf4a580c2957129c`), reported status PASS.

| Case | Position error [m] | Rotation error [rad] |
| --- | --- | --- |
| Zero | 0.0 | 0.0 |
| Nonzero | 2.0874425738630802e-16 | 7.502815654363649e-16 |
| Near-limit | 1.6199010924212654e-16 | 4.856017471462086e-16 |

Prior strict endpoint identity: True; prior RSP exit codes: [0, 0, 0]. No new TF run.



## 3. Workspace Sampling



Algorithm: `independent_uniform_joint_space`. N=[100, 1000, 10000]; seeds=[42, 43, 44].

The approved G2 sampler and G1-A PyKDL FK are reused unchanged. q is uniform independently within URDF limits; Cartesian density is not uniform. All negative-Z samples are retained.

G2 pairs contain float64 q(N,5), position(N,3), rotation(N,3,3), quaternion_xyzw(N,4). Each CSV row identifies its source pair and both file hashes. Full provenance is in validation.json.

Same seed across N uses RNG prefixes; runs are not independent across N.



## 4. Reproducibility



all four arrays: identical shape/dtype and np.array_equal; no tolerance relaxation

Two independent calls: N=1000, seed=42. Result: **PASS**.

| Array | First shape/dtype | Second shape/dtype | Exactly equal | Max absolute difference |
| --- | --- | --- | --- | --- |
| q | [1000, 5] / float64 | [1000, 5] / float64 | True | 0.0 |
| position | [1000, 3] / float64 | [1000, 3] / float64 | True | 0.0 |
| rotation | [1000, 3, 3] / float64 | [1000, 3, 3] / float64 | True | 0.0 |
| quaternion_xyzw | [1000, 4] / float64 | [1000, 4] / float64 | True | 0.0 |

Max position error: 0.0 m; max rotation error: 2.0770370905276127e-16 rad; quaternion orientation error: 1.2412670766236366e-16 rad.

Quaternion q/-q are orientation-equivalent but do not satisfy deterministic array equality. Timestamps, elapsed times and NPZ byte hashes are not required to be equal.

Software versions: `{"PyKDL": "1.5.1", "PyYAML": "5.4.1", "numpy": "1.21.5", "python": "3.10.12", "scipy": "1.8.0", "urdfdom_py": "1.2.1", "xacro": "2.1.1"}`.

same software environment only; no cross-platform or cross-version bitwise guarantee.



## 5. Sampling Stability



Within each dataset, position std uses ddof=0. Across seeds, every scalar metric has mean and sample std (ddof=1); only three seeds in the formal design, with no confidence guarantee.

### X statistics [m]



| N | Seed | Min | Max | Mean | Std (ddof=0) | AABB edge |
| --- | --- | --- | --- | --- | --- | --- |
| 100 | 42 | -0.338592614 | 0.375890685 | 0.0134331365 | 0.143379924 | 0.714483299 |
| 100 | 43 | -0.299384344 | 0.405315647 | 0.0244505986 | 0.127061483 | 0.704699991 |
| 100 | 44 | -0.316005263 | 0.408345618 | 0.0174895023 | 0.153715869 | 0.724350881 |
| 1000 | 42 | -0.341077263 | 0.379926119 | 0.0256473822 | 0.134718233 | 0.721003381 |
| 1000 | 43 | -0.35500306 | 0.405315647 | 0.0270998582 | 0.133928385 | 0.760318707 |
| 1000 | 44 | -0.340291571 | 0.412844496 | 0.0230115355 | 0.142640251 | 0.753136067 |
| 10000 | 42 | -0.364976238 | 0.410507287 | 0.0252026317 | 0.136046808 | 0.775483525 |
| 10000 | 43 | -0.365434461 | 0.409037088 | 0.0251591878 | 0.135206553 | 0.774471549 |
| 10000 | 44 | -0.36050599 | 0.413731284 | 0.0270093262 | 0.137045975 | 0.774237274 |

### Y statistics [m]



| N | Seed | Min | Max | Mean | Std (ddof=0) | AABB edge |
| --- | --- | --- | --- | --- | --- | --- |
| 100 | 42 | -0.365987027 | 0.371290239 | 0.00411810275 | 0.183586077 | 0.737277267 |
| 100 | 43 | -0.305596259 | 0.346177151 | -0.00161327273 | 0.156395176 | 0.65177341 |
| 100 | 44 | -0.346050373 | 0.370267413 | 0.0284566183 | 0.17623793 | 0.716317786 |
| 1000 | 42 | -0.376350733 | 0.378715665 | -0.000316008381 | 0.16936226 | 0.755066398 |
| 1000 | 43 | -0.388745316 | 0.387590891 | 0.00698172368 | 0.165483142 | 0.776336207 |
| 1000 | 44 | -0.380955961 | 0.383791465 | -0.000158565435 | 0.168945046 | 0.764747426 |
| 10000 | 42 | -0.391086551 | 0.390261186 | 0.00194806246 | 0.166976443 | 0.781347737 |
| 10000 | 43 | -0.388745316 | 0.390350153 | -4.61672456e-05 | 0.165847402 | 0.779095469 |
| 10000 | 44 | -0.390989245 | 0.391334402 | 0.0040028875 | 0.16832855 | 0.782323648 |

### Z statistics [m]



| N | Seed | Min | Max | Mean | Std (ddof=0) | AABB edge |
| --- | --- | --- | --- | --- | --- | --- |
| 100 | 42 | -0.184626755 | 0.487812258 | 0.199166838 | 0.173957593 | 0.672439013 |
| 100 | 43 | -0.193363074 | 0.468973582 | 0.199904447 | 0.184664113 | 0.662336656 |
| 100 | 44 | -0.186210752 | 0.445458164 | 0.175368514 | 0.178752749 | 0.631668916 |
| 1000 | 42 | -0.203059415 | 0.49939362 | 0.198377498 | 0.17795405 | 0.702453035 |
| 1000 | 43 | -0.211472449 | 0.49775755 | 0.185231722 | 0.178983412 | 0.709229999 |
| 1000 | 44 | -0.203130012 | 0.502698059 | 0.189599307 | 0.177883864 | 0.70582807 |
| 10000 | 42 | -0.208602724 | 0.503448742 | 0.197002455 | 0.179965087 | 0.712051466 |
| 10000 | 43 | -0.211472449 | 0.504041224 | 0.199104538 | 0.180612883 | 0.715513673 |
| 10000 | 44 | -0.212312321 | 0.504303231 | 0.196411538 | 0.178907463 | 0.716615552 |



| N | Seed | Reused | AABB volume [m3] | FK loop [s] | Hit voxels |
| --- | --- | --- | --- | --- | --- |
| 100 | 42 | True | 0.35422224158362287 | 0.01354999500017584 | 99 |
| 100 | 43 | False | 0.30421434980780815 | 0.013670132000697777 | 99 |
| 100 | 44 | False | 0.3277511571635474 | 0.013249551000626525 | 99 |
| 1000 | 42 | False | 0.3824192440184207 | 0.13218579900058103 | 960 |
| 1000 | 43 | False | 0.4186321850164558 | 0.1318989009996585 | 964 |
| 1000 | 44 | False | 0.4065279366118013 | 0.13105237899981148 | 964 |
| 10000 | 42 | True | 0.4314478598841818 | 1.374149507999391 | 7554 |
| 10000 | 43 | False | 0.4317318454728081 | 1.301120334001098 | 7491 |
| 10000 | 44 | False | 0.4340569981386331 | 1.3048276600002282 | 7614 |

AABB volume is only a bounding-box statistic, not reachable workspace volume. G2 FK loop scope; reused timings are historical, not a controlled benchmark.



| N | Metric | Across-seed mean | Across-seed SD (ddof=1) |
| --- | --- | --- | --- |
| 100 | x_min_m | -0.317994074 | 0.0196796502 |
| 100 | x_max_m | 0.396517317 | 0.0179273151 |
| 100 | x_mean_m | 0.0184577458 | 0.00557218443 |
| 100 | x_std_m | 0.141385759 | 0.013438623 |
| 100 | x_aabb_edge_m | 0.714511391 | 0.00982547508 |
| 100 | y_min_m | -0.33921122 | 0.0307707936 |
| 100 | y_max_m | 0.362578268 | 0.0142129878 |
| 100 | y_mean_m | 0.0103204828 | 0.015965642 |
| 100 | y_std_m | 0.172073061 | 0.01406577 |
| 100 | y_aabb_edge_m | 0.701789487 | 0.0445649074 |
| 100 | z_min_m | -0.18806686 | 0.00465453208 |
| 100 | z_max_m | 0.467414668 | 0.0212200372 |
| 100 | z_mean_m | 0.191479933 | 0.013957771 |
| 100 | z_std_m | 0.179124818 | 0.00536294911 |
| 100 | z_aabb_edge_m | 0.655481528 | 0.0212319283 |
| 100 | aabb_volume_m3 | 0.32872925 | 0.0250182895 |
| 100 | fk_elapsed_s | 0.0134898927 | 0.000216636359 |
| 100 | hit_voxels | 99 | 0 |
| 1000 | x_min_m | -0.345457298 | 0.00827620128 |
| 1000 | x_max_m | 0.399362087 | 0.0172478563 |
| 1000 | x_mean_m | 0.0252529253 | 0.00207250885 |
| 1000 | x_std_m | 0.137095623 | 0.00481800134 |
| 1000 | x_aabb_edge_m | 0.744819385 | 0.0209355941 |
| 1000 | y_min_m | -0.382017336 | 0.00626508669 |
| 1000 | y_max_m | 0.383366007 | 0.00445288309 |
| 1000 | y_mean_m | 0.00216904995 | 0.00416864107 |
| 1000 | y_std_m | 0.167930149 | 0.00212941341 |
| 1000 | y_aabb_edge_m | 0.765383344 | 0.0106491541 |
| 1000 | z_min_m | -0.205887292 | 0.00483701679 |
| 1000 | z_max_m | 0.499949743 | 0.00251676622 |
| 1000 | z_mean_m | 0.191069509 | 0.0066950709 |
| 1000 | z_std_m | 0.178273775 | 0.000615564406 |
| 1000 | z_aabb_edge_m | 0.705837035 | 0.00338849093 |
| 1000 | aabb_volume_m3 | 0.402526455 | 0.0184351068 |
| 1000 | fk_elapsed_s | 0.13171236 | 0.000589286413 |
| 1000 | hit_voxels | 962.666667 | 2.30940108 |
| 10000 | x_min_m | -0.363638896 | 0.00272283232 |
| 10000 | x_max_m | 0.411091886 | 0.0024010799 |
| 10000 | x_mean_m | 0.0257903819 | 0.00105586017 |
| 10000 | x_std_m | 0.136099779 | 0.000920854324 |
| 10000 | x_aabb_edge_m | 0.774730783 | 0.000662334317 |
| 10000 | y_min_m | -0.390273704 | 0.00132451688 |
| 10000 | y_max_m | 0.390648581 | 0.000595602473 |
| 10000 | y_mean_m | 0.0019682609 | 0.00202460294 |
| 10000 | y_std_m | 0.167050798 | 0.00124224412 |
| 10000 | y_aabb_edge_m | 0.780922285 | 0.00165560914 |
| 10000 | z_min_m | -0.210795831 | 0.00194515709 |
| 10000 | z_max_m | 0.503931066 | 0.00043776605 |
| 10000 | z_mean_m | 0.197506177 | 0.00141540259 |
| 10000 | z_std_m | 0.179828477 | 0.000860877983 |
| 10000 | z_aabb_edge_m | 0.714726897 | 0.00238159259 |
| 10000 | aabb_volume_m3 | 0.432412234 | 0.00143146691 |
| 10000 | fk_elapsed_s | 1.32669917 | 0.0411349874 |
| 10000 | hit_voxels | 7553 | 61.5060973 |



Observed comparisons for N=100 versus N=10000 (descriptive, not a convergence test):
- X across-seed average sampled extrema: [-0.31799407, 0.39651732] -> [-0.3636389, 0.41109189] m. These are sampled extrema, not strict boundaries.
- Y across-seed average sampled extrema: [-0.33921122, 0.36257827] -> [-0.3902737, 0.39064858] m. These are sampled extrema, not strict boundaries.
- Z across-seed average sampled extrema: [-0.18806686, 0.46741467] -> [-0.21079583, 0.50393107] m. These are sampled extrema, not strict boundaries.
- Across-seed sample SD of X mean: 0.0055721844 -> 0.0010558602 m; decreased.
- Across-seed sample SD of X std: 0.013438623 -> 0.00092085432 m; decreased.
- Across-seed sample SD of Y mean: 0.015965642 -> 0.0020246029 m; decreased.
- Across-seed sample SD of Y std: 0.01406577 -> 0.0012422441 m; decreased.
- Across-seed sample SD of Z mean: 0.013957771 -> 0.0014154026 m; decreased.
- Across-seed sample SD of Z std: 0.0053629491 -> 0.00086087798 m; decreased.
Only 6/6 mean/std dispersion measures decreased in this comparison. The table retains residual fluctuations; no universal convergence, strict boundary or confidence guarantee follows.



## 6. Voxel Hit Statistics



```json

{
  "origin_m": [
    -0.6,
    -0.6,
    -0.4
  ],
  "upper_exclusive_m": [
    0.6,
    0.6,
    0.7999999999999999
  ],
  "requested_upper_m": [
    0.6,
    0.6,
    0.8
  ],
  "voxel_size_m": 0.02,
  "shape": [
    60,
    60,
    60
  ],
  "axis_order": [
    "x",
    "y",
    "z"
  ],
  "index_rule": "edge[k] = origin + k*size in float64; cell [edge[k],edge[k+1]); index = searchsorted(edges, point, side='right') - 1 on each axis",
  "out_of_bounds": "ERROR with first original row index; never clip or discard",
  "meaning": "sample-hit cells only; empty is not unreachable; hits are not collision-safe",
  "resolution_meaning": "analysis resolution, not robot positioning precision"
}

```

One fixed grid is shared across all runs. Out-of-region points cause an error; there is no clipping or per-run AABB grid fitting. Unique hit counts are in the table/CSV. Empty cells are not proven unreachable; hit cells are not wholly reachable or collision-safe. No sum of hit-cell volumes is claimed as true reachable volume.



## 7. Visualization



G3 was previously accepted by the user. It exports offline 3D and XY/XZ/YZ HTML using scripts/plot_workspace.py --config config/workspace.yaml --input-prefix <G2-prefix> --output-prefix <new-prefix>. G4 does not regenerate these HTML files.

- /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/workspace_n10000_seed42_3d.html (exists=True, SHA256=bd37d31572ae7f0368d30d1b5e94d406bc03a09f108a5df9740c10eb9a3a1cff). File inventory only; no new visual acceptance.

- /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/workspace_n10000_seed42_projections.html (exists=True, SHA256=39dfc0b7705dbea64db7db911262de0ef2bf8716bed00ada531c522271065795). File inventory only; no new visual acceptance.



## 8. Limitations



- Joint-limit-constrained geometric FK samples only; no collision checking.

- Self/environment collision, motor safety limits and dynamics are not evaluated.

- RJ45 orientation, approach, contact and insertion/extraction constraints are not evaluated.

- Finite samples do not establish a strict reachable workspace boundary or volume.

- The URDF EE frame is not a calibrated RJ45 TCP.

- Uniform joint samples are not uniform Cartesian-position samples.

- Only three formal seeds: preliminary evidence, no confidence guarantee.

- Hit voxels are neither a collision map nor a strict reachable volume.

- Hardware positioning accuracy is not validated; URDF limits are not measured motor safety limits.

- Base origin is not ground. EE has no calibrated RJ45 TCP or insertion-direction/contact constraints.



## 9. Conclusion



This run has software status PASS and profile formal. It provides model-based sampling, deterministic reproduction checks and descriptive statistics at a stated voxel resolution. It does not establish a complete planning system or a strict workspace boundary.

User review and formal P2.2 acceptance remain pending.
