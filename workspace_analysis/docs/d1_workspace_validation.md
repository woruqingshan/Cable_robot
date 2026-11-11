# D1 Collision-Constrained Workspace Feasibility Recovery

结论：比较链路、局部几何及审计软件完成；**整体模型可行配置尚未恢复，正式几何审核 PARTIAL**。没有将零结果包装为安全空间，也没有通过扩大 ACM 或任意缩小全局机器人外形提高比例。

## 设计与实际输出

固定 q、官方运动学、S1 场景、r_joint=0.3、零间隙、ACM v1，只比较：

- Official Full Geometry + ACM v1。
- Pair-local Refined Geometry + 同一 ACM v1。

没有 v2。Robot-Only 为单独 SceneSnapshot：清空障碍/实体/Ports、保留坐标图并明确诊断 coverage，具有独立 Scene Hash；不是 IDC 安全空间。100 与 1,000 均有 IDC / Robot-Only 两种情境；10,000 仅为 IDC 探索性对照，避免重复已证明仍被自身阻塞的 Robot-Only 大规模计算。

| Run | 位置 |
|---|---|
| 100 + Robot-Only | `results/constrained_workspace/d1_n100_f36a9a/` |
| 1,000 + Robot-Only | `results/constrained_workspace/d1_n1000_2af837/` |
| 10,000 IDC 探索 | `results/constrained_workspace/d1_n10000_ac67fd/` |
| 最终 85 姿态多点/逐对距离审计 | `results/c2_7/audit_final_638aa7/` |
| Official / Refined 完整机器人对照 | `results/c2_7/paired_viewers_172aec/` |

每个 D1 Run：顶层 manifest、summary、comparison.csv、workspace_results.npz、figures/profile_comparison.html；每个 `idc/official`、`idc/refined`（以及 robot_only 子目录）包含 per_sample.csv、evaluations.json、独立 manifest/summary、原 G2 数组 + 分类掩码 NPZ 和 HTML。分目录避免不同 Profile 结果互相覆盖，所有新输出目录均必须不存在。

`comparison.csv` 逐行保留原 sample_index/q、原/改进拒绝状态、增加/移除 Pair；顶层 summary 保存 recovered_indices、new_rejected_indices、both_rejected、both_candidate、changed_pair_indices。没有按 EE XYZ 去重或重新生成 q。10,000 是探索性诊断模型实验，不是正式安全认证。

## 实际统计

| N | 情境 | Profile | Effective Self | Env / Both | Feasible | Mean / P95 [ms] |
|---:|---|---|---:|---:|---:|---:|
| 100 | idc | official | 100 | 2 / 2 | 0 / 0% | 11.42 / 16.43 |
| 100 | idc | refined | 100 | 2 / 2 | 0 / 0% | 21.94 / 32.59 |
| 100 | robot_only | official | 100 | 0 / 0 | 0 / 0% | 5.34 / 6.51 |
| 100 | robot_only | refined | 100 | 0 / 0 | 0 / 0% | 9.96 / 11.26 |
| 1,000 | idc | official | 1,000 | 21 / 21 | 0 / 0% | 11.87 / 16.61 |
| 1,000 | idc | refined | 1,000 | 21 / 21 | 0 / 0% | 22.58 / 31.85 |
| 1,000 | robot_only | official | 1,000 | 0 / 0 | 0 / 0% | 5.33 / 6.46 |
| 1,000 | robot_only | refined | 1,000 | 0 / 0 | 0 / 0% | 9.87 / 12.04 |
| 10,000 | idc | official | 10,000 | 232 / 232 | 0 / 0% | 11.89 / 16.81 |
| 10,000 | idc | refined | 10,000 | 232 / 232 | 0 / 0% | 22.93 / 32.60 |

全部组 Raw Self = Effective Self = Total；Final CLEAR=0、UNVERIFIED=0、ERROR=0，所有样本 Final COLLIDING。这里 UNVERIFIED=0 并不表示物理覆盖已认证：确定有模型碰撞的样本优先标为 COLLIDING，仍带有诊断 Profile 和缺失覆盖身份。

所有尺度中：recovered=0、new_rejected=0、both_candidate=0、both_rejected=N。**每个 IDC 样本的具体 Pair 集合确有变化**，主要移除 green/link1、link2/servo_link1、gripper/l_in；不能把“最终布尔仍拒绝”解释为 Profile 没有生效。发生 Pair 变化的 Index 为各原始输入的完整 0…N-1，完整列表在 summary/CSV。Robot-Only 也仍全部拒绝，持续性夹爪模型问题是当前主要阻塞。

余下六组持续阻塞：gripper/r_in、l_in/l_out、l_link/l_out、l_link/r_link、r_in/r_out、r_link/r_out。link2/link3 仍有姿态依赖，其完整官方几何保持检测。没有把这些 Pair 简单加进 ACM。源 STL 可见的齿轮啮合与分支闭环机构需要更明确的实体接触/闭环依据，不能从频率或有限接触点得到整对安全豁免。

Clearance 分布：两 Profile 的 Self 最短间距均为 0（仍有拒绝接触）；100 / 1,000 / 10,000 IDC 环境间距中位数均约 0.194814 m；P05 分别约 0.035693 / 0.034708 / 0.038847 m。环境布尔、具体碰撞对及距离在同 q 下保持一致。Robot-Only 没有环境对象，环境距离明确 unavailable/null，没有伪造正无穷间隙。完整分位数、可用/缺失数在每组 summary 的 clearance_distribution。

## API 与数值语义

```python
from workspace_analysis.collision.geometry_profile import load_geometry_profile
from workspace_analysis.collision.evaluator import Evaluator
from workspace_analysis.collision.policy import load_policy

# model、scene 仍来自原 G1/B1 加载器；资产目录必须不存在。
profile = load_geometry_profile(profile_yaml, model, asset_dir=new_asset_dir)
policy = load_policy(policy_yaml, model)
with Evaluator(model, scene, policy, backend_path,
               geometry_profile=profile, gripper=0.3,
               clearance={'environment_m': 0.0, 'self_m': 0.0}) as evaluator:
    rows = evaluator.evaluate(q, batch_size=64)
```

不传 geometry_profile 时保持旧接口和默认官方几何。GeometryProfile、ACM Policy 是两个不同变量。Raw / Effective 对当前组合模型定义；Native 同时保存 official_raw_self_collision、official_contacts_base、official_pair_geometry_distances，便于审计，不把几何修改伪装为策略修改。

行级输出增加 Source URDF Identity、Effective Collision Geometry Identity、Profile、Missing Coverage；诊断 Profile 的 verification_status 为 DIAGNOSTIC_GEOMETRY_ONLY，Final 不会自动变成 CLEAR。模型可行布尔即使未来为 true，也只描述该诊断范围，不能作为物理安全或网线插拔成功率。

INIT 在保留原 URDF/Scene/本地资源校验后递归校验实际派生 Native 模型。身份含源/派生 URDF、源/派生/组合 Geometry、Profile、Policy、Scene、Backend SHA。Run 记录 Git HEAD、工作源码哈希、输入 G2 Pair 哈希、夹爪/阈值和时间戳。Profile canonical 包含资产 URI，跨输出目录可能哈希不同；逐资源 SHA 和参数支持内容对照，从 materialized_profile 可重载原身份且不重写资产。

D0 的原 load_results、采样器、summary 语义、旧命令默认行为未更改；D1 复用其封装，只添加可选 Profile/Scene 输入。现有 G1 包入口和 Scene Schema 没有变更。

## Viewer 和校验

D1 顶层图展示原始工作空间、Official Rejected、Refined Rejected、真实候选、环境碰撞、Unknown/Unverified，等比例 base_link 坐标。候选为空时绿色 Trace 确实为空。类别允许重叠，不能相加当成总样本数；比较与统计均保持配置级定义。

`paired_viewers_172aec/` 中 reference_zero 是装配候选，**目前仍 COLLIDING**；Sample 19 为折叠非相邻碰撞；8/73 为机柜碰撞。Official / Refined 各自 q、夹爪、策略和 Native 身份都记录于 HTML 及 viewer_queries.json。完整机器人默认显示官方环境/非 Target 表面；派生 Target 表面在最终审计 local_*.html 的 Original / Derived 切换中显示，避免用全局缩小外形误导环境覆盖。

`check_geometry_workspace.py` 检查 G2 输入 SHA、全部原 q/Index/XYZ/姿态、每个子 Run 的 CSV/JSON/NPZ/HTML 数组、Profile/Policy/Scene 身份、配对 Pair 差异及统计守恒。人工审阅可直接打开 HTML；本轮还生成并检查了实际三角面的静态 PNG。交互浏览器的最终视觉验收留给用户，不以 pytest 数量代替。

## 可复制用户验收命令

不覆盖已有 Run，不控制实体机器人，不安装依赖。下列命令从同一终端连续执行：

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src:$PYTHONPATH"
review_run="$(date -u +%Y%m%dT%H%M%SZ)_${RANDOM}"
build_dir="$PWD/results/c2_7/${review_run}_build"
audit_dir="$PWD/results/c2_7/${review_run}_audit"
run100="$PWD/results/constrained_workspace/d1_${review_run}_n100"
run1000="$PWD/results/constrained_workspace/d1_${review_run}_n1000"
run10000="$PWD/results/constrained_workspace/d1_${review_run}_n10000"
mkdir -p "$build_dir"
cmake -S cpp/collision_backend -B "$build_dir/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir/build" -j2
ctest --test-dir "$build_dir/build" --output-on-failure
export C1_BACKEND="$build_dir/build/collision_backend"

# 完整 85 姿态，含关节/夹爪/留出姿态及逐 Pair 距离。
.venv/bin/python -B scripts/audit_geometry.py --backend "$C1_BACKEND" \
  --profile refined_assembly_v0 --output-dir "$audit_dir"
# Official / Refined 单次同 q 查询，使用各自新输出目录：
.venv/bin/python -B scripts/audit_geometry.py --backend "$C1_BACKEND" \
  --profile official_full --q 0 0 0 0 0 --gripper 0.3 \
  --output-dir "$PWD/results/c2_7/${review_run}_official_zero"
.venv/bin/python -B scripts/audit_geometry.py --backend "$C1_BACKEND" \
  --profile refined_assembly_v0 --q 0 0 0 0 0 --gripper 0.3 \
  --output-dir "$PWD/results/c2_7/${review_run}_refined_zero"

.venv/bin/python -B scripts/evaluate_geometry_workspace.py --backend "$C1_BACKEND" \
  --input-prefix results/workspace_n100_seed42 --output-dir "$run100"
.venv/bin/python -B scripts/evaluate_geometry_workspace.py --backend "$C1_BACKEND" \
  --input-prefix results/g4_run01/experiments/workspace_n1000_seed42 --output-dir "$run1000"
# 可选探索：同一官方/诊断几何、同一 ACM v1；IDC 38 障碍全部保留。
.venv/bin/python -B scripts/evaluate_geometry_workspace.py --backend "$C1_BACKEND" \
  --input-prefix results/workspace_n10000_seed42 --output-dir "$run10000" --skip-robot-only

.venv/bin/python -B scripts/check_geometry_workspace.py --run-dir "$run100"
.venv/bin/python -B scripts/check_geometry_workspace.py --run-dir "$run1000"
.venv/bin/python -B scripts/check_geometry_workspace.py --run-dir "$run10000"
.venv/bin/python -B scripts/export_geometry_viewers.py --backend "$C1_BACKEND" \
  --run-dir "$run100" --output-dir "$PWD/results/c2_7/${review_run}_paired_viewers"
xdg-open "$run1000/figures/profile_comparison.html"
xdg-open "$audit_dir/figures/local_l_link_r_link.html"
xdg-open "$PWD/results/c2_7/${review_run}_paired_viewers/official_19.html"
xdg-open "$PWD/results/c2_7/${review_run}_paired_viewers/refined_19.html"

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests
```

可直接审阅已生成的 Run：

```bash
.venv/bin/python -B scripts/check_geometry_workspace.py \
  --run-dir results/constrained_workspace/d1_n1000_2af837
xdg-open results/constrained_workspace/d1_n1000_2af837/figures/profile_comparison.html
xdg-open results/c2_7/audit_final_638aa7/figures/local_link2_servo_link1.html
xdg-open results/c2_7/paired_viewers_172aec/official_8.html
xdg-open results/c2_7/paired_viewers_172aec/refined_8.html
```

Software Implementation Status：完成、463 项通过。Model-Level Geometry Validation：PARTIAL；局部诊断变化可解释且通过软件正反例，仍不足以确认剩余实体装配/闭环。Physical Hardware Validation：未执行。D1 Research Readiness：可复现的几何敏感性与受限空间比较研究已可运行；**尚未恢复全机器人可行配置，尚不能作为经过验证的安全规划几何输入**。
