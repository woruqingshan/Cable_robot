# D2 Task-Scoped Constrained Workspace Pilot

结论：系统已经将官方完整碰撞事实、任务范围阻塞、未验证内部机构接触独立计算并记录。**真实 JetRover 数据仍无 Task-Scope Candidate**，原因是 Scope A 的持续近轴装配/几何碰撞，不是内部诊断查询失效。

## 对照、数据与身份

A：Official Full Geometry + ACM v1，全机器人原语义。

B：完全相同几何、q、FK、S1、ACM v1、r_joint=0.3 rad、零 Clearance，加 task_scope_v0。没有采用 C2.7 切面几何，也没有修改 ACM v1。可选 Refined + Scope 对照未运行且 v0 显式拒绝该组合，不隐藏第二个实验变量。

原输入保持：workspace_n100_seed42、g4_run01/experiments/workspace_n1000_seed42、workspace_n10000_seed42。只用原 load_results，无重新采样、XYZ 去重、裁掉负 Z。负 Z 分别保留 14 / 174 / 1,840 个配置。

| N | Run |
|---:|---|
| 100 | `results/constrained_workspace/d2_n100_69f4c1/` |
| 1,000 | `results/constrained_workspace/d2_n1000_b1917d/` |
| 10,000 探索 | `results/constrained_workspace/d2_n10000_2e8d30/` |

每个 Run 保存 manifest.json、summary.json、evaluations.json、per_sample.csv（A/B 全行）、comparison.csv、workspace_results.npz 和 figures/workspace_comparison.html。NPZ 为原 G2 数组 + 五类掩码；它不是改写的 G2 NPZ/JSON 数据对。CSV/JSON 具有完整 Index/q/身份及实际内部 Pair。所有新输出目录必须不存在。

manifest 记录 Git HEAD、工作源码 SHA、Source URDF、Geometry、ACM、Task Scope、Scene、Backend SHA/Version、输入 G2 Pair SHA、夹爪和阈值。旧 Native identity 不因请求 Scope 而变化；单独的 task_scope.binding 与 scope_sha256 绑定每次请求，Evaluator 的 evaluation_sha256 含 Scope。没有依赖进程上一次设置过的范围状态。

## 实际结果

| 配置数 | Full Effective Self | Task Blocking Self | Internal Collision | Environment | Candidate / Ratio | Rejected | Unknown / Error |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 100 | 100 | 100 | 100 | 2 | 0 / 0% | 100 | 0 / 0 |
| 1,000 | 1,000 | 1,000 | 1,000 | 21 | 0 / 0% | 1,000 | 0 / 0 |
| 10,000 | 10,000 | 10,000 | 10,000 | 232 | 0 / 0% | 10,000 | 0 / 0 |

这里 Full / Blocking / Internal / Environment 可重叠，不能相加。Final 四态仍按原官方完整模型保留：全部 COLLIDING，无 CLEAR。内部机械验证仍为 UNVERIFIED_MECHANISM。Task Unknown=0 只表示明确范围内计算完成；不代表物理覆盖已认证。

每个样本的 Blocking Pair 集合均移除了实际存在的七对声明内部接触；改变的是全部原始 Index 0…N-1 的 Pair 集合，最终 Candidate 分类改变数为 0。没有修改名单直到出现绿点。

主要剩余阻塞：

| Pair | N100 | N1,000 | N10,000 |
|---|---:|---:|---:|
| back_shell_green_link / link1 | 100 | 1,000 | 10,000 |
| link2 / servo_link1 | 100 | 1,000 | 10,000 |
| link2 / link3 | 92 | 897 | 8,792 |
| back_shell_black_link / link3 | 8 | 72 | 867 |

前两对属于 Arm/Base 范围，经过活动关节，未被 ACM v1 审核许可。D1 的诊断切面曾改变这些 Pair，D2 按指令回到官方完整几何，因此它们重新成为明确瓶颈。这不是 Scope 状态污染，也不能把它们塞进 Gripper Internal 名单。link2/link3 及 Sample 19 的非相邻干涉持续保留。以上只对已测试配置成立，不能声称所有关节空间都无解。

10,000 在 100/1,000、真实 Native 正反例及稳定统计通过后执行，作为同一冻结 Scope 的探索，不是硬件安全实验。

## Clearance 与性能

任务范围 Self Clearance 仍全部为 0，因两个持续阻塞 Pair。环境中位数约 0.194814 m，P05 分别约 0.035693 / 0.034708 / 0.038847 m；全机器人/环境最短距离包括固定底盘，不能将其误认为纯 EE 间隙。详细分布和缺失数在 summary。

| N | A Mean [ms] | B Mean / P95 [ms] |
|---:|---:|---:|
| 100 | 11.52 | 14.06 / 19.30 |
| 1,000 | 11.85 | 14.60 / 19.37 |
| 10,000 | 11.90 | 14.69 / 19.42 |

不重启每行后端，按原顺序进行持久查询。batch_size 是分块调度，不宣称 FCL 向量化。Source 全模型与 Scope 的两套判定保留，不牺牲真实阻塞检测换取时延。

## API 和判定

```python
from workspace_analysis.collision.policy import load_policy
from workspace_analysis.collision.scope import load_scope
from workspace_analysis.collision.evaluator import Evaluator

policy = load_policy(policy_path, model)
scope = load_scope(scope_path, model, policy)
with Evaluator(model, scene, policy, backend_path,
               task_scope=scope, gripper=0.3,
               clearance={'self_m': 0.0, 'environment_m': 0.0}) as evaluator:
    rows = evaluator.evaluate(q, batch_size=64)
```

输入/模型/阈值不足时逐行 ERROR 或 Scope UNKNOWN，不静默丢行。Scope Candidate 取完整任务阻塞布尔、完整环境布尔及任务范围距离阈值；未知距离的正阈值不自动通过。内部诊断的未验证状态不会令所有范围计算无效，但明确留在 gripper_internal_verification、missing_coverage、model_scope_status、hardware_safe=null。

不传 task_scope，旧 C1–D1 API 和 Raw/Effective/Final 语义保留。Native 查询也可在同一 Client 中交替传入/不传 Scope，局部临时矩阵不会改变既有 ACM 或下一条查询。Native 返回的 zero_clearance_candidate 只代表零阈值谓词；正阈值最终由 Evaluator 记录并判断。

## 审阅与数据检查

最终全 21 对机构审计：`results/c2_8/assembly_allpairs_e7c61f/`。保留早期七对版本 `assembly_8c417a`，不覆盖。

完整机器人示例：`results/c2_8/examples_9842ae/`：

- sample_2：Gripper / Base。
- sample_8、sample_73：真实源模型环境碰撞。
- sample_19：折叠非相邻 Link 碰撞。
- sample_84：Gripper / Arm。

1,000 配置实际检出 Gripper/环境 19 个、Gripper/底盘/外壳 96 个、Gripper/Arm 10 个配置。这些分类可重叠。没有候选机器人 Viewer，因为没有真实候选；合成 Native 正例只作为软件验证，未混入 G2 工作空间。

Workspace 图保留 Original、Full Rejected、Task Rejected、Task Candidates、Environment Rejected、Unknown；空候选 Trace 真正为空，所有点和悬浮信息来自原 Sample Index/q/FK，坐标等比例。Robot Viewer 中紫色是内部未验证诊断，红色是阻塞自碰撞，橙色是环境；Official Full 与任务范围结果同时显示。

check_task_workspace.py 检查输入 SHA、全部 Index/q/XYZ/姿态、A/B 官方事实不变、Scope SHA/Binding、CSV/JSON 对应、NPZ 全数组/掩码、统计守恒、实际 HTML 坐标。静态全 21 对矩阵已视觉检查；浏览器交互仍须用户本机审阅，本轮未宣称完成浏览器交互验收。

## 用户验收：可直接复制执行

不安装依赖、不控制实体、不覆盖旧 Run。所有命令在同一终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src:$PYTHONPATH"
review_run="$(date -u +%Y%m%dT%H%M%SZ)_${RANDOM}"
build_dir="$PWD/results/c2_8/${review_run}_build"
audit_dir="$PWD/results/c2_8/${review_run}_assembly"
run100="$PWD/results/constrained_workspace/d2_${review_run}_n100"
run1000="$PWD/results/constrained_workspace/d2_${review_run}_n1000"
run10000="$PWD/results/constrained_workspace/d2_${review_run}_n10000"
examples="$PWD/results/c2_8/${review_run}_examples"
mkdir -p "$build_dir"
cmake -S cpp/collision_backend -B "$build_dir/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir/build" -j2
ctest --test-dir "$build_dir/build" --output-on-failure
export C1_BACKEND="$build_dir/build/collision_backend"

# 全机构审计、多个开度、Mimic、21 Pair 的 Contact/Distance。
.venv/bin/python -B scripts/audit_assembly.py --backend "$C1_BACKEND" --output-dir "$audit_dir"
# 单独一组 q 的完整任务范围查询与 Viewer，需使用另一个新目录：
.venv/bin/python -B scripts/audit_assembly.py --backend "$C1_BACKEND" \
  --q 0 0 0 0 0 --gripper 0.3 --output-dir "$PWD/results/c2_8/${review_run}_single"

.venv/bin/python -B scripts/evaluate_task_workspace.py --backend "$C1_BACKEND" \
  --input-prefix results/workspace_n100_seed42 --output-dir "$run100"
.venv/bin/python -B scripts/evaluate_task_workspace.py --backend "$C1_BACKEND" \
  --input-prefix results/g4_run01/experiments/workspace_n1000_seed42 --output-dir "$run1000"
# 可选 10,000 探索；本轮已运行，不能当作硬件认证：
.venv/bin/python -B scripts/evaluate_task_workspace.py --backend "$C1_BACKEND" \
  --input-prefix results/workspace_n10000_seed42 --output-dir "$run10000"

.venv/bin/python -B scripts/check_task_workspace.py --run-dir "$run100"
.venv/bin/python -B scripts/check_task_workspace.py --run-dir "$run1000"
.venv/bin/python -B scripts/check_task_workspace.py --run-dir "$run10000"
.venv/bin/python -B scripts/export_task_examples.py --backend "$C1_BACKEND" \
  --run-dir "$run1000" --output-dir "$examples"
xdg-open "$run1000/figures/workspace_comparison.html"
xdg-open "$audit_dir/figures/all_assembly_contact_matrix.html"
xdg-open "$audit_dir/figures/assembly_opening_0.3.html"
xdg-open "$examples/sample_19.html"
xdg-open "$examples/sample_8.html"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests
```

只审阅已有产物：

```bash
.venv/bin/python -B scripts/check_task_workspace.py \
  --run-dir results/constrained_workspace/d2_n1000_b1917d
xdg-open results/constrained_workspace/d2_n1000_b1917d/figures/workspace_comparison.html
xdg-open results/c2_8/assembly_allpairs_e7c61f/figures/all_assembly_contact_matrix.html
xdg-open results/c2_8/examples_9842ae/sample_2.html
xdg-open results/c2_8/examples_9842ae/sample_84.html
```

Software Implementation Status：完成（479 passed、Native 1/1）。Task-Scoped Collision Validation Status：软件正反例和范围隔离通过，实际机器人仍受 Scope A 阻塞。Gripper Internal Mechanical Validation Status：UNVERIFIED。Physical Hardware Validation Status：未执行。D2 Research Readiness：可复现的范围诊断与对照链路可用，尚无实际机器人 Task Candidate，不是完整机器人安全认证或可执行插拔方案。
