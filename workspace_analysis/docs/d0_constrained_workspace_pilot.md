# D0 constrained workspace pilot — pending user review

本轮直接复用既有 G2/G4 数据，不重新采样、不按 EE XYZ 去重、不裁掉负 Z。三种策略使用完全相同 q、S1 场景、官方 24 个 Collision Mesh / 21,568 三角形、r_joint=0.3 rad、零间隙阈值。

## 通用 API

```python
from workspace_analysis.collision.evaluator import Evaluator
from workspace_analysis.collision.policy import load_policy

# model / scene 仍由原 load_model / load_scene 得到；scene 可独立传入。
policy = load_policy('config/collision/assembly_policy_v1.yaml', model)
with Evaluator(model, scene, policy, backend_path,
               gripper=0.3,
               clearance={'environment_m': 0.0, 'self_m': 0.0}) as evaluator:
    rows = evaluator.evaluate(q, start_index=0, batch_size=64)  # q[N,5], m/rad
    identity = evaluator.identity
```

每行保留 sample_index、原始 q、Native EE XYZ/3×3 姿态、Raw/Effective Self、Environment、实际 filtered pairs、remaining pairs、clearance、模型判定、verification_status、final_decision、查询时延及完整身份。batch_size 为有序分块；当前 Native 仍每行一次持久 JSONL 查询，没有伪装成并行/向量化求解。非法 q 返回 ERROR 行，顺序不丢失；传输或身份故障后后续行明确 ERROR，不静默重连或换模型。

`model_scope_collision_free` 只针对当前 URDF/ACM/场景/夹爪/阈值。原始自碰撞可以被显式 ACM 过滤；环境碰撞不受 Self ACM 影响。正间隙使用真实 Native 距离，距离不可用时返回 `CLEARANCE_UNSUPPORTED`，未发生碰撞的模型判定为 null。未发生几何碰撞但正阈值不足时模型判定 false、Final 仍 UNVERIFIED，避免把间隙不足伪装成已穿透。任何模型可行配置的 Final 仍为 UNVERIFIED，本轮没有 CLEAR 实物安全结论。

## 运行和文件身份

- 100：`results/constrained_workspace/n100_b9c0ea5c3f/`
- 1,000：`results/constrained_workspace/n1000_92762fd278/`
- 10,000 探索：`results/constrained_workspace/n10000_208b7e6e1c/`
- 扫描、Clearance 真值、历史 C2 回放：`results/c2_6/refinement_52d15f1e88/`
- 构建中间物与日志：`results/c2_6/build_f99e193af6/`，不作为正式研究数据。

每个 pilot 具有 manifest、per_sample.csv、evaluations.json、summary.json、三份策略 NPZ 和三份 HTML。NPZ 保存原 G2 全部数组及 model_scope_free/self_rejected/environment_rejected/unknown_unverified 四个配置掩码；它是 D0 产物，**不是被改写的 G2 NPZ/JSON 对，也不应交给 G2 load_results**。原 G2 Pair 通过原读取器验证。按策略命名防止三个结果互相覆盖；已有输出目录一律拒绝，即使只剩 .incomplete 也不复用。

manifest 保存原 G2 NPZ/JSON SHA、Git HEAD、URDF/几何/场景/策略/SRDF/Backend SHA、Frame/单位、夹爪、阈值、时间戳及模型范围。10,000 的 manifest 还保存工作树状态与 Python 运行模块哈希；前两次运行的补充追溯记录在 integrity.json。创建新目录可安全复跑，时延和时间戳预期不同。

统一身份：URDF `5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`；Geometry `4858b34b1ff62f8af791809701888028637650c2372dc5f3978a1d5e606b216d`；Scene `d83e76d09f7e69fa0c01e753f0cefb3128dd520fb0292f1fd0b4838f66a5ab3b`。各策略规范 SHA 见 manifest，v0 未覆盖。

## 实际统计

| 样本数 | 策略 | Raw Self | Effective Self | Environment / Both | 模型可行 / 比例 | Final C / U / E | 均值 / P95 [ms] |
|---:|---|---:|---:|---:|---:|---|---:|
| 100 | baseline | 100 | 100 | 2 / 2 | 0 / 0% | 100 / 0 / 0 | 11.94 / 17.11 |
| 100 | assembly_policy_v0 | 100 | 100 | 2 / 2 | 0 / 0% | 100 / 0 / 0 | 11.43 / 16.42 |
| 100 | assembly_policy_v1 | 100 | 100 | 2 / 2 | 0 / 0% | 100 / 0 / 0 | 11.11 / 16.17 |
| 1,000 | baseline | 1,000 | 1,000 | 21 / 21 | 0 / 0% | 1000 / 0 / 0 | 12.58 / 17.26 |
| 1,000 | assembly_policy_v0 | 1,000 | 1,000 | 21 / 21 | 0 / 0% | 1000 / 0 / 0 | 12.35 / 17.14 |
| 1,000 | assembly_policy_v1 | 1,000 | 1,000 | 21 / 21 | 0 / 0% | 1000 / 0 / 0 | 11.95 / 16.69 |
| 10,000 | baseline | 10,000 | 10,000 | 232 / 232 | 0 / 0% | 10000 / 0 / 0 | 12.77 / 17.82 |
| 10,000 | assembly_policy_v0 | 10,000 | 10,000 | 232 / 232 | 0 / 0% | 10000 / 0 / 0 | 12.27 / 17.28 |
| 10,000 | assembly_policy_v1 | 10,000 | 10,000 | 232 / 232 | 0 / 0% | 10000 / 0 / 0 | 11.94 / 16.91 |

所有组 CLEAR=0，碰撞拒绝率=100%。原始命中体素数分别 99、960、7,554；负 Z 配置分别完整保留 14、174、1,840 条。10,000 是探索性模型实验，没有实物安全认证。

所有比例均为配置级的 Model-Scope Feasible Ratio / Collision Rejection Ratio，不是 IDC 插拔成功率、体积可达率或实物安全概率。self/environment 集合允许重叠，拒绝并集用 self+environment-both，未重复计数。Final 的 UNVERIFIED 数不是覆盖缺口的完整计数：已碰撞的样本 Final 为 COLLIDING，但它们仍带有未实物认证的 scope。

原始空间和候选空间体素命中数采用原 G4 VoxelGrid：origin=(-0.6,-0.6,-0.4)、upper exclusive=(0.6,0.6,0.8)、voxel=0.02 m。只报告采样命中，不推断未命中体素不可达。本轮候选命中体素为 0；配置级与体素/位置级没有混用。

## 阻塞原因与可视化

v1 并非未生效：两对固定装配接触确实从 Effective 集合中移除，但 green shell/link1、link2/servo_link1、多对夹爪内部接触仍覆盖所有试验配置。link2/link3 的定向扫描有状态变化，不能无条件允许整对。各 Pair 的计数可重叠，分类频率和来源见 blocker_analysis.json。81 点定向探针没有找到不受这些限制的配置；没有用新 ACM 掩盖剩余碰撞。

三份策略 HTML 各包含 Original、Model-scope candidates、Self rejected、Environment rejected、Unknown/unverified 图例。没有候选时绿色 Trace 真正为空。场景参考 base_link，等比例 XYZ；原始工作空间默认 legendonly，可手动显示。Sample 8/73 机器人查看器使用同一 S1、v1、对应原 q、夹爪 0.3，显示完整官方碰撞几何和真实 FCL 接触，供核对拒绝区域，不能将这些图的 EE 散点当成机器人几何。

数据核验脚本逐项比对 HTML 的真实 x/y/z 数组、CSV/JSON/NPZ 来源 q 顺序、集合掩码与汇总。浏览器工具禁止本机 file://，本轮未取得新图的浏览器视觉验收，仍需用户打开审阅。

## 可复制的用户验收命令

全部从同一终端执行。以下使用唯一运行目录，不覆盖本轮产物；不启动 ROS 控制节点或真实机器人。

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src:$PYTHONPATH"

review_run="$(date -u +%Y%m%dT%H%M%SZ)_${RANDOM}"
build_dir="$PWD/results/c2_6/${review_run}_build"
probe_dir="$PWD/results/c2_6/${review_run}_probes"
run100="$PWD/results/constrained_workspace/${review_run}_n100"
run1000="$PWD/results/constrained_workspace/${review_run}_n1000"
run10000="$PWD/results/constrained_workspace/${review_run}_n10000"
mkdir -p "$build_dir"
cmake -S cpp/collision_backend -B "$build_dir/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build_dir/build" -j2
ctest --test-dir "$build_dir/build" --output-on-failure
export C1_BACKEND="$build_dir/build/collision_backend"
"$build_dir/build/native_tests" >"$build_dir/clearance_truth.log" 2>&1

.venv/bin/python -B scripts/refine_collision.py \
  --backend "$C1_BACKEND" --output-dir "$probe_dir"

.venv/bin/python -B scripts/evaluate_workspace.py \
  --backend "$C1_BACKEND" --gripper 0.3 --batch-size 64 \
  --input-prefix results/workspace_n100_seed42 --output-dir "$run100"
.venv/bin/python -B scripts/evaluate_workspace.py \
  --backend "$C1_BACKEND" --gripper 0.3 --batch-size 64 \
  --input-prefix results/g4_run01/experiments/workspace_n1000_seed42 --output-dir "$run1000"
# 可选探索，不是经过实物验证的正式安全实验：
.venv/bin/python -B scripts/evaluate_workspace.py \
  --backend "$C1_BACKEND" --gripper 0.3 --batch-size 64 \
  --input-prefix results/workspace_n10000_seed42 --output-dir "$run10000"

.venv/bin/python -B scripts/report_collision_refinement.py \
  --refinement-dir "$probe_dir" --pilot-dir "$run1000"
.venv/bin/python -B scripts/check_constrained_run.py --run-dir "$run100"
.venv/bin/python -B scripts/check_constrained_run.py --run-dir "$run1000"
.venv/bin/python -B scripts/check_constrained_run.py --run-dir "$run10000"

xdg-open "$run1000/figures/assembly_policy_v1.html"
xdg-open "$probe_dir/figures/sample8_v1.html"
xdg-open "$probe_dir/figures/sample73_v1.html"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests
```

不重跑即可检查本轮已有 1,000 产物：

```bash
.venv/bin/python -B scripts/check_constrained_run.py \
  --run-dir results/constrained_workspace/n1000_92762fd278
xdg-open results/constrained_workspace/n1000_92762fd278/figures/assembly_policy_v1.html
xdg-open results/c2_6/refinement_52d15f1e88/figures/sample8_v1.html
xdg-open results/c2_6/refinement_52d15f1e88/figures/sample73_v1.html
```

Software Validation 已通过；模型级活动连接/夹爪几何审核为 PARTIAL；Physical Hardware Validation 未执行。下一步应针对现有剩余装配/活动区域进行有证据的局部几何研究，不能把 0% 可行率解释为真实机械臂没有工作空间，不能用频率自动加入 ACM。本轮未开展 IK、规划、RJ45 插入或 VLA。


追溯补充：本轮 pilot 使用的精确 Native 可执行文件保存在
`results/c2_6/build_f99e193af6/collision_backend_pilot_snapshot`，SHA256 为
`20d16045bfa9d8c9fb327d2c138b6c5ef4a436e85e646d3a06d0c4f67c3be4b3`。
最终 Native 重建修正了旧 `unverified_reasons` 中已过时的 clearance 未实现措辞；数值查询不变，最终二进制重新通过全量 452 项回归与 Native CTest。
两种二进制身份及 515 项历史文件保护核对均记录在 refinement 的 `integrity.json`。
前两次 100/1,000 运行未保留独立的 Python 源码快照；它们已有输入、模型、场景、策略、精确 Native、阈值身份，且全部实际数值与 Trace 已再次逐行核验。
索引为 `results/_index/c26_d0_runs.json`，未改写历史索引或任何旧实验结果。
