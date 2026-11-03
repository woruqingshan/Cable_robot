# P2.3-C2 Collision Validation — PARTIAL, Pending User Review

本轮完成显式 provisional ACM、Raw/Effective 分离、完整接触对和覆盖清点、同 100 行对照及轻量查看器。
**没有达到 Validated Collision Model for a Declared Scope**：71 个接触对仍待审核，LiDAR Overlay/实物覆盖及距离间隙未验证。
不把模型/政策机制的单测 PASS 解释为物理安全认证。不进入 D 或 10,000 行筛选。

## 状态与工程变更

起始分支 feat/workspace-planning，HEAD b883199（C1 已提交）。保留原 S1 文档修改和 docs/develop/。
仅扩展 cpp/collision_backend 的 Engine/main/native tests、collision/client；增加 policy.py、coverage.py、
config/collision/baseline.yaml、assembly_policy_v0.yaml、scripts/audit_collision.py、tests/test_collision_policy.py 和本文等三个 C2 文档。
不改 URDF/Mesh、G1–G4、S1 YAML/Schema、C1 已保存诊断或可视化。

Protocol version 1 是向后兼容的加法扩展，新增 POLICY 操作。C1 的 self_collision/collision_detected/decision
继续表示原始 Empty-ACM 查询语义；C2 使用新增 raw_self_collision、effective_self_collision、
effective_decision、effective_self_pairs、filtered_pairs、applied_policy。不能把 C1 decision 当 C2 Effective decision。
旧 Native 不支持 POLICY 时明确 ERROR，不静默回退。backend 二进制 SHA 变化，C1 历史身份不改写。

Native 按 policy canonical JSON 的 SHA 验证机器人身份、两个 Link 是否存在、理由/证据/审核状态、
显式 pair 及 unsupported clearance 后创建独立 self ACM。同会话可切回 baseline，Raw 继续使用空 ACM，
Effective 使用显式 ACM，Environment 始终空 ACM。每次独立 CollisionResult.clear。
Python 规范化同向/反向重复；冲突重复、未知 Link、缺失证据、unresolved 与 allowed 冲突拒绝。
不接受 validated 声明或有工具的 scope，绝不因为自碰撞数量少了就发 CLEAR。

## 完整审计与范围

见 [74 对完整审计](c2_collision_pair_audit.md) 和 [逐 Link 覆盖矩阵](c2_geometry_coverage.md)。
类别 A=5、B=6、C=58、D=0（未充分排查）、E=5，属于模型初步分类。
三对候选固定装配：back_shell_black_link/back_shell_green_link、gripper_link/link5、link1/servo_link1。
明确写在 YAML，不通过程序为所有固定/相邻/夹爪/100% 接触生成豁免。
其他 71 对全部保持检测，包括 link2/link3 和活动臂/底盘碰撞。

## Clearance 与四态

本机 CollisionEnv 有 distanceSelf/distanceRobot 和 padding API，DistanceRequest 支持 signed distance；
本轮只审查头文件，没有完成数值距离真值测试，因此没有启用或宣称最小间隙满足。
策略支持且记录的仅为 environment_m=0、self_m=0、contact_mode=FCL_raw_boolean。
正间隙、非有限/负间隙、未知 contact epsilon 模式明确拒绝；没有用统一 padding 替代 Self Clearance。
Scene 的 contact_epsilon_m=1e-7 不参与本轮 FCL 判据，这一未完成项不能解释为已应用的安全裕量。

Effective Self 或 Environment 碰撞 → COLLIDING；无拒绝性碰撞但覆盖/ACM未验证 → UNVERIFIED；
输入/运行错误 → ERROR。CLEAR 保留语义，本轮完全不产生。
政策 canonical hash 包含范围、审核、全部 pair 理由/证据/未决项、clearance。每次返回 URDF、Mesh、Scene、
policy、backend 二进制身份和完整关节状态，含 r_joint/mimic；Overlay 未启用，没有伪造 overlay 身份。
修改活动策略后客户端只接受相同模型/几何/scene/backend 下的精确新 policy hash。

## 同一 100 行对照

| 指标 | Empty | Assembly v0 |
|---|---:|---:|
| Total | 100 | 100 |
| Raw Self | 100 | 100 |
| Effective Self | 100 | 100 |
| Environment | 2 | 2 |
| Both | 2 | 2 |
| CLEAR | 0 | 0 |
| COLLIDING | 100 | 100 |
| UNVERIFIED | 0 | 0 |
| ERROR | 0 | 0 |
| 平均查询 [s] | 0.005486410199999999 | 0.00527519913 |

环境索引两组均为 8、73；Raw 逐行与 C1 相同。3 对固定候选被过滤，最终分类变化的索引为空，
因为仍有未审核装配接触。pair_audit.json / queries.json / per_sample.csv 给出完整 q、Raw/Effective/filtered，
comparison.csv 保留全部原索引。没有重新采样或过滤负 Z（原 14 行保留），没有按 EE 坐标合并。
这不是筛选得到的无碰撞工作空间；不能为改善数字扩大 ACM。

## 验证

Python 全量：437 passed, 1 warning, 0 skipped，13.53 s；原 429 项保留。Native CTest 1/1 PASS，
聚合 C1 全部断言及新增 A-B 过滤而 A-C 保留、环境不被放过、未知 ACM Link 拒绝。
Python 新测规范化/身份变化/unsupported clearance/tool/validated claim 拒绝、LiDAR 候选身份、
8/73 真后端结果、切回 baseline 不串用状态、不同 gripper Link 变换而 EE 不变。
C1 原有 FK/整树/38 Box 正负旋转及资源/协议/进程测试继续通过。

未完成的测试明确包括：LiDAR 实体 Overlay 的 FCL 覆盖测试、距离/epsilon 数值真值、实物装配接触认证。
未用 Mock 的通过代替这些证据。Native 真实模型支持可配置策略，不代表所有分类证据已完备。

## 可复制命令

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
C2_ID="$(.venv/bin/python -c 'import uuid; print(uuid.uuid4().hex)')"
C2_BUILD="$PWD/results/c2/${C2_ID}_build"
cmake -S cpp/collision_backend -B "$C2_BUILD/build" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$C2_BUILD/build" -j2
ctest --test-dir "$C2_BUILD/build" --output-on-failure
export C1_BACKEND="$C2_BUILD/build/collision_backend"
# 审计、100 行两策略对照和 viewer 一次生成；新输出目录必须不存在。
.venv/bin/python -B scripts/audit_collision.py --backend "$C1_BACKEND" --view-sample 8
# 要为样本 73 再生成一份独立运行，可指定 --view-sample 73；已有本轮 viewer 链接见下。
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests
```

viewer_<index>.html 显示 Plotly 场景和下方数值/策略/接触表。viewer 依赖同目录 sample_<index>.html，
不是独立可移动的单文件。样本 8 图中 Link markers 仅为原点，不代表碰撞体表面；截图不能代替 FCL。

本轮运行：

- Build: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2/20261103T110913Z_build_4e223d9e`
- Diagnostic: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2/20261103T110913Z_diagnostic_dcddb9f7`
- [样本8](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2/20261103T110913Z_diagnostic_dcddb9f7/viewer_8.html)
- [样本73](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c2/20261103T110913Z_diagnostic_dcddb9f7/viewer_73.html)

## Research readiness

现在可以在明确的模型及 provisional 策略下重现 Raw/Effective 静态查询，并定位残留接触。
不能据此认证有效配置集合；两份策略仍全部碰撞。进入 D 前应优先完成固定/活动连接接触区域审核、
剩余夹爪接触与 r_joint 范围、LiDAR 包络和工具覆盖、明确的距离/epsilon 真值验证。
后续若用户授权纯诊断性 D 探索，必须保留这些未验证声明；本轮不自动开展。

P2.3-C2 Collision Validation — PARTIAL — Pending User Review.
