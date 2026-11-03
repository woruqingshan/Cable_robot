# C2.7 Collision Geometry Refinement — software complete, model validation PARTIAL

起始分支 `feat/workspace-planning`，HEAD `2e6e2ecfbd18e28b07bf06f0bfca30d933c71723`。起始未跟踪项只有 `docs/develop/`、`workspace_analysis/results.zip`，均保留。实际起始回归 452 passed / 1 warning；最终 463 passed / 1 warning / 0 skipped；Native CTest 1/1 PASS。

## 方法与实现

新增独立配置：

- `config/collision/geometry_profiles/official_full.yaml`：官方完整几何，未声称硬件覆盖已认证。
- `config/collision/geometry_profiles/refined_assembly_v0.yaml`：**diagnostic_only**，六对限定上下文的局部关节三角面排除实验。

没有修改官方 URDF、Xacro、Mesh，没有新增 ACM v2。baseline/v0/v1 原内容保持不变；本轮全部对照固定采用 ACM v1 的五对固定装配许可。

`collision/geometry_profile.py` 从官方 Mesh 保留原始顶点和三角形；只排除三个顶点均严格位于声明关节轴圆柱内的完整三角形。圆柱为凸集，完全位于其内的三角形才可排除；跨边界三角形保留，不移动顶点、不封盖切口、不替换整对 Link。区域取自 URDF 关节轴与官方 STL 的近轴凸台/法兰尺寸，参数在实验前固定，不依据 G2 通过率优化。

**该表示不是保守封闭实体代理。** 删除的三角面可能属于真实部件；当前只研究内部近轴面的模型敏感性，不能认定所有删除面都是正常装配，更不能推广为硬件安全几何。

| 派生 Link | 原始面数 | 保留面数 |
|---|---:|---:|
| link1 | 1,716 | 604 |
| link2 | 1,144 | 818 |
| l_in_link | 260 | 142 |
| r_in_link | 260 | 138 |
| l_out_link | 1,601 | 1,549 |
| r_out_link | 1,608 | 1,556 |

原始 24 Mesh / 21,568 面；派生资源仍为 24 Mesh，共 19,786 面。高三角面数量不等于体积，不能按面数解释缩小比例。排除区域、轴、原/派生 URI、资源 SHA、排除面索引都保存在 `geometry_refinement.json`。

Native 使用两个独立 MoveIt RobotModel/PlanningScene，同一官方运动学树：原始引擎保留完整几何，派生引擎加载只替换指定 Collision Mesh URI 的内存 URDF。Native 严格比较 Joint 类型、父子连接、origin、axis、限位、Mimic，逐查询核对所有 Link 变换。不存在第二套 FK。

组合查询语义：

1. 六个声明 Target Pair 使用派生面的完整 FCL 查询。
2. 其余全部 Self Pair 使用官方完整面，**包括 link2/link3 和三对未处理的夹爪 Pair**。
3. 所有 Robot/Environment Collision 和 Environment Clearance 均使用官方完整几何。
4. Self Clearance 对非 Target Pair 的官方距离和 Target Pair 的派生距离取最小值，遵循显式 ACM。
5. 原始/派生 Pair、原始/组合 Raw Self、Source/Effective Geometry、Profile/Policy 具有独立身份。

因此它不是一个全局替换成缩小机器人外形的模型；也不是按几个接触点屏蔽整对。FCL 实际检查剩余全部三角面，同一 Target Pair 的远端干涉仍会拒绝。局部切除只适用于声明 Pair，其余外表面检测保持完整。六个 Target Pair 精确列表见 YAML。

## 多接触点和逐对距离

`geometry_audit` 限制每对 1–512 接触点、总数不超过 50,000；默认审计 32 / 20,000。返回真实 Native Contact、两个 Link 局部位置、关节零角轴坐标位置、深度、全局限制状态及达到每对上限的名单。`complete_contact_surface` **始终 false**，即使未达到限制也不作为完整接触面证明。

可请求不超过 32 个指定 Pair 的独立 FCL Collision Boolean / Minimum Distance。距离使用 GST_INDEP，重叠映射为无符号 0；Raw Pair 查询明确不应用 ACM。它与有限接触点的数目无关，不将穿透深度当成负距离。最终源代码同时修正旧 Native Audit 的 `clearance_implemented` 标志为 true。

最终多点扫描：`results/c2_7/audit_final_638aa7/`。85 个查询由零位参考、joint1/2/3 各 17 点、9 个合法夹爪状态 × 零位/8/73、6 个未用于区域定义的 G2 留出姿态组成。真实接触统计：

| Pair | Official 有接触姿态 / 85 | Refined / 85 | 处理依据与限制 |
|---|---:|---:|---|
| green shell / link1 | 85 | 0 | joint1 近轴底法兰接触，局部排除有模型解释；实物尚未确认 |
| link2 / servo_link1 | 85 | 0 | joint2 凸台/轴承侧近轴区域；不豁免该 Pair 的远端表面 |
| gripper / l_in | 85 | 6 | 近轴环可解释部分接触；极端开度仍保留其他相交 |
| gripper / r_in | 85 | 79 | 接触延伸超出近轴环，不能扩大区域强制消除 |
| l_in / l_out | 85 | 85 | 近轴和远离轴区域均有证据，当前切除不足以合理消除 |
| l_link / l_out | 85 | 85 | URDF 为两分支路径，可能涉及实体闭环联接；未处理 |
| l_link / r_link | 85 | 85 | 官方 Mesh 可见啮合齿形，接触与齿轮啮合相符；未据此整体允许 |
| r_in / r_out | 85 | 85 | 与左侧相同，仍有远轴接触 |
| r_link / r_out | 85 | 85 | 实体闭环候选，两分支 Mimic 不能证明真实闭环约束；未处理 |
| link2 / link3 | 83 | 83 | 姿态依赖，始终用官方完整 Pair，未扩大豁免 |

关节路径、局部坐标范围、每个姿态的 q/夹爪、有限接触点及实际深度在 `contact_region_audit.json` 和 `joint_gripper_probes.json`。还输出全部官方 Mesh 的顶点连接分量、边界边、非流形边和退化面统计。它们是表面拓扑诊断，不是实体/装配安全证明。

零位 q=[0,0,0,0,0]、r_joint=0.3 的逐对距离：green/link1 从 0 变为 0.000324726 m；link2/servo_link1 从 0 变为 0.002970036 m；gripper/l_in 从 0 变为 0.000406237 m。其他重点 Pair 仍为 0。**这些是诊断几何的条件距离，不是实物间隙测量。**

当前 r_in 残余点相对轴径向可达约 25.8 mm，l/r in-out 约 25.9 mm；不局限于 4.5 mm 近轴环，不能仅靠扩大圆柱处理。l/r_link 的代表点位于约 13.2–15.5 mm 齿形区域；l_link/l_out、r_link/r_out 则接近驱动杆的远端区域。有限接触位置和深度是 FCL 代表性诊断数据，不等于完整接触流形或精准实体表面边界。

## 正反例和回归

新增 11 项 Python 测试，保留原 452 项与原断言。全部通过：

- 两个独立 Cube 组件构成的合成 Mesh：近轴重叠被排除，90° 时保留面之间距离 0.85 m；转回 0°，同一 Pair 的远端重叠仍检出。
- 添加非相邻第三部件，其真实碰撞仍检出。
- 环境 Box 穿过已排除的近轴表面，原完整环境查询仍能检出，证明没有把孔洞带入环境防护。
- 非零 Collision Origin、非标准 RPY、非均匀 Scale 下，保留顶点与既有 Renderer 完全对应。
- 非法 Profile Hash、改变限位、未声明 Mesh 改动、可能缺失 Pair 覆盖的全局截断均拒绝。
- JetRover 留出 q、不同夹爪/Mimic、全部 Link FK、非 Target Pair 和环境距离不变。
- 真实 100 配置的 D1 封装、Robot-Only、原数组和统计守恒，以及 CLI 启动检查。

参考零位仍是 **COLLIDING / 正常装配候选待验证**。真实运动干涉示例使用 G2 Sample 19，`back_shell_black_link/link3` 保留；环境例为 8/73，两模型都保留原完整机柜碰撞。配对完整机器人查看器：`results/c2_7/paired_viewers_172aec/`。

查看器复用 C2.5：完整机器人默认显示官方完整环境/非 Target 表面；不能把一个全局缩小外形当作本次组合模型。实际派生表面在十份 `local_*.html` 中提供 Original / Derived 切换，含真实接触、轴和坐标。官方齿形和局部改动的静态科学渲染已经检查，位于早期审计 `audit_824aca/figures/geometry_preview.png`；该早期 Run 保留，最终审计另外输出，没有覆盖。

## 身份、文件和未解决事项

- 修改 C++ `engine.hpp/engine.cpp/main.cpp/native_tests.cpp`，扩展 INIT Profile 和 QUERY Audit；协议仍为版本 1，旧参数行为保留。
- 修改 `collision/client.py/protocol.py/evaluator.py`；新增 `geometry_profile.py/geometry_audit.py`。
- 修改 `experiments/constrained_workspace.py`，仅增加可选 GeometryProfile、SceneSnapshot；新增 `geometry_workspace.py`。
- 新增 `viz/geometry_audit.py`，只消费数据，由调用方传入坐标；不反向导入 collision，不放宽原依赖测试。
- 新增 `audit_geometry.py/evaluate_geometry_workspace.py/check_geometry_workspace.py/export_geometry_viewers.py/render_geometry_preview.py`。
- 新增 Profile YAML、`tests/test_geometry_profile.py` 与两份文档，README 增加入口。

身份保持 Source URDF、Source Geometry、Effective URDF、Effective Geometry、组合 Geometry、Geometry Profile、Policy、Scene 和 Backend 独立。Python 不绕过源资源校验，并递归校验实际 Native 派生模型资源。Profile canonical identity 包含实际资产 URI，因此不同输出目录可能有不同身份；资源 SHA 和保留三角面仍可一致。`materialized_profile` 可从原记录重载相同身份，避免重写资产。

D1 使用的精确 Binary/源码快照保存在 `results/c2_7/build_73e5e3/pilot_snapshot/`。最终构建仅追加审计 Pair Distance，并纠正能力标志；默认查询数值逻辑不变，重跑全部测试及配对 Viewer，身份差异明确记录。

Software Implementation：完成。Model-Level Geometry Validation：**PARTIAL，诊断 Profile 可运行，未成为正式安全几何**。Physical Hardware Validation：未执行。仍缺 LiDAR Collision、RJ45/Tool/TCP、Cable、实物装配和真实夹爪闭环约束/标定。没有 ACM v2，因为不能证明整对活动部件在全部声明姿态均可安全忽略。

用户可复制命令和全部 D1 发现见 [D1 文档](d1_workspace_validation.md)。
