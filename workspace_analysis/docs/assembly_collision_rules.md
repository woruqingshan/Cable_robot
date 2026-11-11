# JetRover 装配碰撞规则：根因、实现和正式模型边界

本轮以 HEAD `7d8fb50d72ed` 所在提交为基线，针对两个安装界面做源码核对和少量实际 FCL 对照。
不修改官方模型、严格 SRDF v0、历史实验、FK、采样或 IDC 场景。

## 1. 实际问题

几何层回答“URDF 的三角面是否相交”；机械层回答“这个接触是否是设计允许的安装界面”；
SRDF/ACM 决定哪些整对 Link 不参加检测。三者不能混用。

当前缺少的是第二层的完整装配依据及其第三层表达，不是已经证实的 FCL 算法错误。
C3 已把 Raw 与 Semantic 查询分开，但默认仅保留五对固定装配许可，因此持续活动连接接触仍会拒绝所有配置。
软件自洽与 0% 可行率均不能证明当前模型是合理的实体碰撞模型。

附件中的反事实列表排除现已落实为独立候选 SRDF 的实际 MoveIt/FCL 查询，结果见第5节。
候选基线研究价值明确，但不能据此推定真实安全或直接替换正式规则。

## 2. system_source 提供了什么、没有提供什么

以下路径相对于只读 `/home/siyuen/Desktop/code/cable_robot/system_source/`：

| 源码 | 实际含义 |
|---|---|
| simulations/jetrover_description/urdf/common.urdf.xacro | green shell 通过 back_shell_green_joint 固定到 base_link，平移[-0.0080042,0.0001869,0]m |
| simulations/jetrover_description/urdf/connect.urdf.xacro | base_link→joint1→link1；origin[0.0251328065,0,0.0774026881]m，轴[0,0,-1]，限位±2.09rad |
| simulations/jetrover_description/urdf/arm.urdf.xacro | link1→servo_joint1(fixed,零变换)→servo_link1→joint2→link2；joint2 origin[0,0,0.0338648012]m，轴[0,1,0]，限位±2.09rad |
| 同上 joint3 | link2→joint3→link3；origin[0,0,0.1294164464]m，轴[0,1,0]，限位±2.09rad |
| driver/servo_controller/servo_controller/controller_manager.py | 发布 JointState 时，调用 pos_pulse_to_rad，不直接发布 DH 求解角 |
| driver/servo_controller/servo_controller/joint_position_controller.py | 用配置 init 和 flipped 方向，将脉宽转为 ROS JointState 弧度 |
| driver/kinematics/kinematics/transform.py | 另一套 Modified DH 参数及 pulse2angle 映射；部分角有偏置/反号 |
| driver/controller/config/init_pose.yaml、controller/init_pose.py | 默认执行 action init；备用 servo 数值不是当前已确认的安全姿态 |

因此 green/link1 是同一底座安装界面的活动相对运动，不是两个完全独立的外部物体；servo1/link2 则是直接旋转连接。
URDF 没有定义轴承接触的允许区域、机械止挡、公差或整个 Pair 可忽略的证据。
本地官方资源中未找到补全这些信息的匹配 SRDF 或机械装配 CAD。

### 角度约定是另一个必须区分的问题

对当前源码中的相同脉宽，按两条转换公式进行代数比较（单位：度）：

- DH q1 = - Controller JointState q1。
- DH q2 = Controller JointState q2 -90°。
- DH q3 = Controller JointState q3。
- DH q4 = Controller JointState q4 -90°。
- DH q5 = - Controller JointState q5。

脉宽500对应的 DH q2/q4 为-90°，发布到 ROS 的相应 JointState 为0°。
这不表示 FK 错了，而表示同一机构使用了不同零位/正方向约定。
`transform.py` 的 joint2[-180.2,0.2]° 不能直接当成 URDF joint2 的同一角度区间；
按这两段源码的映射将对应 JointState[-90.2,90.2]°，但仍需实体止挡/标定才能作为运动范围证据。
本轮不据此改 URDF 限位、不裁剪旧 q，不将厂商备用脉宽姿态认定为正常真值。

### 四个网格的实际拓扑

以 STL 顶点坐标完全相等进行合并后，边关联统计如下：

| Mesh | 三角面 | 边界边 | 非流形边 |
|---|---:|---:|---:|
| green shell |1969|61|0|
| link1 |1716|215|17|
| servo_link1 |405|93|0|
| link2 |1144|56|0|

四者均未通过封闭边关联测试；这不是完整实体体积认证，也没有做自动修补或自交认证。
FCL 支持三角面集合，不要求输入为封闭实体；所以“网格不封闭”不等于 FCL 不能工作。
但这些接触/深度值不能直接当作实物不可接受的实体穿透量。

## 3. 两个 Pair 应如何决策

| Pair | 当前支持的解释 | 仍不能证明的内容 | 本轮规则 |
|---|---|---|---|
| green shell/link1 | joint1 底部安装面；旧证据接触距轴约14.5–31.5mm、轴向0–1.5mm；MoveIt Default | 整个允许角域内 Link1 其他表面是否会撞壳体；局部接触是否设计允许 | 严格配置保留检测；候选基线临时整对忽略，标记未验证 |
| link2/servo_link1 | joint2 直接旋转连接；接触距轴约1.4–7.6mm、轴向19.4–26mm；MoveIt Adjacent | 除转轴/连接件外是否有壳体、支架、止挡干涉；实际允许角域 | 同上 |
| link2/link3 | joint3 附近接触、会随角度变化；MoveIt Adjacent | 姿态相关不自动等于危险，也不自动等于正常；整对是否可忽略未知 | 严格仍检测；候选基线忽略，单列实体审核 |

旧64接触上限已达到，这些数字是有限子集，不是完整接触面边界；本轮没有拿它们当几何切除半径。

**正式选择A：整对 SRDF 禁用。** 仅在确认整个 Pair 在明确的运动范围内都属于设计允许装配关系，
且不会因这项禁用掩盖同对 Link 的其他不允许干涉时采用。需要机械结构、真实止挡/限位、正常参考姿态和拒绝性案例。
许可应记录适用范围、URDF/几何身份、装配证据及审核人；保留对其他 Link 和全部环境物体的碰撞检测。

**正式选择B：更合适的局部碰撞表示。** 若只有轴承/转轴区域允许接触，而同对 Link 的其他部分仍可能干涉，
不要禁用整个 Pair。依据 CAD/尺寸/照片建立封闭且保留外部保护包络的复合几何或网格，明确表示安装空腔/间隙。
不能直接将整块部件替换成填满关节空腔的大包围盒，也不能任意缩小或按有限接触点删三角面。
同一 Link 下多 Collision Shape 仍共享 Link Pair 的 ACM 规则；不能一边整对禁用、一边声称其余子几何继续受检。
如需细分子体，后续必须明确子体身份/过滤语义并保留原几何对照，不是本轮重新实现 FK。

现有材料不足以在A/B中替实物作最终选择。不能把“坚持严格检测”和“直接允许所有相邻”当作唯一两种研究状态。
独立候选基线可先检验“装配语义造成系统性拒绝”这个假设。

## 4. 本轮实际代码

新增 `config/collision/semantic/standard_moveit_candidate_v0/`：

- standard_moveit_candidate_v0.srdf：从已冻结标准候选结果中精确选23对 Adjacent +6对 Default。
- candidate_source.json：原候选矩阵原始字节副本；未重新随机生成。
- semantic_manifest.json：来源/URDF/几何/SRDF哈希，逐对分类，status=candidate_baseline_unverified，usage=research_only。

不选 Never/Always，不隐式叠加历史 policy，不修改严格五对配置。当前严格五对恰好都包含在29对内。

`collision/semantic.py` 和 Native `engine.cpp` 双层验证：

1. 候选模型必须显式 opt-in；默认加载不能误用。
2. 来源必须是完整标准 MoveIt 矩阵，与当前 URDF/实际几何身份一致。
3. disabled pairs 必须恰好等于来源的 Adjacent/Default 集合；禁止夹带 Never/Always、丢 Pair 或改分类。
4. 研究决策和固定装配审核使用不同的 review status/decision。
5. Raw Empty ACM、Semantic 独立 FCL、环境全几何和真实 RobotState 继续保留。

`collision/candidate_baseline.py` + `scripts/build_candidate_semantics.py` 提供可复现、禁止覆盖的配置生成。
`collision/decision.py` 为 Evaluator 增加明确的 `configuration_decision`：

- 研究配置无阻塞→RESEARCH_CANDIDATE，不能叫正式 VALID。
- 有阻塞/不满足间隙→REJECTED；未知间隙→UNVERIFIED；异常→ERROR。
- planning_approved=false、hardware_validated=false。
- 明确指出判断依据是 semantic_*、environment 和 semantic_clearance；不使用旧 final_decision。

旧字段保留原意，旧严格 C3 查询缺少新 status 字段时仍兼容；候选查询不能省略未验证身份。
`viz/collision.py` 仅补充实验性标题/标识，复用原 Viewer。
`experiments/assembly_semantics.py` + 新 CLI 只做四个定向案例与只读 system_source 审计，没有重新开发工作空间采样。

## 5. 真正的新 ACM/FCL 对照

运行产物：`results/c3/assembly_rules_20261104_02/`。夹爪0.3rad，零间隙；同一官方几何和 S1 场景。

| 配置 | 严格自碰撞阻塞对 | 候选自碰撞阻塞对 | 环境碰撞 | 新的明确判定 |
|---|---:|---:|---|---|
|诊断零位|10|0|否|RESEARCH_CANDIDATE|
|Sample8|10|0|是|REJECTED|
|Sample73|10|0|是|REJECTED|
|Sample19|15|5|否|REJECTED|

Sample19 的 black_shell/link3 保留。每个模式独立建立 RobotModel/ACM 并实际调用 FCL，不是对旧 Contact List 做减法。
所有 Raw Pair、环境布尔、全 Link Transform、Mimic/完整关节状态、EE Transform 在两模式间一致。
诊断零位不是已确认正常实物姿态；这四例不是可行率统计。
结果支持“旧规则把部分内部装配接触作为全局阻塞”的模型级解释，不能证明29对整体物理安全。

## 6. 复核命令

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis

cmake -S cpp/collision_backend -B results/c3/assembly_user_build/build -DCMAKE_BUILD_TYPE=Release
cmake --build results/c3/assembly_user_build/build -j2
ctest --test-dir results/c3/assembly_user_build/build --output-on-failure
export C1_BACKEND="$PWD/results/c3/assembly_user_build/build/collision_backend"

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests

# 可选：重建候选文件到全新目录，不覆盖当前配置。
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/build_candidate_semantics.py \
  --candidate-matrix config/collision/semantic/standard_moveit_candidate_v0/candidate_source.json \
  --output-dir results/c3/candidate_config_user_review

# 仅四例定向查询；显式开启实验配置。
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/compare_assembly_semantics.py \
  --backend "$C1_BACKEND" \
  --candidate-srdf config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf \
  --allow-candidate-baseline \
  --output-dir results/c3/assembly_user_review

xdg-open results/c3/assembly_user_review/figures/diagnostic_zero_NOT_NORMAL_REFERENCE.html
xdg-open results/c3/assembly_user_review/figures/sample19.html
```

候选来源被固定为既有矩阵，所以本轮不再调用随机候选生成器，也没有100/1,000/10,000工作空间重跑。
正式模型仍需真实正常静止配置及关节安装结构证据。无需先改算法才能确认这些事实。

## 7. 官方资料与证据边界

- [MoveIt Humble Setup Assistant](https://moveit.picknik.ai/humble/doc/examples/setup_assistant/setup_assistant_tutorial.html)：标准矩阵的候选分类和人工调整入口。
- [MoveIt Humble URDF/SRDF](https://moveit.picknik.ai/humble/doc/examples/urdf_srdf/urdf_srdf_tutorial.html)：几何与语义职责及碰撞模型。
- [FCL 官方仓库](https://github.com/flexible-collision-library/fcl)：三角面模型、接触/距离查询，输入不要求特定拓扑。

模型解析/FCL接口已实现，真实机械接触规则仍PARTIAL/PENDING。工程可运行性不替代机械审核。

## 8. 最终验证

本轮完整 Python 回归：**528 passed，1 warning，0 skipped**；Native CTest：**1/1 PASS**。
既有506项测试保留，新增22项参数化/集成用例，覆盖 opt-in、来源篡改、禁止夹带Never、实际候选 FCL、
判定接口、异常状态、旧严格响应兼容，以及厂商角度约定/网格审计。
另已用本机前一版C3二进制实测新客户端的严格会话兼容。

本轮开始时纳入保护的630项结果/严格配置文件哈希全部不变。
审计日志与校验位于 `results/c3/assembly_validation_20261104_01/`。
没有新工作空间采样、随机候选重生成、实体机器人操作、系统依赖安装或Git提交。
Viewer HTML可直接打开；交互显示与机械装配的实体审核仍由用户验收。
