# JetRover 候选碰撞模型验证与研究工作空间恢复

状态：**RESEARCH_ONLY — 软件验证完成，机械整对豁免审核 PARTIAL，正常实物参考姿态 PENDING。**
基线 HEAD：`add16d8`；本轮没有修改两套 SRDF、官方 URDF/Mesh、FK、采样器或 S1 场景。

## 实际实验结果

固定条件：官方完整24个 Collision Mesh、全部38个S1障碍、base_link→end_effector_link、夹爪r_joint=0.3rad，
self/environment附加间隙均0m。两套SRDF分别初始化独立MoveIt/FCL进程；使用独立Semantic查询，不做Raw列表减法。

| 原始样本数 | SRDF | Raw自碰撞 | Semantic自碰撞 | 环境碰撞 | 同时碰撞 | 模型通过/研究候选 | 拒绝 | 候选率 |
|---|---|---:|---:|---:|---:|---:|---:|---:|
|100|strict v0|100|100|2|2|0|100|0%|
|100|standard candidate v0|100|24|2|0|74|26|74%|
|1,000|strict v0|1,000|1,000|21|21|0|1,000|0%|
|1,000|standard candidate v0|1,000|248|21|2|733|267|73.3%|

四个条件的unknown/error均0。候选模型没有新增拒绝；恢复的Sample Index分别完整保存在comparison.json/CSV中。
自碰撞和环境碰撞可以重叠，不能直接相加当作总拒绝数。

输入使用原有G2的100样本和G4的1,000样本。实际核对发现100样本恰为1,000样本的前缀，
因此这不是两次独立随机重复试验，不据此提供独立置信区间。
没有重新采样、按XYZ去重或删除负Z点。所有原始q/position/rotation/quaternion及原索引均保留并核验。

1,000样本的主要剩余自碰撞Pair（每对出现的配置数，可相互重叠）：

| Pair | 配置数 |
|---|---:|
|back_shell_black_link / link3|72|
|base_link / link4|61|
|back_shell_green_link / link3|59|
|back_shell_green_link / link4|49|
|base_link / gripper_link|47|
|back_shell_green_link / link2|47|
|base_link / link3|44|

这表明非相邻折叠干涉及IDC碰撞仍能拒绝配置。并不能证明候选SRDF禁用的29对没有隐藏其他真实干涉。

## 装配与角度验证

复用既有只读system_source审计，新增逐项29对审计和定向实际FCL查询：

- 29对均记录URDF Joint Path、活动关节/Mimic、候选来源分类、严格配置是否已允许、运行中接触计数及机械审核状态。
- 原严格配置5对固定装配许可保持原样；其他24项不升级为正式许可。
- joint1：green shell经固定关节连到base_link，link1通过joint1旋转；两种锚定姿态各7角度，Raw接触14/14。
- joint2：servo_link1固定于link1，link2通过joint2直接连接；Raw接触14/14。
- joint3：link2/link3为直接活动连接，Raw接触12/14，在+2.09rad两种锚定姿态下不接触。它是“姿态相关关节界面接触”，不能直接叫作危险干涉，也不能据此证明整对安全。
- 共42个扫描状态；每对最多64接触、全局20,000接触，保留截断信息。有限接触点不代表完整接触面。
- Sample8/73环境正例、Sample19黑壳/link3正例在夹爪0、0.3、1rad下均保留，共9项正例检查。

当前证据解释了“安装面附近三角面相交”的模型来源，但没有独立证明整个Pair在所有实物允许姿态中均可忽略。
Link2/link3在逐对审计中单列HIGH_PRIORITY_STATE_DEPENDENT_JOINT_INTERFACE；当前候选配置仍是研究假设。

同一脉宽在厂商代码中的角度关系为：DH q1/q5与Controller JointState反号；DH q2/q4=Controller值-90°；q3一致。
Controller通过pos_pulse_to_rad发布ROS JointState。URDF仍是本项目FK、RobotState和限位的权威输入；
DH范围不直接覆盖URDF。上述代数对应不代表实体标定完成。

额外查阅了项目已有insert.yaml、pull_out.yaml及保存逻辑。找到4条可转换的记录（含跨文件重复READY），
其q/夹爪值通过当前URDF限位检查，候选模型下无自碰撞及当前S1环境碰撞。
例如CABLE_HOLD_READY的源约定转换结果：

```text
q = [0, -1.1100294043, 2.0315632493, 1.1728612573, 0] rad
r_joint = 0.8377580410 rad
```

它们只作为“待确认的示教记录”，不用于改变100/1,000实验的q或夹爪状态。
记录名、时间戳、脉宽范围/步长安全检查均不是实体无碰撞确认；保存pose的Frame/TCP也未与当前EE确认。
因此可信正常姿态仍PENDING；没有将零位认定为安全。

## 间隙与适用范围

候选模型的自间距全样本中位数：100样本0.843mm，1,000样本0.820mm；包含碰撞配置的零间距。
本次模型通过的定义使用零附加间隙，不是满足实物公差或计划间隙的证明。
最大自间距代表例：N100 Sample33约1.011mm，N1000 Sample437约1.020mm，最近受检Pair均为link4/link5。
不因其靠近而继续添加ACM豁免。改变间隙要求会改变候选集合，需另行声明和验证。

输出仍缺少：可信的正常姿态/标定、29对机械整对审核、LiDAR碰撞覆盖、RJ45工具/TCP与线缆、实物尺寸公差。
当前结果是“当前几何+实验性SRDF+S1+固定夹爪+零间隙”的配置级模型结果。
不保证网口朝向/接近方向可满足，不保证IK存在，不保证候选之间有连续无碰撞轨迹，也不代表实物安全。

## 输出位置与查看

结果：`results/constrained_workspace/research_recovery_20261104_01/`。

- assembly_validation.json：源代码/关节审计、42状态扫描、9项正例、待确认示教记录与清理证据。
- pair_review_29.json：全部29对的关系、风险和100/1,000原始接触/阻塞计数。
- summary.json、manifest.json：对照统计及HEAD、源码、模型、SRDF、ACM、场景、后端、夹爪/间隙/输入哈希。
- n100/、n1000/：逐行evaluations、comparison JSON/CSV、per_sample.csv、两套原数组+掩码NPZ、READY及summary。
- representatives.json：代表候选/拒绝配置的原始索引、q、EE位姿、完整身份、原始Native查询及Viewer路径。
- figures/：12份完整机器人Viewer（含同一q的严格/候选对照）和4份工作空间交互图。原24网格与38障碍均显示。

关键Viewer：N1000候选Sample0/437/726，自碰撞拒绝Sample2，环境拒绝Sample8；
N1000 strict_same_candidate_0展示同一q在严格SRDF下仍被拒绝。
图中工作空间散点均来自真实输入FK；Robot Mesh来自实际Native RobotState，不把散点当机器人几何。
静态几何预览与自动追溯检查已完成；浏览器交互仍待用户验收。

验证目录：`results/constrained_workspace/research_recovery_validation_20261104_01/`，包含回归日志、历史完整性检查和静态预览。

## 本轮代码与验收命令

新增两个experiment模块（assembly_validation、research_workspace）、CLI和测试；
Evaluator仅新增Semantic Pair截断标志透传，保留旧字段语义。未修改C++、两套SRDF、采样、FK或场景。
研究统计只读取semantic_* / configuration_decision，不使用旧空ACM的final_decision推算可行率。

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
export C1_BACKEND="$PWD/results/c3/assembly_build_20261104/build/collision_backend"

ctest --test-dir results/c3/assembly_build_20261104/build --output-on-failure
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests

# 只读复核现有结果，校验全部产物/输入哈希、原始数组和掩码。
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_research_workspace.py \
  --verify-run results/constrained_workspace/research_recovery_20261104_01

# 如需重新实验，必须使用全新的目录；本命令默认复用既有100/1,000组输入。
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B scripts/evaluate_research_workspace.py \
  --backend "$C1_BACKEND" --allow-candidate-baseline \
  --output-dir results/constrained_workspace/research_user_review_01

xdg-open results/constrained_workspace/research_recovery_20261104_01/figures/n1000_candidate_workspace.html
xdg-open results/constrained_workspace/research_recovery_20261104_01/figures/n1000_candidate_largest_self_separation_437.html
xdg-open results/constrained_workspace/research_recovery_20261104_01/figures/n1000_self_rejected_2.html
```

实际回归：基线528 passed；最终542 passed，1 warning，0 skipped；Native CTest1/1 PASS。
两模式全部Backend正常退出。没有依赖安装、实体控制、Commit/Push、10,000样本实验或规则扩张。

## 下一阶段前应完成的工作

1. 确认一条当前实体静止JointState、夹爪开度与Frame/TCP，核实示教记录和DH/ROS映射。
2. 完成活动装配Pair审核，尤其joint3及夹爪联动；若整对无法许可，则需要有物理依据的局部几何，而非继续加过滤项。
3. 定义任务TCP和网口接近方向、姿态/容差、允许夹爪状态及实物间隙要求，再开展研究级IK筛选。
4. 保留当前RESEARCH_ONLY身份，后续轨迹需独立检查连续运动、自碰撞和环境碰撞；禁止把配置级通过当作轨迹安全。
