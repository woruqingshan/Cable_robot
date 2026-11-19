# R1–R3 动作空间与标定证据分析（静态复核）

日期：2026-11-19
依据：本地只读源码快照，基线 origin/main = dee7ddba62d05098499ce9197cc443905fce21fb。
快照根目录：/private/tmp/idc_real_progress_20261119/repo_snapshot。

本报告未连接机器人或 AutoDL，未检查在线 checkpoint 文件，未实施动作、物理标定或云端实测。下面“Confirmed”仅表示源码/已有文档直接证实；“Unknown”不应被解释为不存在能力。硬件数据、当前厂商驱动和在线模型证据由主任务另行补充。

## 1. 最重要的边界

**Confirmed：输入的“五关节加夹爪六个状态值”，与输出的 [50,6] 原生动作不是同一个空间。** 前者是 Observation 的明确状态绑定；后者在已记录 SmolVLA 验收中仅有形状和数值有效性，六通道的物理含义未知。不能因维数相同就注册转换、截去一维、按关节缩放或将第六维解释为夹爪。

**Confirmed：当前执行 schema 本身刻意只接受合成执行。** 本轮不需要也不应为了观测/云检查去扩宽 RobotActionChunk 或 ValidatedTrajectory 以容许真实机器人执行。

| 类型 | 代码责任和约束 | 源码定位 |
|---|---|---|
| PolicyActionChunk | 保留 native_action_space、channel_names、units、absolute/delta/unknown、[H,D]、完整 horizon、policy/checkpoint/processor/normalization 身份 | vla_protocol/src/vla_protocol/action.py:46–76 |
| RobotActionChunk | [H,5]，joint1..joint5、rad、absolute；mapping_id 只能 synthetic_identity_v1，calibration_status 只能 synthetic_uncalibrated | 同文件:80–104 |
| ValidatedTrajectory | 只允许研究离散验证；continuous_collision_proven=False、hardware_validated=False；带轨迹时要求 simulated=True 的未标定夹爪起点 | 同文件:108–137 |
| ActionAdapterRegistry | 只有合成五关节绝对 rad identity 映射；未知空间、原生非合成动作、normalization 不匹配直接拒绝 | cable_robot_vla/cable_robot_vla/action_adapter.py:37–75 |

应保留 UNKNOWN / UNSUPPORTED_ACTION_PROFILE 的拒绝作为验收证据。PolicyActionChunk 成功并非 RobotActionChunk 成功，更不是可下发的物理轨迹。

## 2. 三种空间的定义

### 2.1 真实反馈 → 协议状态空间

Confirmed：协议要求 joint1..joint5 的确切顺序、5 个有限 rad 值；r_joint 以独立 GripperState 表示，单位仍为 rad，另有源时间戳及 simulated/calibrated 标记（observation.py:31–50）。

Confirmed：控制配置把舵机臂 ID 设为 1–5、夹爪 ID=10，脉冲范围写为 0..1000，夹爪 open/close preset 尚为空（cable_robot_control/config/arm_limits.yaml:1–18）。这些是当前软件配置，不是硬件安全限位或真实零位已校准的证明。

Unknown：新车实际 ID、方向/符号、零点、pulse-to-rad 关系、有效角度范围、反馈是否可信、夹爪是否对应角度还是夹持几何。本轮不应将配置范围当作实测范围，不应将最后一次命令当作反馈。

Confirmed：ArmInterface.get_servo_states() 默认走 topic；独立 get_servo_states_from_bus_service() 构造 get_position 请求（arm_interface.py:61–125）。服务反馈的正确性、时间戳和返回 ID/请求对应仍需新车测量。该类构造会创建两个物理命令 publisher 和写/IK client（同文件:45–55），不能直接实例化为“无控制发布器”的观测适配器。

### 2.2 协议状态 → 模型输入空间

Confirmed：ModelFeatureSpec 明确声明 state_channels、state_units、state_dimension 和 camera binding；当前验证只接受既定 joint/r_joint 名称及 rad 声明，最多六个状态，不做填充或推测顺序（vla_server/src/vla_server/observation_adapter.py:27–45）。bind() 根据名称重排，显式拒绝未允许来源、未知/缺失相机和严格尺寸不符（同文件:68–93）。

Unknown：这套应用输入声明是否与当前 checkpoint 的训练机器人、训练状态单位/零位、数据分布相符。_validate_features() 核对 shape、camera names、processor identity（smolvla.py:109–130），不是训练语义或物理标定证明。“六维输入能 forward”不能升级成“JetRover 观测已具有任务意义”。

Confirmed：原服务器生成配置只许可 synthetic 来源；real profile 必须在独立开发配置明确加入。允许 real_uncalibrated_v1 只允许数据检查，不授予动作执行能力（observation_adapter.py:35、70–72；deployment/cloud/STAGE2.md:140–165）。

### 2.3 原生模型输出 → 物理动作空间

Confirmed：模型代码核对官方输出 [1,H,D]、执行官方 postprocessor、检查有限值，再保留全部 [H,D] 数值和 policy identity，synthetic=False（smolvla.py:145–185）。

Confirmed（历史文档）：已有验收记录为 50 步、6 通道；通道名 native_0..native_5、单位 model_native_unknown、semantics=unknown，20 ms 只是人工声明的检查日程，不是验证过的控制时序（deployment/cloud/STAGE2.md:158–165）。

Unknown：六维分别为关节、末端位姿、相对变化还是其他量；是否含夹爪；坐标系、控制频率、起始状态依赖、插值/保持策略。基础 policy 当前没有可证明的 JetRover 物理动作语义。

因此 Joint-space、End-effector/TCP-space 和模型 latent/native-space 必须分别命名和论证。未来若采用末端动作，需要测定基坐标/工具坐标、TCP、插入轴并在五关节可实现约束下做 IK；不能假定五自由度机械臂能满足任意六维姿态。

## 3. 归一化、图像处理与身份

Confirmed：FeatureObservationAdapter 只解码 PNG/RGB 和构造 float32 state；图像转换为 RGB array 后交给官方 prepare_observation_for_inference（observation_adapter.py:95–110）。SmolVLA 加载官方 make_pre_post_processors，输入 preprocess、输出 postprocess；不应再套自创 scaler、均值/方差或关节归一化（smolvla.py:97–100、149–169）。

Unknown：实际 checkpoint 的归一化统计、训练通道语义和标定尺度，需从已部署 checkpoint/config/processor inventory 取得证据；本报告没有读取在线文件。不能把 source 里的 normalization_id 当作已验证的统计内容。

Confirmed：PolicyIdentity 记录 framework/checkpoint revision、processor_id、normalization_id（action.py:9–15）。Observation 本身的 robot_id、model_id、scene_id/hash 与 policy/checkpoint identity 是不同层次，不应混用。当前仿真节点把 model_id 绑定为 URDF identity；新车实现应明确命名和来源，完整记录 policy 的真实 revision。

代码风险提示：FeatureObservationAdapter.bind() 验证 feature binding，但没有用“物理兼容性证明”检查 observation.model_id 与 checkpoint 语义；PolicyClient 的请求/响应 identity echo 也不能替代输入语义验证。保留身份并据证据配置，比让字段都填同一个模型名更重要。

## 4. 时钟与采样时间语义

Confirmed：
- SourceStamp 是 nanoseconds + clock_domain（observation.py:12–14）。
- SensorObservationSource(kind=real) 选择 ros_system，并从 header 原样读取 sec/nanosec（integration/sources.py:22–38）。
- joints/gripper 目前取自同一个 JointState，采用相同时间戳；image 单独带源 stamp，按最近的实际历史样本配对（同文件:47–63、92–110）。
- ObservationAdapter 同时检查 receive age、source age、future timestamp、clock_domain 和 skew，默认 age=0.5 s、skew=0.02 s（observation_node.py:41–54；profiles.py:36–44）。

Unknown：新车 header 是实际设备采样时刻、驱动处理时刻还是单纯周期发布时间；相机硬件时钟与 ROS 时间是否已关联；总线多舵机顺序读取是否代表同时状态。相同 clock_domain 字符串不证明底层时钟已同步，多个消息刚到达也不证明源样本同步。

服务不带真实采样时戳时，可把请求/响应区间作为独立诊断证据，不应将 callback 到达时间捏成“原始采样时间”通过 20 ms 门槛。若无法建立可信时间语义，保留原始帧与拒绝原因，不宣称完整有效 Observation。

录制重放须区分：
1. 原始样本时间与原始有效性（必须保留）；
2. 当前实时调度/monotonic 到达时间（只用于运行期 freshness）；
3. 模型 native time_offsets（当前只是检查日程）。
离线协议 round-trip 不应将历史帧按当前墙钟判为“实时新鲜”，也不应改写原 stamp；重新送云需要一个明确的 replay/inspection 约定，不能伪装成刚采到的实时样本。

## 5. Home、TCP、相机外参与 P2 前置条件

Confirmed（静态研究文档）：
- 厂商 init_pose.yaml 指向 action/init，完整 init action 不在已提供树中，URDF zero 不是物理 Home。
- CABLE_HOLD_READY 仅候选参考，诊断 q≈[0,-1.1100,2.0316,1.1729,0] rad；
- vendor/URDF EE 数值差约 98 mm，是未注册参考/工具坐标下的差，不是测得物理 FK 误差；
- 候选夹爪约 0.838 rad，与任务模型 0.3 rad 不同；
- physical_home_verified=false，实验标记 REFERENCE_START_TO_READY，不能升级为实机动作。
来源：workspace_analysis/docs/t5_3_c_robustness_shortcut.md:105–130。

未来控制前必须具备：
- 关节 ID/方向/零位及物理停止范围；实际反馈、运动状态与跟踪误差证据；
- 真实 Home、任务 Ready 及其取得过程，不借 YAML 名称当作标定；
- 相机内参/畸变、真实 camera-to-base 外参及时间关联；
- flange/EE/gripper frame 与 RJ45 TCP 的实际变换、插入/键轴、连接器尺寸/夹持几何；
- 与实景一致的 robot/scene/model/TCP/calibration identity；
- 完整碰撞几何与语义检查：线缆、连接器、LiDAR、夹爪和 ACM/装配豁免均需复核；
- 执行前/中反馈、停止与恢复、限速/限加速度、超时及人工监督要求。

P2 当前的 PASS_RESEARCH_ONLY / DISCRETE_ONLY 不是连续碰撞或物理安全证明；其语义审核还绑定确切 expanded URDF hash，不可为了新车 ament 路径变化就静默重写/放宽历史绑定（deployment/local/STAGE23.md:64–86）。本轮只记录 uncalibrated 观测，不将真实输入导向旧的合成执行验证器。

## 6. 对本轮实现最值得防范的事项

1. ArmInterface 初始化就创建运动 publisher；只读反馈采集应独立使用 read client/订阅和纯解析，不能依赖“我不调用 move()”作为无 publisher 保证。
2. SensorObservationSource 同时要求 r_joint；缺测必须拒绝，不能从命令或 Home 值代填（sources.py:42–53）。
3. source 的 robot_id 为 jetrover_+kind，busy=False 固定；需有限参数化/明确保护，不把 busy=False 称为“实机已证实静止”（sources.py:106–109）。
4. PNG/大小/哈希/来源校准标记是数据一致性校验，不证明关节来自硬件、相机已标定或场景几何真实（observation.py:53–65、69–93、107–130）。
5. 不将 real profile 加入通用默认列表或覆盖 synthetic-only 实验配置；使用单独 inspection config，保留错误来源的拒绝测试。
6. 只记录 PolicyActionChunk，不调用仿真 execution backend。输出形状有效、HTTP 成功、甚至 P2 研究通过，均不授予执行权。
7. 本轮没有理由将 RobotActionChunk 的 synthetic literal 改为真实标定值；那属于未来受审动作空间/协议版本设计。
8. 原 LoopRecorder 具有 simulation_only 默认且去除图像载荷；真实 PNG/JSONL 录制需正确源标签和独立图像文件，不能直接把日志当可回放数据集。
9. source invalidation / 指令变更 / 时钟回退须使未消费 HTTP 请求失效；仅比较 server 回传 identity 不能证明响应仍对应当前有效实机状态。
10. 模型处理器和底层 LeRobot/Torch 独立于 ROS Python；R1–R3 不向 JetRover ROS 环境安装 GPU 模型栈。

## 7. 本报告的交付状态

- 已完成：基线源码的动作/状态/时钟/标定边界静态复核，提供可核对的文件行号。
- 未声称完成：新车 feedback 真实性、相机内外参、物理 Home/TCP、checkpoint 实体元数据核查、真实云推理。
- 本轮执行要求：未知动作继续拒绝；真实观测保持 simulated=False、calibrated=False、real_uncalibrated_v1。只有真实来源与时间可信的数据才能作为完整有效 Observation。

