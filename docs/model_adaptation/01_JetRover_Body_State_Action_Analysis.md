# JetRover 本体、状态空间与动作空间分析

> 2026-11-26｜Stage 1 完成后的本体接口参考｜静态资料审计，非物理标定报告

## 1. 结论

JetRover 是 **5 自由度串联机械臂 + 1 个独立夹爪执行器**，不是六自由度机械臂。当前机械臂控制接口与连续反馈的六路顺序为 `[1,2,3,4,5,10]`，对应 `[joint1,joint2,joint3,joint4,joint5,r_joint]`。本体状态与位置动作都可以采用六维表示，但夹爪应与五个臂关节分别定义。底盘与 ID9 不属于本次 VLA 适配范围。

厂商 YAML 已经提供各关节的名义零位、转换方向和软件 Pulse 范围，因此不必从零开发状态转换；目前能够确定的是名义控制语义，不是经过实体测量的实际零位、物理角度精度或安全运动范围。

## 2. 证据来源

- 用户上传 `jetrover_description.zip`：实际解压核对 `jetrover_description/urdf/{connect,arm,gripper}.urdf.xacro`。
- 用户上传 `servo_controller.yaml`：`/**/ros__parameters` 下的 joint 配置及控制器列表。
- 项目 GitHub 检查点 `e77fa3d`：[`hardware_feedback.py`](https://github.com/woruqingshan/Cable_robot/blob/e77fa3d/cable_robot_vla/cable_robot_vla/hardware_feedback.py)、[Stage 1 使用说明](https://github.com/woruqingshan/Cable_robot/blob/e77fa3d/docs/real_operation/STAGE1_USER_GUIDE.md)、[阶段总结](https://github.com/woruqingshan/Cable_robot/blob/e77fa3d/docs/real_operation/STAGE1_DEVELOPMENT_SUMMARY.md)。

本次未连接机器人，未测量机械尺寸、运行控制器或读取当前运行时展开后的 URDF；模型文件与实体安装一致性仍待验证。

## 3. 机械臂结构与关节语义

下表的 axis 为 URDF **关节局部坐标系**中的旋转轴，不能直接视为世界坐标系方向，也不能直接等同于舵机 Pulse 增减方向。所有五个臂关节的 origin rpy 均为 `(0,0,0)`。

| 顺序 | Servo ID | URDF joint | Parent → Child | Axis | 名义功能 |
|---:|---:|---|---|---|---|
| 0 | 1 | joint1 | base_link → link1 | (0,0,-1) | 底座旋转 |
| 1 | 2 | joint2 | servo_link1 → link2 | (0,1,0) | 肩部俯仰 |
| 2 | 3 | joint3 | link2 → link3 | (0,1,0) | 肘部俯仰 |
| 3 | 4 | joint4 | link3 → link4 | (0,1,0) | 腕部俯仰 |
| 4 | 5 | joint5 | servo_link2 → link5 | (0,0,-1) | 腕部旋转 |
| 5 | 10 | r_joint | gripper_link → r_link | (0.99996,-0.004699,-0.0070434) | 夹爪开合驱动 |

`servo_joint1`、`servo_joint2`、`gripper_joint`、`end_effector_joint` 均为固定关节，不增加自由度。夹爪的 `l_joint`、`l_in_joint`、`l_out_joint`、`r_in_joint`、`r_out_joint` 均通过 mimic 依赖 `r_joint`，乘数分别为 -1、-1、+1、-1、+1，偏置均为0。

YAML 另有 `w_joint`（ID9，init=500，min=0，max=1000），但它未出现在 arm_controller/gripper_controller 的受控列表，也不属于本次六路反馈；不能将其加入六维策略接口，具体硬件用途本次不推断。

### 3.1 URDF 名义几何

| 关节/固定变换 | Origin xyz（m） |
|---|---|
| joint1 | (0.0251328065, 0, 0.0774026881) |
| joint2 | (0, 0, 0.0338648012) |
| joint3 | (0, 0, 0.1294164464) |
| joint4 | (0, 0, 0.1294446312) |
| servo_joint2 | (0, 0, 0.0544833340) |
| joint5 | (0, 0, 0) |
| gripper_joint | (0, 0, 0.0054388) |
| end_effector_joint | (0, 0, 0.08) |

肩–肘、肘–腕的关节原点间名义距离均约129.4 mm；这里是模型几何，不是实测 Link 尺寸。`end_effector_link` 是模型中的末端参考点，不代表实际 RJ45 TCP。五自由度不能一般性地实现任意六维笛卡尔位姿。

## 4. 舵机配置与数学转换

| 关节 | ID | init | min 字段 | max 字段 | 名义方向 dq/dp | URDF limit (rad) |
|---|---:|---:|---:|---:|---|---|
| joint1–5（每个） | 1–5 | 500 | 1000 | 0 | 负 | [-2.09, 2.09] |
| r_joint | 10 | 700 | 1000 | 0 | 负 | [-1.57, 1.57] |
| w_joint（排除） | 9 | 500 | 0 | 1000 | 正 | 本次不纳入 |

注意：`min=1000, max=0` 是厂商有方向的配置，不意味着数值范围为空；排序后范围为0–1000。现有 `JointFeedbackConversion` 对 `min>max` 使用 `init-pulse`。

设 k = 240π/(180×1000) = 0.004188790205 rad/Pulse（0.24°/Pulse），p 为真实读回的 Pulse，则当前六路的名义转换为：

```text
q_i = k * (500 - p_i), i=1..5
q_g = k * (700 - p_10)
p_i = round(500 - q_i / k)
p_10 = round(700 - q_g / k)
```

逆变换仅是名义数学关系；不得直接作为已经获准的实机控制指令。实际控制应使用统一驱动的已审核接口，并核对物理限位、夹爪方向、碰撞和停止机制。

### 4.1 软件范围与 URDF 范围并不完全一致

| 对象 | YAML Pulse 全范围映射 | URDF 角度范围 | 仅数学交集对应的整数 Pulse |
|---|---|---|---|
| 五个臂关节 | [-120°, +120°] | ±2.09 rad（约±119.75°） | [2,998] |
| 夹爪 | [-72°, +168°] | ±1.57 rad（约±89.95°） | [326,1000] |

上表最后一列按 YAML 名义公式和 URDF 数学约束求得，**不是物理安全范围，也不是运动授权**。尤其夹爪的厂商 Pulse 全范围远大于 URDF 上限对应的正角度范围，后续需要单独测量夹指开合距离与实际安全端点。URDF 中 effort=1000、velocity=10 同样不能视作实测能力。

### 4.2 真实历史反馈计算示例

取 R1 历史数据中的 `[499,767,15,219,500,499]`（非当前实时姿态）：

| 通道 | Pulse | 名义角度（°） | 名义角度（rad） |
|---|---:|---:|---:|
| joint1 | 499 | 0.24 | 0.004189 |
| joint2 | 767 | -64.08 | -1.118407 |
| joint3 | 15 | 116.40 | 2.031563 |
| joint4 | 219 | 67.44 | 1.177050 |
| joint5 | 500 | 0.00 | 0.000000 |
| r_joint | 499 | 48.24 | 0.841947 |

这不是已标定物理角度。其中 ID3 靠近名义角度上限，不能作为默认安全标定姿态或任意运动起点。

## 5. 建议固定的本体数据契约

### State（读取）

```yaml
robot_profile: jetrover_5dof_gripper_v1 # 候选标识，尚未注册
channels: [joint1, joint2, joint3, joint4, joint5, r_joint]
servo_ids: [1, 2, 3, 4, 5, 10]
units: [rad, rad, rad, rad, rad, rad]
semantics: nominal_joint_position
physical_calibrated: false
inner_association_verified: false
```

保留原始 Pulse、每路事务时间、有效性及标定版本。夹爪 rad 是驱动关节的名义角度，不是开口距离或百分比。名义 rad 可以用于研究观察，不能自动获得正式可信状态资格。

### Action（候选，未来真实控制）

推荐优先采用五关节绝对目标 rad + 独立夹爪目标（初期与名义驱动角度统一，物理标定后再决定是否使用开口距离）。动作中保留目标时间、起始实测状态及映射版本；记录控制器实际发送的目标，不能把反馈状态当作动作标签。

`arm_controller` 列出 joint1–5，`gripper_controller` 只列出 r_joint；两者 YAML 均有 min_velocity=0.1、goal_time=0.05，但这些是配置字段，不等于已经测得的硬件速度、VLA 步长或实际控制周期。

## 6. 与当前系统的关系

Stage 1 已经验证：D405 RGB 与名义六路真实反馈联合上传、云端 GPU 推理、50×6 原生动作返回记录。20 Hz 是底层轮询频率，约10 Hz 是反馈状态发布频率，不是云端 VLA 控制频率；当前连续推理约1.39 Hz。

截至 Stage 1 最后一次记录，ID1 READ_TIMEOUT 导致查询暂停、运动权限关闭；人工断电恢复此前通过，自动恢复未验证。新的硬件实验必须重新核对状态。当前不应执行任何未经语义验证的 VLA 动作。

## 7. 接下来要做的最小工作

1. 以本文件作为 JetRover 名义本体契约，保留现有 Pulse→rad 代码，不重写串口或 URDF。
2. 离线核对与 SO-100 的对应关节、零位定义和单位；与配套的 `02_SmolVLA_JetRover_Adaptation_Analysis.md` 联合使用。
3. 等具备现场条件后逐关节确认实际方向、零位、测量角度、回差和实际范围，并标定夹爪开合；建立可信反馈与受控执行前再验证恢复机制、时间匹配和运动安全。

不需要为了本轮模型语义分析提前进行手眼标定、TCP 标定或复杂工作空间算法开发。

## 8. Stage 2 逐通道映射评估（2026-11-26）

|JetRover / SO-100候选|局部轴（JetRover）|顺序/功能|待验证映射|
|---|---|---|---|
|joint1 / shoulder_pan|(0,0,-1)|0，底座旋转|SO物理轴、方向、零位 UNKNOWN|
|joint2 / shoulder_lift|(0,1,0)|1，肩俯仰|同上|
|joint3 / elbow_flex|(0,1,0)|2，肘俯仰|同上|
|joint4 / wrist_flex|(0,1,0)|3，腕俯仰|同上|
|joint5 / wrist_roll|(0,0,-1)|4，腕旋转|同上|
|r_joint / gripper|近似局部x轴，见第3节|5，独立夹爪|SO归一化开合与JetRover驱动rad关系 UNKNOWN|

JetRover 的名义状态单位 rad、候选目标单位 rad；SO源训练单位和实际目标语义尚未闭合。名义范围见第4节，不升级为安全范围。通道重排和已知单位转换可以纯软件实现；sign、offset、夹爪端点需物理标定。连杆尺寸、轴安装与 TCP 不同会改变同一关节向量的末端轨迹，不能靠缩放角度解决。D405腕部单视角与源 top/wrist/side 顺序也不是简单单位转换。30FPS参考数据、约20Hz轮询和云端约1.39Hz各有不同含义，动作时间仍待定义。

本轮未测 SO实体、未修改 URDF、反馈驱动或执行接口。综合路线及数据方案见配套分析第9–10节。
