# P2.3-A — Technical Audit & Architecture Design

审查日期：2026-11-03。对象：JetRover_Mecanum / LIDAR_TYPE=A1，固定底盘。
状态：**P2.3-A 文档交付，等待用户审核；B/C/D/E 均未实施。**

本文件记录实际源码/环境证据，并设计后续接口。只新增本文件，不修改厂家文件、
P2.2 模块、配置、测试或实验数据；没有安装依赖、编译碰撞程序、运行碰撞实验、
启动 ROS 节点、RViz、Gazebo 或实体机器人。本轮没有重跑 P2.2 正式实验或 pytest。
文中代码块是接口/Schema 草案，不是已经存在的实现。

## 1. 范围、版本与 Git 状态

本轮依据用户的 P2.3 Development Instruction；研究背景另见
[项目任务书](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/docs/develop/IDC_VLA_Project_Specification_and_Research_Context.md)。
背景文件 v1.0 中的 P2.2 状态和 P2.3–P2.5 编号属于旧规划，当前阶段以新指令为准：

| 阶段 | 本轮采用的职责 | 本轮状态 |
| --- | --- | --- |
| P2.2 | 模型/FK、几何采样、可视化、复现与统计 | 用户已验收，复用 |
| P2.3 | 环境表示、整机状态碰撞检查、受限采样比较 | 仅 A 审计与设计 |
| P2.4 | TCP、网口姿态、IK、接近/插入/撤离约束 | 规划接口，不实施 |
| P2.5 | 约束路径/轨迹规划与 VLA 动作协同 | 不实施 |
| P2.6 | 仿真与实物实验 | 不实施 |

这是执行层基础工具链与实验方案，尚未验证的方法不标为论文创新。
研究递进是几何可达 → 无碰撞配置集合 → 任务约束可行性 → 连续运动规划。

实际 Git 根目录 `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot`，分支
`feat/workspace-planning`，HEAD=`be68b19`。最近五条：

```text
be68b19 Merge pull request #2 from woruqingshan/feat/workspace-planning
b659e99 feat(workspace): complete G4 reproducibility and stability validation
f8db867 feat(workspace): implement interactive 3D workspace visualization
acc66e2 feat(workspace): implement Monte Carlo reachable workspace sampling
9cca8c9 feat(workspace): add ROS2 TF cross-validation for forward kinematics
```

开始时未跟踪项：`docs/develop/`、`workspace_analysis/reports/g4_validation.md`、
`workspace_analysis/src/workspace_analysis/validation_report.py`。未发现已跟踪文件修改。
按“工作区不干净则停止 Git 同步”的条件，本轮**没有 fetch 或 Fast-forward merge**，
也没有新建/切换分支、Commit、Push 或 Merge 回 main。
本地缓存的 main、origin/main 和当前分支都指向 be68b19；这不能证明远程当前没有更新，
**远程最新状态 UNVERIFIED**。当前情况已不同于任务描述中的“落后 main 一个提交”。

特别记录：已跟踪的 validation.py 在 run_validation 中导入 validation_report.py，
但后者当前未被 Git 跟踪。当前机器能运行不代表仅克隆仓库就能复现 G4。
这是待用户后续处理的版本管理问题，本轮不暂存、不提交、不删除该文件。

## 2. Verified Existing Capabilities — P2.2 源码与结果

核对了 README、workspace.yaml、所有生产模块及相关测试。它们与此前 G4 开发副本
逐文件字节一致；下面列出对本阶段有约束力的实际接口和边界。

| 能力 | 代码依据 | 后续用法/限制 |
| --- | --- | --- |
| 动态 Xacro/URDF | [model.py:314](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/model.py:314) | 复用 load_model；显式 Mecanum/A1；不要求 need_compile |
| 树和主链 | [model.py:158](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/model.py:158) | joints 保留整树 JointSpec，chain 只含 Base→EE；没有保存 collision geometry |
| 辅助/mimic 状态 | [model.py:102](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/model.py:102) | joint_positions(q, auxiliary_positions) 可核对全关节状态；默认 r_joint=0 |
| PyKDL FK | [fk.py:52](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/fk.py:52) | pose(q) 只返回 base_link→end_effector_link，不返回整机每个 Link 的位姿 |
| 原始采样 | [sampling.py:151](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/sampling.py:151) | 独立均匀关节采样；P2.3-D 优先读取已有 q，不重写采样器 |
| 可信数据读取 | [results.py:228](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/results.py:228) | 必须复用 load_results(prefix, model=model)，校验哈希/模型/数组，并内部重算至多三行 FK |
| 绘图/行号 | [visualization.py:68](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/visualization.py:68) | 保持 position/q/原行号对应；B 新增场景绘制层，不改 G3 核心 |
| 固定体素网格 | [validation.py:64](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/validation.py:64) | VoxelGrid/voxel_hits 仅作统计；不能用 2 cm 网格替代精确碰撞检查 |
| G4 汇总读取 | [validation_report.py:208](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/src/workspace_analysis/validation_report.py:208) | 只读校验已完成的 CSV/JSON/Markdown，拒绝失败 marker |

现有 tests/test_model_fk.py 验证固定段、非零 origin RPY/斜轴、joint5 位置不变和
夹爪不影响 EE；test_tf_validation.py 验证时间戳、模型回读、严格端点与退出；
test_sampling_results.py 验证行对应、限位、哈希、元数据与禁止覆盖；
test_visualization.py 验证原始行号、负 Z、轴比例与离线 HTML；
test_workspace_validation.py 验证确定性、固定网格及汇总一致性。
此前回归结果为 221 passed、1 warning；本轮只审查测试源码，没有把它当作新运行结果。

只读调用 load_validation_bundle 验证 `results/g4_run01`：status=PASS、profile=formal、
九组数据行，复现四数组完全一致，位置误差 0 m，诊断旋转角误差
2.0770370905276127e-16 rad。网格 shape=[60,60,60]，分辨率 0.02 m。
实际报告：[g4_validation_run01.md](/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/reports/g4_validation_run01.md)。
G1-A、G1-B 的历史证据已在该报告引用；没有重新执行 ROS TF。
旧结果保留 pending_user_review 字段是生成时的记录，本轮按最新用户指令认定 P2.2 已验收，
不修改历史 JSON。实物定位精度、TCP 和碰撞有效性仍不在 P2.2 验证范围内。

## 3. Verified Existing Capabilities — 官方机器人几何审计

官方目录 `/home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description`。
既有工作空间 src/jetrover_description 软链接解析到该目录。package.xml 声明 URDF、
Xacro、RSP、JSP、RViz 依赖，没有 MoveIt 配置；setup.py 分发 urdf 和 mesh 资源。
jetrover.xacro 按 Mecanum/A1 展开，包含底盘、外壳、lidar、IMU、腕部相机、机械臂、夹爪。

展开 URDF SHA256：
`5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`。
workspace.yaml SHA256：
`0028f14160dc6bbf1dec0671c681961ebc46eedc0e0e7bd0c3d45c0ca5800b78`。

### 3.1 树、关节与固定变换

展开模型有 **29 Links / 28 Joints**：17 fixed、11 revolute。
11 个 revolute 中只有 6 个独立量：joint1..joint5 与夹爪 r_joint；另 5 个是 mimic。
本 Mecanum URDF 中四个车轮关节均为 fixed，不能据此声称模拟了移动底盘运动。

主链如下；下表数值是审查记录，后续代码仍动态读取，不能硬编码几何。

| 关节 | Parent → Child | origin xyz [m] | 局部 axis | limit [rad] |
| --- | --- | --- | --- | --- |
| joint1 | base_link → link1 | (0.0251328065010765,0,0.0774026880954513) | (0,0,-1) | [-2.09,2.09] |
| servo_joint1 | link1 → servo_link1 | (0,0,0) | fixed | — |
| joint2 | servo_link1 → link2 | (0,0,0.0338648012164686) | (0,1,0) | [-2.09,2.09] |
| joint3 | link2 → link3 | (0,0,0.129416446394797) | (0,1,0) | [-2.09,2.09] |
| joint4 | link3 → link4 | (0,0,0.129444631186569) | (0,1,0) | [-2.09,2.09] |
| servo_joint2 | link4 → servo_link2 | (0,0,0.0544833339503674) | fixed | — |
| joint5 | servo_link2 → link5 | (0,0,0) | (0,0,-1) | [-2.09,2.09] |
| end_effector_joint | link5 → end_effector_link | (0,0,0.08) | fixed | — |

这些主链 origin RPY 均为零。依据
[connect.urdf.xacro](/home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description/urdf/connect.urdf.xacro:4) 和
[arm.urdf.xacro](/home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description/urdf/arm.urdf.xacro)。
主链之外必须包括：

- gripper_joint：link5→gripper_link，固定平移 z=0.0054388 m；末端 EE 是另一条固定分支，不能把 EE 点当成整个夹爪几何。
- r_joint：gripper_link→r_link，origin=(-0.0073166,-0.013992,0.027059)，
  axis=(0.99996,-0.004699,-0.0070434)，limit=[-1.57,1.57]；不能假定为精确 +X。
- l_joint/l_in_joint/r_in_joint = -r_joint；l_out_joint/r_out_joint = +r_joint，offset 全为 0。
  全部原点、斜轴和父子级联都要保留。依据
  [gripper.urdf.xacro:42](/home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description/urdf/gripper.urdf.xacro:42)。
- 腕部相机安装于 link4：camera_connect_joint 有 yaw=-π/2，depth_cam_joint 的
  RPY=(π,-π/2,-π/2)，depth_cam_joint_sim 又有光学参考变换；不能只遍历 EE 链。
  依据 [depth_camera.urdf.xacro:41](/home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description/urdf/depth_camera.urdf.xacro:41)。
- root=base_footprint；base_joint 使 `^footprint T_base` 平移 z=0.1160910821576746 m。
  依据 [car_mecanum.urdf.xacro:11](/home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description/urdf/car_mecanum.urdf.xacro:11)。
  这不是实测地面标定，P2.2 的 Base 保持 base_link。

### 3.2 collision/visual 资源完整性

逐 Link 解析展开 XML，记录每个有效 collision 的 origin、geometry、scale、URI，
通过 ament 定位资源，并只读检查 STL 头、三角形数及有限顶点。结果：

- 25 个 visual、**24 个有效 collision**，均为 mesh，没有有效 collision primitive。
- 24 个 collision URI 均可解析到当前包中的实际文件，均可按 binary STL 解析，
  顶点有限，共 **21,568 个三角形**。不是碰撞运行测试或 mesh 品质认证。
- 当前全部 collision origin xyz/rpy=0，全部未写 scale（URDF 默认 1,1,1）。
  **Joint origin 仍可能非零/有旋转**；未来实现必须通用支持 collision origin 和 scale。
- geometry 单位按 URDF 解释为 m。比如 link2 collision z 范围约 [-0.0158894,0.147317]，
  与模型量级一致；STL 本身不携带可靠物理单位，实物尺寸一致性 UNVERIFIED。

| 部位/Links | collision 来源 | 文件/节点依据 |
| --- | --- | --- |
| base_link、四车轮 | meshes/mecanum/*_collision.stl | car_mecanum.urdf.xacro 的五个 collision |
| 两后壳 | meshes/common/back_shell_*_collision.stl | common.urdf.xacro |
| imu_link | meshes/common/imu_link.stl | imu.urdf.xacro；与 visual 共用 |
| link1、link5 | meshes/arm/link1.STL、link5.STL | arm.urdf.xacro:31、328；与 visual 共用 |
| servo_link1、link2、link3、link4、servo_link2、gripper_link | meshes/arm/*_collision.stl | arm.urdf.xacro 中对应 Link |
| camera_connect_link | meshes/arm/camera_connect_link.STL | depth_camera.urdf.xacro:30；与 visual 共用 |
| depth_cam_link | meshes/arm/depth_cam_link_collision.stl | depth_camera.urdf.xacro:83 |
| 六个夹爪活动 Links | meshes/gripper/{r,l,r_in,l_in,r_out,l_out}_link.STL | gripper.urdf.xacro；与 visual 共用 |

**缺口：lidar_link 有 visual，但 collision 被注释掉**，见
[lidar.urdf.xacro:30](/home/siyuen/Desktop/code/cable_robot/system_source/simulations/jetrover_description/urdf/lidar.urdf.xacro:30)。
其固定安装位置在 base_link 的 (0.0900034859353204,0,0.0405195774179554)。
base_footprint、lidar_frame、depth_cam_frame、end_effector_link 则是没有实体 visual/collision 的四个参考 Link。
“参考 Frame 无几何”可登记为预期情况；不能把实体 lidar_link 的缺失同样忽略。

未发现 RJ45 插头、真实工具 TCP、柔性网线的 URDF 碰撞体。不能用空的 end_effector_link
证明工具无碰撞。官方已有夹爪本体和六个活动零件，但尚未验证与实物的覆盖关系。

这些资源使当前模型**具备建立碰撞后端的主要输入**，但还不满足“完整整机/插头安全性已验证”：
lidar 缺失、无已审查 ACM、mesh 简化误差/几何质量/实物覆盖未验证、工具缺失。
本轮不运行任何关节配置碰撞检查，不推断零位是否自碰撞。

### 3.3 机器人模型与碰撞模型的关联

保留已验收 G1-A，禁止把其 EE 结果当作所有 collision object 的位姿。
推荐后端从同一完整 URDF 构造 MoveIt RobotModel/RobotState，计算整树 Link 位姿：

`^base T_collision_i(q) = ^base T_link_i(q) × ^link_i T_collision_i`。

mesh scale 应施加于局部顶点，不应缩放刚体平移或破坏 SE(3)。碰撞变换先更新所有父链、
固定段和 mimic，再查询 collision body；环境与所有实体 Link 都应参与，首版不限定 arm group。
q 顺序仍为 joint1..joint5，另显式设置独立 r_joint（初期 0），由 RobotState 解析 mimic，
并与现有 model.joint_positions 的结果核对。

若 RobotModel 的 model frame 为 root/base_footprint，则统一桥接：

```text
^base T_link = inverse(^root T_base) × ^root T_link
^root T_obstacle = ^root T_base × ^base T_obstacle
```

先用三组已知 q 和少量额外配置验证后端的 `^base T_ee` 与 G1-A 一致，沿用
位置≤1e-5 m、姿态角≤1e-4 rad；这不是新的硬件精度声明。再检验相机/夹爪固定段与 mimic。

缺失 lidar 的首选补充路径是**自研、可追溯 collision overlay**：经用户确认后评估将
已有 lidar visual mesh 引用为碰撞几何或建立经测量验证的代理；不改官方 Xacro，
不自动把所有 visual 当 collision。派生模型须记录原 URDF 哈希、overlay 内容与哈希、
全部 mesh 哈希、ACM、padding、工具和 backend 版本。

G2 模型哈希只覆盖展开 XML/源 Xacro，并不包括 STL 内容，故它不能单独作为碰撞模型身份。
overlay 即便只加 collision 也会改变 XML 哈希。G2 仍用原始 load_model/load_results 校验；
另建立 `kinematic_parent_sha256 → collision_model_sha256` 关系，验证主链未改变，
不得改写旧 JSON 或放宽原模型哈希检查来兼容 overlay。

若覆盖范围不全，只能输出显式 PARTIAL_MODEL/UNVERIFIED；不得给出全机器人 collision-free 结论。

## 4. Verified Existing Capabilities — 本机碰撞框架

检查手段：source 既有 Humble/overlay 后执行 ros2 pkg list；dpkg-query；ldconfig；
Python find_spec；读取已安装头文件、CMake 导出和厂家源码。没有启动 ROS graph。

| 组件 | 实际状态 | 验证边界 |
| --- | --- | --- |
| MoveIt 2 | ros-humble-moveit/core 2.5.9 已安装 | 包/头文件/共享库存在，未构建自研调用程序 |
| PlanningScene / RobotState / FCL plugin | 对应 libmoveit_*.so 和 .h 已存在；ldd 未报 not found | 不代表真实构造/mesh 加载/碰撞调用已通过 |
| C++ FCL | libfcl-dev/libfcl0.7，0.7.0-3；pkg-config=0.7.0 | 可作为 native 后端依赖；未调用 collide/distance |
| Assimp | libassimp-dev 5.2.2~ds0-1；pkg-config 报 5.2.0 | 版本信息来自不同元数据，不能当成已验证 ABI |
| Eigen / JSON / YAML C++ | Eigen 3.4.0、nlohmann-json3-dev 3.10.5、yaml-cpp-dev 0.7.0 已安装 | C++ 构建依赖可定位；不需先装 Python mesh 包 |
| g++ / cmake / pkg-config | /usr/bin 下可定位 | 本轮不编译 |
| Python fcl / moveit / moveit.core / moveit_commander | 系统 Python 和 .venv 均不可定位 | 不能假定可以 import 后直接离线调用 |
| Python trimesh / pyassimp | 两个解释器均缺失 | 本轮未安装，首版设计不依赖它们 |
| pymoveit2 | ROS 包 4.2.0，模块路径可定位 | 源码需要 Node，发布 collision_object，访问 PlanningScene/规划服务；不是本地无节点碰撞求解器 |
| P2.2 .venv | Python 3.10.12、PyKDL 1.5.1、NumPy 1.21.5、SciPy 1.8.0、Plotly 5.24.1 | 复用，不改环境 |
| JetRover MoveIt 配置/SRDF | 在本地 cable_robot 源树及 jetrover_sim_ws 中未找到 *.srdf 或 JetRover MoveIt 配置 | 限于已搜索路径，不声称厂家其他版本/互联网必然没有 |

厂家 driver/kinematics 的 search_kinematics_solutions_node.py 提供 FK/IK/舵机相关服务，
引用两个 .so；不是碰撞引擎，源码检索未找到可复用的 PlanningScene、ACM 或 FCL 调用。
不能把“某 IK 解存在”当作“已做碰撞过滤”。没有运行该节点或反向推断闭源 .so 内部行为。

关键本机 API 证据：

- `/opt/ros/humble/include/moveit/planning_scene/planning_scene.h:103` 接受 RobotModel，
  :108 接受 URDF/SRDF，构造函数不要求传 Node。
- 同文件 :350 的非 const state 检查会 updateCollisionBodyTransforms；:450 的
  self-collision 调用使用 unpadded collision environment。
- `/opt/ros/humble/include/moveit/robot_state/robot_state.h:195` setVariablePosition
  会标记变换更新并 updateMimicJoint，getGlobalLinkTransform 可取得整树变换。
- `/opt/ros/humble/include/srdfdom/srdfdom/model.h:77` 有 initString。
- pymoveit2/moveit2.py:115、276、292 的 publisher/服务客户端确认它依赖 ROS 通信。

### 4.1 三条方案比较与推荐

| 方案 | 实际可行入口 | 优点 | 工作/风险 | 建议 |
| --- | --- | --- | --- | --- |
| MoveIt PlanningScene | C++ RobotModel+RobotState+FCL；显式 URDF/SRDF/scene | 整树运动学、mimic、mesh、world/attached body、ACM 集中管理，可延续 P2.5 | 无现成 JetRover SRDF/ACM；需要小型 native 构建、Frame 桥接和覆盖审查 | **P2.3-C 首选候选** |
| Direct FCL | 当前可用的是 C++ 0.7；Python binding 缺失 | 独立离线，mesh BVH/primitive 距离和碰撞，控制直接 | 自己管理 mesh 读取、全树变换、pair filtering、工具、distance/接触语义；比只写 FK 多很多职责 | native MoveIt 不适配时的明确备选，不静默切换 |
| 简化 primitives | Box/Cylinder/Capsule，交给成熟 backend 计算 | 可解释、便于合成测试；环境首版尤其合适 | 连杆包络需验证保守覆盖；过小漏检、过大误拒；未验证模型不支持最终安全结论 | 环境首选；机器人只能作明确标记的代理实验 |

推荐 Python 场景层 + 小型 C++ 离线批处理进程（JSON 请求/响应，nlohmann_json 已存在），
backend 进程一次加载模型/scene，多次接收 q；不每行启动一个进程，不要求 Python FCL/pybind。
这不是现在已经可运行的程序。P2.3-C 首先做最小构建/载入/整树 FK 对照，失败就记录原因、
提出备选并等待审查，不用缺失模型或固定假结果代替碰撞检查。

离线实验可直接持有 PlanningScene，输入使用不可变场景快照，不必启动 move_group、RSP、
PlanningSceneMonitor 或实体控制器。**此无 ROS 节点路线是基于本机 API 的设计推论，
运行可行性 UNVERIFIED**。官方在线场景维护推荐 PlanningSceneMonitor，P2.5/P2.6
需要实时机器人/传感器更新时再考虑，不把它与当前离线快照用途混为一谈。

碰撞后端默认检查整机，不把 group_name 限定到五关节 arm 后漏掉固定底盘与附件。
每个请求清空 CollisionResult，更新所有 Link/collision transforms，防止上一个 q 的状态串用。
接触点返回数量可限额，但不能把触点列表截断理解成完整碰撞对列表。

## 5. Proposed Architecture — Frame 与变换规范

`^A T_B` 将 B 中的列向量变换到 A，右乘局部变换，链乘顺序固定：

```text
^base T_port = ^base T_cabinet × ^cabinet T_panel × ^panel T_port
^base T_tcp(q) = ^base T_ee(q) × ^ee T_tcp
^base T_port_from_vision = ^base T_camera × ^camera T_port
```

| Frame | 职责 | 来源/约束 |
| --- | --- | --- |
| base_link | P2.2/场景的统一参考，底盘固定 | 不是实测地面；不隐式加地面碰撞平面 |
| base_footprint | 官方树根，仅用于后端坐标桥接 | 不能给 G2 XYZ 再重复加 0.116091 m |
| cabinet_frame | 柜体安装基准 | 合成或实测安装变换，来源必须标记 |
| panel_frame | 面板基准与端口阵列安装关系 | 父为 cabinet_frame；不自动等同网口平面 |
| port_frame | 端口入口中心与键控方向 | 父为 panel_frame；由测量/视觉定位，不由 EE 可达点猜测 |
| end_effector_link | 已验收 URDF 末端参考 | 不是插头尖端 |
| tcp_frame | 工具操作点和轴 | `^ee T_tcp` 未标定；缺失时禁止任务级位姿判定，不能默认单位阵 |
| camera_frame | 视觉估计的发布参考 | 未知外参/时间戳时不得直接注入；腕相机外参可随 q 变化 |

场景 Frame ID 采用 scene_id/局部名称，不能覆盖机器人保留 Link 名；每帧唯一父节点，
无环，所有分支须连到唯一 base_link。不同 scene 的 Frame 不可交叉引用。
视觉接入将来保存时间戳、来源 frame、标定 ID/哈希和测量不确定度；离线分析固定该快照。
不在 B 中估计相机外参、发布 TF 或进行实物标定。

Pose v1 只接受 position_m[3] 和 quaternion_xyzw[4]，表示 child 在 parent 中的位姿。
右手系，局部点变换为 `p_parent=R_parent_child*p_child+t_parent_child`；四元数单位范数。
URDF 自身 RPY 仍按 `Rz(yaw) Ry(pitch) Rx(roll)`；不要把 mesh/cylinder 局部轴混成世界轴。

**端口操作轴建议约定，待用户审核**：port +Z 指向机柜外/机器人接近侧，-Z 指向插孔内部；
port X 为键控方向基准，Y 由右手系确定，不能只给一个无滚转定义的法向。
接近点 `^base T_pre = ^base T_port × Trans(0,0,+d_approach)`；插入沿 port -Z，
撤离沿 +Z。TCP +Z 拟定义为工具推进轴，操作时须与 port -Z 对齐，工具键控轴另行匹配。
这些是坐标语义，不是已经实现的动作、标定结果或保证可达的六维目标。
五关节 JetRover 不具备任意 SE(3) 位姿能力；P2.4 必须检查位置/轴向/键控约束是否同时可行，
必要时研究基座/面板布置或受限目标族，不能只因 XYZ 落在点云内就给出任务 PASS。

## 6. Proposed Architecture — Scene Schema v1 草案

几何数据与 backend 解耦，首版 YAML（内部规范化 JSON），只支持可控 primitives。
Cabinet/PatchPanel 是带语义和基准 Frame 的**组件集合**，不直接等同一个实心 Box。
Box 的 pose 可旋转，因此 Oriented Box 不需要另设冲突的尺寸定义；Cylinder 局部轴为 +Z。
RJ45 Port 是目标语义和操作轴，**不会自动在 panel solid 中挖孔**。
若研究开口，必须明确用周边 boxes 构造孔洞，或在以后提供审查过的开孔 mesh。

### 6.1 字段与输入验证

| 字段 | v1 规则 |
| --- | --- |
| schema_version | 固定 1，未知版本/字段拒绝 |
| scene.id / provenance | ID 唯一；synthetic/measured/vision、hardware_verified 显式标记 |
| coverage / tool.status | 声明 partial_synthetic_fixture 或经核验的具体覆盖范围；工具 uncalibrated 时禁用 TCP 任务结论，不能推断几何完整 |
| units | 仅 length:m、angle:rad，mm/deg 拒绝，不靠猜测转换 |
| reference_frame | 首版必须 base_link，且与加载的 G2 模型一致 |
| frames | 每个 id/parent/pose 完整；同名、多父、缺失父、循环、游离图均拒绝 |
| pose | position_m 三个有限实数；quaternion_xyzw 四个有限实数且非零，norm 与 1 偏差≤1e-8 |
| quaternion 规范化 | 仅容许范围内归一化浮点误差；按 (w,x,y,z) 首个非零分量为正固定 q/-q 表示，负零规范为正零；原文件另存 raw hash，不静默修正明显错误 |
| entities | kind=cabinet/patch_panel；frame 引用有效；components 引用唯一 collision object |
| collision_objects | 唯一 ID；parent_frame 存在；局部 pose；box 全边长 size_m>0，cylinder radius_m>0/length_m>0 |
| ports | 唯一 ID、所属 panel/frame；操作轴采用统一约定；端口尺寸/插入深度可标未知，未知时禁止任务级评估 |
| clearance | robot_environment_m、self_m 非负有限；与数值 contact_epsilon_m 分开，不把 epsilon 或体素大小当工程安全裕量 |
| 资源/身份 | 记录 raw/canonical scene SHA256、规范化几何、robot/overlay/policy/tool 哈希、来源状态 |

验证布尔值不能当数字；拒绝 NaN/Inf、零/负尺寸、错误长度、重复 YAML key、不受限 alias 展开和未知单位。
拒绝把同一物体同时以 entity solid 和 component solid 加入碰撞世界，避免重复计数。
清晰区分实体和 Frame 的命名空间，所有引用必须可唯一解析；物体、端口、entity ID 各自唯一。
多场景首版使用独立、可完整展开的配置，共用同一 Schema；不引入隐式 YAML 继承/路径搜索。
以后共享模板时先确定展开规则，并将完整展开内容纳入 canonical hash。

### 6.2 明确标注的合成示例（仅 Schema 草案）

下面所有数值均是**人工合成测试参数，不是真实 IDC/RJ45 尺寸、网口坐标或安全间隙**。
柜体使用三面简化壳体，面板由四个 box 围成矩形测试开口；该开口不是实际 RJ45 标准尺寸。
它用于未来 B 的坐标/几何检查，不代表整柜、网线或工具建模完整，也不定义 C1/C2/C3 等级。

```yaml
schema_version: 1
scene:
  id: synthetic_demo
  provenance: {kind: synthetic, hardware_verified: false}
  reference_frame: base_link
  units: {length: m, angle: rad}
  coverage: partial_synthetic_fixture
  frames:
    - id: synthetic_demo/cabinet
      parent: base_link
      pose: {position_m: [0.45, 0.0, 0.20], quaternion_xyzw: [0, 0, 0, 1]}
    - id: synthetic_demo/panel
      parent: synthetic_demo/cabinet
      pose: {position_m: [-0.15, 0.0, 0.0], quaternion_xyzw: [0, -0.7071067811865476, 0, 0.7071067811865476]}
    - id: synthetic_demo/port_01_frame
      parent: synthetic_demo/panel
      pose: {position_m: [0, 0, 0], quaternion_xyzw: [0, 0, 0, 1]}
  entities:
    - id: cabinet_demo
      kind: cabinet
      frame: synthetic_demo/cabinet
      components: [cab_left, cab_right, cab_back]
    - id: panel_demo
      kind: patch_panel
      frame: synthetic_demo/panel
      components: [panel_top, panel_bottom, panel_left, panel_right]
  collision_objects:
    - id: cab_left
      type: box
      parent_frame: synthetic_demo/cabinet
      pose: {position_m: [0, 0.18, 0], quaternion_xyzw: [0, 0, 0, 1]}
      size_m: [0.30, 0.02, 0.40]
    - id: cab_right
      type: box
      parent_frame: synthetic_demo/cabinet
      pose: {position_m: [0, -0.18, 0], quaternion_xyzw: [0, 0, 0, 1]}
      size_m: [0.30, 0.02, 0.40]
    - id: cab_back
      type: box
      parent_frame: synthetic_demo/cabinet
      pose: {position_m: [0.14, 0, 0], quaternion_xyzw: [0, 0, 0, 1]}
      size_m: [0.02, 0.38, 0.40]
    - id: panel_top
      type: box
      parent_frame: synthetic_demo/panel
      pose: {position_m: [0, 0.055, 0], quaternion_xyzw: [0, 0, 0, 1]}
      size_m: [0.12, 0.03, 0.01]
    - id: panel_bottom
      type: box
      parent_frame: synthetic_demo/panel
      pose: {position_m: [0, -0.055, 0], quaternion_xyzw: [0, 0, 0, 1]}
      size_m: [0.12, 0.03, 0.01]
    - id: panel_left
      type: box
      parent_frame: synthetic_demo/panel
      pose: {position_m: [-0.045, 0, 0], quaternion_xyzw: [0, 0, 0, 1]}
      size_m: [0.03, 0.08, 0.01]
    - id: panel_right
      type: box
      parent_frame: synthetic_demo/panel
      pose: {position_m: [0.045, 0, 0], quaternion_xyzw: [0, 0, 0, 1]}
      size_m: [0.03, 0.08, 0.01]
    - id: synthetic_cylinder
      type: cylinder
      parent_frame: base_link
      pose: {position_m: [0.30, 0.10, 0.10], quaternion_xyzw: [0, 0, 0, 1]}
      radius_m: 0.015
      length_m: 0.08
    - id: oriented_test_box
      type: box
      parent_frame: base_link
      pose: {position_m: [0.26, -0.12, 0.12], quaternion_xyzw: [0, 0, 0.3826834323650898, 0.9238795325112867]}
      size_m: [0.08, 0.02, 0.06]
  ports:
    - id: port_01
      entity: panel_demo
      frame: synthetic_demo/port_01_frame
      outward_axis_local: [0, 0, 1]
      insertion_axis_local: [0, 0, -1]
      key_axis_local: [1, 0, 0]
      task_parameters_status: unmeasured
  clearance:
    robot_environment_m: 0.0
    self_m: 0.0
    contact_epsilon_m: 0.0000001
  tool:
    status: uncalibrated
    ee_frame: end_effector_link
    tcp_frame: null
```

该示例 panel/port 的 +Z 通过合成旋转指向 base -X，-Z 指向柜内。
零 margin 与 1e-7 m epsilon 只用于合成协议说明，不是实物运行参数；C 前须单独核验。
ports 中的外向轴与插入轴必须单位且相反，键控轴与它们正交，并符合 v1 约定；只凭法向不能唯一恢复键控方向。

## 7. Proposed Architecture — 接口与数据契约

下面只是拟议 API，不创建对应空模块。

```python
load_scene(path, *, reference_frame="base_link") -> SceneSnapshot
resolve_transform(scene, target_frame, source_frame) -> T_target_source
build_collision_model(parent_urdf, overlay, pair_policy, tool_spec) -> CollisionModelIdentity
CollisionBackend.load(model_identity, scene_snapshot, policy) -> BackendSession
BackendSession.check(q, *, auxiliary_positions, sample_index) -> StateCollisionResult
analyze_constrained_workspace(g2_prefix, scene, backend, grid) -> ConstrainedSamples
```

SceneSnapshot 保存不可变规范化 Frame 图/几何和两个 scene 哈希。transforms 模块不导入
MoveIt/FCL；scene_model 不知道 ROS Node。collision.py 是协议与 native 进程适配层，
加载/碰撞实现留在 C++ backend；场景渲染放到独立 scene_visualization.py。

StateCollisionResult 拟包含 sample_index、decision（CLEAR/COLLIDING/UNVERIFIED/ERROR）、
self_collision/environment_collision（未知时 null）、body pair、可选 contact/distance、
model/scene/policy/tool 身份、检查覆盖范围和耗时。仅在所声明覆盖完整、状态合法、
backend 成功、所有要求的 clearance 判据满足时才能 CLEAR；不能把异常当无碰撞。
未配置工具时注明 bare_gripper_only，不能对已持 RJ45 的实际操作宣称工具检查完成。

clearance 与接触策略独立于 scene 的原始几何。接触默认视为拒绝通过；边界附近使用
明确 epsilon 做数值保守决策，未能确定则 UNVERIFIED。MoveIt self-collision 默认 unpadded，
**给 robot-world 加 padding 并不自动实现 self margin**。若声明 self_m>0，后端必须
提供已验证的 pair-distance/代理策略，否则明确拒绝该策略。不要以“统一加 padding”
未经验证地承诺任意 mesh 的精确 Minkowski 安全包络。

ACM/允许接触对是独立版本化配置：只允许有装配或任务依据、经审查的具体 pair。
不能直接屏蔽所有相邻关节、所有固定连接 Link、夹爪对外部物体或“随机采样中未碰撞的对”。
固定部件内部装配关系可以形成候选过滤，但必须保存理由、检查范围和例外，不能影响
这些 Link 对机械臂其余部件/环境的检测。实体缺几何时直接报错；四个参考 Link 登记豁免。

数据身份由原 G2 NPZ/JSON 哈希 + 原运动学模型/配置哈希 + collision/mesh manifest/overlay +
canonical scene + auxiliary/gripper/tool + ACM/clearance/contact 策略 + backend 版本决定。
cache key 必须包含全部影响判定的内容，不能只用 EE XYZ。
joint5 虽不改变当前 EE XYZ，却会转动夹爪；同一 XYZ 的不同 q 可能有不同碰撞结果。

P2.3-D 原样读取 G2 q/position/rotation，不排序、不去重、不丢掉负 Z。
新结果用独立 P2.3 schema，保存 source_sample_index、decision_code、有效子集原行号、
逐状态分类/耗时与可选碰撞对，引用原 G2 数据；不把过滤结果伪装成原 G2 NPZ/JSON。
绘制子集必须用同一 original_indices 索引全部数组；2 cm voxel 仍用 G4 固定网格。

计数必须分别记录总 N、CLEAR、碰撞并集、UNVERIFIED、ERROR，以及 self/env/both。
self/env 会重叠，不得相加当并集。未知和错误仍留在总分母并单列；不能丢弃后提高成功率。
P2.3 的 valid ratio 指当前声明模型/场景/策略下的静态配置有效率；需要明确统计定义，
不能解释成路径成功率、RJ45 操作成功率或真实安全概率。

## 8. Verification & Acceptance Design

后续测试使用人工可解析小几何，再到机器人模型；不只靠点云观感。

| 类别 | 必测用例 | 验收证据 |
| --- | --- | --- |
| Scene 合法性 | 合法配置、缺失/重复 Frame、环、多父、错误引用、重复 ID/YAML key、未知字段/单位、非正尺寸、无效 quaternion | 明确错误定位到字段/ID；不静默转换或裁剪 |
| Transform | 非零平移+90°旋转、连乘与逆、非交换次序、parent/child 方向 | 已知点经手算变换；T*T^-1=I，R 正交、det≈1；绝对误差标准预先固定 |
| 再现性 | 同一 YAML 两次加载，YAML/规范化 JSON 身份规则 | 几何/Frame/规范化哈希一致；原文本哈希另记 |
| Geometry | 柜体空腔、面板实际孔洞、物体局部轴/原点、旋转 box、轴向 cylinder | 人工尺寸/相对位置可核对；Plotly 坐标轴叠加；语义 port 不得自动消除 panel 碰撞 |
| Native 最小验证 | 空场景与人工 box，整树载入，完整 mesh 数量，三组 q 的 EE/附件 FK | 构建/运行/载入日志；G1-A 阈值一致；不要求硬件/ROS 节点 |
| 碰撞基础 | 分离、相交、恰好接触、边界±epsilon、旋转体、多障碍 | 解析可判几何，接触策略前后一致；所有请求清空上次结果 |
| mesh 特例 | 完全包含但表面不相交、退化/缺失 mesh、缩放/非零 collision origin | 明确验证 backend 对所需实体语义的表现；无法保证时禁止该模型或返回 UNVERIFIED |
| 自碰撞与附件 | 非相邻臂段、臂-底盘/后壳/lidar、腕相机、夹爪-环境、合成 attached tool | 命中正确 body pair，不能只检查 EE；检查与不检查名单可追溯 |
| mimic/状态 | 改变 r_joint 的夹爪形状；joint5 改变姿态但同 XYZ；固定段 | 与完整树变换一致，不把 mimic 当独立采样量 |
| 覆盖/身份 | lidar 缺失、参考帧豁免、改 mesh/overlay/ACM/scene/tool、错误 G2 模型 | 缺失实体不通过；身份变化使缓存无效；不放宽 G2 校验 |
| Workspace | 相同行号一一映射、同数据跨场景、保留全量决策、复用固定网格 | 所有原行数守恒；valid indices 同时索引 q/position；未知/错误不丢弃 |
| 输出与失败 | 现有文件、半写结果、native 进程失败/超时、schema 错误 | 非零退出码/失败报告，无伪 PASS，不覆盖原数据或其他节点 |
| 语义边界 | C0 有自碰撞可能；无碰撞状态但不连通的配置集合；缺少 TCP | 不承诺完整路径、任务插拔或严格边界 |

primitive 解析用例应给出数值真值；实际机器人自碰撞真值需结合资源和装配审查确认，
不在 A 中编造“已知无碰撞姿态”。mesh 是否闭合不能单独当作碰撞/包含语义已正确的证明。
对照数值容差、距离与接触判据必须在 C 运行前固定，不能根据失败结果调到通过。

## 9. P2.3-B/C/D/E 实施与验收计划

### B — Parametric IDC Scene Modeling

- 输入：本 Schema、已验收 base_link 模型、明确标注的合成尺寸，之后可换实测数据。
- 首个任务：实现 transforms.py + scene_model.py 的严格解析/Frame 图/规范化快照，
  先用一个 synthetic_dev.yaml 验证 box/cylinder/旋转 box 和 cabinet/panel/port 的层级。
- 再增加 scene_visualization.py、inspect_scene.py，在相同 base_link 下显示对象/轴，
  可复用 G3 Figure 叠加经校验的 G2 小点集；不改 visualization.py，不接碰撞 backend。
- 输出：规范化 scene 摘要/哈希、Frame 变换清单、独立 HTML。实体复杂形状按组件集合表示。
- 难点：孔洞/空腔、frame 方向、单位与四元数、来源标识；不把合成场景写成实物机柜。
- 测试：第 8 节 Scene/Transform/Geometry/再现性；先小数据离线。
- 验收：合法数据确定性加载，非法输入拒绝，坐标轴和相对位置与解析值一致，
  原 G1–G4 与数据不变；用户验收 B 后才进入 C。

### C — Collision Checking（依赖 B 与机器人覆盖决策）

- 输入：不可变 SceneSnapshot、原 URDF、审查过的 collision overlay/mesh manifest、
  r_joint/工具声明、SRDF/ACM、clearance/contact 策略。
- C0 技术探针：在用户授权后，用当前 C++ 依赖构建最小离线程序，验证
  RobotModel→RobotState→PlanningScene/FCL；先比较整树 FK、载入几何计数和人工 primitive。
  不启动 move_group，不接机器人控制接口；不在本轮编写该程序。
- 然后实现 collision.py 协议与 cpp/collision_backend 的批处理 API、完整性门槛、
  self/world/附加工具分类与失败诊断；正负测试均须明确。
- 输出：逐配置判定与碰撞对/覆盖说明/版本身份，原模型内容只读。
- 难点：lidar 缺口、装配固有接触/ACM、原始 mesh 质量、完整树与 root 桥接、
  contact/distance/padding 语义、native ABI 和资源解析。
- 验收：第 8 节碰撞/状态/身份用例通过，无实体漏建，不伪造 CLEAR；
  若只是 partial 模型，只能作为显式受限诊断，不能通过整机 collision-free 的 C 验收。
  若 native 方案失败，给出原因再讨论 Direct FCL，不能静默更换后端。

### D — Collision-Free Workspace Sampling（依赖 B+C）

- 输入：相同 G2 原始数据、场景快照、通过验收的 backend/policy 和固定 VoxelGrid。
- 实施：遍历原 q，不重新生成 Monte Carlo 或重写 FK，保存所有原始行号与分类；
  先 100 点，再由用户执行 10,000 点；映射出无碰撞配置对应的 EE 点集与体素统计。
- 输出：新 schema NPZ/JSON、逐类计数/比例、加载/整树更新/窄相检测/I/O 耗时、
  模型/场景/原数据身份；原始、CLEAR、碰撞点分层图。
- 难点：同 XYZ 多 q、错误/未知分母、不同 gripper/tool/policy 导致结果不可比。
- 验收：行号对应、分类数守恒、同输入可复现，固定网格不变；没有新采样或丢弃失败行；
  collision-free 是在当前模型/场景/策略下的静态配置集合，不承诺从起点可到达。

### E — Scenario Benchmark（依赖 D）

- 输入：C0/C1/C2/C3 明确几何参数、相同原始 q/哈希、固定机器人安装/夹爪、
  同一 overlay/ACM/clearance/tool/backend/体素网格。真实值待测，不在 A 写四个假实物场景。
- C0：没有外部障碍的基线，仍检查自碰撞、臂-底盘和夹爪；有效率不预设为 100%。
- C1–C3：建议用同一侧壁/通道几何的间隙参数分级，保存 gap、位置、姿态、面板关系。
  如需可解释的逐步收紧，设计占据集合 O_C1⊆O_C2⊆O_C3，保持目标/机器人不变。
  只有这些包含关系和检测策略成立时，才预期同一数据上的有效配置集合不增；
  任意挪动物体的四个场景不能仅靠命名声称“越来越难”或保证比例单调。
- 输出：几何参数表、valid/collision/self/env/unknown/error 比例、分布、统一体素命中、
  计算成本与失败来源，CSV/JSON/Markdown 和可选场景对照图。
- 测试：配置身份完整、同源样本、参数/集合关系检查、统计定义一致，极端场景仍明确失败。
- 验收：场景分级可由配置重建，结论只覆盖实际数据与建模范围；未知/错误完整呈现；
  不把配置有效率直接称作轨迹或插拔成功率。由用户执行正式对照并审核结果。

## 10. 建议文件结构（全部为后续规划）

```text
workspace_analysis/
  docs/p23_architecture.md              # 本轮唯一新增项目文件
  config/scenes/synthetic_dev.yaml      # B：先一个明确合成场景
  config/scenes/c0_free.yaml            # E：实验参数确定后才新增
  config/scenes/c1_light.yaml
  config/scenes/c2_medium.yaml
  config/scenes/c3_heavy.yaml
  config/collision/policy.yaml          # C：ACM、coverage、margin、接触语义
  config/collision/overlay.yaml         # C：明确来源的派生碰撞描述，形式届时确定
  src/workspace_analysis/transforms.py  # B：SE(3) 与 Frame 图
  src/workspace_analysis/scene_model.py # B：规范化几何/语义和严格输入验证
  src/workspace_analysis/scene_visualization.py # B：独立叠加渲染
  src/workspace_analysis/collision.py   # C：协议、身份和 native 适配
  src/workspace_analysis/constrained_workspace.py # D：按原 q 行过滤/统计
  cpp/collision_backend/                # C：本机 MoveIt/FCL 小型 CMake target
  scripts/inspect_scene.py              # B
  scripts/analyze_constrained_workspace.py # D
  tests/test_transforms.py              # B
  tests/test_scene_model.py             # B
  tests/test_collision.py               # C，另含 native 小几何测试
  tests/test_constrained_workspace.py   # D
  results/                             # 新前缀实验输出，禁止覆盖旧数据
```

不创建空实现；native build 可使用已有 ROS/CMake 依赖和当前工作空间环境，
不重建或移动官方工作空间。是否需要单独 ament package 是 C 构建探针后的具体选择，
不在 A 擅自建立新工作空间、ROS package 或安装绑定。

## 11. Unverified Dependencies — 进入碰撞实现前的门槛

| 未验证项 | 当前证据/缺口 | 下一步归属 |
| --- | --- | --- |
| Native 离线构建/运行 | 有 headers/libs/CMake/JSON；没有编译/调用实测 | C 最小构建与 RobotModel/FK 对照 |
| JetRover SRDF/ACM | 搜索路径内无现成配置；相邻/固定件未审查 | C 显式 pair 表与理由，不自动全屏蔽 |
| 整机几何完整 | lidar collision 缺失；工具/网线未建模 | C 前确认 overlay 与研究 scope |
| mesh 拓扑/近似误差 | 只验证存在、binary STL、有限顶点与数量 | C 资源品质/包含与接触用例；实物覆盖待测 |
| margin/distance/包含语义 | 没有实际调用，尤其 self unpadded 与 mesh 包含情形 | C 人工解析测试，不承诺精确包络 |
| 整树 FK | G1-A 只核验 EE 链，RobotState 相机/夹爪分支未对照 | C 与原 FK/独立参考核对 |
| 网络最新分支 | 工作区未跟踪内容导致停止同步 | 用户整理工作区后按原规则 fetch/ff-only |
| 跨机复现 G4 | validation_report.py 未跟踪；目前仅本机可用 | 用户后续决定版本管理，不在 A 提交 |

不能把“lib 可找到”“文件存在”“URDF 可加载”分别扩展为“编译已通过”“物理几何完整”
或“碰撞检测已通过”。这些限制不阻止 B 的独立场景开发，但限制 C/D 的正式结论。

## 12. Pending Hardware Measurements — 待测项目与基准

| 项目 | 坐标/参考基准 | 待记录内容 |
| --- | --- | --- |
| 机柜安装 | base_link→cabinet_frame | 相对位置/姿态、测量方法与不确定度；真实地面另设基准 |
| 柜体壳/障碍 | cabinet_frame | 板厚、空腔、开口、隔板、突出件，物体局部轴/尺寸 |
| 配线架/面板 | cabinet_frame→panel_frame | 安装偏置、板厚、可用开口而非仅外接盒 |
| 端口 | panel_frame→port_frame | 入口中心、外法向、键控方向、周边边框/锁扣空间；不能只量 XYZ |
| 工具/TCP | end_effector_link→tcp_frame | 插头尖端与夹持姿态、夹爪开度关系、外形、安装偏置、重复性 |
| 夹爪/附件覆盖 | 对应 URDF Link | 实物外包络与 STL、lidar、相机、支架、线缆的差异 |
| 相机外参 | base_link 或运动的相机安装 Link→camera_frame | 标定 ID、时间戳、精度；不能从视觉坐标直接当 base 坐标 |
| 操作行程/裕量 | port/TCP 局部轴 | 接近/插入/撤离距离、锁扣/线缆需求与可接受误差，P2.4 决定 |

本轮没有实测数据，不把合成示例、URDF base_joint 或 2 cm 网格当作这些测量值。
硬件误差、动态障碍和柔性线缆不包含在首版静态刚体碰撞结论中，后续需明确处理范围。

## 13. Decisions Needed Before Implementation

审核 A 后，建议先批准 **B 的严格 Schema/Frame + 合成场景可视化**，它不需要提前
解决全部实物测量或碰撞依赖；本轮停止，不自动实施。

需要用户确认的关键决策：

1. 是否接受 port +Z 向外、插入 -Z、TCP +Z 推进及键控 X 的约定？改变应在 B 固定前确定。
2. 是否接受 C 首选离线 C++ MoveIt PlanningScene/FCL + Python JSON 适配，先做构建探针？
3. lidar 补充采用审查后的现有 visual mesh 引用还是实测代理？是否先限定裸夹爪研究，
   何时纳入真实 RJ45/线缆？不解决时不能标为整机完整。
4. 机柜/面板采用哪个测量基准、由谁提供数据；首版 B 可先使用明确合成的测试夹具。
5. C 前如何审查 ACM、确定 margin/contact 判据？实际数值尚未选定，不预批准合成示例值。
6. 未跟踪 G4 helper/文档如何纳入后续版本管理，何时恢复安全 Git 同步？本轮不提交。

后续任务与论文价值必须由基线、场景定义、失败分析和正式对照实验支撑，
不预先把该架构或第三方碰撞库称作已验证的新算法。

## 14. 官方参考与本轮检查范围

- [MoveIt Humble Planning Scene](https://moveit.picknik.ai/humble/doc/examples/planning_scene/planning_scene_tutorial.html)：
  C++ 碰撞接口、ACM、self unpadded 语义。在线监视与离线快照设计的区别见本文第 4 节。
- [MoveIt Humble Planning Scene Monitor](https://moveit.picknik.ai/humble/doc/examples/planning_scene_monitor/planning_scene_monitor_tutorial.html)：在线同步职责。
- [MoveIt Humble PlanningScene 头文件](https://github.com/moveit/moveit2/blob/humble/moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.h)：
  与本机已安装头文件交叉参考；以本机 2.5.9 接口为实现依据。
- [FCL 0.7.0 官方仓库](https://github.com/flexible-collision-library/fcl/tree/0.7.0)：mesh BVH、primitive、碰撞和距离接口能力；不代表本机调用已验证。

已完成只读检查：Git/软链接、现有模块和全部相关测试、实际 G4 完成 bundle、
当前 Xacro 展开/29 Links 清单/24 collision 资源与 mesh 指纹、官方其他运动学服务源码、
本地 SRDF/MoveIt 配置搜索、ROS 包与系统 C++/Python 依赖、PlanningScene/RobotState 头文件。
本轮没有新增测试或碰撞实现；模型/数据等既有文件的哈希在文档写入前后核对不变。
对文档中的合成 YAML 做了临时只读一致性检查：可解析，ID/组件引用、尺寸和 quaternion
一致；合成 panel 的外向法向确为 base -X。这不是 B 的场景解析模块或碰撞测试实现。

**P2.3-A Technical Audit & Architecture Design 完成，等待用户审核后再实施 P2.3-B。**

## 附录 A：实际 collision mesh 清单与资源指纹

以下均为 package://jetrover_description/ 下的路径。所有条目 origin xyz/rpy=0，scale 默认 1。
这是只读资源审计，不代表碰撞程序或实际包络已验收。四个参考 Link 之外唯一实体缺失为 lidar_link。

| Link | Mesh 相对路径 | 三角形数 | 文件 SHA256 |
| --- | --- | --- | --- |
| base_link | meshes/mecanum/base_link_collision.stl | 1980 | `d6f1d30e125ad3bcbde5f8619a93257413c17a4af3bb3cbe73ba0bec8aa418cb` |
| wheel_left_front_link | meshes/mecanum/wheel_left_front_link_collision.stl | 660 | `1e20a25ed14d9d3faabc9c0a5286ecc6ec477f447fa6721bea2003d5fb1488ef` |
| wheel_left_back_link | meshes/mecanum/wheel_left_back_link_collision.stl | 660 | `1e20a25ed14d9d3faabc9c0a5286ecc6ec477f447fa6721bea2003d5fb1488ef` |
| wheel_right_front_link | meshes/mecanum/wheel_right_front_link_collision.stl | 668 | `265671256dd51133f5f9c2510dac6f5f6ee1982c9aff089d38bbd0c747f446cb` |
| wheel_right_back_link | meshes/mecanum/wheel_right_back_link_collision.stl | 668 | `265671256dd51133f5f9c2510dac6f5f6ee1982c9aff089d38bbd0c747f446cb` |
| imu_link | meshes/common/imu_link.stl | 280 | `4016bd52396df07cbe3222be3f571aa7d9b752fa275ffc1f1878ac3af3cf2a07` |
| camera_connect_link | meshes/arm/camera_connect_link.STL | 354 | `f672b8554113ae682dc7028f8bca79dcae579a1f528fdad8e8fcfb4cf78e4e82` |
| depth_cam_link | meshes/arm/depth_cam_link_collision.stl | 1011 | `446b56296c35f4f7e2edc9a711e2ac4ad0e5202be998c601b2b66ae3712d810a` |
| back_shell_black_link | meshes/common/back_shell_black_link_collision.stl | 1977 | `9191a265e3ff6c7358721cf710637c73b2991b572408b3705e0027678ab06f5d` |
| back_shell_green_link | meshes/common/back_shell_green_link_collision.stl | 1969 | `2cd793f40561d1fe54f0daafc26769d0836a04f1c30eeb4d8258f02224a8bc5e` |
| link1 | meshes/arm/link1.STL | 1716 | `4fcb56ef62f7dedbe6382cdeb73ca27539051681eeb1f36a92d05269763b41a3` |
| servo_link1 | meshes/arm/servo_link1_collision.stl | 405 | `0aa32832cbc0cec52cdd6dafd458576f2a083534d37975db3f508320cf0b86bc` |
| link2 | meshes/arm/link2_collision.stl | 1144 | `65e347d25a91abd103370d6933e9ec177e72787f2f9981b719ea44b1b60e32c4` |
| link3 | meshes/arm/link3_collision.stl | 1135 | `1718fb80ed11489d71a7ce9beef23014897c0608f62e489142d5f59b6bd10944` |
| link4 | meshes/arm/link4_collision.stl | 487 | `475179a9c7bf24c10a558fec1f196466195b0bc883ce0209f83a056381e471cf` |
| servo_link2 | meshes/arm/servo_link2_collision.stl | 360 | `c3d7f0ba39ab708dc1576c61da87a0a55bcf9afd6d989bf2cf47ef14197b155e` |
| link5 | meshes/arm/link5.STL | 813 | `3465b3928cccc2462c25f21c86f887e8c0515784e2799774d93eead48c48284c` |
| gripper_link | meshes/arm/gripper_link_collision.stl | 683 | `5d8c8714db1ce14df4081c5bbe60fcd0024086874a44a4903eb6877f0039f62f` |
| r_link | meshes/gripper/r_link.STL | 384 | `6df08ecd230db46e66655c2d6f272b7af2307ddf16d6e1ff2876d458f4bcae9d` |
| l_link | meshes/gripper/l_link.STL | 485 | `196519b292059564c3dc9f62cc2efe50ae4ea425c91cc1d54ecc18c6a2b03db6` |
| l_in_link | meshes/gripper/l_in_link.STL | 260 | `ec13b7c4ce769cf3c6521ad6b039d52b45fb7f8c9595a83b67c9d11a810224b0` |
| l_out_link | meshes/gripper/l_out_link.STL | 1601 | `3db09b7aa1de7a5bde3c49ca0fec168485957ad65756223ec118500918ced86d` |
| r_in_link | meshes/gripper/r_in_link.STL | 260 | `1a76838849e8aaefa9ba441a61a73decf4dd835a4c6cbd7af02432d7d051d0df` |
| r_out_link | meshes/gripper/r_out_link.STL | 1608 | `d51971e443fc653d0bb2dac8998073d715dc362fb351ae2ba204fe5e9466d71a` |

24 条资源的审计清单指纹：
`5d97c485ade8548dafa03b8b83d11892c470d69d2daf48df2d6e97ee57d298de`。

计算约定：按展开 XML Link/collision 顺序，构成包含 link、uri、sha256、origin（原 XML 属性字符串）、
scale（缺省记为字符串 1 1 1）的记录列表，对 json.dumps(sort_keys=True,separators=(',',':'))
的 UTF-8 字节做 SHA256。该指纹不包含 ACM、工具或策略，不能冒充未来完整 collision_model_sha256。
