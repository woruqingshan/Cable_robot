# P2.3-C1 — Offline Collision MVP

P2.3-C1 Collision MVP Completed — Pending User Review.

本轮完成 Functional MVP，不是 Validated Collision Backend。真实计算链路为
Python 已验证 SceneSnapshot / G1 全关节状态 → 持久 C++ JSONL 进程 →
MoveIt RobotModel / RobotState / PlanningScene / CollisionEnvFCL → Python 原始行诊断。
没有 Direct FCL 替代实现，没有 AABB/EE 点近似、IK、规划或 ROS 节点。

## 1. 初始状态及依赖

分支 feat/workspace-planning，起始 HEAD 9aea0d8。原有未提交修改
docs/s1_idc_prototype.md 来自 R2.1；docs/develop/ 未跟踪；均保留。
起始完整回归 403 passed, 1 warning, 0 skipped，11.64 s。

实际依赖：Ubuntu 22.04、Python 3.10.12、GCC 11.4.0、CMake 3.22.1、
MoveIt core/ros_planning 2.5.9、FCL 0.7.0、Eigen 3.4.0、
Assimp Debian 5.2.2（pkg-config 报 5.2.0）、nlohmann_json 3.10.5、OpenSSL 3.0.2。
没有安装、升级或覆盖系统依赖。构建使用系统 /usr/bin/python3 执行 ament 的 CMake 工具。

本机 API 与某些版本示例不同：resource_retriever 的头文件是 retriever.hpp；
PlanningScene 使用 allocateCollisionDetector(FCL::create())，随后回读 detector 必须等于 FCL。
首轮编译发现并修正这些接口差异，没有改换后端。

## 2. 文件与单向依赖

```text
cpp/collision_backend/
  CMakeLists.txt
  include/engine.hpp
  src/engine.cpp
  src/main.cpp
  tests/native_tests.cpp
src/workspace_analysis/collision/
  __init__.py
  protocol.py
  client.py
  diagnostics.py
scripts/probe_collision.py
tests/test_collision_protocol.py
tests/test_collision_client.py
docs/c1_collision_mvp.md
```

唯一调整的既有测试是 test_architecture.py 的依赖白名单：加入 collision 可依赖
geometry/robot/scene/workspace/collision，并允许 experiments 使用 collision。
没有改任何旧数值断言、阈值、fixture 或 skip。现有 403 项仍全部运行。
原有 geometry/robot/scene/workspace/viz 生产实现和 Scene Schema/S1 YAML 不变。

collision.protocol 消费 Snapshot，使用既有 object_transform 完整展开父链，不实现另一套 Python FK 或碰撞判断。
collision.client 管理一个子进程与串行请求、超时和退出；diagnostics 复用 load_model、load_results(model=model)
及 PyKDL，只做输入复核、FK 门槛、记录和统计。没有全局进程或 import 时启动行为。

## 3. URDF、SRDF 与资源身份

Python 使用现有 config/workspace.yaml 内 MACHINE_TYPE=JetRover_Mecanum、LIDAR_TYPE=A1 展开模型，
直接把与 G1 同一个完整 XML 传入 Native，没有另存简化机器人模型。
Native 检查 XML SHA256、资源存在性、每个碰撞 Mesh 的字节 SHA256、scale、origin、实际加载的 shape 和三角形数。
只支持本地 file/package 资源；缺失、空资源、丢失 shape 或空 mesh 均不能 READY。

SRDF 在内存创建，仅有与 URDF 同名的空 robot 元素；没有 group、virtual joint 或 disable_collisions。
PlanningScene 的 ACM 显式 clear，大小 0；不自动允许相邻、固定连接、夹爪或随机抽样碰撞对。
MoveIt 自动假设固定根关节，故 MoveIt joint 数包含一个额外虚拟根，URDF joint 数不包含它。
完整组检测，不指定 arm group；使用 unpadded CollisionEnv，零 padding，不实现正安全间隙。
请求正 robot_environment_m/self_m 会明确拒绝，而不是忽略。
Scene contact_epsilon_m 尚未作为 FCL 接触判据配置；当前使用 FCL 原始布尔接触语义，C2 需明确该策略。

每个 QUERY 需要全部 28 个 URDF joint 值（含 fixed=0 和 mimic）。
Native 先校验完整键集、有限数值、URDF 限位和 mimic 关系，再重置默认状态、
设置独立变量、更新全部 Link/collision transform，并核对 MoveIt 使用的 mimic 值。
初始化阶段还回读 MoveIt 的全部变量上下限，与 URDF 限位逐项比较，差异即阻止 READY；
结果存于 robot_audit.json 的 moveit_variable_bounds。
两个新的 CollisionResult 分别 clear，然后调用真实 checkSelfCollision 与 checkRobotCollision。
接触对按 self/environment 分类；每对最多返回 1 个接触，总上限 10,000，返回截断提示。
该接触点限制不改变 collision_detected 布尔值，不能将接触列表当作无条件完备的接触流形。

模型根为 base_footprint。^root T_base_link 的 Z=0.11609108215767461 m；
环境对象初始化时只转换一次至 model root，回读审计和 FK 输出转换回 base_link。
Native 拒绝会运动的 root→base 链，避免把固定停靠场景误用于移动底盘。
EE 仍为 end_effector_link；不是插头尖端或经标定 TCP。

### 实际模型清单

实际：29 Links / 28 URDF Joints / 29 MoveIt Joints（含虚拟根）。
24 碰撞几何 = 24 实际 shapes，全部 Mesh；共 21568 个三角形。

| Link | 碰撞几何数 |
|---|---:|
| back_shell_black_link | 1 |
| back_shell_green_link | 1 |
| base_footprint | 0 |
| base_link | 1 |
| camera_connect_link | 1 |
| depth_cam_frame | 0 |
| depth_cam_link | 1 |
| end_effector_link | 0 |
| gripper_link | 1 |
| imu_link | 1 |
| l_in_link | 1 |
| l_link | 1 |
| l_out_link | 1 |
| lidar_frame | 0 |
| lidar_link | 0 |
| link1 | 1 |
| link2 | 1 |
| link3 | 1 |
| link4 | 1 |
| link5 | 1 |
| r_in_link | 1 |
| r_link | 1 |
| r_out_link | 1 |
| servo_link1 | 1 |
| servo_link2 | 1 |
| wheel_left_back_link | 1 |
| wheel_left_front_link | 1 |
| wheel_right_back_link | 1 |
| wheel_right_front_link | 1 |

下列关节 origin 是父坐标中的 xyz / XYZW，axis 在局部 joint frame；从程序审计读取，未用于硬编码计算。

| Joint | 类型 | 父→子 | origin xyz / XYZW | axis | 限位 [rad] / mimic |
|---|---|---|---|---|---|
| back_shell_black_joint | fixed | base_link → back_shell_black_link | [-0.00800423021156377, 0.000186898416208928, 0.0] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| back_shell_green_joint | fixed | base_link → back_shell_green_link | [-0.0080042, 0.0001869, 0.0] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| base_joint | fixed | base_footprint → base_link | [0.0, 0.0, 0.11609108215767461] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| camera_connect_joint | fixed | link4 → camera_connect_link | [-0.0507060266977644, 0.0, 0.0505384841187764] / [0.0, 0.0, -0.7071067811865451, 0.7071067811865499] | [0.0, 0.0, 0.0] | — |
| depth_cam_joint | fixed | camera_connect_link → depth_cam_link | [0.0, 0.0, 0.014475] / [0.5000000000000001, -0.5, 0.49999999999999994, 0.49999999999999994] | [0.0, 0.0, 0.0] | — |
| depth_cam_joint_sim | fixed | depth_cam_link → depth_cam_frame | [0.0, 0.0, 0.0] / [-0.5, 0.4999999999999999, -0.5, 0.5000000000000001] | [0.0, 0.0, 0.0] | — |
| end_effector_joint | fixed | link5 → end_effector_link | [0.0, 0.0, 0.08] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| gripper_joint | fixed | link5 → gripper_link | [0.0, 0.0, 0.0054388] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| imu_joint | fixed | base_link → imu_link | [0.0048416, 0.011168, -0.0057398] / [0.7071067811865476, -0.7071067811865475, -4.329780281177466e-17, 4.329780281177467e-17] | [0.0, 0.0, 0.0] | — |
| joint1 | revolute | base_link → link1 | [0.0251328065010765, 0.0, 0.0774026880954513] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, -1.0] | [-2.09, 2.09] |
| joint2 | revolute | servo_link1 → link2 | [0.0, 0.0, 0.0338648012164686] / [0.0, 0.0, 0.0, 1.0] | [0.0, 1.0, 0.0] | [-2.09, 2.09] |
| joint3 | revolute | link2 → link3 | [0.0, 0.0, 0.129416446394797] / [0.0, 0.0, 0.0, 1.0] | [0.0, 1.0, 0.0] | [-2.09, 2.09] |
| joint4 | revolute | link3 → link4 | [0.0, 0.0, 0.129444631186569] / [0.0, 0.0, 0.0, 1.0] | [0.0, 1.0, 0.0] | [-2.09, 2.09] |
| joint5 | revolute | servo_link2 → link5 | [0.0, 0.0, 0.0] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, -1.0] | [-2.09, 2.09] |
| l_in_joint | revolute | gripper_link → l_in_link | [-0.0071623, 0.0049403, 0.046019] / [0.0, 0.0, 0.0, 1.0] | [0.99996, 0.0046983, -0.0070439] | [-1.57, 1.57] / {'joint': 'r_joint', 'multiplier': -1.0, 'offset': 0.0} |
| l_joint | revolute | gripper_link → l_link | [-0.0073352, 0.013977, 0.027057] / [0.0, 0.0, 0.0, 1.0] | [0.99997, 0.0055811, -0.0058397] | [-1.57, 1.57] / {'joint': 'r_joint', 'multiplier': -1.0, 'offset': 0.0} |
| l_out_joint | revolute | l_in_link → l_out_link | [0.00020427, 0.0, 0.028999] / [0.0, 0.0, 0.0, 1.0] | [0.99996, 0.0046983, -0.0070439] | [-1.57, 1.57] / {'joint': 'r_joint', 'multiplier': 1.0, 'offset': 0.0} |
| lidar_frame_joint | fixed | lidar_link → lidar_frame | [0.0, 0.0, 0.0] / [0.0, 0.0, 1.0, 6.123233995736766e-17] | [0.0, 0.0, 0.0] | — |
| lidar_joint | fixed | base_link → lidar_link | [0.0900034859353204, 0.0, 0.0405195774179554] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| r_in_joint | revolute | gripper_link → r_in_link | [-0.0071623, -0.004943, 0.046019] / [0.0, 0.0, 0.0, 1.0] | [-0.99996, 0.0046987, 0.0070436] | [-1.57, 1.57] / {'joint': 'r_joint', 'multiplier': -1.0, 'offset': 0.0} |
| r_joint | revolute | gripper_link → r_link | [-0.0073166, -0.013992, 0.027059] / [0.0, 0.0, 0.0, 1.0] | [0.99996, -0.004699, -0.0070434] | [-1.57, 1.57] |
| r_out_joint | revolute | r_in_link → r_out_link | [0.00020427, 0.0, 0.028999] / [0.0, 0.0, 0.0, 1.0] | [-0.99996, 0.0046987, 0.0070436] | [-1.57, 1.57] / {'joint': 'r_joint', 'multiplier': 1.0, 'offset': 0.0} |
| servo_joint1 | fixed | link1 → servo_link1 | [0.0, 0.0, 0.0] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| servo_joint2 | fixed | link4 → servo_link2 | [0.0, 0.0, 0.0544833339503674] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| wheel_left_back_joint | fixed | base_link → wheel_left_back_link | [-0.0975130776687404, 0.112129499199832, -0.0678410821576746] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| wheel_left_front_joint | fixed | base_link → wheel_left_front_link | [0.0975130769499663, 0.11212949984639, -0.0678410821576751] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| wheel_right_back_joint | fixed | base_link → wheel_right_back_link | [-0.097513076949967, -0.11212949984639, -0.0678410821576744] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |
| wheel_right_front_joint | fixed | base_link → wheel_right_front_link | [0.0975130776687398, -0.112129499199832, -0.0678410821576747] / [0.0, 0.0, 0.0, 1.0] | [0.0, 0.0, 0.0] | — |

碰撞 Mesh 的 URI、字节哈希、scale、origin、加载后的 shape origin/三角形数，见本轮
robot_audit.json 的 geometry/loaded_shapes 字段（每个碰撞几何一条）。
本机全部 scale=(1,1,1)、collision origin 为单位变换。LiDAR 的 lidar_link 有视觉却没有碰撞几何，
明确列入 missing_visual_collision_links；base_footprint/depth_cam_frame/end_effector_link/lidar_frame
也没有实体碰撞几何，它们是参考帧，不凭其存在声称实体已覆盖。

## 4. FK 与完整状态验证

每次诊断初始化后先查询三组 G1 姿态，与 PyKDL 对比；任一位置误差>1e-5 m 或角误差>1e-4 rad，
则阻止正常诊断，并为所有输入行记录 ERROR，不把失败丢掉。实际误差：

| 姿态 | 位置误差 [m] | 旋转角误差 [rad] |
|---|---:|---:|
| Zero | 0.0 | 0.0 |
| Nonzero | 0.0 | 0.0 |
| Near-limit | 0.0 | 0.0 |

真实 Native 集成测试另外对三组姿态 × r_joint=0/0.3 的所有 29 个 Link，
与测试专用 NumPy/SciPy URDF origin×motion 连乘参考对比（atol=1e-10，rtol=0），均通过。
包含 fixed、mimic、gripper、腕相机及 root/base 转换；不是只核对 EE。
这仍是软件模型一致性检验，不代表实物精度。

## 5. 场景与人工真值测试

S1 通过原 load_scene 得到 SceneSnapshot，Python 展开 38 个对象，保留 ID、完整边长和完整矩阵。
Native 独立按 canonical frame graph 复核平铺姿态与尺寸，再加入 PlanningScene World。
回读真实 World 的 shape 尺寸/global_shape_poses，换回 Base，与 Python 输入逐一对照。
没有增加 Cabinet/Router 分组或 9 个 Port/箭头实体。
正负父 Frame 旋转、合成场景 cylinder 的 radius/length 也通过真实后端回读。

Native CTest 可执行程序包含以下人工真值断言，均通过：

- 单机器人 Box + 远离环境 Box：raw self=false、environment=false。
- 离轴旋转 Box 对环境：q=1 → q=0 → q=1，环境结果 false → true → false；验证连续更新和清理。
- 重叠机器人两 Link + 远离环境：self=true、environment=false。
- 重叠机器人 + 环境重叠：self=true、environment=true。
- 越界状态报错；其后的合法状态可由 Native 正确计算。
- 错误 URDF 哈希、缺失 Mesh 明确失败。

人工夹具是 Native 测试专用简化机器人，不替换 JetRover；没有为它增加豁免，也没有把它的完整性推广到实机。
CTest 为 1 个聚合 executable 测试，结果 1/1 PASS；不将内部断言数量伪报为多项 CTest。
另有 Python 驱动的真实 Native 集成测试，不能由 Mock 代替。

## 6. JSONL 协议与 Python API

Envelope：protocol_version=1、唯一字符串 id、op。Native 不接受未知请求选项。
INIT：完整 urdf_xml/hash、Base/EE/joint_order、SceneSnapshot canonical JSON/hash、平铺对象。
READY：模型/场景/几何/政策/SRDF/backend 二进制 SHA256，以及整树、几何和 World 审计。
QUERY：完整 joint_positions；响应包含相同 id/identity、self/environment/collision_detected、
colliding_pairs、decision、使用状态、Base 下所有 Link 变换与 EE 变换、耗时。
SHUTDOWN：STOPPED，客户端回收自己的直接子进程；不使用 pkill/名称匹配。

Backend stdout 只写 JSONL，MoveIt/错误日志写 stderr。日志存临时文件以免 stderr pipe 满阻塞。
客户端是持久进程，100 行不创建 100 个后端；串行非阻塞 pipe I/O 有有界读写等待。
超时、异常退出、Malformed JSON、重复键、ID/身份不一致均报错并使连接不可继续用于计算，
避免把延迟响应误配给下一行。Native 请求 id 不可重复。
资源释放、退出码、正常 SHUTDOWN 分别记录；仅进程被回收不等于正常退出。

```python
from workspace_analysis.robot.model import load_model
from workspace_analysis.scene.schema import load_scene
from workspace_analysis.collision.client import CollisionClient

model = load_model('config/workspace.yaml')
scene = load_scene('config/scenes/idc_prototype_v0.yaml')
with CollisionClient('/absolute/path/to/collision_backend') as client:
    ready = client.initialize(model, scene)
    result = client.query(model, [0,0,0,0,0], auxiliary_positions={'r_joint': 0.0})
    print(result['self_collision'], result['environment_collision'], result['decision'])
```

Python 的协议状态包含 CLEAR/COLLIDING/UNVERIFIED/ERROR，但 C1 生产政策不会发出 CLEAR：

| 状态 | C1 含义 |
|---|---|
| COLLIDING | 当前声明几何中 FCL 检测到 self 或 environment 碰撞，二者保留独立布尔值 |
| UNVERIFIED | 当前几何未检测到碰撞，但覆盖/ACM/工具/开度/clearance 审核不完整 |
| ERROR | 输入、模型、进程或协议失败；未知原始布尔值使用 null，不冒充 false |
| CLEAR | 保留将来经过范围/覆盖/政策审核后的语义，本轮不产生 |

当前 identity 包含二进制/URDF/实际 Mesh/Scene/SRDF/政策身份。系统动态库的精确文件哈希不在协议锁定范围；
依赖版本记录在构建日志和环境说明中，不承诺跨版本逐位一致。长时间共享客户端或 Native 进程重载不是 C1 范围。

## 7. 100 行诊断结果及解释

运行目录：`/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c1/collision_probes/20261103T101535Z_n100_2cec62ce`。

原始 G2 100 行全部保留，sample_index=0..99，原 q 顺序不变；其中 14 行负 Z 未过滤。
不按 EE 坐标合并关节配置；r_joint=0，mimic 由原 G1 语义生成并由 Native 复核。

| 指标 | 结果 |
|---|---:|
| total_samples | 100 |
| colliding_samples | 100 |
| self_collision_count | 100 |
| environment_collision_count | 2 |
| both_collision_count | 2 |
| unverified_count | 0 |
| error_count | 0 |
| CLEAR | 0 |
| 平均 Native 查询 [s] | 0.0027707066499999995 |
| p95 Native 查询 [s] | 0.0035128631499999996 |

并集计数=100+2−2=100，不能将 self/env 简单相加。环境碰撞对应原始行 8 和 73，
接触对象包括 cab_bottom、Router 2/3 前面板/薄壳与夹爪部件，完整名称保存在 per_sample.csv/diagnostics.json。
Zero 单条示例为 self=true、environment=false、COLLIDING。

持续出现的 raw self pairs（仅作为 C2 审核候选，不自动豁免）：

| first | second | 100 行出现次数 |
|---|---|---:|
| back_shell_black_link | back_shell_green_link | 100 |
| back_shell_green_link | base_link | 100 |
| back_shell_green_link | link1 | 100 |
| gripper_link | l_in_link | 100 |
| gripper_link | link5 | 100 |
| gripper_link | r_in_link | 100 |
| l_in_link | l_out_link | 100 |
| l_link | l_out_link | 100 |

这类装配/相邻几何重叠说明空 ACM 下会产生大量持续碰撞。100 个 COLLIDING 不证明真实机械臂全部姿态不可用，
也不能通过自动加 ACM、忽略底盘/夹爪来人为增加 CLEAR。需在 C2 逐对审核几何与接触设计。
工具/RJ45、真实开度和 LiDAR 缺失还会造成漏检范围；任一无碰撞布尔结果也不能表述为整机实物安全。

输出包含 diagnostics.json（READY 审计、FK 门槛、输入证据、100 行和清理）、per_sample.csv、summary.json。
summary 为最后发布的统计；.incomplete 保留表示中途失败。输出目录必须不存在，拒绝覆盖。
后端初始化失败仍为已读取的全部 100 行记录 ERROR；进程/协议失效后连接不复用，后续行仍保留 ERROR。
失败行不参与已知碰撞集合，但保留在 total_samples/ERROR 分母中。CSV 中 unknown 布尔值为空，对应 JSON null。
这些是诊断产物，不是正式筛选、无碰撞工作空间或轨迹可行性结果。

## 8. 构建、测试与用户验收

独立构建，不使用 colcon，不写系统或既有 ROS workspace。新建唯一 run ID：

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
C1_RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)_$(.venv/bin/python -c 'import uuid; print(uuid.uuid4().hex[:8])')"
C1_BUILD_DIR="$PWD/results/c1/build_checks/$C1_RUN_ID"
mkdir -p "$C1_BUILD_DIR"
cmake -S cpp/collision_backend -B "$C1_BUILD_DIR/build"   -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build "$C1_BUILD_DIR/build" -j2
ctest --test-dir "$C1_BUILD_DIR/build" --output-on-failure
export C1_BACKEND="$C1_BUILD_DIR/build/collision_backend"

# 单条 Zero 查询；输出目录自动创建唯一 run ID。
.venv/bin/python -B scripts/probe_collision.py --backend "$C1_BACKEND" --q 0 0 0 0 0

# 原始 100 行诊断，默认 S1 Scene；不指定 output-dir 则自动使用唯一目录。
.venv/bin/python -B scripts/probe_collision.py --backend "$C1_BACKEND"   --input-prefix results/workspace_n100_seed42

# 可显式加 --output-dir results/c1/collision_probes/<new_run_id>，但目录必须不存在。
# 全量测试包含真实 Native 集成，必须设置上面的 C1_BACKEND；缺失时失败而不是 skip/mock。
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest   -q -rs -p no:cacheprovider tests
```

构建/人工测试实际 PASS；完整 Python 回归 **429 passed, 1 warning, 0 skipped**（13.29 s），
其中原有 403 项 + 新增 26 项。Native 聚合 CTest 1/1 PASS（0.02 s）。
Python 中的进程故障测试用受控 fake worker；真实 JetRover/资源/场景/FK 测试均调用本次编译的 C++ 后端。
没有用 fake worker 的通过替代 Native 功能完成。额外验证了初始化失败时保留 100 行 ERROR 以及禁止覆盖。

本轮现成产物（可直接用于复核）：

- build: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c1/build_checks/20261103T100756Z_7986f5da`
- backend: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c1/build_checks/20261103T100756Z_7986f5da/build/collision_backend`
- single: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c1/collision_probes/20261103T101535Z_single_a601e348`
- n100: `/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/c1/collision_probes/20261103T101535Z_n100_2cec62ce`

build 目录含 configure/compile/ctest 日志、robot_audit.json、fk_validation.json、verification.json。
results/ 受 Git 忽略；R2 索引是早前 40 个历史产物的快照，本轮没有重写该快照或改动 current 可视化选择。
后续索引更新可按路径发现 C1 唯一运行目录，但不得把诊断升级为正式验收证据。

## 9. C2 优先事项与研究边界

1. 补齐/核验 LiDAR 和其他实体几何覆盖，核对 Mesh 拓扑、scale/origin、包络与实物；保留资源身份。
2. 逐对审阅持续 raw self 接触，建立有理由、有版本和测试的 ACM；不默认忽略相邻或固定连接。
3. 研究接触 epsilon、零接触/穿透、环境 padding 与 self clearance 的一致策略及边界测试。
4. 校准夹爪实际开度、RJ45 插头/工具、TCP 与线缆范围。空闲夹爪模型不能代表抓持网线作业。
5. 完成政策/覆盖声明后再讨论 CLEAR，随后才开展 D 的受限采样统计；C1 仅 100 行诊断。

本轮没有启动实体机器人、move_group、RSP、仿真或控制节点；没有安装依赖、改官方资源、
改 G1–G4 算法/数据、改 S1 YAML/Schema、Commit/Push/Merge 或新分支。没有自动进入 C2/D/E。
