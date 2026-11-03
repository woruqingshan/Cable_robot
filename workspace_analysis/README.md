# JetRover Workspace Analysis — G1-A / G1-B / G2 / G3 / G4

G1-A 已由用户运行并验收：三组 PyKDL/独立参考一致，pytest 为
`48 passed, 1 warning`；本机 `results/g1a_fk.json` 的最大位置误差约
`1.22e-16 m`，最大姿态误差约 `1.50e-16 rad`。

G1-B v3 也已由用户执行并验收：Zero、Nonzero、Near-limit 均 PASS，严格端点
身份检查通过，三个 RSP 均正常退出（退出码 0），见 results/g1b_tf_v3.json。

G2 已由用户运行并验收：100 / 10,000 行实验 PASS，两组 NPZ 哈希与 JSON
一致，G2 完整回归为 `152 passed, 1 warning`。既有数据保存在 results/。
G3 已由用户完成测试及交互式可视化验收；G3 代码已提交。
本轮新增 G4 复现、采样稳定性与基础体素命中分析，**等待用户正式实验与验收**。
G1-A/G2/G3/G4 不启动 ROS 节点；G1-B 只启动自己的 Robot State Publisher。
所有入口均不启动机器人驱动或控制器，不调用实体机器人服务。
不含碰撞检测、IK、环境建模或规划算法。

## 模型和坐标约定

- 配置：`config/workspace.yaml`；默认从 ament package index 定位
  `jetrover_description/urdf/jetrover.xacro`。需先 source 既有 overlay。
- Base=`base_link`，EE=`end_effector_link`。
- q 顺序=`joint1, joint2, joint3, joint4, joint5`，长度单位 m，角度单位 rad。
- 关节轴、origin xyz/rpy 和限位完全来自展开的 URDF。
- `MACHINE_TYPE=JetRover_Mecanum` 和 `LIDAR_TYPE=A1` 在 YAML 中显式指定；
  展开期间临时设置，结束或失败时恢复进程内原值。
- **不读取、不要求 `need_compile`**。它只是官方 Launch 的资源路径开关。
- `r_joint` 默认固定为 0。五个 mimic 关节依 URDF 计算，不增加主链自由度。
- 主链保留 `servo_joint1`、`servo_joint2`、`end_effector_joint` 三个固定关节。
  `gripper_joint` 位于另一分支，不应加入到 EE 的串行链。
- `base_footprint -> base_link` 属于所选 Base 的上游，不应重复加入
  `T_base_link_to_ee`；测试另外核对转换到 `base_footprint` 的结果。
- EE 是模型参考点，尚非经标定的 RJ45 TCP；URDF 限位并非实物安全限位。

G1-A 明确支持 fixed 和带有限上下限的 revolute 关节。其他类型、主链
mimic、非法树、重复名称、错误关节顺序、零运动轴、非有限参数、缺失
Frame、非法 q、越界及非法辅助状态均报错，绝不静默固定或裁剪。
主链须是 Base 到 EE 的有向祖先链，不自动反转任意两 Frame 之间的路径。

## 实现与独立性

`model.py` 用 `urdf_parser_py` 读取标准 URDF；按真实父子关系提取完整链。
Xacro 仅在内存展开；记录原文件及实际 includes 的 SHA-256、展开 XML
哈希、Base/EE 和关节元数据，不复制官方 Mesh 或创建第二套模型。
展开 XML 包含本机资源路径，跨机器的原始 XML 哈希可能不同。

`fk.py` 构造 KDL Segment，并调用 `ChainFkSolverPos_recursive`。
转换采用 ROS `kdl_parser` 的语义：关节 pivot 为 origin 的平移，关节轴为
`R_origin * axis_local`，segment 零位变换为完整 origin。固定段同样保留。
适配代码没有 JetRover 尺寸常数，也没有依赖缺失的 `kdl_parser_py`。
参考语义：https://github.com/ros/kdl_parser/blob/humble/kdl_parser/src/kdl_parser.cpp

`tests/reference_fk.py` 不导入生产模块、`urdf_parser_py` 或 PyKDL；
它用 ElementTree 独立提取链，使用 SciPy 旋转和 NumPy 直接连乘
`T_origin @ T_motion_local(q)`。此路径不复用 KDL 的 parent-frame pivot
表示，用于发现轴坐标、乘法顺序、固定段等转换错误。
两条路径仍然共享同一个 URDF，不能证明 URDF 与真实硬件相符。

## 依赖与导入

沿用 G0 已检查的系统 Python 3.10、NumPy、PyYAML、urdf_parser_py、
PyKDL、ament_index_python、Xacro；测试和独立参考另外使用 SciPy、pytest。
不需要 Pinocchio、Plotly、MoveIt Python 或新增系统包。
本阶段没有安装/升级依赖，没有提供会触发自动安装的入口。

脚本通过自身绝对位置定位 `src/`，pytest 通过 `tests/conftest.py` 定位源代码。
运行脚本和测试都不要求恰好位于此目录，也不用 pip install。
在其他程序中使用 API 时，先将本目录的绝对 `src/` 路径加入 PYTHONPATH。
配置中的相对 `model.path` 相对 YAML 所在目录；CLI 的 `--model` 相对调用者 CWD。

## 用户运行命令（Codex 本轮不执行）

Ubuntu 终端执行，沿用已经编译好的工作空间：

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash

/usr/bin/python3 /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/scripts/check_fk.py --reference

/usr/bin/python3 -m pytest -v /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/tests/test_model_fk.py
```

不需要 export `need_compile`、启动 RViz/Gazebo、重新 colcon build 或连接实车。
YAML 已固定 Xacro 配置；shell 的 MACHINE_TYPE/LIDAR_TYPE 不覆盖它。
不要用其他 venv/Conda 的 Python 替换 `/usr/bin/python3`。

如需保存三组配置及其完整矩阵、误差、模型哈希和版本信息：

```bash
/usr/bin/python3 /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/scripts/check_fk.py \
  --reference \
  --output /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/g1a_fk.json
```

输出目录受本地 `.gitignore` 保护。文件已存在时明确拒绝覆盖，请选择新文件名。
JSON 同时打印到终端。异常时退出码为 1，pytest 异常也必须处理，不能忽略。

替换模型或配置的入口（仍需与配置中的 Frame、joint_order、辅助状态匹配）：

```bash
/usr/bin/python3 /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/scripts/check_fk.py \
  --config /absolute/path/to/workspace.yaml \
  --model /absolute/path/to/robot.urdf --reference
```

## API

```python
from workspace_analysis.model import load_model
from workspace_analysis.fk import ForwardKinematics

model = load_model("/absolute/path/to/config/workspace.yaml")
fk = ForwardKinematics(model)
q = [0.35, -0.60, 0.80, -0.40, 0.20]
T = fk(q)                       # float64 ndarray, shape (4, 4)
pose = fk.pose(q)
xyz = pose.position             # shape (3,), metres
R = pose.rotation               # shape (3, 3)
T = pose.transform              # shape (4, 4)
limits = model.limits           # shape (5, 2), read from URDF, radians
state = model.joint_positions(q, {"r_joint": 0.3})  # includes mimic states
same_T = fk(q, auxiliary_positions={"r_joint": 0.3})
```

T 把 EE 坐标下的点转换到 Base 坐标：`p_base = T @ p_ee`。
FK 每次返回新数组；同一 solver 不跨线程并发使用。
辅助位置只允许主链外的独立运动关节，不能直接设置 mimic 或覆盖 q。
基于完整 URDF 对辅助和 mimic 的限位校验不是碰撞检测。

## 预期结果与验收边界

`check_fk.py --reference` 应退出 0，JSON 中应出现：

- `status: SOFTWARE_CHECKS_PASSED`（仅表示本次脚本检查成功）；
- `stage_acceptance: pending_user_review`、`tf_checked: false`、`hardware_checked: false`；
- 三组 Zero、Nonzero、Near-limit，各有 XYZ、R、完整 T；
- 每组 `valid_transform`、`deterministic`、`reference_agrees` 均为 true；
- 每组 `position_error_m <= 1e-5`、`rotation_error_rad <= 1e-4`。

pytest 各项应 PASSED，不能有 FAILED/ERROR，也不因缺少模型而静默跳过：

| 测试组 | 应证明的性质 |
|---|---|
| official_chain_and_dynamic_limits | 活动顺序/固定段正确；改变测试 XML 限位后重新读取 |
| official_three_poses | 三组 FK 合法，并与独立参考一致 |
| deterministic | 同模型同 q 结果一致 |
| limits_and_out_of_range / bad_joint_vectors | 精确上下界接受；越界、NaN、Inf、错形状等拒绝 |
| joint5_position_invariance | joint5 改变姿态，但不改变当前 EE 的 XYZ |
| gripper_and_mimic_invariance | 辅助状态和 mimic 正确解析，EE 位姿不随夹爪变化 |
| fixed_segments_and_zero_pose | 官方零位固定偏移与 Base/footprint 转换一致 |
| rotated_origins_and_oblique_axes | 非零 RPY、斜轴、前/中/后固定段都正确参与 FK |
| invalid_models / missing_frames_order_and_mimic_cycle | 异常模型和配置明确报错 |
| loader_from_another_cwd_without_need_compile | 任意 CWD 加载、无需 need_compile、恢复 shell 环境 |
| plain_urdf_and_relative_config_path | 标准 URDF、相对配置路径、模型路径覆盖正常 |
| urdf_default_axis_and_origin | 缺省 origin/axis 遵循标准 URDF 语义 |
| xacro_failure_restores_environment / invalid_config | 展开失败也恢复环境；配置错误明确报错 |

测试会在 pytest 的临时目录生成最小测试文件，不写官方模型或配置。
独立测试 URDF 使用人工构造参数，仅存在于 tests，不替代官方模型。
上述 G1-A 验收结果由用户提供；G2 开发仅运行离线单元回归，不重新执行 TF 集成。
阈值仅代表两种软件实现一致性，不代表真实定位精度。
几何可达不等于姿态可行、无碰撞或存在完整 RJ45 插拔轨迹。

## G1-B：用户运行 ROS2 TF 交叉验证

新增文件：`scripts/validate_tf.py`、`src/workspace_analysis/tf_validation.py`、
`tests/test_tf_validation.py`。G1-A 的 model.py、fk.py、参考实现和测试不改动。
额外使用现有 rclpy、tf2_ros、tf2_msgs、sensor_msgs、rcl_interfaces、
robot_state_publisher；没有安装/升级依赖。

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash

/usr/bin/python3 /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/scripts/validate_tf.py \
  --config /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/config/workspace.yaml \
  --output /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/g1b_tf_v3.json
```

Hotfix 的默认输出是 results/g1b_tf_v3.json。**已有报告时，在启动任何 ROS
节点之前拒绝覆盖**；前两份 g1b_tf.json、g1b_tf2.json 保留不动。
重跑第三版时请另选文件名，例如 `--output .../g1b_tf_v3_run2.json`。
不需要 `need_compile`、RViz、Gazebo、Joint State Publisher GUI 或实车。
可用 `--timeout 20` 延长各个发现/参数/TF 等待阶段的时限。

隔离原则：默认测试 Domain 为 81；若调用者原 ROS_DOMAIN_ID 为 81，则
用 82。也可指定 `--domain-id 83`，必须在 0..101，且与调用者原 Domain
不同。该范围是保守 Linux 配置选择，不代表自动证明 Domain 空闲。
建议使用自己确认未使用的 Domain。脚本不连接默认 ROS2 CLI daemon。

脚本还使用随机运行 ID 的独立话题，例如
`/g1b_<run_id>/case_1/{joint_states,tf,tf_static}`，并设
`ROS_LOCALHOST_ONLY=1`。即使另一个程序碰巧使用同一 Domain，也不会
订阅本测试之外的 /tf 或 /joint_states。测试期间逐次检查私有话题的
端点名称、命名空间和 publisher GID，发现额外/变更发布者即失败。
环境变量只在当前脚本及其子进程生效，退出恢复，不修改用户终端或系统配置。

### 为什么采用每组独立上下文

单一 RSP + 严格时间戳可以实现，但需要额外处理前一姿态缓存、插值区间
和切换期间消息。当前只有三个固定配置，因此选用每组新 RSP、独立 ROS
Context、私有话题及空 Buffer，启动成本可接受，状态归属更易核查。
不是仅调用 `Buffer.clear()`，也不使用“发布后 sleep 再读 latest”的方法。

每组流程：

1. 复用一次加载的 G1-A 模型和 FK，预检所有 q；不再次展开 Xacro。
2. 创建测试 Context/Node/Buffer；直接启动已安装 RSP 可执行文件，
   用临时 YAML 参数明确传入同一个 `model.urdf_xml`。
3. 先等待 RSP ROS 节点出现在 Graph、参数服务可访问且实际返回结果。
   逐字检查 robot_description 与 G1-A 字符串相等，并核对 use_sim_time=false、
   frame_prefix 为空；此后才进入严格 Endpoint 检查。
4. 有界等待私有 TF 发布者、JointState 端点和 listener 端点身份解析。
   检查数量、名称、Namespace、类型、GID、匹配所需 QoS 及按节点查询的
   Topic Graph 一致。通过 transient-local 的 tf_static 订阅收齐固定边。
   动态通道是否完成更新还必须由后续精确时间戳数据证明。
5. 以系统时钟给本组 JointState 打一次时间戳。发布模型中所有非 mimic
   独立运动关节：当前为 joint1..joint5 和配置的 r_joint。mimic 由 RSP 计算。
6. 必要时每 0.2 s 重发同一状态与同一时间戳；这是消息重试周期，不是
   完成判据。RSP 设置 ignore_timestamp=true，仅绕过其重发节流；
   不允许验证端接受旧时间戳。
7. 自定义订阅回调按 tf2_ros TransformListener 的 Buffer 接口接收 TF，
   显式采用可靠动态 QoS 和 transient-local 静态 QoS。每条 TF 核查
   私有话题的唯一 RSP 端点及 GID、父子 Frame、静态/动态类型；旧动态时间戳被记录并丢弃，
   不进入 Buffer。要求完整模型的全部动态边（包括 mimic）具有本组时间戳。
8. 使用 `lookup_transform(base_link, end_effector_link, exact_request_time)`，
   不查询最新时间 0。固定边按 TF 语义无时间限制；合成变换的时间戳必须
   精确等于本组请求。比较前后再次确认 RSP 的模型参数未变化。
9. 用 `norm(p_KDL-p_TF)` 和
   `Rotation.from_matrix(R_KDL.T @ R_TF).magnitude()` 比较，保存完整矩阵、
   四元数、原始各边时间戳、GID、重试次数、模型哈希与 ROS 环境。
10. 在 finally 中清理本组 RSP、executor、node 和 Context，再进入下一组。

RSP 使用 `Popen(..., start_new_session=True)`，直接启动二进制而非 ros2
launch 包装器。清理只针对保存的本脚本子进程组，依次 SIGINT/SIGTERM/
SIGKILL，等待回收并检查该进程组消失；不使用 pkill、不终止其他 ROS 节点。
清理期间暂时忽略重复 Ctrl+C/SIGTERM，防止中断资源回收。临时参数、日志
目录自动删除，RSP 日志末尾保存到报告用于诊断。

### 结果与异常

逐组输出 q、KDL/TF 的 XYZ、旋转矩阵、四元数、T、两种误差及同步和清理
信息。报告始终包含 `hardware_checked=false` 和
`stage_acceptance=pending_user_review`。

只有三组 Zero、Nonzero、Near-limit 均满足：位置误差 ≤ 1e-5 m、旋转误差
≤ 1e-4 rad、同步证据完整、模型及严格端点身份一致、资源释放且 RSP 退出码
为 0，整体才输出 `PASS`，退出 0。
任何失败、超时、Frame 缺失、发布失败、RSP 异常退出或清理失败均不得整体
PASS。失败返回非零；Ctrl+C 返回 130。报告在运行前独占创建；启动失败也
保存错误。初始化时 RUNNING 的报告不是通过证据。

可先单独运行无 DDS 的单元测试（不启动节点/子进程）：

```bash
/usr/bin/python3 -m pytest -v /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/tests/test_tf_validation.py
```

该文件用假进程/假时钟覆盖精确时间戳、旧缓存拒绝、来源和 Frame 错误、
超时、非法 q、模型身份、启动失败、异常/中断清理、信号升级和报告失败
判定。它不能代替上面的真实三组 ROS2 集成命令；Hotfix 已运行单元测试，
实际 ROS2 集成验证仍由用户执行。

限制：ROS 发现/QoS/自定义 DDS profiles 可能造成超时，应检查报告和日志，
不得改成接受旧 TF。RMW 的特殊发现服务器配置也可能影响 localhost 隔离。
本机 Humble 的 rclpy executor 只向订阅回调传消息，不暴露逐消息 GID。
因此接收时及每次等待循环均核查私有话题的唯一发布端点和 GID；报告明确
记录 `source_check=exclusive_private_topic_endpoint_graph` 和
`per_message_gid_available=false`。这不是逐数据包来源认证；Domain、随机
私有话题及每组单一 RSP 用于避免普通 ROS 节点串扰，不承担恶意发布者防护。
强制 SIGKILL、系统掉电无法保证 Python finally 执行；正常退出、异常和
Ctrl+C 的清理路径已实现，实际运行效果待用户验证。
RSP 与 PyKDL 共用 KDL 后端；本项验证 ROS 集成一致性，G1-A 的 NumPy/SciPy
参考仍承担不同数学实现的交叉检查。软件一致性不代表硬件定位精度。

上述是 G1-B 的运行流程；当前 G1-B v3 已由用户验收，后续进度见 G2 部分。

## G1-B Hotfix v3：Endpoint 发现竞态

前两份报告使用同一模型、配置、RMW 和环境。第一次在 Zero 阶段发现
UNKNOWN 就中止，第二次 Zero/Nonzero 已完成精确时间戳和位姿比较，第三组
又在模型回读前因 UNKNOWN 中止。因此失败点在发现和启动流程，不能视为
Near-limit FK 误差。旧代码把 UNKNOWN 与错误身份混为一谈，又立即发 SIGINT
中断仍在初始化的 RSP；日志中的 context invalid / -6 是该退出竞争的证据。

Humble rmw_dds_common GraphCache 将尚未发现 ROS 节点归属的 Endpoint 标为
UNDISCOVERED_ROS_NODE，并返回这两个 UNKNOWN 字符串：
https://github.com/ros2/rmw_dds_common/blob/humble/rmw_dds_common/src/graph_cache.cpp

| Endpoint 状态 | v3 行为 |
|---|---|
| 没有 Endpoint | NOT_DISCOVERED，继续有界等待 |
| 节点名称/Namespace、GID 或匹配 QoS 尚未解析 | UNRESOLVED_DISCOVERY，继续查询，阻止结果接受 |
| 唯一且身份明确 | 校验名称、Namespace、类型、角色、非零且稳定的 GID、匹配 QoS、节点/Topic Graph |
| 明确错误身份、类型/QoS 不兼容、多个发布者或 GID 改变 | REJECTED，立即失败并保留证据 |
| 持续 UNKNOWN 达到时限 | ENDPOINT_IDENTITY_UNRESOLVED，不能通过 |

报告 endpoint_diagnostics 保存各端点最新 Graph、GID、完整可用 QoS、等待
时长及状态转换时的快照。匹配检查采用 reliability/durability；history、
depth、deadline、lifespan、liveliness 等保留作诊断，不把不参与本测试
匹配的远端 history UNKNOWN 错当身份失败。

没有启用降级验证模式。只有严格 Graph 身份检查满足才记录
strict_endpoint_identity_verified=true；isolated_topic_source_checked 另行
记录隔离 Topic、模型回读和精确 TF 数据验证是否完成，不替代前者。
这仍是端点 Graph 级检查，不是逐消息 GID 或密码学身份认证。若 RMW 持续
不能提供所需元数据，程序明确失败，不能承诺必定完成严格验证。

运行期间若 Graph 暂时变为 UNKNOWN，暂停数据接受并有界等待；两条 TF Topic
各保留一条待验证消息，避免丢失只发布一次的 latched 静态数据，未验证的数据
不进入 Buffer。全程保留原始位置/旋转阈值和精确时间戳要求。

退出报告区分：

- cleanup_resources_released：持有的进程和 ROS 资源是否已释放。
- rsp_exit_code：RSP 实际退出码。
- rsp_exit_normal：只有实际退出码 0 才为 true；-6/-9 等均为 false。

旧 cleanup.ok 保留为资源释放的兼容别名，不能用它推断正常退出。任何 RSP
非正常退出都阻止本组/整体 PASS。若在初始化完成前遇到异常或 Ctrl+C，清理
会先尝试有界等待参数服务实际响应，之后才停止子进程；无法确认初始化就绪
时记录 shutdown_readiness_error，再执行受控清理，不无限等待。

Hotfix 新增 UNKNOWN→正确、持续 UNKNOWN、UNKNOWN→错误、多发布者、启动
顺序、QoS/Graph 不一致、GID 改变、运行中身份短暂未解析、-6 退出及清理
屏障等测试；全部保留原有测试。

v3 实际集成验证现已由用户执行并验收；本轮不重跑 ROS2 集成验证。

## G2：几何可达工作空间采样

开发分支：feat/workspace-planning。新增 sampling.py、results.py、
scripts/sample_workspace.py 和 tests/test_sampling_results.py，不修改已验证
的 model.py、fk.py 或 tf_validation.py，不引入新依赖。

数学定义为 joint-limit-constrained geometric reachable workspace：从 URDF
动态读取关节上下限，使用 NumPy default_rng(seed) 在各关节区间独立均匀采样，
再顺序调用已有 ForwardKinematics.pose(q)。底盘固定，Base=base_link，
EE=end_effector_link，q 的顺序为 joint1..joint5，单位 m/rad。
采样/保存核心支持模型的动态关节数量；当前 CLI 固定检查已批准的五关节链。

每个样本保存完整 q、XYZ、旋转矩阵及 xyzw 四元数。joint5 虽不改变当前
EE 的 XYZ，仍参与五维采样并保留其姿态。失败样本带零起始 sample_index
明确报错，立即终止，不静默丢弃或返回部分成功数据。

### 先运行 100 样本快速验证

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash

/usr/bin/python3 /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/scripts/sample_workspace.py \
  --config /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/config/workspace.yaml \
  --samples 100 --seed 42 \
  --output-prefix /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/workspace_n100_seed42
```

确认快速验证的数字、文件和报告后，再由用户执行正式 10,000 样本：

```bash
/usr/bin/python3 /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/scripts/sample_workspace.py \
  --config /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/config/workspace.yaml \
  --samples 10000 --seed 42 \
  --output-prefix /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/results/workspace_n10000_seed42
```

不需要 need_compile、RViz、Gazebo、RSP 或机器人驱动。所有入口用自身位置
解析默认配置和本地模块，支持任意 CWD。显式提供的相对配置/输出路径相对
调用者 CWD；建议使用上述绝对路径。N 默认 10,000，seed 默认 42；不要把
不带参数执行脚本当成小规模烟雾测试。未指定输出前缀时，默认放在本目录
results/workspace_n<N>_seed<seed>。前缀不包含 .npz/.json 后缀。

正常输出应有模型、关节顺序、N/seed、完成数量、XYZ 范围、FK 耗时、两份
文件路径，最后为 status: PASS 和 stage_acceptance: pending_user_review。
计时包含顺序 FK、位姿检查、四元数转换与进度回调，不是纯 KDL 内核性能基准。
非法参数/输出冲突/计算或保存错误返回非零，Ctrl+C 返回 130。

### 文件格式与验证

同一前缀生成 .npz 和 .json。NPZ 用标准 numpy.savez 格式，含四个 float64 数组：

| 字段 | 当前形状 | 含义 |
|---|---|---|
| q | (N,5) | 全部五个关节角，rad |
| position | (N,3) | EE XYZ，m |
| rotation | (N,3,3) | EE 旋转矩阵 |
| quaternion_xyzw | (N,4) | 归一化四元数，xyzw |

四个数组的相同行号对应同一个 q。旋转矩阵额外保存以核对四元数编码；
四元数 q/-q 等价，不以符号差异判断姿态误差。

JSON schema_version=1，主要字段：

- experiment：算法、请求/完成数、seed、完成 UTC 时间、FK 循环耗时和软件版本。
- robot_model：Base/EE、关节顺序和动态限位、固定段、辅助状态、模型来源、
  Xacro 配置、源文件清单哈希、展开模型 SHA-256。
- configuration：配置绝对路径及 SHA-256；加载前后和写入前检查配置未变化。
- dataset：NPZ 文件名、SHA-256、各数组 shape/dtype、单位和四元数约定。
- workspace_statistics：按 XYZ 排列的 min/max/mean/std（ddof=0）、AABB 边长
  和 aabb_volume_m3。这个体积仅是轴对齐包围盒体积，不是实际可达空间体积。
- reproducibility：Generator/default_rng、实际 BitGenerator 名称、seed、NumPy
  版本、模型/配置/数据身份，以及按数组值比较的复现约定。
- validation：全量数组、限位、姿态编码校验和抽取首/中/末至多三行的 FK 重算误差。
- scope：明确不检查碰撞、RJ45 任务约束、严格边界或已标定 TCP。

写入前验证所有行的有限值、dtype、shape、限位、R 正交性/行列式及四元数
归一化和旋转一致性，并使用原 G1-A FK 抽取至多三行重算。NPZ 暂存文件
写完后重新读取并逐数组比较。CLI 在成对保存后再读回，校验文件哈希、元数据、
统计、数组和模型身份，并再执行抽样 FK 核对，然后才打印 PASS。
PASS 仅表示软件数据检查通过，不代表 G2 用户验收或真实机械臂精度。

复现比较以同一软件环境、模型、N、seed 下的数组值为准；计时和时间戳会
变化，NPZ 原始二进制哈希不要求跨运行相同。跨 NumPy/SciPy 版本不承诺
位级一致，文件内记录版本。均匀关节采样不等于均匀笛卡尔空间采样。

### 写入保护与中断残留

默认拒绝覆盖任何已有 NPZ、JSON 或同前缀 .lock，包括悬空符号链接。
先创建独占 .lock，再在同一个目标目录创建两份临时文件，以实际验证两份
暂存路径均可写。文件写完并 fsync 后，用 hard-link 原子 NO-REPLACE 发布
NPZ，最后发布作为提交标志的 JSON；删除临时文件和锁后读者才接受该对文件。
发生普通异常或 Ctrl+C 时回滚本次拥有的文件，不删除其他写入者的文件。

两个独立文件名无法在一次文件系统操作中同时提交。强制 SIGKILL/掉电可能
留下 .tmp、.lock 或孤立 NPZ，读取器会拒绝 .lock 或不完整文件对。不要将
残留文件当成有效结果；保留现场检查，重跑使用新前缀，不自动删除旧文件。
该提交策略面向当前 Ubuntu 本地文件系统；不支持 hard-link 的文件系统会
明确失败，不降级为覆盖写入。results/ 已由现有 .gitignore 忽略，本轮不改动它。

### Python API 与离线单元测试

```python
from workspace_analysis.model import load_model
from workspace_analysis.sampling import sample_workspace
from workspace_analysis.results import file_sha256, save_results, load_results

config = "/absolute/path/to/config/workspace.yaml"
config_hash = file_sha256(config)
model = load_model(config)
run = sample_workspace(model, samples=100, seed=42)
save_results("/absolute/path/to/results/workspace_n100_seed42", run, model,
             config_path=config, config_sha256=config_hash)
arrays, metadata = load_results("/absolute/path/to/results/workspace_n100_seed42", model=model)
```

在其他 Python 程序中导入 API 时，仍按前述方式设置绝对 src 路径。
load_results 不传 model 时检查文件/数组/元数据；传 model 时另外核对模型
身份、限位并重算抽样 FK。所有 NPZ 读取均使用 allow_pickle=False。

```bash
/usr/bin/python3 -m pytest -q /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/tests
```

G2 测试覆盖合法性、动态关节数量、seed 复现、独立 FK 参考、joint5/零位、
四元数编码、成对读写、哈希/统计、禁止覆盖、损坏/不完整文件、写入/中断
回滚及任意 CWD 的 CLI。CLI 单测仅运行 7 个样本；不执行正式 10,000 样本，
不需要实时 DDS、RSP、RViz、Gazebo 或实车，保留全部 G1-A/G1-B 测试。

G2 仅研究模型关节限位约束下的原始几何 FK 点集。自碰撞、环境碰撞、真实
电机有效范围、动力学、RJ45 姿态/接近方向/接触约束均未检查。有限样本不能
证明严格边界，模型 EE 不等于标定后的 RJ45 TCP。G2 不包含可视化或规划算法。

**G2 已由用户验收；以下 G3 仅消费既有 G2 数据。**

## G3 — 交互式工作空间可视化

新增文件：src/workspace_analysis/visualization.py、scripts/plot_workspace.py、
tests/test_visualization.py。本轮只另外更新本 README，不改 G1/G2 核心、配置、
官方模型、原始 NPZ/JSON。没有新增采样、批量 FK、碰撞/IK/规划或 G4。

### 数据与绘图接口

`load_workspace(input_prefix, config_path, max_points=None)` 先加载现有模型，
**复用 `results.load_results(prefix, model=model)`** 校验 NPZ 哈希、完整数组、
JSON 元数据、统计和模型身份。该 G2 接口内部仍会重算首/中/末至多三行 FK；
这属于既有读取验证，不是重新采样或执行整批 FK。缺失或损坏输入直接报错。

返回 WorkspaceView，包含 position、q、sample_indices、total_samples、
metadata 和全量 sampled min/max。默认保留全部原始行，包含负 Z。
`build_3d_figure(view)` / `build_projections_figure(view)` 返回 Plotly Figure，
`export_workspace(view, output_prefix)` 返回两份 HTML 的绝对 Path。
`prepare_view` 供已验证数组使用，继续调用 G2 数组验证器；文件输入必须走
load_workspace，不得用它绕过 load_results 的文件/模型身份检查。

3D 使用 Scatter3d、Viridis 高度着色与 Z [m] Colorbar；XYZ 单位 m，
scene.aspectmode=data。橙色菱形单独标出 Base 原点，图例与样本分开。
可以旋转、缩放和平移，工具栏可切换视角操作；脚本不会自动打开浏览器。
三个 2D 视图是同一批行的 XY、XZ、YZ，用 Scattergl 显示，共享 Z 色标。
各图设置相同的初始米范围跨度，yaxis.scaleanchor 指向对应 x 轴且
scaleratio=1，避免长宽拉伸；原点始终显示。子图等宽，初始每米像素尺度一致。
浏览器缩放后的布局仍需用户目视确认。

customdata 列为 `[原始 NPZ 行号, X, Y, Z, q1, ..., q5]`。悬停显示全部
位置和关节角 rad；Sample Index 从 0 开始。默认不排序、不筛掉负 Z。
可选 `--max-points M` 仅用确定性的 linspace 整数行号减少显示数量，同时
用相同行号索引 position/q，不重新随机采样。标题显示 shown/total，HTML
meta 记录选择方法；色标和初始 2D 范围仍取全量数据，CLI 范围也是全量极值。

### G3 开发时的依赖状态与可选安装方案（历史记录）

G3 开发时实测 `/usr/bin/python3` 为 3.10.12，**Plotly 和 tenacity 缺失**；
packaging=21.3 可用。没有执行任何安装或升级，也没有安装 Kaleido。
缺少 Plotly 时，数据验证仍可执行，绘图/导出明确报 MissingPlotlyError，
CLI 返回 1，不生成 HTML，不打印 PASS。

用户可先检查自己的运行解释器：

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
/usr/bin/python3 -c 'import plotly; print(plotly.__version__, plotly.__file__)'
```

若决定补齐依赖，建议另建带系统包只读可见性的虚拟环境，复用既有
NumPy/SciPy/PyKDL/ROS 包，仅向虚拟环境安装固定版本的 Plotly 与 tenacity。
下列是**供用户决定并手动执行的方案，本轮未执行**；不是 G3 当前依赖已就绪
的声明。venv 可用性与 pip/ensurepip 已只读检查，venv 创建和实际安装未验证。

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
/usr/bin/python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-deps 'plotly==5.24.1' 'tenacity==8.5.0'
.venv/bin/python -c 'import plotly, PyKDL, numpy, scipy; print(plotly.__version__)'
```

参考版本支持 Python 3.10，且现有 packaging 满足 Plotly 依赖要求，见
[Plotly 5.24.1 元数据](https://pypi.org/pypi/plotly/5.24.1/json) 与
[tenacity 8.5.0 元数据](https://pypi.org/pypi/tenacity/8.5.0/json)。
`--no-deps` 禁止自动解析/升级其他依赖。不要用 sudo pip 或升级系统科学计算栈。
如果采用此方案，下面命令的 `/usr/bin/python3` 要替换成
`/home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/.venv/bin/python`；
仍先 source ROS 和 overlay。创建此 venv 不会让 `/usr/bin/python3` 自动获得 Plotly。
如果用其他方式补齐依赖，应先确认所用解释器能同时导入 Plotly 和 PyKDL。

### 用户运行：先 100 点，再 10,000 点

依赖就绪后，在同一终端执行（不需要 need_compile、RSP、RViz 或 Gazebo）：

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis

# 100 点快速验证；在当前缺少 Plotly 的 /usr/bin/python3 下会明确失败。
/usr/bin/python3 scripts/plot_workspace.py \
  --config config/workspace.yaml \
  --input-prefix results/workspace_n100_seed42 \
  --output-prefix results/workspace_n100_seed42

# 100 点输出与交互效果确认后，再执行正式 10,000 点可视化。
/usr/bin/python3 scripts/plot_workspace.py \
  --config config/workspace.yaml \
  --input-prefix results/workspace_n10000_seed42 \
  --output-prefix results/workspace_n10000_seed42
```

入口按自身路径导入 src 和解析默认配置，支持任意 CWD；显式相对路径相对
调用者 CWD。输入前缀是既有 NPZ/JSON 共同前缀，不带扩展名。未给输出前缀
时使用输入前缀。可将输出前缀改成新的目录/文件名前缀；正式结果建议仍放
被现有 .gitignore 忽略的 results/。不要把导出数据目录提交到 Git。

每次生成：

```text
results/workspace_n100_seed42_3d.html
results/workspace_n100_seed42_projections.html
results/workspace_n10000_seed42_3d.html
results/workspace_n10000_seed42_projections.html
```

正常输出有输入来源、模型/Base/EE、displayed/total、seed、全量 XYZ 采样
极值和两份绝对 HTML 路径，最终 `status: PASS` 及
`stage_acceptance: pending_user_review`。PASS 只表示程序完成导出，用户仍需
打开 HTML 检查交互效果。默认每份 HTML 内嵌 Plotly.js、无外部 script src，
可离线打开，文件通常为数 MB；无需 CDN、浏览器扩展或静态导出工具。
[Plotly 独立 HTML 导出说明](https://plotly.com/python/interactive-html-export/)。

默认拒绝覆盖任一已有 HTML 或同前缀 `_visualization.lock`（含悬空符号链接）。
重新导出时选新前缀；不会改动源 NPZ/JSON。两图均完成内存渲染和完整性检查
后才创建独占锁，用同目录临时文件和 hard-link NO-REPLACE 发布两份 HTML。
正常异常或 Ctrl+C 清理本次文件；中途失败不打印 PASS，错误退出码为 1，
Ctrl+C 为 130。不支持 hard-link 的输出文件系统会明确失败。
强制 SIGKILL/掉电仍可能留下锁/临时文件/部分 HTML，不能提供双文件同时
提交的保证；应保留现场检查并换新前缀，不自动删除残留或覆盖旧结果。

### 离线单元测试与人工检查

```bash
# 同样先 source ROS 和 overlay；若用 venv，替换解释器。
/usr/bin/python3 -m pytest -q -rs \
  /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/tests/test_visualization.py
/usr/bin/python3 -m pytest -q -rs \
  /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis/tests
```

文件测试复制已有 100 点数据到 pytest 临时目录，不重采样缺失输入；如需
使用其他位置，设置 G3_TEST_INPUT_PREFIX 为现有 100 点文件前缀。
绘图结构测试使用 5 个合成点，正向 CLI 使用复制的 100 点数据。
既有 G2 回归包含 7 点 CLI 采样测试，本轮保留；不执行 10,000 点采样/渲染，
所有单测均不启动 ROS 节点或机器人程序。

G3 开发时，在临时测试副本中运行全部原有测试和 G3 测试：
**176 passed, 5 skipped, 1 warning**。其中 G3 为 24 passed、5 skipped；
warning 是既有 URDF 解析依赖的 Python 3.11 提示。
通过项覆盖读取复用、原行号/关节配对、负 Z 保留、非法数组/元数据、损坏
文件/哈希/模型不匹配、显示数量限制、源文件不变、拒绝覆盖、错误 CLI、
渲染失败与写入失败回滚。5 个跳过项分别是实际 3D Figure、子集悬停 Figure、
2D Figure、离线 HTML 正向导出、100 点正向 CLI，均因 Plotly 缺失。
**这些 SKIP 不表示绘图或 HTML 已通过；补齐依赖后应重跑且无此类 SKIP。**

100 点应显示全部 100 行，10,000 点默认显示全部 10,000 行；悬停 q 与 NPZ
相同行对应，原点为独立菱形，Z 色标存在。人工检查三个投影方向、等比例坐标、
旋转/缩放/平移、说明文字的完整可见性，以及断网打开 HTML。
10,000 点全量采样极值应约为 X [-0.364976, 0.410507]、
Y [-0.391087, 0.390261]、Z [-0.208603, 0.503449] m；不是严格边界。
浏览器需支持 WebGL；GPU/窗口大小/高 DPI 下的观感与性能尚待用户实际验证。

### 研究边界

图表说明与本节共同标明：这是 **Joint-limit-constrained geometric reachable
workspace samples**。不等于 collision-free、physically valid、task-feasible
或 RJ45 insertion workspace。负 Z 原样保留，可能涉及碰撞或不实用姿态，
但本阶段不作判断。base_link 原点不等于地面，end_effector_link 不是标定
后的网线插头 TCP。点云密度取决于关节空间采样分布，不是空间均匀密度或
可操作性指标；点云极值/投影外形不是严格边界，也不从凸包推断真实可达体积。

**G3 此后已由用户运行并验收；以下为当前 G4 开发说明。**

## G4 — 复现性、采样稳定性与基础体素命中

新增 validation.py（验证、实验编排、体素统计）、validation_report.py
（CSV/JSON/Markdown）、scripts/validate_workspace.py、
tests/test_workspace_validation.py 和 reports/g4_validation.md 待实验模板。
仅另行更新 README，G1-A/G1-B/G2/G3 核心、官方模型、配置和旧数据保持原样。

### 当前环境与实验设计

G4 只读检查现有 .venv 可导入 Python 3.10.12、NumPy 1.21.5、SciPy 1.8.0、
PyKDL 1.5.1、Plotly 5.24.1、pytest 6.2.5；没有安装或升级任何依赖。
以下命令使用用户已建立的 .venv，仍须 source ROS/overlay 以定位官方模型。

- 复现：两次独立调用原 sample_workspace，均 N=1000、seed=42；分别保存
  独立的 G2 NPZ/JSON 对，再读回核对。四数组都要求 shape/dtype 一致且
  np.array_equal；姿态等价误差另行记录，不放宽确定性判据。时间、耗时、NPZ
  二进制哈希不要求相同。仅承诺同环境复现，不宣称跨版本/跨系统逐位一致。
- 稳定性：N={100,1000,10000} × seed={42,43,44}，共九组。传入的既有数据
  必须经 load_results(model=model) 校验，另外核对配置 SHA256、完整模型/
  源文件身份、N/seed、原七项软件版本、NPZ 哈希。重复前缀或重复 (N,seed)、
  设计之外的数据、不完整或损坏数据均拒绝。不会根据文件名猜测实验参数。
- 原 N=100/10000、seed=42 可核验复用。命令显式指定这两个输入；缺失时
  直接失败，不静默换成新采样。其余七组另存新目录；复现两次与稳定性组
  各自独立记录，不覆盖既有数据。
- 每组 XYZ min/max/mean/std 用 G2 workspace_statistics（std ddof=0），
  另保存 AABB 边长、AABB 体积、原 FK 循环耗时、统一网格命中数。对这些
  标量跨 seed 计算均值与样本标准差（ddof=1），不是置信区间。
- 同 seed 跨 N 的数据为 RNG 前缀，不是 N 之间的独立重复。历史复用的
  耗时与新耗时并列记录，不可当成受控性能测试。稳定性只描述观测变化，
  不预设随 N 单调收敛，也不把软件 PASS 解释成统计收敛证明。

### 统一体素网格

默认 origin=(-0.6,-0.6,-0.4) m、upper=(0.6,0.6,0.8) m、voxel_size=0.02 m，
shape=(60,60,60)。它是覆盖既有样本范围的固定实验区域，不是模型理论边界；
每一组仍逐点检查越界。`--grid-origin X Y Z`、`--grid-upper X Y Z` 和
`--voxel-size H` 可调整，跨度须为 H 的整数倍。所有组共用同一实例，不能
用各自 AABB 建网格；改变网格应新建 run，不能混合比较。

每轴 float64 edge[k]=origin+k*H，体素为半开区间 [edge[k],edge[k+1])；
通过 searchsorted(side='right')-1 索引，原点含、实际 upper 不含。
此规则明确解决十进制浮点边界；JSON 保存请求 upper 和实际对齐后的 upper。
跨度对整数体素数的允许舍入误差为 1e-8 个体素，实际网格边界以输出为准。
区域外样本报出原始 sample_index 与位置，不裁剪或丢弃。voxel_hits 返回
逐点索引、unique_indices、各体素命中次数和不同命中体素数；汇总保存命中数
及完整网格参数，所有逐点/体素索引均可从原数据与索引规则重算。

命中不证明整个体素可达或无碰撞；未命中不证明不可达。没有占据概率、
OctoMap 或碰撞地图，也不计算“严格可达体积”。2 cm 是分析分辨率，
不是机器人定位精度；后续研究可以在相同输入上比较其他分辨率。

### 用户执行正式 G4（本轮未执行）

先运行离线回归：

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis

.venv/bin/python -m pytest -q -rs tests
```

正式运行以下命令。输出目录和报告路径**必须全新**，不要提前 mkdir 输出
目录；已有空目录、文件或符号链接也拒绝。若路径已用，换 g4_run02 等新名。

```bash
.venv/bin/python scripts/validate_workspace.py \
  --config config/workspace.yaml \
  --output-dir results/g4_run01 \
  --reuse-prefix results/workspace_n100_seed42 \
  --reuse-prefix results/workspace_n10000_seed42 \
  --voxel-size 0.02 \
  --grid-origin -0.6 -0.6 -0.4 \
  --grid-upper 0.6 0.6 0.8 \
  --report-path reports/g4_validation_run01.md
```

N/seed 和复现 N/seed 默认就是正式设计。G1-A/G1-B 原报告默认从项目
results/g1a_fk.json、results/g1b_tf_v3.json 读取，验证 stage/模型身份、
三组误差与成功证据后引用路径及哈希，**不启动 RSP、不重跑 TF**。
G3 默认清点 workspace_n10000_seed42 的两份 HTML，记录路径与哈希，
不重新渲染或声称本轮再次视觉验收。入口导入和默认配置不依赖 CWD；
显式相对路径相对调用者 CWD，可使用绝对路径运行。

默认成功状态应包含 profile: formal、reproducibility: PASS、status: PASS、
stage_acceptance: pending_user_review；四个数组一致性和具体误差见 JSON/报告。
PASS 表示所声明的软件实验检查完成，不是 P2.2 用户验收，也不是证明统计收敛。
复现失败会生成真实 FAIL 报告，CLI 返回 1；运行异常返回 1，Ctrl+C 返回 130。
不带必需的 --output-dir 只会显示参数错误，不会启动默认实验。

如仅需小规模诊断，可另外指定 `--sample-counts 3 7 --seeds 42 43
--repro-samples 5`，同时移除不属于该设计的既有复用前缀，并使用新输出名。
非正式 N/seed 配置会明确标为 profile: diagnostic，不得作为九组正式实验验收。

### 输出内容与防覆盖

```text
results/g4_run01/
  reproducibility/run1_n1000_seed42.npz + .json
  reproducibility/run2_n1000_seed42.npz + .json
  experiments/workspace_n<N>_seed<S>.npz + .json  # 七组新数据
  experiments.csv       # 九组逐实验统计、来源/哈希与 reused 标志
  across_seed.csv       # 每个 N/标量统计量的跨 seed 均值和样本 SD
  validation.json       # 全部参数、身份、数组误差、统计、网格和输出哈希
reports/g4_validation_run01.md  # 实际数值与基于观测的解释
reports/g4_validation.md        # 本轮交付的 NOT_RUN 模板，保持不覆盖
```

JSON 记录模型和配置 SHA256、原七项软件版本、环境、模块源码哈希、每组源
NPZ/JSON 哈希和数组 schema、G1 历史证据及 G3 文件清单。CSV 数值保留 Python
float 的往返精度；JSON 包含相同的逐实验与跨 seed 行，发布前后核对一致性。
Markdown 展示逐轴统计、各组 AABB/FK/体素、跨 seed 表和观测解释，不伪造结果。

新输出目录独占创建，运行期间保留 .g4_incomplete。汇总 CSV/Markdown 暂存
后采用本地 hard-link NO-REPLACE 发布，最后发布 validation.json；所有读回
检查结束才移除 marker。正常异常保留已生成的 G2 数据和 failure.json 诊断，
不删除已有实验或重新覆盖运行。强制结束/掉电可能留下不完整目录；含 marker
或 failure.json 的 run 不能作为已完成结果。多文件无法一次原子提交，不承诺
掉电事务恢复；输出文件系统必须支持 hard-link。重新运行请使用新目录。

可在其他 Python 程序中按既有导入方式调用
`validation_report.load_validation_bundle(run_dir)`，拒绝不完整 run，验证输出
哈希、CSV/JSON 行一致性、跨 seed 聚合和复现状态。数据源的路径和哈希用于
追溯，不是密码学身份认证；移动结果时需保留或更新外部 Markdown 路径。

### 本轮测试与待验收项

本轮临时副本的完整离线回归为 **221 passed, 1 warning**，没有 SKIP；
其中新增 G4 为 40 项，既有 G1/G2/G3 共 181 项全部保留并通过。
warning 是既有 URDF 解析依赖的 Python 3.11 私有变量提示。
G4 测试仅使用小数组、2/3/5/7 点采样和诊断设计，不运行正式九组实验。
保留全部 G1/G2/G3 测试；G3 回归最多渲染 100 点，TF 测试不启动 RSP。
测试覆盖同 seed 复现、不同 seed、四元数符号、维度/dtype、统计定义、
固定网格命中/边界/越界、配置/模型/版本身份、损坏/重复数据、输出防覆盖、
CSV/JSON 一致性、真实 FAIL、异常退出和诊断保留。实际测试结果见本轮交付。

待用户执行正式两次复现与九组稳定性实验，检查具体误差、跨 seed 波动、
体素命中、报告来源及输出一致性后，才能验收 G4。当前报告模板没有正式
PASS 或编造的实验值，P2.2 尚不能直接宣布全部完成。

P2.2 之后仍需研究：实物 TCP/模型标定与硬件误差、真实安全限位、
自碰撞/环境碰撞、IDC 约束建模、RJ45 姿态与接近方向约束、IK 和轨迹规划。
这些是后续技术问题，本轮不实现。

**G4 代码开发完成，等待用户运行正式实验与验收。**
