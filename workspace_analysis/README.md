# JetRover Workspace Analysis — G1-A / G1-B

G1-A 已由用户运行并验收：三组 PyKDL/独立参考一致，pytest 为
`48 passed, 1 warning`；本机 `results/g1a_fk.json` 的最大位置误差约
`1.22e-16 m`，最大姿态误差约 `1.50e-16 rad`。

本次新增 G1-B ROS2 TF 对照，**代码完成，等待用户运行验收**。
G1-A 入口不启动 ROS 节点；G1-B 只启动自己的 Robot State Publisher。
两者均不启动机器人驱动或控制器，不调用实体机器人服务。
不含工作空间采样、三维可视化或碰撞/规划算法。

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
上述 G1-A 验收结果由用户提供；本轮没有重跑 G1-A，也没有执行 G1-B 验收。
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

**当前状态：G1-B 代码完成，等待用户运行验收；不继续 G2。**

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

**v3 实际集成验证尚未执行，不据单元测试宣称 G1-B 验收通过。**
