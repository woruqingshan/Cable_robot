# JetRover Workspace Analysis — G1-A

当前仅实现 Xacro/URDF 模型加载、PyKDL FK 及测试用独立参考。
**开发完成不等于阶段验收通过：需用户运行下列命令并评估结果。**
不含 TF 对照（G1-B）、工作空间采样、三维可视化或碰撞/规划算法。
所有入口均不启动 ROS 节点或控制器，不调用实体机器人服务。

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
本轮没有运行上述脚本或 pytest；数值结果、PyKDL 绑定及运行时兼容性待用户验证。
阈值仅代表两种软件实现一致性，不代表真实定位精度。
几何可达不等于姿态可行、无碰撞或存在完整 RJ45 插拔轨迹。
