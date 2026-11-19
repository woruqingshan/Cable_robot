# JetRover 实机观测与云端推理：开发交付与审查说明

日期：2026-11-19（Asia/Shanghai）。分支：`feat/real-operation-pipeline`，由 `origin/main` 的 `dee7ddba62d05098499ce9197cc443905fce21fb` 创建。没有提交、推送、合并或云端生产部署。本轮源代码修改位于小车本地仓库 `/home/ubuntu/zsy_projects/Cable_robot`；Mac 的 `/Volumes/ubuntu_home` 当前未挂载。

## 当前判断

软件观测、录制、回放及现有 HTTP 服务的诊断记录已实现并进行分层测试。**尚未完成“真实图像 + 可信关节反馈 → 云端 GPU”的端到端验收。** 本轮没有有效实机 Observation、真实图像样本或基于它们的 GPU 输出，不能把测试夹具或已有合成输入 GPU 结果当作实机成功。

执行顺序为 A（真实接口核查）→ B（观测/动作契约对齐）→ C（只读软件与隔离测试）。遇到不可读接口后保留问题，继续不依赖它的软件工作。所有模型输出只进入诊断记录；没有机械臂、夹爪、底盘命令，没有驱动重启或整车启动程序重启。

## A：相机与 ROS 接口的真实运行结果

| 项目 | 分类 | 本轮证据及边界 |
|---|---|---|
| 相机 USB 设备 | Confirmed（运行结果） | `lsusb` 发现 `2bc5:0559 Orbbec DaBai DCW RGB Camera` 和 `2bc5:0659 ORBBEC Depth Sensor`。最初未枚举，后续复查已枚举；不能沿用最初“未接入”的结论。 |
| 相机图像、CameraInfo、深度、TF | Unknown | 有界 ROS 探针收到 0 路传感器消息，只发现标准参数/日志话题；没有图像可保存。实际分辨率、编码、频率、frame_id、时间戳来源、内参、畸变、RGB/depth 对齐、外参均未验证。USB 枚举不等于 ROS 图像验收。 |
| 相机启动配置 | Confirmed（文件声明） | 厂商配置包含 `orbbec_camera::OBCameraNodeDriver`、640×360/30FPS、颜色 MJPG、深度 Y11、`use_hardware_time=false`，以及 `/depth_cam/rgb/image_raw` 重映射。这些只是期望配置，**不是实测图像格式或当前可用话题**。 |
| 依赖导入 | Confirmed（运行结果） | NumPy 1.26.4、OpenCV 4.10.0-dev、ROS Humble `cv_bridge` 导入成功。未安装或替换包。 |
| 已运行的驱动进程 | Confirmed（进程表） | 看到 robot_state_publisher、底层机器人控制器、servo_controller、相机 component_container 等进程；相关进程启动时间为本日 16:22。进程存在不证明话题或设备健康。 |
| 通信异常线索 | Confirmed + Suspected | 当前 wlan0 为 `10.29.23.190`；底层控制器现存 UDP 套接字仍绑定 `192.168.43.25`。其加载的是 Fast DDS。怀疑网络地址变化与发现失败有关，尚未证明唯一因果。没有修改网络、DDS 配置或重启驱动。 |
| 当前控制器/动作/服务可用性 | Unknown | 当前探针未发现厂商的关节流、相机流或舵机读取服务。不能把历史名称当成当前运行接口。读取服务未被发现，因此未发送任何舵机请求。 |

主要实测证据：`/home/ubuntu/zsy_projects/real_operation_evidence/probe_6b91d307dd6c4b7caf97431ec9c1156f/audit.json`。探针源代码为 `deployment/local/probe_real_sensors.py`：有界采样、保存首帧和原始元数据；可选舵机查询仅设置 `get_position`，不创建任何运动接口。服务响应的请求/响应时间区间只作为诊断，不被伪装为硬件采样时间。

### 机械臂、夹爪与反馈来源

- **Confirmed（厂商代码）**：`ServoManager.get_position()` 返回内部 `servos`；`set_position()` 写入待发送位置；controller_manager 从该缓存构建关节消息，并以发布时刻打戳。它不是当前已证实的测量反馈源。节点拒绝 `/joint_states` 与 `/controller_manager/joint_states`，即使配置文件尝试声称它们可信也拒绝。
- **Confirmed（配置）**：joint1–joint5 对应舵机 ID 1–5；r_joint 对应 ID 10。厂商初始脉冲为 500，夹爪为 700，符号由 min/max 配置推导。其原始脉冲换算公式并非实测零位或物理标定证据。未将任何指令缓存代替测量值。
- **Confirmed（URDF 声明）**：joint2–joint5 的位置界限为 ±2.09 rad、速度 10 rad/s；夹爪相关关节声明 ±1.57 rad。它们是模型声明，不构成物理安全限位或测量验证。joint1、真实限位、符号、Home、Ready、TCP 都需后续核实。
- **Confirmed（接口源码）**：GetBusServoState 逐个查询舵机，但响应没有采样 Header；成功标志不保证每个舵机都有位置。只能在真实服务可达后测量完整性、时延和采样时间语义。当前查询没有成功执行，实际位置和采样频率仍为 **Unknown**。
- **Unknown（运行状态）**：当前机械臂/夹爪位置、真实反馈顺序、运动状态、TF、FK/IK 服务及控制器就绪状态。本轮不以固定 `busy=False` 推断机械臂静止。

厂商文件 SHA256：

- servo_controller.yaml：`d0b7ad3eed5688702988dd0ccacbaea692f67e68cdd37273ab1e277300a26c19`
- servo_controller.py：`709e42977ea8c98b0e31a4f58f89f5fdf2e2e83a35589eef1e37a39e2e7c84a4`
- controller_manager.py：`e49cba56878e9d80a751d2826da4650819eb78a651e7c2e56cc4bb375c645142`

这些厂商文件均未修改。

## B：既有协议、模型输入与动作空间

复用 `vla_protocol` Observation v1，JSON 通过现有 `POST /v1/infer` 传输，现有 `GET /health` 报告就绪状态。不增加第二套 HTTP 服务或序列化协议。

- 五个显式具名关节、独立夹爪状态，单位为协议定义的 rad；保留原始 SourceStamp、时钟域和相机 frame_id。
- real 数据固定使用 `real_uncalibrated_v1`、`simulated=False`、夹爪 `calibrated=False`。不升级为已标定。
- 图像为 RGB8、PNG、SHA256，遵守现有图片/请求上限。BGR8 采用显式转换并记录原始编码与步长；不支持的格式拒绝。
- 继续使用 `SensorObservationSource(kind="real")` / `ObservationAdapter` 的最近原始样本匹配、20 ms 偏差和 0.5 s 新鲜度检查。不使用回调时间替代源时间，不放宽阈值。
- 独立夹爪反馈保留自身原始时间戳和接收年龄。映射与可信来源通过外部证据清单及 SHA256 固定；没有可信证据则不生成完整观测。

### 本轮云端核实

**Confirmed（直接登录/读取）**：账号密码认证可用；GPU 为 NVIDIA GeForce RTX 4090 D、49140 MiB，驱动 580.76.05；LeRobot HEAD 为 `ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2`。没有打印或保存认证密码。

**Confirmed（checkpoint 文件）**：`observation.state` 为 [6]；`observation.images.camera1/2/3` 各 [3,256,256]；动作维度 6，chunk_size=50。STATE/ACTION 使用 MEAN_STD，VISUAL 为 IDENTITY；官方处理链负责 batch、tokenizer、设备及归一化，模型内还声明自己的图像 resize/padding。不能自行再加归一化或重复填充相机。

**Confirmed（已有证据文件）**：云端 `runs/bootstrap/smolvla_gpu_20261119T093947Z_4046.json` 记录合成观测下 genuine GPU forward PASS、[1,50,6]、finite=True。这是已有记录，不是本轮真实 JetRover 输入验收。

**Unknown**：训练数据中六个 state/action 通道的物理名称、单位、顺序、绝对/增量语义及与本车的一致性。六维 state 不证明输入应直接使用本车五关节 rad 加夹爪，也不证明六维动作可用于本车。此处停止绑定，不修改 checkpoint 配置或权重。

**Confirmed（服务检查）**：本轮 `127.0.0.1:8000/health` 连接被拒绝，未发现项目推理服务进程。没有为此部署或启动生产服务。云端代码库保持 main 干净。

详细动作与物理标定前提见 `ACTION_CALIBRATION.md`；该文件原始分析是静态分析，以上云端/硬件结果为本轮补充。不得把 20 ms 原生动作偏移当成已验证控制周期，不得注册任意六维到关节的映射。

## C：已实现的软件

| 文件 | 作用 |
|---|---|
| cable_robot_vla/.../real_observation.py | 可信状态来源校验、显式名称映射、独立夹爪、RGB/BGR 适配；复用既有 SensorObservationSource。 |
| cable_robot_vla/.../real_observation_node.py | `/idc_vla/real` 独立节点，系统时钟、只读订阅、私有诊断发布、可选原有 PolicyClient。参数不允许运行中切换为模拟时间。 |
| cable_robot_vla/.../integration/sources.py | 兼容增加 robot_id 以及独立夹爪时间戳/接收时间入口，既有默认行为保留。 |
| cable_robot_vla/.../real_recording.py | 唯一 run 目录，PNG 独立存储、JSONL、图像和标定清单 SHA256、会话重置拒绝、离线回放及协议复核。 |
| cable_robot_vla/launch/real_observation.launch.py | 只启动观测节点；不启动厂商驱动、Gazebo、初始化程序或控制器。 |
| cable_robot_vla/config/real_observation.yaml | 必须审阅填写的模板，推理默认关闭；缺少相机/场景/可信反馈证据时拒绝。 |
| cable_robot_vla/setup.py | 新增 real_observation、real_replay 入口。 |
| vla_server/.../diagnostics.py | 原生 PolicyActionChunk 完整数值、形状、finite、时间、身份、输入元数据和错误的惰性诊断记录。没有执行方法。 |
| vla_server/.../app.py、config.py | 在原有接口添加可选诊断目录与记录，不改变 endpoint、模型加载或协议。 |
| deployment/cloud/config/real-shadow.example.yaml | 独立开发模板，模型/特征/动作绑定故意保持未解决，不能直接宣称可部署。未来须明确允许 real_uncalibrated_v1。 |
| deployment/local/probe_real_sensors.py | 有界只读硬件采样和证据保存。 |
| 新增 test_real_observation.py、test_real_node_ros.py、test_shadow_diagnostics.py | 协议与录制、节点边界/离线网络、云端诊断测试。 |

录制目录包含 manifest.json、observations.jsonl、images/、calibration/calibration_manifest.json、diagnostics/events.jsonl。完整流只接受有效 real 未标定 Observation；失败只进入诊断。JSONL 不重复嵌入 PNG base64。回放只做校验，不发送 HTTP，也不执行动作。

**实现并测试**：RGB/BGR 转换、拒绝无效图像/缺少夹爪/不可信状态/过期与不同步数据、独立夹爪源时间、PNG/标定清单篡改检测、不覆盖目录、私有诊断发布、模拟时钟拒绝、断网仍录制。

**实现但未通过真实传感器验收**：真实数据采集与录制/回放、真实观测 HTTP 推理、重复/连续 GPU 调用。原因是运行接口不可达与模型通道语义未定，不能伪造通过。

**等待物理标定/后续授权**：可信采样时间的舵机反馈、关节零位/方向、物理 Home/Ready、夹爪、TCP、相机外参、RJ45 插入轴及场景几何。真实动作执行始终在本阶段之外。

## 后续审查与部署步骤（本轮没有执行）

1. 先解决 ROS 可发现性，并确认相机/关节/读取服务的真实运行数据。对可能触发初始化动作的驱动重启应另行审查，不能照抄整车 bringup。
2. 运行被动探针取得真实 PNG/CameraInfo/TF、频率与时间统计；验证舵机读数是真实反馈并获得可信采样时间。无法满足时间语义时保持拒绝，不用接收时间补洞。
3. 用真实证据填写外部状态清单、场景清单及本地 YAML。清单必须包含 schema_version、topic、gripper_topic、source_kind=measured_feedback、stamp_semantics=driver_acquisition、clock_domain=ros_system、joint_mapping、gripper_name、units=rad、evidence_reference。保存清单 SHA256；它表达经过审阅的来源证据，不代表已完成物理标定。
4. 在原有 ROS Python 3.10 环境中设置源码路径，先用 `python3 -m cable_robot_vla.real_observation_node --ros-args --params-file <已审阅的绝对路径>` 运行只录制模式。不安装 GPU 模型栈。使用本仓库正常构建流程后才使用 `ros2 launch cable_robot_vla real_observation.launch.py config:=<配置绝对路径>`；本轮没有替用户执行部署构建。
5. 用 `python3 -m cable_robot_vla.real_recording <run绝对路径>` 离线校验。来源会话因时间回退而重置时必须新建 run，不能将多个会话拼成一份有效录制。先取得一条真实有效记录，再做受控重复和连续采集。
6. 查明 checkpoint state 通道单位与语义，复用官方处理器。若仍未知，停在这里；不通过改模型配置让形状勉强匹配。
7. 审查本轮 Git diff 与测试报告后，再决定提交/推送，以及 AutoDL 检出哪一个明确提交。云端原始 synthetic 配置与实验文件保持独立。没有本轮新提交 SHA 可以部署。
8. 未来仅部署 loopback 服务，在外部新配置指定绝对诊断目录、真实模型 receipt/pin 和经过审阅的 feature_spec；通过 SSH 隧道把小车本地回环端口转发到云端回环端口。交互认证或用户配置的安全认证均不写入源码/文档。
9. 按单条 → 重复 → 连续顺序，用现有 PolicyClient 发送通过本地校验的真实数据，核实实际 GPU forward、finite、原生输出、请求对应与延迟。当前服务诊断文件记录的是官方 postprocessor 之后的原生策略响应；每条 HTTP 响应不等于每次均重新执行 forward，重复请求可能命中已有缓存，需联合原有服务状态/推理记录判断。
10. 原生输出仅存入诊断，继续拒绝未知动作映射。任何真实执行开发都需要另一个阶段及明确授权。

本轮只读节点启动、隔离 ROS 与云端内存 ASGI 测试见 TEST_RESULTS.md。尚未完成实机端到端目标，后续最值得做的是恢复可靠的真实传感器可见性及测量反馈时间语义，而不是先开放动作执行。
