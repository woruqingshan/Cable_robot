# JetRover → AutoDL Shadow 联调：2026-11-20

最新连续实验：20Hz目标下30秒静态600组/3600回复通过，实测19.998Hz；停录后持续运行到周期5995出现DEVICE_ERROR并停读。动态运动前被阻止，未发A/B命令。独立连续实验路径已实现，原可信保护保留；故障留存修补94cd090仅离线，现场仍为a6c828c故障包。详见[本轮实验报告](experiments/r1_continuous_20261124/experiment_report.md)。

以下为既有架构/历史阶段说明，旧状态不能覆盖上述当前结果。

最新实测（2026-11-24）：C3定向重绑定后打开/关闭通过，D2五路及D3有限18次外层回复全部通过；D4实收3组未校准诊断状态，过期拒绝与来源隔离通过。FK依赖缺失；内层关联、物理标定与采样时间仍未验证，R2保持BLOCKED。当前维护包a04beaf运行，父服务Restart=no、ModemManager临时屏蔽。详见[R1_C3_D2_D4_RESULT_20261124.md](R1_C3_D2_D4_RESULT_20261124.md)。

以下为此前阶段的历史记录，旧PID、授权与状态不代表当前现场。

最新实测（2026-11-23 18:10 +08）：获批C2仅打开一次，DTR阶段仍Errno110，初始化15.4136秒，FD已释放；没有位置查询、重试或条件续测。USB重绑定及D2-D4实机为NOT_RUN；串口无打开者，正常硬件驱动未运行。C3提案尚未获批；详见[C2实测结果与C3恢复提案](R1_C2_RESULT_AND_C3_PLAN_20261123.md)。

以下保留此前阶段的记录及各自范围。

## 版本与原则

继续开发分支为 feat/real-operation-pipeline。原 feat/real-shadow-e2e-integration 已快进合入并删除；代码提交 e6c62779791bfe92ba8a54b496b01c957301d8a4 已推送，双端本次联调检出同一 SHA。正式源码只在 JetRover 改动，AutoDL 仅从 GitHub 检出指定提交，没有修改厂商源码或训练 checkpoint。

JetRover：Jetson Orin NX / Ubuntu 22.04 / ROS2 Humble / Python 3.10；NumPy1.26.4、OpenCV4.10.0-dev、CvBridge。AutoDL本次实际GPU为RTX4080、32760MiB；Python3.12.3、Torch2.8.0+cu128，LeRobot锁定ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2。历史4090D报告仅作历史资料，不能算本实例验收。

复用现有 real_observation_node、SensorObservationSource、ObservationAdapter、vla_protocol、PolicyClient 与 vla_server；没有第二套HTTP服务/协议。CameraRecordAdapter保存独立RGB/Depth/CameraInfo证据，RobotStateProvider暴露缓存来源质量；完整Observation仍由既有校验器拒绝不可信状态。Depth不进入SmolVLA RGB特征。

## 本地真实采集与测试

Level1 Camera Ready：PASS。已知四话题实际采集成功。整合只录制运行保存14张RGB和14张16位深度图，原始元数据与CameraInfo保留。完整Observation=0，因为当前状态只能确认为缓存；这不是Real Observation Ready。

自动化：139 passed, 1 skipped，4.91秒。跳过项为小车缺少FastAPI的服务测试；没有为消除跳过安装GPU/服务栈。

测试环境：source /opt/ros/humble/setup.bash；source ~/ros2_ws/install/setup.bash；PYTHONPATH包含仓库根、vla_protocol/src、vla_server/src、cable_robot_vla、workspace_analysis/src及原ROS路径；PYTHONDONTWRITEBYTECODE=1；PYTEST_DISABLE_PLUGIN_AUTOLOAD=1。仅测试进程使用ROS_DOMAIN_ID=117。

执行：python3 -m pytest -q -rs -p no:cacheprovider cable_robot_vla/test/test_real_sensor_quality.py cable_robot_vla/test/test_real_observation.py cable_robot_vla/test/test_real_node_ros.py vla_protocol/tests cable_robot_vla/test/test_edge_core.py cable_robot_vla/test/test_isolation.py cable_robot_vla/test/test_p2_adapter.py cable_robot_vla/test/test_stage15_adaptation_preview.py cable_robot_vla/test/test_stage23_boundaries.py vla_server/tests/test_shadow_diagnostics.py。

新增覆盖CvBridge RGB转换、16位Depth无损保存、独立传感器证据、抽样跳帧与未知驱动丢帧区分、缓存值/ID/单位/发布时间保留、缺失与重复ID、禁止动作方法。保留原时间偏差20ms、最大年龄0.5秒，以及未知模型动作拒绝。

## 网络与部署

最初AutoDL直连github.com:443在130秒后超时。复用了已有Mihomo回环代理17890，并按用户指示 source /root/autodl-tmp/idc-vla/runs/bootstrap/use_proxy.sh；GitHub拉取随后恢复。没有复制源码绕过GitHub流程。

云端推理仅监听127.0.0.1:8000，JetRover通过SSH将本地127.0.0.1:18000转发到该地址。账号密码不写入源码、环境文件或报告；主机密钥验证保持启用。独立的部署配置和证据目录不覆盖旧synthetic报告。

## 验收边界

Test A采用显式synthetic_uncalibrated_v1，检验远程真实GPU及完整原生50×6返回，不代表实机状态已验证。

Test B（真实RGB+合成state）：BLOCKED_PROTOCOL。当前共享协议只有synthetic/real未标定/real已标定三个来源配置，不能诚实表示混合来源；不将真实图像藏在synthetic profile下，也不为过关随意扩展schema。

Test C及Level3：BLOCKED。厂商SDK持锁无界等待，尚无可信六维测量、采样时间及Pulse→rad标定。硬件RPC为NOT_RUN；没有新车实测卡死的断言。模型六个通道的单位/语义仍Unknown，不能映射到舵机。

本轮没有运动/舵机/夹爪/底盘命令，没有执行旧轨迹或Torque Release/Hold，没有重启厂商服务。全部动作返回只进JSON诊断。

## 修改文件

- cable_robot_vla/cable_robot_vla/real_camera.py：新相机证据适配。
- cable_robot_vla/cable_robot_vla/robot_state_provider.py：新只读来源质量接口。
- cable_robot_vla/cable_robot_vla/real_observation_node.py：接入相机、CameraInfo、Depth与缓存诊断。
- cable_robot_vla/config/real_observation.yaml：既有实测话题与抽样参数。
- cable_robot_vla/package.xml：CvBridge和消息运行时依赖声明。
- cable_robot_vla/test/test_real_sensor_quality.py：相机/状态质量测试。
- deployment/local/probe_real_sensors.py：阻断已知无界舵机RPC循环。
- deployment/local/verify_shadow_link.py：使用现有PolicyClient的单条/重复synthetic真实GPU链路验证及完整动作记录。
- docs/real_operation/HARDWARE_INTERFACE_AUDIT.md。
- docs/real_operation/CAMERA_AND_JOINT_STATE.md。
- docs/real_operation/END_TO_END_INTEGRATION.md。

联调结果和精确部署文件路径在完成后追加于下文。所有PASS只覆盖已执行项目；相机外参、TCP及真实执行继续待验证。

## 实际云端联调结果

Level2 Communication Ready：PASS。代码SHA为e6c62779791bfe92ba8a54b496b01c957301d8a4，云端新GPU报告status=PASS、genuine_forward_pass_verified=true；五组含单相机的真实forward均[1,50,6]且finite，首组约0.975秒、单相机约0.255秒。

JetRover通过私有SSH隧道发起3次Synthetic Observation，全部返回完整50×6原生动作；相机槽位名real_rgb是逻辑绑定，此测试像素和state均为合成夹具，明确使用synthetic_uncalibrated_v1，没有混入真实图像。

客户端完整动作与元数据：~/zsy_projects/real_operation_evidence/synthetic_http_e6c6277_20261120.json。
云端证据根：/root/autodl-tmp/idc-vla/runs/real_shadow_e2e_20261120_e6c6277/。
GPU报告gpu_forward.json；清单deployment-runtime/manifest.real-shadow.yaml；配置server.real-shadow.pinned.yaml；服务日志http_service_pinned.log；动作诊断diagnostics/inference_998bbb64c0784b15a52595aed8a2e6f1.jsonl。云端3条请求ID与客户端对应，全部finite、execution_enabled=false。

此前两个启动错误都保留：installed-freeze的PYTHONPATH呈现差异，通过在与启动器相同的环境中重新生成清单解决，没有安装包或绕过哈希检查；复制旧部署配置导致checkpoint receipt路径与新清单不同，通过使用同一新清单的配置基底及绝对路径解决。没有修改checkpoint。

最终文档提交后，双端正式检出同一最终SHA；最终服务来源及一次同SHA请求记录在外部final_source_report.json/ synthetic_http_final_20261120.json，避免在Git文档内自引用其提交哈希。

请求详情（合成数据通信验收）：
- 21f78fdc66d1445d9196c119e3e129ea: RTT=0.368539s，policy=0.307926s，shape=[50, 6]，finite=True，映射拒绝=UNSUPPORTED_ACTION_PROFILE。
- 50f37a6a45aa4ef8822e4ccbb12c33a0: RTT=0.322782s，policy=0.302336s，shape=[50, 6]，finite=True，映射拒绝=UNSUPPORTED_ACTION_PROFILE。
- b03e6506e41e4e468c299d880ef92089: RTT=0.347555s，policy=0.328231s，shape=[50, 6]，finite=True，映射拒绝=UNSUPPORTED_ACTION_PROFILE。

## G0–G7状态与复现

|阶段|状态|证据边界|
|---|---|---|
|G0环境/历史|PASS|远程最新基线、Notion/官方教程/厂商源码已审计|
|G1相机|PASS|实际RGB/Depth/CameraInfo及接收统计|
|G2硬件state|BLOCKED|缓存诊断PASS；无界服务端读取不发RPC，原始测量NOT_RUN|
|G3本地Observation|BLOCKED（完整）；PASS（独立传感器录制）|valid=0，拒绝缓存来源|
|G4GitHub|PASS|代码已提交推送；转入real-operation分支|
|G5通信/GPU|PASS|3次真实GPU HTTP请求，完整50×6动作和双端记录|
|G6真实联调|BLOCKED|Test B混合来源无法由现有schema诚实表达；Test C测量质量未满足|
|G7初步稳定性|PASS（限定范围）|3次低频真实请求；单元测试断网/超时/队列边界。长时间流和真实混合数据NOT_RUN|

JetRover只录制启动（Ctrl+C退出，不启动厂商驱动）：

```bash
cd /home/ubuntu/zsy_projects/Cable_robot
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export PYTHONPATH="$PWD:$PWD/vla_protocol/src:$PWD/vla_server/src:$PWD/cable_robot_vla:$PYTHONPATH"
python3 -m cable_robot_vla.real_observation_node --ros-args --params-file /home/ubuntu/zsy_projects/real_operation_evidence/real_observation_20261120.yaml
```

该配置固定已验证话题，场景清单明确unknown/uncalibrated，推理关闭。实际原始传感器可以录制，可信state证据缺失时完整Observation继续拒绝。

云端代理先执行用户提供的source脚本；部署清单生成前先设置VLA_PYTHON并在同一子shell source deployment/cloud/scripts/common.sh，以保持pip freeze的PYTHONPATH与启动器一致。后续启动使用独立manifest/config，不覆盖旧报告。最终配置路径记录在外部final_source_report.json。

JetRover的synthetic验证入口为deployment/local/verify_shadow_link.py；指定--url http://127.0.0.1:18000、--server-sha当前双端SHA、--camera-id real_rgb、--report全新JSON路径。它只接收并记录原生动作，没有执行器。此前3次使用--count 3 --interval 2。

下一步优先请求厂商提供可恢复、有界的硬件读回路径，先验证单ID请求及超时后的驱动恢复，再验证六ID完整性、实际单位和采样区间；不并行占用串口，不改本轮vendor源码，不用缓存FK充当测量。然后处理Pulse到协议rad、模型训练state语义、同步证据，最后才做Level3。混合来源schema须另行设计审查，不能靠改标记通过。


## R1固件身份续审后的当前边界（2026-11-20，基线e86d0e6）

本轮只有只读元数据、参考协议分析、候选补丁及原始状态离线准备；
没有新云端请求、AutoDL检出、服务部署或重启。
以上GPU/50×6 PASS属于历史验证，不代表本轮真实六执行器状态推理已通过。

本轮选定回归229 passed、1 skipped；R1专项90 passed。
参考C桩复现错配回复和迟到完成风险，不是本车固件行为的实测证明。
固件身份UNKNOWN，单ID实测NOT_READY，R1连续读取/R2真实状态与同步均BLOCKED，
Robot Execution保持DISABLED。继续复用现有相机、Observation、PolicyClient及服务模块。
最新验收矩阵和复现命令见[R1_R2_ACCEPTANCE.md](R1_R2_ACCEPTANCE.md)。


## 历史：串口拥有者Stage A部署前（2026-11-20）

本轮继续推进R1，已完成相关备份、仅蜂鸣器ROS代理、固定版本部署包准备和离线测试。
263 passed、1 skipped；用户按键/Wi-Fi/关机分支均用Mock验证，实际均未触发。
维护工具放在patches/jetrover_readback/，没有向VLA观察/推理包引入硬件消息依赖。隔离ROS生命周期测试只创建domain117发布端，不发送任何命令。
此轮没有AutoDL部署或端到端请求。真实反馈未验证，完整真实Observation仍为0。
Stage A只处理已知额外串口打开者，部署/按键服务重启仍需用户单独授权；
SDK与固件前提保持原保护，不能由Stage A批准推导为真实舵机读取批准。

## 最新进展：Stage A已部署，SDK维护模式仅离线（2026-11-20）

Confirmed：
- Stage A经用户单独批准后，只重启按键服务；原SDK/节点和整车服务未重启。
- 已知额外串口打开者移除；时点快照只见ROS驱动PID3756。
  这不是物理端口内核独占或固件事务关联验证。
- 53个原始文件SHA与备份一致，0次硬件位置RPC。
- SDK候选增加显式维护模式：跳过启动和中断运动指令、移除运动订阅、
  有限且串行化的完整帧TX、既有FD的TIOCEXCL候选。
  所有这些SDK变化仅在项目补丁及临时测试副本中，未安装到运行厂商目录。
- 完整选定回归306 passed、1 skipped in12.25s。
  新增43项相对Stage A提交；唯一skip为FastAPI缺失。
  PTY与Mock结果不作为真实控制板验证。

Unknown：
- 用户推测的主机USB口3不证明STM32 UART3/ROS。
- 实际板型/固件版本及重开串口复位风险、内层关联/迟到边界仍未确认。

R1：首次ID1 NOT_RUN；六路实测/连续采集BLOCKED。
R2：完整有效真实Observation=0，真实状态云端联调BLOCKED；无新AutoDL部署或请求。
默认DISARMED、KEY_REUSE_UNSAFE和内层匹配前提未解除；动作执行DISABLED。

详见[R1_OWNER_STAGE_A_PLAN.md](R1_OWNER_STAGE_A_PLAN.md)实测追加、
[R1_SDK_MAINTENANCE_PLAN.md](R1_SDK_MAINTENANCE_PLAN.md)及验收矩阵。

## 2026-11-23 R1首帧诊断准备

SDK维护覆盖候选已完成相关回归332 passed/1 skipped，固定版本包准备后等待单独授权。
新增ID1一次诊断与实际外层帧证据；unknown来源不接Observation，保留KEY_REUSE_UNSAFE。
没有新云端部署、GPU请求、真实state或50×6返回记录。
详情见R1_R2_ACCEPTANCE.md及R1_SDK_MAINTENANCE_PLAN.md；R2保持BLOCKED。

## M1＋D1授权执行结果（2026-11-23）

M1维护覆盖运行b020afb；父服务PID799未重启，旧驱动4186退出，维护9714唯一持有端口。
ID1仅查询一次：Raw Pulse502，Status0，CRC/长度/ID/Command校验通过；
主机读取2.723470ms，客户端RTT5.204781ms。无重试、无其它ID、无运动/Torque指令。
source_type仍unknown、设备采样时间未知，未接Observation；连续读取及R2仍BLOCKED。
当前保留维护模式及整车临时Restart=no，正常控制未恢复；后续与恢复须另行授权。
之前本轮NOT_RUN/待授权文字属于执行前状态，以此段和R1_R2_ACCEPTANCE.md当前矩阵为准。
原始帧及完整事实边界见R1_ID1_LIVE_RESULT_20261123.md、evidence/r1_m1_d1_20261123.json。

## 最新执行终点：USB控制面打开失败

切换已单独获批。旧维护进程退出后，新ede2020驱动在open期间的DTR ioctl Errno110失败。
未运行D2/D3，未自动重试或恢复。只读观察器已停止，当前无串口驱动/正常控制；
整车父服务仍运行且Restart=no。D4实机反馈/FK和R2不声称通过。
代码与证据保留，详见R1_D2_D4_RESULT_20261123.md。
