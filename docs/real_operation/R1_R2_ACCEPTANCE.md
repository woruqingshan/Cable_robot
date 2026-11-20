# R1 / R2 验收与交接：2026-11-20

唯一开发分支：`feat/real-operation-pipeline`，本轮起点 `e86d0e6`。
正式修改只在JetRover仓库进行。未部署AutoDL、未修改厂商运行文件。
本轮结论：离线准备完成；实际固件身份UNKNOWN；单舵机实测准入NOT_READY。

## 验收矩阵

|项目|结果|证据边界|
|---|---|---|
|R1 Protocol|BLOCKED|参考C缺陷已离线复现，本车固件未对应；不能把测试通过解释为协议安全|
|R1 Live Single ID|NOT_RUN|0次硬件RPC|
|R1 Six Servo Snapshot|BLOCKED|仅有Mock六ID批次；实测未执行|
|R1 Continuous Readback|BLOCKED|KEY_REUSE_UNSAFE保留，默认DISARMED|
|R1 State Conversion|BLOCKED|厂商软件公式已查阅，真实零位/方向/单位转换证据未验收|
|R2 Real RGB|PASS（历史实测）|此前RGB/Depth/CameraInfo实测，本轮没有重测相机|
|R2 Real State|BLOCKED|缓存与未知来源原始Pulse均不允许进入完整Observation|
|R2 Synchronization|BLOCKED|设备采样时间/相机时钟关联未知，20ms与0.5s规则未放宽|
|R2 Record Only|BLOCKED（完整Observation）|原始传感器录制历史PASS；本轮完整有效Observation=0|
|R2 Cloud GPU Inference|BLOCKED（真实完整Observation）|历史Synthetic真实GPU验证不代替本轮真实状态验证|
|R2 Native Action Return|BLOCKED（真实完整Observation）|历史50×6通信结果保持独立，本轮未发送云端推理请求|
|Robot Execution|DISABLED|无执行后端接入、无动作映射、无控制请求|

## 本轮完成

1. 重新SSH确认主机ubuntu/aarch64，fetch后本地/远程同为e86d0e6，保留并完成暂停前自己的草稿。
2. 核验用户资料包哈希并复制到小车独立参考目录；未重新下载。
3. 采集USB/udev、SDK函数和DDS图、串口打开者、限定目录固件文件及日志线索。
4. 确认参考目标芯片F407VET6，不将其宣称为本车身份。未找到已验证的固件版本查询接口。
5. 编译并执行纯主机C桩，复现参考代码ID/命令未匹配与迟到完成信号风险。
6. 加强离线SDK候选的整帧TX检查和内层关联前提，保持一次key限制、默认禁用及失败停止。
7. 增加RawFeedbackAdapter并通过RobotStateProvider暴露独立诊断；不进行rad转换或时间戳伪装。

核心模块仍按已有职责分层：
Real Camera → RobotStateProvider → RealObservationCore → PolicyClient → VLA Server → Native Action Recorder。
新增适配器只处理原始反馈质量诊断，没有第二套HTTP、串口驱动或Observation schema。

## 自动化结果

- R1四组专项：90 passed in 3.41s（52项既有/扩展SDK测试、1项新增arming测试、
  30项原始反馈/拥有者测试、7项参考C测试，共90）。
- 完整选定回归：229 passed, 1 skipped in 8.32s。
- Skip：vla_server/tests/test_shadow_diagnostics.py:26，缺少FastAPI；没有安装软件。
- 参考C测试输入为用户资料中的精确SHA文件，UART/GPIO/RTOS完全替换为桩，
  不链接app.c，不生成STM32可刷写映像。PASS包含成功复现缺陷，不表示硬件通过。
- 真实硬件读回、静态机械臂状态测量、完整真实Observation录制与云推理均未执行。

完整复现命令（在JetRover仓库根目录）：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/vla_protocol/src:$PWD/vla_server/src:$PWD/cable_robot_vla:$PWD/workspace_analysis/src:$PYTHONPATH"
ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -m pytest -q -rs -p no:cacheprovider \
  cable_robot_vla/test/test_r1_readback_guard.py \
  cable_robot_vla/test/test_r1_vendor_patch.py \
  cable_robot_vla/test/test_r1_raw_feedback.py \
  cable_robot_vla/test/test_r1_reference_protocol.py \
  cable_robot_vla/test/test_real_sensor_quality.py \
  cable_robot_vla/test/test_real_observation.py \
  cable_robot_vla/test/test_real_node_ros.py \
  vla_protocol/tests \
  cable_robot_vla/test/test_edge_core.py \
  cable_robot_vla/test/test_isolation.py \
  cable_robot_vla/test/test_p2_adapter.py \
  cable_robot_vla/test/test_stage15_adaptation_preview.py \
  cable_robot_vla/test/test_stage23_boundaries.py \
  vla_server/tests/test_shadow_diagnostics.py
```

## 修改文件与证据

|路径|职责|
|---|---|
|cable_robot_vla/cable_robot_vla/hardware_feedback.py|原始Pulse批次质量和主机时间诊断；始终拒绝直接作为Observation|
|cable_robot_vla/cable_robot_vla/robot_state_provider.py|显式诊断入口；保留缓存来源校验|
|patches/jetrover_readback/guarded_readback.py|增加内层关联审查前提，保留故障锁存|
|patches/jetrover_readback/build_candidate_patch.py|完整TX计数检查；生成审核补丁|
|patches/jetrover_readback/sdk_bounded_readback_candidate.patch、baseline.json|候选及源/产物哈希；未安装|
|patches/jetrover_readback/audit_environment.py|只读文件/proc清单，不打开串口|
|patches/jetrover_readback/README.md|新增限制和复现方式|
|cable_robot_vla/test/test_r1_readback_guard.py、test_r1_vendor_patch.py|启用前提与短写/兼容性测试|
|cable_robot_vla/test/test_r1_raw_feedback.py|原始状态、过期、错误、重放、缓存隔离与进程可见性测试|
|cable_robot_vla/test/test_r1_reference_protocol.py|精确参考C的纯主机桩行为测试|
|docs/real_operation/SERVO_READBACK_DESIGN.md|完整审计、方案、风险、回滚和实测准入|
|docs/real_operation/HARDWARE_INTERFACE_AUDIT.md、CAMERA_AND_JOINT_STATE.md、END_TO_END_INTEGRATION.md|本轮状态更新|
|docs/real_operation/evidence/r1_*.json|来源哈希及只读元数据，不含凭据或完整固件源码|

## 继续开发前需要补齐

- 本车控制板PCB丝印/硬件修订，以及厂商确认的运行固件版本、对应完整源码/构建哈希。
- 将该版本关联到本车的可靠交付/刷写记录，或已知安全的版本查询接口说明；不能猜测命令后试发。
- 内层回复ID/命令/长度校验、完成信号代际、超时中断收尾、队列及重复/迟到边界。
- 唯一串口拥有者方案，以及不改变原控制语义的有限完整TX设计。
- SDK整体部署及回滚审核，处理任何服务管理可能引发的初始化动作风险。
- 后续另行授权后才依次ID1、六ID快照、重复读取；失败停止，不自动恢复。
- R1可信后再核验Pulse→rad和时间证据，继续复用现有R2组件；不以模拟值补齐实机数据。

当前不具备申请直接执行单舵机读回的充分技术条件。下一步优先补身份与协议保证，
不是批准调用已知可能阻塞的原接口。
