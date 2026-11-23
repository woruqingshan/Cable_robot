# R1 / R2 当前验收与交接：2026-11-23

最新USB续审：设备1a86:55d4仍绑定cdc_acm；PM control=on、active、挂起计数0，
无串口打开者。D1与D2的Board构造函数AST一致。最可能是关闭/重开相关的USB控制应答/
状态路径异常，具体责任设备/桥接/内核仍Unknown；不能归因到STM32舵机命令。
应用初始化/退出候选修补和离线验证完成：382 passed/2 skipped。
本轮真实串口打开、USB reset、D2/D3查询均0；C2与恢复后D2-D4为NOT_RUN待授权。
见[USB_CDC_DIAGNOSIS_AND_RECOVERY.md](USB_CDC_DIAGNOSIS_AND_RECOVERY.md)。


最新状态：D2-D4软件已提交；获批切换后，新驱动在USB CDC ACM的DTR控制ioctl上超时。
D2/D3实际查询0；当前串口无拥有者，正常硬件驱动未运行，父服务仍Restart=no。
D1=502是历史单次回复，不能作为当前六路状态。R2未执行。
详情见[R1_D2_D4_RESULT_20261123.md](R1_D2_D4_RESULT_20261123.md)。


分支feat/real-operation-pipeline，起点4f2ba9b；用户新技术参考原文保留。
M1维护替换与D1唯一一次ID1查询已获授权并执行；原厂文件不改，无AutoDL部署。

|项目|当前结果|证据边界|
|---|---|---|
|SDK候选修复|PASS（离线）|有限锁/TX/响应、整帧写入、外层校验、失败停止、服务清除部分结果|
|Stage A|PASS（限定）|已无额外打开者；当前打开者0，不重复开发|
|原文件/备份|PASS|原SDK/节点SHA不变，53份源文件匹配旧备份|
|维护覆盖包|新包启动失败|b020afb历史部署成功；ede2020在open/DTR Errno110退出，当前无驱动|
|ID1诊断授权|EXECUTED|用户单独批准M1+D1并确认现场监督；未扩展范围|
|首次ID1|PASS_OUTER_FRAME_CAPTURE|1次请求，Pulse502，主机2.723470ms/RTT5.204781ms；内层关联未验证|
|六ID快照/频率/时延/跨度|NOT_RUN|无真实数值，不用Mock代填|
|同ID连续读取|BLOCKED|KEY_REUSE_UNSAFE保留，不按sleep清队列放行|
|STM32问题|UNKNOWN|参考缺陷存在，本车是否相同未实测，不刷写|
|Pulse→rad/零位/方向|AWAITING_PHYSICAL_VERIFICATION|未执行标定动作，软件公式不作实物核验|
|真实RGB|PASS（历史）|本轮未重测|
|完整真实Observation/R2|BLOCKED|有效数0；真实来源、单位和20ms/0.5s同步证据未满足|
|Robot Execution|DISABLED|没有模型动作映射、Torque或运动指令|

## 本轮代码

- guarded_readback.py：独立一次诊断模式，仅ID1、source unknown、终态未验证；
  保留常规arming前提、重复key拒绝和失败锁存。
- build_candidate_patch.py及补丁：官方位置路径、实际外层帧证据、
  ROS诊断入口、普通接口隔离、未知服务异常清理、维护就绪返回false。
- prepare_sdk_stage.py：只生成经源哈希核验的覆盖包、固定维护参数、启动器与清单。
  不复制GUI、不另建串口驱动，不安装厂商文件或重启服务。
- test_r1_vendor_patch.py、test_r1_readback_guard.py：本轮新增26项验证。
- 新技术参考已读取并按原字节纳入版本管理，没有改写用户整理内容。

## 测试

相关完整选定回归332 passed，1 skipped，15.90秒。
FastAPI未安装导致唯一skip；没有为测试安装软件。
这是SDK、协议、Observation及既有边界的回归，不是全仓全部依赖组合验收。
HTTP全套、原生P2/FCL、GPU forward本轮未运行。
隔离ROS参数解析及测试子进程SIGTERM通过，不接触实机串口；Mock不作为硬件通过。

复现：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/vla_protocol/src:$PWD/vla_server/src:$PWD/cable_robot_vla:$PWD/workspace_analysis/src:$PYTHONPATH"
ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -m pytest -q -rs -p no:cacheprovider \
  cable_robot_vla/test/test_wifi_button_adapter.py \
  cable_robot_vla/test/test_r1_maintenance_transport.py \
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

## 实机方案与退出维护

方案见[R1_SDK_MAINTENANCE_PLAN.md](R1_SDK_MAINTENANCE_PLAN.md)，
实测见[R1_ID1_LIVE_RESULT_20261123.md](R1_ID1_LIVE_RESULT_20261123.md)。
申请范围为临时禁止整车服务自动重启、只退出原驱动、启动维护覆盖，
被动检查后单次ID1；不是整车重启、六ID或重复读取授权。
原文件不变；原驱动恢复可能发送原启动指令，不能作为自动回滚。
本轮已应用获批临时Restart=no并重新打开一次维护串口；未自动恢复正常控制。

固件身份未知不再无限阻塞单次诊断准备；连续采集仍需真实证据和另行批准。
首帧审核后再逐级扩展六ID、重复行为、单位/时钟来源和RobotStateProvider接入。

SDK modification required: YES（覆盖包已部署；原厂文件保持不变）。
STM32 firmware modification required: UNDETERMINED。
Offline tests: PASS。
Live hardware readback: PASS_SINGLE_ID1_OUTER_CAPTURE（内层/采样时刻未验证）。
R1 status: STOPPED_USB_CDC_CONTROL_IOCTL_TIMEOUT（历史D1已获有效外层回复）。
R2 readiness: BLOCKED。

本轮相关回归364 passed/2 skipped；客户端及打开超时专项4 passed。
D4转换、来源隔离与ROS候选发布实现完成，物理零位/方向仍未标定；
实机FK缺urdf_parser_py/PyKDL。六路Pulse、重复读取Hz、均值/P95/最大延迟和跨度均未测得。
切换失败后无重开、无恢复、无刷固件；恢复需另行受控方案。
