# R1 SDK 维护部署与 ID1 单次诊断方案（M1＋D1已获授权执行）

更新：2026-11-23；唯一分支 feat/real-operation-pipeline。
本方案取代2026-11-20“必须先确认固件身份才能申请首次诊断”的限制。
固件身份未知不等于本车故障；允许在明确风险和证据边界下申请一次有界诊断。
可靠连续采集及Observation接入的要求不变。

## 已完成与当前运行

- Stage A已经生效，不重复修改或重启按键服务。
- 本轮现场快照：原驱动PID4186，直接父进程/整车服务PID799；按键服务PID784。
  重启后PID已变化，执行前必须重新核对，不直接照抄旧PID。
- 管理员FD清单仅见原驱动持有/dev/rrc→/dev/ttyACM0，不可读目录0。
- 旧备份53个源文件仍匹配，SDK SHA c4dd48f3…、节点SHA 4d37d84c…未变。
- 真实舵机RPC=0；本轮没有停止驱动、重开串口、修改运行厂商文件或系统服务。

## SDK修改

复用官方arm_pc相同的Board.bus_servo_read_position/0x05读取逻辑，不运行GUI或另开端口。
在原候选上保留有限锁/TX/回复等待、完整写入数、CRC/长度/ID/命令/状态/范围校验。
异常和超时停止，部分ROS服务状态清除；新增未预料服务异常的故障锁存。
记录原接收解析器实际收到且CRC通过的完整外层帧，包括收到的CRC字节。
不会把重新计算的CRC当作抓取到的CRC；坏CRC仍丢弃并使本次读取有界超时。
解析器除新增这个证据字段外，AST逐项比较保持原处理语义。

新增独立诊断路径：
- readback_maintenance=true且readback_probe_reference非空时才暴露
  /ros_robot_controller/bus_servo/probe_id1_once（std_srvs/srv/Trigger）。
- 启动不arm、不发送位置查询；第一次显式调用才进入诊断模式。
- 硬编码仅ID1、一次请求；再次调用不再发送，超时/迟到/重复/错误均不重试。
- 原有标准get_state接口不能冒用诊断模式；原可信arming前提不变。
- 返回原始Pulse、TX/RX十六进制、主机单调时间/延迟、系统报告时间、错误与gate状态。
- success只表示外层位置帧通过校验；source_type=unknown、
  inner_association_verified=false、canonical_observation_eligible=false始终保留。
- 返回帧可能受STM32内部关联问题影响，单次成功不证明新鲜测量或连续可靠性。
  DEVICE采样时间为None；不转rad、不接完整Observation。
- 保留KEY_REUSE_UNSAFE，没有新增自动恢复、清队列/sleep绕过或连续采集许可。

维护模式跳过原节点启动的PWM偏移/电机零速及Ctrl+C电机分支；
不创建任何运动/Torque/PWM控制订阅，也不暴露无界PWM读服务。
原init_finish在维护模式返回false，避免把维护驱动标记为正常控制已就绪。
维护TX只允许位置查询与原蜂鸣器接口，后者本轮不主动触发。

## 离线验收

选定完整R1/协议/Observation/隔离/既有仿真边界回归：
332 passed，1 skipped，15.90秒；相对4f2ba9b新增26项。
唯一skip为FastAPI未安装；没有安装软件、修改模型或弱化隔离断言。
并非全仓库所有依赖组合的验收：HTTP全套、原生P2/FCL和真实GPU此次未运行。
精确测试命令沿用R1_R2_ACCEPTANCE.md所列测试集合。

新增模拟故障、单次限制、诊断来源隔离、部分失败、覆盖包校验测试均通过。
真实ROS参数解析和SIGTERM测试只使用隔离domain117及测试自己创建的子进程；
SIGTERM测试执行原main AST，Fake Board未观察到电机调用。
这不等于给真实PID发过信号，也不证明USB电气线路/控制板复位行为。

## 固定覆盖包（仅准备，不安装）

prepare_sdk_stage.py在Git干净、提交已固定后生成独立目录；
复制经原SHA核对的SDK/节点候选及helper、参数JSON、启动器和manifest。
原厂文件保持不变；启动器验证包内哈希及模块加载路径，固定维护参数，不接受运行参数覆盖。
准备工具不导入运行SDK、不打开串口、不调用systemctl或硬件RPC。

目录：
/home/ubuntu/zsy_projects/real_operation_deployments/r1_sdk_<完整来源SHA>/
参数reference=R1-ID1-20261123，只是审核案例标识，不是自动授权或内层关联证明。
包的完整来源SHA、清单SHA和最终工作区状态在本轮交接终端输出。

## 申请执行范围：M1维护替换 + D1首次ID1

以下全部尚未执行，必须取得用户单独授权，依据本轮约束第5节。

1. 现场确认机器人静止、按键释放、人员可监督和断电。
   核对最新PID/源哈希/备份/打开者/包清单；发现变化先停止。
2. 为start_app_node.service仅新增临时运行时Restart=no覆盖，
   路径/run/systemd/system/start_app_node.service.d/91-idc-r1-no-restart.conf。
   daemon-reload并验证生效，不停止/重启整车服务。
   作用是防止原驱动退出牵连父launch退出时，systemd自动重启整车init_pose。
   不覆盖已有同名文件，不改原unit和永久启动配置。
3. 只对已核验的原驱动子进程发送SIGTERM；禁止Ctrl+C/SIGINT和整车服务重启。
   检查父服务/其它节点、串口打开者，确认原驱动结束、没有自动替代者。
   异常则停止；不自动强杀、重启或恢复。
4. 使用固定覆盖包启动单一维护驱动，作为Restart=no的独立维护进程管理。
   使用当前物理ROS域（当前图为默认domain0）；测试domain117不得带入实机。
   不运行bringup、init_pose或GUI。旧驱动已释放端口后才允许新进程打开。
   在既有新FD申请TIOCEXCL，随后再次核验只有这一个打开者。
5. 先被动验收实际模块路径/维护参数/无控制订阅/诊断服务/父服务未重启；
   未满足任何一项，停止，不发送RPC。
6. 满足以上检查后，只调用一次probe_id1_once。
   使用有限客户端等待并保存完整响应到新目录；客户端超时标UNKNOWN，不重发。
   服务端读取总预算100ms，TX有限50ms；软件期限不是硬实时保证。
7. 无论结果如何，停止扩展测试，提交Pulse、请求回复、时延及错误。
   不自动进入六ID、同ID重复、rad转换或R2。
   维护驱动不恢复正常控制；需要用户审阅后决定后续或退出维护。

## 风险与停止/回滚

USB端口3不等于STM32 UART3。本车固件、实际USB-UART接口以及DTR/RTS重开复位效应仍Unknown。
候选保留原SDK打开前rts/dtr=false的处理，但操作系统/转接芯片仍可能产生瞬态。
不能承诺打开端口绝不复位或固件绝无自主初始化；本方案需现场监督并接受这一不确定性。
如不接受重开风险，则不部署，先由厂商确认接线/复位行为，不猜测刷固件解决。

正常控制接口在维护期间暂时不可用；本方案不发运动、Torque或VLA动作。
停止维护进程也要按已授权方案处理；故障不自动重试/重启。
原厂文件始终未改，文件级回滚无须恢复源码。
恢复原驱动会重新执行原PWM/电机启动逻辑，恢复整车会涉及init_pose，
因此两者不能自动作为此次回滚动作，必须另行授权。
临时Restart覆盖保留至人工退出维护时审核移除；不得在无监督时恢复自动重启。
同一上电周期重启进程也不能作为解除KEY_REUSE_UNSAFE或清除总线旧回复的依据。


## 已执行交接

本方案M1＋D1已获用户单独批准并完成，以上待执行措辞保留为方案归档。
首个有效外层位置回复Pulse502；范围终点已到，不继续六ID或重复。
维护服务与临时Restart=no保留，原厂源码未改，未自动恢复控制。
实际操作、原始帧、指标及局限见R1_ID1_LIVE_RESULT_20261123.md。
