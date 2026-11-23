# R1：舵机反馈协议、固件身份与离线读回设计

最新实测（2026-11-23 18:10 +08）：获批C2仅打开一次，DTR阶段仍Errno110，初始化15.4136秒，FD已释放；没有位置查询、重试或条件续测。USB重绑定及D2-D4实机为NOT_RUN；串口无打开者，正常硬件驱动未运行。C3提案尚未获批；详见[C2实测结果与C3恢复提案](R1_C2_RESULT_AND_C3_PLAN_20261123.md)。

以下保留此前阶段的记录及各自范围。

更新：2026-11-23（Asia/Shanghai）。唯一分支 `feat/real-operation-pipeline`。
本轮起点 `e86d0e6c784847e75b870ccc22f6dfcc272aba86`；此前候选起点为 `2da91cb`。
本轮先离线开发；M1+D1单独授权后已替换为维护进程并捕获一次ID1位置回复。未刷固件、发送Torque或运动指令。

## 1. 当前结论（D2-D4本轮最新结果）

D2-D4软件检查点已完成；实际推进因USB CDC ACM端口打开失败停止。
serialposix._update_dtr_state的TIOCMBIC(DTR)返回Errno110。
失败发生在位置服务/接收线程启动之前，本轮D2/D3查询为0。
旧维护进程已退出，新维护进程失败，当前串口无拥有者。
不能把本次失败归因于STM32位置回复关联缺陷；内层0x1C路径本轮未执行。
此前D1的有效外层Pulse502仍成立，但不是本轮新鲜六路状态。

原厂文件不改，原控制状态不自动恢复；父整车服务未重启且Restart=no。
软件有限诊断模式与可信状态严格分离；KEY_REUSE_UNSAFE没有在可信路径删除。
最低下一步是保存USB控制面故障现场并制定受控恢复，不忽略DTR异常或盲目重开。
完整结果见R1_D2_D4_RESULT_20261123.md。下文早前状态与行号为历史证据。

## 2. 本车已安装实现与固件身份

厂商根目录：`/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/`。

|对象|实际证据|判定|
|---|---|---|
|SDK，597行|SHA256 `c4dd48f3176a5dce1e8331846691ce995c0b3c2f79341cfc6c9e89dce1418a89`|Confirmed；本轮结束再次核对|
|ROS节点|SHA256 `4d37d84c6eff0629f5f73ddca2b140cc0af555175fe3f68f8a3359123a716627`|Confirmed；没有修改运行文件|
|USB串口描述符|VID:PID=1a86:55d4，product=USB Single Serial，bcdDevice=0444|Confirmed；不能识别 STM32 固件或 PCB|
|udev别名|99-ttyACM0.rules 按 ttyACM* 创建 rrc 别名|Confirmed；别名不是控制板身份证明|
|SDK固件查询|实际函数清单未发现固件版本查询；SYS通道可见电池0x04解析|Confirmed于所审查代码；不排除未公开协议|
|ROS查询接口|3秒DDS图发现没有控制板固件版本服务；没有发送任何服务请求|限定窗口内Confirmed；不是对所有隐藏能力的证明|
|相似名称接口|/depth_cam/get_sdk_version、/rosapi/get_ros_version 属于相机/ROS；get_node_state直接返回success=True|不能用于固件身份确认|
|已有固件文件/记录|在ros2_ws/src、software、Desktop、Downloads、Documents等指定目录未找到匹配固件文件；读取的相关ROS日志为空|Unknown；有限搜索和空日志不能证明没有其他文件/记录|

`/opt/hiwonder` 不存在。文件名搜索未覆盖所有存储；补充遍历记录到的错误为0，缺失根目录另列；
没有读取 MCU Flash、UID、SWD 或打开串口查询。此前独立参考目录中的 HEX 是下载参考物，不是刷写记录。
当前源码哈希只能确认上位机文件，不能反推 STM32 中运行的映像。
早前 find_spec 解析到 build 路径并最终 resolve 至 src；这不是检查运行中 Python 内存。

充电后元数据采集时，可见 PID790（wifi_manager/button_scan.py）与 PID3189（ros_robot_contr）
均打开 ttyACM0。另有247个进程FD目录不可见，不能据此证明只存在这两个拥有者。
按钮程序源码创建 Board、在按钮事件发送蜂鸣器命令；未发现该脚本启用接收的调用。
打开描述符不证明它正在抢读字节，但共享打开和可能写入已使“唯一串口拥有者”前提不成立。
本轮没有停止/修改这些进程、udev规则或GPIO设置。

证据：
- `evidence/r1_firmware_identity_20261120.json`
- `evidence/r1_ros_metadata_20261120.json`
- `evidence/r1_readonly_environment_after_charge_20261120.json`
- 早前被动订阅原始证据：`/home/ubuntu/zsy_projects/real_operation_evidence/r1_passive_audit_20261120.json`。
  当时4秒窗口内收到56条joint_states、55条servo_states；该计数包含发现时间，不作为实测发布频率。
  缓存Pulse=[500,765,15,220,500,500]，不称为当前硬件实测位置。

## 3. 上位机完整调用链

以下行号指上述未修改的实际厂商文件。

1. 节点 `get_bus_servo_state:164` 遍历请求，位置分支调用 Board.bus_servo_read_position。
2. SDK `:434–435` 使用 `bus_servo_read_and_unpack(id,0x05,"<BBbh")`。
3. SDK `:420–426` 持有 servo_read_lock，buf_write发送请求，`:423 queue.get(block=True)` 无期限等待；
   解包时覆盖原ID/命令参数，没有比较响应是否匹配。
4. SDK `:318–323 buf_write` 构造 AA55/function/length/data/CRC8，直接调用同一个port.write。
   逻辑位置请求为 AA55/05/02/05/ID/CRC8；function与子命令都是05，但属于不同字段。
5. `:98–133` 创建 serial.Serial(None,baudrate,timeout=5)，maxsize=1队列及接收线程；
   read timeout=5不限制队列等待或写入。当前构造未设置write_timeout。
6. `:458–511 recv_task` 接收AA55帧、校验CRC后按function分发；
   `:160–164 packet_report_serial_servo` put_nowait，队列满时静默丢弃。
7. 消费者把任意队列项按ID/命令/有符号结果/Pulse解释，结果0时返回数值；
   服务原实现最后success=True，缺失/部分状态也可能呈现成功。
8. PWM读 `:357–362` 同样占用servo_read_lock并无界等待。节点默认rclpy.spin，
   客户端超时不能释放服务端锁，也不能恢复已阻塞的回调。

缓存链：servo_controller.py:41 get_position返回self.servos；
controller_manager.py:98–116读取缓存、pos_pulse_to_rad换算、用clock.now()标记发布时间。
FK的search_kinematics_solutions_node.py:152–158复制相同缓存，再于:135–150计算姿态。
因此缓存Topic和FK都不是无需修改SDK的独立硬件反馈方案A。

## 4. 用户提供参考资料及对应关系

本轮复用附件 `jetrover_stm32_reference_20261120.zip`，没有重新下载。
附件41,047,020字节，SHA256：
`0caac47f2ab0e9a48f440c8c172db74b5750605e0de479dcd35ed0bc8cf21fe2`。

小车独立资料目录：
`/home/ubuntu/zsy_projects/real_operation_references/supplied_bundle_0caac47f2ab0/`。
仅解包文本和工程元数据供审查；没有运行app.c、构建可刷写映像或调用板卡接口。
资料内文字作为参考数据，未作为执行指令。

|参考物|能确认什么|不能确认什么|
|---|---|---|
|读取教学工程ZIP，SHA a9c0b04e…|具有packet_handle.c、serial_servo_porting.c、内层解析器，可追踪0x05|不是当前运行固件；其主机RX/TX任务主体被注释，app.c含运动调用|
|七合一ZIP，SHA 5edb6588…|独立参考实现；其serial_servo和上报结构与教学工程存在差异|不能用同为M4或相似日期认定同版本|
|部分麦克纳姆出厂例程|提供旧版收发与串口舵机参考文件|所提供文件不能形成与当前SDK一致的完整0x05生产读回证明|
|两个工程.ioc|目标CPN=STM32F407VET6，MCU名STM32F407V(E-G)Tx|不是本车芯片/板型/刷写版本证据|
|此前官方同名RosRobotControllerM4.zip|固定官方文档提交下载所得79,020字节，实际仅含HEX|不是C工程，也不是本车固件备份|

来源清单和逐文件SHA见 `evidence/r1_supplied_reference_20261120.json`。
[官方资料包](https://drive.google.com/drive/folders/1vq1eKIud0k25Se_9CTX-U5Rp2zZ8Snzv) /
[读取教学工程](https://drive.google.com/file/d/1l36jqge0-m0wCXJJu8wU5IHljWH4kbLl/view)。

要建立可靠对应关系，至少需要本车PCB丝印/修订、厂商确认的构建版本及完整对应源码，
以及能把该构建关联至本机实际刷写内容的交付/刷写记录或经认可的只读身份查询。
仅有同名HEX、源码注释日期、同一目标芯片或上位机SDK相同不够。

## 5. 参考工程0x05处理链与缺口

以下位置基于资料包中
`servo_read_example/RosRobotControllerM4_serial_servo_read/`：

|步骤|参考代码|确认的行为|
|---|---|---|
|主机帧解析|Hiwonder/Misc/packet.c:32–86|CRC正确后分发function回调|
|BUS回调|System/packet_handle.c:150–170|0x05调用serial_servo_read_position，随后请求一次5字节外层上报|
|内层查询|Peripherals/serial_servo.c:71–80|构造SERIAL_SERVO_POS_READ=28/0x1C，经serial_write_and_read等待，成功后取共享rx_frame.args|
|半双工发送与等待|Portings/serial_servo_porting.c:12–32|重置接收状态、启动UART中断，等待共享完成信号量，结束时设置方向引脚|
|内层接收|Portings/serial_servo.h:117–172|解析帧并检查校验和；没有与tx_frame比较返回ID/命令或位置命令的精确长度|
|完成通知|Core/Src/stm32f4xx_it.c:529–558|发送完成或接收解析成功时释放同一完成信号量|
|主机上报|packet_handle.c:128–132、164–169|report ID和子命令复制请求；结果来自read_position，position复制两字节|

**外层ID回显不是内层身份匹配。** 错误舵机/命令的有效校验帧可能进入共享rx_frame，
而上报仍带本次请求ID；仅检查Python收到的外层ID会遗漏此类风险。

初始化将proc_timeout设为4，FreeRTOSConfig.h配置tick为1000Hz，即名义4tick等待。
它不覆盖主机TX、STM32调度、上报队列、USB和Python分发的总延迟，也不是实测延迟保证。
当前参考函数没有每次读取建立独立完成代际；启动时才清一次完成信号量。
模拟“首次超时、旧完成随后出现、第二次调用消费它”的主机桩调度会返回旧位置。
这证明参考逻辑允许该软件场景，**不证明本车方向引脚、实际ISR和调度已触发该场景**。

packet_transmit走发送队列，队列入队期限不是整条响应的交付期限；
教学工程主机收发任务主体被注释，不能把这个教学工程当作生产协议的运行证明。
还需核对内层发送长度与TX中断、失败时中断收尾、其它MCU调用者是否串行化、
队列满/传输失败的上报行为，以及真实回复是否存在迟到/重复。

## 6. 串口写入与连续事务方案

本机PySerial 3.5 POSIX实现默认write_timeout=None，写入可能等待无期限。
有限write_timeout能限制软件等待，但不能证明设备已经处理请求；
返回字节数仍须检查，异常/取消/短写也不能当作整帧完成。
flush/tcdrain无显式期限；取消写入不证明USB/设备缓存中没有剩余字节。
参见[PySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html)。

本轮不修改共享串口的全局参数。后续须在唯一串口所有者中审查：
所有发送者的串行化、完整帧写入、有限期限与失败后的帧边界。
不能新开端口、放弃一个仍在运行的后台发送线程，或用sleep/queue.clear冒充恢复。

可靠连续读回至少需要：
1. 本车固件身份及可审查的内层响应匹配；上报请求字段不能替代它。
2. 主机与MCU两侧共享事务对象都被串行化；没有其它消费者/发送者破坏关联。
3. 有界完整TX、有界等待、CRC/ID/命令/长度/状态校验。
4. 成功事务的回复唯一性或等效协议保证；错误、迟到、重复均停止采集。
5. 超时后的可验证终止/隔离边界。无法证明恢复时锁存故障，不继续下一ID。
6. 如增强外层协议，可增加transaction_id并保留版本兼容；但外层序号仍不能修复内层错配，
   内层也必须校验并正确终止超时事务。

如果实际固件已有这些保证，可能只需SDK改进；否则可能需要厂商修正版或下位机最小补丁。
**是否必须修改本车固件：UNDETERMINED**。本轮不生成刷写产物或自动采用参考固件。

## 7. 当前独立SDK候选与离线上层适配

`patches/jetrover_readback/` 中：
- guarded_readback.py：继续默认DISARMED、每个(ID,0x05)只允许一次、错误锁存；
  新增inner_response_match_verified前提。单纯确认外层回显不能激活候选。
  所有arming参数仍是外部审查断言，不能用离线夹具中的True作为硬件证明。
- build_candidate_patch.py / sdk_bounded_readback_candidate.patch：只匹配已审核SDK/节点SHA；
  BUS读取使用已有串口和接收线程；不创建第二套串口驱动。
- 本轮在buf_write增加可选require_complete；原有调用默认False、字节格式及None返回行为保持；
  只有候选读请求要求完整字节数，短写即故障，即使已有回复也不返回状态。
- 有界等待锁/响应、接收线程故障传播、失败清除旧sample、服务端清除部分结果继续保留。
  PWM旧路径不在修复范围，整节点不因此获得全面可恢复性证明。
- audit_environment.py：只读取文件和/proc元数据，不打开设备、不导入SDK、不发送RPC。

e0f151e历史候选SHA256：`6a96c91671eeefa2e8ad3e94b1fa75438a5171e68368fb95d8379ef3b8569f8e`；
详情见baseline.json。旧e86d0e6候选SHA为2d57222e…，不可混用。
当前SDK默认write_timeout=None，候选即使其它条件满足也拒绝发送。
没有实机arming接口、安装器、自动恢复/重启或固件升级命令。

新增 `hardware_feedback.py:RawFeedbackAdapter`，通过RobotStateProvider显式诊断入口复用：
- 校验ID1–5/10完整性、局部序列/记录重放、Pulse范围、负载字段一致性、顺序事务及主机新鲜度。
- 保存六个通道原始Pulse、负载、主机读取开始/解析回调时间和事务跨度；错误后清除旧快照。
- 不创建ROS接口或服务客户端，不进行Pulse→rad，也不把缓存改称实测。
- source_type保持unknown、canonical_observation_eligible=False；控制器缓存仍独立保留controller_cached。
- 记录中的序列和时间检查仅防止本地记录误用，不能建立总线事务关联，也不能绕过SDK保护。

R2可复用现有RealObservationCore、real_observation_node、相机、录制、PolicyClient和vla_server。
当前无经验证的六路测量输入，因此没有连接该诊断入口到Observation生成路径。

## 8. 单位与时间

已安装pos_pulse_to_rad公式/配置提供软件线索：角度比例为240°/1000，init和flipped按关节配置，
r_joint单独配置；这些尚未通过本车物理零点/方向/标定证据核验。
不把Pulse作为rad，不用控制器缓存换算值替代实际测量。

候选read_started_at_ns在事务锁等待前记录，response_received_at_ns为主机解析回调时刻；
它们不是精确TX边沿或设备采样时刻。RawFeedbackAdapter明确保留：
tx_start_monotonic_ns=None、device_sample_time_ns=None、sample_interval_ns=None、
ros_publication_time_ns=None、raw_frame_evidence=None（现有sample只有已解析负载，尚非完整线级trace）。
host_read_latency含锁/软件等待，500ms主机记录新鲜度不证明设备测量年龄。
host_read_span_exceeds_20ms只作诊断；不能据此认证相机/状态同步。

未来需保留逐通道采样不确定区间，核验host_monotonic与ros_system关系；
现有Observation的20ms/0.5s规则保持不变。
若单个stamp无法诚实表示顺序采样，先提交兼容时间证据设计审查，不通过重写stamp过关。
完整物理Home、相机外参、TCP和模型动作语义仍未标定。

## 9. 测试与复现

本轮JetRover执行：
- R1专项：**90 passed in 3.41s**，没有skip。
- 完整选定回归：**229 passed, 1 skipped in 8.32s**。
- 唯一skip：vla_server/tests/test_shadow_diagnostics.py:26缺少FastAPI；没有安装软件消除skip。
- 此前e86d0e6结果185 passed/1 skipped属于历史记录。

新增参考源码测试仅编译主机C程序，UART/GPIO/RTOS全部为桩；
不链接app.c、不构建/运行STM32映像、不打开设备。
7项C测试通过包含“成功复现参考缺陷”，不能在验收矩阵中记为硬件协议PASS。
另新增30项原始状态/拥有者元数据测试、6项短写兼容性测试、1项内层匹配前提测试。
原SDK控制方法AST比较（buf_write单独测试）、模拟报文字节一致性、临时副本补丁应用/反向恢复继续通过。

专项复现：
```bash
cd /home/ubuntu/zsy_projects/Cable_robot
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/cable_robot_vla:$PWD/vla_protocol/src"
python3 -m pytest -q -rs -p no:cacheprovider \
  cable_robot_vla/test/test_r1_readback_guard.py \
  cable_robot_vla/test/test_r1_vendor_patch.py \
  cable_robot_vla/test/test_r1_raw_feedback.py \
  cable_robot_vla/test/test_r1_reference_protocol.py
```

参考C测试默认使用上述独立资料路径，或通过JETROVER_STM32_REFERENCE_ROOT指定教学工程根目录。
缺少精确文件/编译器时会skip，源码哈希不符时失败；其它机器的skip不能视为本次实测通过。
ROS回归仅测试进程使用domain117；完整命令见R1_R2_ACCEPTANCE.md。
普通源码/文档diff空白检查通过；统一diff文件保留格式所需的空白上下文，另以临时应用/反向恢复验证。

## 10. 后续应用、回滚与准入条件

本轮所有运行中厂商文件不变。以下均是未来方案，不是已执行步骤。

|单ID准入项|当前状态|
|---|---|
|本车PCB/固件/源码对应关系|UNKNOWN|
|内层真实响应匹配与超时收尾|UNKNOWN；参考源码有可复现风险|
|唯一串口拥有者|未满足；两个可见打开者|
|主机完整有界TX|未验证；当前默认None|
|干净且独占的通信epoch、接收链|未验证|
|离线失败路径与补丁兼容性|PASS，限已测试范围|
|运行变更、重启风险和回滚审核|尚未执行；无可直接激活路径|
|单ID/六ID/连续真实读取|NOT_RUN / BLOCKED / BLOCKED|

后续顺序：
1. 补齐板型/版本及完整对应源码，由厂商说明内层匹配、完成信号和迟到响应处理。
2. 明确按钮程序与ROS驱动的端口访问安排，以及MCU侧所有调用者的事务所有权。
3. 审查有界TX与失败停止设计；必要时设计固件/协议增强，单独批准后才修改或刷写。
4. 部署前备份真实SDK、节点、安装解析路径、SHA、权限和构建版本；SDK/guard/service wrapper整体审阅。
   新文件在src中不保证自动进入build/install。任何停止/启动必须先处理初始化运动风险：
   原节点KeyboardInterrupt分支有set_motor_speed零速写入，整车bringup也可能初始化执行器。
5. 满足前提并得到另行授权，先ID1 Position Only，记录完整原始TX/RX、真实来源、Pulse、时间及错误。
   异常即停止，不重试、不转下一ID、不以重启或Torque恢复。
6. 单ID通过后才评估六ID顺序快照，再在可重复关联机制成立后评估连续读取。
7. 回滚只恢复审核备份和对应安装构建，确认新增guard属于本次部署才移除；
   保留证据，不覆盖其它用户修改。不能把回滚重启当作本轮已授权操作。

SDK modification required: YES（已审核版本）
Actual firmware identity: UNKNOWN
Firmware modification required: UNDETERMINED
Offline tests: PASS
Live hardware readback: NOT_RUN
Single-ID controlled test readiness: NOT_READY
R1 status: OFFLINE_PREPARED; FIRMWARE_IDENTITY_AND_TRANSACTION_GUARANTEES_BLOCKED
R2 readiness: BLOCKED
Robot Execution: DISABLED


## 11. 串口拥有者Stage A准备（2026-11-20，基线e0f151e）

新一轮已核对真实加载路径及button_scan/start_app_node两个systemd服务；
启动链包含init_pose，因此不重启整车bringup。
相关53文件已备份并逐文件校验，清单为evidence/r1_predeployment_backup_20261120.json。
管理员只读/proc快照无不可读FD目录，当前已知两个打开者仍存在；
该快照不是内核独占机制。

已完成仅通过现有BuzzerState话题替代按键程序Board依赖的维护适配器，
原按键脚本不改写。源码SHA锁定且默认只检查，--run才运行原GPIO/Wi-Fi/关机逻辑。
部署包只生成，不安装、不重启。完整范围、风险、回滚与授权前条件见
[R1_OWNER_STAGE_A_PLAN.md](R1_OWNER_STAGE_A_PLAN.md)。

新增33项离线测试及1项隔离ROS生命周期测试；完整回归263 passed、1 skipped in 8.84s。
VLA隔离断言原样保留，硬件维护工具位于patches目录。
SDK候选未部署、未激活，KEY_REUSE_UNSAFE未解除。Stage A部署等待用户单独授权，
单ID读回仍NOT_RUN，R1连续读取与R2仍BLOCKED。

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

最新项目候选补丁SHA256：0ef5decd928521d931dde747a205cffb0f5472dee21e3d92919bffca6009e9f1；未应用。

## 2026-11-23 实现补充

probe_sources复用原Board/接收线程，只增加完整CRC通过帧证据和独立ID1诊断方法。
COMPLETE_UNVERIFIED不代表hardware_measured；服务success只代表外层校验通过。
常规get_state不能使用诊断授权；未知错误清除部分状态并锁存SERVICE_READ_FAILED。
精确原接收器AST在移除新增证据赋值后与原件一致，原控制兼容断言未放宽。
prepare_sdk_stage只生成固定哈希覆盖包，不执行驱动/串口/服务操作。
本轮首次读回、六路频率/成功率/跨度与Pulse→rad实物核验均NOT_RUN。
原厂公式可作标定候选，不作为已核验的实物零位与方向。

## M1＋D1授权执行结果（2026-11-23）

M1维护覆盖运行b020afb；父服务PID799未重启，旧驱动4186退出，维护9714唯一持有端口。
ID1仅查询一次：Raw Pulse502，Status0，CRC/长度/ID/Command校验通过；
主机读取2.723470ms，客户端RTT5.204781ms。无重试、无其它ID、无运动/Torque指令。
source_type仍unknown、设备采样时间未知，未接Observation；连续读取及R2仍BLOCKED。
当前保留维护模式及整车临时Restart=no，正常控制未恢复；后续与恢复须另行授权。
之前本轮NOT_RUN/待授权文字属于执行前状态，以此段和R1_R2_ACCEPTANCE.md当前矩阵为准。
原始帧及完整事实边界见R1_ID1_LIVE_RESULT_20261123.md、evidence/r1_m1_d1_20261123.json。

## USB只读续审与应用层修补（2026-11-23）

D1/D2串口构造函数AST相同；当前USB PM为on/active、挂起计数0；无新tty打开。
用户确认16:46相机分支断开对应换相机操作，不能据此归因DTR超时。
补充原顺序初始化错误分类、FD一次清理、维护接收线程取消/关闭和自有ROS关停；
保留ioctl异常，不绕过DTR、不自动重试。382 passed/2 skipped，尚未部署或恢复硬件。
恢复方案与证据见USB_CDC_DIAGNOSIS_AND_RECOVERY.md；C2及D2-D4均NOT_RUN待授权。
