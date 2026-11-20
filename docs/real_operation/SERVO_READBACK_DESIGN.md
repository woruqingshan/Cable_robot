# R1：真实舵机位置读取审计与离线补丁设计

日期：2026-11-20。开发分支始终为 feat/real-operation-pipeline；起点为
2da91cb5b3c94c09e893c47c4da12b5fa8684deb。本轮不涉及 AutoDL 部署。

## 1. 结论和适用范围

推荐方案 B：当前已检查的接口中，没有找到经过验证的、有界且可恢复的硬件位置反馈方案。
需要修改 SDK 的读回保护与服务端错误处理，但本轮只准备独立候选补丁和离线验证，不应用厂商文件。

**不能把本候选称为持续硬件反馈修复已经完成。** 当前报文缺少事务序号；初始干净/独占通信 epoch、
实际回显约定及串口写入有界性均未取得硬件证据。候选默认 DISARMED，即使审查后激活，
每个 (ID,position命令) 也只允许使用一次。重复流式读取仍然 BLOCKED。

本车是五自由度机械臂 ID 1–5，加独立夹爪 ID 10/r_joint。六个舵机不等于六自由度机械臂。
不添加 joint6，不进行 Pulse→rad 猜测，不产生动作或 Torque 命令。

## 2. 实际源码身份与只读运行结果

厂商目录：/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/。

- SDK：ros_robot_controller_sdk.py，597行，
  SHA256=c4dd48f3176a5dce1e8331846691ce995c0b3c2f79341cfc6c9e89dce1418a89。
- ROS节点：ros_robot_controller_node.py，
  SHA256=4d37d84c6eff0629f5f73ddca2b140cc0af555175fe3f68f8a3359123a716627。
- 当前 ROS 环境 find_spec 指向
  /home/ubuntu/ros2_ws/build/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py，
  resolve 后为上述 src 文件，哈希一致。这是文件解析证据，不是对运行中 Python 内存的检查。
- 本轮结束前再次校验厂商哈希；没有复制补丁进去、没有重启服务、没有硬件 RPC。

被动订阅证据：/home/ubuntu/zsy_projects/real_operation_evidence/r1_passive_audit_20261120.json。
4秒窗口内收到 joint_states 56条、servo_states 55条；窗口包含发现时间，不用计数推断实际发布频率。
服务 /ros_robot_controller/bus_servo/get_state 可见，但没有调用。

缓存样本：IDs=[1,2,3,4,5,10]，Pulse=[500,765,15,220,500,500]。
joint_states 同时包含 joint1–joint5/r_joint，其数值是缓存的角度换算，不是本轮实测角度。

|接口|来源判定|时间戳|超时及恢复|真实位置证据|
|---|---|---|---|---|
|/controller_manager/servo_states|Confirmed：控制器缓存|发布时刻|订阅不执行硬件读取；流是否新鲜不证明硬件新鲜|没有|
|/controller_manager/joint_states|Confirmed：缓存经 pos_pulse_to_rad 换算|同一发布时刻|同上|没有|
|/ros_robot_controller/bus_servo/get_state|Confirmed：源码设计为串口查询；运行可靠性 Unknown|响应无 Header|队列与写入风险见下文；客户端超时不恢复服务端|本轮没有舵机响应报文实测|

## 3. 完整调用链与准确原始位置

以下行号均指上述未修改基线。

1. ROS GetBusServoState 服务 -> ros_robot_controller_node.py:164 get_bus_servo_state；
   依次处理 request.cmd，get_position 调用 board.bus_servo_read_position。
2. SDK:434 bus_servo_read_position -> :435 bus_servo_read_and_unpack(id,0x05,"<BBbh")。
3. SDK:420–426：持有 servo_read_lock；:422 通过 buf_write 发送 [cmd,id]；
   :423 bus_servo_queue.get(block=True) 无期限等待；
   :424 解包 reply_id/reply_cmd/success/info，却覆盖局部参数且未比较 reply_id/reply_cmd。
4. SDK:318–323 buf_write：帧为 AA 55 / function / length / data / CRC8，
   直接 self.port.write；不是另一套串口驱动。审查格式中没有请求事务序号。
5. SDK:98–133 初始化唯一 port、两个读取锁、maxsize=1 的队列，启动接收线程。
   serial.Serial(None,baudrate,timeout=5) 只设置字节读取超时；PySerial 签名的 write_timeout 默认为 None，
   SDK 没有另行设置。不能将 read timeout=5 当成服务整体超时。
6. SDK:458–511 recv_task：enable_recv 开启后读取串口，匹配 AA55、function、长度，:498 计算 CRC；
   :500–503 仅校验 CRC 后按 function 分发有效载荷。
7. SDK:160–164 packet_report_serial_servo：将有效载荷 put_nowait 到 BUS 队列；队列满时静默忽略。
8. :423 消费者取出任意队列项，按 "<BBbh" 解释为 ID/命令/结果/有符号 Pulse；
   success==0 返回 info。ROS 回调最后 success=True，即使某项 position 缺失。

缓存链：servo_controller.py:41 get_position 返回 self.servos。
controller_manager.py:98–116 publish_joint_states 读取该对象、换算角度、以 clock.now() 打戳，
并发布两个缓存 Topic；sleep(0.02) 是源码标称周期，不是硬件采样周期。

FK链：search_kinematics_solutions_node.py:152–158 get_servo_position 从 ServoStateList
复制 ID1–5 的缓存位置；:135–150 get_current_pose_srv 对该数组 pulse2angle 后计算 FK。
FK成功不能证明硬件真实姿态，因此它不是方案 A 的替代测量。

SDK:357–362 的 PWM 读取也使用 servo_read_lock，尽管构造器另外创建了 pwm_servo_read_lock。
BUS卡死会占用这个锁；本候选不修复 PWM 路径。所有读写共享同一个 port，
运动命令继续使用 buf_write，不能新开串口与原驱动竞争。

## 4. 为什么仅增加 queue.get(timeout) 不够

- 无响应：消费者长期持锁，ROS节点使用 rclpy.spin 默认单线程执行器，可能拖住同进程回调。
- 错配：有效 CRC 只证明传输完整，不证明属于当前请求；原代码未检查ID/命令。
- 迟到/重复：超时后重试同ID/同命令，旧响应可能被下一次使用。
  即使第一次成功，稍后重复包也可能与下一次相同请求不可区分。
- 容量1的队列满时丢弃新响应，可能保留旧帧；简单 drain/flush 不证明未来不会再到旧包。
- 字节接收线程异常没有向等待者传播；只改客户端或换多线程执行器不能释放已卡住的锁。
- 发送本身可能无界；不能让“后台线程继续发送、前台超时返回”冒充安全恢复。
- UART响应没有已确认的设备采样时间，主机收包时间不是设备采样时间。

当前没有串口舵机响应的实测 trace；实际固件是否严格回显、是否缓存、最大迟到和重复行为均 Unknown。
不能凭 SDK 的解包命名把这些硬件约定认定为已验证。

## 5. 方案 A/B 选择

方案 A 暂不成立：已审查的 Topic 是缓存，FK来自缓存，所有 BUS 查询最终落到同一无界函数；
未发现另一条已验证的可靠测量 API。这不是断言厂商所有版本都没有其他能力。

方案 B 为本轮推荐：最小读路径候选 + 严格协议匹配 + 失败锁存，先离线验证。
若厂商能提供带事务序号的新回报协议，或可独立验证的迟到上界/无重复保证，应优先采用，
再审查持续读取方案。没有这些证据时，不承诺现有无序号协议能安全重复同一请求。

## 6. 独立候选补丁

仓库 patches/jetrover_readback/：

- guarded_readback.py：纯标准库状态机；没有 serial.Serial、ROS客户端或命令发布器。
- build_candidate_patch.py：核对两个原始 SHA，生成补丁；不向 vendor 路径写入。
- sdk_bounded_readback_candidate.patch：三个文件的完整候选变化，含新增支持模块。
- baseline.json：原始和补丁哈希；applied_to_vendor=false。

候选仅改变 BUS position 读回相关代码，不改执行/脉冲/夹爪/底盘命令或 buf_write 的字节格式：

1. 复用既有接收线程和串口，将已通过 CRC 的 BUS响应送到有界单请求状态机；
   生产者不等待事务锁，避免“读线程持锁等接收线程”的死锁。
2. 有界等待事务锁、使用主机单调时钟截止期限，严格检查 ID、命令、负载长度、结果、Pulse范围。
3. 默认 DISARMED；必须有外部审阅的干净epoch、独占所有者、回显及写入有界性证据才考虑激活。
   arm()接收的是外部断言，代码本身不能建立这些物理事实；离线测试断言不得复制到实机。
4. 串口 write_timeout 未验证、接收关闭时拒绝发送。没有设置共享端口超时，以免暗改原控制行为。
   因此当前 SDK 的默认 None 写入期限仍是实机阻塞项。
5. 任意传输/协议不确定性锁存 FAULT；没有自动reset、重试、清空队列继续读或新建连接恢复。
6. 同一个 (ID,0x05) 即使成功也不再次发送；最多规划一个经过独立epoch验证的六ID顺序快照。
   这不是流式反馈，不支持跨快照重复读取。
7. SDK失败的新读取会清除旧 last_bus_position_sample；服务端遇错误清空整份部分结果并返回失败，
   防止把前几个ID的旧/部分状态包装成完整成功。
8. 只有 ID1–5、10 且 get_position=1 的请求可进入候选读路径，拒绝广播、重复ID和其他读标志。
   原温度/电压/PWM读能力不是此实验候选的支持范围；控制指令逻辑保持原样。
9. 接收线程退出/异常向状态机传播故障，不重启接收线程，不关闭/重开共享串口。

PositionSample 保留 raw_pulse、原始负载、read_started_at_ns、response_received_at_ns，
clock_domain=host_monotonic；device_sample_time_ns=None、source_type=unknown。
主机事务时间窗口不是已证实的设备采样窗口。真正硬件来源须另有实机验收证据。

## 7. 离线验证和边界

Mock接收报文验证解析、锁、截止期限、故障锁存；所有 Serial 构造被替换为直接抛错，
SDK测试通过 Board.__new__ 和 FakePort 运行，不执行真实构造器。

候选补丁只对 pytest 临时副本执行 git apply/check/reverse。
测试使用原 SDK 的真实 CRC/帧解析代码，验证正常帧可到达读回保护器、错误CRC不返回状态。
使用 FakePort 比较补丁前后控制报文，并对原控制方法 AST 做逐一一致性比较；
这些是离线兼容性证据，不是实机控制安全证明。

覆盖：正常/无响应/错误长度/错误ID/错误命令/设备错误/非法Pulse/CRC来源；
迟到、重复、同key再次调用、超时后下一读、接收线程故障、写入失败、
锁竞争、六ID顺序读取、部分服务结果清空和默认不发送。

详细命令和最终数字见下方追加的实际测试结果。缺少本机精确vendor文件时，
临时补丁兼容性测试会skip；本轮在JetRover使用实际原始文件执行，不能把skip算PASS。

## 8. 未来实机应用与回滚（本轮均未执行）

以下步骤需要用户单独授权；不存在自动安装器、自动arming服务或恢复脚本。

1. 厂商确认报文回显与新旧响应边界、共享串口写入的有界机制、唯一SDK拥有者；
   不得仅设置四个True断言就宣称epoch已证实。不能证明时保持DISARMED。
2. 在专门维护窗口保存原SDK/节点/安装解析路径、SHA、权限与构建版本，审阅整份候选。
3. SDK、服务wrapper、guard模块必须整体部署并检查实际ROS可解析路径；
   不假设在src放新模块就自动进入build/install。任何构建/服务管理须另行审查。
4. 受控改变现有唯一SDK实例的激活机制须另行设计；本轮未暴露ROS开关，不新开串口。
5. 获得授权后先读ID1，仅position，记录TX/RX原文、Pulse、结果、主机耗时与来源。
   无响应/错配即故障停止，不继续其它ID，也不靠重启证明恢复。
6. 单ID验证通过后才在明确新epoch/事务机制下扩展ID1–5和10顺序快照，记录整体跨度。
   当前候选禁止同key复用，不能直接开始连续采集。还需排除旧PWM无界回调影响。
7. 回滚仅恢复事先备份且SHA匹配的原文件及安装构建；新增guard仅在确认属于本次部署后移除，
   保留测试/故障证据，不覆盖用户其它改动。是否停止/启动驱动由用户监督决定。
   原节点 KeyboardInterrupt 分支含 set_motor_speed 零速写入，整车bringup还可能触发初始化；
   不能把这些作为本轮允许的只读操作。

## 9. R1剩余阻塞与R2复用

R1尚缺：真实回包、初始干净/独占epoch证据、实际响应回显、共享端口写入界限、
同key重复读的协议级关联、无响应后服务/驱动可恢复性，以及六ID真实快照的时间统计。
这是具体的协议/驱动条件缺口，不是笼统的“找不到机械臂数据”。

本轮没有将候选接入现有运行中的 RobotStateProvider，后者仍拒绝缓存。
未来 Hardware Feedback Adapter 可从已验收读回路径取得原始Pulse及质量，扩展现有provider，
不重新实现ArmInterface或串口驱动。经过关节零位/方向/夹爪标定后才转换到协议rad，
并独立验证相机/反馈时钟、采样不确定性与20ms/0.5s校验条件。

real_observation.py、real_observation_node.py、既有相机、PolicyClient、vla_protocol及AutoDL服务继续复用；
R2保持BLOCKED，所有50×6返回动作仍仅记录。SDK离线测试通过不代表物理标定或硬件安全。

SDK modification required: YES（基于已审查版本）
Firmware/protocol change required for continuous reads: UNDETERMINED
Live hardware readback: NOT_RUN
R1 status: OFFLINE_CANDIDATE_PREPARED; LIVE_AND_CONTINUOUS_READBACK_BLOCKED
R2 readiness: BLOCKED

## 10. 本轮实际测试结果

2026-11-20，在 JetRover 上执行最终回归：**185 passed, 1 skipped in 6.94s**。
其中R1新增离线用例46项全部通过；没有R1测试跳过。
唯一skip是vla_server/tests/test_shadow_diagnostics.py:26，当前ROS环境缺少FastAPI，未安装软件绕过。

执行环境和命令：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/vla_protocol/src:$PWD/vla_server/src:$PWD/cable_robot_vla:$PWD/workspace_analysis/src:$PYTHONPATH"
ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -m pytest -q -rs -p no:cacheprovider \
  cable_robot_vla/test/test_r1_readback_guard.py \
  cable_robot_vla/test/test_r1_vendor_patch.py \
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

ROS集成测试使用独立domain117；模拟数据验证不得视为真实硬件读回。
候选补丁SHA256：2d57222ef6a4faf682fc567eee062aa5b535066130e715cea3ba1b2983bfa9f2。

Offline tests: PASS
Live hardware readback: NOT_RUN

本轮只增加报告、独立补丁和测试，未改动运行中的厂商文件；
未发送硬件RPC、未启动/重启驱动、未执行机械臂/夹爪/底盘运动。

普通新增文件的 git diff --cached --check 通过。补丁文件空白上下文行含统一diff必需的单个空格，
直接把补丁作为普通新增文本检查会提示 trailing whitespace；未删除这些结构字符，已通过临时副本应用及反向恢复验证。
