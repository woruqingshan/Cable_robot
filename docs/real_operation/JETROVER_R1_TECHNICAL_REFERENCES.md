# JetRover R1 真实舵机位置反馈：官方资料精要与修复依据

> **用途**：交给 Codex 的单文件技术参考。已从官方教程、协议 PDF、官方 `arm_pc`、舵机二次开发示例，以及已有项目审计中提取当前任务所需内容；常规开发不必重新阅读所有原始 PDF。出现协议歧义或需要修改固件时，再回查原件。  
> **整理日期**：2026-11-23；**仓库**：`woruqingshan/Cable_robot`；**唯一分支**：`feat/real-operation-pipeline`；整理时分支 HEAD：`4f2ba9bf2744cb67b961f47c806bdc36636943bb`。  
> **证据级别**：`[DOC]` 官方手册描述；`[CODE]` 已检查源码行为；`[ROBOT]` 既有车载审计/实测记录；`[RISK]` 可发生的设计风险，非本车故障实证；`[UNKNOWN]` 未验证。  
> **任务边界**：真实 RGB + 五个机械臂关节（ID 1–5）+ 独立夹爪（ID 10）→ 真实 Observation → AutoDL SmolVLA → 返回并记录原生 `[50, 6]` Action；本阶段不执行模型动作。

## 0. 一页结论：已知、未知、应该修改哪里

- **[DOC/CODE] 硬件支持真实位置回读**：官方机械臂教程有“角度回读”；`arm_pc` → `Board.bus_servo_read_position(id)`；舵机原生查询命令为 `0x1C`。不是缺少读取函数。
- **[CODE] 确认要改 Jetson SDK**：原 `bus_servo_read_and_unpack()` 使用无期限 `queue.get(block=True)`，拆包后未核对 `servo_id/cmd`；写端未设置有限 `write_timeout`；GUI 上层重试无法修复底层无限等待。
- **[CODE/RISK] STM32 参考工程有内层关联缺陷**：解析接收帧只检查结构/校验和，不与本次发送的舵机 ID、命令比较；外层返回的 ID/命令来自上位机请求。共享完成信号量的迟到事件有污染下一次事务的风险。
- **[UNKNOWN] 本车运行固件的实际行为**：`RosRobotControllerM4-20260414.hex` 和教学源码并非车上 Flash 的身份证明；尚未做任何受控真实位置 RPC。因此**不能断言当前 STM32 实机必然错配或不能连续读取**，也不能断言 Python 单独修复必然足够。
- **[ROBOT] 串口多打开者问题已部分解决**：Stage A 已使 `button_scan.py` 不再直接打开 `/dev/ttyACM0`，历史时点快照只见 ROS 驱动持有该端口；这不等于内核绝对独占，但不应再把旧的双打开者视为尚未处理的主要瓶颈。
- **路线**：先在现有 `patches/jetrover_readback/` 上完成并审核 **Jetson SDK/ROS2 维护模式补丁** → 受控单 ID 实机读取 → 顺序六 ID → 同 ID 重复读取与异常恢复验证 → Pulse/rad/时间戳 → Observation → 云端推理；若底层证据证明必须改 STM32，再进入固件最小修复路线。

## 1. 原始资料索引：各文件只需参考这些部分

| 编号 | 已收到的资料 | 关键位置／确切用途 | 别误用为 |
|---|---|---|---|
| S1 | `7.机械臂控制.pdf`（132 页） | **第 2、4 页**：角度回读、舵机能力；**第 9 页**：`手掰编程` + `角度回读`；**第 32 页**：ID 1/2/3/4/5/10 的结构映射；**第 38–45 页**：读取/下载偏差和 GUI 卡死说明；**第 50–51 页**：典型控制侧 `set_pose_target`、`set_servo_position`（目标值，不是硬件回读） | 文中“6 自由度”不等于 ROS 里存在第六个臂关节；本车软件接口为 5 臂关节 + 1 夹爪。`读取偏差`不同于实时位置读取。 |
| S2 | `arm_pc.zip` | `arm_pc/main.py:32,104,435–454`; `servo_controller.py:6,12–17,44–58`; `bus_servo_control.py:56–70`; `ros_robot_controller_sdk.py:418–433`。**这是官方 GUI 真实位置回读的主要证据和逻辑参考**。 | 不能启动整套 GUI 与车上 ROS 驱动并行抢串口；不能移植 GUI 的无界等待、自动重试、Torque 释放。 |
| S3 | `Archive.zip` → `4.RRC与上位机通信协议/第14节 RRC与上位机通信协议分析.pdf`（28 页） | **第 1 页**：外层 RRC 帧；**第 10 页**：位置查询子命令；**第 16 页**：位置上传字段与长度。 | 外层合法 ID 不是 STM32 已验证底层舵机回复 ID 的证明。文中 UART3 是控制板接口说法，不能由“Jetson USB 3号口”推断。 |
| S4 | `Archive.zip` → `RosRobotControllerM4-20260414.hex` | 文件名版本线索，SHA256：`a32af05c2296da7b6d10bc8365494f3377bf537d0f5cd956b568c1ab37c99d48`；是编译映像。 | 不是 C 源码、不是本车已刷写版本的直接证据；**不刷写**。 |
| S5 | `1.HTD-35H总线舵机使用说明.pdf`（24 页） | **第 3–4 页**：115200、半双工 UART、支持位置/温度/电压回读；**第 10–15 页**：调试软件读取位置与手动转动后的反馈；标称 0–1000 ↔ 0–240°。 | 调试板直连舵机不等于 JetRover 当前 STM32 RRC 链路已验证。 |
| S6 | `1.HTS-20H总线舵机使用说明.pdf`（23 页） | **第 3–4 页**：115200、半双工 UART、位置反馈；**第 10–14 页**：调试软件读取、位置显示。 | 标称 Pulse→角度关系不能代替机器人关节零位/符号/偏差标定。 |
| S7 | `7.总线舵机二次开发教程-Jetson Nano版本.zip` | `案例1 获取总线舵机信息/serial_servo_status.py` 与 `程序文件/所需库文件/.../hiwonder_servo_controller.py`；核心 `get_servo_position()`（库 **137–141 行**）、`read()`（库 **约 90–115 行**）。 | 该代码使用 `/dev/ttyTHS1`、Jetson.GPIO 半双工方向引脚，**直连舵机**，不是现车的 RRC USB 串口路径。 |
| S8 | `4.总线舵机二次开发教程-STM32版本.zip` | PDF **案例 1 获取总线舵机信息**；`案例1/.../RosRobotControllerM4_serial_servo_read.rar` 提供教学工程。对照 S9 的已提取 C 源码。 | 教学例程有舵机运动与刷机演示；不要执行示例固件或运动脚本。 |
| S9 | `jetrover_stm32_reference_20261120.zip`（先前收到） | `servo_read_example/RosRobotControllerM4_serial_servo_read/` 的 `Hiwonder/System/packet_handle.c:150–170`、`Hiwonder/Peripherals/serial_servo.c:71–80`、`Hiwonder/Portings/serial_servo_porting.c:12–48`、`Hiwonder/Portings/serial_servo.h:117–172`、`Core/Src/stm32f4xx_it.c:533–557`。 | 此源码是**参考工程**，不能以其缺陷直接认定本车运行固件缺陷。 |
| S10 | GitHub `docs/real_operation/` 及 `patches/jetrover_readback/` | 已完成本车原驱动 SHA 审计、串口拥有者 Stage A 和 **306 passed / 1 skipped** 离线维护模式候选。 | Mock/PTY/主机 C 桩测试不等于真实硬件持续读回通过。 |

**若需要核对原件哈希**：`arm_pc.zip` SHA256 `ea892d4920b24279f12c8d5934e5276f55a8235adc12eb3478efa11afde8ef22`；RRC 协议 PDF SHA256 `9d0e522adaa8fcd0b19f562e4f513624ab3094f2557868164582a722242b5856`；两个二次开发 ZIP SHA256 分别为 Jetson Nano `dce33660babfe5cfd3a83bf4825a6310133326b1367e046f625a3a6e63d0c64d`、STM32 `2105f19dbc9e3d807e3a5843c00fde26f2c6192f4bb3f67ae34e68a060734384`。这些是资料校验值，不是固件身份。

## 2. 三层通信协议（Codex 可直接据此核对接口）

```text
Jetson Orin NX / ROS2
   └─ RRC 外层帧 [AA 55][Function=05][Length=02][Subcmd=05][Servo ID][CRC8]
      ↓ 现有 ROS driver 持有的 USB-Serial 端口 /dev/rrc -> /dev/ttyACM0
STM32 RRC
   └─ 舵机内层查询 [55 55][Servo ID][Length][Command=0x1C][Checksum]
      ↓ 半双工总线，115200bps（参考例程为 UART6）
HTD-35H / HTS-20H / 其他型号总线舵机
   └─ 内层返回 [55 55][Servo ID][Length][Command][Pulse bytes][Checksum]
      ↓
STM32 RRC 外层上报 [AA 55][Function=05][Length=05]
                  [Servo ID][Subcmd=05][Status:int8][Position:int16 LE][CRC8]
      ↓
ROS2 Board.bus_servo_read_position(id) → Pulse
```

**两个 `0x05` 不要混淆**：外层 `Function=0x05` 是 `PACKET_FUNC_BUS_SERVO`，内层 RRC 数据 `Subcmd=0x05` 是位置读；舵机原生位置读取命令是 `0x1C`（28）。

**外层解析**：`Status=0` 表示成功，`-1` 表示失败；`Position` 是小端有符号 int16。在实际代码里复用厂商既有 CRC8 与帧解析，不根据手册文字自行猜算法。官方 RRC PDF 第 16 页“833 Pulse ↔ 30°”示例与第 10 页的角度示例不一致，不作为关节标定依据。

**协议固有限制**：外层没有事务序列号。同一 ID 重复请求的迟到或重复回复不能只靠 ID、命令、CRC 判别；必须让底层事务与收尾有可审查的关联保障，或者通过已验证的隔离/恢复机制控制风险。

## 3. 官方 `arm_pc` 真实回读调用链（可复用与不可复用）

```python
# arm_pc/main.py:32，GUI 六通道：机械臂 ID1–5 + 夹爪 ID10
self.servo_index = {'6': 10, '5': 5, '4': 4, '3': 3, '2': 2, '1': 1}

# main.py:435–454，'angularReadback' 回调概念（精简）
for i in range(1, self.joints + 1):
    pulse = getServoPulse(self.servo_index[str(i)])
    if pulse is None:
        return
    data.append(str(pulse))

# servo_controller.py:12–17
def getServoPulse(servo_id):
    data = bsc.getBusServoPulse(servo_id)
    return data[0] if data is not None else None

# bus_servo_control.py:56–70
res = self.board.bus_servo_read_position(servo_id)

# ros_robot_controller_sdk.py:432–433
def bus_servo_read_position(self, servo_id):
    return self.bus_servo_read_and_unpack(servo_id, 0x05, '<BBbh')
```

**已证实**：`angularReadback` 触发新硬件查询，**不是**读取 `current_pulse` 缓存。GUI 的 `servoPowerDown` 是另一个分支，位置回读调用本身无需调用 Torque 释放。但教程的“手掰编程”场景会让舵机松弛，勿误把这个操作复制到实验中。

**不可靠的底层**（`arm_pc/ros_robot_controller_sdk.py:418–424`）：

```python
with self.servo_read_lock:
    self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id])
    data = self.bus_servo_queue.get(block=True)  # 无界等待
    servo_id, cmd, success, *info = struct.unpack(unpack, data)
    if success == 0:
        return info
```

问题：队列等待无限期；解包后的 `servo_id/cmd` 覆盖请求变量、不验证与请求一致；没有证明旧回复隔离。GUI `getBusServoPulse()` 的 `time_out=30` 只是外层**重试次数**，不能中断一次已卡死的 `queue.get`。

`servo_controller.py:6` 顶层 `board = Board()` 创建新串口拥有者；`enable_reception()` 还尝试通过 ROS2 Topic 切换接收端。**不要运行整个 GUI**，也不要在原 ROS2 驱动工作时另外打开该串口。

## 4. Jetson Nano 与 STM32 二次开发实例：正确复用层次

### Jetson Nano 直连舵机示例（S7）

```python
# serial_servo_status.py:8,15，简化示意
servo_control = HiwonderServoController('/dev/ttyTHS1', 115200)
pos = servo_control.get_servo_position(servo_id)
```

`hiwonder_servo_controller.py` 的 `get_servo_position()` 调用 `read(id, HIWONDER_SERVO_POS_READ)`，常量为 `28/0x1C`；回包按舵机原生 `0x55 0x55` 帧解析。示例使用 Jetson.GPIO 控制收发方向。**它不走 STM32 RRC，不能直接导入 ROS2 驱动或使用其引脚/设备路径。**

代码自身还有 `for i in range(10)` 与 `if i == 49` 矛盾的异常路径，不要复制其无限/无效重试策略。可复用的是原生命令号、数据字段和校验思路。

### STM32 教学源码（S8/S9）

```c
// Hiwonder/Peripherals/serial_servo.c:71–80（摘取核心行为）
cmd_frame_init(&frame, servo_id, SERIAL_SERVO_POS_READ); // 28 = 0x1C
cmd_frame_complete(&frame, 0);
if (0 == self->serial_write_and_read(self, &frame, false)) {
    *position = *((int16_t*)self->rx_frame.elements.args);
    return 0;
}
```

`Hiwonder/Portings/serial_servo_porting.c:12–48`：配置 UART6 半双工收发、重置接收状态、启动中断、`osSemaphoreAcquire(..., self->proc_timeout)`；初始化设 `proc_timeout=4` tick。参考工程 FreeRTOS tick 为 1 kHz，但 **4 tick 不是 Jetson→STM32→舵机→Jetson 总延迟上界**。

`Hiwonder/Portings/serial_servo.h:117–172`：解析 `0x55 0x55`、ID、Length、Command、Args、Checksum；未见与**当前发起的请求**比较 ID、Command 以及针对位置回复的精确长度。  
`Core/Src/stm32f4xx_it.c:533–557`：UART6 中断中完成发送/正确解析接收时使用共享完成信号量。  
`Hiwonder/System/packet_handle.c:164–170`：STM32 通过 `serial_servo_read_position()` 读取值，再把**请求 ID/请求子命令**写入 RRC 外层回复，因此仅检查外层回显不能证明内层身份。

参考 C 的迟到/共享信号量风险已在离线主机桩复现；**是否与本车实际固件相同仍 UNKNOWN**。不要把教学示例 `app.c`、演示 HEX、示教模式的运动或 Torque 逻辑运行到现车上。

## 5. 当前 JetRover 实机基线和已完成的补丁

以整理时 Git HEAD `4f2ba9bf2744cb67b961f47c806bdc36636943bb` 为准，后续开发先 `git fetch` 核验最新状态：

- 车端：`/home/ubuntu/zsy_projects/Cable_robot`；ROS2 Humble；原厂 SDK 位置：`/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`。
- 原厂 SDK SHA256 `c4dd48f3176a5dce1e8331846691ce995c0b3c2f79341cfc6c9e89dce1418a89`；原 ROS Node SHA256 `4d37d84c6eff0629f5f73ddca2b140cc0af555175fe3f68f8a3359123a716627`（这是**此前确认的文件**，开工需复核当前）。
- 可用 Service：`/ros_robot_controller/bus_servo/get_state`；当前 `/controller_manager/servo_states`、`/controller_manager/joint_states` 是**控制器缓存**，不是实测。FK `/kinematics/get_current_pose` 沿用缓存，不能替代真实反馈。
- 已部署 Stage A：将 `wifi_manager/button_scan.py` 的重复串口访问改为通过 ROS 蜂鸣器接口；已知额外串口打开者消除，但应在部署时重新检查 FD 快照。
- 当前 **SDK 维护模式仅是离线候选**：`patches/jetrover_readback/guarded_readback.py`、`build_candidate_patch.py`、`sdk_bounded_readback_candidate.patch`；有界完整写入、故障锁存、维护模式禁止运动、单串口发送锁、可选 TIOCEXCL；**未安装到原厂运行文件**。候选仍 `DISARMED`、`KEY_REUSE_UNSAFE`，不支持可靠重复相同 ID 的在线采集。
- 历史选定回归 `306 passed / 1 skipped`（Skip 为缺 FastAPI），只证明离线代码逻辑；首次真实 ID1 RPC `NOT_RUN`，六路真实状态 `BLOCKED`，真实 Observation=0。
- SDK 原节点启动和中断路径可能发送 PWM/底盘零速命令，整车 bringup 可能执行 `init_pose`；所以修改厂商运行文件、重启驱动、重开串口需要提前确认进程生命周期与是否发生非预期动作。

仓库现有权威记录：

- `docs/real_operation/SERVO_READBACK_DESIGN.md`
- `docs/real_operation/R1_SDK_MAINTENANCE_PLAN.md`
- `docs/real_operation/R1_R2_ACCEPTANCE.md`
- `docs/real_operation/HARDWARE_INTERFACE_AUDIT.md`
- `docs/real_operation/END_TO_END_INTEGRATION.md`

## 6. 更新后的修复决策与实施顺序

### 阶段 1：完成上位机最小、可审核修复（可 SSH 开始，离线测试）

1. 检查 `feat/real-operation-pipeline` 最新 HEAD、工作区、SDK 加载路径与哈希；无新分支。基于现有候选开发，**不要创建第二套串口驱动**。
2. `bus_servo_read_and_unpack()`：有限锁等待、有限 TX（写入超时 + 完整字节数校验）、有限响应等待；验证外层 CRC/长度/ID/子命令/状态；失败时锁存故障并清除部分状态；ROS Service 不得在失败时返回伪成功。一个客户端超时不得被当作服务端事务已终止。
3. 同一物理串口由车上现有 ROS 驱动统一收发；避免 GUI 或实例程序重新打开 `/dev/ttyACM0`。发送与读取采用一致的事务所有权，既有运动链路保持兼容；维护模式不允许运动/Torque。
4. 基于新资料补充测试：合法位置帧、无响应、短写、外层错误 ID/命令/长度/CRC、旧回复、重复查询、超时后故障锁存、串口打开冲突、原有行为回归。重复查询 Mock 通过**不代表**内层风险自动解除。
5. 设计实际部署与回滚审核方案（只做项目目录内候选修改，用户确认后才能改变运行厂商驱动）。

### 阶段 2：受控单 ID → 六 ID → 连续读取（需要实机审批）

按阶段授权逐步测量：第一次只查询 ID 1 并记录请求和回复、raw Pulse、主机时间与延迟；观察是否卡死。再验证 ID 1–5 和 ID10 各一次。**同 ID 低频重复读**必须有已验证的事务关联/隔离依据；遇到超时和不明数据不自动进入下一轮，也不能仅通过 `sleep/queue.clear` 宣称恢复成功。通过真实数据判断实际需求，不要求事先证明“固件永远不会出错”。

若现有 STM32 固件能稳定满足读取和异常处理，继续只改 Jetson；若抓包或实测显示底层错配/超时恢复缺陷，才设计 STM32 最小修复。若修改下位机则必须先取得本车兼容的固件构建与恢复依据，不能用教学工程替代生产版本。

### 阶段 3：状态转换与真实 VLA 推理

`ID1–5 → joint1–joint5`，`ID10 → r_joint`；实测 Pulse 经关节方向、零位、限位标定转换为 rad；必要时用经过验证的 URDF/FK 推算末端位姿。六路顺序读取必须记录每路采样时间不确定性及总采集跨度，不能把主机到达时间伪装成舵机采样时刻。现有 Observation 对同步差 `20ms`、新鲜度 `0.5s` 有约束，是否满足需实测。

继续复用 `hardware_feedback.py`、`robot_state_provider.py`、`real_observation.py`、`PolicyClient`、`vla_server`，得到 ≥10 条有效真实 Observation → 上传 AutoDL 真 SmolVLA GPU 推理 → JetRover 记录有效原生 `[50,6]` Action 和 Request ID、RTT、GPU 时间；**始终 record-only，不执行模型动作**。

## 7. 验收定义与事实边界

| 验收项目 | 通过条件 | 当前状态 |
|---|---|---|
| R1 SDK 修复 | 离线错误路径与回归通过，部署包审核、实际加载版本可核验 | 候选已离线测试，未部署 |
| R1 单 ID | 用户批准后获得一次真实 ID1 Pulse + 时间与请求响应证据 | NOT_RUN |
| R1 六路与重复 | ID1–5/10 真实读取、重复 ID 低频验证、错误/迟到处理与采样跨度记录 | BLOCKED |
| R1 关节位置 | Pulse→rad 映射和零位/方向符合实物，可辨别 measured/cached/commanded | BLOCKED |
| R2 Observation | ≥10 条实测 RGB + 五关节 + 夹爪，时间及来源校验通过 | BLOCKED |
| R2 云端 | 同 Git SHA、AutoDL 真 SmolVLA、返回并记录 `[50,6]`，ID 与耗时可对应 | 历史合成数据 PASS；真实数据未做 |

**最终技术判断**：读取协议已明确、舵机具备反馈能力、Python SDK 缺陷明确；是否必须修改本车 STM32 固件仍未定。所有“参考工程存在风险”的论述不得写成“本车硬件已确认故障”。
