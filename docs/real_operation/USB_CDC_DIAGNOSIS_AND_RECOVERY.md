# USB CDC ACM DTR 超时：诊断、应用层修补与恢复方案

最新实测（2026-11-23 18:10 +08）：获批C2仅打开一次，DTR阶段仍Errno110，初始化15.4136秒，FD已释放；没有位置查询、重试或条件续测。USB重绑定及D2-D4实机为NOT_RUN；串口无打开者，正常硬件驱动未运行。C3提案尚未获批；详见[C2实测结果与C3恢复提案](R1_C2_RESULT_AND_C3_PLAN_20261123.md)。


日期：2026-11-23（Asia/Shanghai）。分支feat/real-operation-pipeline。
初始审计阶段基线cb113ce，仅只读检查和离线测试，打开/位置RPC均为0；后续C2结果见页首。

## 1. 结论分级

**Confirmed**
- /dev/rrc仍指向/dev/ttyACM0；设备1a86:55d4、bcd0444、USB Single Serial。
- 控制接口1-2.1:1.0绑定cdc_acm，USB设备路径1-2.1，经上游Hub1-2连接。
- 本轮管理员FD清单无串口打开者；失败服务未重试，父整车服务PID799仍Restart=no。
- PySerial3.5实际从/home/ubuntu/.local/lib/python3.10/site-packages/serial加载。
  serialposix.py SHA256=5d56f98513391e1766a1847a04e2e1202ced4172a152803ba1c823a149a9b6f9。
- D1成功与D2失败包的Board构造函数AST相同，node.main AST也相同；
  因而没有证据支持两版DTR/RTS配置顺序发生变化。
- 失败位置是open中的TIOCMBIC(DTR)，不是RRC位置子命令0x05或舵机0x1C。
- PySerial失败分支已关闭底层FD并设fd=None；现场零打开者与此吻合，未发现遗留FD证据。
- 设备PM control=on、runtime_status=active、runtime_suspended_time=0；
  上游Hub与xHCI当前active，挂起计数也为0。没有证据支持autosuspend是主因。
- 内核仅见16:46:58相机分支1-2.2断开，用户确认曾换相机、调整连接；
  RRC分支1-2.1未见断开。不能把人工换相机直接写成DTR故障原因。
- 旧维护进程关停抛ExternalShutdownException并延迟退出；新的打开阶段又发生Errno110。
  两者时间相关，但现有记录不能证明应用关停异常造成USB设备状态故障。

**Suspected（最可能发生故障的层次）**
关闭/重开过程中的USB CDC控制请求应答/状态路径异常，比新的舵机读取逻辑错误、
配置顺序变化或重复FD更符合已知证据。不能进一步确定责任在USB端点/桥接设备、
主机cdc_acm/tegra-xusb状态、连接/供电或板端固件。

**Unknown**
- 未抓取失败时的USB控制传输，不能确认SET_CONTROL_LINE_STATE失败的具体总线阶段。
- 未核验本车tegra驱动二进制对应完整源码；下述上游5.15调用链是参考，不是本车源码证明。
- 控制板固件身份、DTR/RTS电气意义、开关端口是否复位及复位后的自主动作仍未知。
- 不能依据Errno110断言必须修改或刷写STM32舵机固件。

## 2. 为什么原超时不起作用

实际序列为Serial(None) -> rts=False -> dtr=False -> setPort -> open。
PySerial.open先os.open，再配置termios，然后更新DTR、RTS，之后才完成open。
read timeout=5只约束读取，write_timeout=.05只约束写入；
ReadbackGate的100ms期限在串口初始化完成后才起作用，均不覆盖open/DTR ioctl。

参考上游Linux5.15：
tiocmset -> acm_set_control -> SET_CONTROL_LINE_STATE -> endpoint0 usb_control_msg。
上游acm_ctrl_msg对单个请求使用5000ms期限，不等于整个open/close的应用期限。
Errno110只说明期限内传输未完成，不识别坏的是哪一侧。
不以后台线程超时并遗弃仍在执行的ioctl冒充已恢复。

参考：
- [Linux5.15 CDC ACM源码](https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/usb/class/cdc-acm.c)
- [Linux5.15 USB错误码说明](https://www.kernel.org/doc/html/v5.15/driver-api/usb/error-codes.html)

## 3. 最小应用层修补（未部署）

继续在原SDK候选中修改，不另建串口驱动：
- initialize_existing_serial包装原有唯一Serial实例，严格保留信号顺序和False值。
- 拒绝重用已打开/遗留FD的对象；记录初始化阶段、errno、耗时和FD清理状态。
- 区分USB_DTR_CONTROL_TIMEOUT、USB_RTS_CONTROL_TIMEOUT及一般初始化错误；
  原异常类型/errno继续抛出，不忽略Errno110，不改变dsrdtr/rtscts或跳过DTR。
- PySerial已清理失败FD时不再次close；部分已打开对象只做一次所属资源清理。
- 维护接收线程保存句柄；关停先停止新读取、取消原read内部等待管道，
  join(.5s)后才关闭所属端口。线程未退出则报告失败，不强杀、不抢关可能使用中的FD。
- 维护close幂等，不发flush、reset、Torque、PWM、电机或舵机动作。
- 维护main处理正常关停，使用自有执行器及100ms spin_once。
  自有SIGTERM请求标志、关闭rclpy自动信号处理，避免启动时全局执行器None及
  GuardCondition部分构造竞态；最后由main统一销毁节点和ROS上下文。
- 原普通模式Ctrl+C电机零速语义及原控制报文测试保持；维护模式不执行它们。
- 保留KEY_REUSE_UNSAFE、有限D2/D3预算、unknown来源及Observation拒绝条件。
- 内核open/close仍可能阻塞；上述软件清理修补不是USB端点故障的已验证修复。

测试：382 passed、2 skipped，22.70秒。Skip=PyKDL、FastAPI。
包含实际PySerial.open代码配模拟OS I/O、不同关停时机的真实隔离ROS子进程、
虚拟PTY接收取消/FD关闭、现有控制接口与协议回归。
这些均不是控制板恢复实测。原厂运行文件、已有冻结包和系统服务未改变。

## 4. 恢复分级及本次可申请范围

### C1：应用层修补

代码和离线测试已完成。尚未部署到新进程；仅能改善错误可见性与资源收尾，
不能声称已经让当前USB端点恢复。

### C2：一次受监督打开/关闭（需单独授权）

使用probe_serial_open_close.py：
- 默认inspect_only：读取设备/FD元数据，不构造Serial对象。
- 明确--execute-once、完整预期SHA、干净Git及新输出文件才允许一次尝试。
- 前后管理员FD清单必须可见且无额外打开者，核对1a86:55d4与cdc_acm。
- 使用原参数1Mbps、rts/dtr=False、read timeout5、write timeout.05。
- 只调用同一初始化/关闭helper；不创建接收线程、ROS驱动或发送数据/位置查询。
- 输出open/close耗时、原始异常、清理情况和前后FD快照。
- 失败立即停止，不第二次open、不跳过DTR、不reset/unbind。
- open/close涉及USB控制信号，不能保证不会复位板卡；必须静止并有人监督。
  内核ioctl/close可能超过软件观察窗口，不能据客户端超时直接当作设备已清理。

初始仅运行inspect_only（0次打开）。后续获批C2已执行一次并失败：USB_DTR_CONTROL_TIMEOUT，15.4136秒，FD已清理。条件续测未执行。

### C2成功后的可选后续授权范围

若用户明确一并授权：C2完整成功且关闭确认后，再单次启动新固定维护包。
这一分支会产生第二次串口打开，必须明示；不能从“一次打开关闭”授权自动推导。
维护启动被动检查通过后，复用现有D2五个未查询ID、最多3个1Hz六路诊断周期及D4记录；
任何初始化/通信错误立即停止，无自动恢复。
旧ID1使用历史继续携带；清缓冲、重新打开或进程重启不构成干净事务epoch证明。
未知内层关联的数据仍只作诊断，不能自动接入真实Observation/AutoDL。

### C3：必要时才申请USB重新枚举/连接恢复

只有C2仍失败并审阅新证据后，才提出针对控制板1-2.1的独立操作。
cdc_acm接口解绑重绑、设备USB reset、物理拔插或电源循环均未执行且不在C2范围内。
不得reset上游Hub1-2或整个xHCI，避免影响相机分支；不卸载全局cdc_acm作为快捷修复。
板端复位可能影响执行器，须单独说明机械/供电/串口与原初始化风险，不直接刷HEX。

## 5. 回滚、状态与验收

- 原厂SDK/节点保持原SHA，53份原始备份继续有效；只改仓库中的审核候选。
- 当前无运行串口驱动，正常控制不可用；父整车服务仍运行且Restart=no。
- 不启动原bringup、不恢复自动启动、不擅自恢复原控制驱动。
- C2：FAIL（DTR Errno110）；恢复后D2/D3/D4：NOT_RUN。
- D1 Pulse502为历史记录，不能当作当前六路值；R2完整Observation仍未启用。
- 下一步是审阅并单独批准C3定向重绑定提案，不自动重复C2或跳过ioctl。

原始证据目录：
/home/ubuntu/zsy_projects/real_operation_evidence/r1_usb_cdc_20261123T093735Z/
摘要：evidence/r1_usb_cdc_20261123.json。
