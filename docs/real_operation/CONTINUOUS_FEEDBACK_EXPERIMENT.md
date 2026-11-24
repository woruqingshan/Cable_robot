# R1连续反馈与受控A/B实验

正式分支feat/real-operation-pipeline；开发起点4314ae9。
这是来源/标定未验证的连续实验路径，不是可信闭环或SmolVLA状态验收。
默认20Hz，可配置0<hz<=50；没有总查询次数或运行时长限制。

## 运行边界

- ReadbackGate保留原可信epoch/KEY_REUSE_UNSAFE及有限诊断语义。
- ExperimentalReadbackGate是明确独立的重复实验入口，不能arm成可信路径。
- 复用PositionCapture、原接收线程/CRC解析器、SDK和同一个Board实例。
- 采集线程串行查询六路，周期超限跳过积压而不补发突发请求。
- 读取/报文错误锁存、数据失效并停读；没有清队列、sleep或自动重试来假定协议恢复。
- ROS/记录使用有界队列。停止记录、磁盘/队列错误不会停止正常采集。
- 独立反馈JointState使用unset采样时间戳；原始主机时间和unknown来源保留。
- RobotStateProvider的目标缓存与原始反馈隔离；不输出正式VLA Observation。

## 唯一串口与运动

固定包复用原维护SDK，启动时仍跳过运动/PWM/Torque初始化；原厂运行文件不改。
唯一串口工作线程在读取事务边界协调控制，完整帧使用同一有界writer。
停止请求优先；最坏等待受读取期限约束，不能宣称硬实时停止或物理安全认证。

默认没有运动能力。只有外部审核路径及明确运动授权引用齐全、文件SHA匹配、
当前A匹配新鲜反馈时，才允许一次B然后一次A。仅限3–5个机械臂关节；
夹爪/底盘/Torque/其它目标拒绝。复用ArmInterface、FixedTrajectoryExecutor及限位检查。
运动反馈发生读错、突跳、非参与关节变化或明显冻结时，发参与关节停止请求；
不自动返回A。串口失效时停止指令可能无法发送，必须报告现场人员处置。

## 部署与静态验收

用户已确认静止、监督和安全断电手段，授权本轮静态实验。
执行前核对设备1a86:55d4、序列号5A25000846、实际接口/FD、原文件和固定包哈希。
新启动周期如恢复了原厂驱动，先加临时Restart=no，再只SIGTERM该子进程；
不重启bringup。隔离ModemManager后，确认0拥有者才启动一次实验包。
若再次DTR失败，仅按本轮授权在重新核验后做一次定向C3；不循环重启/reset。

prepare_feedback_stage.py只生成版本固定包与清单，不打开串口。
record_feedback_trial.py只控制记录窗口，不发运动，结束后检查采集继续。
静态窗口30秒；输出真实CSV/JSONL、指标、图像，不以理论600组替代实测。
动态窗口约20秒，具体A/B路径和运动授权确认后才执行；状态未知不填模拟动态曲线。

私有接口前缀/idc_vla/real/feedback：
- status：source_sha、有效性、故障、周期/跳过统计和记录状态。
- start_recording / stop_recording：独立录制，生成不覆盖的run目录。
- feedback_joint_states位于/idc_vla/real/feedback_joint_states。
- reviewed_servo_command / reviewed_stop_command：默认无能力，仅审核计划可用。
- arm_reviewed_motion / stop_motion：一次计划能力和优先停止，不含Torque。

运行时Restart=no与ModemManager屏蔽只在当前启动周期有效；不能将Git回退当硬件回退。
部署、实测结果、实际运行SHA和退出状态在experiment_report.md中记录。
