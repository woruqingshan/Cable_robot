# 真实接口审计：2026-11-20

基线 origin/feat/real-operation-pipeline：857ae5113a0813a6f24f9bec6a3dcc88c5c6ae44。
正式源码只在 JetRover 修改；代码提交 e6c62779791bfe92ba8a54b496b01c957301d8a4。

## 历史与版本约束

已读取 [20260810–0811 Notion 记录](https://app.notion.com/p/3b839272420f80959b0ec6993018f7db)。其中记录了 Service 直读卡死、Topic 为缓存、基于已知目标累计的 MOVE/CART/GRIPPER 调试，以及固定轨迹 10/10 拔出成功。这些历史结果不证明 VLA 动作已验证。本轮不实例化带发布器的 ArmInterface，不执行历史轨迹、Torque Release/Hold、初始化或厂商服务重启。

已检查原控制模块、arm_limits/pull_out/insert 配置及厂商接口。ArmInterface.get_servo_states 默认走 Topic；旧 _call_service 的客户端超时不能恢复被阻塞的服务端。

[官方 Quick Start](https://wiki.hiwonder.com/projects/JetRover/en/jetson-orin-nano/docs/1.Quick_Start_Guide.html) 描述 Orin NX 的 Ubuntu 22.04 / ROS2 Humble；[OpenCV 教程](https://wiki.hiwonder.com/projects/JetRover/en/jetson-orin-nano/docs/5.ROS%2BOpenCV_Course.html) 混有 ROS1 终端描述及 ROS2 命令，且指出重启应用会使机械臂回初始位置；[机械臂教程](https://wiki.hiwonder.com/projects/JetRover/en/jetson-orin-nano/docs/7.Robot_Arm_Control_Course.html) 提供 CvBridge bgr8 示例。没有照搬其中运动/重启指令或泛化硬件规格。

## 当前运行接口

PASS：四个既有相机 Topic 均实际收到消息：/depth_cam/rgb/image_raw、/depth_cam/depth/image_raw、/depth_cam/rgb/camera_info、/depth_cam/depth/camera_info。Image 与 CameraInfo 使用 qos_profile_sensor_data。

PASS（接口可见性）：/controller_manager/joint_states 为 sensor_msgs/JointState；/controller_manager/servo_states 为 servo_controller_msgs/ServoStateList。后者包含 Header 和 servo_state 列表，每个 ServoState 有 id/goal/position/error/voltage 字段。字段名称不证明测量来源。/ros_robot_controller/bus_servo/get_state 的 GetBusServoState 服务实际存在；arm/gripper FollowJointTrajectory 接口可见，但没有连接或发送目标。

## 硬件读取：BLOCKED

厂商源码路径：~/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py。
SHA256：c4dd48f3176a5dce1e8331846691ce995c0b3c2f79341cfc6c9e89dce1418a89。

bus_servo_read_position -> bus_servo_read_and_unpack，在 servo_read_lock 内执行 bus_servo_queue.get(block=True)，没有等待期限。服务回调逐项调用；success=True 不保证每项有 position，响应也没有采样 Header。

本轮没有再次触发这个可能永久占锁的调用：单次硬件 RPC 为 NOT_RUN，原因是已确认服务端缺少有界等待，不能用客户端超时证明其恢复。没有宣称新车实测发生了卡死。必须先获得厂商提供的有界读取/可靠回报路径或单独批准驱动层修订，才恢复试验；本轮不改厂商源码。

SDK 除同步查询队列外未建立已验证的独立测量发布路径。厂商 ServoManager.get_position 返回内部 servos 缓存；FK 也必须追溯输入，不能将缓存 FK 作为硬件实测。本轮未调用 FK/IK。

这些约束已落实进 RobotStateProvider 和探针：缓存只进诊断、无弧度/时间戳伪装、硬件 RPC 不自动发送、无连续重试。五关节 ID 1–5，夹爪 ID 10；Pulse 范围按当前配置 0–1000，不等于已验证关节零位/方向。


## R1续审：固件身份（2026-11-20，基线e86d0e6）

本车PCB型号、精确MCU和运行固件版本仍UNKNOWN。USB 1a86:55d4 / bcdDevice=0444
仅为串口侧标识，参考.ioc的STM32F407VET6不能作为本车身份证明。
实际SDK和3秒DDS图没有发现控制板固件版本查询接口；
相机/ROS版本接口和直接返回success=True的init_finish均已排除，未调用任何服务。

当前仍有两个可见串口打开者：PID790 button_scan.py、PID3189 ros_robot_contr，
另247个进程FD目录不可见，未取得唯一拥有者证明，也未停止或修改进程。
限定目录未发现固件文件，相关已读ROS日志为空；不据此推断没有其它版本记录。

R1专项90项离线测试通过，包含参考C的缺陷复现，不是本车协议安全PASS。
最新证据、方案和实测准入条件见[SERVO_READBACK_DESIGN.md](SERVO_READBACK_DESIGN.md)
及[R1_R2_ACCEPTANCE.md](R1_R2_ACCEPTANCE.md)。单ID实测仍NOT_READY。
