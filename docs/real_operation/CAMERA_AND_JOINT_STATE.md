# Camera 与 Joint State 验收：2026-11-20

## Level 1：PASS

参考用户脚本 ~/zsy_projects/record_rgbd_5s.py，SHA256=33ce34c4f095cb809a2bd5d796ffee63dad1784223bc61f452d690ee661e8985。已阅读 on_rgb/on_depth 实现，复用 CvBridge bgr8 / passthrough 与 sensor_data QoS。脚本 SECONDS=10.0，FPS=30.0 只是写视频配置，不是实测频率。本轮未把脚本另建成 VLA 系统。

首次复验：RGB 640×360 rgb8，约28.196 FPS；Depth 640×360 16UC1，约29.067 FPS。已实际转换并保存 rgb.png、depth.png，以及两份 CameraInfo。证据目录：~/zsy_projects/real_operation_evidence/g1_d3ca94afc6c94d359044975ab1f32a21。

整合后只录制8秒：RGB收到218帧、约27.846 FPS，Depth收到224帧、约29.188 FPS；按2Hz抽样分别保存14张，主动抽样略过204/210帧，不将其声称为驱动丢帧。RGB最大源时间间隔约99.93ms、一次大间隔；精确驱动丢帧数 Unknown。

证据：~/zsy_projects/real_operation_evidence/run_20261120T060516Z_d7880fdad76d4f65b33a09e3e15a6bb4/acceptance.json。PNG及SHA256、每张原始编码/尺寸/frame_id/源时间戳/接收单调时间在 diagnostics/events.jsonl，目录camera/与depth/分开。

共同 frame_id 为 depth_cam_color_optical_frame；不据此断言 RGB-D 严格同步或相机已对齐机器人base。RGB CameraInfo fx=fy=360.0032958984375，cx=323.74591064453125，cy=179.73245239257812；畸变模型rational_polynomial，D=[-0.01779046654701233,0.003028521779924631,0.0002316461323061958,0.000051954197260783985,0,0,0,0]。保存了原始K/D/R/P，未标记外参/TCP已标定。

## 真实六维状态：BLOCKED

缓存实际值：IDs=[1,2,3,4,5,10]，Pulse=[500,765,15,220,500,500]。对应项目约定joint1–joint5/r_joint，单位仍是原始Pulse。Header为控制器发布时间，不是硬件采样时间；采样区间、硬件读取延迟及稳定性未测得，保持null/Unknown。

已确认该流来自内部缓存，只用于诊断。硬件单次RPC为NOT_RUN：SDK持锁执行无期限队列等待，缺少安全有界的服务端读取路径。详见HARDWARE_INTERFACE_AUDIT.md。

整合节点valid=0、submitted=0、responses=0。完整Observation没有生成；没有用缓存、旧waypoint、到达时间或任意Pulse→rad值填充。real_uncalibrated_v1仍须真实状态及时间/单位证据后才成立。

## 安全与边界

节点只创建 /idc_vla/real/diagnostics 与标准 /parameter_events 发布器，未创建GPU客户端（只录制）。没有厂商动作发布器、轨迹/夹爪/底盘请求，没有ArmInterface实例。原始Depth不进入SmolVLA RGB特征。record_only传感器证据为PASS，完整真实Observation录制/回放仍BLOCKED。
