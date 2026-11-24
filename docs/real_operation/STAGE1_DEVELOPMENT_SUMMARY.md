# Stage 1 开发进度与实验总结

日期2026-11-24；分支feat/real-operation-pipeline。阶段目标：真实视觉+真实读取记录→云端模型→本地原生动作记录；不提前实现论文的末端目标计算、约束轨迹适配或执行算法。

## 起点与本轮实现

起点d76c263：R1静态600组、动态403组通过，424 passed/2 skipped；未知DEVICE_ERROR后查询暂停。相机旧配置仍为Dabai，尚无真实视觉和真实状态联合推理。

|任务|复用|本轮实现/原因|
|---|---|---|
|Task1 D405|系统RealSense、real_camera.py、CameraRecordAdapter|camera/ros2/d405_bringup提供薄launch包装；不重装librealsense。真实RGB/Depth/CameraInfo通过已有adapter消费|
|Task2反馈恢复|唯一串口、固定73aafa9维护包、独立故障日志|保留未知终结语义隔离；先恢复会话曾再次遇错，随后由用户人工完全断电重上电并恢复，只读30秒验证通过。没有新增危险清锁/盲重试|
|Task3 Observation|现有codec、Identity、ImagePayload、JointPositions、GripperState、RobotStateProvider及名义转换|独立ResearchObservation，不改变正式Observation。stage1_inspection.py负责静止检查采集、保存及串行提交|
|Task4推理|既有PolicyClient、vla_server、SerialPolicyRunner、SmolVLA官方processor|PolicyClient明确opt-in研究模式；默认仍/v1/infer。新增禁用为默认的/v1/inspect入口，共用原worker、模型、响应Schema和诊断记录，不新建HTTP服务或模型加载器|

主要源码：`vla_protocol/src/vla_protocol/research.py`、`cable_robot_vla/cable_robot_vla/policy_client.py`、`vla_server/src/vla_server/{app,config}.py`、`deployment/local/{stage1_inspection,analyze_stage1_inspection,replay_stage1_inspection}.py`及相机包。录制回放工具显式标为recorded_replay，本轮成功验收数据来自live程序，不是回放。

## 实际恢复与证据边界

参考STM32源码的serial_write_and_read在信号量等待失败时返回-1，随后关闭收发GPIO；但共享完成信号量及迟到中断关联仍存在参考风险，实际车载固件身份尚未确认。这不足以给本车Status=-1发放“安全自动重用事务”的契约。

本轮第一次重新建反馈会话后，完成602组30秒录制，但随后再次DEVICE_ERROR，查询暂停，未重试。不能将主机重新启动等同于STM32事务已经复位。用户随后确认并执行人工断电重上电，建立新的硬件会话；再核验RRC身份、退出原厂串口拥有者、启动唯一只读包。

第二次人工恢复后30.014秒600组六路完整，实测19.9979Hz，3600次读回；单路平均3.329ms、最大12.840ms，停录后继续采集。动作权限未授权。验收为人工恢复PASS_WITH_LIMITATIONS，自动恢复NOT_RUN，根因UNKNOWN。没有为“通过”而将-1加入terminal_statuses；默认白名单仍为空。若再出现未知错误，保留暂停和人工恢复边界。

原厂SDK/启动脚本没有改。按本轮授权使用/run中的Restart=no临时保护并只退出实际原驱动，不重启bringup。两个开机周期的保护和PID均分别核验。没有运动、Torque、USB重绑、STM32刷写或系统依赖升级。

## 相机、状态与研究数据包

D405序列262622274500，RGB/Depth848×480@30FPS配置，RGB8/Z16；CameraInfo和原始header光学frame完整保存。联调订阅RGB约26.5–28.0Hz，Depth约27.9Hz，具体见camera_metrics.json。ROS wrapper4.55.1/librealsense2.55.1/firmware5.15.1.55。相机自动参数日志含power_line_frequency默认值越界警告，但流已正常启动；未为此调整已验证曝光/图像配置。深度实际单位本轮参数查询未确认，UNKNOWN。

状态为ID1–5及10，厂商逐关节init与方向换算；不是六自由度臂，不把模型六通道当六舵机。原始Pulse、TX/RX、区间时间和配置哈希随包保存。physical_calibrated=false、source_type=unknown、inner_association_verified=false。组末host接收时刻仅作包内代表值，逐路真实区间未改；相机header保持原有独立时钟标记。

Research Profile只用于推理检查；不是放宽正式可信Observation。实际完整Schema和请求样本见实验目录，保留真实RGB的PNG/base64/哈希。静止场景对本地接收年龄执行0.5秒检查，记录图像/状态到达差；未声称设备同时采样。正式20ms限制未变。深度、内参不输入SmolVLA，不新增模型特征。模型收到的是名义rad与RGB，物理语义尚未与JetRover完成对齐。

## 端到端实测结果

|指标|首批有效请求|30秒连续联调|
|---|---:|---:|
|实际时长s|10.178723024000021|30.250200421000017|
|有效Observation|10|42|
|成功请求|10|42|
|HTTP失败|0|0|
|构建拒绝|4|12|
|有效推理Hz|0.9824415082738162|1.388420553102952|
|RGB订阅Hz|26.506869835858577|28.011277393848243|
|Depth订阅Hz|27.87300617551383|28.37278375885496|
|实际反馈轮询Hz（周期计数）|19.995853729985|20.000341049325687|
|状态订阅Hz|9.796507860043226|9.781174895590732|
|RTT s mean|0.5020295680999993|0.42242279628571905|
|RTT s p50|0.3977228050000008|0.4050112669999635|
|RTT s p95|1.2355014590000053|0.5036572460000457|
|RTT s max|1.2355014590000053|0.6011293230000092|
|CUDA同步model forward s mean|0.3387132607400417|0.2714210597886926|
|CUDA同步model forward s p50|0.2616351991891861|0.2641257159411907|
|CUDA同步model forward s p95|1.0405289977788925|0.3161040358245373|
|CUDA同步model forward s max|1.0405289977788925|0.3450605273246765|
|JSON编码 s mean|0.003828567200002908|0.0037548410238052894|
|JSON编码 s p50|0.0037401809999835223|0.0037721589999932803|
|JSON编码 s p95|0.004497029000020802|0.004296186999965812|
|JSON编码 s max|0.004497029000020802|0.00505773700001555|

共52次真实实时输入、52次成功返回，两端ID逐个对应。全部50×6、300个有限数值；没有截断、缩放、物理关节映射或执行。首批10次满足多轮验收，连续段42次。合计RTT Mean/P50/P95/Max=0.437732/0.405011/0.598850/1.235501秒；纯模型forward对应0.284362/0.263138/0.336782/1.040529秒，首次调用开销包含在统计里。

构建拒绝与HTTP失败分开：前者包含等待首张图像/状态及>0.5秒旧图像；后者为0。原始失败试次、拒绝原因和本地路径保留，没有用合成数据补齐。有效输入频率低于2Hz，实际受传感器录制/PNG处理、单请求在途及新鲜度拒绝共同约束；本轮未分离测量全部开销，不能把剩余时间全算成网络延迟。JSON编码是client内实际计时，不包括所有图像采集封装成本。相机采集/PNG封装单独耗时UNKNOWN。云端preprocess/model/postprocess在逐请求summary中分别有CUDA同步计时，HTTP RTT包含传输、处理与返回，不是纯网络时间。

相较历史合成输入，本轮图像、Pulse、名义角度均来自本轮当前设备。部署仍用同一checkpoint及官方处理器；这里只证明真实输入能触发genuine GPU forward，不能证明模型理解JetRover的关节语义或学会RJ45任务。native_0..5为model_native_unknown，20ms偏移仍是检查日程，不是控制周期。

## 验收矩阵

|ID|项目|结论|
|---|---|---|
|S1-01|统一入口启动D405|PASS|
|S1-02|RGB格式适配|PASS|
|S1-03|Depth/CameraInfo|PASS_WITH_LIMITATIONS：接口/原始值已录，单位/几何配准未确认|
|S1-04|故障后恢复读取|PASS_WITH_LIMITATIONS：人工完全断电恢复；自动恢复NOT_RUN|
|S1-05|20Hz反馈保留|PASS：恢复验证600组/30秒|
|S1-06|真实RGB与反馈联合采集|PASS|
|S1-07|Observation构建|PASS_WITH_LIMITATIONS：独立研究检查Profile，非正式可信Observation|
|S1-08|实际SSH隧道|PASS|
|S1-09|实际VLA服务|PASS|
|S1-10|真实输入上传|PASS|
|S1-11|真实CUDA推理|PASS|
|S1-12|Native Action返回本地|PASS|
|S1-13|多轮和连续实验|PASS：10+42|
|S1-14|完整记录且未执行|PASS|
|S1-15|数据/分析/图表|PASS|
|S1-16|使用说明|PASS，见STAGE1_USER_GUIDE.md|
|S1-17|阶段总结|PASS，本文件|
|S1-18|必要回归|PASS；云端pytest/httpx缺失未安装，另执行ASGI边界检查|

Stage 1真实推理链路PASS_WITH_LIMITATIONS。正式控制输入、物理标定、自动故障恢复和长期运行不在本轮通过范围。

## 部署与证据

实验时JetRover客户端与AutoDL服务源码一致：95621edcb758e653d42473958374802d8ab326e1。独立RRC反馈运行冻结的73aafa9包，并非声称它已升级成同一源码。相机包本轮构建；云端依赖未升级。云端实际RTX 4080/32760MiB，不沿用历史4090的设备假设。HTTP只监听127.0.0.1:8000，机器人隧道18000。

机器人原始数据：`/home/ubuntu/zsy_projects/real_operation_evidence/stage1_20261124T1834`；完整RGB/Depth/逐次请求保留在live_single及live_continuous下。原始反馈在本轮trial JSON指向的recordings路径。云端配置和diagnostics：`/root/autodl-tmp/idc-vla/runs/stage1_20261124`。Git保存代表性完整请求、全部52次动作、指标和文件哈希；不提交模型或凭证。

代码检查点aee695d（D405/研究Profile）和95621ed（来源身份/显式回放），后续记录提交见git log。实际运行版本及最终状态以实验runtime_manifest.json为准，不能把文档HEAD自动当成已运行二进制版本。

## 后续入口

Stage 2依次处理逐关节零位/方向/实际限位、夹爪物理标定、URDF/FK一致性、相机手眼/TCP、真实时间关联及VLA输入输出语义。之后再讨论JetRover示教数据、必要后训练、Action Adapter、受控实际执行和控制闭环，最终进入工作空间约束规划。当前不实现这些功能。

仍存在未知STM32身份与内层关联、Status=-1恢复契约、物理标定和模型动作语义问题；阻止可信自主闭环，但不阻止显式研究检查的真实推理记录。

## 最终运行状态（实验后）

人工恢复后的实时推理和30秒验证完成后，周期8675的ID1请求0x05发生READ_TIMEOUT（约101ms、无RX），系统PAUSED_NO_QUERY。独立证据已保存，未重试、未清锁、未自动恢复运动。当前反馈无效，motion_permission=false，进程仍存活；再次联调必须经明确的人工恢复边界，不能直接假设当前可采集。该状态不抹去已完成的52次实时推理，也不代表故障自动恢复已经解决。

本轮去重回归378 passed、1 skipped（PyKDL）；研究Schema针对性11 passed；云端4项ASGI边界断言通过。早期453/2统计含重复收集，不作为唯一测试数使用。云端pytest/httpx未安装，未为此升级环境。
