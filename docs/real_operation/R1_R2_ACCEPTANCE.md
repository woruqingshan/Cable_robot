# R1功能验收与R2入口：2026-11-24

[本轮联合实验报告与全部数据](experiments/r1_f1_joint_20261124/experiment_report.md)。
R1基础功能：PASS_WITH_LIMITATIONS。本轮结束集中开发，不延伸VLA执行。

|项目|结论|证据边界|
|---|---|---|
|20Hz连续反馈|PASS|正常无次数/时长上限；一个Board/串口；读写有界串行|
|静态30秒|PASS|600组，3600/3600有效外层回复，19.999491Hz|
|动态约20秒|PASS|403组，2418/2418，19.999647Hz，同一进程无重启|
|真实往返|PASS_BASELINE|ID1–4完成A→B→A；用户现场确认无意外动作|
|读写协调|PASS_BASELINE|两条运动报文，六路反馈连续，未观测冲突/丢组/周期跳过|
|回位相对初始|MEASURED|ID1–4中位Pulse误差[-4,+1,-1,0]；非物理精度标定|
|未参与ID5/10|PASS_BASELINE|仅只读，约1Pulse波动；无Torque/底盘动作|
|采集/有效性/权限分离|PASS_SOFTWARE|无效组不必退出线程；恢复数据不恢复运动|
|已确认终结设备错误|PASS_MOCK|有限预算、下一周期继续；实际Status白名单默认空，需语义证据|
|未知Status/超时/错配|PROTECTED|暂停关联风险查询但保留诊断/停止通道|
|独立故障留存|PASS_MOCK|不依赖录制，原始包、Status/ID、时刻、版本和决策|
|真实DEVICE_ERROR恢复|NOT_RUN|本轮窗口未自然复现，未知根因没有被宣称解决|
|状态转换/ROS2|PASS_UNCALIBRATED|真实Pulse→名义rad，目标缓存分离，采样时间未知|
|既有功能回归|PASS|424 passed，2 skipped；PyKDL/FastAPI仍未验收|
|完整物理标定|DEFERRED_R2_A|零位/方向/安装偏差/夹爪/URDF-FK未验证|
|完整Observation/AutoDL|NOT_RUN|本轮无需云端；平均六路主机跨度约22ms，旧20ms同步条件未放宽|

运行SHA：73aafa9ecac071206f17afb095eb938e5a56a670。
客户端修补5b9423e：处理授权发布滞后；只对同SHA计划、空命令历史和健康状态使用未用能力。
首次无运动的失败前置试次保留；成功试次dynamic_trial_confirmed.json，不能混同。

当前采集器继续运行，运动能力已用完；未恢复正常整车控制。
临时Restart=no和ModemManager屏蔽仍在当前启动周期生效，重启后消失。
原厂SDK/节点未改，不刷写STM32、升级依赖或启动第二串口。

R2-A入口：各关节零位/方向/Pulse→rad、实际限位与偏差、夹爪距离和方向、
URDF/FK一致性、D405接入、真实Observation时间匹配。基础R1通过不等于可信闭环。
后续在R2联调中积累可靠性，不为验收额外进行长时间专项压测。
