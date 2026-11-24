# R1 / R2 当前验收：2026-11-24

唯一分支：feat/real-operation-pipeline。
本轮测试源码67b763c；固定维护包a04beaf；原始证据检查点3636f61。
本轮只更新验收记录，原SDK/D2–D4代码复用且原厂运行文件未修改。

**USB通路与有限D2–D4诊断验收已通过，可信连续反馈和R2仍未通过。**
实测详情：[R1_C3_D2_D4_RESULT_20261124.md](R1_C3_D2_D4_RESULT_20261124.md)。
完整数据：[r1_c3_d2d4_20261124.json](evidence/r1_c3_d2d4_20261124.json)。

|项目|当前结果|证据与边界|
|---|---|---|
|现场与版本|PASS|有人监督、静止、可安全断电；身份、串口所有者和包哈希先核验|
|M1前置|PASS|另获授权；Restart=no保护后仅SIGTERM原驱动，未重启bringup|
|Stage A|保持|按键代理未重启，没有第二个串口拥有者|
|ModemManager|隔离完成|临时停止和runtime mask；udev忽略规则有异常，未认定为根因|
|C3重绑定|PASS|仅核验接口1-2.1:1.0一次unbind/bind；无设备reset、Hub操作或刷写|
|独立打开/关闭|PASS|5.012498ms/4.330509ms；无Errno110，前后FD为0|
|维护包|PASS|a04beaf启动一次，唯一串口拥有者；无运动/Torque订阅|
|D2|PASS_OUTER_CAPTURE|ID2/3/4/5/10：764/15/219/500/499，5/5外层回复有效|
|D3|PASS_FINITE_DIAGNOSTIC|18/18；约0.999484Hz，平均2.750845ms，P95/最大3.558081ms|
|六路主机跨度|PASS_DIAGNOSTIC_METRIC|16.883573/18.153595/16.651849ms；不是设备采样跨度或RGB同步证明|
|D4映射与换算|PASS_SOFTWARE_CONVENTION|五个机械臂关节加r_joint；名义rad来自厂商配置，实物未标定|
|D4 ROS2与Provider|PASS_DIAGNOSTIC_ONLY|实收3组，时间戳unset，过期拒绝，正式状态未被替代|
|FK|NOT_RUN_DEPENDENCY_MISSING|urdf_parser_py/PyKDL/kdl_parser_py缺失；未安装或升级环境|
|可信连续反馈|NOT_PROVEN|STM32内层关联、迟到/重复边界和实际固件身份未知；保护不删除|
|物理标定|AWAITING_PHYSICAL_VERIFICATION|零位、方向、内部偏差和夹爪单位未核验|
|RGB与云端模型|历史PASS|本轮不重复相机/GPU验收；未用旧结果冒充新真实状态联调|
|真实Observation/R2|BLOCKED|valid=0，submitted=0；来源、标定、真实采样时间和图像同步仍缺证据|
|原厂与备份|PASS|53份原文件及备份哈希一致|
|机器人执行|DISABLED|0运动、Torque和VLA执行指令|

## 测试

选定完整相关回归382 passed/2 skipped（25.52秒）。
清除测试子进程LD_PRELOAD后的同组回归382 passed/2 skipped（22.60秒）。
跳过PyKDL与FastAPI，不等于相应功能通过；本轮未跑GPU、全HTTP依赖和原生P2/FCL组合。
真实实机证据另有23对帧校验、3组ROS数据、过期和正式状态隔离检查。
实机测试已到23次查询预算终点，以下命令仅运行离线/隔离ROS回归，不重发实机查询。

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD:$PWD/vla_protocol/src:$PWD/vla_server/src:$PWD/cable_robot_vla:$PWD/workspace_analysis/src:$PYTHONPATH"
env -u LD_PRELOAD ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
 python3 -B -m pytest -q -rs -p no:cacheprovider \
 cable_robot_vla/test/test_wifi_button_adapter.py cable_robot_vla/test/test_r1_*.py \
 cable_robot_vla/test/test_real_sensor_quality.py cable_robot_vla/test/test_real_observation.py \
 cable_robot_vla/test/test_real_node_ros.py vla_protocol/tests \
 cable_robot_vla/test/test_edge_core.py cable_robot_vla/test/test_isolation.py \
 cable_robot_vla/test/test_p2_adapter.py cable_robot_vla/test/test_stage15_adaptation_preview.py \
 cable_robot_vla/test/test_stage23_boundaries.py vla_server/tests/test_shadow_diagnostics.py
```

## 当前状态与后续

维护PID9236独占串口的现场快照；D2/D3已结束，后续查询不自动继续。
父服务PID846 active、Restart=no；按键PID841保持；ModemManager停用且runtime masked。
正常控制未恢复；运行时保护在重启后消失，不据此宣称永久禁用整车初始化。
退出维护/恢复正常驱动或ModemManager须审核实际影响，不能用Git回退替代硬件处置。

下一步补足实际固件关联与失败边界、物理标定、设备采样时刻和RGB同步证据。
完整Observation通过后才复用原PolicyClient/AutoDL；本轮不执行模型动作。

历史D1与维护方案：[R1_ID1_LIVE_RESULT_20261123.md](R1_ID1_LIVE_RESULT_20261123.md)、
[R1_SDK_MAINTENANCE_PLAN.md](R1_SDK_MAINTENANCE_PLAN.md)。
历史USB失败：[R1_C2_RESULT_AND_C3_PLAN_20261123.md](R1_C2_RESULT_AND_C3_PLAN_20261123.md)。

SDK modification required: YES（已有维护覆盖，原厂文件未改）。
STM32 firmware modification required: UNDETERMINED。
Offline tests: PASS。
Live hardware readback: PASS_FINITE_OUTER_CAPTURES（23/23）。
R1 status: FINITE_DIAGNOSTIC_PASS_TRUSTED_FEEDBACK_NOT_YET_PROVEN。
R2 readiness: BLOCKED。
Robot execution: DISABLED。
