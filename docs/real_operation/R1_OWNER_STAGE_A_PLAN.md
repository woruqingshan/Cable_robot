# R1 Stage A：移除按键程序的已知串口竞争（已按授权部署）

日期：2026-11-20。唯一分支feat/real-operation-pipeline；本轮起点e0f151e。
本方案只处理已知的第二个串口打开者，不代表SDK、固件或舵机读回已经修复。

## 1. 实际运行与备份

- SDK和ROS节点实际从ros2_ws/build路径解析，最终resolve到ros2_ws/src/driver/ros_robot_controller。
  SDK SHA c4dd48f3…、节点SHA 4d37d84c…均未改。
- 部署前button_scan.service：PID809，User=ubuntu，Restart=always，KillMode=control-group。
- 当前start_app_node.service：PID815；其中ros_robot_controller PID3756持有串口。
- 管理员只读/proc快照中只有按键PID809和驱动PID3756打开ttyACM0，不可读FD目录数=0。
  这是时点观察，不证明未来无法再打开串口。
- start_app_node.service启动整车bringup；bringup含init_pose/action_name=init。
  因此本方案不停止、重启或改动该服务，也不重启ROS驱动。

备份已逐文件校验：
/home/ubuntu/zsy_projects/real_operation_backups/r1_before_owner_stage_a_20261120T101317Z/
共53个文件，134,200字节，包括SDK/节点、controller/servo_controller、启动入口、按键脚本及两个服务配置。
manifest.json SHA256：
69f8948912ad2fa96353e368a995085d8ab233b618e71dde12e73c7578733b80。

变更目标涉及的原始配置已备份。用户.zshrc与/etc/wifi不读取、不复制、不改动；
原unit对用户shell环境的引用在新配置和回滚时保持原样。
备份与加载路径证据见evidence/r1_predeployment_backup_20261120.json、
evidence/r1_full_proc_owners_20261120.json。

## 2. 已准备实现

维护工具位于patches/jetrover_readback/，不进入VLA观察/推理包：

- wifi_button_adapter.py：默认仅校验源码和输出检查结果。
  明确--run才运行经SHA锁定的原按键程序。
- 只在内存AST中移除那一条SDK Board导入，以仅有set_buzzer的ROS代理替代；
  原按钮GPIO逻辑、Wi-Fi重置和用户按键关机逻辑的其它AST节点逐一保持一致。
- 原/home/ubuntu/wifi_manager/button_scan.py不改写。
- 仅使用现有/ros_robot_controller/set_buzzer和BuzzerState消息；
  无Serial/Board实例、无舵机/底盘/夹爪/轨迹接口、无串口回退。
- 提示音进入有限异步队列，ROS故障不会阻塞按键Wi-Fi/关机流程；
  未送达提示音可能被丢弃，不能宣称故障时仍保留直接串口蜂鸣器能力。
- 保留原SDK整数毫秒值，显式处理ROS float32传输后可能截断的时长；
  已用原SDK set_buzzer函数的实际字节结果离线对照。
- prepare_button_stage.py只生成固定版本的独立包和建议drop-in，
  不安装、不调用systemctl、不运行按键逻辑。要求完整Git SHA、干净工作区及有效备份。

原按键脚本SHA256：
ea04002b6b5095c4224706639ad5da0f821fdf3c22ea7a4ecd018acbe9b7e4dc。
原button_scan.service SHA256：
45972d2cc0f5b995cb7232f9aae8d464c732b62581e70389b8a779a4d8dca635。

## 3. 具体变更范围

经用户单独批准后，计划：

1. 核对上述原文件、备份和独立包的SHA，确认目标drop-in尚不存在。
2. 只新增：
   /etc/systemd/system/button_scan.service.d/90-idc-serial-owner.conf
3. 该drop-in仅覆盖button_scan.service的ExecStart，保留原zsh环境来源，
   改为执行独立包中的代理入口和原按键脚本。
   原User、Restart、日志配置等均继承原unit，不修改.zshrc、Wi-Fi设置或SDK。
4. systemd daemon-reload，然后只重启button_scan.service一次。
5. 被动检查服务、进程FD和DDS图，不主动按键、不发布测试蜂鸣器消息、不发舵机RPC。

独立包放在：
/home/ubuntu/zsy_projects/real_operation_deployments/r1_owner_stage_a_<完整GitSHA>/
包含wifi_button_adapter.py、button_scan.override.conf和manifest.json。
manifest记录代码SHA、模块/配置哈希、备份哈希、操作范围及回滚范围。
源码提交后生成包，运行不依赖未来分支工作树变化。

## 4. 授权前条件与风险

- 必须取得本方案的单独部署/按键服务重启授权。
- 用户确认两个实体按键均已释放，小车及机械臂静止，现场能够监督。
  原按键逻辑保持不变：按键持续按下可能重置Wi-Fi或关机。不能用真实按键模拟作为此次验证。
- 重启期间按键检测短暂中断；蜂鸣器新增对ROS驱动可用性的依赖。
  ROS故障时提示音可能缺失，但不会退回第二个串口驱动。
- 停止旧按键进程会释放它的串口FD；ROS驱动保持运行，不主动关闭/重开其端口，
  不修改串口参数，也不发送复位、Torque或运动命令。
- 执行前后的只读/proc核对均必要；即使只剩一个打开者，后续还需要审查真正的
  内核独占机制、MCU内部事务所有权、有界TX和读回复关联。
  不能把PySerial advisory flock或一次FD快照当作完整协议保证。
- 本方案不解决本车固件身份未知、内层回复关联与超时收尾问题，
  不激活SDK候选、不解除KEY_REUSE_UNSAFE。

## 5. 验收与停止条件

本次只接受以下有限结果：
- 原SDK、ROS节点、按键脚本和基础unit哈希不变；start_app_node进程未重启。
- 按键代理进程正常，DDS中可见/idc_vla/real/wifi_button_buzzer_proxy及唯一预期蜂鸣器发布端。
- /dev/rrc当前打开者只剩既有ROS驱动；没有代理直接打开串口。
- 未主动触发Wi-Fi切换、关机、提示音、舵机读取或任何运动。

若启动失败、ROS代理不可见、仍有额外打开者或原文件发生未预期变化，停止本阶段，
保留日志和清单，不继续SDK部署或读回测试。蜂鸣器实机功能未主动测试时须记NOT_RUN。

## 6. 回滚（同样需在已批准维护范围内）

1. 确认已安装drop-in哈希与本包一致；不覆盖或删除其它用户配置。
2. 只移除90-idc-serial-owner.conf，daemon-reload，再只重启button_scan.service。
3. 核对原unit/脚本哈希、服务状态和进程；原SDK始终未改，不需要重启驱动。
4. 原按键逻辑恢复后会重新打开第二个串口FD，因此硬件读回必须继续禁用。
   不能以恢复按键为由继续已故障的舵机事务。

没有自动应用/回滚开关；准备工具只生成数据。
部署前状态如上；下文实测结果取代原待授权状态。

## 7. 测试与后续

新增33项离线测试与1项隔离ROS生命周期测试全部通过，包括：
- 默认inspect不导入GPIO、ROS或SDK；未知源码拒绝执行。
- 原AST除SDK导入外不变；Wi-Fi/关机/空闲分支使用模拟GPIO且全部os.system调用被拦截。
- 提示音字节兼容、有限队列、阻塞/故障隔离、资源清理、私有ROS上下文及唯一蜂鸣器接口。
- 备份损坏拒绝生成包、原文件不变、完整SHA要求及只生成部署数据。

完整选定回归：263 passed, 1 skipped in 8.84s。
唯一skip仍是缺少FastAPI。VLA隔离断言原样保留；维护适配器已放到patches目录，
未放宽任何断言。测试没有GPIO/串口访问；只有domain117生命周期测试创建真实ROS发布端，且禁止publish，未发蜂鸣器命令。

专项复现：
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/cable_robot_vla:$PWD/vla_protocol/src:$PYTHONPATH"
ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -m pytest -q -rs -p no:cacheprovider cable_robot_vla/test/test_wifi_button_adapter.py

本轮Stage A状态：PASS_KNOWN_EXTRA_OPENER_REMOVED；详见实测结果。
R1实际单ID/六ID/连续读取：NOT_RUN / BLOCKED / BLOCKED。
R2真实Observation、云端真实状态推理：BLOCKED。没有以合成state代替实际反馈。

Stage A完成后，SDK部署/驱动进程管理/首次ID1读取仍需另行提出具体方案并取得授权；
固件问题如不能在上位机解决，则先做对应版本的下位机最小修改设计，不自行刷写。


## 8. 安全的准备命令（不安装、不重启）

代码提交且工作区干净后，在仓库根目录：

```bash
REV=$(git rev-parse HEAD)
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$PWD" \
python3 patches/jetrover_readback/prepare_button_stage.py \
  --output "/home/ubuntu/zsy_projects/real_operation_deployments/r1_owner_stage_a_${REV}" \
  --backup-manifest "/home/ubuntu/zsy_projects/real_operation_backups/r1_before_owner_stage_a_20261120T101317Z/manifest.json" \
  --backup-sha256 69f8948912ad2fa96353e368a995085d8ab233b618e71dde12e73c7578733b80
```

生成后先审阅manifest和override文件。安装该override、daemon-reload和重启按键服务均是
待单独批准的后续操作。不得将准备工具成功当作部署成功或硬件验证成功。

## 9. 已授权的Stage A实测结果

用户单独批准Stage A，并确认小车静止、按键释放后，已完成：
- 新增唯一drop-in、daemon-reload、仅重启button_scan.service。
- 固定部署来源94a4e449b76949dea6bbfa965f73030c550eb08d。
- 新按键代理PID9606，active/running、NRestarts=0。
- 整车服务PID815与ROS驱动PID3756保持不变。
- 管理员/proc快照无不可读FD目录，仅驱动PID3756的fd38打开串口。
- 原53文件SHA全部不变，原按键脚本/SDK/ROS节点未修改。
- DDS可见/idc_vla/real/wifi_button_buzzer_proxy，蜂鸣器发布端连接原驱动。
- 没有主动提示音、真实按键、Wi-Fi/关机测试；物理功能验证NOT_RUN。
- 没有舵机RPC；SDK未部署，内核独占尚未在物理端口验证。

证据：evidence/r1_owner_stage_a_live_20261120.json及post_proc/post_dds两份快照。
固定部署包及回滚范围仍按上文；后续文档提交不改变运行包来源。
SDK后续准备见[R1_SDK_MAINTENANCE_PLAN.md](R1_SDK_MAINTENANCE_PLAN.md)。
