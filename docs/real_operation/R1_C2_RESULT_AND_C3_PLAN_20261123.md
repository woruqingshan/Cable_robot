# C2 单次打开失败结果与 C3 受控恢复提案

最新实测（2026-11-24）：C3定向重绑定后打开/关闭通过，D2五路及D3有限18次外层回复全部通过；D4实收3组未校准诊断状态，过期拒绝与来源隔离通过。FK依赖缺失；内层关联、物理标定与采样时间仍未验证，R2保持BLOCKED。当前维护包a04beaf运行，父服务Restart=no、ModemManager临时屏蔽。详见[R1_C3_D2_D4_RESULT_20261124.md](R1_C3_D2_D4_RESULT_20261124.md)。

以下为此前阶段的历史记录，旧PID、授权与状态不代表当前现场。

日期：2026-11-23；分支：feat/real-operation-pipeline。
实测版本：a04beaf95746ab3f4246beda40bba504057a465e。
C3 尚未获批、尚未执行。本文不是自动恢复脚本。

## 1. 已执行结果

用户批准 C2 一次打开/关闭，并仅在完整成功后条件续测。
18:10:45 至 18:11:01（+08），实际 open 尝试一次；初始化总耗时
15.413597661 秒，在 _update_dtr_state / TIOCMBIC 返回 Errno 110。
这是整体初始化/helper 耗时，不能当作单次 USB 总线传输耗时。

- Confirmed：status=STOP_NO_RETRY，USB_DTR_CONTROL_TIMEOUT，fd_cleared=true。
- Confirmed：前后串口打开者均为0，未读到的进程FD目录为0；这不是永久独占的证明。
- Confirmed：没有接收线程、ROS驱动或串口数据写入；位置查询0次，未重试。
- Confirmed：没有reset、unbind/bind、固件操作或运动/Torque指令。
- Confirmed：原SDK与节点哈希未改变；C2时段内核日志没有新增条目。
- Confirmed：条件续测未满足，因此新维护包未启动，D2/D3/D4实机未运行。
- Suspected：USB控制请求应答/状态路径故障；此复现不依赖ROS接收线程和读取事务。
- Unknown：设备/桥接固件、主机内核、线缆/供电各自责任；不能归因于STM32舵机0x1C。

应用层退出/FD清理候选已通过离线测试，但尚未恢复当前USB控制路径。
不能用“FD已清理”推导“设备已恢复”或“协议epoch已清空”。

## 2. 新发现的重绑定风险

ModemManager 1.20.0正在运行（PID779）。现有99-ttyACM0.rules的忽略行
结尾为 ENV{ID_MM_PORT_IGNORE}="1"1"；设备属性未见忽略标记，
但有ID_MM_CANDIDATE=1。现有rrc别名规则还匹配所有ttyACM设备。
这不能证明ModemManager实际探测过本板或造成超时，但重新绑定可能引发探测。

默认网络路由走wlan0；仍需执行前核对SSH连接路径及是否依赖蜂窝调制解调器。
不自动修改该规则、不触发全局udev、不在后台运行新的串口探测工具。

## 3. C3a最小操作（需新的单独授权）

执行前重新核对静止/现场监督、HEAD/清洁工作区、包清单、父服务Restart=no、
0串口FD、USB身份1a86:55d4/0444及序列号5A25000846、接口路径和绑定关系。
任一不符即停，不按旧PID或tty编号盲目操作。

1. 记录ModemManager及DBus别名的原单元状态，确认无蜂窝网络依赖。
   临时runtime mask并停止ModemManager和其DBus激活别名，防止设备重现时后台探测。
   不停Wi-Fi、NetworkManager、Tailscale或整车服务；已有串口FD不为0则停止。
2. 仅对cdc_acm控制接口1-2.1:1.0执行一次unbind、一次bind。
   操作端点是/sys/bus/usb/drivers/cdc_acm/unbind和bind，载荷均为1-2.1:1.0。
   两步间确认该接口已解绑、相关tty节点撤销且无自动打开者，再执行bind。
   同一CDC设备的数据接口1-2.1:1.1可能随主接口释放/重新绑定。
3. 仅观察sysfs/udev和内核日志，限时确认同一USB身份、两个接口绑定和rrc别名。
   允许tty编号变化但必须核对物理设备，不能接受通配别名误指向别的ttyACM。
4. 若只批准C3a，到此停止，不打开串口。若批准附带C2验证，才进行下一节。

上述mask、stop、unbind或bind任一步失败立即停，不自动重试、重置或恢复原驱动。
接口重绑定会重新执行内核初始化、可能改变控制线或复位板卡；无法保证板卡无自主反应。
它仅重建Linux驱动绑定，不是USB设备reset，也不能保证恢复板端控制应答。
不操作上游Hub1-2、相机1-2.2、xHCI，不卸载全局模块，不拔插或循环供电。

## 4. 可一并批准的条件续测

C3a通过后，重新审核干净源码SHA，使用相同probe仅打开/关闭一次，参数不变；
记录新的证据文件。此为恢复操作后的新授权尝试，不属于已用尽的旧C2授权。
若再次超时，停止，不启动维护包或继续查询。

仅在打开、关闭、FD和身份检查全部成功后，允许再启动一次已审核固定维护包a04beaf。
包路径：/home/ubuntu/zsy_projects/real_operation_deployments/r1_sdk_a04beaf95746ab3f4246beda40bba504057a465e/
包清单SHA256：e4a8d3d60bdebc691b9095bfb6a7a7384ec4a259c2faa739d4d337067a2b41de。
该条件分支总共最多两次串口打开；第二次就是维护进程启动，禁止打开第三次。

维护被动验收通过后，复用现有D2：ID2、3、4、5、10各一次；
再按原审查边界执行最多三个1Hz六路诊断周期及D4只读记录，遇错立即停且不重试。
继续携带历史ID1使用记录；重绑定或重开不证明STM32迟到回复已清空。
有限重复结果仍是诊断，不能将关联未知数据宣称可信闭环或送入完整真实Observation。

## 5. 状态保全与回退

原厂53份备份及SDK/节点哈希保持有效；源码回退不等于USB或板卡状态回退。
若unbind成功而bind失败，设备可能留在解绑状态；记录现场后停止，不自动反复bind。
ModemManager的runtime屏蔽将在本轮维护期间保留，退出时报告新建的屏蔽路径和原状态。
恢复它可能重新探测控制板，须先建立经核验的定向忽略或单独评审恢复操作；不静默恢复。
不恢复整车自动重启、不启动原厂驱动；原厂驱动初始化可能发动作，需另行授权。

当前父服务PID799 active且Restart=no，按钮代理PID784保持；串口无打开者，
正常硬件驱动未运行。历史D1仅ID1=502，当前六路结果、Hz/延迟/跨度均NOT_RUN。
完整真实Observation=0，R2仍BLOCKED；本轮没有AutoDL推理或运动。

## 6. 测试与证据

源码a04beaf的既有离线回归：382 passed、2 skipped（PyKDL、FastAPI）。
本次新增的是实机C2失败记录与恢复提案，无生产代码修改，不重复运行相同离线回归。
新增证据JSON、原始文件摘要哈希及Git diff须核验；不把离线通过等同于USB恢复。

原始目录：/home/ubuntu/zsy_projects/real_operation_evidence/r1_c2_20261123T101045Z/
摘要及完整C2报告：evidence/r1_c2_result_20261123.json。
原只读审计证据r1_usb_cdc_20261123.json保持历史含义，不覆盖其当时0次打开计数。

参考（用于机制解释，不证明本机实际发生后台探测）：
- [ModemManager官方设备检测](https://modemmanager.org/docs/modemmanager/port-and-device-detection/)
- [ModemManager官方忽略标记](https://www.freedesktop.org/software/ModemManager/doc/latest/ModemManager/ModemManager-Common-udev-tags.html)
- [Linux5.15 CDC ACM参考实现](https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/usb/class/cdc-acm.c)

SDK modification required: YES（仓库候选已完成，原厂文件未改）。
Offline tests: PASS（382 passed/2 skipped）。
C2 live open/close: FAIL_USB_DTR_CONTROL_TIMEOUT。
Live D2/D3/D4: NOT_RUN。
R1 status: STOPPED_USB_CONTROL_PATH_NOT_RECOVERED。
R2 readiness: BLOCKED。
