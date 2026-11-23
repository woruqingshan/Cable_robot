# R1 M1＋D1 实机结果：2026-11-23

## 结论

M1维护替换PASS。D1单次ID1位置查询PASS_OUTER_FRAME_CAPTURE。
收到RRC外层成功位置回复，Raw Pulse=502；不是连续采集或物理标定验收。
没有执行其它ID、重复读取、Torque、运动指令或VLA动作；没有AutoDL联调。

## 实际结果

|字段|数值/状态|
|---|---|
|运行源码SHA|b020afb16f8112b479c2104f9af7f52e676ec891|
|固定包manifest SHA256|ff4f441efb05d71f8e5f7c91e9747ce3b363b57cc4e472682e5bfc15cfe22cdf|
|舵机ID / 子命令|1 / 0x05|
|Status|0|
|Raw Pulse|502|
|TX尝试帧|aa55050205016f|
|实际CRC通过RX帧|aa550505010500f60103|
|主机事务开始→主机解析回调|2.723470 ms|
|捕获方法总耗时|2.975359 ms|
|客户端往返|5.204781 ms|
|服务请求 / 自动重试|1 / 0|
|源类型 / 内层关联|unknown / 未验证|
|设备采样时间|None|
|Observation可用|false|
|gate终态（返回时）|COMPLETE_UNVERIFIED；used_keys=[[1,5]]|

主机单调时间保留在JSON；不解释为舵机采样时间、控制周期或相机同步证据。
离线独立复算CRC及解包确认帧长度、ID、Command、Status与Pulse；没有再访问硬件。
单次1/1不能估计持续成功率、Hz或六路总采样跨度，也不能排除随后迟到/重复帧。
未经校准，不将502换算成已验证的rad。

## M1实际执行

用户单独批准M1＋D1，并确认静止、现场监督后：
1. 复核Git、6个包文件、manifest及53份原文件/备份，均一致。
2. 新增临时运行时覆盖：
   /run/systemd/system/start_app_node.service.d/91-idc-r1-no-restart.conf
   内容[Service] / Restart=no；daemon-reload，不重启整车。
3. 通过pidfd向经核验的原驱动PID4186发送一次SIGTERM；
   约491ms后观察退出，串口打开者变为0。
4. 整车服务PID799始终active/running，NRestarts=0；没有重新运行bringup/init_pose。
5. 启动idc-r1-readback-b020afb.service，Restart=no，维护进程PID9714。
   原厂源码未替换，使用固定版本独立覆盖包。
6. 被动DDS及参数元数据检查通过：维护参数true、reference正确；
   唯一控制类订阅为原蜂鸣器接口，无运动/Torque/PWM订阅。
   参数查询1次是元数据RPC，不是位置读取。
7. 只调用一次probe_id1_once；回复如上，之后停止所有位置查询。

TIOCEXCL在固定维护构造路径中申请，启动未报错；未尝试第二次物理open验证其效果。
管理员FD快照只见维护进程fd21，不可读FD目录0；不宣称能阻止特权程序再次打开。
固件身份、内层回复关联和USB重开复位影响仍Unknown；
本次服务正常不证明板卡没有发生复位，也没有独立物理运动监测证据。

## 当前保留的运行状态

- 维护服务active/running，PID9714，NRestarts=0，Restart=no。
- 整车父服务active/running，PID799，NRestarts=0，临时Restart=no。
- Stage A按键服务PID784未重启。
- 原厂SDK、节点和其它53份源文件保持原哈希；未刷固件。
- 正常执行器控制接口仍停用，诊断一次机会已消耗。
- 不自动退出/重启维护驱动，不自动恢复原控制驱动或整车自动重启配置。
  恢复原驱动可能发送原PWM/电机启动指令，须另行授权。
- 已达到本次M1＋D1授权终点，未进入六ID或同ID重复测试。

## 证据

车端原始目录：
/home/ubuntu/zsy_projects/real_operation_evidence/r1_id1_b020afb_20261123T083017Z/

包含授权范围、前后FD清单、原驱动退出、维护元数据、D1唯一请求意图、
完整服务回复、离线帧解码、服务状态、日志及逐文件SHA清单。
Git内摘要：evidence/r1_m1_d1_20261123.json。
运行包来源固定为b020afb；本结果文档后续提交不改变运行源码。

## 后续边界

本次证明USB/RRC→舵机位置查询→Python/ROS服务可以产生一次有效外层位置回复。
支持继续准备分阶段受控采样，不证明必须修改本车STM32固件。
其它ID、同ID重复、故障恢复及六路时序仍须进一步实现/验证并分别按授权范围执行。
不能重启进程以冒充新的干净事务epoch，不能用队列清空或固定sleep解除KEY_REUSE_UNSAFE。
R2完整真实Observation仍BLOCKED；继续保存raw Pulse，待来源、标定和时钟证据满足再接入。
