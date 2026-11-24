# R1 / R2 当前验收：连续反馈实验

日期2026-11-24；唯一分支feat/real-operation-pipeline。

**R1 Final尚未通过：静态30秒成功，停录后持续采集发生DEVICE_ERROR，动态未执行。**
[完整实验报告与图表](experiments/r1_continuous_20261124/experiment_report.md)。
[结构化证据](experiments/r1_continuous_20261124/evidence_summary.json)。

|项目|当前状态|证据边界|
|---|---|---|
|连续采集模块|IMPLEMENTED|默认20Hz，可配置；无总查询/时长限制；保持unknown来源|
|串口/调度|STATIC_PASS|唯一Board/FD，串行有界事务，无调度积压或跳过周期|
|可信/有限保护|PRESERVED|原KEY_REUSE_UNSAFE不删除，独立实验重复不升级可信来源|
|异步独立录制|LIVE_PASS|30秒停录后采集继续；600组完整写入，未丢队列|
|静态窗口|PASS|600组、3600/3600外层回复；19.998218Hz；3600对TX/RX离线核验|
|静态延迟|MEASURED|Mean/P95/P99/Max=3.509593/5.716457/7.175272/11.837745ms|
|六路主机跨度|MEASURED|Mean22.118906ms、Max31.687296ms；不满足旧20ms同步条件，未放宽|
|持续运行可靠性|NOT_PASSED|在周期5995看到DEVICE_ERROR；锁存并停止，无自动重试|
|具体错误ID/Status|UNKNOWN|旧包停录后未独立保全故障帧，不能猜测补齐|
|统一读/运动串口|IMPLEMENTED_OFFLINE_TESTED|模拟串口+真实ROS通过；实机运动并发未验收|
|动态轨迹|AUTHORIZED_NOT_RUN|用户批准ID1–4 A→B→A，但反馈在arm前已失效；运动0|
|真实状态转换|STATIC_PASS_DIAGNOSTIC|ID1–5与ID10分离，名义rad与缓存隔离，采样stamp unset|
|物理标定/内层关联|UNKNOWN|零位/方向/夹爪、设备采样时刻、STM32事务关联仍未验证|
|故障留存修补|OFFLINE_PASS_NOT_DEPLOYED|停录时也保存故障帧/日志；改进具体错误提示和停止清理|
|FK|NOT_RUN|既有依赖缺失，本轮未安装或重写FK|
|完整真实Observation/R2|BLOCKED|来源、标定、时间同步和当前有效反馈均未满足；本轮未调用AutoDL|
|原厂/备份|PASS|53份源文件与备份哈希一致；无运行厂商文件替换或刷写|
|USB/系统|保持|本轮无DTR失败、USB reset/rebind、bringup重启或依赖升级|

## 测试与版本

最终相关回归416 passed、2 skipped，27.30秒；PyKDL、FastAPI未验收。
新增真实ROS/模拟串口测试覆盖唯一端口、停录后继续、获准命令协调、故障停读与独立留存。
Mock不代表实机动态通过；不存在动态性能数值或回位误差。

实际运行包a6c828c3e6d26d41c1e94e401d42455739890e80。
故障修补94cd090d209e42f1b32d6addcaf06d68ff9ebc69已准备，未部署到现场。
结果文档最新提交不代表实机已切到新代码。

当前PID8413持有唯一串口FD，进程active仅表示仍在报告故障，周期数固定5995。
父服务PID853 Restart=no，按键代理PID834未重启；ModemManager临时屏蔽。
正常控制未恢复；不自动重开串口、恢复bringup或强制返回A。
运行时保护在重启后消失，不能作为永久禁用启动动作的保证。

下一步最小工作是审阅通信故障恢复边界，再在带独立故障留存的版本下受控复测，
取得Servo ID/Status/原始帧后针对处理。当前没有证据要求修改或刷写STM32。

R1 status: PARTIAL_STATIC_PASS_DYNAMIC_BLOCKED_BY_DEVICE_ERROR。
R2 readiness: BLOCKED。
Actual motion commands: 0。
Live dynamic feedback: NOT_RUN。

历史有限实测：[R1_C3_D2_D4_RESULT_20261124.md](R1_C3_D2_D4_RESULT_20261124.md)。
