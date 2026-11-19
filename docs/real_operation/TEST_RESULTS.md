# 测试结果与验收边界

2026-11-19；待审分支 `feat/real-operation-pipeline`，基线 `dee7ddba62d05098499ce9197cc443905fce21fb`。未提交、推送或部署。

## 最终自动化结果

**133 passed, 1 skipped，3.84 秒。** 在 Jetson 原有 Python 3.10 环境中运行，ROS 用独立测试域 117；该域仅用于测试进程，未改物理驱动环境。没有安装测试依赖或模型栈。

```bash
cd /home/ubuntu/zsy_projects/Cable_robot
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$PWD:$PWD/vla_protocol/src:$PWD/vla_server/src:$PWD/cable_robot_vla:$PWD/workspace_analysis/src:$PYTHONPATH"
ROS_DOMAIN_ID=117 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -m pytest -q -rs -p no:cacheprovider \
  cable_robot_vla/test/test_real_observation.py \
  cable_robot_vla/test/test_real_node_ros.py \
  vla_server/tests/test_shadow_diagnostics.py \
  vla_protocol/tests \
  cable_robot_vla/test/test_edge_core.py \
  cable_robot_vla/test/test_isolation.py \
  cable_robot_vla/test/test_p2_adapter.py \
  cable_robot_vla/test/test_stage15_adaptation_preview.py \
  cable_robot_vla/test/test_stage23_boundaries.py
```

覆盖原有协议、客户端/边界、Stage 1.5、P2 适配与 Stage 2.3 单元行为。新增测试覆盖 RGB/BGR、行填充、无效图像/非有限状态、缺少夹爪、独立源时间、过期/不同步、来源证据 SHA、缓存状态拒绝、PNG/标定清单篡改、唯一目录、会话重置拒绝、原生动作映射拒绝。

3 项 ROS 测试覆盖：只录制启动及发布器集合、模拟时钟拒绝、网络断开后继续录制。网络测试使用明确标记的夹具；没有以夹具记录冒充真实采集。

跳过项是 `test_existing_http_app_records_response_and_validation_error`：小车没有 FastAPI。没有为消除跳过而安装服务器依赖。

## 隔离的云端接口复核

云端现有 Python 3.12 环境有 FastAPI，但没有 httpx 和 pytest。因此没有宣称完整云端 pytest 回归通过。

用**内存中的修改模块**运行 FastAPI ASGI 请求，不开启监听端口、不修改云端项目文件、不运行 GPU 模型、不更改模型环境安装。内存模块的源文件 SHA256 与机器人待审修改对应；输入明确为合成测试夹具。

已检查：有效请求生成响应并记录完整原生数值；无效 JSON 产生 422 并记录错误。另对策略错误、超时、模型启动失败检查对应的错误诊断。它们是接口与记录器验证，不是实际网络传图或 genuine SmolVLA 验收。

最终 app SHA256：`d66527c00eea49f8d1830cb474dbbca07caac4d8197f54e887c1f2d74a079a5b`；config SHA256：`d266bb93e0fc598ea4a81d3e7210f4e854a553a6fbd81696e483394e7e1e7d83`。云端库在测试后仍是干净的 main，未拉取开发分支、未重启服务。

## 真实设备检查

- USB 已确认 DaBai DCW RGB 与深度设备；当前 ROS 被动采样 0 路消息，舵机服务未发现，未发送请求。
- 证据：`/home/ubuntu/zsy_projects/real_operation_evidence/probe_6b91d307dd6c4b7caf97431ec9c1156f/audit.json`。
- 实机只录制节点在当前物理环境启动并退出，0 条有效观测；只创建私有诊断、标准 rosout/parameter_events 发布器，不创建推理客户端。
- 当前 Wi-Fi 与驱动绑定旧 IP 不同，是 ROS 发现问题的线索；没有执行驱动重启、网络修改或初始化。
- NumPy/OpenCV/cv_bridge 导入成功。

## 没有完成的验收

真实 PNG 及 CameraInfo/TF、可信关节/夹爪采样时间、真实 Observation 的录制回放、真实 HTTP 网络传输、实际 GPU 实机观测推理、重复/连续实机推理，均**未通过/未执行**。缺少这些证据时不启动生产部署，不放宽 20 ms/0.5 s 校验，不猜测通道语义。

Gazebo 运行时、GPU 正式回归、原生 P2 完整后端/工作空间实验全套未在本轮重新运行。已运行的单元回归不能替代它们，也不证明物理标定或安全。
