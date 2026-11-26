# DEVELOPMENT_ENVIRONMENT — 开发与部署环境入口

> 更新：2026-11-26｜历史可复现配置来自 Stage 1（2026-11-24）使用说明与实验记录。IP、端口、进程、磁盘路径和硬件状态均需在使用时轻量核验。
> 当前开发限制见 [DEVELOPMENT_STATUS.md](DEVELOPMENT_STATUS.md)。本文件不含密码、Token、代理订阅或私钥。

## 1. Git 仓库和工作方式

- Repository: `https://github.com/woruqingshan/Cable_robot`
- 当前指定工作分支：`feat/real-operation-pipeline`（用户要求沿用，不另建分支）。
- Stage 1 稳定证据锚点：`e77fa3d1cc18f3ead5a6b94438054e8048271436`。2026-11-26 生成本文件时远端 HEAD 为此提交；以之后实际 `git fetch` 为准。
- 本地开发机/Codex 可远程操作 AutoDL；AutoDL 运行模型相关环境；JetRover 运行 ROS2/真实传感器和控制。两端共享 Git 仓库与协议，运行依赖分离。
- 先检查工作区，不覆盖未提交文件；同步后再开发。只提交本轮相关文件；推送需符合用户授权。不执行 force push、reset --hard、clean -fd 等破坏性操作。

```bash
# 进入实际项目目录后执行
git status --short --branch
git remote -v
git fetch origin feat/real-operation-pipeline
# 如本地已有分支，确认无冲突后：
git switch feat/real-operation-pipeline
git merge --ff-only origin/feat/real-operation-pipeline
# 如本地不存在该分支：git switch --track origin/feat/real-operation-pipeline
git rev-parse HEAD
```

如果快进合并失败、工作区不干净或当前远端身份异常，先核对差异，不强行覆盖。上述 Git 命令是本轮建议操作流程，不代表本文件生成时已经执行了远端同步。

## 2. AutoDL 云端

```bash
ssh -p 46118 root@connect.bjb1.seetacloud.com
cd /root/autodl-tmp/idc-vla/Cable_robot
```

| 项目 | 已知配置 |
|---|---|
| 根目录 | `/root/autodl-tmp/idc-vla` |
| 代码 | `/root/autodl-tmp/idc-vla/Cable_robot` |
| 模型环境 | `/root/autodl-tmp/idc-vla/venvs/vla-model/bin/python` |
| 历史运行版本 | Python 3.12 / PyTorch 2.8.0+cu128 |
| 历史 GPU | RTX 4080，Stage 1 实验记录；当前无卡模式，用户报告约2 GB内存 |
| Stage 1 云端记录 | `/root/autodl-tmp/idc-vla/runs/stage1_20261124` |
| 推理绑定 | `127.0.0.1:8000`，不可直接暴露公网 |

当前不加载完整模型，不执行 GPU/CPU 完整 forward，不升级依赖。核查模型配置、统计和源码可在无卡模式完成；官方 processor 导入也可能超过内存限制，不能保证可运行。先用 `free -h`、`df -h` 和实际环境核对资源，不自动扩容、租卡或重启实例。

## 3. 网络代理与 GitHub 连通性

Stage 1 已验证的 mihomo 启动路径：`/root/.local/bin/mihomo -d /root/.config/idc-vla-mihomo`。已有环境脚本：`/root/autodl-tmp/idc-vla/runs/bootstrap/use_proxy.sh`。

建议流程（先验证文件和进程，避免重复启动）：

```bash
pgrep -x mihomo || true
ss -ltnp
ls -l /root/.local/bin/mihomo
ls -ld /root/.config/idc-vla-mihomo
ls -l /root/autodl-tmp/idc-vla/runs/bootstrap/use_proxy.sh
```

若确认历史代理未运行、配置目录与可执行文件仍有效，可按历史方式启动（在单独终端运行，或使用已有受管启动方式）：

```bash
/root/.local/bin/mihomo -d /root/.config/idc-vla-mihomo
```

在需要访问 GitHub/Hugging Face 的 shell 中加载代理环境：

```bash
source /root/autodl-tmp/idc-vla/runs/bootstrap/use_proxy.sh
curl -I --max-time 15 https://github.com
git ls-remote --heads origin feat/real-operation-pipeline
```

仅在对应路径真实存在且端口与环境匹配时，才考虑用户另有的 `~/bin/mihomo -d ~/.config/mihomo`；`source /etc/network_turbo` 是历史备选，未作为 Stage 1 验收路径。代理端口以真实监听及脚本为准，不硬编码猜测。Git 若使用 SSH remote，HTTP 代理环境变量未必生效，按实际 remote/认证方式处理。不得打印包含订阅 URL、密钥或认证信息的配置，也不得为了连通性关闭 TLS/SSH 主机身份验证。GitHub 凭证由用户提供或使用已配置的安全认证。

## 4. SmolVLA checkpoint 与模型环境

| 对象 | 已知记录 |
|---|---|
| checkpoint | `lerobot/smolvla_base` |
| checkpoint revision | `5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6` |
| 本地目录 | `/root/autodl-tmp/idc-vla/models/smolvla_base` |
| LeRobot revision | `ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2` |
| VLM backbone | `HuggingFaceTB/SmolVLM2-500M-Video-Instruct` |
| 历史 backbone revision | `4cde9d208e19519c36917aa170fffc7e381aa0ad`（历史记录，使用时核验实际 receipt/cache） |

本模型已经下载。用户 2026-11-26 提供的目录输出显示 `model.safetensors` 约865 MiB，另有 `config.json`、`policy_preprocessor.json`、`policy_postprocessor.json`、两个640字节统计文件及 README。不要重新下载整个模型或把权重提交 GitHub。

```bash
cd /root/autodl-tmp/idc-vla/models/smolvla_base
ls -lh
# 只读查看配置，不加载权重
cat config.json policy_preprocessor.json policy_postprocessor.json
/root/autodl-tmp/idc-vla/venvs/vla-model/bin/python - <<'PY'
from pathlib import Path
from safetensors.numpy import load_file
for p in sorted(Path('.').glob('policy_*_processor.safetensors')):
    print(p.name)
    for k,v in load_file(str(p)).items():
        print(k, v.shape, v.tolist())
PY
```

当前归一化审计发现见 STATUS 与专项分析。历史启动使用固定 checkout、checkpoint inventory/receipt 与官方 processors；实际 LeRobot checkout 和模型缓存路径从当前 server.yaml 与部署文件核对，不猜测目录或改写原始权重/统计文件。

### 未来有 GPU 时的历史服务入口（本轮不启动）

```bash
cd /root/autodl-tmp/idc-vla/Cable_robot
export PYTHONPATH="$PWD:$PWD/vla_protocol/src:$PWD/vla_server/src:$PWD/cable_robot_vla"
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 \
  ../venvs/vla-model/bin/python -m vla_server.app \
  --config ../runs/stage1_20261124/server.yaml
```

上述为 Stage 1 实测使用方式。启动前核对源码、配置和固定模型版本；确认8000端口没有重复服务。

```bash
curl --noproxy '*' http://127.0.0.1:8000/health
nvidia-smi
```

历史日志 `../runs/stage1_20261124/server.v2.log`，逐请求 diagnostics 位于同一 run 下。当前无卡模式不要运行 GPU 服务。

## 5. JetRover 实机环境

```bash
ssh ubuntu@100.70.140.51
cd /home/ubuntu/zsy_projects/Cable_robot
```

| 项目 | 已知配置 |
|---|---|
| 平台 | Jetson Orin NX，Ubuntu 22.04，ROS2 Humble |
| 厂商工作空间 | `/home/ubuntu/ros2_ws` |
| 项目路径 | `/home/ubuntu/zsy_projects/Cable_robot` |
| 相机 | Intel RealSense D405，librealsense 2.55.1 / ROS wrapper 4.55.1 / FW 5.15.1.55 |
| 机械臂 | 五关节 ID1–5，夹爪 ID10；ID9 不参与本轮 |
| 舵机配置 | `/home/ubuntu/ros2_ws/src/driver/servo_controller/config/servo_controller.yaml` |
| RRC | STM32，历史 `/dev/rrc` → USB CDC ACM；VID:PID 1a86:55d4，序列号 5A25000846 |
| 反馈 | 底层约20 Hz，状态发布约10 Hz；物理采样时刻未验证 |
| 原始实验目录 | `/home/ubuntu/zsy_projects/real_operation_evidence/stage1_20261124T1834` |

本轮小车不在线。以下命令是未来实际启动参考，而不是要求现在执行：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
# D405 已构建 overlay（如目录仍有效）
source /home/ubuntu/zsy_projects/stage1_camera_install/setup.bash
ros2 launch d405_bringup d405.launch.py
```

相机 launch 复用现有驱动；如 `idc-stage1-d405.service` 已运行，不重复启动。完整构建步骤见 Stage 1 User Guide。

| 数据 | 已知 ROS2 Topic / 格式 |
|---|---|
| RGB | `/d405/wrist/color/image_rect_raw`，rgb8，848×480，30 FPS配置 |
| Depth | `/d405/wrist/depth/image_rect_raw`，16UC1，848×480；深度单位尚未确认 |
| CameraInfo | `/d405/wrist/color/camera_info`、`/d405/wrist/depth/camera_info` |
| 反馈诊断 | `/idc_vla/real/feedback/status`；检查 valid、gate.error、motion permission |

### 串口独占与恢复边界

实验实际运行冻结包 `73aafa9`：`/home/ubuntu/zsy_projects/real_operation_deployments/r1_f1_73aafa9ecac071206f17afb095eb938e5a56a670/run_feedback.py`。

原厂 ROS 驱动与研究反馈程序不能同时占用 RRC。历史 `/run` 中的 Restart=no 为临时维护保护，重启即失效；原厂启动重新运行不等于代码丢失。必须核验设备身份、实际串口所有者、PID 与运动安全后按授权执行维护流程。不得复制历史 PID、擅自重启 bringup、清锁、自动重试未知事务、刷写 STM32 或修改厂商系统依赖。`DEVICE_ERROR` 与 `READ_TIMEOUT` 均有真实记录；人工断电恢复曾通过，自动恢复没有验证。重新连接设备时不能假定反馈有效或运动已授权。

启动/恢复细节以 [STAGE1_USER_GUIDE.md](real_operation/STAGE1_USER_GUIDE.md) 与 [R1_SDK_MAINTENANCE_PLAN.md](real_operation/R1_SDK_MAINTENANCE_PLAN.md) 为准。

## 6. 端云通信与真实推理

历史结构：JetRover `127.0.0.1:18000` → SSH 隧道 → AutoDL `127.0.0.1:8000`。

未来在 JetRover 建立隧道（先确认已核验的 SSH host key 和没有同端口旧隧道）：

```bash
ssh -fN -o StrictHostKeyChecking=yes -o ExitOnForwardFailure=yes \
  -p 46118 -L 127.0.0.1:18000:127.0.0.1:8000 \
  root@connect.bjb1.seetacloud.com
curl --noproxy '*' http://127.0.0.1:18000/health
```

`/v1/infer` 是原正式路径；`/v1/inspect` 是 Stage 1 opt-in 研究检查路径，允许独立 `real_research_inspection_v1` profile，不授予执行权限。历史实测启动顺序为云端模型服务→SSH 隧道→相机→只读反馈→`deployment/local/stage1_inspection.py`。本轮仅离线开发，不建立实机隧道、不启用模型动作执行。协议、数据格式、实际启动命令和预期输出详见 [STAGE1_USER_GUIDE.md](real_operation/STAGE1_USER_GUIDE.md)。

## 7. 维护约定

- 环境变更时更新本文件，阶段性实验结果放入 STATUS 或实验报告。不要将历史配置自动宣称为当前实况。
- 认证信息只通过用户批准的安全方式提供；不要在终端输出、代码、Git 提交或文档中记录秘密。
- 环境入口文件可以包含必要连接地址和路径，但推送到共享/公开仓库前应核对访问权限及敏感信息；若需要对外发布，另做脱敏版本。
- 专项参考：[Stage 1 使用说明](real_operation/STAGE1_USER_GUIDE.md)、[云端部署说明](../deployment/cloud/README.md)、[历史 GPU 部署](../deployment/cloud/STAGE2.md)、[历史本地仿真联调](../deployment/local/STAGE23.md)。

## 8. 2026-11-26 只读环境核验

- Ubuntu 22.04.5；容器 cgroup memory.max=2147483648（2 GiB），free 显示宿主机内存，不作为容器额度。
- 数据盘约20G/50G，剩余约31G；登录横幅未发现 GPU，本轮未启动推理服务。
- 既有 Mihomo 原先未运行，已按第3节路径启动并加载既有代理脚本；GitHub fetch 成功。
- Python 3.12.3；通过 importlib.metadata（未导入模型）确认 torch 2.8.0+cu128、lerobot 0.7.1、transformers 5.22.2。
- 独立 LeRobot checkout HEAD 确认为 ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2。
- 项目同步基线 e77fa3d；本轮为文档提交，不改变 Stage 1 实际运行源码版本。
