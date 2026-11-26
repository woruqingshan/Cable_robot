# DEVELOPMENT_STATUS — 当前开发状态与文档索引

> 更新：2026-11-26｜仓库 `woruqingshan/Cable_robot`｜工作分支 `feat/real-operation-pipeline`。
> 已核对远端 HEAD：`e77fa3d1cc18f3ead5a6b94438054e8048271436`（2026-11-24）。此为记录的基线，不表示未来 HEAD 不变。研究目标见 [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md)，环境见 [DEVELOPMENT_ENVIRONMENT.md](DEVELOPMENT_ENVIRONMENT.md)。

## 1. 当前结论

**Stage 2：M1 已实测，M2/M3 已形成有边界的决策，M4 进入自身数据后训练准备。Stage 1：PASS_WITH_LIMITATIONS。** 已经通过真实观测→云端 GPU 推理→动作返回并记录的端到端实验，尚未通过 VLA 实际执行和反馈控制闭环。当前模型研究进入 SmolVLA–JetRover 状态与动作空间对齐。

```text
D405 真实 RGB + 六路舵机位置反馈 + Language
    → Research Observation → SSH / HTTP
    → AutoDL SmolVLA → Native Action [50,6]
    → JetRover 接收、校验、记录（已验证终点）
    ── 未启用执行 ──
    → Action Adapter → 任务/工作空间约束规划 → 真实控制 → 重新观测
```

| 模块 | 状态 | 证据边界 |
|---|---|---|
| 确定性机械臂运动 | PASS（历史） | 已审核 A→B→A；历史固定轨迹拔出10/10不是当前 VLA 成功率 |
| 六路连续反馈 | PASS（短时） | 约20 Hz 静态与动态读取；真实物理角度及内层事务关联未验证 |
| 故障恢复 | PASS_WITH_LIMITATIONS | 人工断电恢复通过；自动恢复 NOT_RUN，后来出现 READ_TIMEOUT |
| D405 | PASS | RGB/Depth/CameraInfo 已采集；深度单位、配准、外参尚未核实 |
| Real Observation | PASS_WITH_LIMITATIONS | 独立 Research Profile；正式可信输入校验未放宽 |
| SSH/HTTP + GPU 推理 | PASS | 52次真实实时请求成功，原生动作完整返回 |
| 模型状态/动作适配 | IN_PROGRESS（研究） | 名义本体契约已整理；训练物理语义及归一化待确认 |
| Action Adapter | IMPLEMENTED（合成路径） | 仅支持 synthetic_identity_v1；真实 native action 仍拒绝 |
| 工作空间研究 | RESEARCH BASELINE | 已有 URDF/FK、参数化场景、离散碰撞和规划；未认证实机安全 |
| 物理关节/夹爪、手眼、TCP 标定 | DEFERRED | 无物理精度证明 |
| VLA 真机执行、约束规划集成、RJ45 任务实验 | NOT_RUN | 后续目标，不将真实推理称为物理闭环 |

## 2. 关键实验记录

Stage 1 实验客户端与服务端运行源码 `95621edcb758e653d42473958374802d8ab326e1`；反馈为冻结包 `73aafa9ecac071206f17afb095eb938e5a56a670`。最终文档提交 e77fa3d 并不是全部运行组件的版本。

| 指标 | 真实实验结果（2026-11-24） |
|---|---|
| 首批 / 连续实验 | 10/10 + 42/42 成功；连续段30.250 s，约1.388 Hz |
| 动作与身份 | 52份完整 50×6 有限值，两端请求 ID 逐一对应，未执行 |
| HTTP RTT | Mean 437.732 ms；P50 405.011 ms；P95 598.850 ms；Max 1235.501 ms |
| GPU forward | Mean 284.362 ms（CUDA同步计时） |
| Observation 构建拒绝 | 4+12次（等待有效图像/状态或图像过期）；不是HTTP失败 |
| 连续采集 | RGB 28.01 Hz、Depth 28.37 Hz、舵机轮询20.00 Hz、状态发布9.78 Hz |
| 人工恢复后反馈 | 30.014 s，600组六路，3600次有效外层应答，19.9979 Hz |
| 软件回归 | 去重378 passed / 1 skipped；研究 Schema 11 passed；云端4项ASGI边界断言 |

早期 R1-F1 静态600组/动态403组，分别3600/3600、2418/2418外层有效回复，动态 A→B→A 已经通过，不能再沿用旧交接中“动态未运行”的结论。之后曾出现 ID10 Status=-1；Stage 1 最后一次记录 ID1 READ_TIMEOUT，约101 ms无回复，采集 PAUSED、反馈无效、运动权限关闭。故障根因 UNKNOWN，不能断言由模型、网络或相机导致。当前实际小车状态未重新核验。

数据来源：[Stage 1 实验报告](real_operation/experiments/stage1_e2e_20261124/experiment_report.md)、[阶段总结](real_operation/STAGE1_DEVELOPMENT_SUMMARY.md)、[R1-F1 实验报告](real_operation/experiments/r1_f1_joint_20261124/experiment_report.md)。RTT 包含传输和服务处理，不等于纯网络延迟；推理成功率仅指已上传有效请求。

### 已有工作空间研究

`workspace_analysis/` 已实现 URDF/FK、工作空间采样、参数化 IDC 场景、全臂离散碰撞检查、任务可达性、标准 RRT-Connect、多起点、shortcut 与 RViz 回放。历史 T5.3-C 主试验90次中80次接受，10次 TIME_LIMIT；补充 Local-only 10/10。90条接受路径的标准 shortcut 中位关节路径长度下降7.399%、TCP长度下降27.460%，但31条间隙下降，额外处理时间中位82.807 s。8次准备运动属于参考起点到 ready，不是物理 Home 验证。结果说明需要进一步研究可行位姿与在线规划效率，不证明新算法创新或真实碰撞安全。

证据：[T5.3-C 历史实验](../workspace_analysis/docs/t5_3_c_robustness_shortcut.md)。不要在本轮模型语义审计中重写工作空间模块。

## 3. 当前核心问题：模型语义与归一化

### JetRover 名义本体

`[joint1,joint2,joint3,joint4,joint5,r_joint]` 对应舵机 `[1,2,3,4,5,10]`。五个机械臂关节加一个夹爪，非六自由度串联臂。厂商 YAML 配置五臂关节 init=500，夹爪 init=700，均 min=1000/max=0；现有转换 `q=(init-pulse)*240π/(180*1000)`。名义 rad 不代表独立物理标定，夹爪 rad 不是开口距离。SO-100 的肩/肘/腕/夹爪功能顺序存在候选对应，不能据此假定源模型训练通道单位及运动学完全一致。

### SmolVLA 实际 checkpoint 审计（2026-11-26）

- 实际 config: `observation.state [6]`、`action [6]`、`chunk_size=50`、`n_obs_steps=1`，STATE/ACTION 声明 MEAN_STD，VISUAL IDENTITY。
- 两个已导出的统计文件均仅含 `so100-blue.buffer.action`、`so100-red.buffer.action`、`so100.buffer.action` 的 mean/std。缺 `observation.state` 和无前缀 `action` 统计键。
- 固定 LeRobot 版本 `ff0acf5...` 的 `normalize_processor.py` 中，缺少特征统计键时会直接返回原 Tensor。项目现有官方 processor 调用没有显式选择统计组的逻辑；因此存在高置信的静默跳过风险。尚未独立复现实际 processor 运行状态，也没有证明历史52次每次都跳过。不要将该风险当成已经修复或实测确认的缺陷。
- 所有原生动作仍按 `native_0..5`、`model_native_unknown`、`semantics=unknown` 保留；50步和历史20ms offsets 不是已验证的物理控制时序。

专项分析（用户提供，已纳入仓库）：

- [JetRover 本体分析](model_adaptation/01_JetRover_Body_State_Action_Analysis.md)
- [SmolVLA 适配分析](model_adaptation/02_SmolVLA_JetRover_Adaptation_Analysis.md)

两份文件基于实际用户上传的 URDF/Xacro、厂商 servo_controller.yaml、云端 checkpoint 配置/统计输出与源码审计。本轮必须阅读，原始证据按需核对。

## 4. 后续开发候选工作（等待正式 Instruction）

条件：AutoDL 无卡模式、约2 GB内存；JetRover 不在线；沿用 `feat/real-operation-pipeline`。最终具体执行范围由用户新 Instruction 决定。

1. SSH 登录 AutoDL，复用代理，同步分支，阅读上述三份入口和两份专项分析。核对模型/处理器实际版本与 checkpoint 文件，检查历史日志是否保存处理前后 Tensor。
2. 核验归一化：源码和 NumPy 轻量数值验证优先；内存允许才独立测试官方 processor，不加载完整模型。若不能运行，明确 NOT_RUN。不能任选一组 SO-100 统计或用动作统计填补状态统计。
3. 研究预训练数据、源机器人驱动与数据集 metadata，确定通道、单位、绝对/增量、零位、方向、夹爪、归一化及频率。若有可靠契约，设计显式 State/Action Adapter 并 CPU 离线测试；无证据则保留 UNKNOWN，不注册真实执行映射。
4. 复用历史真实 Observation/动作记录做离线接口和回归验证；历史动作只能用于研究检查，不能冒充正确控制标签。若直接迁移不可证实，准备 JetRover 数据采集和后训练方案，不提前训练。
5. 记录本轮实现、证据、测试和未完成项。保持原正式协议及执行保护；提交推送到指定分支。更新 STATUS 及相关专项文档。

**不做：** 完整模型推理、GPU租用、实机动作、串口/相机驱动修改、物理标定、重新设计通信、工作空间算法扩展、无证据的模型参数修改。不为了文档整理而重构源码或迁移部署目录。

## 5. 后续路线（不是本轮授权）

先解决训练语义与本体映射可行性；有 GPU 后使用已记录的真实 Observation 回放，核对实际 processors 与新旧推理输出。若 base 不能可靠迁移，采集 JetRover 自身的图像、真实状态、实际示教目标动作和时间，建立数据集并微调；state 与 action 分别记录。实机可用后验证故障恢复、关节/夹爪标定及可信时间关联；按任务需求完成手眼和 TCP；经过离线、仿真及受控执行验证，再建立真实反馈闭环并集成论文两项核心算法。未明确语义前禁止连接 VLA 输出到控制器。

## 6. 代码导航

| 路径（仓库根目录相对） | 职责 |
|---|---|
| `patches/jetrover_readback/` | 唯一串口、有界读回、持续反馈、故障隔离 |
| `cable_robot_control/` | 传统控制与经审核的运动执行接口 |
| `camera/ros2/d405_bringup/` | D405 launch 包装 |
| `cable_robot_vla/cable_robot_vla/hardware_feedback.py` / `robot_state_provider.py` | 反馈证据和名义关节转换 |
| `cable_robot_vla/cable_robot_vla/real_camera.py` / `real_observation_node.py` | 感知及 Observation 基础 |
| `vla_protocol/src/vla_protocol/research.py` / `action.py` | 独立研究观测与动作协议 |
| `vla_server/src/vla_server/observation_adapter.py` | 模型特征绑定，非训练语义证明 |
| `vla_server/src/vla_server/policies/smolvla.py` | 官方模型、pre/postprocess、native action |
| `cable_robot_vla/cable_robot_vla/policy_client.py` / `action_adapter.py` | HTTP 客户端、显式映射注册框架（真实动作未启用） |
| `deployment/local/stage1_inspection.py` / `replay_stage1_inspection.py` / `analyze_stage1_inspection.py` | 真实推理、明确回放、分析 |
| `workspace_analysis/` | 工作空间、可达性、碰撞和规划研究 |

## 7. 文档目录与维护约定

本节组织方案已完成入口与专项文档落库；仅归档两份早期 Git 快照，其余历史计划按用途索引并保留路径。2026-11-26 的远端树已核实：`docs/real_operation/` 含使用说明、设计、诊断、实验、证据；`workspace_analysis/docs/` 和 `deployment/` 含模块专属资料。先建立三份入口和专项目录，不必移动全部文件才开始开发。

```text
docs/
├── PROJECT_CONTEXT.md
├── DEVELOPMENT_ENVIRONMENT.md
├── DEVELOPMENT_STATUS.md
├── model_adaptation/
│   ├── 01_JetRover_Body_State_Action_Analysis.md
│   └── 02_SmolVLA_JetRover_Adaptation_Analysis.md
├── real_operation/
│   ├── STAGE1_USER_GUIDE.md
│   ├── STAGE1_DEVELOPMENT_SUMMARY.md
│   ├── [现有设计/验收/维护文档]
│   ├── evidence/              # 保留原路径和原始证据
│   └── experiments/           # 保留实验目录、图片、JSON、CSV
└── archive/                   # 可选：经核验已过时的历史计划/交接
workspace_analysis/docs/        # 模块自己的科研资料，不强制搬迁
deployment/cloud/              # 模块自己的部署资料
deployment/local/
```

### 当前资料导航（原路径）

| 用途 | 阅读入口 |
|---|---|
| 系统启动与数据格式 | [STAGE1_USER_GUIDE](real_operation/STAGE1_USER_GUIDE.md) |
| 最新真实实验总结 | [STAGE1_DEVELOPMENT_SUMMARY](real_operation/STAGE1_DEVELOPMENT_SUMMARY.md) |
| 验收与当前边界 | [R1_R2_ACCEPTANCE](real_operation/R1_R2_ACCEPTANCE.md)、[END_TO_END_INTEGRATION](real_operation/END_TO_END_INTEGRATION.md) |
| 早期动作空间审计 | [ACTION_CALIBRATION](real_operation/ACTION_CALIBRATION.md)（历史，最新专项分析优先） |
| 硬件故障与维护 | [JETROVER_R1_TECHNICAL_REFERENCES](real_operation/JETROVER_R1_TECHNICAL_REFERENCES.md)、[R1_SDK_MAINTENANCE_PLAN](real_operation/R1_SDK_MAINTENANCE_PLAN.md)、[USB_CDC_DIAGNOSIS_AND_RECOVERY](real_operation/USB_CDC_DIAGNOSIS_AND_RECOVERY.md) |
| 历史云部署与仿真 | [STAGE2](../deployment/cloud/STAGE2.md)、[STAGE23](../deployment/local/STAGE23.md)（这里的历史 Stage 编号不等于当前总体阶段） |
| 原始实验 | [Stage 1](real_operation/experiments/stage1_e2e_20261124/experiment_report.md)、[R1-F1](real_operation/experiments/r1_f1_joint_20261124/experiment_report.md) |

### 整理规则

- 入口文件仅保留3份，避免新的总览、handoff、progress 在根目录反复堆叠。专项结论更新原文或增加日期清楚的版本；历史实验不覆盖。
- 先 `git ls-files docs workspace_analysis/docs deployment`，按“有效操作/设计、专项分析、实验与原始证据、已被取代的历史计划”分类。对于 `R1_*_PLAN`、`R1_*_PROGRESS`、`git_status_before_*` 等先判断是否仍被现行文档引用；只有确认过时的才考虑归档。`R1_SDK_MAINTENANCE_PLAN.md` 等仍有效的操作文档不能因名称含 PLAN 就归档。
- 如用户批准实际迁移，单独使用 `git mv` 做文档整理提交，更新所有 Markdown 相对链接、README、代码/脚本里的文件引用及索引；检查链接并保留必要兼容入口。不要删除旧证据或改写已提交实验数据/版本标记；已发布的固定 SHA 链接保持有效。若路径影响运行或数据追溯，先保留原路径，仅索引分类。
- 专项分析文件统一保留上面带 01/02 的原始文件名，避免其内部交叉链接失效；如采用用户截图中的无编号命名，需同时改相对链接和索引，不留重复副本。
- 敏感原始配置、个人任务书、完整模型权重和大型实验原始文件不默认入库；仅提交必要的脱敏材料、分析结果与可追溯引用。
- 每轮结束更新本文件的基线、状态、指标、问题和下一步；环境变化更新 ENVIRONMENT，研究范围变化更新 CONTEXT。新聊天先读三份入口，再按任务读取对应专项资料，不需要遍历全部历史文档。

## 8. 2026-11-26 上下文对齐与目录审计

本轮仅文档整理；第4节及专项文档中的开发步骤是后续候选任务，不构成本轮开发授权。未加载模型、运行推理、连接机器人或修改算法。

同步前服务器为干净 detached HEAD `95621ed`；经既有 Mihomo 成功 fetch，检出同名跟踪分支 `feat/real-operation-pipeline`，同步基线 `e77fa3d1cc18f3ead5a6b94438054e8048271436`。本地此前无该分支，本次只建立该原远端分支的本地跟踪，不另起功能分支。

### 分类与保留决定

|类别|路径与决定|
|---|---|
|长期研究/开发入口|本目录三份入口；architecture.md、vla_integration.md、jetrover_vendor_api.md 保留|
|专项技术分析|model_adaptation/ 两份编号分析；real_operation/ACTION_CALIBRATION.md 为早期审计，最新专项结论优先|
|运行维护与接口设计|real_operation/ 的 USER_GUIDE、CAMERA_AND_JOINT_STATE、SERVO_READBACK_DESIGN、HARDWARE_INTERFACE_AUDIT、技术参考、USB诊断及 R1_SDK_MAINTENANCE_PLAN 保留原路径|
|阶段总结与历史验收|STAGE1_DEVELOPMENT_SUMMARY、R1_R2_ACCEPTANCE、END_TO_END_INTEGRATION、TEST_RESULTS、PHASE2、CONTINUOUS_FEEDBACK_EXPERIMENT 与根目录 stage15*/vla_stage1_verification 保留；旧状态以最新 Stage 1 证据为准|
|历史计划与执行过程|R1_OWNER_STAGE_A_PLAN、R1_D2_D4_EXECUTION_PLAN、R1_D2_D4_PROGRESS、R1_*RESULT*、CONTINUOUS_MOTION_REVIEW 保留；包含已执行结果、安全边界或被现行文档引用，避免仅按文件名移动|
|历史 Git 快照|两份 git_status_before_* 经 git mv 迁至 archive/git_snapshots/；内容原样保留|
|实验与原始证据|real_operation/evidence/、experiments/ 全部原样保留|
|模块资料|workspace_analysis/docs/、deployment/cloud/、deployment/local/ 保持原路径|

归档快照：[轨迹工作前](archive/git_snapshots/git_status_before_insert_pullout.md)、[topic读回前](archive/git_snapshots/git_status_before_topic_readback.md)。

当前阻塞仍为反馈故障恢复、物理标定/时间关联、模型训练契约及归一化有效性。运行 processor 探针、源数据研究和映射开发均等待正式 Instruction。

核验补充：以标准库只读解析服务器两个 processor safetensors 文件头，均仅有三组 SO-100 action mean/std，确认缺 observation.state 与无前缀 action 键；没有运行 processor，实际转换行为仍 NOT_RUN。新增五份文档33个本地链接目标均存在；两份归档快照无旧完整路径引用。git diff --check 用于提交前格式检查；本次未运行代码测试。

## 9. Stage 2 本轮结果（2026-11-26）

工作基线 `abf447e8198f1ae2576049a177027f4e94f43604`，沿用原分支。本节是最新进展，第4节与专项分析第7节的待执行清单作为历史候选保留，后续优先按本节推进。

- M1 COMPLETE：固定官方 normalization step 和完整 pre/postprocessor（CPU device override）运行成功，未加载模型。state零/非零/历史输入、action零/非零及完整链路检查均原样返回；两个隔离正控制变换断言通过。10项结构化检查。三组统计被加载但不匹配标准特征键，实际未使用。峰值RSS约744MiB。
- 受管checkpoint哈希与原receipt完全相同，inventory `340a52a2b92f41fc9b8410bed0cd93743543bdb276e36125a74b73e0446add28`；原server配置只override device，无dataset_stats。52条历史diagnostics缺中间Tensor，不能逐次实证旧变换。Stage 1链路结论不变，动作物理语义仍unknown。
- M2 COMPLETE_WITH_UNKNOWNS：固定模型卡、论文、参考数据v2.1/v3.0 metadata、固定SO驱动与迁移issue完成一轮定向调查。六维参考顺序与30FPS确认；source训练单位、零位、方向、完整state统计、三组选组与精确动作时序未恢复。停止扩大溯源，不下载旧权重或大数据集。
- M3 COMPLETE：功能对应可行；sign/offset、夹爪映射需实测，运动学与腕部视角差异不由单位转换解决。直接映射未达到开发条件，复杂IK重定向本轮不开发。
- M4 ROUTE_B_OFFLINE_READY：主推荐JetRover数据后训练，保留直接映射为有条件的研究基线。已实现独立六维state/action staging契约、流式训练集统计与校验器；6项单元测试含多种拒绝子例通过。严格区分真实反馈与实际下发目标；合成数据不计作真实示教。LeRobot转换器/视频解码检查、真实数据采集、训练与闭环NOT_RUN。

实现：[processor探针](../deployment/cloud/scripts/probe_processors.py)、[Dataset校验器](../deployment/local/validate_adaptation_dataset.py)、[测试](../deployment/local/tests/test_adaptation_dataset.py)。证据：[数值](model_adaptation/results/stage2_20261126/processor_probe.json)、[运行配置/receipt](model_adaptation/results/stage2_20261126/runtime_audit.json)、[数据metadata](model_adaptation/results/stage2_20261126/source_metadata.json)、[离线测试](model_adaptation/results/stage2_20261126/dataset_tests.txt)。

路线论证、训练契约表、数据格式及采集/转换/pilot/正式训练方案统一维护于[专项分析第9–10节](model_adaptation/02_SmolVLA_JetRover_Adaptation_Analysis.md)。没有修改推理部署、官方统计、驱动、协议、工作空间算法或执行保护。

### 下一轮

先复用本轮结果，不重做Stage 1。实机可用后先验证人工恢复边界与反馈稳定性、逐关节/夹爪标定、原始时间关联；冻结数据契约、采样周期和容差，再受监督示教。GPU可用后先完成LeRobot转换与视频round-trip、独立split/统计核对、实际processor归一化正控制，然后小规模pilot训练与冻结测试。依学习曲线扩展数据；真实执行另经受控验证。不得用旧native action充当正确目标标签。
