# SmolVLA → JetRover 状态与动作适配分析

> 2026-11-26｜基于 AutoDL 实际 checkpoint 文件输出、JetRover 本体资料与固定版本 LeRobot 源码｜不执行实机动作

## 1. 核心结论

- JetRover 与 SO-100 均可按五个臂关节和一个夹爪表示，关节功能顺序基本一致，直接关节空间适配值得优先研究。
- AutoDL 已有 `lerobot/smolvla_base` 权重及处理器文件，不需要重新下载整个模型。
- **最高优先级问题不是 rad→degree，而是归一化统计缺键及错误的特征绑定。** 当前文件仅保存 SO-100 三组动作统计，缺少 `observation.state` 统计，且没有处理器查找的 `action` 键；固定版本代码会静默跳过缺键转换。
- 不得通过重命名任意一组统计、使用动作统计代替状态统计、修改 robot_id 或把模型输出直接乘常数来声称完成适配。
- 建议先对实际运行处理器做离线数值验证，然后判断能否恢复可追溯的 SO-100 训练契约。若不能，应采用 JetRover 示教数据微调，建立自己的状态、动作与统计参数。

## 2. 证据与审计边界

本次读取用户上传的 `粘贴的 Markdown (1)(4).md`（AutoDL config、pre/postprocessor、README、两个 safetensors 的实际打印结果）、`servo_controller.yaml`、`jetrover_description.zip`；参照项目已有 `03_Action_Space_and_Robot_Adaptation(1).md` 与 Stage 1 代码/实验记录。

历史部署：checkpoint `lerobot/smolvla_base`，revision `5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6`；LeRobot `ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2`；实验客户端和云端源码 `95621ed`，总结检查点 `e77fa3d`。本次没有直接登录 AutoDL 或运行 GPU，只审计导出内容和对应源码；尚未复核当前实际权重/receipt 哈希、处理器实例运行状态及旧52次推理实际 Tensor。因此，归一化跳过属于有明确静态代码依据的高置信判断，仍须在服务器复现后才能认定历史实验每次都发生了该行为。

## 3. 实际 checkpoint 的模型契约

| 项目 | 实际配置 | 解释 |
|---|---|---|
| type | smolvla | 官方基础策略 |
| observation.state | [6] | 六个输入状态数值，配置未附通道物理名称 |
| action | [6] | 每个动作步六维，配置未定义物理单位 |
| n_obs_steps | 1 | 单次观测步 |
| chunk_size / n_action_steps | 50 / 50 | 原生预测50步，不代表实机必须一次执行50步 |
| max_state_dim / max_action_dim | 32 / 32 | 模型内部最大维度，不表示机器人32自由度 |
| camera1/2/3 | [3,256,256] | 三个声明的视觉特征；Stage 1 只实际绑定一台D405 |
| resize_imgs_with_padding | [512,512] | 模型官方图像处理配置 |
| VISUAL / STATE / ACTION | IDENTITY / MEAN_STD / MEAN_STD | 声明的归一化模式，是否真正生效需要验证 |
| adapt_to_pi_aloha | false | 未开启 PI-ALOHA 专用转换 |
| use_delta_joint_actions_aloha | false | 未开启 ALOHA 专用 delta 选项，不能据此证明全部训练动作均是绝对角度 |
| num_steps | 10 | 模型生成过程参数，不是舵机控制步数 |
| min_period/max_period | 0.004 / 4.0 | 模型内部配置，不是物理执行周期或控制频率的证据 |
| backbone | HuggingFaceTB/SmolVLM2-500M-Video-Instruct | 视觉语言骨干 |
| freeze_vision_encoder / train_expert_only / train_state_proj | true / true / true | 当前 checkpoint 记录的训练配置，可作为微调起点，实际冻结参数需复核 |

模型卡明确定位为面向目标用例进一步微调的基础模型，不能将其当作已经训练好 RJ45 插拔的 JetRover 专用策略。`50×6` 只证明输出结构，不能证明六通道已经是 JetRover 可执行目标。

## 4. 归一化统计：本轮最重要的发现

两个 safetensors 文件的打印内容完全一致，各包含6个长度为6的 Tensor，分别为以下三组 `*.buffer.action.mean/std`。下表保留三位小数以便阅读，精确值以服务器文件为准。

| 统计组 | action mean（通道0–5） | action std（通道0–5） |
|---|---|---|
| so100-blue | [1.421,125.721,125.419,63.783,-106.540,3.348] | [14.082,39.431,18.238,23.490,38.111,5.048] |
| so100-red | [2.439,124.938,123.558,66.993,-103.877,2.891] | [14.299,42.772,18.876,21.652,33.534,4.478] |
| so100 | [1.596,119.944,109.770,56.706,-27.423,12.003] | [26.392,52.411,49.854,36.998,59.360,19.040] |

### 4.1 文件直接证明了什么？

这些统计键明确关联 SO-100 命名的三个组，支持 SO-100 本体相关的预训练证据，但不能证明所有预训练数据只包含这些机器人或通道统计就是某一台实际机器的标定参数。数值尺度不像直接采用普通 rad；然而单凭均值不能确定角度单位、零位偏置、旧版电机归一化或具体控制语义。

- 两个文件均没有 `observation.state.mean/std`。
- 两个文件均没有无前缀的 `action.mean/std`。
- 两个文件均只有 action 统计，不能视为同一组同时适用于 state。
- blue、red、so100 的统计分布明显不同。例如第4通道的均值分别约 -106.54、-103.88、-27.42；没有依据任意挑选其中之一。
- preprocessor JSON 声明的视觉键为 `observation.image`、`observation.image2`、`observation.image3`，config 使用 `observation.images.camera1/2/3`，rename_map 为空。该差异需核对实际绑定；由于 VISUAL 为 IDENTITY，不能据此单独判定图像处理错误。Stage 1 已有单相机成功 forward 证据，尚无训练视角匹配证明。

### 4.2 为什么官方处理器可能没有真正归一化？

已直接核对项目固定 LeRobot revision 的 [`normalize_processor.py`](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/processor/normalize_processor.py)：

- L227–244：`load_state_dict` 使用 `flat_key.rsplit('.',1)`，因此 `so100.buffer.action.mean` 被加载到 `_tensor_stats['so100.buffer.action']['mean']`。
- L269–288：观察归一化按 feature 名称查找，例如 `observation.state`。
- L290–303：动作归一化/反归一化固定传入 `ACTION`（标准动作键）。
- L332–334：如果 mode 为 IDENTITY **或者统计键不存在**，直接返回 Tensor。
- L352–365：仅当正确统计存在时才执行均值/标准差转换。

因此在项目使用默认 checkpoint processor、没有其他 stats override 或重新绑定的前提下：

```text
state 输入：observation.state → 查不到该统计键 → 原样通过
动作输出：action → 查不到该统计键 → 原样通过
```

现有 `FeatureObservationAdapter` 把六路名义 rad 作为 `observation.state`，`SmolVLAPolicyAdapter.start()` 只传入设备 override，`predict()` 调用官方 preprocess → predict_action_chunk → postprocess；没有显式的本体统计绑定。固定版 `prepare_observation_for_inference` 把 robot_type 作为字段保存，但上述统计查找本身并不据此选择 SO-100 组。单纯把 robot_type 改成 so100 既不保证修复，也会错误标注 JetRover 来源。

上游已公开讨论相同类型的迁移问题：[LeRobot issue #4415](https://github.com/huggingface/lerobot/issues/4415)、[#2374](https://github.com/huggingface/lerobot/issues/2374)。这是对风险的旁证，不代替本版本实际运行验证。

### 4.3 对 Stage 1 的影响

此前52次推理全部成功、动作有限且 shape 正确，仍然证明真实端到端数据链路已通。但调用 postprocessor 不等于完成实际反归一化；此前输出很可能是未反归一化的模型输出，而非 SO-100 物理动作。不能据此评估动作幅度、关节目标合理性或 RJ45 任务能力。历史数据不删除、不重标物理单位；如后续离线重跑，应记录新的处理器身份并与旧实验分开保存。

## 5. JetRover 与 SO-100 的本体对应

SO-100 参考通道来源：[官方 pickplace 数据集 meta/info.json](https://huggingface.co/datasets/lerobot/svla_so100_pickplace/blob/main/meta/info.json)、固定版本 [SO follower 驱动](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/robots/so_follower/so_follower.py)。这套顺序是有依据的**候选对应**，不是当前 base checkpoint 六通道的完整训练契约。

| 模型候选顺序 | SO-100 | JetRover | ID | 对齐情况 |
|---:|---|---|---:|---|
| 0 | shoulder_pan | joint1 | 1 | 底座旋转功能对应 |
| 1 | shoulder_lift | joint2 | 2 | 肩部俯仰功能对应 |
| 2 | elbow_flex | joint3 | 3 | 肘部俯仰功能对应 |
| 3 | wrist_flex | joint4 | 4 | 腕部俯仰功能对应 |
| 4 | wrist_roll | joint5 | 5 | 腕部旋转功能对应 |
| 5 | gripper | r_joint | 10 | 夹爪功能对应，开口关系待定 |

固定版本 SO follower 驱动使用 Present_Position 读取状态、Goal_Position 写目标，臂关节根据 use_degrees 选择 DEGREES 或 RANGE_M100_100，夹爪为 RANGE_0_100。它支持绝对位置目标，但不能据此直接认定历史多数据集 base checkpoint 的所有动作标签都采用此版本定义；训练数据集和采集版本需要核对，尤其旧版关节 frame 可能不同。

两者关节功能相同不意味着相同角度产生相同 TCP；连杆长度、旋转轴局部方向、机械安装零位、夹爪结构及相机视角可能不同。具体本体数值见配套 `01_JetRover_Body_State_Action_Analysis.md`。

## 6. 建议的适配架构

```mermaid
flowchart LR
 P[JetRover raw pulse] --> Q[本体名义或已标定状态]
 Q --> O[标准 Observation / SSH HTTP]
 O --> I[云端 State Adapter]
 I --> N[匹配训练契约的官方 preprocessor]
 N --> V[SmolVLA]
 V --> U[匹配训练契约的官方 postprocessor]
 U --> A[原生动作返回 / 保留证据]
 A --> R[本地 Action Adapter]
 R --> W[关节目标 / 夹爪目标]
 W --> X[未来约束规划与执行]
```

本地上传保持机器人自身名义/标定单位和真实来源，不在串口驱动内做模型统计归一化。模型相关单位与关节映射放云端 State Adapter；官方 pre/postprocessor 负责统计归一化，不重复手工 scaler。输出适配留在本地，未来与规划模块连接，当前执行保持关闭。

### 6.1 若能够取得可验证的源机器人契约：直接映射作为基线

设 JetRover 五关节为 q_J（rad），若已证实 SO-100 源状态是 degree，则候选变换为：

```text
s_SO[i] = sign[i] * (180/pi) * q_J[i] + offset_deg[i]
a_J[i] = sign[i] * (pi/180) * (a_SO[i] - offset_deg[i])
```

仅对已证明相同对应关系且为绝对角度的通道成立；sign/offset 必须有实际定义或标定依据，不预设为 +1/0。若源动作是增量、速度或末端位姿，必须重新定义映射，不能套用绝对位置逆式。夹爪单独使用经验证的单调映射，不能直接把 r_joint 的 rad 当 0–100 百分比。

模型统计处理（已知且匹配对应训练组时）：

```text
z_state = (s_SO - mean_state) / (std_state + eps)
a_SO = z_action * std_action + mean_action
```

**state 与 action 的统计应分别取得**，且正确绑定 `observation.state` 与 `action`。不能以动作统计冒充状态统计，也不能仅靠 round-trip 数学正确就宣称物理语义正确。输出应经过离线限位/轨迹检查；越界要报告，不可静默裁剪后当作模型适配成功。即使数值完全一致也仍需验证迁移能力。

### 6.2 如果 base 训练契约无法完整恢复：推荐 JetRover 后训练

官方基础模型本身面向微调。以 JetRover 的统一六维状态、目标动作、D405 图像与语言指令建立 LeRobot Dataset，用同一数据契约计算独立的 state/action 统计，官方训练流程中验证统计的正确加载并输出完整新 checkpoint。未来推理时使用该 checkpoint 绑定的 processor，而不是沿用上述三组 SO-100 动作统计。

建议先用少量受控示教做数据采集与训练流程 pilot，正式数据量根据任务覆盖和学习曲线决定。记录实际反馈与控制器下发目标的区别、时间关联、任务和 episode；训练/验证/测试按完整 episode 或场景划分。当前不具备可信反馈、物理标定和实机执行准入时，不开始自主运动或将不可信观测当作正式标签。

这种方案避免把跨本体运动重定向变成新的科研负担；模型的本体适配属于基础设施，论文重点仍是任务条件末端位姿与受限轨迹修正。关节功能对应可保留为零样本迁移基线，但不是必须完成的硬件实验。

## 7. 下一步：只做一次模型处理器审计

下一轮交给 Codex 的目标为**只读核对实际云端 processor 的有效统计和实际前后变换，然后提交一份输入输出契约及直接映射可行性结论**。不立即重构或训练。

### 7.1 云端具体核查

在既有 Python 环境及固定 LeRobot checkout 下复用实际 `server.yaml`、模型 config、`make_pre_post_processors` 调用方式：

1. 核对实际 checkpoint revision/receipt、运行源码 revision、官方模型和 processor 文件哈希；确保未加载其他模型或覆盖统计。不要重新下载权重或升级依赖。
2. 逐个输出 processor step 的类型、features、norm_map 及 `_tensor_stats.keys()`；明确 `observation.state` 和 `action` 是否实际存在，记录统计来源与 override。
3. 使用离线探针记录 preprocessor 前后六维 state，以及 postprocessor 输入输出。例如对 action 使用 `[1,50,6]` 的全零和非零 Tensor，比较是否原样返回；真正的 mean/std 反归一化应将零向量转换成所绑定的 mean。测试不需要机械臂，也不必重新启动 GPU 服务。不能只看 JSON 配置宣称通过。
4. 查询 checkpoint 的训练历史、旧迁移版本及 SO-100 数据集/采集代码，找到 state 与 action 分别对应的通道、单位、零位、动作类型、统计组和 fps。对旧版或不同数据集的定义保持版本区分。若无法取到可靠证据，标记 UNKNOWN，不自行补全。
5. 如能恢复确切匹配的统计，创建独立、版本化的离线研究配置，保留旧文件和日志；用相同已记录输入对比处理前后数值（明确 recorded_replay，不冒充 live），验证范围及语义。不得覆盖正式控制配置或启用动作执行。

### 7.2 验收结果应回答的三个问题

| 问题 | 当前答案 |
|---|---|
| JetRover 与 SO-100 关节功能能否对应？ | 能，候选顺序基本对应；方向/零位与物理精度待验证 |
| 现有 base checkpoint 是否具有可直接使用的完整 state/action 契约？ | 目前不能证明，统计键缺失/错配必须先核查 |
| 是否可以立即开发真实动作映射？ | 不可以；先完成离线处理器验证，若无法恢复契约则准备 JetRover 后训练 |

**本轮不做：** 真实 VLA 动作执行、底层通信重写、手眼/TCP 标定、复杂 IK 重定向、工作空间规划算法开发、盲目改统计或直接缩放输出。

## 8. 参考链接与源文件定位

- 本地证据：用户上传 `粘贴的 Markdown (1)(4).md`，模型 config（L13–205）、preprocessor（L205–379）、postprocessor（L379–443）、README（L443–739）、统计打印（L763–791）；`servo_controller.yaml` L2–63；`jetrover_description.zip`。
- [官方 SmolVLA checkpoint](https://huggingface.co/lerobot/smolvla_base)；[模型论文](https://arxiv.org/abs/2506.01844)；[官方微调说明](https://huggingface.co/docs/lerobot/main/smolvla)。
- [项目 Observation Adapter](https://github.com/woruqingshan/Cable_robot/blob/e77fa3d/vla_server/src/vla_server/observation_adapter.py)；[SmolVLA policy](https://github.com/woruqingshan/Cable_robot/blob/e77fa3d/vla_server/src/vla_server/policies/smolvla.py)；[Action Adapter](https://github.com/woruqingshan/Cable_robot/blob/e77fa3d/cable_robot_vla/cable_robot_vla/action_adapter.py)；[Stage 1 实验总结](https://github.com/woruqingshan/Cable_robot/blob/e77fa3d/docs/real_operation/STAGE1_DEVELOPMENT_SUMMARY.md)。
- [固定版本归一化源码](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/processor/normalize_processor.py)；[固定版本 SO follower](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/robots/so_follower/so_follower.py)；[SO-100 示例数据集](https://huggingface.co/datasets/lerobot/svla_so100_pickplace/blob/main/meta/info.json)。

## 9. Stage 2 实测与路线决定（2026-11-26，取代第7节的待执行状态）

### M1：实际官方 processor 已验证，不加载模型

结果：[processor_probe.json](results/stage2_20261126/processor_probe.json)、[身份与历史日志审计](results/stage2_20261126/runtime_audit.json)。当前 checkpoint 全部受管文件哈希匹配历史 receipt；排除 `.cache` 后 inventory 为 `340a52a2b92f41fc9b8410bed0cd93743543bdb276e36125a74b73e0446add28`，与 Stage 1 配置一致。receipt 是历史操作记录，不是独立 Hub 权重证明。

`deployment/cloud/scripts/probe_processors.py` 使用固定 LeRobot，直接实例化官方 normalization steps，并通过 config-only `make_pre_post_processors` 加载完整官方 pipeline。仅将 device override 为 CPU，复用离线 tokenizer cache；未实例化 SmolVLAPolicy、未 forward。峰值 RSS 761844 KiB（约744 MiB）。

|证据层级|结果|
|---|---|
|官方单独 normalization step|零/非零/历史六维 state 原样通过；零/0.5 的 `[1,50,6]` action 原样通过|
|完整官方 pipeline|历史 state 与零/非零 action 原样通过；合成 RGB/task 仅为触发 processor，非实机回放或模型实验|
|隔离正控制|state 的构造 mean=[1..6], std=2 将零转为 -mean/2；action 将零转为 mean，断言通过|
|静态调用链|Observation 按 joint1..5,r_joint 绑定名义 rad；服务只覆盖 device，不传 dataset_stats；空 rename_map 不会改统计键；robot_type 不参与 normalizer 分组|

三组统计实际被加载，但键为 `so100*.buffer.action`，没有匹配 `observation.state` / `action`，因此未被这些转换使用。文件声明 MEAN_STD 不等于执行了变换。没有修改任何 checkpoint 文件或统计，也没有采用 issue 中“选择首个匹配组”的建议。

历史 diagnostics 有52条完整记录，包含原生动作、观测摘要与运行时计时，没有 pre/post 中间 state/action Tensor。当前重建支持历史跳过的强推断，但无法逐请求直接核对；历史52次链路成功结论保持，动作继续 unknown。禁止重标为物理角度。

复现（从仓库根目录，输出必须是新路径）：

```bash
HF_HOME=/root/autodl-tmp/idc-vla/cache/huggingface \
../venvs/vla-model/bin/python deployment/cloud/scripts/probe_processors.py \
 --checkpoint ../models/smolvla_base --lerobot ../third_party/lerobot \
 --observation docs/real_operation/experiments/stage1_e2e_20261124/observation_samples/real_observation.json \
 --output /tmp/processor-probe-NEW.json
```

本次成功不保证其他2GiB会话一定可运行；先核对剩余内存，在独立有超时进程运行。数值变换不生效是审计结论，不是模型适配 PASS。

### M2：训练契约表与证据边界

以下来源均于2026-11-26检查：

- S1：[固定 checkpoint 模型卡](https://huggingface.co/lerobot/smolvla_base/blob/5e8d12a6e2975b0e5e5fce7c8caf47c371d257b6/README.md)，仅明确连续动作和面向特定用途微调，未定义完整物理契约。
- S2：[论文 v1](https://arxiv.org/html/2506.01844v1)，§3.2/附录：多社区数据，不能把实验平台或当前配置当作全部训练历史；相机标准化顺序 top/wrist/side。未据此恢复每个训练组。
- S3：[参考数据 metadata](https://huggingface.co/datasets/lerobot/svla_so100_pickplace/blob/a440cbb2b4403dd16f1d47eb84a09abbb97717a4/meta/info.json)，v2.1；当前 v3.0 revision `728583b5eaf9e739a7f119e2def466fa1d552402`。小文件快照及哈希见 [source_metadata.json](results/stage2_20261126/source_metadata.json)。它是官方参考数据集，不是已确认的 base 全部预训练清单。
- S4：[固定 SO follower](https://github.com/huggingface/lerobot/blob/ff0acf5ee6434369eeeb9d26e76d1e2ddc20c0d2/src/lerobot/robots/so_follower/so_follower.py)：Present_Position/Goal_Position，use_degrees 决定 DEGREES 或 RANGE_M100_100，夹爪 RANGE_0_100。这不是已绑定到 S3 采集的代码版本。
- S5：[官方迁移 PR](https://huggingface.co/lerobot/smolvla_base/discussions/11)，说明内置统计迁移到外部 pipeline；[#4415](https://github.com/huggingface/lerobot/issues/4415) 报告多组键错配和旧 revision `3326b100334f` 的状态统计；[#2374](https://github.com/huggingface/lerobot/issues/2374) 提供另一复现。issue 作者打印值是线索，未独立核验原权重，不作为可直接部署的统计来源。
- S6：[官方微调说明](https://huggingface.co/docs/lerobot/smolvla)，当前文档，不能用于证明历史训练定义。

|属性|参考数据/源码已知|当前 base checkpoint 的可用契约|
|---|---|---|
|State channels|S3: main_shoulder_pan/lift, main_elbow_flex, main_wrist_flex/roll, main_gripper，六维顺序一致|候选对应，全部训练组同序 UNKNOWN|
|Action channels|S3 同 state 名称与顺序|候选，不等于已证明物理语义|
|State units|S3 无单位；S4 可选 degree 或归一化范围|UNKNOWN，不能依据统计幅值判定|
|Action semantics/units|S4 为位置目标，S3 未绑定采集实现|绝对/增量与单位完整绑定 UNKNOWN|
|Zero/direction/range|S4 依赖每台机器人 calibration；S3 无实体校准文件|UNKNOWN，阻止可靠 sign/offset/范围映射|
|Gripper|S4 为0–100归一化，非 JetRover rad|历史组定义 UNKNOWN|
|State statistics|issue 提供旧文件线索；当前文件缺失|独立可追溯统计 UNKNOWN|
|Action statistics|三组 SO-100 mean/std 已读|分组来源、选组规则与采集单位 UNKNOWN|
|Dataset FPS|S3 两版本均30 FPS|不能外推全部训练组；不能用历史20ms检查时序代替|
|Action timing|S3 元数据30Hz并非硬件下发/达到目标时间|UNKNOWN|
|Camera binding|S3 top+wrist；S2 top/wrist/side；本项目 D405 wrist→camera1|具体训练视角匹配未证实，存在位置分布差异|

SO-100 是明确相关来源，但“三组存在”仅证明导出保存了多个命名统计组，不能证明三个数据集、三种单位或只含一种机器人。迁移可能遗失状态统计的解释有上游支持；没有恢复完整训练记录、原 calibration 与选组逻辑。完成这一轮定向调查后停止扩大溯源，不下载旧完整权重或大数据集。

### M3/M4：主推荐自身数据后训练，直接映射保留待证实基线

|方案|证据与成本|RJ45与论文影响|
|---|---|---|
|A 直接关节映射|功能对应成立；源单位/零位/方向、独立统计与夹爪映射未闭合。代码成本低，验证与标定成本仍高|即使数学可逆也不保证精细任务迁移；条件不足，本轮不实现真实 Adapter|
|B 运动学重定向|需双方可信 FK/TCP、任务空间动作语义、五自由度可行性及 IK；成本高，仍绕不过未知 native 语义|不能用 IK 修补语义缺失，可能偏离论文重点；本轮仅评估|
|C 自身数据后训练|需要可信示教、独立 state/action 统计和 GPU；可消除对未知 SO-100 统计组的依赖|能覆盖自身夹爪、腕部视角及 RJ45 子任务，工程适配与论文规划模块分离；主推荐|

选择 C 的依据同时包括：S1/S6 的 base 定位、当前实际转换不生效、源物理契约不足、机器人几何与视角不同，以及没有 JetRover RJ45 能力证据。不是断言只有后训练才可能成功。未来若获得完整源契约和实测映射，可重新评估 A；当前不构造 sign=1/offset=0 的伪标定，也不注册真实执行映射。

## 10. JetRover Dataset 与后训练契约 v1（离线准备）

当前实现 `deployment/local/validate_adaptation_dataset.py` 是标准库离线 staging 校验器，不是第二套机器人协议，不是已完成的 LeRobot 转换器，不连接任何驱动。单元测试使用显式 synthetic_test 数据；图像仅做路径/哈希验证，未声称完成解码、尺寸或视频完整性检查。

### 10.1 State / Action 独立契约

两者顺序均为 `[joint1,joint2,joint3,joint4,joint5,r_joint]`，单位均为 rad，六维 float32（训练导出时转换）。state=实际 bus feedback 的名义位置；action=控制器实际发送的绝对位置目标，不是当前反馈，不是未经验证 native action，也不是示教器未经本体转换的手柄坐标。夹爪作为独立驱动关节角，不能写成开口百分比。物理标定后冻结 calibration_id 与安全范围，不在 episode 内变更；重标定生成新版本。

Manifest JSON 必需字段：schema_version=1、channels、units、state_semantics=nominal_joint_position、action_semantics=absolute_commanded_target、data_kind（synthetic_test/real_demonstration）、calibration_id、clock_domain、camera_id、sample_period_ns、max_alignment_ns、joint_bounds_rad（6组明确上下界）。当前无实测安全范围，不提供可直接用于硬件的默认上下界。source SHA、相机序列/固件/分辨率/编码/标定文件哈希、采集程序及安全检查记录应随 manifest 外部证据保存。

每帧 JSONL 必需：episode_id、scene_group、split、frame_index、data_kind、calibration_id、instruction、subtask（insert/unlock/pullout）、state、action、state_source=bus_feedback、action_source=controller_sent_target、clock_domain、timestamp_ns、state_timestamp_ns、image_timestamp_ns、command_timestamp_ns、target_timestamp_ns、camera_id、image_path、image_sha256。real_demonstration 还要求 feedback_valid、physical_calibrated、timing_verified 全真；这些标记不是硬件证明，采集准入仍须核验其原始证据。每路 Pulse、TX/RX 区间、反馈异常、实际发送的逐通道目标及命令ID、原始设备时钟独立留存，以文件引用关联帧。

一个 episode 是一个连续示教尝试，保留成功/失败/中止、任务标签、场景及操作者/采集 session；不把失败重写成成功。建议逐子任务示教，再记录完整组合任务；失败试次可保留用于诊断，训练是否纳入需人工明确标注。

### 10.2 采集、相机与时间

D405 使用现有 RGB8 848×480 配置与腕部相机身份；depth/CameraInfo 在旁路保存，不默认为 SmolVLA 特征。训练与推理固定同一 wrist→camera feature 映射（候选 camera1），不声称匹配旧 base 视角；缺少 top 相机则不伪造。未来如果加外部视角，另定数据版本。

使用经审核的遥操作/示教或受监督位置目标接口，复用统一驱动记录 actual sent target；当前不实施示教。必须先恢复可靠反馈、完成关节/夹爪方向/范围及所需时间标定。优先保留相机原始header与host接收时间、每路反馈区间和command时刻；确认时钟关联后才能生成统一样本时间，不能把host到达改写为设备采样。

pilot 候选采样10Hz（不是已验证的执行频率），从实测完整反馈与图像可用率决定是否接受；manifest 必须显式设置周期与对齐容差，冻结前需测量验证。每个 t_k 样本关联 t_k 前已可用的反馈/图像和在 t_k 发出的目标，记录计划到达 t_target；目标与反馈分别保存。插值/最近邻只允许在明确有效区间内，保留残差和方法；超龄、缺帧、跨故障、未知事务关联则拒绝或切分 episode，不静默填补。训练 chunk 的 k..k+49 对应冻结后的50个时间步，不复用历史20ms offsets。校验器检查同域、对齐、连续帧与周期；不替代采样时钟审计。

### 10.3 LeRobot 转换与统计

下一步使用固定 LeRobot v3 的 `LeRobotDataset.create`、`add_frame`、`save_episode` 导出，而不是手写 Parquet/video。features 显式定义 `observation.state:[6]`、`action:[6]` 与 RGB video feature；task 保存指令，帧按 episode 时间顺序插入；raw manifest 保留绝对时刻，LeRobot timestamp 为 episode 相对秒。camera rename_map 显式固定；确认 reader 的 delta_timestamps 与50步时序，episode 末端 padding mask 不作为额外训练样本。当前未实现/运行转换与视频 round-trip，标为 NOT_RUN。

校验器流式计算**仅 train**帧的 state/action 独立均值与总体标准差，使用 Welford；验证/测试帧不参与统计。输出分别绑定 `observation.state` 与 `action`，不使用 SO-100 组、不双重归一化。零方差通道报告出来，训练前人工决定冻结或处理，不自动替换标准差。真实数据训练前再与 LeRobot 官方导出统计对照；图像统计由官方流程处理。

按完整 episode 且按 scene_group 隔离 train/validation/test；同一接口、布线布局及重复尝试成组，避免近重复泄漏。训练脚本独立读取 manifest split 清单，不能依赖数据集默认全train。校验器拒绝跨split场景和episode、NaN/Inf、错误维数/顺序、越界、时间跳跃、state/action来源混用、图像缺失或哈希不符；不裁剪后冒充合格数据。

### 10.4 Pilot 与正式训练

先在自由空间分别覆盖接近后插入、解锁、拔出，保留接口位置、初始姿态、线缆与夹持变化。pilot 的目的先是验证示教质量、时间对齐、六维标签、图像映射、统计实际生效、batch/损失和checkpoint导出；不预设固定数据量。取得GPU后从固定 base 初始化，优先复用记录的冻结视觉骨干/训练expert与state projection配置，实际 trainable 参数需审计；不预先改变网络结构。

正式训练数量依据 pilot 学习曲线和未覆盖任务变化扩展；比较未训练/后训练与确定性基线，固定独立测试场景。记录验证损失、各通道误差、越界比例、实际processor正控制、离线轨迹连续性；这些不代替任务成功率。受控实机阶段报告插入/解锁/拔出成功率、执行时间、中止和故障率，以及对准误差/力等实际可测指标。pilot 成功判据应在采集前冻结；没有达到可信反馈和执行准入不得仅凭离线损失启用闭环。

### 当前完成边界

Dataset contract/校验与独立统计工具 IMPLEMENTED；6项单元测试（含多种错误输入子例）PASS。采集、LeRobot格式转换、视频检查、训练、GPU回放与真实闭环均 NOT_RUN。后训练路线是当前工程推荐，不是已证明的策略能力。
