# P2.3-R1 Modular Architecture

本轮仅重构已经验收的 G1–G4、B1/B2；不包含碰撞、IK 或规划实现。
P2.3-R1 Modular Refactoring Completed — Pending User Review.

## 1. 审计与重构前基线

- 分支：`feat/workspace-planning`，未切换或创建分支。
- 起始 HEAD：`357a7b659f4bf64d58c935c349ad91484cc262b8`，B2 visualization commit。
- 起始 Git 状态：只有已有未跟踪的 `docs/develop/`；本轮未修改它。
- 重构前完整回归：**360 passed, 1 warning, 0 skipped**，8.62 s。
- 原有 28 个 Python 文件（12 个生产模块、6 个 CLI、10 个测试/辅助文件）均审计了
  imports、公开定义、调用方向及路径定位。README、P2.3 架构、既有正式报告作为上下文审查。
- 所有六个 CLI 通过自身 `__file__` 找到 `src/`；不依赖 CWD，没有打包安装要求。
- G1 model/FK 与 G1-B TF 生命周期已验收，本次整文件搬迁，不拆分其算法/同步状态机。

重构前的主要问题：

| 原模块 | 职责 / 主要接口 | 内部依赖 | 外部依赖与拆分结论 |
|---|---|---|---|
| `model.py` | Xacro、URDF 树/链、JointSpec、KinematicModel、load_model/model_from_xml | 无 | NumPy、YAML、urdf_parser_py；Xacro/ament 按需，迁入 robot |
| `fk.py` | FKResult、ForwardKinematics.pose/__call__、URDF→KDL | model | NumPy、PyKDL；转换/求解保留一处 |
| `sampling.py` | WorkspaceSamples、sample_joint_positions/evaluate_joint_samples/sample_workspace、validate_arrays/recheck_fk | fk | NumPy、SciPy；将 FK 导入延迟到求值/复核 |
| `results.py` | NPZ/JSON schema、统计、save_results/load_results、防覆盖事务、file_sha256 | sampling | NumPy、标准库，保存时 PyKDL 版本信息；仅抽出无依赖文件哈希 |
| `transforms.py` | SE(3)、Pose、Frame/FrameGraph、resolve_transform | 无 | NumPy、SciPy；数学与场景 Frame 图分离 |
| `scene_model.py` | 严格 YAML/schema、canonical JSON/hash、SceneSnapshot、load_scene/parse_scene | transforms | NumPy、PyYAML；解析单向构造数据对象 |
| `visualization.py` | WorkspaceView、prepare_view/load_workspace、3D/projections、HTML pair | model/results/sampling | NumPy、按需 Plotly；迁入 viz，模型仅在 loader 使用 |
| `scene_visualization.py` | primitive meshes、object/world transform、Frame/Port、Figure、single HTML | scene_model/transforms；按需 visualization | NumPy、按需 Plotly；纯网格独立 |
| `tf_validation.py` | endpoint audit、等待/初始化/回收、run_case、finalize_report | 无，model/FK 由调用方传入 | NumPy、SciPy、YAML；ROS 消息/rclpy/tf2 仅实际运行导入 |
| `validation.py` | VoxelGrid、compare_arrays、实验复用、prior_evidence、run_validation | model/results/sampling；延迟 validation_report | NumPy/SciPy/PyKDL；编排与共享统计契约分离 |
| `validation_report.py` | CSV/Markdown、write_bundle/load_validation_bundle | validation；results.file_sha256 | 标准库；原本间接加载 robot/PyKDL，解除该依赖 |
| `__init__.py` | 四个 G1 根包公共 API | 惰性 model | 只改新模块目的地，保留身份和 `__all__` |

原有循环是 `validation.run_validation → validation_report → validation`，只是靠函数内导入避开
初始化错误，并非真正单向。其他生产模块没有发现静态导入循环。

## 2. 目录迁移

重构前（相对 `src/workspace_analysis/`）：

```text
__init__.py
model.py
fk.py
sampling.py
results.py
transforms.py
scene_model.py
visualization.py
scene_visualization.py
tf_validation.py
validation.py
validation_report.py
```

重构后完整生产包：

```text
src/workspace_analysis/
├── __init__.py
├── geometry/
│   ├── __init__.py
│   └── se3.py
├── robot/
│   ├── __init__.py
│   ├── model.py
│   └── fk.py
├── scene/
│   ├── __init__.py
│   ├── frames.py
│   ├── schema.py
│   └── model.py
├── workspace/
│   ├── __init__.py
│   ├── artifacts.py
│   ├── sampling.py
│   └── results.py
├── viz/
│   ├── __init__.py
│   ├── meshes.py
│   ├── workspace.py
│   └── scene.py
└── experiments/
    ├── __init__.py
    ├── statistics.py
    ├── tf_validation.py
    ├── workspace_validation.py
    └── reporting.py
```

其他改动：6 个既有 `scripts/*.py`、8 个既有 `tests/test_*.py` 更新导入目的地，
不改 CLI 参数与测试验收逻辑；`conftest.py`、独立 `reference_fk.py` 不变。
新增 `tests/test_architecture.py`、`tests/fixtures/r1_numeric_baseline.json` 和本文；更新 README。
`config/`、`results/`、`reports/`、既有 P2.3-A 架构记录和官方资源不变。

| 原内部模块路径（`workspace_analysis.` 后） | 新路径 / 拆分 |
|---|---|
| `model` / `fk` | `robot.model` / `robot.fk` |
| `sampling` | `workspace.sampling` |
| `results` | `workspace.results`；file_sha256 唯一实现在 `workspace.artifacts`，results 正常显式导入它 |
| `transforms` | 纯数学与 Pose → `geometry.se3`；Frame/FrameGraph/resolve_transform → `scene.frames` |
| `scene_model` | load_scene/parse_scene/schema/canonicalization → `scene.schema`；SceneSnapshot/SceneError/deep freeze → `scene.model` |
| `visualization` | `viz.workspace` |
| `scene_visualization` | box_mesh/cylinder_mesh/线框拓扑与尺寸验证 → `viz.meshes`；object_mesh/Frame/Port/Plotly/HTML → `viz.scene` |
| `tf_validation` | `experiments.tf_validation` |
| `validation` | `experiments.workspace_validation`；共享 ROW_FIELDS/METRICS/ValidationError/aggregate_seeds → `experiments.statistics` |
| `validation_report` | `experiments.reporting` |

与建议目录相比只增加 `workspace/artifacts.py`、`experiments/statistics.py`：前者避免报告读取
为计算文件哈希加载整条采样依赖，后者消除 G4 循环。不是新功能或算法副本。
Pose 属于纯局部刚体数学，不引用场景 ID；FrameGraph 的 `base_link` 与命名规则归 scene。
object_mesh 需 SceneSnapshot 与世界变换，故留在 viz.scene；meshes 只生成局部几何，不导入 scene/Plotly。

## 3. 单向依赖与加载边界

箭头表示调用/导入方向，包含函数内惰性导入：

```text
scene.schema → scene.model → scene.frames → geometry.se3
scene.schema ───────────────→ scene.frames / geometry.se3
robot.fk → robot.model
workspace.results → workspace.sampling → robot.fk
workspace.results → workspace.artifacts
viz.workspace → workspace.results / workspace.sampling
viz.workspace.load_workspace → robot.model
viz.scene → viz.meshes / scene.schema / scene.model / scene.frames / geometry.se3
viz.scene（仅 workspace overlay）→ viz.workspace
experiments.workspace_validation → robot / workspace / experiments.statistics / experiments.reporting
experiments.reporting → experiments.statistics / workspace.artifacts
experiments.tf_validation：接收 model/FK 对象；实际运行才导入 ROS
```

- geometry 只使用 NumPy/SciPy/标准库，不依赖其他领域。
- robot 不依赖 scene、viz、experiments；没有未来后端反向依赖。
- scene 仅依赖 geometry、NumPy/SciPy、标准库；schema 额外依赖 PyYAML。
  SceneSnapshot 不导入 YAML 解析器；schema 调用构造器，不反向导入 schema。
- workspace 可依赖 robot；数据数组校验和无 model 的已有结果读取不因 import 加载 PyKDL。
  实际 FK 求值及带 model 的复核继续使用原 PyKDL；没有弱化校验。
- viz 的 Plotly 仍在绘图/导出时导入；纯 meshes 只依赖 NumPy。纯 Scene 模式不加载 robot 或 workspace Figure 模块。
- experiments 位于上层；底层不导入 experiments。reporting 读取既有报告不加载实验编排/PyKDL。
- 六个子包 `__init__.py` 仅含文档，不进行批量 re-export、初始化或注册。
- 根包仅延续既有四个 G1 属性的惰性导出。没有 sys.modules 模块别名、sys.path 导入重定向或 monkeypatch 生产实现。

原有受锁保护的 Xacro 环境临时覆盖/还原、YAML 专用 Loader 的 resolver、根包懒加载缓存均保留。
没有新增隐式全局业务状态、ROS context、进程或网络连接。

## 4. 公共与内部 API、行为兼容

根包公共 API 保持：

```python
from workspace_analysis import KinematicModel, ModelError, load_model, model_from_xml
```

`__all__` 的四个名称及顺序、对象身份、参数默认值不变。ForwardKinematics 原本不是根包导出。
推荐显式领域导入：

```python
from workspace_analysis.robot.fk import ForwardKinematics
from workspace_analysis.geometry.se3 import Pose, pose_to_matrix, transform_point
from workspace_analysis.scene.frames import Frame, FrameGraph, resolve_transform
from workspace_analysis.scene.model import SceneSnapshot, SceneError
from workspace_analysis.scene.schema import load_scene, parse_scene
from workspace_analysis.workspace.results import load_results
from workspace_analysis.viz.meshes import box_mesh, cylinder_mesh
from workspace_analysis.viz.scene import build_scene_figure, load_scene_figure
from workspace_analysis.experiments.reporting import load_validation_bundle
```

各领域的公开命名函数保留签名/数据语义；以 `_` 开头的解析、转换、发布辅助函数不承诺外部 API。
SceneVisualizationError 由 meshes 唯一定义、scene 显式导入，所有原显示错误仍使用同一异常类型。
没有保留旧平铺子模块 shim。仓库外 notebook/脚本、按旧 `__module__` 保存的 pickle、运行中的
长寿命 Python 会话需要更新导入/重新启动；NPZ/JSON 不使用 Python pickle，不受影响。
未声称已经搜索或修复仓库外所有调用者。

必须保留且已核对：

- G1–G4 数值运算、顺序、阈值、seed/RNG、统计 ddof、体素半开区间与实验状态语义。
- Schema v1、不可变快照、全父链固定变换、Port local +Z outward / -Z insertion / +X key。
- 原始 YAML 与 canonical JSON 算法/哈希、NPZ/JSON 数组格式与模型/哈希复核。
- B2 显式三角网格、坐标、颜色、legend、customdata 原始行号、负 Z、Figure 元数据与提示。
- G2/G3/G4/B2 各自既有防覆盖、锁/marker、原子 hard-link、回滚机制：原实现搬迁，未合并为新事务框架。

两个必要路径适配：

1. `viz.scene.load_scene_figure` 默认配置从新的 `parents[3]` 定位 workspace 根目录；显式配置/CWD 语义不变。
2. G4 `_code_hashes` 由平铺 glob 改为 package 根目录递归 rglob，覆盖所有新子包实现及初始化文件。
   新实验的源码路径/hash 必然变化，仍是原 `code_sha256: {absolute_path: sha256}` 格式。
   已有正式报告的历史 code_sha256 原样保留，不伪造为新代码的实验结果。

## 5. 基线与验证结果

固定测试基线来自重构前实际执行，保存在 `tests/fixtures/r1_numeric_baseline.json`。
它包含三组 4×4 FK、全部场景 Frame、Port、九个对象的 mesh shape/dtype/bytes SHA、16 行 seed42
采样四数组的 shape/dtype/bytes SHA。JSON 中的数值没有手工重新计算或降低精度。
这些逐位比较只适用于本次软件环境，不承诺不同 BLAS/CPU/NumPy/SciPy/PyKDL 版本逐位一致。

Scene SHA256，前后完全一致：

```text
Raw YAML:
cae02112d6a051b0bff3a6c7c8698ee1acc9a12e226fd75bd4df9b683ab5fba4
Canonical Scene:
0196cd18fc10ef9e4a23cc054e1aecfcb4e3509f220344ee538d152a5fb210a6
```

| FK 输入 | XYZ [m]，重构前后相同 |
|---|---|
| Zero | [0.0251328065010765, 0, 0.5046119008436533] |
| Nonzero | [-0.04445128177737234, 0.025400175008429574, 0.4767464692649549] |
| Near-limit | [0.05642523430448581, 0.05604811047047301, 0.4910684981535641] |

三组完整 T 的最大元素差均为 0。Cabinet=(0.45,0,0.20)、Panel/Port=(0.30,0,0.20) m；
outward=(-1,0,0)、insertion=(+1,0,0)、key=(0,0,+1)，前后矩阵/向量逐位一致。
九个对象 Box/Cylinder 顶点和 faces 全部相同。16 行四数组逐位相同。

对同一实际输入路径，G3 3D、投影、B2 Scene Only、B2 100 点叠加的 Figure.to_json() 前后均相同。
这是完整 Figure 数据/布局对比，不比较随机 HTML div ID，也不重新宣称浏览器视觉验收。
已有 100/10,000 点 G2 数据经原 loader/model 复核返回相同四数组和 metadata；
已有 `g4_run01` 9 组实验 bundle 校验/解析相同，Markdown 重渲染文本 SHA 相同。
正式 NPZ/JSON/HTML/CSV/Markdown、synthetic_dev.yaml、官方资源均进行文件哈希保护检查。

原有测试仅调整 imports、限定符和隔离子进程中的模块路径。归一化这些路径后，
全部既有测试 AST 相同，没有删测试、改 expected、改阈值或加 skip。
G1 robot/model.py、robot/fk.py、experiments/tf_validation.py 搬迁前后字节相同。

新增测试覆盖：根目录约束、子包无副作用、包含 lazy imports 的全图无环/依赖方向、13 个模块
在阻止 ROS/URDF/PyKDL/Plotly/MoveIt/FCL 导入时独立加载、禁止导入时发起进程/网络/环境变更、
纯场景读取与既有 G4 报告读取、冻结数值基线、完整源码哈希覆盖、默认配置定位、六个 CLI help、
G1 reference CLI 和 G1-B 非法 timeout 在创建报告/ROS 前拒绝。

最终实际项目目录回归：**388 passed, 1 warning, 0 skipped**，10.76 s。
即原有 360 项加新增 28 项；warning 与重构前相同，来自 urdf_parser_py/Xacro 使用的
旧版依赖对 Python 3.11 私有属性的提示，没有新增 warning。

最终再次捕获的关键 API/数值/完整 Figure/G2/G4/CLI help 基线全部相同。
120 个受保护既有文件 SHA256 不变。149 个原函数/类在归一化 import 后 AST 与原实现相同且
各有唯一权威定义；另外仅根包导入目的地、默认配置层级和递归源码哈希三处做必要适配。
`git diff --check` 通过；没有 Git commit/push/merge。

CLI 检查边界：

| CLI | 验证方式 |
|---|---|
| check_fk.py | help 前后文本相同；从其他 CWD 执行 --reference，三组结果与基线相同 |
| sample_workspace.py | help 相同；既有小样本成功/防覆盖/错误退出测试 |
| plot_workspace.py | help 相同；既有 100 点读取、离线导出及错误路径测试 |
| inspect_scene.py | help 相同；Scene Only 隔离运行、100 点叠加、离线 HTML、防覆盖测试 |
| validate_workspace.py | help 相同；小规模 diagnostic 编排、复用、回读/失败/防覆盖测试 |
| validate_tf.py | help 相同；非法 timeout 拒绝；原模拟 endpoint/清理测试全保留；**未运行 ROS 集成** |

## 6. 未来 Collision Backend 边界与未解决问题

未来上层编排应消费 `robot.model.KinematicModel` 的 URDF/关节顺序/完整关节状态以及
`scene.model.SceneSnapshot` 的 canonical 数据/FrameGraph；backend 自己管理全机 collision geometry、
mesh 身份、ACM、padding、tool scope、运行时对象和 native 生命周期。
backend 不应反向注入 geometry/scene/robot，也不能把渲染 mesh 当作已验证的机器人碰撞几何。
已有 workspace loader 提供强校验 q/数组及原始行号，未来分类模块应消费它，不改采样算法和既有数据。
当前没有创建任何 Collision/IK/Planning 占位模块或实际接口实现。

仍需明确：

- TF 实际 ROS2/RMW 行为没有因本次单测再次验收；状态机源代码虽字节相同，真实集成由用户决定何时运行。
- G4 编排仍需 PyKDL/ROS 模型加载环境；独立报告读取已经解除该依赖。
- `tf_validation.py` 仍较大，服务于同一受控实验生命周期；后续如需拆解，应独立审查同步/清理契约。
- strict schema 在 schema.py 内仍集中处理跨记录一致性；Snapshot 直接 dataclass 构造并非验证入口。
  外部应使用 load_scene/parse_scene，不把未经解析的手工对象视为已验证场景。
- G2、G3、G4、B2 发布事务有相似机制但提交标记与文件数量不同，本轮不为统一外观改写安全语义。
- 模型只支持既有 fixed/bounded revolute 链；PyKDL 求解器实例不适合共享并发调用。
- 官方 lidar collision 缺失、TCP 未标定、ACM/完整碰撞范围尚未审核，依旧阻止物理安全结论。
- 源码路径属于内部 API；外部旧路径、pickle、旧 P2.3-A 历史文档的文件定位需参考本迁移表。
  正式报告和已验收历史文档不重写成重构后的实验记录。
- 未新增 wheel/安装配置；保持 CLI/pytest 自定位 src，API 用户仍需显式 PYTHONPATH。

## 7. 用户验收命令

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest -q -rs -p no:cacheprovider tests

# 不生成或覆盖既有报告
.venv/bin/python -B scripts/check_fk.py --reference
.venv/bin/python -B scripts/validate_tf.py --help
.venv/bin/python -B scripts/sample_workspace.py --help
.venv/bin/python -B scripts/plot_workspace.py --help
.venv/bin/python -B scripts/validate_workspace.py --help
.venv/bin/python -B scripts/inspect_scene.py --help
```

纯 Scene 模式可在未 source ROS 的新终端检查：

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
PYTHONPATH="$PWD/src" .venv/bin/python -B -c 'from workspace_analysis.scene.schema import load_scene; s=load_scene("config/scenes/synthetic_dev.yaml"); print(s.raw_yaml_sha256); print(s.canonical_scene_sha256)'
```

本轮不重新生成正式工作空间或运行真实 TF。没有安装依赖、修改系统/官方模型、Git commit/push/merge。
