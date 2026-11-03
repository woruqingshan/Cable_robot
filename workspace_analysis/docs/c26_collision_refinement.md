# C2.6 collision refinement — software complete, model review PARTIAL

本轮基于 `feat/workspace-planning`，起始 HEAD `13f89033f0208e3ca90dc98e494140175ba6e45d`。起始 Python 回归 441 passed / 1 warning / 0 skipped；Native 1/1。未提交修改；HEAD 不能单独标识本轮工作树，运行身份同时保存 Backend SHA256、模型、场景、策略身份。历史文件保护核对见本轮 `integrity.json`。

## 实现与边界

- `cpp/collision_backend/include/engine.hpp`、`src/engine.cpp`：复用 MoveIt RobotModel / PlanningScene / RobotState、原碰撞布尔查询、原接触结果；增加实际 FCL 最小无符号集合间距。缓存同一 MoveIt 几何，保持 Collision Origin、Scale、固定/活动/Mimic 变换一致。没有 Python 距离替代算法或第二套 FK。
- `cpp/collision_backend/src/main.cpp`：协议 v1 保持原 QUERY；增加 EVALUATE 同义入口。现有 Client 仍使用 QUERY。
- `cpp/collision_backend/tests/native_tests.cpp`：保留旧断言，增加距离真值和输入顺序回归。
- `config/collision/assembly_policy_v1.yaml`：新增 2 对模型级固定装配许可，共 5 对；baseline、v0 保持原内容。69 个历史未解决 Pair 仍保留。
- `collision/evaluator.py`、`experiments/constrained_workspace.py`：通用配置判定和实验封装，详见 D0 文档。
- `scripts/refine_collision.py`：定向关节扫描、夹爪状态、81 点 joint2/joint3 探针、真实机器人 Viewer。
- `scripts/report_collision_refinement.py`：从实际查询记录再生成接触区域与 ACM 对照图，输出目录必须不存在。
- `scripts/check_constrained_run.py`：只读核对 D0 输入哈希、原始 q 顺序、NPZ、JSON 汇总和 HTML 实际坐标。
- 新增 `tests/test_constrained_evaluator.py`，未修改既有测试断言。

## ACM v1 的依据

数据：`results/c2_6/refinement_52d15f1e88/`。

| 新增 Pair | 51 个扫描姿态的相对矩阵最大变化 | 记录到的最大接触深度 |
|---|---:|---:|
| back_shell_green_link / base_link | 0 | 0.0019233382 m |
| link4 / servo_link2 | 1.67e-16 | 0.0001541115 m |

两对在 URDF 中具有完全固定的连接路径；与 C2.5 实际 Mesh 和接触区域证据一致。许可定义为同一刚性装配内部碰撞面的模型豁免，审核状态始终为 `provisional_model_only`，没有推广到硬件安全或相邻活动 Link。相对矩阵、真实 FCL 接触点、历史拓扑证据及其来源哈希分别保存于 `pair_refinement.json`、`joint_probes.json`、`focus_topology.json`。

未许可的重点活动 Pair：

| 扫描 | 目标 Pair | 有接触的扫描姿态 | FCL 代表接触深度范围 [m] |
|---|---|---:|---:|
| joint1，URDF 限位内 17 点 | green shell / link1 | 17/17 | 8.50e-8 … 1.23e-3 |
| joint2，URDF 限位内 17 点 | link2 / servo_link1 | 17/17 | 2.19e-4 … 1.95e-3 |
| joint3，URDF 限位内 17 点 | link2 / link3 | 15/17 | 2.57e-6 … 5.17e-4 |

其他活动关节取 0，夹爪 0.3 rad。接触点额外变换到对应关节的零角轴坐标系，图中颜色表示扫描角度。这里每对最多记录一个 FCL 代表接触点，不是完整接触流形、最深穿透或实物表面测量。joint3 存在接触消失的角度，其他 Pair 也跨越活动连接；频率高不足以证明整对 Link 在全部姿态都应豁免，因此全部保留检测。两个 joint3 极值和 Sample 8/73 的完整 Mesh Viewer 保存在 `figures/`。

夹爪：r_joint 测试 -1.57、-0.8、0、0.3、0.8、1.57 rad，分别在机械臂零位及原 Sample 8/73 查询，共 18 姿态。Native 使用全树 Mimic 状态，自动化测试核对每条 Mimic 公式。0→0.3 时 l_out/r_out 接触消失；gripper/l_in、gripper/r_in、l_in/l_out、l_link/l_out、r_in/r_out、r_link/r_out 在这些测试开度中持续存在。测试中的环境布尔判定未改变：零位无环境碰撞，8/73 有环境碰撞；不能据此推断所有 q 的开度影响均相同。

Bare-Gripper 参考状态预先声明为 r_joint=0.3 rad，它是合法、非零、可复现的模型参考；不是实物夹爪开口毫米值，也不是为提高通过率优化出的开度。81 点 joint2/joint3 网格、其他活动关节零位、v1 下没有发现模型可行配置，完整 q 和查询结果在 `directed_search.json`。

## Clearance 验证

本机 MoveIt 默认 distanceCallback 使用 FCL 默认距离求解器。独立小夹具复现：0.2 m Box 与 0.1 m Box、中心间隔 0.5 m，GST_LIBCCD 按输入顺序返回 0.35 与 0.4092676386 m；GST_INDEP 两个顺序均返回 0.35 m。证据在 `clearance_validation.json`。官方调用位置可查 [MoveIt Humble collision_common.cpp](https://github.com/moveit/moveit2/blob/humble/moveit_core/collision_detection_fcl/src/collision_common.cpp)。

因此实现显式选择 **FCL GST_INDEP**，直接使用 MoveIt 已加载的相同 CollisionGeometry 和 Native RobotState 变换。先使用既有 MoveIt Boolean CollisionResult 判断碰撞；碰撞集合间距记为 0，不将 penetration depth 伪装成负最短距离。非碰撞时对候选几何调用 FCL 距离，AABB 下界仅用于安全剪枝；机器人自身同 Link 不比较，Self 使用显式策略 ACM，环境不被 Self ACM 过滤。未加入新 FK、Python 几何距离或未测试的包。

Native 真值：正间距 0.35 m、刚好接触 0、重叠 0、Sphere/Box 0.35 m、旋转 Box 0.32928932188134524 m、多对象最近距离 0.15 m，且验证最近对象 ID。Self 已知间距 1.85 m。所有 11 个标量断言误差上限 1e-6 m，实际最大误差 2.22e-16 m。

支持范围：完整模型 Environment 最短间距、**剔除显式 ACM 对之后**的 Self 最短间距、原始碰撞 Boolean、真实接触深度诊断。没有符合条件的几何对时返回 unavailable/null；正阈值不能自动通过。没有有符号穿透距离、硬件误差裕量或 RJ45/线缆 clearance 保证。原 C2 policy schema 的 clearance 仍固定零；新正阈值通过 Evaluator/CLI 显式传入并进入 evaluation identity，避免悄悄改变旧策略身份。

## 验证与剩余问题

Python：452 passed、1 warning、0 skipped；Native：1/1 PASS。warning 与起始基线一致。新测试覆盖固定关系、活动极值、Mimic、千行顺序/EE 一致、不同 batch、错误保留、场景/策略/阈值身份、正间隙与不可用状态、统计守恒和不覆盖输出。

额外实际回放历史 C2 200 次查询（baseline/v0，各 100，夹爪 0）：Raw/Effective/Environment、完整碰撞对与过滤对均逐项一致，EE 矩阵最大变化 0。见 `legacy_c2_regression.json`。

Software Validation：通过。Model-Level Collision Validation：PARTIAL，固定内部装配证据足以支持这两项模型豁免，活动连接与夹爪仍需局部几何审计。Physical Hardware Validation：未执行。LiDAR 缺 Collision、RJ45/Tool/TCP/Cable、装配误差和夹爪标定继续列为覆盖缺口。没有隐藏这些限制，也没有据此停止软件实验。

浏览器工具拒绝本机 file://，因此本轮没有新 HTML 的浏览器视觉验收。数据核验脚本确认实际 Trace 坐标、空可行集合、拒绝集合重叠与来源 q。请本机打开 Viewer 完成人工评审。

完整用户运行命令见 [D0 文档](d0_constrained_workspace_pilot.md)。
