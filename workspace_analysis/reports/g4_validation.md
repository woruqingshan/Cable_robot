# G4 Workspace Validation — 正式实验待运行

**status: NOT_RUN — 这是报告模板，不是正式实验 PASS 报告。**

G4 代码开发阶段仅执行小规模离线测试。N=1000 的两次正式复现与完整
9 组稳定性实验尚待用户运行。不能据此宣布 P2.2 验收完成。
正式命令见 ../README.md 的 G4 部分。不要覆盖此模板；正式输出使用新的
results/g4_run01/ 及 reports/g4_validation_run01.md。

## 1. Robot Model

只读审查确认：官方 JetRover_Mecanum，五个独立机械臂关节 joint1..joint5，
Base=base_link，EE=end_effector_link；m/rad。底盘固定，夹爪状态固定；
主链固定关节仍由 G1-A 保留。限位由 URDF 读取，当前五个关节均为
[-2.09, 2.09] rad，此处记录不构成硬编码计算参数或真实电机安全限位。

当前展开模型 SHA256：
`5ff0007d08a798ebe31bc4e243d457e0f463aa7b896c088a889c919571ccd72d`。

当前 workspace.yaml SHA256：
`0028f14160dc6bbf1dec0671c681961ebc46eedc0e0e7bd0c3d45c0ca5800b78`。

正式报告会从实际加载模型重新记录这些值、完整关节参数与源文件哈希，
不会直接把本模板当作运行证据。

## 2. FK Validation — 引用既有验收

此前用户已验收 G1-A 和 G1-B v3。本轮仅只读核对了原报告，**没有启动
RSP，也没有重新执行 ROS2 TF 集成验证**。

- 原 G1-A 报告：[g1a_fk.json](../results/g1a_fk.json)。Zero/Nonzero/Near-limit
  的最大位置误差约 1.22e-16 m，最大姿态误差约 1.50e-16 rad。
- 原 G1-B 报告：[g1b_tf_v3.json](../results/g1b_tf_v3.json)。三组 PASS，
  严格端点身份已核验，RSP 退出码均为 0；最大位置误差约 2.09e-16 m，
  最大姿态误差约 7.50e-16 rad。

上述是历史软件模型验证结果，不是实体机械臂定位精度。
正式 G4 报告会引用原文件路径与 SHA256，并保留 rerun_in_g4=false。

## 3. Workspace Sampling

复用 G2 的 NumPy default_rng(seed) 独立均匀关节空间采样与 G1-A PyKDL FK。
保存 float64 的 q(N,5)、position(N,3)、rotation(N,3,3)、
quaternion_xyzw(N,4)。四个数组的同一行始终对应同一个样本。

既有 N=100/10000、seed=42 数据已通过 G2 用户验收。本轮只读核对其
NPZ/JSON、模型、配置和哈希，可供正式 G4 命令复用；不是 G4 新实验结果。
其余七组由用户正式运行时生成，所有输出采用新路径。

## 4. Reproducibility — 待正式运行

预先定义的判据：同一模型、配置和软件环境，两次独立 N=1000、seed=42
采样中，全部四个数组的 shape/dtype 一致，且 np.array_equal 为真。
时间戳、耗时和 NPZ 文件的二进制哈希不要求相同。

| 数组 | Shape / dtype | 两次数组一致性 |
| --- | --- | --- |
| q | (1000,5) / float64 | 待运行 |
| position | (1000,3) / float64 | 待运行 |
| rotation | (1000,3,3) / float64 | 待运行 |
| quaternion_xyzw | (1000,4) / float64 | 待运行 |

最大位置误差：待运行。最大旋转角误差：待运行。最终 PASS/FAIL：待运行。
四元数 q/-q 的姿态等价误差另行计算；符号翻转不会被当作确定性数组复现通过。
不承诺跨 OS、NumPy/SciPy/PyKDL 版本逐位一致。

## 5. Sampling Stability — 待正式运行

| N | Seed 42 | Seed 43 | Seed 44 |
| --- | --- | --- | --- |
| 100 | 正式 G4 中核验复用 | 待生成 | 待生成 |
| 1000 | 待生成 | 待生成 | 待生成 |
| 10000 | 正式 G4 中核验复用 | 待生成 | 待生成 |

每组保存 XYZ min/max/mean/std（ddof=0）、AABB 边长/体积、原 G2 FK 循环
耗时及统一网格命中数。跨 seed 对每个标量统计量计算均值及样本标准差
（ddof=1）。复用耗时是历史运行值，不能作为受控性能基准。

正式结果将比较不同 N 的采样极值、mean/std 的跨 seed 波动和剩余变化；
不会预设所有统计量一定变稳或收敛。相同 seed 跨 N 使用 RNG 前缀，
不是不同 N 之间的独立实验。三个 seed 只能提供初步描述证据，不能证明
严格边界或普遍统计置信保证。AABB 体积不是真实可达体积。

## 6. Voxel Hit Statistics — 待正式运行

默认固定分析区域：origin=(-0.6,-0.6,-0.4) m，upper=(0.6,0.6,0.8) m，
分辨率 0.02 m，shape=(60,60,60)。此区域是覆盖既有采样范围的实验选择，
不是机械臂理论包络；新数据仍必须逐点检查。CLI 可指定其他区域和分辨率，
但所有待比较实验必须共同使用同一网格，跨度须是分辨率的整数倍。

float64 网格边界 edge[k]=origin+k*size；区间为 [edge[k],edge[k+1])，
上边界不包含在区域内。逐轴 searchsorted(side='right')-1 得到整数索引，
与理想实数网格的 floor 定义等价。记录实际使用的 origin/upper/shape、
用户请求 upper、索引规则与分辨率；不静默裁剪、不按每组 AABB 重建网格。

命中不同体素数及其跨 seed 波动：待运行。未命中不证明不可达；命中不表示
整个体素可达或无碰撞。不把命中体素体积之和称作严格可达工作空间体积。
2 cm 是分析分辨率，不是定位精度。

## 7. Visualization — 引用 G3 用户验收

G3 已由用户运行并验收。现有输出为：

- [10,000 点 3D](../results/workspace_n10000_seed42_3d.html)
- [10,000 点二维投影](../results/workspace_n10000_seed42_projections.html)
- [100 点 3D](../results/workspace_n100_seed42_3d.html)
- [100 点二维投影](../results/workspace_n100_seed42_projections.html)

使用 scripts/plot_workspace.py 消费经 G2 验证的数据，并导出离线 HTML；
本轮没有重新生成这些文件。G4 正式报告仅记录相关文件路径与哈希。

## 8. Limitations

- 未进行自碰撞或环境碰撞检测。
- 未验证真实电机安全限位、动力学或实体机械臂定位精度。
- 未考虑 RJ45 插拔姿态、方向、接近、接触和插入力约束。
- end_effector_link 不是标定后的网线插头 TCP，base_link 原点不等于地面。
- 未验证严格工作空间边界；有限样本、AABB 和体素命中均不证明真实可达体积。
- 均匀关节采样不等于均匀笛卡尔密度；负 Z 原样保留。
- 仅三个 seed，不作普遍收敛或统计置信保证。

## 9. Conclusion — 保持待验收

已实现模型驱动 FK/采样/可视化及 G4 复现、描述统计和体素命中分析代码。
正式复现误差、九组实验表、稳定性解释与体素统计尚待用户实际运行生成。
这些基础工具不构成完整运动规划系统。

**G4 代码开发完成，等待用户运行正式实验与验收。**
