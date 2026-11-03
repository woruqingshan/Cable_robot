# S1 — IDC Front-Facing Operation Scene v0

S1 IDC Prototype v0 Completed — Pending User Review.

本场景使用 Synthetic Prototype Parameters，不代表实测 IDC、19 英寸机柜或 RJ45 标准尺寸。
本轮只验证环境几何、坐标和可视化；没有运行碰撞检测、IK、轨迹规划或真实机器人。

## 1. 设计及坐标

机器人固定停靠于柜前，Base +X 指向机柜，+Z 向上。机柜坐标与 Base 平行，固定
`^base T_cabinet = [I, (0.43, 0, 0.22) m]`。机柜前边界 X=0.33 m，后边界 X=0.53 m。
0.33 m 是 Base 原点到前平面的距离，不是底盘外轮廓的实际安全间隙。base_link 原点也不是地面。

三个 Router Frame 均位于各自前面板中心，不在设备体积中心：

| Frame | 相对 Cabinet 平移 [m] | Base 平移 [m] |
|---|---|---|
| router_1（上） | (-0.10,0,+0.10) | (0.33,0,0.32) |
| router_2（中） | (-0.10,0,0) | (0.33,0,0.22) |
| router_3（下） | (-0.10,0,-0.10) | (0.33,0,0.12) |

Router 相对 Cabinet 的 XYZW 四元数均为 `(0,-sqrt(0.5),0,sqrt(0.5))`。
Router 局部 X 向上，Y 指向正面观察者左边，Z 朝柜外/机器人。每个 Port Frame
在对应 Router Frame 下只做局部 Y 平移，姿态为单位四元数。

编号 Pij：i=1/2/3 表示上/中/下，j=1/2/3 表示机器人朝 +X 看向机柜时的左/中/右。
因此左列是 Base +Y；从机柜内向外看时，视觉左右会相反。P22 是文档指定的首个目标；
Schema v1 没有新增 first_target 字段，也没有据此判断操作可行性。

## 2. 合成尺寸与实体构造

所有长度单位为 m；以下括号标明 mm 仅帮助阅读。

| 参数 | 值 |
|---|---|
| 机柜外包络：深 X × 宽 Y × 高 Z | 0.20 × 0.30 × 0.36 |
| 柜体 Base 范围 | X [0.33,0.53]，Y [-0.15,0.15]，Z [0.04,0.40] |
| 左右侧壁厚、顶/底厚、后壁厚 | 均 0.010 |
| 柜内净宽、净高 | 0.28、0.34 |
| 每台设备：深 × 宽 × 高 | 0.14 × 0.26 × 0.06 |
| 各设备 Base X 范围 | [0.33,0.47] |
| 设备中心垂直节距；净层间距 | 0.10；0.04 |
| 设备侧/顶/底壳厚；后壳厚 | 0.004；0.005 |
| 设备后壁至机柜后壁内表面间距 | 0.05 |
| 设备两侧至柜内侧间距 | 每侧 0.01 |
| 网口水平中心节距 | 0.07 |
| 网口开口：水平宽 × 垂直高 | 0.018 × 0.016（18 × 16 mm） |
| 前面板厚度 | 0.005 |
| 端口面至设备后壳前表面的空腔深度 | 0.135 |

设备上/中/下的 Base Z 范围分别为 [0.29,0.35]、[0.19,0.25]、[0.09,0.15]。
这是根据现有模型工作空间尺度选出的缩比例夹具；没有强行使九个目标都可达。
100 点点云仅供尺度对照，稀疏点云的外观不能证明位置可达或不可达。

机柜只由五块薄 Box 组成：左右侧壁、顶、底、后壁；不添加前门或填满内部的实体 Box。
每台设备用 11 个 Box：6 个前面板分块、上下左右 4 个壳体块、1 个后壁。三台尺寸完全一致。

前面板在 Router 局部坐标下的构造如下：

- Z 范围 [-0.005,0]，Port 原点在 Z=0 外表面。
- 上下横条的 X 范围分别 [0.008,0.030]、[-0.030,-0.008]，Y 覆盖 [-0.13,0.13]。
- 中间高度带 X∈[-0.008,0.008] 内，4 个竖条的 Y 区间依次为
  [-0.13,-0.079]、[-0.061,-0.009]、[0.009,0.061]、[0.079,0.13]。
- 三个未被实体填充的 Y 区间分别为 [-0.079,-0.061]、[-0.009,0.009]、[0.061,0.079]。
  从机器人看对应右、中、左三个开口，中心正是 -0.07、0、+0.07。
- 相同的开口横截面向设备内部延伸到局部 Z=-0.135，随后遇到后壁。
  壳体没有堵住此横截面，设备后壁也没有置于端口入口。

这些结构检查针对本夹具的解析尺寸与坐标，不是通用碰撞算法，也没有检验机器人/插头扫掠体。
薄板在拼接处接触，共同组成实体并集；不要求各块之间预留间隙。

## 3. Port 的 Base 位姿和方向

以下位置及方向由现有 FrameGraph、resolve_transform 和 port_directions 实际计算，
表中将浮点舍入误差显示为 0。所有端口旋转矩阵相同：

```text
R_base_port = [[0, 0,-1],
               [0, 1, 0],
               [1, 0, 0]]
```

所以每个完整 `^base T_port = [[R_base_port, XYZ], [0,0,0,1]]`。

| Port | Entity | Base XYZ [m] | Outward | Insertion | Key |
|---|---|---|---|---|---|
| P11 | router_1 | (0.33,+0.07,0.32) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P12 | router_1 | (0.33,0,0.32) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P13 | router_1 | (0.33,-0.07,0.32) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P21 | router_2 | (0.33,+0.07,0.22) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P22 | router_2 | (0.33,0,0.22) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P23 | router_2 | (0.33,-0.07,0.22) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P31 | router_3 | (0.33,+0.07,0.12) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P32 | router_3 | (0.33,0,0.12) | (-1,0,0) | (+1,0,0) | (0,0,+1) |
| P33 | router_3 | (0.33,-0.07,0.12) | (-1,0,0) | (+1,0,0) | (0,0,+1) |

P22：

```text
^base T_P22 = [[0, 0,-1, 0.33],
               [0, 1, 0, 0.00],
               [1, 0, 0, 0.22],
               [0, 0, 0, 1.00]]
```

YAML 仅定义 Port local +Z outward、-Z insertion、+X key；世界方向由父链旋转推导，
没有重复写入九组 Base 方向。全部 13 个显式 Frame（机柜 1、设备 3、端口 9）连接到隐式 base_link 根。

## 4. 语义、显示与未来碰撞输入

- collision_objects：38，全部为 Box（机柜 5 + 设备 3×11）。
- entities：4（cabinet 1、patch_panel 3）。使用 router_1/2/3 ID 区分设备，不改变 Schema 类型。
- ports：9，均 task_parameters_status=unmeasured；tool 仍 uncalibrated、tcp_frame=null。
- Visual Geometry：由既有 object_mesh/Plotly 对上述 38 个实体生成显示表面和线框。
- Collision Geometry：相同 collision_objects 可供将来环境后端消费，本轮未作碰撞判断。
- Semantic Port：任务坐标/方向与标签，不增加任何填充 Box；标签和箭头不构成孔洞证据。

覆盖范围仅为外框、三台设备薄壳和九个矩形开口。未建模：真实插座内部隔片/触点/卡扣、
PCB、电源、风扇、安装耳、门、螺钉、线缆/已插入插头、机房地板/相邻机柜、抓持工具/TCP。
大面积设备空腔是假设，不能视为实物允许插入的深度或安全空间。
clearance 值 0 与 epsilon=1e-7 沿用开发占位语义，不是测量得到的安全距离。
机器人模型自身的覆盖/TCP 标定等既有问题仍需未来阶段处理。

## 5. 可视化和用户检查

已使用现有 CLI 生成：

- `results/s1_idc_scene_only.html`
- `results/s1_idc_workspace_n100.html`

两者为离线自包含 HTML；第二份读取既有 `workspace_n100_seed42`，未重新采样，保留 100 行及负 Z。
本轮没有改动绘图默认行为或 Figure 基线，仅使用已有 `--axis-length 0.015` 缩短箭头。
现有场景色彩仍为机柜灰色、设备紫色，设备用 ID/位置区分。

浏览器审核时从 X 较小的一侧看正面，必要时转为正视，使用图例隐藏 Frame、方向或对象组：

1. 检查三层排列、左中右编号及 P22 中心位置。
2. 检查柜内留空，三台设备均显示真实开口；旋转观察壳体与后壁。
3. 检查尺度、端口与点云相对位置，区别几何重合与任务可行性。
4. 确认标签/箭头在你的浏览器窗口可辨识。默认图例项目较多，面板中心与中间 Port Frame 共点，
   标签可能重叠；可通过图例切换对应组。尚未宣称浏览器视觉验收通过。

生成命令（首次使用新文件名；现有输出不会被覆盖）：

```bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
.venv/bin/python -B scripts/inspect_scene.py \
  --scene config/scenes/idc_prototype_v0.yaml --axis-length 0.015 \
  --output results/s1_idc_scene_only.html

source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
.venv/bin/python -B scripts/inspect_scene.py \
  --scene config/scenes/idc_prototype_v0.yaml --config config/workspace.yaml \
  --input-prefix results/workspace_n100_seed42 --axis-length 0.015 \
  --output results/s1_idc_workspace_n100.html
```

本轮文件已存在；重跑请改为 `s1_idc_scene_only_review02.html` / `s1_idc_workspace_n100_review02.html` 等新名字。

## 6. 测试、哈希及变更范围

新增 test_idc_prototype.py：15 项，覆盖 schema/引用、9 个 Port 的人工真值与开口通道尺寸、
完整 P22 矩阵、空心柜/等尺寸设备/层间距、canonical 重现、原 synthetic_dev 哈希、
Scene Only 与 100 点叠加，并在叠加测试中禁止调用采样器。

完整实际回归：**403 passed, 1 warning, 0 skipped**，11.72 s。
原有 388 项全部保留，未改断言或验收阈值。warning 与原回归相同。

```bash
source /opt/ros/humble/setup.bash
source /home/siyuen/Desktop/jetrover_sim_ws/install/setup.bash
cd /home/siyuen/Desktop/code/cable_robot/cable_vla_robot/workspace_analysis
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -B -m pytest \
  -q -rs -p no:cacheprovider tests
```

新场景哈希：

```text
Raw YAML: 50e1cf4f8478d7643d8e2f8ad7256db9c34c21df34f40c330f2be427f2107734
Canonical: d83e76d09f7e69fa0c01e753f0cefb3128dd520fb0292f1fd0b4838f66a5ab3b
```

synthetic_dev.yaml 保持原 Raw `cae02112d6a051b0bff3a6c7c8698ee1acc9a12e226fd75bd4df9b683ab5fba4`，
Canonical `0196cd18fc10ef9e4a23cc054e1aecfcb4e3509f220344ee538d152a5fb210a6`。

Git 范围仅新增 YAML、测试和本文；两份新 HTML 位于已忽略的 results/。
原有未跟踪 docs/develop/ 保留；没有修改 R1 核心、现有数据/报告/配置、官方模型或运行 Git 写操作。

## 7. 替换为实测参数

未来先测量/标定停靠 `^base T_cabinet`，再测机柜/设备外壳、面板平面及每个 Port Frame。
尺寸修改需同步展开的前面板分块，不能仅移动端口标签。对齐 Port 的局部操作轴，并独立标定
夹持 RJ45 后的 TCP、插头/线缆几何和允许插入深度。保留现有 synthetic 配置作为可重复夹具，
实测配置另起文件，用 measured provenance 并附测量依据。当前 schema 对 task/tool 的限制
不能靠改字符串绕过，需要后续明确的 schema 升级与验收。
修改后重新加载、核对图与尺寸并记录新的 Raw/Canonical 哈希；本轮不自动进入 C1。
