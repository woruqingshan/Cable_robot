# C2 Geometry Coverage — PARTIAL

声明范围：bare_gripper_model，固定底盘，官方 URDF 中已有几何。24 个碰撞实体、4 个无实体参考 Frame、1 个物理部件缺失（LiDAR）。这是 Link 数量清点，不是体积覆盖率或硬件覆盖认证。

| Link | 碰撞几何数 | 分类 |
|---|---:|---|
| lidar_frame | 0 | reference_frame |
| base_footprint | 0 | reference_frame |
| base_link | 1 | modeled_geometry |
| wheel_left_front_link | 1 | modeled_geometry |
| wheel_left_back_link | 1 | modeled_geometry |
| wheel_right_front_link | 1 | modeled_geometry |
| wheel_right_back_link | 1 | modeled_geometry |
| lidar_link | 0 | missing_physical_geometry |
| imu_link | 1 | modeled_geometry |
| camera_connect_link | 1 | modeled_geometry |
| depth_cam_link | 1 | modeled_geometry |
| depth_cam_frame | 0 | reference_frame |
| back_shell_black_link | 1 | modeled_geometry |
| back_shell_green_link | 1 | modeled_geometry |
| link1 | 1 | modeled_geometry |
| servo_link1 | 1 | modeled_geometry |
| link2 | 1 | modeled_geometry |
| link3 | 1 | modeled_geometry |
| link4 | 1 | modeled_geometry |
| servo_link2 | 1 | modeled_geometry |
| link5 | 1 | modeled_geometry |
| gripper_link | 1 | modeled_geometry |
| end_effector_link | 0 | reference_frame |
| r_link | 1 | modeled_geometry |
| l_link | 1 | modeled_geometry |
| l_in_link | 1 | modeled_geometry |
| l_out_link | 1 | modeled_geometry |
| r_in_link | 1 | modeled_geometry |
| r_out_link | 1 | modeled_geometry |

Base/四轮/后壳/五自由度臂/舵机壳/夹爪/腕相机均由上述命名 Link 的现有 Mesh 表示；不推断实物误差。

## LiDAR 来源与未完成项

官方 lidar.urdf.xacro 的 collision 被注释，视觉 lidar_link.stl 可读。程序验证其二进制 STL 顶点，视觉 origin 为 identity。

```json
{
  "resource": "/home/siyuen/Desktop/jetrover_sim_ws/install/jetrover_description/share/jetrover_description/meshes/common/lidar_link.stl",
  "resource_sha256": "1cf6fef70b8fb6105a9fd02c0451b5c983e50e3f47fbf7d3d8073e1edac6300c",
  "triangles": 2300,
  "visual_origin_xml": "<origin rpy=\"0 0 0\" xyz=\"0 0 0\" />",
  "mesh_local_bounds_m": [
    [
      -0.04402979835867882,
      -0.03994760662317276,
      -0.0385047048330307
    ],
    [
      0.04402979463338852,
      0.040093500167131424,
      0.012595001608133316
    ]
  ],
  "overlay_implemented": false,
  "physical_envelope_validated": false,
  "reason": "visual mesh bounds establish a model-only candidate; hardware envelope/mount tolerance unmeasured"
}
```

这些是 Mesh 局部边界，不是 Base 坐标；实际挂载位姿应从 lidar_joint 与完整 Frame 链得到。本轮没有将这个视觉边界加入 FCL，不宣称覆盖真实 A1 扫描器、线缆或安装误差。Overlay 的来源/外包络需要进一步审核；缺乏实物包络证据时保持 UNVERIFIED。

## 工具/线缆

RJ45 插头、TCP、柔性线缆都未建模。bare-gripper 的模型查询不能升级为 gripper-with-RJ45；该 scope 请求被拒绝。没有编造尺寸/抓持位姿。Self/Environment 间隙和物理夹爪开度均未验证。
