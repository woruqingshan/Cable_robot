# cable_vla_robot

ROS2 Humble overlay package for JetRover cable insertion/removal experiments.

This repository intentionally contains only our overlay code. The JetRover vendor workspace stays in `~/ros2_ws` on the robot and `/Volumes/jetrover_vendor` on the Mac.

## First phase scope

- Read JetRover servo states from the real bus-servo state service.
- Read current Cartesian pose from the vendor kinematics service.
- Provide a cautious teaching CLI scaffold.
- Keep fixed trajectory data in YAML, never hard-code taught pulse values.
- Do not move or release the arm automatically at node startup.

## JetRover build

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/cable_vla_ws
colcon build --symlink-install --packages-select cable_robot_control
source ~/cable_vla_ws/install/setup.bash
```

See [docs/jetrover_vendor_api.md](docs/jetrover_vendor_api.md) for confirmed vendor interfaces and the first robot test sequence.

## 项目上下文入口

- [研究目标](docs/PROJECT_CONTEXT.md)
- [开发环境](docs/DEVELOPMENT_ENVIRONMENT.md)
- [当前状态与文档分类](docs/DEVELOPMENT_STATUS.md)

模型适配专项分析见 [JetRover 本体](docs/model_adaptation/01_JetRover_Body_State_Action_Analysis.md) 和 [SmolVLA 适配](docs/model_adaptation/02_SmolVLA_JetRover_Adaptation_Analysis.md)。历史实验与运行说明保留原路径；当前结论以最新可追溯证据为准。
