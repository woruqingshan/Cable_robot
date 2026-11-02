# Architecture

The fixed trajectory system is structured so future visual servoing, Cloud VLA, and semantic communication can reuse the same robot boundary.

```text
FixedTrajectoryPolicy / future CloudVLA policy
        ->
Task or Skill Command
        ->
Trajectory or Skill Executor
        ->
Safety Layer
        ->
ArmInterface
        ->
JetRover ROS2 vendor nodes
```

## Current first phase

`ArmInterface` owns all direct ROS calls:

- `/ros_robot_controller/bus_servo/get_state`
- `/ros_robot_controller/bus_servo/set_state`
- `/servo_controller`
- `/kinematics/get_current_pose`
- `/kinematics/set_pose_target`

`TeachTrajectoryNode` only calls `ArmInterface`. It starts in read-only mode and never releases torque automatically.

`FixedTrajectoryExecutor` is intentionally a loader and dry-run executor in this phase. Real sequence execution remains gated by an explicit `execute` flag.

## Safety split

Hard safety blocks invalid data: illegal servo IDs, pulse values outside vendor limits, invalid duration, missing services, unknown commands, and IK failure.

Soft safety warns on large motion deltas and other risky but potentially valid taught moves.
