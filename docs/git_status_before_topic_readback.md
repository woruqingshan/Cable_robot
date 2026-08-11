# Git Status Before Topic Readback Work

```text
## No commits yet on feat/fixed-trajectory
?? .gitignore
?? README.md
?? cable_robot_control/
?? docs/
?? waypoints/
```

Reason for this change: `/ros_robot_controller/bus_servo/get_state` exists but blocks during direct hardware read on the robot. `/controller_manager/servo_states` and `/kinematics/get_current_pose` were verified to return data, so teach readback should use the stable controller-manager topic by default.
