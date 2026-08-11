# JetRover Vendor API Notes

These notes are based on files read from `/Volumes/jetrover_vendor`.

## Files inspected

- `bringup/launch/bringup.launch.py`
- `driver/controller/controller/odom_publisher_node.py`
- `driver/servo_controller/config/servo_controller.yaml`
- `driver/servo_controller/launch/servo_controller.launch.py`
- `driver/servo_controller/servo_controller/controller_manager.py`
- `driver/servo_controller/servo_controller/servo_controller.py`
- `driver/servo_controller/servo_controller/bus_servo_control.py`
- `driver/servo_controller/servo_controller/joint_position_controller.py`
- `driver/servo_controller_msgs/msg/ServoPosition.msg`
- `driver/servo_controller_msgs/msg/ServosPosition.msg`
- `driver/servo_controller_msgs/msg/ServoState.msg`
- `driver/servo_controller_msgs/msg/ServoStateList.msg`
- `driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py`
- `driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py`
- `driver/ros_robot_controller_msgs/msg/BusServoState.msg`
- `driver/ros_robot_controller_msgs/msg/GetBusServoCmd.msg`
- `driver/ros_robot_controller_msgs/msg/SetBusServoState.msg`
- `driver/ros_robot_controller_msgs/msg/ServoPosition.msg`
- `driver/ros_robot_controller_msgs/msg/ServosPosition.msg`
- `driver/ros_robot_controller_msgs/srv/GetBusServoState.srv`
- `driver/kinematics/kinematics/search_kinematics_solutions_node.py`
- `driver/kinematics/kinematics/kinematics_control.py`
- `driver/kinematics/launch/kinematics_node.launch.py`
- `driver/kinematics_msgs/srv/GetRobotPose.srv`
- `driver/kinematics_msgs/srv/SetRobotPose.srv`
- `driver/kinematics_msgs/srv/SetJointValue.srv`
- `example/example/rgbd_function/track_and_grab.py`
- `example/example/hand_trajectory/finger_control.py`

## Confirmed topics and services

### High-level joint pulse command

Topic: `/servo_controller`

Message: `servo_controller_msgs/msg/ServosPosition`

```text
float64 duration
string position_unit
servo_controller_msgs/ServoPosition[] position
```

`position_unit` must be `pulse` for raw taught waypoints. The vendor helper `servo_controller.bus_servo_control.set_servo_position()` publishes this topic.

### Controller-manager servo states

Topic: `/controller_manager/servo_states`

Message: `servo_controller_msgs/msg/ServoStateList`

The vendor `controller_manager.py` publishes this from its internal `ServoManager.get_position()` cache. That cache is updated when commands are sent, so this topic is useful for kinematics but is not treated here as authoritative hardware readback after torque release.

### Direct bus-servo state read

Service: `/ros_robot_controller/bus_servo/get_state`

Service type: `ros_robot_controller_msgs/srv/GetBusServoState`

Request contains `GetBusServoCmd[] cmd`. To read current pulse, send a command with:

```text
id: servo id
get_position: 1
```

The response state position is populated from `Board.bus_servo_read_position()`, which calls the hardware read command `0x05`.

### Direct bus-servo torque/state write

Topic: `/ros_robot_controller/bus_servo/set_state`

Message: `ros_robot_controller_msgs/msg/SetBusServoState`

For each `BusServoState`, vendor code checks two-element arrays where index `0` is an apply flag and index `1` is the value. Examples:

```text
present_id: [1, servo_id]
enable_torque: [1, 0]  # release/unload
enable_torque: [1, 1]  # load/enable
position: [1, pulse]   # target pulse
stop: [1, 0]           # stop this servo id; value is ignored
```

The vendor node batches `position` entries into one `bus_servo_set_position(duration, data)` call after processing all states.

### Kinematics

Services from node name `kinematics`:

- `/kinematics/get_current_pose`: `kinematics_msgs/srv/GetRobotPose`
- `/kinematics/set_pose_target`: `kinematics_msgs/srv/SetRobotPose`
- `/kinematics/set_joint_value_target`: `kinematics_msgs/srv/SetJointValue`
- `/kinematics/get_joint_range`: `kinematics_msgs/srv/GetJointRange`

`get_current_pose` computes FK from `/controller_manager/servo_states` for servo IDs 1 through 5.

## Servo IDs and limits

From `driver/servo_controller/config/servo_controller.yaml`:

- Arm joints: IDs `1, 2, 3, 4, 5`
- Gripper `r_joint`: ID `10`
- Ackermann steering `w_joint`: ID `9` only when `MACHINE_TYPE=JetRover_Acker`
- Configured pulse bounds for arm IDs 1 through 5 and gripper ID 10: `0..1000`
- Default `r_joint` init pulse is `700`

Examples use gripper pulses around `200` open and `600` closed in `track_and_grab.py`, but the first phase does not command gripper movement.

## Torque release and hold

Confirmed vendor implementation:

- Release/unload torque: publish `SetBusServoState` on `/ros_robot_controller/bus_servo/set_state` with `present_id=[1, id]` and `enable_torque=[1, 0]`.
- Load/enable torque: publish the same message with `enable_torque=[1, 1]`.
- Stop command: publish `stop=[1, 0]`; vendor collects the IDs and calls `Board.bus_servo_stop(ids)`.
- Read torque state: call `/ros_robot_controller/bus_servo/get_state` with `get_torque_state=1`.
- Read current pulse: call `/ros_robot_controller/bus_servo/get_state` with `get_position=1`.

Safe hold design for teaching:

1. Read current pulse for arm servo IDs 1 through 5 from `/ros_robot_controller/bus_servo/get_state`.
2. Publish the current pulse as the new target through `/ros_robot_controller/bus_servo/set_state` with `position=[1, pulse]` and a short duration.
3. Publish enable torque for the same IDs with `enable_torque=[1, 1]`.
4. Re-read pulse and torque state.

This avoids enabling torque against an old target held only in the vendor controller cache.

## First JetRover test sequence

Run these in order on the robot. Do not skip directly to trajectory execution.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/cable_vla_ws
colcon build --symlink-install --packages-select cable_robot_control
source ~/cable_vla_ws/install/setup.bash
```

Read-only checks:

```bash
ros2 launch cable_robot_control teach.launch.py readonly:=true
```

In the prompt:

- `P` prints current bus-servo states for IDs 1 through 5 and 10.
- `O` reads `/kinematics/get_current_pose`.
- `Q` exits.

Torque commands are blocked by default in read-only mode.

Future torque single-step test, after read-only output is reviewed:

```bash
ros2 launch cable_robot_control teach.launch.py readonly:=false allow_torque_commands:=true
```

Use only one command at a time:

- `R` release arm torque.
- Move the arm gently by hand.
- `H` hold current pulse and enable torque.
- `P` verify pulse and torque state.
- `Q` exits; the node attempts `hold_arm()` if it had released torque.
