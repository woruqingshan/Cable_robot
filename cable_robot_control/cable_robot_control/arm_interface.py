"""JetRover arm interface shared by fixed trajectories and future VLA skills."""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

import rclpy
from rclpy.node import Node

from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from ros_robot_controller_msgs.msg import BusServoState, GetBusServoCmd, SetBusServoState
from ros_robot_controller_msgs.srv import GetBusServoState
from servo_controller_msgs.msg import ServoPosition, ServoStateList, ServosPosition

from cable_robot_control.safety import ArmLimits, validate_duration, validate_pulses, validate_servo_ids


@dataclass
class ServoReadback:
    id: int
    position: Optional[int] = None
    torque_enabled: Optional[bool] = None
    voltage_mv: Optional[int] = None
    temperature_c: Optional[int] = None


class ArmInterface:
    def __init__(
        self,
        node: Node,
        limits: ArmLimits,
        servo_controller_topic: str = '/servo_controller',
        bus_servo_get_state_service: str = '/ros_robot_controller/bus_servo/get_state',
        bus_servo_set_state_topic: str = '/ros_robot_controller/bus_servo/set_state',
        servo_states_topic: str = '/controller_manager/servo_states',
        get_current_pose_service: str = '/kinematics/get_current_pose',
        set_pose_target_service: str = '/kinematics/set_pose_target',
        service_timeout_sec: float = 2.0,
        topic_timeout_sec: float = 2.0,
    ):
        self.node = node
        self.limits = limits
        self.service_timeout_sec = service_timeout_sec
        self.topic_timeout_sec = topic_timeout_sec
        self._last_servo_states_msg: Optional[ServoStateList] = None
        self.servo_position_pub = node.create_publisher(ServosPosition, servo_controller_topic, 1)
        self.bus_state_pub = node.create_publisher(SetBusServoState, bus_servo_set_state_topic, 1)
        self.servo_states_sub = node.create_subscription(
            ServoStateList,
            servo_states_topic,
            self._servo_states_callback,
            1,
        )
        self.bus_state_client = node.create_client(GetBusServoState, bus_servo_get_state_service)
        self.current_pose_client = node.create_client(GetRobotPose, get_current_pose_service)
        self.pose_target_client = node.create_client(SetRobotPose, set_pose_target_service)

    def wait_for_read_services(self) -> bool:
        pose_ready = self.current_pose_client.wait_for_service(timeout_sec=self.service_timeout_sec)
        return bool(pose_ready)

    def get_servo_states(
        self,
        servo_ids: Optional[Iterable[int]] = None,
        read_torque: bool = False,
        read_voltage: bool = False,
        read_temperature: bool = False,
    ) -> Dict[int, ServoReadback]:
        if read_torque or read_voltage or read_temperature:
            self.node.get_logger().warning(
                'servo state topic provides pulse only; torque/voltage/temperature are unavailable'
            )
        return self.get_servo_states_from_topic(servo_ids)

    def get_servo_states_from_topic(
        self,
        servo_ids: Optional[Iterable[int]] = None,
    ) -> Dict[int, ServoReadback]:
        ids = list(servo_ids or self.limits.arm_servo_ids)
        validate_servo_ids(ids, self.limits)
        msg = self._wait_for_servo_states_msg()
        result: Dict[int, ServoReadback] = {}
        for state in msg.servo_state:
            if int(state.id) in ids:
                result[int(state.id)] = ServoReadback(id=int(state.id), position=int(state.position))

        missing = [servo_id for servo_id in ids if servo_id not in result]
        if missing:
            raise RuntimeError(f'servo_states topic missing servo id(s): {missing}')
        return result

    def get_servo_states_from_bus_service(
        self,
        servo_ids: Optional[Iterable[int]] = None,
        read_torque: bool = True,
        read_voltage: bool = False,
        read_temperature: bool = False,
    ) -> Dict[int, ServoReadback]:
        ids = list(servo_ids or self.limits.arm_servo_ids)
        validate_servo_ids(ids, self.limits)
        request = GetBusServoState.Request()
        for servo_id in ids:
            cmd = GetBusServoCmd()
            cmd.id = int(servo_id)
            cmd.get_position = 1
            cmd.get_torque_state = 1 if read_torque else 0
            cmd.get_voltage = 1 if read_voltage else 0
            cmd.get_temperature = 1 if read_temperature else 0
            request.cmd.append(cmd)

        response = self._call_service(self.bus_state_client, request, 'bus servo get_state')
        if not response.success:
            raise RuntimeError('bus servo get_state returned success=false')

        result: Dict[int, ServoReadback] = {}
        for servo_id, state in zip(ids, response.state):
            readback = ServoReadback(id=servo_id)
            if state.position:
                readback.position = int(state.position[0])
            if state.enable_torque:
                readback.torque_enabled = bool(state.enable_torque[0])
            if state.voltage:
                readback.voltage_mv = int(state.voltage[0])
            if state.temperature:
                readback.temperature_c = int(state.temperature[0])
            result[servo_id] = readback
        return result

    def get_current_pose(self):
        response = self._call_service(
            self.current_pose_client,
            GetRobotPose.Request(),
            'kinematics get_current_pose',
        )
        if not response.success:
            raise RuntimeError('get_current_pose returned success=false')
        if not response.solution:
            raise RuntimeError('get_current_pose returned solution=false')
        return response.pose

    def release_arm(self) -> None:
        self.stop_arm()
        msg = SetBusServoState()
        msg.duration = 0.02
        for servo_id in self.limits.arm_servo_ids:
            state = BusServoState()
            state.present_id = [1, int(servo_id)]
            state.enable_torque = [1, 0]
            msg.state.append(state)
        self.bus_state_pub.publish(msg)

    def hold_arm(self, current_pulses: Optional[Dict[int, int]] = None) -> Dict[int, int]:
        pulses = current_pulses or {
            servo_id: readback.position
            for servo_id, readback in self.get_servo_states(self.limits.arm_servo_ids).items()
            if readback.position is not None
        }
        validate_pulses(pulses, self.limits, include_gripper=False)
        missing = [servo_id for servo_id in self.limits.arm_servo_ids if servo_id not in pulses]
        if missing:
            raise RuntimeError(f'missing current pulse for servo id(s): {missing}')

        position_msg = SetBusServoState()
        position_msg.duration = 0.08
        for servo_id in self.limits.arm_servo_ids:
            state = BusServoState()
            state.present_id = [1, int(servo_id)]
            state.position = [1, int(pulses[servo_id])]
            position_msg.state.append(state)
        self.bus_state_pub.publish(position_msg)

        enable_msg = SetBusServoState()
        enable_msg.duration = 0.02
        for servo_id in self.limits.arm_servo_ids:
            state = BusServoState()
            state.present_id = [1, int(servo_id)]
            state.enable_torque = [1, 1]
            enable_msg.state.append(state)
        self.bus_state_pub.publish(enable_msg)
        return pulses

    def stop_arm(self) -> None:
        msg = SetBusServoState()
        msg.duration = 0.02
        for servo_id in self.limits.arm_servo_ids:
            state = BusServoState()
            state.present_id = [1, int(servo_id)]
            state.stop = [1, 0]
            msg.state.append(state)
        self.bus_state_pub.publish(msg)

    def move_joint(self, pulses: Dict[int, int], duration: float) -> None:
        validate_duration(duration, self.limits)
        validate_pulses(pulses, self.limits)
        self._wait_for_publisher_subscribers(self.servo_position_pub, '/servo_controller')
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = 'pulse'
        for servo_id, pulse in pulses.items():
            position = ServoPosition()
            position.id = int(servo_id)
            position.position = float(pulse)
            msg.position.append(position)
        self.servo_position_pub.publish(msg)

    def set_gripper(self, pulse: int, duration: float) -> None:
        self.move_joint({self.limits.gripper_servo_id: int(pulse)}, duration)

    def solve_ik(
        self,
        position: List[float],
        pitch: float,
        pitch_range: Optional[List[float]] = None,
        resolution: float = 1.0,
    ):
        request = SetRobotPose.Request()
        request.position = [float(value) for value in position]
        request.pitch = float(pitch)
        request.pitch_range = [float(value) for value in (pitch_range or [-180.0, 180.0])]
        request.resolution = float(resolution)
        response = self._call_service(self.pose_target_client, request, 'kinematics set_pose_target')
        if not response.success or not response.pulse:
            raise RuntimeError('set_pose_target failed to find an IK pulse solution')
        return response

    def move_cartesian(self, *args, **kwargs) -> None:
        raise NotImplementedError('Cartesian motion is reserved for a later phase')

    def _call_service(self, client, request, label: str):
        if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise RuntimeError(f'service unavailable: {label}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.service_timeout_sec)
        if not future.done():
            raise TimeoutError(f'timeout waiting for {label}')
        result = future.result()
        if result is None:
            raise RuntimeError(f'{label} returned no result')
        return result

    def _wait_for_publisher_subscribers(self, publisher, label: str) -> None:
        end_time = self.node.get_clock().now().nanoseconds / 1e9 + self.topic_timeout_sec
        while rclpy.ok() and publisher.get_subscription_count() < 1:
            if self.node.get_clock().now().nanoseconds / 1e9 >= end_time:
                raise TimeoutError(f'timeout waiting for subscriber on {label}')
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _servo_states_callback(self, msg: ServoStateList) -> None:
        self._last_servo_states_msg = msg

    def _wait_for_servo_states_msg(self) -> ServoStateList:
        self._last_servo_states_msg = None
        end_time = self.node.get_clock().now().nanoseconds / 1e9 + self.topic_timeout_sec
        while rclpy.ok() and self._last_servo_states_msg is None:
            if self.node.get_clock().now().nanoseconds / 1e9 >= end_time:
                raise TimeoutError('timeout waiting for /controller_manager/servo_states')
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if self._last_servo_states_msg is None:
            raise RuntimeError('servo_states topic returned no data')
        return self._last_servo_states_msg
