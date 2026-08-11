"""Interactive joint teaching scaffold for JetRover cable trajectories."""

import os
import re
import time
from typing import Dict

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from cable_robot_control.arm_interface import ArmInterface, ServoReadback
from cable_robot_control.config import limits_from_config, load_yaml
from cable_robot_control.experiment_logger import ExperimentLogger


def param_bool(value) -> bool:
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def pose_to_dict(pose):
    return {
        'position': {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
        },
        'orientation': {
            'x': float(pose.orientation.x),
            'y': float(pose.orientation.y),
            'z': float(pose.orientation.z),
            'w': float(pose.orientation.w),
        },
    }


class TeachTrajectoryNode(Node):
    def __init__(self):
        super().__init__('teach_trajectory')
        self.declare_parameter('readonly', True)
        self.declare_parameter('allow_torque_commands', False)
        self.declare_parameter('limits_file', '')
        self.declare_parameter('waypoint_file', 'waypoints/taught_waypoints.yaml')
        self.declare_parameter('log_dir', 'episodes')
        self.declare_parameter('jog_step', 10)
        self.declare_parameter('jog_duration', 0.4)

        limits_file = self.get_parameter('limits_file').value
        if not limits_file:
            limits_file = os.path.join(
                get_package_share_directory('cable_robot_control'),
                'config',
                'arm_limits.yaml',
            )
        self.config = load_yaml(limits_file)
        self.limits = limits_from_config(self.config)
        topics = self.config.get('topics', {})
        services = self.config.get('services', {})
        self.readonly = param_bool(self.get_parameter('readonly').value)
        self.allow_torque_commands = param_bool(self.get_parameter('allow_torque_commands').value)
        self.waypoint_file = str(self.get_parameter('waypoint_file').value)
        self.jog_step = int(self.get_parameter('jog_step').value)
        self.jog_duration = float(self.get_parameter('jog_duration').value)
        self.logger = ExperimentLogger(str(self.get_parameter('log_dir').value))
        self.released = False
        self.current_pulse = None
        self.current_pose_xyz = None

        self.arm = ArmInterface(
            self,
            self.limits,
            servo_controller_topic=topics.get('servo_controller', '/servo_controller'),
            bus_servo_get_state_service=services.get(
                'bus_servo_get_state',
                '/ros_robot_controller/bus_servo/get_state',
            ),
            bus_servo_set_state_topic=topics.get(
                'bus_servo_set_state',
                '/ros_robot_controller/bus_servo/set_state',
            ),
            servo_states_topic=topics.get('servo_states', '/controller_manager/servo_states'),
            get_current_pose_service=services.get('get_current_pose', '/kinematics/get_current_pose'),
            set_pose_target_service=services.get('set_pose_target', '/kinematics/set_pose_target'),
        )

    def run_cli(self) -> None:
        self.get_logger().info('Teach CLI started. Startup is read-only and does not move or release the arm.')
        self.get_logger().info(f'readonly={self.readonly}, allow_torque_commands={self.allow_torque_commands}')
        self.get_logger().info(
            'Commands: MOVE name, SYNC name, SET id pulse..., P=internal, CACHE=controller cache, '
            'O=pose, IK x y z pitch, CART dx dy dz pitch, 1+/1-..5+/5-=jog arm, '
            'G+/G-=jog gripper, STEP n, DUR n, S=save, R/H=experimental torque, Q=quit'
        )
        self.get_logger().warning('Jog and save are disabled until MOVE, SET, or SYNC initializes current_pulse.')
        while rclpy.ok():
            command = input('teach> ').strip().upper()
            if command == 'P':
                self.print_servo_states()
            elif command == 'CACHE':
                self.print_controller_cache()
            elif command == 'O':
                self.print_pose()
            elif command == 'R':
                self.release()
            elif command in ('H', 'L'):
                self.hold()
            elif command == 'S':
                self.save_waypoint()
            elif command.startswith('MOVE '):
                self.move_waypoint(command)
            elif command.startswith('LOAD '):
                self.get_logger().warning('LOAD is deprecated; treating it as MOVE.')
                self.move_waypoint(command.replace('LOAD ', 'MOVE ', 1))
            elif command.startswith('SYNC '):
                self.load_waypoint(command, move=False)
            elif command.startswith('SET '):
                self.set_absolute_pulses(command)
            elif command.startswith('IK '):
                self.move_ik_absolute(command)
            elif command.startswith('CART '):
                self.move_cartesian_delta(command)
            elif command.startswith('STEP '):
                self.set_jog_step(command)
            elif command.startswith('DUR '):
                self.set_jog_duration(command)
            elif self.try_jog(command):
                continue
            elif command == 'Q':
                break
            elif not command:
                continue
            else:
                self.get_logger().warning(f'Unknown command: {command}')
        self.safe_exit()

    def print_servo_states(self) -> None:
        if self.current_pulse is None:
            self.get_logger().warning('Internal current_pulse is UNINITIALIZED. Run MOVE, SET, or SYNC before jog/save.')
        else:
            self.get_logger().info(f'internal current_pulse={self.current_pulse}')
        if self.current_pose_xyz is None:
            self.get_logger().warning('Internal current_pose_xyz is UNINITIALIZED. Run MOVE/SYNC with pose, O, or IK before CART.')
        else:
            self.get_logger().info(f'internal current_pose_xyz={self.current_pose_xyz}')

    def print_controller_cache(self) -> Dict[int, ServoReadback]:
        states = self.arm.get_servo_states(self.limits.arm_servo_ids + [self.limits.gripper_servo_id])
        for servo_id in sorted(states):
            state = states[servo_id]
            self.get_logger().info(
                f'controller_cache id={servo_id} pulse={state.position} torque={state.torque_enabled} '
                f'voltage={state.voltage_mv} temp={state.temperature_c}'
        )
        self.logger.record(stage='read_servo_states', current_joint_pulse=self._pulses_only(states), success=True)
        return states

    def print_pose(self):
        pose = self.arm.get_current_pose()
        pose_data = pose_to_dict(pose)
        self.current_pose_xyz = [
            pose_data['position']['x'],
            pose_data['position']['y'],
            pose_data['position']['z'],
        ]
        self.get_logger().info(f'pose={pose_data}')
        self.logger.record(stage='read_current_pose', actual_pose=pose_data, success=True)
        return pose

    def release(self) -> None:
        if not self._torque_allowed('release'):
            return
        self.arm.release_arm()
        self.released = True
        self.logger.record(stage='release_arm', success=True)
        self.get_logger().warning('Arm torque release command published.')

    def hold(self) -> None:
        if not self._torque_allowed('hold'):
            return
        states = self.arm.get_servo_states(self.limits.arm_servo_ids)
        pulses = self._pulses_only(states)
        held = self.arm.hold_arm(pulses)
        self.released = False
        self.logger.record(stage='hold_arm', current_joint_pulse=held, success=True)
        self.get_logger().warning(f'Hold command published at current pulses: {held}')

    def try_jog(self, command: str) -> bool:
        match = re.fullmatch(r'([1-5]|G)([+-])', command)
        if not match:
            return False
        target, direction = match.groups()
        servo_id = self.limits.gripper_servo_id if target == 'G' else int(target)
        delta = self.jog_step if direction == '+' else -self.jog_step
        self.jog_servo(servo_id, delta)
        return True

    def jog_servo(self, servo_id: int, delta: int) -> None:
        if self.readonly:
            self.get_logger().error('Jog blocked: readonly:=true. Restart with readonly:=false to move.')
            return
        if self.current_pulse is None:
            self.get_logger().error('Jog blocked: run MOVE <waypoint>, SET ..., or SYNC <waypoint> first.')
            return
        current = self.current_pulse[servo_id]
        target = max(self.limits.pulse_min, min(self.limits.pulse_max, int(current) + int(delta)))
        self.arm.move_joint({servo_id: target}, self.jog_duration)
        self.current_pulse[servo_id] = target
        self.current_pose_xyz = None
        self.logger.record(
            stage='jog',
            waypoint=f'servo_{servo_id}',
            target_joint_pulse={servo_id: target},
            current_joint_pulse={servo_id: int(current)},
            duration=self.jog_duration,
            success=True,
        )
        self.get_logger().info(
            f'Jog servo {servo_id}: {current} -> {target} '
            f'(step={self.jog_step}, duration={self.jog_duration:.2f}s)'
        )

    def move_waypoint(self, command: str) -> None:
        try:
            name = command.split(maxsplit=1)[1].strip()
        except IndexError:
            self.get_logger().error('Usage: MOVE CABLE_HOLD_READY')
            return
        try:
            waypoint = self._read_waypoint(name)
            pulses = self._waypoint_all_pulses(waypoint)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        if self.readonly:
            self.get_logger().error('MOVE blocked: readonly:=true. Restart with readonly:=false to move.')
            return
        duration = float(waypoint.get('duration', self.config.get('arm', {}).get('default_move_duration', 2.0)))
        if not self._publish_full_pulse_target(pulses, duration):
            return
        self.get_logger().warning(f'Moving to waypoint {name}; waiting {duration:.2f}s before enabling jog.')
        time.sleep(duration)
        self.current_pulse = dict(pulses)
        self._set_current_pose_from_waypoint(waypoint)
        self.logger.record(
            stage='move_waypoint',
            waypoint=name,
            target_joint_pulse=self.current_pulse,
            duration=duration,
            success=True,
        )
        self.get_logger().info(f'Moved to waypoint {name}: current_pulse={self.current_pulse}')

    def load_waypoint(self, command: str, move: bool) -> None:
        try:
            name = command.split(maxsplit=1)[1].strip()
        except IndexError:
            self.get_logger().error('Usage: SYNC CABLE_HOLD_READY')
            return
        try:
            waypoint = self._read_waypoint(name)
            pulses = self._waypoint_all_pulses(waypoint)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        self.current_pulse = dict(pulses)
        self._set_current_pose_from_waypoint(waypoint)
        self.logger.record(
            stage='sync_waypoint',
            waypoint=name,
            target_joint_pulse=self.current_pulse,
            duration=0.0,
            success=True,
        )
        self.get_logger().warning('SYNC does not move the robot; use only if the robot is already at this waypoint.')
        self.get_logger().info(f'Synced internal pulse from waypoint {name}: current_pulse={self.current_pulse}')

    def set_absolute_pulses(self, command: str) -> None:
        try:
            pulses = self._parse_set_command(command)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        if self.readonly:
            self.get_logger().error('SET blocked: readonly:=true. Restart with readonly:=false to move.')
            return
        if not self._publish_full_pulse_target(pulses, self.jog_duration):
            return
        self.get_logger().warning(f'SET target published; waiting {self.jog_duration:.2f}s before enabling jog.')
        time.sleep(self.jog_duration)
        self.current_pulse = dict(pulses)
        self.current_pose_xyz = None
        self.logger.record(
            stage='set_absolute_pulses',
            target_joint_pulse=self.current_pulse,
            duration=self.jog_duration,
            success=True,
        )
        self.get_logger().info(f'SET complete: current_pulse={self.current_pulse}')

    def move_ik_absolute(self, command: str) -> None:
        try:
            x, y, z, pitch = self._parse_four_floats(command, 'IK x y z pitch')
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        self._move_to_cartesian_target([x, y, z], pitch, stage='ik_absolute')

    def move_cartesian_delta(self, command: str) -> None:
        try:
            dx, dy, dz, pitch = self._parse_four_floats(command, 'CART dx dy dz pitch')
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        if self.current_pose_xyz is None:
            self.get_logger().error('CART blocked: current_pose_xyz is unknown. Run MOVE <waypoint> with pose, SYNC <waypoint>, O, or IK first.')
            return
        target = [
            self.current_pose_xyz[0] + dx,
            self.current_pose_xyz[1] + dy,
            self.current_pose_xyz[2] + dz,
        ]
        self._move_to_cartesian_target(target, pitch, stage='cartesian_delta')

    def _move_to_cartesian_target(self, target_xyz, pitch: float, stage: str) -> None:
        if self.readonly:
            self.get_logger().error(f'{stage} blocked: readonly:=true. Restart with readonly:=false to move.')
            return
        if self.current_pulse is None:
            self.get_logger().error(f'{stage} blocked: run MOVE <waypoint>, SET ..., or SYNC <waypoint> first.')
            return
        try:
            response = self.arm.solve_ik(target_xyz, pitch)
        except Exception as exc:
            self.get_logger().error(f'IK failed; robot was not moved: {exc}')
            self.logger.record(stage=stage, target_pose={'position': target_xyz, 'pitch': pitch}, success=False, error_reason=str(exc))
            return
        arm_pulses = {
            servo_id: int(response.pulse[index])
            for index, servo_id in enumerate(self.limits.arm_servo_ids)
        }
        all_pulses = dict(self.current_pulse)
        all_pulses.update(arm_pulses)
        if not self._publish_full_pulse_target(all_pulses, self.jog_duration):
            return
        self.get_logger().warning(
            f'{stage} target published xyz={target_xyz}, pitch={pitch}; '
            f'waiting {self.jog_duration:.2f}s before enabling jog.'
        )
        time.sleep(self.jog_duration)
        previous_pulse = dict(self.current_pulse)
        self.current_pulse = all_pulses
        self.current_pose_xyz = [float(value) for value in target_xyz]
        self.logger.record(
            stage=stage,
            target_joint_pulse=self.current_pulse,
            current_joint_pulse=previous_pulse,
            target_pose={'position': {'x': target_xyz[0], 'y': target_xyz[1], 'z': target_xyz[2]}, 'pitch': pitch},
            duration=self.jog_duration,
            success=True,
        )
        self.get_logger().info(
            f'{stage} complete: arm_pulses={arm_pulses}, gripper={self.current_pulse[self.limits.gripper_servo_id]}, '
            f'current_pose_xyz={self.current_pose_xyz}'
        )

    def set_jog_step(self, command: str) -> None:
        try:
            step = int(command.split(maxsplit=1)[1])
        except (IndexError, ValueError):
            self.get_logger().error('Usage: STEP 10')
            return
        if step <= 0 or step > 200:
            self.get_logger().error('Jog step must be in 1..200 pulse.')
            return
        self.jog_step = step
        self.get_logger().info(f'Jog step set to {self.jog_step} pulse.')

    def set_jog_duration(self, command: str) -> None:
        try:
            duration = float(command.split(maxsplit=1)[1])
        except (IndexError, ValueError):
            self.get_logger().error('Usage: DUR 0.4')
            return
        if duration < 0.02 or duration > 5.0:
            self.get_logger().error('Jog duration must be in 0.02..5.0 seconds.')
            return
        self.jog_duration = duration
        self.get_logger().info(f'Jog duration set to {self.jog_duration:.2f}s.')

    def save_waypoint(self) -> None:
        if self.current_pulse is None:
            self.get_logger().error('Save blocked: run MOVE <waypoint>, SET ..., or SYNC <waypoint> first.')
            return
        name = input('waypoint name> ').strip()
        if not name:
            self.get_logger().error('Waypoint name is required.')
            return
        all_pulses = dict(self.current_pulse)
        pulses = {servo_id: all_pulses[servo_id] for servo_id in self.limits.arm_servo_ids}
        gripper_pulse = all_pulses[self.limits.gripper_servo_id]
        pose = self.arm.get_current_pose()
        pose_data = pose_to_dict(pose)
        self.current_pose_xyz = [
            pose_data['position']['x'],
            pose_data['position']['y'],
            pose_data['position']['z'],
        ]

        waypoint = {
            'joint_pulse': {int(servo_id): int(pulse) for servo_id, pulse in pulses.items()},
            'gripper_pulse': {int(self.limits.gripper_servo_id): int(gripper_pulse)},
            'pose': pose_data,
            'timestamp': time.time(),
            'duration': float(self.config.get('arm', {}).get('default_move_duration', 2.0)),
            'held_after_save': False,
        }
        self._write_waypoint(name, waypoint)
        self.logger.record(
            stage='save_waypoint',
            waypoint=name,
            current_joint_pulse=all_pulses,
            actual_pose=pose_data,
            success=True,
        )
        self.get_logger().info(
            f'Saved waypoint {name}: pulses={pulses}, gripper={gripper_pulse}; no torque command was sent'
        )

    def safe_exit(self) -> None:
        if self.released and self._torque_allowed('exit hold', quiet=True):
            try:
                self.arm.hold_arm()
                self.get_logger().warning('Exit safety: hold_arm() was published because this node released torque.')
            except Exception as exc:
                self.get_logger().error(f'Exit safety hold failed: {exc}')
        self.get_logger().info('Teach CLI stopped.')

    def _torque_allowed(self, action: str, quiet: bool = False) -> bool:
        allowed = (not self.readonly) and self.allow_torque_commands
        if not allowed and not quiet:
            self.get_logger().warning(
                f'Blocked {action}: launch with readonly:=false allow_torque_commands:=true to enable torque commands.'
            )
        return allowed

    def _write_waypoint(self, name: str, waypoint: Dict) -> None:
        path = os.path.abspath(os.path.expanduser(self.waypoint_file))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if os.path.exists(path):
            with open(path, 'r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}
        else:
            data = {}
        data.setdefault('waypoints', {})
        data['waypoints'][name] = waypoint
        with open(path, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)

    def _read_waypoint(self, name: str) -> Dict:
        path = os.path.abspath(os.path.expanduser(self.waypoint_file))
        if not os.path.exists(path):
            raise RuntimeError(f'waypoint file does not exist: {path}')
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        waypoints = data.get('waypoints', {}) or {}
        if name not in waypoints:
            matches = [key for key in waypoints if key.upper() == name.upper()]
            if len(matches) == 1:
                return waypoints[matches[0]]
            raise RuntimeError(f'waypoint {name} not found in {path}')
        return waypoints[name]

    def _waypoint_all_pulses(self, waypoint: Dict) -> Dict[int, int]:
        joint_pulse = waypoint.get('joint_pulse') or {}
        gripper_pulse = waypoint.get('gripper_pulse') or {}
        pulses = {int(servo_id): int(pulse) for servo_id, pulse in joint_pulse.items()}
        pulses.update({int(servo_id): int(pulse) for servo_id, pulse in gripper_pulse.items()})
        required = set(self.limits.arm_servo_ids + [self.limits.gripper_servo_id])
        missing = sorted(required - set(pulses))
        if missing:
            raise RuntimeError(f'waypoint missing pulse for servo id(s): {missing}')
        return pulses

    def _parse_set_command(self, command: str) -> Dict[int, int]:
        parts = command.split()[1:]
        if len(parts) % 2 != 0:
            raise RuntimeError('Usage: SET 1 500 2 765 3 15 4 220 5 500 10 500')
        pulses = {}
        try:
            for index in range(0, len(parts), 2):
                servo_id = int(parts[index])
                pulse = int(parts[index + 1])
                pulses[servo_id] = pulse
        except ValueError as exc:
            raise RuntimeError('SET expects numeric id/pulse pairs') from exc
        required = set(self.limits.arm_servo_ids + [self.limits.gripper_servo_id])
        missing = sorted(required - set(pulses))
        if missing:
            raise RuntimeError(f'SET requires complete 1-5 and gripper pulses; missing id(s): {missing}')
        extra = sorted(set(pulses) - required)
        if extra:
            raise RuntimeError(f'SET contains illegal servo id(s): {extra}')
        for servo_id, pulse in pulses.items():
            if pulse < self.limits.pulse_min or pulse > self.limits.pulse_max:
                raise RuntimeError(f'servo {servo_id} pulse {pulse} outside {self.limits.pulse_min}..{self.limits.pulse_max}')
        return pulses

    def _parse_four_floats(self, command: str, usage: str):
        parts = command.split()[1:]
        if len(parts) != 4:
            raise RuntimeError(f'Usage: {usage}')
        try:
            return [float(value) for value in parts]
        except ValueError as exc:
            raise RuntimeError(f'{usage} expects numeric values') from exc

    def _set_current_pose_from_waypoint(self, waypoint: Dict) -> None:
        pose = waypoint.get('pose') or {}
        position = pose.get('position') or {}
        try:
            self.current_pose_xyz = [
                float(position['x']),
                float(position['y']),
                float(position['z']),
            ]
        except (KeyError, TypeError, ValueError):
            self.current_pose_xyz = None

    def _publish_full_pulse_target(self, pulses: Dict[int, int], duration: float) -> bool:
        required = set(self.limits.arm_servo_ids + [self.limits.gripper_servo_id])
        missing = sorted(required - set(pulses))
        if missing:
            self.get_logger().error(f'Cannot move: missing pulse for servo id(s): {missing}')
            return False
        try:
            self.arm.move_joint({servo_id: pulses[servo_id] for servo_id in sorted(required)}, duration)
        except Exception as exc:
            self.get_logger().error(f'Move publish failed; internal current_pulse was not updated: {exc}')
            return False
        return True

    def _pulses_only(self, states: Dict[int, ServoReadback]) -> Dict[int, int]:
        pulses = {
            int(servo_id): int(readback.position)
            for servo_id, readback in states.items()
            if readback.position is not None
        }
        missing = [servo_id for servo_id in states if servo_id not in pulses]
        if missing:
            raise RuntimeError(f'missing pulse readback for servo id(s): {missing}')
        return pulses


def main():
    rclpy.init()
    node = TeachTrajectoryNode()
    try:
        node.run_cli()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
