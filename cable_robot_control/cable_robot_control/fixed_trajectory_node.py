"""Fixed trajectory node for insert and pull-out YAML sequences."""

import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from cable_robot_control.arm_interface import ArmInterface
from cable_robot_control.config import limits_from_config, load_yaml
from cable_robot_control.experiment_logger import ExperimentLogger
from cable_robot_control.trajectory_executor import FixedTrajectoryExecutor


def param_bool(value) -> bool:
    if isinstance(value, str):
        return value.lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class FixedTrajectoryNode(Node):
    def __init__(self, default_trajectory_filename=None):
        super().__init__('fixed_trajectory')
        self.declare_parameter('trajectory_file', '')
        self.declare_parameter('limits_file', '')
        self.declare_parameter('execute', False)
        self.declare_parameter('log_dir', 'episodes')
        self.declare_parameter(
            'cartesian_freeze_servo_ids',
            '5',
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )

        share_dir = get_package_share_directory('cable_robot_control')
        limits_file = self.get_parameter('limits_file').value or os.path.join(
            share_dir,
            'config',
            'arm_limits.yaml',
        )
        trajectory_file = self.get_parameter('trajectory_file').value
        if not trajectory_file:
            trajectory_file = os.path.join(
                share_dir,
                'config',
                default_trajectory_filename or 'fixed_trajectory.yaml',
            )

        self.config = load_yaml(limits_file)
        self.trajectory = load_yaml(trajectory_file)
        self.limits = limits_from_config(self.config)
        self.trajectory_file = trajectory_file
        self.episode_logger = ExperimentLogger(str(self.get_parameter('log_dir').value))
        self.frozen_cartesian_servo_ids = self._parse_servo_id_list(
            self.get_parameter('cartesian_freeze_servo_ids').value
        )
        topics = self.config.get('topics', {})
        services = self.config.get('services', {})
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
        self.trajectory_executor = FixedTrajectoryExecutor(
            self.arm,
            self.limits,
            self.trajectory,
            logger=self.episode_logger,
            ros_logger=self.get_logger(),
            frozen_cartesian_servo_ids=self.frozen_cartesian_servo_ids,
        )

    def run_once(self) -> None:
        execute = param_bool(self.get_parameter('execute').value)
        self.get_logger().info(f'trajectory_file={self.trajectory_file}')
        self.get_logger().info(
            f'Cartesian frozen servo ids={self.frozen_cartesian_servo_ids}; '
            f'current pulses for these ids are preserved during CART.'
        )
        for line in self.trajectory_executor.dry_run():
            self.get_logger().info(line)
        if not execute:
            self.get_logger().info('Dry-run complete. No motion command was published.')
            return

        self.get_logger().warning('execute:=true received. Starting fixed trajectory execution.')
        try:
            self.trajectory_executor.execute()
            self.episode_logger.record(stage='sequence_complete', success=True)
            self.get_logger().info('Fixed trajectory execution complete.')
        except Exception as exc:
            self.get_logger().error(f'Fixed trajectory failed: {exc}')
            self.episode_logger.record(
                stage='sequence_error',
                success=False,
                error_reason=str(exc),
            )
            try:
                self.arm.stop_arm()
                self.get_logger().warning('stop_arm() command published after failure.')
            except Exception as stop_exc:
                self.get_logger().error(f'stop_arm() failed after sequence error: {stop_exc}')
            raise

    def _parse_servo_id_list(self, value):
        if isinstance(value, str):
            raw_items = [item.strip() for item in value.split(',') if item.strip()]
        elif isinstance(value, (list, tuple)):
            raw_items = value
        else:
            raw_items = [value]
        servo_ids = []
        for item in raw_items:
            servo_id = int(item)
            if servo_id not in self.limits.arm_servo_ids:
                self.get_logger().warning(f'Ignoring non-arm frozen servo id: {servo_id}')
                continue
            servo_ids.append(servo_id)
        return servo_ids


def main():
    run_with_default_trajectory()


def insert_main():
    run_with_default_trajectory('insert.yaml')


def pull_out_main():
    run_with_default_trajectory('pull_out.yaml')


def run_with_default_trajectory(default_filename=None):
    rclpy.init()
    node = FixedTrajectoryNode(default_trajectory_filename=default_filename)
    try:
        node.run_once()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
