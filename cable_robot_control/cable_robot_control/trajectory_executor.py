"""Fixed trajectory YAML loading, dry-run, and guarded execution."""

import time
from typing import Any, Dict, List, Optional

from cable_robot_control.arm_interface import ArmInterface
from cable_robot_control.cartesian_motion import CartesianMotionPlanner
from cable_robot_control.safety import ArmLimits, soft_delta_warnings, validate_duration, validate_pulses


class FixedTrajectoryExecutor:
    def __init__(
        self,
        arm: ArmInterface,
        limits: ArmLimits,
        trajectory: Dict[str, Any],
        logger=None,
        ros_logger=None,
        frozen_cartesian_servo_ids=None,
    ):
        self.arm = arm
        self.limits = limits
        self.waypoints = trajectory.get('waypoints', {}) or {}
        self.sequence = trajectory.get('sequence', []) or []
        self.gripper = trajectory.get('gripper', {}) or {}
        self.cartesian = trajectory.get('cartesian', {}) or {}
        self.logger = logger
        self.ros_logger = ros_logger
        frozen_servo_ids = frozen_cartesian_servo_ids
        if frozen_servo_ids is None:
            frozen_servo_ids = self.cartesian.get('frozen_servo_ids', [5])
        self.cartesian_planner = CartesianMotionPlanner(
            self.limits.arm_servo_ids,
            frozen_servo_ids=frozen_servo_ids,
        )

    def dry_run(self) -> List[str]:
        lines: List[str] = []
        if not self.sequence:
            return ['sequence is empty; no motion command would be published']
        previous: Optional[Dict[int, int]] = None
        current_pose_xyz: Optional[List[float]] = None
        for index, step in enumerate(self.sequence, start=1):
            command = self._normalize_step(step)
            op = command['command']
            if op == 'MOVE':
                name = command['waypoint']
                pulses, duration = self._waypoint_pulses(name)
                waypoint = self._waypoint(name)
                warnings = soft_delta_warnings(previous, pulses, self.limits)
                lines.append(f'{index}. MOVE {name} duration={duration:.2f}s pulses={pulses}')
                for warning in warnings:
                    lines.append(f'   WARNING: {warning}')
                previous = pulses
                current_pose_xyz = self._waypoint_xyz(waypoint)
            elif op == 'CART':
                dx, dy, dz, pitch, duration = self._cartesian_command(command)
                if current_pose_xyz is None:
                    raise ValueError(f'CART at step {index} requires a previous MOVE waypoint with pose')
                target_xyz = [
                    current_pose_xyz[0] + dx,
                    current_pose_xyz[1] + dy,
                    current_pose_xyz[2] + dz,
                ]
                lines.append(
                    f'{index}. CART dx={dx:.4f} dy={dy:.4f} dz={dz:.4f} '
                    f'pitch={pitch:.2f} duration={duration:.2f}s target_xyz={target_xyz}'
                )
                current_pose_xyz = target_xyz
            elif op == 'GRIPPER':
                pulse, duration = self._gripper_command(command)
                lines.append(
                    f"{index}. GRIPPER {command['state']} duration={duration:.2f}s pulse={pulse}"
                )
            elif op == 'SET_GRIPPER':
                pulse, duration = self._gripper_command(command)
                lines.append(f'{index}. SET_GRIPPER duration={duration:.2f}s pulse={pulse}')
            elif op == 'WAIT':
                duration = float(command['duration'])
                validate_duration(duration, self.limits)
                lines.append(f'{index}. WAIT duration={duration:.2f}s')
            else:
                raise ValueError(f'unknown trajectory command: {op}')
        return lines

    def execute(self) -> None:
        previous: Optional[Dict[int, int]] = None
        current_pulse: Optional[Dict[int, int]] = None
        current_pose_xyz: Optional[List[float]] = None
        for index, step in enumerate(self.sequence, start=1):
            command = self._normalize_step(step)
            op = command['command']
            if op == 'MOVE':
                name = command['waypoint']
                pulses, duration = self._waypoint_pulses(name)
                waypoint = self._waypoint(name)
                warnings = soft_delta_warnings(previous, pulses, self.limits)
                self._log_info(f'{index}. MOVE {name} duration={duration:.2f}s pulses={pulses}')
                for warning in warnings:
                    self._log_warning(warning)
                self.arm.move_joint(pulses, duration)
                self._record(
                    stage='move',
                    waypoint=name,
                    target_joint_pulse=pulses,
                    duration=duration,
                    success=True,
                )
                time.sleep(duration)
                previous = pulses
                current_pulse = dict(pulses)
                current_pose_xyz = self._waypoint_xyz(waypoint)
            elif op == 'CART':
                if current_pulse is None or current_pose_xyz is None:
                    raise ValueError(f'CART at step {index} requires a previous MOVE waypoint with pose')
                dx, dy, dz, pitch, duration = self._cartesian_command(command)
                plan = self.cartesian_planner.plan_delta(
                    self.arm,
                    current_pulse,
                    current_pose_xyz,
                    [dx, dy, dz],
                    pitch,
                )
                self._log_cartesian_plan(index, plan, duration)
                if not plan.safe:
                    self._record(
                        stage='cartesian',
                        target_pose={'position': plan.target_xyz, 'pitch': pitch},
                        target_joint_pulse=plan.arm_pulses,
                        current_joint_pulse=current_pulse,
                        duration=duration,
                        success=False,
                        error_reason=plan.reason,
                    )
                    raise ValueError(f'CART step {index} rejected: {plan.reason}')
                target_pulses = dict(current_pulse)
                target_pulses.update(plan.arm_pulses)
                self.arm.move_joint(target_pulses, duration)
                self._record(
                    stage='cartesian',
                    target_pose={'position': {'x': plan.target_xyz[0], 'y': plan.target_xyz[1], 'z': plan.target_xyz[2]}, 'pitch': pitch},
                    target_joint_pulse=target_pulses,
                    current_joint_pulse=current_pulse,
                    duration=duration,
                    success=True,
                )
                time.sleep(duration)
                previous = target_pulses
                current_pulse = target_pulses
                current_pose_xyz = [float(value) for value in plan.target_xyz]
            elif op == 'GRIPPER':
                pulse, duration = self._gripper_command(command)
                self._log_info(
                    f"{index}. GRIPPER {command['state']} duration={duration:.2f}s pulse={pulse}"
                )
                self.arm.set_gripper(pulse, duration)
                self._record(
                    stage='gripper',
                    waypoint=command['state'],
                    target_joint_pulse={self.limits.gripper_servo_id: pulse},
                    duration=duration,
                    success=True,
                )
                time.sleep(duration)
            elif op == 'SET_GRIPPER':
                pulse, duration = self._gripper_command(command)
                self._log_info(f'{index}. SET_GRIPPER duration={duration:.2f}s pulse={pulse}')
                self.arm.set_gripper(pulse, duration)
                self._record(
                    stage='set_gripper',
                    target_joint_pulse={self.limits.gripper_servo_id: pulse},
                    duration=duration,
                    success=True,
                )
                time.sleep(duration)
            elif op == 'WAIT':
                duration = float(command['duration'])
                validate_duration(duration, self.limits)
                self._log_info(f'{index}. WAIT duration={duration:.2f}s')
                self._record(stage='wait', duration=duration, success=True)
                time.sleep(duration)
            else:
                raise ValueError(f'unknown trajectory command: {op}')

    def _waypoint(self, name: str):
        if name in self.waypoints:
            return self.waypoints[name]
        matches = [key for key in self.waypoints if key.upper() == name.upper()]
        if len(matches) == 1:
            return self.waypoints[matches[0]]
        raise ValueError(f'unknown waypoint: {name}')

    def _waypoint_pulses(self, name: str):
        waypoint = self._waypoint(name)
        joint_pulse = waypoint.get('joint_pulse') or {}
        gripper_pulse = waypoint.get('gripper_pulse') or {}
        pulses = {int(servo_id): int(pulse) for servo_id, pulse in joint_pulse.items()}
        pulses.update({int(servo_id): int(pulse) for servo_id, pulse in gripper_pulse.items()})
        required = set(self.limits.arm_servo_ids + [self.limits.gripper_servo_id])
        missing = sorted(required - set(pulses))
        if missing:
            raise ValueError(f'waypoint {name} missing pulse for servo id(s): {missing}')
        validate_pulses(pulses, self.limits, include_gripper=True)
        duration = float(waypoint.get('duration', 2.0))
        validate_duration(duration, self.limits)
        return pulses, duration

    def _waypoint_xyz(self, waypoint) -> Optional[List[float]]:
        pose = waypoint.get('pose') or {}
        position = pose.get('position') or {}
        try:
            return [
                float(position['x']),
                float(position['y']),
                float(position['z']),
            ]
        except (KeyError, TypeError, ValueError):
            return None

    def _cartesian_command(self, command):
        duration = float(command.get('duration', self.cartesian.get('default_duration', 1.2)))
        validate_duration(duration, self.limits)
        try:
            dx = float(command['dx'])
            dy = float(command['dy'])
            dz = float(command['dz'])
            pitch = float(command.get('pitch', self.cartesian.get('default_pitch', 11.0)))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('CART requires dx dy dz pitch [duration]') from exc
        return dx, dy, dz, pitch, duration

    def _gripper_command(self, command):
        duration = float(command.get('duration', self.gripper.get('default_duration', 0.5)))
        validate_duration(duration, self.limits)
        if command['command'] == 'SET_GRIPPER':
            if 'pulse' not in command:
                raise ValueError('SET_GRIPPER requires pulse')
            pulse = int(command['pulse'])
        else:
            state = str(command['state']).lower()
            presets = self.gripper.get('presets', {}) or {}
            if state not in presets or presets[state] is None:
                raise ValueError(f'gripper preset "{state}" is not configured in trajectory YAML')
            pulse = int(presets[state])
        validate_pulses({self.limits.gripper_servo_id: pulse}, self.limits)
        return pulse, duration

    def _normalize_step(self, step):
        if isinstance(step, str):
            parts = step.split()
            if not parts:
                raise ValueError('empty trajectory step')
            if parts[0].upper() == 'MOVE' and len(parts) == 2:
                return {'command': 'MOVE', 'waypoint': parts[1]}
            if parts[0].upper() == 'WAIT' and len(parts) == 2:
                return {'command': 'WAIT', 'duration': float(parts[1])}
            if parts[0].upper() == 'CART' and len(parts) in (5, 6):
                command = {
                    'command': 'CART',
                    'dx': float(parts[1]),
                    'dy': float(parts[2]),
                    'dz': float(parts[3]),
                    'pitch': float(parts[4]),
                }
                if len(parts) == 6:
                    command['duration'] = float(parts[5])
                return command
            if parts[0].upper() == 'GRIPPER' and len(parts) == 2:
                return {'command': 'GRIPPER', 'state': parts[1].upper()}
            if parts[0].upper() == 'SET_GRIPPER' and len(parts) == 2:
                return {'command': 'SET_GRIPPER', 'pulse': int(parts[1])}
            raise ValueError(f'cannot parse trajectory step: {step}')
        command = dict(step)
        command['command'] = str(command.get('command', '')).upper()
        return command

    def _log_info(self, message: str) -> None:
        if self.ros_logger:
            self.ros_logger.info(message)

    def _log_warning(self, message: str) -> None:
        if self.ros_logger:
            self.ros_logger.warning(message)

    def _log_cartesian_plan(self, index: int, plan, duration: float) -> None:
        status = 'SAFE' if plan.safe else 'REJECT'
        self._log_info(
            f'{index}. CART {status} target_xyz={plan.target_xyz} pitch={plan.pitch:.2f} '
            f'duration={duration:.2f}s scale={plan.scale} raw_ik_pulses={plan.raw_ik_pulses} '
            f'command_arm_pulses={plan.arm_pulses} joint_delta={plan.joint_delta} '
            f'total_delta={plan.total_delta} reason={plan.reason}'
        )

    def _record(self, **kwargs) -> None:
        if self.logger:
            self.logger.record(**kwargs)
