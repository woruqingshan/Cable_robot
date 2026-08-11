"""Fixed trajectory YAML loading, dry-run, and guarded execution."""

import time
from typing import Any, Dict, List, Optional

from cable_robot_control.arm_interface import ArmInterface
from cable_robot_control.safety import ArmLimits, soft_delta_warnings, validate_duration, validate_pulses


class FixedTrajectoryExecutor:
    def __init__(
        self,
        arm: ArmInterface,
        limits: ArmLimits,
        trajectory: Dict[str, Any],
        logger=None,
        ros_logger=None,
    ):
        self.arm = arm
        self.limits = limits
        self.waypoints = trajectory.get('waypoints', {}) or {}
        self.sequence = trajectory.get('sequence', []) or []
        self.gripper = trajectory.get('gripper', {}) or {}
        self.logger = logger
        self.ros_logger = ros_logger

    def dry_run(self) -> List[str]:
        lines: List[str] = []
        if not self.sequence:
            return ['sequence is empty; no motion command would be published']
        previous: Optional[Dict[int, int]] = None
        for index, step in enumerate(self.sequence, start=1):
            command = self._normalize_step(step)
            op = command['command']
            if op == 'MOVE':
                name = command['waypoint']
                pulses, duration = self._waypoint_pulses(name)
                warnings = soft_delta_warnings(previous, pulses, self.limits)
                lines.append(f'{index}. MOVE {name} duration={duration:.2f}s pulses={pulses}')
                for warning in warnings:
                    lines.append(f'   WARNING: {warning}')
                previous = pulses
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
        for index, step in enumerate(self.sequence, start=1):
            command = self._normalize_step(step)
            op = command['command']
            if op == 'MOVE':
                name = command['waypoint']
                pulses, duration = self._waypoint_pulses(name)
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

    def _waypoint_pulses(self, name: str):
        if name not in self.waypoints:
            raise ValueError(f'unknown waypoint: {name}')
        waypoint = self.waypoints[name]
        joint_pulse = waypoint.get('joint_pulse') or {}
        pulses = {int(servo_id): int(pulse) for servo_id, pulse in joint_pulse.items()}
        validate_pulses(pulses, self.limits, include_gripper=False)
        duration = float(waypoint.get('duration', 2.0))
        validate_duration(duration, self.limits)
        return pulses, duration

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

    def _record(self, **kwargs) -> None:
        if self.logger:
            self.logger.record(**kwargs)
