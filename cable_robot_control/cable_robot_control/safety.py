"""Safety checks for taught JetRover arm waypoints."""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


@dataclass(frozen=True)
class ArmLimits:
    arm_servo_ids: List[int]
    gripper_servo_id: int
    pulse_min: int = 0
    pulse_max: int = 1000
    min_duration: float = 0.02
    max_duration: float = 30.0
    soft_per_joint_delta: int = 250
    soft_total_delta: int = 900


class SafetyError(ValueError):
    """Raised when a hard safety rule blocks a command."""


def validate_servo_ids(ids: Iterable[int], limits: ArmLimits, include_gripper: bool = True) -> None:
    allowed = set(limits.arm_servo_ids)
    if include_gripper:
        allowed.add(limits.gripper_servo_id)
    invalid = [servo_id for servo_id in ids if servo_id not in allowed]
    if invalid:
        raise SafetyError(f'illegal servo id(s): {invalid}; allowed={sorted(allowed)}')


def validate_pulses(pulses: Dict[int, int], limits: ArmLimits, include_gripper: bool = True) -> None:
    validate_servo_ids(pulses.keys(), limits, include_gripper=include_gripper)
    for servo_id, pulse in pulses.items():
        if pulse < limits.pulse_min or pulse > limits.pulse_max:
            raise SafetyError(
                f'servo {servo_id} pulse {pulse} outside {limits.pulse_min}..{limits.pulse_max}'
            )


def validate_duration(duration: float, limits: ArmLimits) -> None:
    if duration < limits.min_duration or duration > limits.max_duration:
        raise SafetyError(f'duration {duration} outside {limits.min_duration}..{limits.max_duration}')


def soft_delta_warnings(
    current: Optional[Dict[int, int]],
    target: Dict[int, int],
    limits: ArmLimits,
) -> List[str]:
    if not current:
        return []

    warnings = []
    total = 0
    for servo_id, target_pulse in target.items():
        if servo_id not in current:
            continue
        delta = abs(target_pulse - current[servo_id])
        total += delta
        if delta > limits.soft_per_joint_delta:
            warnings.append(
                f'servo {servo_id} delta {delta} exceeds warning threshold '
                f'{limits.soft_per_joint_delta}'
            )
    if total > limits.soft_total_delta:
        warnings.append(f'total pulse delta {total} exceeds warning threshold {limits.soft_total_delta}')
    return warnings
