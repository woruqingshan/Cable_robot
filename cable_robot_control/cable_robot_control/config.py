"""Local configuration loading for cable_robot_control."""

from typing import Any, Dict

import yaml

from cable_robot_control.safety import ArmLimits


def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return data or {}


def limits_from_config(data: Dict[str, Any]) -> ArmLimits:
    arm = data.get('arm', {})
    gripper = data.get('gripper', {})
    soft = arm.get('soft_warning', {})
    return ArmLimits(
        arm_servo_ids=[int(value) for value in arm.get('servo_ids', [1, 2, 3, 4, 5])],
        gripper_servo_id=int(gripper.get('servo_id', 10)),
        pulse_min=int(arm.get('pulse_min', 0)),
        pulse_max=int(arm.get('pulse_max', 1000)),
        soft_per_joint_delta=int(soft.get('per_joint_delta', 250)),
        soft_total_delta=int(soft.get('total_delta', 900)),
    )
