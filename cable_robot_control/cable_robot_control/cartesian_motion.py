"""Safety wrapper for JetRover Cartesian IK steps."""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional


@dataclass
class CartesianPlan:
    target_xyz: List[float]
    pitch: float
    arm_pulses: Dict[int, int]
    raw_ik_pulses: Dict[int, int]
    joint_delta: Dict[int, int]
    total_delta: int
    scale: float
    safe: bool
    reason: str = ''


class CartesianMotionPlanner:
    def __init__(
        self,
        arm_servo_ids: Iterable[int],
        frozen_servo_ids: Optional[Iterable[int]] = None,
        per_joint_delta_limit: int = 40,
        total_delta_limit: int = 120,
        retry_scales: Optional[List[float]] = None,
    ):
        self.arm_servo_ids = [int(servo_id) for servo_id in arm_servo_ids]
        self.frozen_servo_ids = {
            int(servo_id)
            for servo_id in (frozen_servo_ids or [])
            if int(servo_id) in self.arm_servo_ids
        }
        self.per_joint_delta_limit = int(per_joint_delta_limit)
        self.total_delta_limit = int(total_delta_limit)
        self.retry_scales = retry_scales or [1.0, 0.5, 0.25]

    def plan_absolute(self, arm, current_pulse: Dict[int, int], target_xyz, pitch: float) -> CartesianPlan:
        return self._solve_and_check(arm, current_pulse, [float(value) for value in target_xyz], float(pitch), 1.0)

    def plan_delta(self, arm, current_pulse: Dict[int, int], current_xyz, delta_xyz, pitch: float) -> CartesianPlan:
        last_plan = None
        for scale in self.retry_scales:
            target_xyz = [
                float(current_xyz[index]) + float(delta_xyz[index]) * float(scale)
                for index in range(3)
            ]
            plan = self._solve_and_check(arm, current_pulse, target_xyz, float(pitch), float(scale))
            if plan.safe:
                return plan
            last_plan = plan
        return last_plan

    def _solve_and_check(self, arm, current_pulse: Dict[int, int], target_xyz, pitch: float, scale: float) -> CartesianPlan:
        response = arm.solve_ik(target_xyz, pitch)
        raw_ik_pulses = {
            servo_id: int(response.pulse[index])
            for index, servo_id in enumerate(self.arm_servo_ids)
        }
        arm_pulses = dict(raw_ik_pulses)
        for servo_id in self.frozen_servo_ids:
            arm_pulses[servo_id] = int(current_pulse[servo_id])
        joint_delta = {
            servo_id: int(arm_pulses[servo_id]) - int(current_pulse[servo_id])
            for servo_id in self.arm_servo_ids
        }
        abs_delta = {servo_id: abs(delta) for servo_id, delta in joint_delta.items()}
        total_delta = int(sum(abs_delta.values()))

        largest_servo = max(abs_delta, key=abs_delta.get)
        if abs_delta[largest_servo] > self.per_joint_delta_limit:
            return CartesianPlan(
                target_xyz=target_xyz,
                pitch=pitch,
                arm_pulses=arm_pulses,
                raw_ik_pulses=raw_ik_pulses,
                joint_delta=joint_delta,
                total_delta=total_delta,
                scale=scale,
                safe=False,
                reason=(
                    f'servo {largest_servo} delta {abs_delta[largest_servo]} exceeds '
                    f'per-joint limit {self.per_joint_delta_limit}'
                ),
            )
        if total_delta > self.total_delta_limit:
            return CartesianPlan(
                target_xyz=target_xyz,
                pitch=pitch,
                arm_pulses=arm_pulses,
                raw_ik_pulses=raw_ik_pulses,
                joint_delta=joint_delta,
                total_delta=total_delta,
                scale=scale,
                safe=False,
                reason=f'total delta {total_delta} exceeds limit {self.total_delta_limit}',
            )
        return CartesianPlan(
            target_xyz=target_xyz,
            pitch=pitch,
            arm_pulses=arm_pulses,
            raw_ik_pulses=raw_ik_pulses,
            joint_delta=joint_delta,
            total_delta=total_delta,
            scale=scale,
            safe=True,
        )
