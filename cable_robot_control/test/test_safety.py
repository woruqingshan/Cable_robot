from cable_robot_control.safety import ArmLimits, SafetyError, validate_duration, validate_pulses


def test_validate_pulses_accepts_arm_ids():
    limits = ArmLimits(arm_servo_ids=[1, 2, 3, 4, 5], gripper_servo_id=10)
    validate_pulses({1: 500, 5: 1000}, limits, include_gripper=False)


def test_validate_pulses_rejects_unknown_id():
    limits = ArmLimits(arm_servo_ids=[1, 2, 3, 4, 5], gripper_servo_id=10)
    try:
        validate_pulses({9: 500}, limits, include_gripper=False)
    except SafetyError:
        return
    raise AssertionError('expected SafetyError')


def test_validate_duration_rejects_out_of_range():
    limits = ArmLimits(arm_servo_ids=[1, 2, 3, 4, 5], gripper_servo_id=10)
    try:
        validate_duration(0.0, limits)
    except SafetyError:
        return
    raise AssertionError('expected SafetyError')
