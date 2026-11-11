"""Create an explicit unvalidated synthetic fixture for read-only preview."""
import argparse
from hashlib import sha256
from pathlib import Path
from time import time_ns
from uuid import uuid4
from vla_protocol import codec
from vla_protocol.action import RobotActionChunk, ValidatedTrajectory
from vla_protocol.profiles import JOINT_NAMES


def synthetic_fixture():
    stamp = {"nanoseconds": time_ns(), "clock_domain": "ros_system"}
    joints = {"names": list(JOINT_NAMES), "positions": [0.] * 5, "unit": "rad", "stamp": stamp}
    gripper = {"name": "r_joint", "position": 0.3, "unit": "rad", "stamp": stamp, "simulated": True, "calibrated": False}
    action = RobotActionChunk(schema_version=1, request_id=str(uuid4()), session_id="preview_fixture",
        observation_seq=0, task_revision=0, robot_id="jetrover_mock", robot_profile_id="jetrover_joint_abs_rad_v1",
        state_revision=0, scene_id="synthetic_preview_no_scene", scene_sha256="0" * 64, model_id="synthetic_joint_state_v1",
        action_id=str(uuid4()), source_policy_action_id="explicit_synthetic_fixture_v1", action_profile="jetrover_joint_abs_rad_v1",
        channel_names=list(JOINT_NAMES), units=["rad"] * 5, semantics="absolute", shape=[3, 5],
        values=[[0.10, 0.10, -0.10, 0.10, 0.0], [-0.10, 0.15, -0.15, 0.15, 0.10], [0., 0., 0., 0., 0.]],
        time_offsets_s=[1.0, 2.0, 3.0], mapping_id="synthetic_identity_v1", calibration_status="synthetic_uncalibrated")
    return ValidatedTrajectory(schema_version=1, source_action_id=action.action_id,
        starting_joints=joints, starting_gripper=gripper, status="NOT_CHECKED", trajectory=action,
        failure_reasons=[], scene_id=action.scene_id, scene_sha256=action.scene_sha256, model_id=action.model_id,
        validation_config={"mode": "disabled"}, provenance={"fixture": "explicit_synthetic_fixture_v1", "execution_authorized": False},
        discrete_only=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    args = parser.parse_args()
    data = codec.dumps(synthetic_fixture())
    with Path(args.output).expanduser().open("xb") as stream:
        stream.write(data)
    print(str(Path(args.output).resolve()))
    print("sha256=" + sha256(data).hexdigest())


if __name__ == "__main__":
    main()
