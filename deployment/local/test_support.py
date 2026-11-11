"""Small explicit synthetic fixtures shared by Stage 1 tests (no model weights)."""
import base64
from hashlib import sha256
import time
from uuid import uuid4
from vla_protocol.observation import Observation
from vla_protocol.png import encode_rgb
from vla_protocol.profiles import JOINT_NAMES


def observation(q=None, **changes):
    png = encode_rgb(2, 2, bytes([30, 60, 90] * 4))
    timestamp = {"nanoseconds": time.time_ns(), "clock_domain": "ros_system"}
    data = dict(schema_version=1, request_id=str(uuid4()), session_id="test_session", observation_seq=0,
        task_revision=0, state_revision=0, robot_id="jetrover_mock", robot_profile_id="jetrover_joint_abs_rad_v1",
        scene_id="stage1_synthetic", scene_sha256="0" * 64, model_id="synthetic_joint_state_v1",
        instruction="Hold position", simulated=True, calibration_status="synthetic_uncalibrated",
        joints={"names": list(JOINT_NAMES), "positions": list(q or [0.] * 5), "unit": "rad", "stamp": timestamp},
        gripper={"name": "r_joint", "position": 0.3, "unit": "rad", "stamp": timestamp, "simulated": True, "calibrated": False},
        images=[{"camera_id": "front", "frame_id": "mock_camera_optical_frame", "stamp": timestamp,
            "width": 2, "height": 2, "codec": "png", "pixel_encoding": "rgb8", "sha256": sha256(png).hexdigest(),
            "data_base64": base64.b64encode(png).decode()}])
    data.update(changes)
    return Observation.model_validate(data)


def moved_candidate(obs, waypoints):
    from vla_server.policies.mock import HoldPositionPolicy
    candidate = HoldPositionPolicy().predict(obs).model_dump()
    candidate.update(shape=[len(waypoints), 5], values=waypoints,
                     time_offsets_s=[0.1 * (i + 1) for i in range(len(waypoints))])
    from vla_protocol.action import PolicyActionChunk
    return PolicyActionChunk.model_validate(candidate)


def motion_fixture_app():
    """Explicit test-only HTTP policy; the production CLI always uses Hold Position."""
    from vla_server.app import create_app
    from vla_server.policies.mock import HoldPositionPolicy
    class StepFixture(HoldPositionPolicy):
        def predict(self, obs):
            target = list(obs.joints.positions)
            target[0] += 0.01
            return moved_candidate(obs, [target])
    return create_app(StepFixture())
