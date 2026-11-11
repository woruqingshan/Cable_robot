"""Prepare and recheck simulation-only execution; no ROS or vendor imports."""
from collections import deque
from vla_protocol import codec
from vla_protocol.action import RobotActionChunk, ValidatedTrajectory
from vla_protocol.observation import identity_fields
from vla_protocol.errors import VLAError
from .action_adapter import adapt_action


def assert_current(action, source, current):
    for field in ("session_id", "task_revision", "robot_id", "robot_profile_id", "state_revision", "scene_id", "scene_sha256", "model_id"):
        if getattr(action, field) != getattr(current, field) or getattr(source, field) != getattr(current, field):
            raise VLAError("STATE_INVALIDATED", "current context changed at " + field)
    if source.joints.positions != current.joints.positions or source.gripper.position != current.gripper.position:
        raise VLAError("STATE_INVALIDATED", "starting joint/gripper positions changed")


class MockExecutor:
    def __init__(self, validator, *, advance_simulation=False, allow_unvalidated=False, selection=None):
        self.validator = validator
        self.advance_simulation = advance_simulation
        self.allow_unvalidated = allow_unvalidated
        self.selection = selection
        self.consumed = deque(maxlen=1024)

    def prepare(self, inference):
        action = adapt_action(inference.candidate, selection=self.selection)
        if not inference.observation.simulated:
            raise VLAError("REAL_OBSERVATION_EXECUTION_DISABLED", "this executor accepts only synthetic observations")
        assert_current(action, inference.observation, inference.observation)
        result = self.validator.validate(action, inference.observation)
        result = codec.validate(result.model_dump(), ValidatedTrajectory)
        if result.source_action_id != action.action_id or (result.trajectory is not None and result.trajectory != action):
            raise VLAError("IDENTITY_MISMATCH", "validator returned a different candidate")
        data = result.model_dump()
        data["provenance"]["adaptation"] = {"mapping_id": action.mapping_id,
            "source_policy_action_id": action.source_policy_action_id, "robot_action_id": action.action_id,
            "action_profile": action.action_profile,
            "horizon": action.horizon.model_dump() if action.horizon is not None else None}
        return codec.validate(data, ValidatedTrajectory)

    def consume(self, validated, inference, current, now, max_age_s):
        validated = codec.validate(validated.model_dump(), ValidatedTrajectory)
        if now - inference.submitted_at > max_age_s:
            raise VLAError("STALE_ACTION", "candidate expired before mock execution")
        if validated.source_action_id in self.consumed:
            raise VLAError("DUPLICATE_ACTION", "candidate already consumed")
        self.consumed.append(validated.source_action_id)
        if validated.trajectory is None:
            return "REJECTED"
        if (identity_fields(validated.trajectory) != identity_fields(inference.observation)
                or validated.trajectory.source_policy_action_id != inference.candidate.action_id):
            raise VLAError("IDENTITY_MISMATCH", "validated action differs from inference identity")
        assert_current(validated.trajectory, inference.observation, current)
        if not self.advance_simulation:
            return "RECORDED_ONLY"
        if validated.status == "NOT_CHECKED" and not self.allow_unvalidated:
            return "BLOCKED_UNVALIDATED"
        return "SIMULATE_UNVALIDATED" if validated.status == "NOT_CHECKED" else "SIMULATE_RESEARCH_ONLY"

    def close(self):
        self.validator.close()
