"""Timed visualization of an already interpreted robot trajectory, never native actions."""
from bisect import bisect_right
import json
import math
from pathlib import Path
from vla_protocol import codec
from vla_protocol.action import ValidatedTrajectory
from vla_protocol.errors import VLAError
from vla_protocol.profiles import MAX_BODY_BYTES


def load_record(path, *, action_id=""):
    path = Path(path).expanduser()
    if path.stat().st_size > 64_000_000:
        raise VLAError("PLAYBACK_FILE_TOO_LARGE", "select a bounded record or a smaller JSONL run")
    if path.suffix != ".jsonl":
        result = codec.loads(path.read_bytes(), ValidatedTrajectory)
        if action_id and result.source_action_id != action_id:
            raise VLAError("PLAYBACK_SELECTION_MISSING", "requested action does not match record")
        return result
    found = []
    with path.open("rb") as stream:
        for index in range(10001):
            line = stream.readline(MAX_BODY_BYTES + 1)
            if not line:
                break
            if index == 10000 or len(line) > MAX_BODY_BYTES:
                raise VLAError("PLAYBACK_FILE_TOO_LARGE", "record/line budget exceeded")
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except ValueError as exc:
                raise VLAError("INVALID_JSON", "invalid JSONL record") from exc
            if not isinstance(item, dict) or "validated_trajectory" not in item:
                continue
            data = item["validated_trajectory"]
            if not isinstance(data, dict) or (action_id and data.get("source_action_id") != action_id):
                continue
            result = codec.validate(data, ValidatedTrajectory)
            if result.trajectory is not None:
                found.append(result)
            if len(found) > 1:
                raise VLAError("AMBIGUOUS_PLAYBACK_SELECTION", "select one source_action_id from the JSONL run")
    if len(found) != 1:
        raise VLAError("PLAYBACK_SELECTION_MISSING", "no accepted robot trajectory matches selection")
    return found[0]


class TrajectoryPlayback:
    def __init__(self, validated, *, allow_synthetic=False):
        self.record = codec.validate(validated.model_dump(), ValidatedTrajectory)
        if self.record.trajectory is None or self.record.status not in ("NOT_CHECKED", "PASS_RESEARCH_ONLY"):
            raise VLAError("PLAYBACK_TRAJECTORY_REJECTED", "no accepted robot-space candidate to visualize")
        if self.record.status == "NOT_CHECKED" and not allow_synthetic:
            raise VLAError("SYNTHETIC_PREVIEW_OPT_IN_REQUIRED", "unchecked synthetic preview requires an explicit flag")
        self.action = self.record.trajectory
        self.times = (0.0, *self.action.time_offsets_s)
        self.positions = (tuple(self.record.starting_joints.positions), *(tuple(q) for q in self.action.values))
        self.duration = self.times[-1]
        self.label = "SYNTHETIC / NOT_CHECKED" if self.record.status == "NOT_CHECKED" else "RESEARCH / DISCRETE_ONLY"

    def at(self, elapsed):
        if type(elapsed) not in (int, float) or not math.isfinite(elapsed) or elapsed < 0:
            raise VLAError("INVALID_PLAYBACK_TIME", "finite nonnegative elapsed time required")
        if elapsed >= self.duration:
            return self.positions[-1]
        index = bisect_right(self.times, elapsed) - 1
        fraction = (elapsed - self.times[index]) / (self.times[index + 1] - self.times[index])
        # Display interpolation only, not a planner or an additional validity check.
        return tuple(a + fraction * (b - a) for a, b in zip(self.positions[index], self.positions[index + 1]))

    def bind_model(self, model):
        expected = "urdf:" + model.metadata()["expanded_urdf_sha256"]
        if self.action.model_id != expected and not (self.record.status == "NOT_CHECKED" and self.action.model_id == "synthetic_joint_state_v1"):
            raise VLAError("PLAYBACK_MODEL_MISMATCH", "record is not bound to this robot model")
        for q in self.positions:
            model.joint_positions(q, {"r_joint": self.record.starting_gripper.position})


def load_model(workspace_root, config_path=None):
    from .description import load_description_model
    try:
        return load_description_model(workspace_root, config_path)
    except (ValueError, ImportError, OSError) as exc:
        raise VLAError("PLAYBACK_MODEL_MISSING", str(exc)) from exc


def joint_values(model, q, gripper):
    # Delegate all independent/mimic state resolution and limits to existing P2.
    values = model.joint_positions(q, {"r_joint": gripper})
    independent = {name for name, joint in model.joints.items() if joint.kind != "fixed" and not joint.mimic_joint}
    names = tuple(model.joint_order) + tuple(sorted(independent - set(model.joint_order)))
    return names, tuple(float(values[name]) for name in names)
