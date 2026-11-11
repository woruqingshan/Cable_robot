"""Pure simulated state machine. Only a ROS mock node publishes its state."""
from collections import deque
from uuid import uuid4
from vla_protocol import codec
from vla_protocol.action import ValidatedTrajectory
from vla_protocol.errors import VLAError


class SimulationState:
    def __init__(self, initial_q, *, gripper=0.3, instruction="Hold the simulated arm position",
                 robot_id="jetrover_mock", scene_id="stage1_synthetic", scene_sha256="0" * 64,
                 model_id="synthetic_joint_state_v1", allow_unvalidated=False, validation_mode="disabled"):
        self.initial_q = list(initial_q)
        self.initial_gripper = gripper
        self.instruction = instruction
        self.robot_id, self.scene_id, self.scene_sha256, self.model_id = robot_id, scene_id, scene_sha256, model_id
        self.allow_unvalidated, self.validation_mode = allow_unvalidated, validation_mode
        self.seen = deque(maxlen=1024)
        self.reset()

    def reset(self):
        self.session_id = str(uuid4())
        self.task_revision = self.state_revision = 0
        self.q, self.gripper = list(self.initial_q), self.initial_gripper
        self.active = None
        self.seen.clear()

    def set_instruction(self, text):
        if not isinstance(text, str) or not text.strip() or len(text) > 2048:
            raise VLAError("INVALID_INSTRUCTION", "nonempty instruction of at most 2048 characters required")
        if text != self.instruction:
            self.instruction = text
            self.task_revision += 1
            self.state_revision += 1
            self.active = None

    def accept(self, validated, now, now_ros_ns, max_age_s):
        validated = codec.validate(validated.model_dump(), ValidatedTrajectory)
        action = validated.trajectory
        if action is None or validated.status not in ("NOT_CHECKED", "PASS_RESEARCH_ONLY"):
            raise VLAError("TRAJECTORY_REJECTED", "trajectory has no accepted simulation candidate")
        if validated.source_action_id in self.seen:
            raise VLAError("DUPLICATE_ACTION", "mock command already consumed")
        if self.active is not None:
            raise VLAError("ROBOT_BUSY", "mock trajectory already active")
        if self.validation_mode == "required" and validated.status != "PASS_RESEARCH_ONLY":
            raise VLAError("P2_REQUIRED", "required mode blocks unchecked simulation")
        if validated.status == "NOT_CHECKED" and not self.allow_unvalidated:
            raise VLAError("UNVALIDATED_SIMULATION_DISABLED", "explicit development opt-in required")
        for field in ("session_id", "task_revision", "state_revision", "robot_id", "scene_id", "scene_sha256", "model_id"):
            if getattr(action, field) != getattr(self, field):
                raise VLAError("STATE_INVALIDATED", "mock context changed at " + field)
        if validated.starting_joints.positions != self.q or validated.starting_gripper.position != self.gripper:
            raise VLAError("STATE_INVALIDATED", "mock starting state changed")
        age = (now_ros_ns - validated.starting_joints.stamp.nanoseconds) / 1e9
        if validated.starting_joints.stamp.clock_domain != "ros_system" or not 0 <= age <= max_age_s:
            raise VLAError("STALE_ACTION", "mock source timestamp is stale or uses a different clock")
        self.seen.append(validated.source_action_id)
        self.active = {"action": action, "start": now, "next": 0, "validation_status": validated.status}
        self.state_revision += 1

    def tick(self, now):
        active = self.active
        if active is None:
            return None
        action = active["action"]
        while active["next"] < len(action.values) and now - active["start"] >= action.time_offsets_s[active["next"]]:
            self.q = list(action.values[active["next"]])
            active["next"] += 1
            self.state_revision += 1
        if active["next"] == len(action.values):
            self.active = None
            return {"source_action_id": action.action_id, "request_id": action.request_id,
                    "status": "SIMULATED_UNVALIDATED" if active["validation_status"] == "NOT_CHECKED" else "SIMULATED_RESEARCH_ONLY",
                    "positions": list(self.q), "state_revision": self.state_revision}
        return None
