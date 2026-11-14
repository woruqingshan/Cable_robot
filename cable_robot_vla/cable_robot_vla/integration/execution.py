"""Simulation-only execution with P2 and observed feedback at the local boundary."""
from collections import deque
from time import monotonic
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk, PolicyIdentity
from vla_protocol.observation import identity_fields
from vla_protocol.profiles import JOINT_NAMES, ROBOT_PROFILE
from vla_protocol.errors import VLAError
from uuid import uuid4
from .interfaces import RegisteredActionInterpreter


def known_simulation_candidate(observation, target, duration_s=2.0):
    """An explicitly rule-generated scenario; consumes no native VLA action values."""
    return PolicyActionChunk(**identity_fields(observation), action_id=str(uuid4()),
        policy=PolicyIdentity(policy_type="synthetic_scenario", policy_version="simulation_scenario_v1",
            framework_revision=None, checkpoint_revision=None, processor_id="mock_identity_v1", normalization_id="none"),
        native_action_space=ROBOT_PROFILE, channel_names=list(JOINT_NAMES), units=["rad"] * 5,
        semantics="absolute", shape=[1, 5], values=[list(target)], time_offsets_s=[duration_s],
        inference_duration_s=0.0, synthetic=True)


class InspectionBackend:
    kind = "inspection"
    active = False
    def __init__(self, *, validator, interpreter=None, **_):
        self.validator = validator
        self.interpreter = interpreter or RegisteredActionInterpreter()
    def submit(self, candidate, observation):
        action = self.interpreter.interpret(candidate)
        return self.validator.validate(action, observation)
    def tick(self, current): return None
    def cancel(self, reason): pass
    def close(self): self.validator.close()


class GazeboBackend(InspectionBackend):
    kind = "gazebo"
    def __init__(self, *, enabled=False, simulation_guard=None, publish=None, clock=monotonic, **kwargs):
        super().__init__(**kwargs)
        self.enabled, self.guard, self.publish, self.clock = enabled, simulation_guard, publish, clock
        self.active = False
        self.consumed = deque(maxlen=1024)
        self.command_count = 0
        self.last_status = "IDLE"
        self.last_command = None
        self.generation = 0
        if self.validator.config.mode != "required" or not self.validator.config.workspace_enabled:
            raise VLAError("P2_REQUIRED", "Gazebo execution requires native P2 collision and workspace validation")

    def submit(self, candidate, observation, expected_generation=None):
        generation = self.generation if expected_generation is None else expected_generation
        if generation != self.generation:
            raise VLAError("REQUEST_INVALIDATED", "simulation validation was cancelled")
        # Interpretation occurs before all execution paths: unknown native actions fail here.
        action = self.interpreter.interpret(candidate)
        if not self.enabled or not self.guard or self.guard() is not True:
            raise VLAError("SIMULATION_NOT_ENABLED", "explicit simulation enablement and matching live runtime required")
        if self.active:
            raise VLAError("EXECUTOR_BUSY", "a simulation trajectory is active")
        if not observation.simulated or not candidate.synthetic:
            raise VLAError("UNCALIBRATED_EXECUTION_DISABLED", "only explicit synthetic known-semantic scenarios may execute")
        if action.action_id in self.consumed or candidate.action_id in self.consumed:
            raise VLAError("DUPLICATE_ACTION", "scenario already consumed")
        result = self.validator.validate(action, observation)
        if generation != self.generation:
            raise VLAError("REQUEST_INVALIDATED", "simulation validation was cancelled")
        if result.status != "PASS_RESEARCH_ONLY" or result.trajectory is None:
            self.last_status = "REJECTED_BY_P2"
            return result
        if not result.provenance.get("workspace_checked") or not result.discrete_only:
            raise VLAError("P2_REQUIRED", "explicit P2 research validation evidence required")
        self.consumed.append(candidate.action_id)
        self.record, self.source = result, observation
        self.start = self.clock()
        self.active = True
        self.last_command = list(observation.joints.positions)
        self.last_status = "EXECUTING_SYNTHETIC_SCENARIO"
        return result

    def tick(self, current):
        if not self.active:
            return None
        try:
            if self.guard() is not True or not current.simulated:
                raise VLAError("SIMULATION_RUNTIME_LOST", "simulation guard failed")
            for name in ("session_id", "task_revision", "scene_id", "scene_sha256", "model_id", "robot_id"):
                if getattr(current, name) != getattr(self.source, name):
                    raise VLAError("STATE_INVALIDATED", "execution context changed at " + name)
            if abs(current.gripper.position - self.source.gripper.position) > 0.02:
                raise VLAError("STATE_INVALIDATED", "gripper feedback differs from validated state")
            if max(abs(a-b) for a,b in zip(current.joints.positions, self.last_command)) > 0.02:
                raise VLAError("TRACKING_ERROR", "feedback deviates from the validated path")
            action = self.record.trajectory
            elapsed = self.clock() - self.start
            times = [0.0, *action.time_offsets_s]
            points = [self.source.joints.positions, *action.values]
            index = next((i for i in range(len(times)-1) if elapsed < times[i+1]), len(times)-2)
            fraction = min(1.0, max(0.0, (elapsed-times[index])/(times[index+1]-times[index])))
            q = [a+(b-a)*fraction for a,b in zip(points[index], points[index+1])]
            self.publish(q)
            self.command_count += 1
            self.last_command = q
            if elapsed >= times[-1]:
                error = max(abs(a-b) for a,b in zip(current.joints.positions, action.values[-1]))
                if error <= 0.015:
                    self.active = False
                    self.last_status = "SIMULATION_COMPLETED_WITH_FEEDBACK"
                    return {"status": self.last_status, "tracking_error_rad": error, "positions": current.joints.positions,
                        "source": "rule_generated_simulation_only", "hardware_validated": False}
                if elapsed > times[-1] + 2.0:
                    raise VLAError("TRACKING_TIMEOUT", "simulation did not reach target")
        except Exception:
            self.cancel("EXECUTION_FAULT")
            raise

    def cancel(self, reason):
        self.generation += 1
        self.active = False
        self.last_status = "STOPPED_" + reason
        # Position controllers retain their last setpoint. No queued waypoints resume.

    def close(self):
        self.cancel("SHUTDOWN")
        super().close()
