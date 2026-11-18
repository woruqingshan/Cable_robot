"""Local interfaces; simulation and future sensors share the unchanged wire protocol."""
from typing import Protocol
from vla_protocol.observation import Observation
from vla_protocol.action import PolicyActionChunk, RobotActionChunk, ValidatedTrajectory
from vla_protocol.errors import VLAError
from ..action_adapter import DEFAULT_REGISTRY


class ObservationSource(Protocol):
    kind: str
    def snapshot(self, sequence: int, now_ros_ns: int) -> Observation: ...
    def close(self) -> None: ...


class ActionInterpreter(Protocol):
    def interpret(self, candidate: PolicyActionChunk) -> RobotActionChunk: ...


class TrajectoryValidator(Protocol):
    def validate(self, action: RobotActionChunk, observation: Observation) -> ValidatedTrajectory: ...
    def close(self) -> None: ...


class ExecutionBackend(Protocol):
    kind: str
    def submit(self, candidate: PolicyActionChunk, observation: Observation): ...
    def tick(self, current: Observation): ...
    def cancel(self, reason: str): ...
    def close(self) -> None: ...


class RegisteredActionInterpreter:
    """Unknown native outputs stay inspectable and never acquire robot semantics."""
    def __init__(self, registry=None):
        self.registry = registry or DEFAULT_REGISTRY

    def interpret(self, candidate):
        return self.registry.adapt(candidate)


def execution_backend(kind, **kwargs):
    from .execution import InspectionBackend, GazeboBackend
    if kind == "inspection":
        return InspectionBackend(**kwargs)
    if kind == "gazebo":
        return GazeboBackend(**kwargs)
    raise VLAError("HARDWARE_EXECUTION_DISABLED", "no physical execution adapter is implemented or enabled")
