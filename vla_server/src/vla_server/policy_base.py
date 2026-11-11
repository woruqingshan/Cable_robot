"""Future model adapters implement this boundary and own their preprocessing."""
from abc import ABC, abstractmethod
from vla_protocol.action import PolicyActionChunk, PolicyIdentity
from vla_protocol.observation import Observation


class Policy(ABC):
    identity: PolicyIdentity
    native_action_spaces: list[str]
    model_type = "mock"
    model_loaded = False
    gpu_loaded = False
    genuine_inference_verified = False

    def start(self) -> None:
        pass

    def close(self) -> None:
        pass

    def reset(self, context=None, reason="session_change") -> None:
        """Clear model temporal state/action queues; server calls this serially."""
        pass

    @abstractmethod
    def predict(self, observation: Observation) -> PolicyActionChunk:
        """Return native candidate actions, never motor commands."""
