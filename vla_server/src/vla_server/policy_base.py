"""Future model adapters implement this boundary and own their preprocessing."""
from abc import ABC, abstractmethod
from vla_protocol.action import PolicyActionChunk, PolicyIdentity
from vla_protocol.observation import Observation


class Policy(ABC):
    identity: PolicyIdentity
    native_action_spaces: list[str]

    def start(self) -> None:
        pass

    def close(self) -> None:
        pass

    @abstractmethod
    def predict(self, observation: Observation) -> PolicyActionChunk:
        """Return native candidate actions, never motor commands."""
