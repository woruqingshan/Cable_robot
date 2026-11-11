"""Configuration selects a policy; optional frameworks stay lazy."""
from .config import PolicyConfig


def create_policy(config: PolicyConfig):
    if config.type == "mock":
        from .policies.mock import HoldPositionPolicy
        return HoldPositionPolicy()
    if config.type == "fake":
        from .policies.fake import FakeNativePolicy
        return FakeNativePolicy(config)
    from .policies.smolvla import SmolVLAPolicyAdapter
    return SmolVLAPolicyAdapter(config)
