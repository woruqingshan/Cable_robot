"""Offline workspace tools; keep scene-only imports independent of ROS/URDF."""

from importlib import import_module

__all__ = ["KinematicModel", "ModelError", "load_model", "model_from_xml"]


def __getattr__(name: str) -> object:
    """Load the unchanged G1 exports only when explicitly requested."""
    if name in __all__:
        value = getattr(import_module('.robot.model', __name__), name)
        globals()[name] = value
        return value
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


def __dir__() -> list[str]:
    """Expose the same discoverable G1 API without importing its dependencies."""
    return sorted(set(globals()) | set(__all__))
