"""Shared collision failures without protocol/configuration dependencies."""


class CollisionError(RuntimeError):
    """Runtime, identity or protocol failure; never interpreted as CLEAR."""
