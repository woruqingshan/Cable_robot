"""Bounded JSON with duplicate-key rejection and stable validation errors."""
import json
import math
from pydantic import ValidationError
from .errors import VLAError
from .profiles import MAX_BODY_BYTES, RuntimeLimits


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise VLAError("INVALID_JSON", "duplicate JSON key: " + key)
        result[key] = value
    return result


def loads(data: bytes | str, model, limits: RuntimeLimits | None = None):
    limits = limits or RuntimeLimits()
    if len(data.encode() if isinstance(data, str) else data) > limits.max_body_bytes:
        raise VLAError("PAYLOAD_TOO_LARGE", "JSON exceeds configured body limit")
    try:
        def constant(_):
            raise VLAError("NONFINITE_NUMBER", "nonfinite JSON constants are forbidden")
        value = json.loads(data, object_pairs_hook=_object, parse_constant=constant)
    except (ValueError, UnicodeDecodeError, RecursionError) as exc:
        if isinstance(exc, VLAError):
            raise
        raise VLAError("INVALID_JSON", "invalid JSON encoding") from exc
    return validate(value, model, limits)


def validate(value, model, limits: RuntimeLimits | None = None):
    limits = limits or RuntimeLimits()
    def finite_tree(item):
        if isinstance(item, float) and not math.isfinite(item):
            raise VLAError("NONFINITE_NUMBER", "nonfinite numeric value")
        if isinstance(item, dict):
            for child in item.values():
                finite_tree(child)
        elif isinstance(item, (list, tuple)):
            for child in item:
                finite_tree(child)
    try:
        finite_tree(value)
    except RecursionError as exc:
        raise VLAError("INVALID_SCHEMA", "excessive nesting") from exc
    try:
        result = model.model_validate(value)
    except ValidationError as exc:
        for error in exc.errors():
            original = error.get("ctx", {}).get("error")
            if isinstance(original, VLAError):
                raise original from exc
        errors = exc.errors(include_input=False, include_context=False)
        first = errors[0]
        field = ".".join(str(p) for p in first["loc"])
        code = "UNSUPPORTED_VERSION" if first["loc"] == ("schema_version",) else "INVALID_SCHEMA"
        if first["type"] == "finite_number":
            code = "NONFINITE_NUMBER"
        raise VLAError(code, field + ": " + first["msg"], request_id=value.get("request_id") if isinstance(value, dict) and isinstance(value.get("request_id"), str) else None) from exc
    for image in getattr(result, "images", []):
        if len(image.payload()) > limits.max_image_bytes:
            raise VLAError("PAYLOAD_TOO_LARGE", "configured image limit")
    shape = getattr(result, "shape", None)
    if shape and shape[0] > limits.max_horizon:
        raise VLAError("ACTION_SHAPE_MISMATCH", "configured horizon limit")
    return result


def wire_value(value):
    """Keep default v1 messages readable by strict Stage 1 peers.

    Required nullable identity fields remain present. Only newly optional
    extension fields are omitted when inactive; opt-in extensions are explicit.
    """
    from .observation import Observation
    from .action import PolicyActionChunk, RobotActionChunk, ValidatedTrajectory
    data = value.model_dump(mode="json")
    if isinstance(value, Observation):
        if value.source_profile is None: data.pop("source_profile", None)
        if not value.capabilities: data.pop("capabilities", None)
    if isinstance(value, (PolicyActionChunk, RobotActionChunk)) and value.horizon is None:
        data.pop("horizon", None)
    if isinstance(value, PolicyActionChunk) and value.session_generation is None:
        data.pop("session_generation", None)
    if isinstance(value, ValidatedTrajectory) and value.trajectory is not None:
        data["trajectory"] = wire_value(value.trajectory)
    return data


def dumps(value, limits: RuntimeLimits | None = None) -> bytes:
    result = json.dumps(wire_value(value), allow_nan=False, separators=(",", ":")).encode()
    if len(result) > (limits or RuntimeLimits()).max_body_bytes:
        raise VLAError("PAYLOAD_TOO_LARGE", "encoded JSON exceeds body limit")
    return result
