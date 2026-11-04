"""Validated per-port derivation of a task template; no world-coordinate tables."""
from .spec import TaskSpec, TaskError, parse_task
from .targets import build_targets
from ..scene.model import SceneSnapshot


def tasks_for_ports(template: TaskSpec, scene: SceneSnapshot,
                    port_ids: list[str]) -> list[TaskSpec]:
    """Retain the template's identity only for its original port.

    Other tasks carry distinct canonical IDs/hashes. All mounting, tolerance and
    solver assumptions are inherited unchanged; targets still come from the scene.
    """
    if not isinstance(port_ids, (list, tuple)) or not port_ids:
        raise TaskError('nonempty explicit port list required')
    if any(not isinstance(p, str) for p in port_ids) or len(set(port_ids)) != len(port_ids):
        raise TaskError('invalid or duplicate port ID')
    # Validate even directly constructed TaskSpec objects, not only YAML inputs.
    parse_task(template.data)
    result = []
    for port in port_ids:
        data = template.data
        if port != data['target']['port_id']:
            data['id'] = template.data['id'] + '__' + port
            data['target']['port_id'] = port
            spec = parse_task(data)
        else:
            spec = template
        build_targets(spec, scene)  # rejects missing/ambiguous ports and frames
        result.append(spec)
    return result
