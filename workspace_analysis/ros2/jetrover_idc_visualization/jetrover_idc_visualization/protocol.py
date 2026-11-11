"""Small versioned GUI protocol; no hardware action/service names or motion goals."""
from .errors import ReplayError
from .storage import json_object

NAMESPACE = '/idc_replay'
JOINT_TOPIC = NAMESPACE+'/joint_states'
SCENE_TOPIC = NAMESPACE+'/scene_markers'
PORT_TOPIC = NAMESPACE+'/port_markers'
COMMAND_TOPIC = NAMESPACE+'/command'
STATUS_TOPIC = NAMESPACE+'/status'


def command(text: str, session_id: str, revision: int) -> dict:
    if not isinstance(text,str) or len(text.encode()) > 4096:
        raise ReplayError('COMMAND', 'bounded JSON string required')
    data = json_object(text)
    base = {'version','session_id','request_id','selection_revision','action'}
    action = data.get('action')
    fields = base | ({'port_id','goal_id','planner','phase'} if action == 'select' else set())
    if set(data) != fields or type(data.get('version')) is not int or data['version'] != 1:
        raise ReplayError('COMMAND', 'protocol fields/version mismatch')
    if data['session_id'] != session_id: raise ReplayError('STALE_SESSION', 'reconnect to current replay session')
    if type(data['selection_revision']) is not int or data['selection_revision'] != revision:
        raise ReplayError('STALE_SELECTION', 'selection changed; use current status')
    if not isinstance(data['request_id'],str) or not 1 <= len(data['request_id']) <= 80:
        raise ReplayError('COMMAND', 'request ID required')
    if action not in ('select','play','pause','resume','reset'):
        raise ReplayError('COMMAND', 'unsupported replay action')
    if action == 'select' and (not isinstance(data['port_id'],str) or
            data['goal_id'] is not None and type(data['goal_id']) is not int):
        raise ReplayError('COMMAND', 'port/goal types invalid')
    return data
