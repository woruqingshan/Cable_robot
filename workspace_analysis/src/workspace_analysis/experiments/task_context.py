"""Explicit borrowed planning context; caller owns the Native evaluator session."""
from dataclasses import dataclass
import json
from typing import Any
from ..task.spec import TaskError, parse_task
from ..task.targets import TaskTargets, build_targets
from ..scene.model import SceneSnapshot
from ..robot.model import KinematicModel
from ..planning.validity import ConfigurationValidity


class TaskSystemError(RuntimeError):
    """Backend/identity/unknown-result failure, never ordinary task rejection."""


@dataclass(frozen=True)
class TaskContext:
    """Borrowed references, used serially. No process/model is created here."""
    model: KinematicModel
    scene: SceneSnapshot
    targets: TaskTargets
    checker: ConfigurationValidity

    def validate(self) -> None:
        spec = self.targets.spec
        parse_task(spec.data)
        if self.model.base_frame != spec.data['target']['reference_frame'] or self.model.ee_frame != spec.data['target']['ee_frame']:
            raise TaskError('task/model frame mismatch')
        if build_targets(spec, self.scene).to_dict() != self.targets.to_dict():
            raise TaskError('derived targets/scene identity mismatch')
        evaluator = self.checker.evaluator
        if evaluator.model is not self.model or evaluator.scene.canonical_scene_sha256 != self.scene.canonical_scene_sha256:
            raise TaskError('borrowed evaluator model/scene mismatch')
        collision = spec.data['collision']
        if evaluator.gripper != collision['gripper_rad'] or evaluator.clearance != {
                'self_m': collision['self_clearance_m'], 'environment_m': collision['environment_clearance_m']}:
            raise TaskError('task/evaluator clearance or gripper mismatch')
        if evaluator.semantic is None:
            raise TaskError('explicit validated SRDF required')
        wire = json.loads(evaluator.semantic.payload_json)
        expected = {'urdf_sha256': self.model.metadata()['expanded_urdf_sha256'],
                    'scene_sha256': self.scene.canonical_scene_sha256,
                    'srdf_sha256': wire['srdf_sha256'], 'semantic_review_sha256': wire['review_sha256']}
        if any(evaluator.identity.get(k) != v for k, v in expected.items()):
            raise TaskSystemError('Native/model/scene/SRDF identity mismatch')


def require_resolved(records: list[dict[str, Any]]) -> None:
    """Unknown native results must not be counted as normal planner failures."""
    for row in records:
        if row.get('status') == 'ERROR' or any(reason in row.get('reasons', []) for reason in
                ['COLLISION_QUERY_ERROR', 'CLEARANCE_OR_COLLISION_UNVERIFIED',
                 'INCONSISTENT_SEMANTIC_RESULT', 'INVALID_VALIDITY_FIELD']):
            raise TaskSystemError('unresolved collision query: ' + str(row))


def select_common_start(checker: ConfigurationValidity, q_samples: Any) -> dict:
    """First valid original G4 row, once, independent of any task outcome."""
    import numpy as np
    values = np.asarray(q_samples)
    if values.ndim != 2 or values.shape[1:] != (5,) or len(values) == 0:
        raise TaskError('nonempty original q[N,5] required')
    for q in values:
        checker.model.validate_q(q)
    tested = []
    for index, q in enumerate(values):
        state = checker.check(q, fresh=True)
        require_resolved([state]); tested.append(state)
        if state['valid']:
            return {'status': 'VALID_SIMULATION_START', 'q': q.tolist(),
                    'original_sample_index': index, 'tested': tested,
                    'not_a_physical_home': True, 'rule': 'first valid original ascending G4 row'}
    return {'status': 'INVALID_START', 'q': None, 'original_sample_index': None,
            'tested': tested, 'not_a_physical_home': True}
