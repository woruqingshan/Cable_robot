"""JSONL episode logger for cable insertion experiments."""

import json
import os
import time
import uuid
from typing import Any, Dict, Optional


class ExperimentLogger:
    def __init__(self, log_dir: str = 'episodes', episode_id: Optional[str] = None):
        self.episode_id = episode_id or time.strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8]
        self.log_dir = log_dir
        os.makedirs(self.log_dir, exist_ok=True)
        self.path = os.path.join(self.log_dir, f'{self.episode_id}.jsonl')

    def record(self, **kwargs: Any) -> None:
        row: Dict[str, Any] = {
            'episode_id': self.episode_id,
            'timestamp': time.time(),
            'stage': None,
            'waypoint': None,
            'target_joint_pulse': None,
            'current_joint_pulse': None,
            'target_pose': None,
            'actual_pose': None,
            'duration': None,
            'success': None,
            'error_reason': None,
            'recovery_count': 0,
            'observation_bytes': None,
            'semantic_bytes': None,
            'network_latency_ms': None,
            'vla_latency_ms': None,
            'end_to_end_latency_ms': None,
            'alignment_error': None,
            'communication_mode': None,
        }
        row.update(kwargs)
        with open(self.path, 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(row, sort_keys=True) + '\n')
