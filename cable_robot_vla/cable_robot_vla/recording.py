"""JSONL with image hashes/descriptors; payloads are never embedded in logs."""
from datetime import datetime, timezone
import json
from pathlib import Path


class LoopRecorder:
    def __init__(self, path):
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        self.stream = target.open("x", encoding="utf-8")

    def record(self, event, **fields):
        row = {"schema_version": 1, "event": event,
               "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
               "simulation_only": True, "hardware_validated": False, **fields}
        self.stream.write(json.dumps(row, allow_nan=False, sort_keys=True) + "\n")
        self.stream.flush()

    def observation(self, observation):
        data = observation.model_dump()
        for image in data["images"]:
            image.pop("data_base64")
        self.record("observation", **data)

    def close(self):
        self.stream.close()
