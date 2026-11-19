"""Inert inference evidence. There is deliberately no ROS/controller dependency."""
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4
import json
import math


class InferenceDiagnostics:
    def __init__(self, directory):
        root = Path(directory).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self.path = root/("inference_"+uuid4().hex+".jsonl")
        with self.path.open("x"):
            pass

    def success(self, observation, result, *, started_utc, duration_s, policy):
        row = {"diagnostic_schema": "idc_shadow_inference_v1", "kind": "inference_result",
            "request_id": observation.request_id, "session_id": observation.session_id,
            "observation_seq": observation.observation_seq, "robot_id": observation.robot_id,
            "model_id": observation.model_id, "source_profile": observation.source_profile.model_dump() if observation.source_profile else None,
            "observation_timestamps": {"joints": observation.joints.stamp.model_dump(), "gripper": observation.gripper.stamp.model_dump()},
            "images": [image.model_dump(exclude={"data_base64"}) for image in observation.images],
            "joints": observation.joints.model_dump(), "gripper": observation.gripper.model_dump(),
            "started_utc": started_utc, "completed_utc": datetime.now(timezone.utc).isoformat(),
            "http_duration_s": duration_s, "policy_duration_s": result.inference_duration_s,
            "device": getattr(getattr(policy, "settings", None), "device", "not_applicable"),
            "runtime": getattr(policy, "runtime_info", {}), "transport_shape": list(result.shape),
            "finite": all(math.isfinite(v) for step in result.values for v in step),
            "output_stage": "native_policy_response_after_official_postprocessor",
            "action": result.model_dump(mode="json"), "execution_enabled": False}
        self._write(row)

    def error(self, *, request_id, code):
        self._write({"diagnostic_schema": "idc_shadow_inference_v1", "kind": "inference_error",
            "request_id": request_id, "code": code, "recorded_utc": datetime.now(timezone.utc).isoformat(),
            "execution_enabled": False})

    def _write(self, row):
        with self.path.open("a") as stream:
            stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(",", ":"))+"\n")
