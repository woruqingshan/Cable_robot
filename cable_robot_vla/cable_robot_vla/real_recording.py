"""Exclusive physical observation runs and offline protocol replay; no execution."""
import base64
import json
from hashlib import sha256
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from vla_protocol import codec
from vla_protocol.errors import VLAError
from vla_protocol.observation import Observation


def json_text(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


class RealRecorder:
    def __init__(self, root, context, *, run_name=None):
        name = run_name or "run_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ_") + uuid4().hex
        if Path(name).name != name or name in (".", ".."):
            raise ValueError("invalid run name")
        self.path = Path(root).expanduser()/name
        self.path.mkdir(parents=True, exist_ok=False)
        for folder in ("images", "calibration", "diagnostics"):
            (self.path/folder).mkdir()
        manifest = {"recording_schema": "idc_real_recording_v1", "mode": "shadow_only",
            "robot_execution_enabled": False, "profile": "real_uncalibrated_v1",
            "created_utc": datetime.now(timezone.utc).isoformat(), "context": context}
        (self.path/"manifest.json").write_text(json_text(manifest)+"\n")
        calibration = {"schema_version": 1, "calibrated": False, "version": "unavailable",
            "joint_zero": "unknown", "TCP": "unknown", "camera_extrinsics": "unknown",
            "physical_home": "unknown", "evidence": []}
        (self.path/"calibration/calibration_manifest.json").write_text(json_text(calibration)+"\n")
        manifest["calibration_manifest_sha256"] = sha256((self.path/"calibration/calibration_manifest.json").read_bytes()).hexdigest()
        (self.path/"manifest.json").write_text(json_text(manifest)+"\n")
        self.observations = (self.path/"observations.jsonl").open("x")
        self.events = (self.path/"diagnostics/events.jsonl").open("x")
        self.session_id = None
        self.last_seq = -1
        self.seen = set()

    def event(self, kind, **data):
        self.events.write(json_text({"kind": kind, "recorded_utc": datetime.now(timezone.utc).isoformat(), **data})+"\n")
        self.events.flush()

    def record(self, observation, provenance):
        observation = codec.validate(observation.model_dump(), Observation)
        if observation.simulated or observation.calibration_status != "real_uncalibrated":
            raise VLAError("RECORDING_PROFILE_INVALID", "only real uncalibrated observations accepted")
        if observation.observation_seq <= self.last_seq or observation.request_id in self.seen:
            raise VLAError("DUPLICATE_OBSERVATION", "sequence/request must be unique and increasing")
        if self.session_id is not None and observation.session_id != self.session_id:
            raise VLAError("SESSION_CHANGED", "start a new recording run after source session reset")
        data = observation.model_dump()
        for index, image in enumerate(observation.images):
            relative = f"images/frame_{observation.observation_seq:08d}_{index}.png"
            with (self.path/relative).open("xb") as output:
                output.write(image.payload())
            data["images"][index].pop("data_base64")
            data["images"][index]["png_path"] = relative
        self.observations.write(json_text({"recording_schema": "idc_real_recording_v1", "valid": True,
            "observation": data, "provenance": provenance})+"\n")
        self.observations.flush()
        self.session_id = observation.session_id
        self.last_seq = observation.observation_seq
        self.seen.add(observation.request_id)

    def close(self):
        self.observations.close()
        self.events.close()


def replay(run):
    """Verify recorded bytes, hashes, profile and identity, without sending requests."""
    root = Path(run).resolve()
    manifest = json.loads((root/"manifest.json").read_text())
    if (manifest["recording_schema"] != "idc_real_recording_v1"
            or manifest["robot_execution_enabled"] is not False):
        raise VLAError("INVALID_RECORDING", "unsupported recording")
    calibration = root/"calibration/calibration_manifest.json"
    if sha256(calibration.read_bytes()).hexdigest() != manifest["calibration_manifest_sha256"]:
        raise VLAError("INVALID_RECORDING", "calibration manifest hash mismatch")
    last_seq, seen, session = -1, set(), None
    with (root/"observations.jsonl").open() as stream:
        for line in stream:
            if len(line) > 6000000:
                raise VLAError("INVALID_RECORDING", "oversized recording entry")
            row = json.loads(line)
            if row["recording_schema"] != "idc_real_recording_v1" or row["valid"] is not True:
                raise VLAError("INVALID_RECORDING", "invalid sample in complete observation stream")
            data = row["observation"]
            for image in data["images"]:
                relative = Path(image.pop("png_path"))
                path = (root/relative).resolve()
                if relative.is_absolute() or not path.is_relative_to(root/"images") or path.stat().st_size > 1048576:
                    raise VLAError("INVALID_RECORDING", "unsafe/oversized PNG reference")
                image["data_base64"] = base64.b64encode(path.read_bytes()).decode()
            observation = codec.validate(data, Observation)
            if observation.simulated or observation.calibration_status != "real_uncalibrated":
                raise VLAError("INVALID_RECORDING", "real uncalibrated profile required")
            if (observation.observation_seq <= last_seq or observation.request_id in seen
                    or session is not None and observation.session_id != session):
                raise VLAError("INVALID_RECORDING", "duplicate/out-of-order/mixed session")
            last_seq, session = observation.observation_seq, observation.session_id
            seen.add(observation.request_id)
            yield codec.loads(codec.dumps(observation), Observation)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Offline verification only; never executes or sends actions")
    parser.add_argument("run")
    args = parser.parse_args()
    count = sum(1 for _ in replay(args.run))
    print(json_text({"verified_observations": count, "hardware_validation": False, "execution": False}))


if __name__ == "__main__":
    main()
