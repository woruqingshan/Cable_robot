"""Read-only physical sensor adaptation. No controller imports or commands."""
import copy
import json
import math
from hashlib import sha256
from pathlib import Path
from time import monotonic
from types import SimpleNamespace

from vla_protocol.errors import VLAError
from vla_protocol.observation import GripperState
from vla_protocol.profiles import JOINT_NAMES
from .integration.sources import SensorObservationSource

CACHED_TOPICS = {"/joint_states", "/controller_manager/joint_states"}


def load_evidence(path, digest):
    """Operator-reviewed evidence is external to the uncalibrated source profile."""
    if not path or not digest:
        return None
    with Path(path).open("rb") as stream:
        raw = stream.read(65537)
    if len(raw) > 65536 or sha256(raw).hexdigest() != digest:
        raise VLAError("STATE_EVIDENCE_INVALID", "state evidence SHA256/size mismatch")
    data = json.loads(raw)
    required = {"schema_version", "topic", "gripper_topic", "source_kind", "stamp_semantics",
                "clock_domain", "joint_mapping", "gripper_name", "units", "evidence_reference"}
    if set(data) != required or type(data["schema_version"]) is not int or data["schema_version"] != 1:
        raise VLAError("STATE_EVIDENCE_INVALID", "unsupported state evidence schema")
    if (data["source_kind"] != "measured_feedback" or data["units"] != "rad"
            or data["stamp_semantics"] != "driver_acquisition"
            or data["clock_domain"] != "ros_system" or not data["evidence_reference"]):
        raise VLAError("UNTRUSTED_STATE", "measured feedback and verified source timestamps required")
    mapping = data["joint_mapping"]
    if (set(mapping) != set(JOINT_NAMES) or len(set(mapping.values())) != 5
            or not all(isinstance(n, str) and n for n in mapping.values())
            or not isinstance(data["gripper_name"], str) or not data["gripper_name"]):
        raise VLAError("STATE_EVIDENCE_INVALID", "explicit unique five-joint mapping required")
    if not data["gripper_topic"] and data["gripper_name"] in mapping.values():
        raise VLAError("STATE_EVIDENCE_INVALID", "gripper cannot alias an arm joint")
    if data["topic"] in CACHED_TOPICS or data["gripper_topic"] in CACHED_TOPICS:
        raise VLAError("CACHED_COMMAND_STATE", "current vendor topics contain command cache, not measurements")
    return data


def rgb_message(message):
    """Explicit RGB/BGR conversion; preserve header and reject unsupported layouts."""
    w, h = message.width, message.height
    if (message.encoding not in ("rgb8", "bgr8") or message.is_bigendian != 0
            or not 0 < w <= 1024 or not 0 < h <= 1024 or w*h > 1048576
            or message.step < w*3 or len(message.data) != message.step*h):
        raise VLAError("INVALID_IMAGE", "require bounded RGB8/BGR8 rows with exact stride/data size")
    raw = bytes(message.data)
    packed = b"".join(raw[i*message.step:i*message.step+w*3] for i in range(h))
    if message.encoding == "bgr8":
        rgb = bytearray(len(packed))
        rgb[0::3], rgb[1::3], rgb[2::3] = packed[2::3], packed[1::3], packed[0::3]
        packed = bytes(rgb)
    return SimpleNamespace(header=copy.deepcopy(message.header), encoding="rgb8", is_bigendian=0,
                           width=w, height=h, step=w*3, data=packed)


def positions(message):
    if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
        raise VLAError("INVALID_JOINT_STATE", "duplicate/unpaired state names")
    values = dict(zip(message.name, message.position))
    if not all(math.isfinite(v) for v in values.values()):
        raise VLAError("INVALID_JOINT_STATE", "nonfinite state")
    return values


class RealObservationCore:
    def __init__(self, *, state_topic, gripper_topic="", evidence=None, clock=monotonic, **context):
        self.source = SensorObservationSource("real", clock=clock, **context)
        self.clock, self.evidence = clock, evidence
        self.state_topic, self.gripper_topic = state_topic, gripper_topic
        self.gripper_sample = None
        self.image_provenance = None
        self.last_error = "WAITING_FOR_SENSORS"

    def clear_state(self):
        self.source.joint_history.clear()
        self.source.collector.parts.pop("joints", None)
        self.source.collector.parts.pop("gripper", None)

    def trust(self):
        if self.state_topic in CACHED_TOPICS or self.gripper_topic in CACHED_TOPICS:
            raise VLAError("CACHED_COMMAND_STATE", "vendor joint state is cached commanded position")
        if (not self.evidence or self.evidence["topic"] != self.state_topic
                or self.evidence["gripper_topic"] != self.gripper_topic):
            raise VLAError("UNTRUSTED_STATE", "reviewed measured-feedback evidence is required")

    def gripper(self, message):
        try:
            self.trust()
            values = positions(message)
            name = self.evidence["gripper_name"]
            if name not in values:
                raise VLAError("MISSING_GRIPPER", "measured gripper state absent")
            self.gripper_sample = (GripperState(name="r_joint", position=float(values[name]), unit="rad",
                stamp=self.source.stamp(message), simulated=False, calibrated=False), self.clock())
        except Exception:
            self.gripper_sample = None
            self.clear_state()
            raise

    def joints(self, message):
        try:
            self.trust()
            values = positions(message)
            mapping = self.evidence["joint_mapping"]
            if not set(mapping.values()) <= values.keys():
                raise VLAError("MISSING_JOINT_STATE", "measured arm joint absent")
            received = self.clock()
            if self.gripper_topic:
                if self.gripper_sample is None:
                    raise VLAError("MISSING_GRIPPER", "waiting for independent measured gripper")
                gripper, grip_received = self.gripper_sample
                received = min(received, grip_received)
            else:
                name = self.evidence["gripper_name"]
                if name not in values:
                    raise VLAError("MISSING_GRIPPER", "measured gripper state absent")
                gripper = GripperState(name="r_joint", position=float(values[name]), unit="rad",
                    stamp=self.source.stamp(message), simulated=False, calibrated=False)
            mapped = SimpleNamespace(header=copy.deepcopy(message.header), name=JOINT_NAMES+["r_joint"],
                position=[float(values[mapping[n]]) for n in JOINT_NAMES]+[gripper.position])
            self.source.joint_state(mapped, gripper_state=gripper, received_at=received)
        except Exception:
            self.clear_state()
            raise

    def image(self, message):
        try:
            converted = rgb_message(message)
            self.source.image(converted)
            self.image_provenance = {"original_encoding": message.encoding, "original_step": message.step,
                "transport_encoding": "rgb8", "conversion": "bgr_to_rgb" if message.encoding == "bgr8" else "rgb_rows",
                "source_stamp_ns": self.source.stamp(message).nanoseconds, "frame_id": message.header.frame_id}
        except Exception:
            self.source.collector.parts.pop("image", None)
            self.image_provenance = None
            raise

    def snapshot(self, sequence, now_ros_ns):
        self.trust()
        return self.source.snapshot(sequence, now_ros_ns)

    def close(self):
        self.source.close()
