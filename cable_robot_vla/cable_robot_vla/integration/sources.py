"""Timestamped ROS sensor collection, independent of camera/vendor drivers."""
from collections import deque
import base64
from hashlib import sha256
from time import monotonic
from uuid import uuid4
from vla_protocol.observation import ImagePayload, JointPositions, GripperState, SourceStamp, ObservationSource
from vla_protocol.profiles import JOINT_NAMES
from vla_protocol.png import encode_rgb
from vla_protocol.errors import VLAError
from ..observation_node import ObservationAdapter, StateMetadata


class SensorObservationSource:
    def __init__(self, kind, *, scene_id, scene_sha256, model_id, instruction, limits=None,
                 camera_id="mock_rgb", clock=monotonic, session_id=None):
        if kind not in ("synthetic", "gazebo", "real"):
            raise VLAError("INVALID_OBSERVATION_SOURCE", "choose synthetic, gazebo or an explicit real sensor adapter")
        if not instruction.strip():
            raise VLAError("INVALID_TASK", "nonblank language instruction required")
        self.kind, self.camera_id = kind, camera_id
        self.clock_domain = "ros_sim" if kind == "gazebo" else "ros_system"
        self.profile = ObservationSource(profile_id="real_uncalibrated_v1" if kind == "real" else "synthetic_uncalibrated_v1")
        self.collector = ObservationAdapter(limits, clock=clock, source_profile=self.profile)
        self.scene_id, self.scene_sha256, self.model_id = scene_id, scene_sha256, model_id
        self.instruction, self.task_revision, self.state_revision = instruction, 0, 0
        self.session_id = session_id or str(uuid4())
        self.last_q = None
        self.last_joint_stamp = None
        self.frame_id = None
        self.joint_history = deque(maxlen=1024)
        self.clock = clock

    def stamp(self, message):
        if message.header.stamp.sec < 0 or not 0 <= message.header.stamp.nanosec < 1_000_000_000:
            raise VLAError("INVALID_SENSOR_STAMP", "invalid ROS source timestamp")
        return SourceStamp(nanoseconds=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
            clock_domain=self.clock_domain)

    def joint_state(self, message):
        try:
            if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
                raise VLAError("INVALID_JOINT_STATE", "joint names/positions must be unique and paired")
            values = dict(zip(message.name, message.position))
            if not set([*JOINT_NAMES, "r_joint"]) <= values.keys():
                raise VLAError("MISSING_JOINT_STATE", "arm and gripper state required")
            stamp = self.stamp(message)
            if self.last_joint_stamp is not None and stamp.nanoseconds < self.last_joint_stamp.nanoseconds:
                self.session_id = str(uuid4())
                self.joint_history.clear()
                self.collector.parts.clear()
                self.last_q, self.state_revision = None, 0
            joints = JointPositions(names=list(JOINT_NAMES), positions=[float(values[n]) for n in JOINT_NAMES], unit="rad", stamp=stamp)
            gripper = GripperState(name="r_joint", position=float(values["r_joint"]), unit="rad", stamp=stamp,
                simulated=self.kind != "real", calibrated=False)
            if self.last_q is not None and max(abs(a-b) for a,b in zip(joints.positions, self.last_q)) > 0.0001:
                self.state_revision += 1
            self.last_q, self.last_joint_stamp = list(joints.positions), stamp
            self.collector.offer("joints", joints)
            self.collector.offer("gripper", gripper)
            self.joint_history.append((joints, gripper, self.clock(), self.state_revision))
        except Exception:
            self.collector.parts.pop("joints", None)
            self.collector.parts.pop("gripper", None)
            self.joint_history.clear()
            raise

    def image(self, message):
        try:
            if message.encoding != "rgb8" or message.is_bigendian != 0 or not message.header.frame_id:
                raise VLAError("INVALID_IMAGE", "require rgb8, little-endian and an explicit sensor frame")
            if not 0 < message.width * message.height <= 1_048_576 or message.step < message.width * 3 or len(message.data) != message.step * message.height:
                raise VLAError("INVALID_IMAGE", "image dimensions/stride exceed schema")
            raw = bytes(message.data)
            rgb = b"".join(raw[row * message.step:row * message.step + message.width * 3] for row in range(message.height))
            png = encode_rgb(message.width, message.height, rgb)
            self.collector.offer("image", ImagePayload(camera_id=self.camera_id, frame_id=message.header.frame_id,
                stamp=self.stamp(message), width=message.width, height=message.height, codec="png", pixel_encoding="rgb8",
                sha256=sha256(png).hexdigest(), data_base64=base64.b64encode(png).decode()))
            self.frame_id = message.header.frame_id
        except Exception:
            self.collector.parts.pop("image", None)
            raise

    def set_instruction(self, text):
        if not isinstance(text, str) or not text.strip() or len(text) > 2048:
            raise VLAError("INVALID_TASK", "require a bounded nonblank instruction")
        if text != self.instruction:
            self.instruction = text
            self.task_revision += 1

    def snapshot(self, sequence, now_ros_ns):
        if self.last_joint_stamp is None:
            raise VLAError("OBSERVATION_INCOMPLETE", "waiting for joint state")
        if not self.joint_history or self.clock() - self.joint_history[-1][2] > self.collector.limits.max_observation_age_s:
            raise VLAError("STALE_OBSERVATION", "joint stream is stale")
        image_entry = self.collector.parts.get("image")
        if image_entry is None:
            raise VLAError("OBSERVATION_INCOMPLETE", "waiting for camera stream")
        # Pair real samples near the camera stamp; never replace source timestamps.
        image_stamp = image_entry[0].stamp.nanoseconds
        joints, gripper, received, revision = min(self.joint_history,
            key=lambda row: abs(row[0].stamp.nanoseconds - image_stamp))
        self.collector.parts["joints"] = (joints, received)
        self.collector.parts["gripper"] = (gripper, received)
        self.collector.offer("metadata", StateMetadata(schema_version=1, session_id=self.session_id,
            task_revision=self.task_revision, state_revision=revision, robot_id="jetrover_" + self.kind,
            scene_id=self.scene_id, scene_sha256=self.scene_sha256, model_id=self.model_id,
            instruction=self.instruction, stamp=joints.stamp, busy=False))
        observation = self.collector.snapshot(sequence, now_ros_ns)
        return observation.model_copy(update={"capabilities": ["native_actions_v1", "session_state_v1", "source_profiles_v1"]})

    def close(self):
        self.collector.parts.clear()
        self.joint_history.clear()
