"""Gazebo/sensor -> existing HTTP client -> native inspection; separate scenario control."""
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha256
import json
import math
import os
from pathlib import Path
from time import monotonic
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String, Float64, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from vla_protocol.errors import VLAError
from vla_protocol.profiles import RuntimeLimits, JOINT_NAMES
from ..policy_client import PolicyClient
from ..adapters.p2 import P2Adapter, P2Config
from ..recording import LoopRecorder
from ..ros_support import require_isolation, parameter
from .sources import SensorObservationSource
from .interfaces import execution_backend
from .execution import known_simulation_candidate
from .gazebo import PREFIX


class SimulationIntegrationNode(Node):
    def __init__(self, **kwargs):
        require_isolation()
        super().__init__("vla_simulation", namespace=PREFIX, **kwargs)
        if self.get_namespace() != PREFIX or not self.get_parameter("use_sim_time").value:
            raise VLAError("SIMULATION_ISOLATION", "Gazebo node requires private namespace and simulation time")
        self.runtime_path = Path(parameter(self, "runtime_path", "")).resolve()
        self.runtime = json.loads(self.runtime_path.read_text())
        if self.runtime["namespace"] != PREFIX or int(os.environ["ROS_DOMAIN_ID"]) != self.runtime["domain_id"]:
            raise VLAError("SIMULATION_ISOLATION", "namespace/domain differs from generated runtime")
        if os.environ.get("IGN_PARTITION") != self.runtime["partition"]:
            raise VLAError("SIMULATION_ISOLATION", "Gazebo transport partition differs")
        for name in ["camera/image", "joint_states", "instruction", "status", "native_actions", "native_values", "validation", "markers", "observation_state", *["control/" + n + "/position" for n in JOINT_NAMES]]:
            if self.resolve_topic_name(name) != PREFIX + "/" + name:
                raise VLAError("SIMULATION_ISOLATION", "topic remap escaped its exact simulation endpoint")
        self.limits = RuntimeLimits(max_horizon=64, request_timeout_s=30.0, max_sync_skew_s=0.08)
        self.validator = P2Adapter(P2Config.model_validate(self.runtime["p2_config"]))
        self.validator._load(0.3)
        if self.validator.model.metadata()["expanded_urdf_sha256"] != self.runtime["source_model_sha256"]:
            raise VLAError("P2_IDENTITY_MISMATCH", "runtime source model differs")
        if sha256((self.runtime_path.parent / "source_robot.urdf").read_bytes()).hexdigest() != self.runtime["source_model_sha256"]:
            raise VLAError("P2_IDENTITY_MISMATCH", "display/simulation source URDF differs")
        if self.validator.evaluator.identity != self.runtime["p2_identity"]:
            raise VLAError("P2_IDENTITY_MISMATCH", "runtime P2 backend/scene/semantic identity differs")
        self.source = SensorObservationSource("gazebo", scene_id=self.runtime["scene_id"],
            scene_sha256=self.runtime["scene_sha256"], model_id="urdf:" + self.runtime["source_model_sha256"],
            instruction=parameter(self, "instruction", "Inspect the simulated robot and cabinet; native action semantics are unknown"), limits=self.limits)
        self.client = PolicyClient(parameter(self, "server_url", "http://127.0.0.1:18000"), limits=self.limits)
        mode = parameter(self, "execution_backend", "inspection")
        enabled = parameter(self, "enable_simulation", False)
        # Inspection has no command publishers at all.
        self.control = {n: self.create_publisher(Float64, "control/" + n + "/position", 1) for n in JOINT_NAMES} if mode == "gazebo" and enabled else {}
        self.backend = execution_backend(mode, validator=self.validator, enabled=enabled,
            simulation_guard=self.runtime_guard, publish=self.publish_positions,
            clock=lambda: self.get_clock().now().nanoseconds / 1e9)
        self.recorder = LoopRecorder(Path(parameter(self, "log_dir", str(self.runtime_path.parent / "loop"))) / "integration.jsonl")
        self.status_pub = self.create_publisher(String, "status", 30)
        self.native_pub = self.create_publisher(String, "native_actions", 10)
        self.values_pub = self.create_publisher(Float64MultiArray, "native_values", 10)
        self.validation_pub = self.create_publisher(String, "validation", 10)
        self.observation_pub = self.create_publisher(String, "observation_state", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "markers", 10)
        self.subscriptions_owned = [self.create_subscription(Image, "camera/image", self.image, qos_profile_sensor_data),
            self.create_subscription(JointState, "joint_states", self.joints, qos_profile_sensor_data),
            self.create_subscription(String, "instruction", self.instruction, 5)]
        self.validation_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla_sim_p2")
        self.validation_future = None
        self.sequence = self.native_count = 0
        self.next_inference = 0.0
        self.scenario_started = False
        self.stopped = False
        self.last_error = None
        self.last_candidate = None
        self.last_latency = None
        self.last_marker_time = 0.0
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.report("ready", execution_backend=mode, source_kind=self.source.kind, hardware_execution_enabled=False)

    def runtime_guard(self):
        if os.environ.get("IGN_PARTITION") != self.runtime["partition"]:
            return False
        if sha256((self.runtime_path.parent / "stage23.sdf").read_bytes()).hexdigest() != self.runtime["world_sha256"]:
            return False
        try:
            owner = json.loads((self.runtime_path.parent / "owner.json").read_text())
            command = Path(f"/proc/{int(owner['simulator_pid'])}/cmdline").read_bytes()
            return str(self.runtime_path.parent / "stage23.sdf").encode() in command
        except (OSError, ValueError, KeyError):
            return False

    def report(self, event, **data):
        payload = {"event": event, "source_kind": "gazebo", "simulation_only": True, "hardware_validated": False,
            "command_count": self.backend.command_count if hasattr(self.backend, "command_count") else 0, **data}
        self.recorder.record(event, **{k:v for k,v in payload.items() if k != "event"})
        self.status_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def publish_positions(self, q):
        if not self.control or self.runtime_guard() is not True:
            raise VLAError("SIMULATION_NOT_ENABLED", "simulation command publishers are disabled or runtime was lost")
        for name, value in zip(JOINT_NAMES, q):
            self.control[name].publish(Float64(data=float(value)))

    def source_error(self, exc):
        self.client.invalidate("SENSOR_INVALIDATED")
        self.backend.cancel("SENSOR_INVALIDATED")
        error = exc if isinstance(exc, VLAError) else VLAError("INVALID_SENSOR", str(exc))
        if self.last_error != error.code:
            self.report("sensor_rejected", error=error.report())
            self.last_error = error.code

    def image(self, message):
        try: self.source.image(message)
        except Exception as exc: self.source_error(exc)

    def joints(self, message):
        try: self.source.joint_state(message)
        except Exception as exc: self.source_error(exc)

    def instruction(self, message):
        try:
            self.source.set_instruction(message.data)
            self.client.invalidate("TASK_CHANGED")
            self.backend.cancel("TASK_CHANGED")
        except Exception as exc: self.source_error(exc)

    def native_markers(self):
        markers = []
        label = Marker()
        label.header.frame_id, label.header.stamp = "base_link", self.get_clock().now().to_msg()
        label.ns, label.id, label.type, label.action = "native_inspection", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.z, label.pose.orientation.w, label.scale.z = 0.52, 1.0, 0.022
        label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 0.8, 0.25, 1.0
        label.text = f"SmolVLA native {self.last_candidate.shape if self.last_candidate else 'waiting'} — INSPECTION ONLY\nNo robot mapping | RTT {self.last_latency or 0:.3f} s\n{self.backend.last_status if hasattr(self.backend,'last_status') else 'READ ONLY'}"
        markers.append(label)
        if self.last_candidate:
            for channel in range(self.last_candidate.shape[1]):
                line = Marker()
                line.header = label.header
                line.ns, line.id, line.type, line.action = "native_channel_plot", channel, Marker.LINE_STRIP, Marker.ADD
                line.pose.orientation.w, line.scale.x = 1.0, 0.003
                line.color.r, line.color.g, line.color.b, line.color.a = (channel%3)/2, 1-(channel%3)/3, 0.8, 1.0
                line.points = [Point(x=-0.12 + index*0.005, y=-0.18,
                    z=0.30 + channel*0.025 + math.tanh(row[channel])*0.008) for index,row in enumerate(self.last_candidate.values)]
                markers.append(line)
        self.markers_pub.publish(MarkerArray(markers=markers))

    def tick(self):
        if self.stopped: return
        try:
            current = self.source.snapshot(self.sequence, self.get_clock().now().nanoseconds)
            self.last_error = None
            result = self.client.poll()
            if result is not None:
                candidate = result.candidate
                self.last_candidate, self.last_latency = candidate, result.round_trip_s
                self.native_count += 1
                self.native_pub.publish(String(data=json.dumps(candidate.model_dump(), allow_nan=False)))
                values = Float64MultiArray()
                values.layout.dim = [MultiArrayDimension(label="horizon", size=candidate.shape[0], stride=candidate.shape[0]*candidate.shape[1]),
                    MultiArrayDimension(label="native_channels_unknown", size=candidate.shape[1], stride=candidate.shape[1])]
                values.data = [value for row in candidate.values for value in row]
                self.values_pub.publish(values)
                self.report("native_received", native_action=candidate.model_dump(), round_trip_s=result.round_trip_s,
                    observation_positions=result.observation.joints.positions, observation_seq=result.observation.observation_seq,
                    state_revision=result.observation.state_revision)
                try:
                    self.backend.interpreter.interpret(candidate)
                except VLAError as exc:
                    self.validation_pub.publish(String(data=json.dumps({"status": "NOT_INTERPRETABLE", "p2_called": False, "error": exc.report()})))
                    self.report("native_rejected", error=exc.report())
                else:
                    raise VLAError("UNEXPECTED_NATIVE_MAPPING", "base checkpoint output unexpectedly acquired robot semantics")
            if self.validation_future is not None:
                if not self.validation_future.done(): return
                future, self.validation_future = self.validation_future, None
                validated = future.result()
                self.validation_pub.publish(String(data=json.dumps(validated.model_dump(), allow_nan=False)))
                self.report("scenario_validation", validated_trajectory=validated.model_dump(), scenario_source="rule_generated_simulation_only")
            if self.backend.active:
                outcome = self.backend.tick(current)
                if outcome: self.report("scenario_complete", **outcome)
            elif self.control and self.native_count >= 2 and not self.scenario_started and not self.client.busy:
                # An independent known-semantic fixture. No native values enter this path.
                candidate = known_simulation_candidate(current, self.runtime["scenario_target"])
                self.scenario_started = True
                self.validation_future = self.validation_worker.submit(self.backend.submit, candidate, current, self.backend.generation)
            if monotonic() - self.last_marker_time >= 0.2:
                self.native_markers(); self.last_marker_time = monotonic()
            if not self.backend.active and self.validation_future is None and not self.client.busy and monotonic() >= self.next_inference:
                self.recorder.observation(current)
                self.observation_pub.publish(String(data=json.dumps({"sequence": current.observation_seq, "positions": current.joints.positions,
                    "state_revision": current.state_revision, "stamp": current.joints.stamp.model_dump(), "image_frame": current.images[0].frame_id,
                    "image_sha256": current.images[0].sha256, "source_kind": self.source.kind})))
                self.client.submit(current)
                self.sequence += 1
                self.next_inference = monotonic() + 1.0
        except Exception as exc:
            error = exc if isinstance(exc, VLAError) else VLAError("INTEGRATION_ERROR", str(exc))
            self.backend.cancel(error.code)
            self.client.invalidate(error.code)
            if self.last_error != error.code:
                self.report("request_rejected", error=error.report()); self.last_error = error.code
            self.next_inference = monotonic() + 1.0

    def destroy_node(self):
        if not self.stopped:
            self.stopped = True
            self.timer.cancel()
            self.client.close()
            self.validation_worker.shutdown(wait=True, cancel_futures=True)
            self.backend.close()
            self.source.close()
            self.recorder.record("shutdown", command_count=self.backend.command_count if hasattr(self.backend,"command_count") else 0)
            self.recorder.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimulationIntegrationNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__": main()
