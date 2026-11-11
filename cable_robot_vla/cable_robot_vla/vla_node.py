"""ROS orchestration: DDS snapshots -> HTTP -> local validation -> mock DDS only."""
import base64
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha256
import json
from pathlib import Path
from time import monotonic
from uuid import uuid4
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from vla_protocol import codec
from vla_protocol.errors import VLAError
from vla_protocol.observation import ImagePayload, JointPositions, GripperState
from vla_protocol.png import encode_rgb
from vla_protocol.profiles import RuntimeLimits
from .adapters.p2 import P2Adapter, P2Config
from .mock_executor import MockExecutor
from .observation_node import ObservationAdapter, StateMetadata
from .policy_client import PolicyClient
from .recording import LoopRecorder
from .ros_support import NAMESPACE, require_isolation, configure_topics, parameter, stamp


class VLANode(Node):
    def __init__(self, **kwargs):
        require_isolation()
        super().__init__("vla_pipeline", namespace=NAMESPACE, **kwargs)
        self.topics = configure_topics(self)
        default = RuntimeLimits()
        self.limits = RuntimeLimits(**{name: parameter(self, name, value) for name, value in default.model_dump().items()})
        self.collector = ObservationAdapter(self.limits)
        p2_file = parameter(self, "p2_config_path", "")
        mode = parameter(self, "validation_mode", "disabled")
        p2 = {}
        if p2_file:
            import yaml
            p2 = yaml.safe_load(Path(p2_file).expanduser().read_text())
            if not isinstance(p2, dict):
                raise VLAError("INVALID_CONFIG", "P2 YAML must be a mapping")
        p2["mode"] = mode
        self.mock_executor = MockExecutor(P2Adapter(P2Config.model_validate(p2)),
            advance_simulation=parameter(self, "advance_simulation", False),
            allow_unvalidated=parameter(self, "allow_unvalidated_simulation", False))
        self.client = PolicyClient(parameter(self, "server_url", "http://127.0.0.1:8000"), limits=self.limits)
        self.interval = parameter(self, "inference_interval_s", 0.25)
        if not 0.05 <= self.interval <= 60:
            raise VLAError("INVALID_CONFIG", "inference interval must be 0.05..60s")
        log_dir = Path(parameter(self, "log_dir", "log/vla")).expanduser()
        self.recorder = LoopRecorder(log_dir / ("loop_" + uuid4().hex + ".jsonl"))
        self.outcomes = self.create_publisher(String, self.topics["outcomes_topic"], 10)
        self.commands = self.create_publisher(String, self.topics["execute_topic"], 5)
        self.subscriptions_owned = [
            self.create_subscription(Image, self.topics["rgb_topic"], self.image, 5),
            self.create_subscription(JointState, self.topics["joints_topic"], self.joints, 5),
            self.create_subscription(JointState, self.topics["gripper_topic"], self.gripper, 5),
            self.create_subscription(String, self.topics["task_topic"], self.metadata, 5),
            self.create_subscription(String, self.topics["execution_status_topic"], self.robot_event, 10)]
        self.validation_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla_local_validation")
        self.validation_future = self.inference = None
        self.awaiting_revision = None
        self.sequence, self.next_inference = 0, 0.0
        self.last_source_error = None
        self.stopped = False
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info("VLA mock pipeline ready; JSONL: " + str(log_dir))

    def report(self, event, **data):
        self.recorder.record(event, **data)
        self.outcomes.publish(String(data=json.dumps({"schema_version": 1, "event": event,
            "simulation_only": True, "hardware_validated": False, **data}, allow_nan=False)))

    def source_error(self, exc, kind=None):
        if kind is not None:
            self.collector.parts.pop(kind, None)
        error = exc if isinstance(exc, VLAError) else VLAError("INVALID_OBSERVATION_SOURCE", str(exc))
        self.client.invalidate("OBSERVATION_INVALIDATED")
        if error.code != self.last_source_error:
            self.report("source_rejected", error=error.report())
            self.last_source_error = error.code

    def image(self, message):
        try:
            if message.encoding != "rgb8" or message.is_bigendian != 0 or not 0 < message.width * message.height <= 1_048_576:
                raise VLAError("INVALID_IMAGE", "expected bounded little-endian rgb8 image")
            if message.step < message.width * 3 or len(message.data) != message.step * message.height:
                raise VLAError("INVALID_IMAGE", "invalid ROS image buffer/stride")
            raw = bytes(message.data)
            rgb = b"".join(raw[i * message.step:i * message.step + message.width * 3] for i in range(message.height))
            png = encode_rgb(message.width, message.height, rgb)
            self.collector.offer("image", ImagePayload(camera_id="mock_rgb", frame_id=message.header.frame_id,
                stamp=stamp(message), width=message.width, height=message.height, codec="png", pixel_encoding="rgb8",
                sha256=sha256(png).hexdigest(), data_base64=base64.b64encode(png).decode("ascii")))
        except Exception as exc:
            self.source_error(exc, "image")

    def joints(self, message):
        try:
            self.collector.offer("joints", JointPositions(names=list(message.name), positions=list(message.position), unit="rad", stamp=stamp(message)))
        except Exception as exc:
            self.source_error(exc, "joints")

    def gripper(self, message):
        try:
            if list(message.name) != ["r_joint"] or len(message.position) != 1:
                raise VLAError("INVALID_GRIPPER", "expected separate simulated r_joint")
            self.collector.offer("gripper", GripperState(name="r_joint", position=message.position[0], unit="rad",
                stamp=stamp(message), simulated=True, calibrated=False))
        except Exception as exc:
            self.source_error(exc, "gripper")

    def metadata(self, message):
        try:
            meta = codec.loads(message.data, StateMetadata)
            if self.client.pending is not None:
                pending = self.client.pending
                if (pending.session_id, pending.task_revision, pending.state_revision, pending.scene_sha256, pending.model_id) != meta.token:
                    self.client.invalidate("REQUEST_INVALIDATED")
            self.collector.offer("metadata", meta)
            if self.awaiting_revision is not None:
                session, revision, sent_at = self.awaiting_revision
                if meta.session_id != session or meta.state_revision != revision:
                    self.awaiting_revision = None
        except Exception as exc:
            self.source_error(exc, "metadata")

    def robot_event(self, message):
        try:
            # Bound and parse the local status stream without preserving image payloads.
            if len(message.data.encode()) > 16_384:
                raise VLAError("PAYLOAD_TOO_LARGE", "robot event too large")
            event = json.loads(message.data)
            self.recorder.record("robot_event", robot_event=event)
            if event.get("status") == "REJECTED":
                self.awaiting_revision = None
        except Exception as exc:
            self.source_error(exc)

    def tick(self):
        if self.stopped:
            return
        try:
            result = self.client.poll()
            if result is not None:
                self.inference = result
                self.report("candidate_received", request_id=result.observation.request_id,
                    encoding_s=result.encoding_s, network_round_trip_s=result.round_trip_s,
                    policy_action=result.candidate.model_dump())
                self.validation_future = self.validation_worker.submit(self.mock_executor.prepare, result)
            if self.validation_future is not None and self.validation_future.done():
                future, inference = self.validation_future, self.inference
                self.validation_future = self.inference = None
                try:
                    validated = future.result()
                    current = self.collector.snapshot(self.sequence, self.get_clock().now().nanoseconds)
                    outcome = self.mock_executor.consume(validated, inference, current, monotonic(), self.limits.max_action_age_s)
                except Exception as exc:
                    error = exc if isinstance(exc, VLAError) else VLAError("PIPELINE_ERROR", str(exc))
                    self.report("candidate_rejected", request_id=inference.observation.request_id,
                        policy_action_id=inference.candidate.action_id, error=error.report())
                    self.next_inference = monotonic() + self.interval
                    return
                self.report("execution", request_id=inference.observation.request_id, status=outcome,
                    validated_trajectory=validated.model_dump())
                if outcome.startswith("SIMULATE_"):
                    self.awaiting_revision = (current.session_id, current.state_revision, monotonic())
                    self.commands.publish(String(data=codec.dumps(validated).decode()))
                self.next_inference = monotonic() + self.interval
            if self.awaiting_revision is not None:
                if monotonic() - self.awaiting_revision[2] > self.limits.max_action_age_s:
                    self.report("execution_ack_timeout", error=VLAError("EXECUTION_ACK_TIMEOUT", "mock robot did not acknowledge state change").report())
                    self.awaiting_revision = None
                return
            if self.client.busy or self.validation_future is not None or monotonic() < self.next_inference:
                return
            observation = self.collector.snapshot(self.sequence, self.get_clock().now().nanoseconds)
            self.client.submit(observation)
            self.recorder.observation(observation)
            self.sequence += 1
            self.last_source_error = None
        except VLAError as exc:
            if exc.code not in ("OBSERVATION_INCOMPLETE", "ROBOT_BUSY", "OBSERVATION_SKEW") or self.client.busy:
                if self.last_source_error != exc.code:
                    self.report("rejected", error=exc.report())
                    self.last_source_error = exc.code
            self.next_inference = monotonic() + self.interval
        except Exception as exc:
            self.report("rejected", error=VLAError("PIPELINE_ERROR", str(exc)).report())
            self.next_inference = monotonic() + self.interval

    def destroy_node(self):
        if not self.stopped:
            self.stopped = True
            self.timer.cancel()
            self.client.close()
            self.validation_worker.shutdown(wait=True, cancel_futures=True)
            self.mock_executor.close()
            self.recorder.record("shutdown", status="CLEAN")
            self.recorder.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VLANode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
