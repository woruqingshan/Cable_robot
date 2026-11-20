"""Physical SHADOW collector. This module has no actuator interface."""
import json
from urllib.parse import urlparse
from time import monotonic

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, CameraInfo
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from vla_protocol.errors import VLAError
from vla_protocol.profiles import RuntimeLimits
from .policy_client import PolicyClient
from .real_observation import RealObservationCore, load_evidence
from .real_recording import RealRecorder
from .real_camera import CameraRecordAdapter
from .robot_state_provider import RobotStateProvider

NAMESPACE = "/idc_vla/real"


class RealObservationNode(Node):
    """Only sensor subscriptions and one private diagnostics publisher are owned."""
    def __init__(self, **kwargs):
        super().__init__("real_observation", namespace=NAMESPACE, **kwargs)
        self.recorder = self.client = self.core = None
        try:
            self._configure()
        except Exception:
            self.close()
            self.destroy_node()
            raise

    def _configure(self):
        def param(name, default):
            return self.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
        if self.get_namespace() != NAMESPACE or self.get_parameter("use_sim_time").value:
            raise VLAError("INVALID_REAL_RUNTIME", "physical namespace and system clock required")
        self.add_on_set_parameters_callback(self._validate_clock_parameter)
        camera = param("camera_topic", "/depth_cam/rgb/image_raw")
        joint = param("joint_state_topic", "/controller_manager/joint_states")
        gripper = param("gripper_state_topic", "")
        if not camera.startswith("/") or not joint.startswith("/") or gripper and not gripper.startswith("/"):
            raise VLAError("INVALID_SENSOR_TOPIC", "explicit absolute camera/state topics required")
        evidence_path = param("state_evidence_path", "")
        evidence_sha = param("state_evidence_sha256", "")
        evidence = load_evidence(evidence_path, evidence_sha)
        context = {"robot_id": param("robot_id", "jetrover_real"),
            "scene_id": param("scene_id", "unregistered_real_scene"),
            "scene_sha256": param("scene_sha256", ""),
            "model_id": param("model_id", "unregistered_model"),
            "instruction": param("instruction", "Observe the RJ45 workspace; do not execute actions."),
            "camera_id": param("camera_id", "real_rgb")}
        if len(context["scene_sha256"]) != 64:
            raise VLAError("SCENE_IDENTITY_REQUIRED", "SHA256 of explicit real scene manifest required; unknown geometry is permitted")
        hz = param("observation_hz", 2.0)
        if not 0 < hz <= 30:
            raise VLAError("INVALID_RATE", "observation_hz must be in (0,30]")
        limits = RuntimeLimits(max_horizon=256, request_timeout_s=param("request_timeout_s", 30.0))
        self.core = RealObservationCore(state_topic=joint, gripper_topic=gripper, evidence=evidence,
                                        limits=limits, **context)
        self.recorder = RealRecorder(param("output_directory", "real_observations"), {
            **context, "camera_topic": camera, "joint_state_topic": joint, "gripper_state_topic": gripper,
            "state_evidence_sha256": evidence_sha or None, "state_evidence": evidence,
            "limits": limits.model_dump(), "motion_status": "unknown"})
        self.camera = CameraRecordAdapter(self.recorder, sample_hz=param("sensor_record_hz", 2.0))
        self.state_provider = RobotStateProvider()
        info_topic = param("rgb_camera_info_topic", "/depth_cam/rgb/camera_info")
        depth_topic = param("depth_topic", "")
        depth_info_topic = param("depth_camera_info_topic", "")
        cache_topic = param("controller_cache_topic", "")
        enable = param("enable_inference", False)
        endpoint = param("server_url", "http://127.0.0.1:8000")
        if enable:
            if urlparse(endpoint).hostname not in ("127.0.0.1", "::1", "localhost"):
                raise VLAError("PRIVATE_INFERENCE_REQUIRED", "use a loopback SSH tunnel endpoint")
            # Same one-flight client as simulation. No execution backend or action adapter.
            self.client = PolicyClient(endpoint=endpoint, limits=limits)
        self.sequence = 0
        self.counts = {"valid": 0, "rejected": 0, "submitted": 0, "responses": 0, "network_errors": 0}
        self.last_image_stamp = None
        self.last_rejection = None
        self.last_event_time = 0.0
        diagnostic_topic = self.resolve_topic_name("diagnostics")
        if diagnostic_topic != NAMESPACE + "/diagnostics":
            raise VLAError("INVALID_REAL_RUNTIME", "diagnostics must remain inside physical private namespace")
        self.status_publisher = self.create_publisher(String, "diagnostics", 10)
        self.subscriptions_owned = [
            self.create_subscription(Image, camera, lambda m: self._sensor("image", m), qos_profile_sensor_data),
            self.create_subscription(JointState, joint, lambda m: self._sensor("joints", m), qos_profile_sensor_data)]
        if gripper:
            self.subscriptions_owned.append(self.create_subscription(JointState, gripper,
                lambda m: self._sensor("gripper", m), qos_profile_sensor_data))
        if info_topic:
            self.subscriptions_owned.append(self.create_subscription(CameraInfo, info_topic,
                lambda m: self._camera_info("rgb", m), qos_profile_sensor_data))
        if depth_topic:
            self.subscriptions_owned.append(self.create_subscription(Image, depth_topic,
                self._depth, qos_profile_sensor_data))
        if depth_info_topic:
            self.subscriptions_owned.append(self.create_subscription(CameraInfo, depth_info_topic,
                lambda m: self._camera_info("depth", m), qos_profile_sensor_data))
        if cache_topic:
            from rosidl_runtime_py.utilities import get_message
            self.subscriptions_owned.append(self.create_subscription(get_message("servo_controller_msgs/msg/ServoStateList"),
                cache_topic, self._controller_cache, qos_profile_sensor_data))
        self.timer = self.create_timer(1/hz, self._tick)
        self.recorder.event("startup", inference_enabled=enable, motion_status="unknown", execution_enabled=False)
        self.get_logger().info("SHADOW observation run: " + str(self.recorder.path))

    def _validate_clock_parameter(self, parameters):
        if any(p.name == "use_sim_time" and p.value is not False for p in parameters):
            return SetParametersResult(successful=False, reason="physical observer requires system time")
        return SetParametersResult(successful=True)

    def _sensor(self, kind, message):
        try:
            if kind == "image":
                converted = self.camera.rgb(message)
                if converted is None:
                    return
                self.core.image(converted)
                self.core.image_provenance.update(self.camera.provenance)
            else:
                getattr(self.core, kind)(message)
        except Exception as exc:
            if kind == "image":
                self.core.source.collector.parts.pop("image", None)
            self._reject(exc)

    def _camera_info(self, kind, message):
        try:
            self.camera.camera_info(kind, message)
        except Exception as exc:
            self._reject(exc)

    def _depth(self, message):
        try:
            self.camera.depth(message)
        except Exception as exc:
            self._reject(exc)

    def _controller_cache(self, message):
        try:
            self.state_provider.offer_controller_cache(message)
        except Exception as exc:
            self._reject(exc)

    def _reject(self, exc):
        code = getattr(exc, "code", type(exc).__name__)
        self.counts["rejected"] += 1
        if self.client:
            self.client.invalidate(code)
        # Rate-limit a persistent sensor failure while retaining aggregate counts.
        if code != self.last_rejection or monotonic()-self.last_event_time >= 1:
            self.recorder.event("observation_rejected", code=code, message=str(exc)[:512])
            self.last_rejection, self.last_event_time = code, monotonic()

    def _poll(self):
        if not self.client:
            return
        try:
            result = self.client.poll()
            if result is not None:
                self.counts["responses"] += 1
                self.recorder.event("native_action_diagnostic", request_id=result.observation.request_id,
                    candidate=result.candidate.model_dump(mode="json"), round_trip_s=result.round_trip_s,
                    encoding_s=result.encoding_s, execution_enabled=False)
        except Exception as exc:
            self.counts["network_errors"] += 1
            self.recorder.event("inference_error", code=getattr(exc, "code", type(exc).__name__), message=str(exc)[:512])

    def _tick(self):
        self._poll()
        try:
            observation = self.core.snapshot(self.sequence, self.get_clock().now().nanoseconds)
            stamp = observation.images[0].stamp.nanoseconds
            if self.last_image_stamp != stamp:
                self.recorder.record(observation, {"image": self.core.image_provenance,
                    "state_evidence": self.core.evidence, "motion_status": "unknown"})
                self.sequence += 1
                self.counts["valid"] += 1
                self.last_image_stamp = stamp
                if self.client and not self.client.busy:
                    self.client.submit(observation)
                    self.counts["submitted"] += 1
        except Exception as exc:
            self._reject(exc)
        status = {"mode": "shadow_only", "execution_enabled": False, "counts": self.counts,
            "last_rejection": self.last_rejection, "run": str(self.recorder.path), "motion_status": "unknown"}
        sensor_status = {"camera": self.camera.status(), "robot_state": self.state_provider.get_source_status()}
        self.recorder.event("sensor_status", **sensor_status)
        self.status_publisher.publish(String(data=json.dumps(status)))

    def close(self):
        if self.client:
            self.client.close()
            self.client = None
        if self.core:
            self.core.close()
        if self.recorder:
            self.recorder.close()
            self.recorder = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RealObservationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
