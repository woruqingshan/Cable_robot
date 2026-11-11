"""Private JointState/marker playback and live preview; zero command clients."""
import json
from time import monotonic
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from vla_protocol import codec
from vla_protocol.action import ValidatedTrajectory
from vla_protocol.errors import VLAError
from ..ros_support import require_isolation, parameter
from .playback import TrajectoryPlayback, load_record, load_model, joint_values

NAMESPACE = "/idc_vla/preview"


class PreviewNode(Node):
    def __init__(self, **kwargs):
        require_isolation()
        super().__init__("trajectory_preview", namespace=NAMESPACE, **kwargs)
        if self.get_namespace() != NAMESPACE or self.get_parameter("use_sim_time").value:
            raise VLAError("ROS_ISOLATION", "preview requires its private namespace and ROS system clock")
        for name in ("joint_states", "status", "markers", "candidate"):
            if not self.resolve_topic_name(name).startswith(NAMESPACE + "/"):
                raise VLAError("ROS_ISOLATION", "preview topic remap escaped private namespace")
        self.model = load_model(parameter(self, "workspace_root", ""), parameter(self, "model_config", "") or None)
        self.allow_synthetic = parameter(self, "allow_synthetic", False)
        self.loop = parameter(self, "loop", False)
        rate = parameter(self, "replay_hz", 20.0)
        if not 1 <= rate <= 100:
            raise VLAError("INVALID_PLAYBACK_RATE", "replay_hz must be 1..100")
        self.playback = None
        self.start = monotonic()
        self.last_error = None
        self.path_points = []
        self.joints = self.create_publisher(JointState, "joint_states", 10)
        self.status = self.create_publisher(String, "status", 10)
        self.markers = self.create_publisher(MarkerArray, "markers", 10)
        self.candidates = self.create_subscription(String, "candidate", self.receive, 5)
        path = parameter(self, "input_path", "")
        selected = parameter(self, "source_action_id", "")
        if path:
            self.select(load_record(path, action_id=selected))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def select(self, record):
        playback = TrajectoryPlayback(record, allow_synthetic=self.allow_synthetic)
        playback.bind_model(self.model)
        from workspace_analysis.robot.fk import ForwardKinematics
        fk = ForwardKinematics(self.model)
        self.path_points = [tuple(float(v) for v in fk(q)[:3, 3]) for q in playback.positions]
        self.playback, self.start, self.last_error = playback, monotonic(), None

    def receive(self, message):
        try:
            self.select(codec.loads(message.data, ValidatedTrajectory))
        except Exception as exc:
            error = exc if isinstance(exc, VLAError) else VLAError("PLAYBACK_INVALID", str(exc))
            self.last_error = error.report()

    def tick(self):
        now = self.get_clock().now().to_msg()
        report = {"schema_version": 1, "usage": "READ_ONLY_VISUALIZATION", "execution_authorized": False,
            "hardware_validated": False, "loaded_model_sha256": self.model.metadata()["expanded_urdf_sha256"],
            "error": self.last_error, "status": "WAITING_FOR_TRAJECTORY"}
        if self.playback is not None:
            elapsed = monotonic() - self.start
            if self.loop:
                elapsed %= self.playback.duration
            q = self.playback.at(elapsed)
            names, values = joint_values(self.model, q, self.playback.record.starting_gripper.position)
            state = JointState()
            state.header.stamp, state.header.frame_id = now, self.model.base_frame
            state.name, state.position = list(names), list(values)
            self.joints.publish(state)
            report.update(status="COMPLETE" if elapsed >= self.playback.duration else "PLAYING",
                label=self.playback.label, elapsed_s=elapsed, duration_s=self.playback.duration,
                source_action_id=self.playback.action.action_id, validation_status=self.playback.record.status,
                source_model_id=self.playback.action.model_id, scene_id=self.playback.action.scene_id,
                scene_sha256=self.playback.action.scene_sha256, provenance=self.playback.record.provenance,
                q=list(q), independent_joint_names=list(names))
            label = Marker()
            label.header = state.header
            label.ns, label.id, label.type, label.action = "read_only", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.z, label.pose.orientation.w = 0.65, 1.0
            label.scale.z = 0.022
            label.color.r, label.color.g, label.color.a = 1.0, 0.75, 1.0
            # Avoid Ogre's oversized default space advance in small world text.
            label.text = self.playback.label.replace(" / ", ":")
            authorization = Marker()
            authorization.header = state.header
            authorization.ns, authorization.id, authorization.type, authorization.action = "read_only", 2, Marker.TEXT_VIEW_FACING, Marker.ADD
            authorization.pose.position.z, authorization.pose.orientation.w = 0.60, 1.0
            authorization.scale.z = 0.018
            authorization.color.r, authorization.color.g, authorization.color.a = 1.0, 0.75, 1.0
            authorization.text = "READ-ONLY:NO-EXECUTION"
            line = Marker()
            line.header = state.header
            line.ns, line.id, line.type, line.action = "ee_preview", 1, Marker.LINE_STRIP, Marker.ADD
            line.pose.orientation.w, line.scale.x = 1.0, 0.003
            line.color.b, line.color.g, line.color.a = 1.0, 0.75, 1.0
            line.points = [Point(x=x, y=y, z=z) for x, y, z in self.path_points]
            self.markers.publish(MarkerArray(markers=[label, authorization, line]))
        self.status.publish(String(data=json.dumps(report, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PreviewNode()
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
