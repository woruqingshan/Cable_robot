"""ROS RGB/JointState mock with reset and simulation-only trajectory updates."""
import json
from pathlib import Path
from time import monotonic
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vla_protocol import codec
from vla_protocol.action import ValidatedTrajectory
from vla_protocol.errors import VLAError
from vla_protocol.observation import JointPositions, SourceStamp
from vla_protocol.profiles import JOINT_NAMES
from .observation_node import StateMetadata
from .ros_support import NAMESPACE, require_isolation, configure_topics, parameter
from .simulation import SimulationState


class MockRobotNode(Node):
    def __init__(self, **kwargs):
        require_isolation()
        super().__init__("mock_robot", namespace=NAMESPACE, **kwargs)
        topics = configure_topics(self)
        initial_q = parameter(self, "initial_positions", [0., 0., 0., 0., 0.])
        # Reuse protocol validation before exposing any simulated state.
        JointPositions(names=list(JOINT_NAMES), positions=initial_q, unit="rad",
                       stamp=SourceStamp(nanoseconds=0, clock_domain="ros_system"))
        self.state = SimulationState(initial_q,
            gripper=parameter(self, "gripper_rad", 0.3),
            instruction=parameter(self, "instruction", "Hold the simulated arm position"),
            robot_id=parameter(self, "robot_id", "jetrover_mock"),
            scene_id=parameter(self, "scene_id", "stage1_synthetic"),
            scene_sha256=parameter(self, "scene_sha256", "0" * 64),
            model_id=parameter(self, "model_id", "synthetic_joint_state_v1"),
            allow_unvalidated=parameter(self, "allow_unvalidated_simulation", False),
            validation_mode=parameter(self, "validation_mode", "disabled"))
        if self.state.validation_mode not in ("disabled", "required"):
            raise VLAError("INVALID_CONFIG", "validation_mode must be disabled or required")
        self.max_action_age = parameter(self, "max_action_age_s", 5.0)
        self.frame_id = parameter(self, "camera_frame", "mock_camera_optical_frame")
        hz = parameter(self, "publish_hz", 10.0)
        if not 1 <= hz <= 100 or not 0 < self.max_action_age <= 30:
            raise VLAError("INVALID_CONFIG", "invalid publish rate/action age")
        asset = parameter(self, "image_path", "")
        if asset:
            from PIL import Image as PILImage
            with PILImage.open(Path(asset).expanduser()) as image:
                if image.width * image.height > 1_048_576:
                    raise VLAError("INVALID_IMAGE", "image asset exceeds pixel limit")
                image = image.convert("RGB")
                self.width, self.height, self.rgb = image.width, image.height, image.tobytes()
        else:
            self.width, self.height = 64, 48
            self.rgb = bytes(v for y in range(self.height) for x in range(self.width) for v in (x * 4, y * 5, 96))
        self.rgb_pub = self.create_publisher(Image, topics["rgb_topic"], 5)
        self.joint_pub = self.create_publisher(JointState, topics["joints_topic"], 5)
        self.gripper_pub = self.create_publisher(JointState, topics["gripper_topic"], 5)
        self.task_pub = self.create_publisher(String, topics["task_topic"], 5)
        self.events_pub = self.create_publisher(String, topics["execution_status_topic"], 10)
        self.command_sub = self.create_subscription(String, topics["execute_topic"], self.command, 5)
        self.instruction_sub = self.create_subscription(String, topics["instruction_topic"], self.instruction, 5)
        reset_name = self.resolve_service_name("reset")
        if not reset_name.startswith(NAMESPACE + "/"):
            raise VLAError("ROS_ISOLATION", "reset service must stay private")
        self.reset_service = self.create_service(Trigger, reset_name, self.reset)
        self.timer = self.create_timer(1.0 / hz, self.publish_state)
        self.get_logger().info("Mock robot ready; no hardware execution capability")

    def event(self, **data):
        self.events_pub.publish(String(data=json.dumps({"schema_version": 1, "session_id": self.state.session_id,
            "simulation_only": True, "hardware_validated": False, **data}, allow_nan=False)))

    def reset(self, request, response):
        self.state.reset()
        response.success, response.message = True, "new mock session: " + self.state.session_id
        self.event(status="RESET")
        self.publish_state()
        return response

    def instruction(self, message):
        try:
            self.state.set_instruction(message.data)
            self.event(status="TASK_CHANGED", task_revision=self.state.task_revision)
            self.publish_state()
        except Exception as exc:
            self.event(status="REJECTED", error=error_report(exc))

    def command(self, message):
        try:
            validated = codec.loads(message.data, ValidatedTrajectory)
            self.state.accept(validated, monotonic(), self.get_clock().now().nanoseconds, self.max_action_age)
            self.event(status="SIMULATION_STARTED", source_action_id=validated.source_action_id)
            self.publish_state()
        except Exception as exc:
            self.event(status="REJECTED", error=error_report(exc))

    def publish_state(self):
        completed = self.state.tick(monotonic())
        if completed:
            self.event(**completed)
        now = self.get_clock().now()
        timestamp = now.to_msg()
        joint = JointState()
        joint.header.stamp = timestamp
        joint.header.frame_id = "base_link"
        joint.name, joint.position = list(JOINT_NAMES), list(self.state.q)
        gripper = JointState()
        gripper.header = joint.header
        gripper.name, gripper.position = ["r_joint"], [self.state.gripper]
        image = Image()
        image.header.stamp, image.header.frame_id = timestamp, self.frame_id
        image.width, image.height, image.encoding = self.width, self.height, "rgb8"
        image.is_bigendian, image.step, image.data = 0, self.width * 3, self.rgb
        meta = StateMetadata(schema_version=1, session_id=self.state.session_id,
            task_revision=self.state.task_revision, state_revision=self.state.state_revision,
            robot_id=self.state.robot_id, scene_id=self.state.scene_id, scene_sha256=self.state.scene_sha256,
            model_id=self.state.model_id, instruction=self.state.instruction,
            stamp=SourceStamp(nanoseconds=now.nanoseconds, clock_domain="ros_system"), busy=self.state.active is not None)
        self.joint_pub.publish(joint)
        self.gripper_pub.publish(gripper)
        self.rgb_pub.publish(image)
        self.task_pub.publish(String(data=meta.model_dump_json()))


def error_report(exc):
    return exc.report() if isinstance(exc, VLAError) else VLAError("INVALID_INPUT", str(exc)).report()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockRobotNode()
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
