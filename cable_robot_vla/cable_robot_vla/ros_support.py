"""Private ROS graph guard; topic remaps are checked before endpoint creation."""
import os
import re
from vla_protocol.errors import VLAError
from vla_protocol.observation import SourceStamp

NAMESPACE = "/idc_vla/mock"
TOPICS = {"rgb_topic": "rgb", "joints_topic": "joint_states", "gripper_topic": "gripper_state",
          "task_topic": "task", "execute_topic": "execute", "outcomes_topic": "outcomes",
          "execution_status_topic": "execution_status", "instruction_topic": "task_instruction"}


def require_isolation():
    try:
        domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    except ValueError:
        domain = 0
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not 1 <= domain <= 101:
        raise VLAError("ROS_ISOLATION", "require ROS_LOCALHOST_ONLY=1 and private ROS_DOMAIN_ID in 1..101")


def configure_topics(node):
    require_isolation()
    if node.get_namespace() != NAMESPACE:
        raise VLAError("ROS_ISOLATION", "mock namespace must remain " + NAMESPACE)
    if node.get_parameter("use_sim_time").value:
        raise VLAError("CLOCK_DOMAIN_MISMATCH", "Stage 1 uses the local ROS system clock")
    result = {}
    for parameter, default in TOPICS.items():
        node.declare_parameter(parameter, default)
        name = node.get_parameter(parameter).value
        if not isinstance(name, str) or not re.fullmatch(r"[a-zA-Z][a-zA-Z0-9_/]*", name):
            raise VLAError("ROS_ISOLATION", "topic parameters must be relative names")
        resolved = node.resolve_topic_name(name)
        if not resolved.startswith(NAMESPACE + "/"):
            raise VLAError("ROS_ISOLATION", "refusing topic remap outside mock namespace")
        result[parameter] = resolved
    if len(set(result.values())) != len(result):
        raise VLAError("ROS_ISOLATION", "mock topics must be distinct")
    return result


def stamp(message):
    return SourceStamp(nanoseconds=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
                       clock_domain="ros_system")


def parameter(node, name, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value
