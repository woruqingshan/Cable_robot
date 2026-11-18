"""Generate a Fortress world from the exact external JetRover/P2 model and scene."""
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from ..adapters.p2 import P2Adapter, P2Config
from .execution import known_simulation_candidate

PREFIX = "/idc_vla/sim"


def element(parent, tag, text=None, **attrs):
    result = ET.SubElement(parent, tag, attrs)
    if text is not None: result.text = str(text)
    return result


def plugin(parent, filename, name):
    return element(parent, "plugin", filename=filename, name=name)


def fixture_observation(q, validator):
    """Explicit synthetic preparation fixture; never cloud inference evidence."""
    import base64
    from time import time_ns
    from uuid import uuid4
    from vla_protocol.observation import Observation
    from vla_protocol.png import encode_rgb
    from vla_protocol.profiles import JOINT_NAMES
    stamp = {"nanoseconds": time_ns(), "clock_domain": "ros_system"}
    png = encode_rgb(2, 2, bytes([20, 40, 60] * 4))
    return Observation(schema_version=1, request_id=str(uuid4()), session_id=str(uuid4()), observation_seq=0,
        task_revision=0, state_revision=0, robot_id="synthetic_simulation_fixture", robot_profile_id="jetrover_joint_abs_rad_v1",
        scene_id=validator.scene.scene_id, scene_sha256=validator.scene.canonical_scene_sha256,
        model_id="urdf:" + validator.model.metadata()["expanded_urdf_sha256"], instruction="Validate a synthetic scenario",
        joints={"names": list(JOINT_NAMES), "positions": list(q), "unit": "rad", "stamp": stamp},
        gripper={"name": "r_joint", "position": 0.3, "unit": "rad", "stamp": stamp, "simulated": True, "calibrated": False},
        images=[{"camera_id": "mock_rgb", "frame_id": "synthetic_fixture", "stamp": stamp, "width": 2, "height": 2,
            "codec": "png", "pixel_encoding": "rgb8", "sha256": sha256(png).hexdigest(), "data_base64": base64.b64encode(png).decode()}],
        simulated=True, calibration_status="synthetic_uncalibrated", capabilities=["native_actions_v1", "session_state_v1"])


def prepare_runtime(*, workspace_root, backend, output, domain=84):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    root = Path(workspace_root).resolve()
    config = P2Config(mode="required", workspace_root=str(root), model_config_path=str(root / "config/workspace.yaml"),
        scene_path=str(root / "config/scenes/idc_prototype_v0.yaml"),
        srdf_path=str(root / "config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf"),
        backend_path=str(Path(backend).resolve()), allow_candidate_semantics=True, workspace_enabled=True,
        task_path=str(root / "config/tasks/p22_research_v0.yaml"), max_validation_time_s=5.0)
    validator = P2Adapter(config)
    try:
        validator._load(0.3)
        from workspace_analysis.workspace.sampling import sample_joint_positions
        initial = rejected_q = None
        for q in sample_joint_positions(validator.model.limits, 100, 42):
            check = validator.checker.check(q, fresh=True)
            if check["valid"] is True and initial is None: initial = q.tolist()
            if check.get("semantic_self_collision") and rejected_q is None: rejected_q = q.tolist()
            if initial is not None and rejected_q is not None: break
        if initial is None or rejected_q is None:
            raise ValueError("could not establish genuine native P2 fixtures")
        obs = fixture_observation(initial, validator)
        target = list(initial)
        target[0] += 0.08
        from .interfaces import RegisteredActionInterpreter
        interpreter = RegisteredActionInterpreter()
        valid = validator.validate(interpreter.interpret(known_simulation_candidate(obs, target)), obs)
        if valid.status != "PASS_RESEARCH_ONLY":
            raise ValueError("scenario path is not accepted by required P2: " + str(valid.failure_reasons))
        bad_obs = fixture_observation(rejected_q, validator)
        collision = validator.validate(interpreter.interpret(known_simulation_candidate(bad_obs, rejected_q)), bad_obs)
        outside = list(initial); outside[0] = float(validator.model.limits[0, 1] + 1.0)
        bad_limit = validator.validate(interpreter.interpret(known_simulation_candidate(obs, outside)), obs)
        if collision.trajectory is not None or bad_limit.trajectory is not None:
            raise ValueError("P2 failed to reject a collision or invalid joint limit")
        for name, result in [("valid", valid), ("collision", collision), ("joint_limit", bad_limit)]:
            (output / ("p2_" + name + ".json")).write_text(json.dumps(result.model_dump(), indent=2) + "\n")
        (output / "source_robot.urdf").write_text(validator.model.urdf_xml)
        sdf_text = subprocess.check_output(["ign", "sdf", "-p", str(output / "source_robot.urdf")], text=True)
        model = ET.fromstring(sdf_text).find("model")
        model.set("name", "jetrover_stage23")
        # The source base_footprint sits 0.116... m below P2's base_link.
        base_offset = validator.model.joints["base_joint"].xyz[2]
        element(model, "pose", f"0 0 {-base_offset} 0 0 0")
        anchor = element(model, "joint", name="stage23_fixed_base", type="fixed")
        element(anchor, "parent", "world"); element(anchor, "child", "base_footprint")
        positions = validator.model.joint_positions(initial, {"r_joint": 0.3})
        init = plugin(model, "libstage23_initialize.so", "Stage23Initialize")
        for joint in model.findall("joint"):
            name = joint.get("name")
            if joint.get("type") != "revolute": continue
            entry = element(init, "joint")
            element(entry, "name", name); element(entry, "position", positions[name])
            control = plugin(model, "ignition-gazebo-joint-position-controller-system", "ignition::gazebo::systems::JointPositionController")
            element(control, "joint_name", name); element(control, "topic", PREFIX + "/control/" + name + "/position")
            element(control, "initial_position", positions[name])
            element(control, "use_velocity_commands", "true"); element(control, "cmd_max", 0.5)
        states = plugin(model, "ignition-gazebo-joint-state-publisher-system", "ignition::gazebo::systems::JointStatePublisher")
        element(states, "topic", PREFIX + "/joint_states")
        # Resolve URI locations without copying or rewriting upstream geometry.
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory("jetrover_description")).resolve()
        for uri in model.iter("uri"):
            if uri.text.startswith("model://jetrover_description/"):
                uri.text = "file://" + str(share / uri.text.removeprefix("model://jetrover_description/"))
        sdf = ET.Element("sdf", version="1.9")
        world = element(sdf, "world", name="stage23")
        element(world, "gravity", "0 0 0")
        physics = element(world, "physics", name="research_kinematics", type="ignored")
        element(physics, "max_step_size", 0.001); element(physics, "real_time_factor", 1.0)
        for filename, classname in [("physics", "Physics"), ("sensors", "Sensors"), ("scene-broadcaster", "SceneBroadcaster"), ("user-commands", "UserCommands")]:
            plug = plugin(world, "ignition-gazebo-" + filename + "-system", "ignition::gazebo::systems::" + classname)
            if classname == "Sensors": element(plug, "render_engine", "ogre2")
        scene = element(world, "scene")
        element(scene, "ambient", "0.6 0.6 0.6 1"); element(scene, "background", "0.12 0.15 0.2 1")
        light = element(world, "light", type="directional", name="sun")
        element(light, "pose", "0 0 3 0 0 0"); element(light, "diffuse", "0.8 0.8 0.8 1"); element(light, "direction", "-0.5 0.1 -0.9")
        world.append(model)
        from scipy.spatial.transform import Rotation
        for obj in validator.scene.collision_objects:
            transform = validator.scene.object_transform(obj["id"])
            pose = [*transform[:3, 3], *Rotation.from_matrix(transform[:3, :3]).as_euler("xyz")]
            body = element(world, "model", name=obj["id"].replace("/", "_"))
            element(body, "static", "true"); element(body, "pose", " ".join(map(str, pose)))
            link = element(body, "link", name="object")
            for tag in ("visual", "collision"):
                shape = element(link, tag, name=tag)
                geometry = element(shape, "geometry")
                primitive = element(geometry, obj["type"])
                if obj["type"] == "box": element(primitive, "size", " ".join(map(str, obj["size_m"])))
                else:
                    element(primitive, "radius", obj["radius_m"]); element(primitive, "length", obj["length_m"])
                if tag == "visual":
                    material = element(shape, "material"); element(material, "ambient", "0.2 0.45 0.6 1"); element(material, "diffuse", "0.2 0.45 0.6 1")
        camera = element(world, "model", name="inspection_camera")
        element(camera, "static", "true"); element(camera, "pose", "1 -1 0.9 0 0.55 2.35")
        link = element(camera, "link", name="inspection_camera_link")
        sensor = element(link, "sensor", name="rgb", type="camera")
        element(sensor, "always_on", "true"); element(sensor, "update_rate", 15)
        element(sensor, "topic", PREFIX + "/camera/image")
        cam = element(sensor, "camera"); element(cam, "horizontal_fov", 1.047)
        image = element(cam, "image"); element(image, "width", 256); element(image, "height", 256); element(image, "format", "R8G8B8")
        clip = element(cam, "clip"); element(clip, "near", 0.03); element(clip, "far", 10)
        world_path = output / "stage23.sdf"
        world_path.write_text(ET.tostring(sdf, encoding="unicode"))
        bridges = []
        for topic, ros_type, gz_type in [(PREFIX + "/joint_states", "sensor_msgs/msg/JointState", "ignition.msgs.Model"),
                (PREFIX + "/camera/image", "sensor_msgs/msg/Image", "ignition.msgs.Image"),
                ("/world/stage23/clock", "rosgraph_msgs/msg/Clock", "ignition.msgs.Clock")]:
            bridges.append({"ros_topic_name": PREFIX + "/clock" if "clock" in topic else topic,
                "gz_topic_name": topic, "ros_type_name": ros_type, "gz_type_name": gz_type, "direction": "GZ_TO_ROS"})
        for name in validator.model.joint_order:
            topic = PREFIX + "/control/" + name + "/position"
            bridges.append({"ros_topic_name": topic, "gz_topic_name": topic, "ros_type_name": "std_msgs/msg/Float64",
                "gz_type_name": "ignition.msgs.Double", "direction": "ROS_TO_GZ"})
        import yaml
        (output / "bridge.yaml").write_text(yaml.safe_dump(bridges))
        runtime = {"schema_version": 1, "namespace": PREFIX, "domain_id": domain,
            "partition": "cable_vla_stage23_" + sha256(str(output).encode()).hexdigest()[:12],
            "world_sha256": sha256(world_path.read_bytes()).hexdigest(), "source_model_sha256": validator.model.metadata()["expanded_urdf_sha256"],
            "scene_id": validator.scene.scene_id, "scene_sha256": validator.scene.canonical_scene_sha256,
            "initial_positions": initial, "scenario_target": target, "p2_config": config.model_dump(),
            "p2_identity": validator.evaluator.identity,
            "control_semantics": "bounded ideal velocity position control; fixed base; zero gravity; research simulation only",
            "camera_frame": "inspection_camera/inspection_camera_link/rgb", "scenario_source": "rule_generated_simulation_only"}
        (output / "runtime.json").write_text(json.dumps(runtime, indent=2) + "\n")
        return runtime
    finally:
        validator.close()
