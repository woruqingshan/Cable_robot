"""Load the official description without copying or editing its resources.

Only fixed and bounded revolute joints are supported in G1-A. Unsupported
models fail explicitly rather than being silently converted to fixed joints.
"""

from dataclasses import asdict, dataclass, field
from hashlib import sha256
import math
import os
from pathlib import Path
from threading import RLock
from typing import Mapping, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from urdf_parser_py.urdf import URDF


JOINT_ORDER = ("joint1", "joint2", "joint3", "joint4", "joint5")
_XACRO_LOCK = RLock()


class ModelError(ValueError):
    """An invalid model, configuration, or joint input."""


def _number(value, label):
    if isinstance(value, (bool, str, bytes)):
        raise ModelError(f"{label}: expected a finite number, got {value!r}")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ModelError(f"{label}: expected a finite number") from exc
    if not math.isfinite(result):
        raise ModelError(f"{label}: value must be finite")
    return result


def _vector(values, label):
    if values is None or len(values) != 3:
        raise ModelError(f"{label}: expected three components")
    return tuple(_number(v, label) for v in values)


def _mapping(value, label):
    if not isinstance(value, Mapping):
        raise ModelError(f"{label}: expected a mapping")
    return dict(value)


@dataclass(frozen=True)
class JointSpec:
    name: str
    kind: str
    parent: str
    child: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    axis: Optional[Tuple[float, float, float]] = None
    lower: Optional[float] = None
    upper: Optional[float] = None
    mimic_joint: Optional[str] = None
    multiplier: float = 1.0
    offset: float = 0.0


@dataclass(frozen=True)
class KinematicModel:
    urdf_xml: str
    base_frame: str
    ee_frame: str
    chain: Tuple[JointSpec, ...]
    joint_order: Tuple[str, ...]
    joints: Mapping[str, JointSpec]
    auxiliary_positions: Mapping[str, float] = field(default_factory=dict)
    source_path: Optional[str] = None
    source_hashes: Mapping[str, str] = field(default_factory=dict)
    xacro_environment: Mapping[str, str] = field(default_factory=dict)

    @property
    def limits(self):
        """A new (n, 2) array of URDF limits in radians."""
        return np.array([(self.joints[n].lower, self.joints[n].upper)
                         for n in self.joint_order], dtype=float)

    def validate_q(self, q):
        """Require a real finite vector in configured chain order; never clip."""
        try:
            raw = np.asarray(q)
        except (ValueError, TypeError) as exc:
            raise ModelError("q: expected a real one-dimensional joint vector") from exc
        if raw.shape != (len(self.joint_order),) or raw.dtype.kind not in "iuf":
            raise ModelError(f"q: expected {len(self.joint_order)} real numbers, "
                             f"shape ({len(self.joint_order)},)")
        values = raw.astype(float, copy=True)
        for name, value in zip(self.joint_order, values):
            _check_limit(self.joints[name], value)
        return values

    def joint_positions(self, q, auxiliary_positions=None):
        """Resolve a full joint state for checking auxiliary and mimic inputs.

        Off-chain independent joints default to configured values, otherwise
        zero. These states are validated but cannot enter the selected FK chain.
        """
        q = self.validate_q(q)
        auxiliary = dict(self.auxiliary_positions)
        if auxiliary_positions is not None:
            auxiliary.update(_mapping(auxiliary_positions, "auxiliary_positions"))
        for name in auxiliary:
            joint = self.joints.get(name)
            if (joint is None or joint.kind == "fixed" or joint.mimic_joint
                    or name in self.joint_order):
                raise ModelError(f"auxiliary joint {name!r}: must be an independent "
                                 "movable joint outside the selected chain")
        positions = dict(zip(self.joint_order, q.tolist()))

        def resolve(name):
            if name in positions:
                return positions[name]
            joint = self.joints[name]
            if joint.kind == "fixed":
                value = 0.0
            elif joint.mimic_joint:
                value = joint.multiplier * resolve(joint.mimic_joint) + joint.offset
            else:
                value = _number(auxiliary.get(name, 0.0), name)
            if joint.kind != "fixed":
                _check_limit(joint, value)
            positions[name] = value
            return value

        for name in self.joints:
            resolve(name)
        return positions

    def metadata(self):
        return {
            "base_frame": self.base_frame, "ee_frame": self.ee_frame,
            "joint_order": list(self.joint_order), "units": {"length": "m", "angle": "rad"},
            "chain": [asdict(j) for j in self.chain],
            "auxiliary_positions": dict(self.auxiliary_positions),
            "source_path": self.source_path, "source_hashes": dict(self.source_hashes),
            "expanded_urdf_sha256": sha256(self.urdf_xml.encode("utf-8")).hexdigest(),
            "xacro_environment": dict(self.xacro_environment),
        }


def _check_limit(joint, value):
    value = _number(value, joint.name)
    if not joint.lower <= value <= joint.upper:
        raise ModelError(f"{joint.name}: {value} rad outside URDF limits "
                         f"[{joint.lower}, {joint.upper}]")


def model_from_xml(xml, *, base_frame="base_link", ee_frame="end_effector_link",
                   joint_order: Sequence[str] = JOINT_ORDER, auxiliary_positions=None,
                   source_path=None, source_hashes=None, xacro_environment=None):
    """Parse a standard URDF string and extract the complete directed chain."""
    if (not isinstance(base_frame, str) or not base_frame
            or not isinstance(ee_frame, str) or not ee_frame or base_frame == ee_frame):
        raise ModelError("base_frame and ee_frame must be distinct nonempty link names")
    if (not isinstance(joint_order, (list, tuple)) or not joint_order
            or any(not isinstance(n, str) or not n for n in joint_order)
            or len(set(joint_order)) != len(joint_order)):
        raise ModelError("joint_order must contain unique nonempty joint names")
    try:
        root = ET.fromstring(xml)
        if root.tag != "robot":
            raise ModelError("URDF root element must be <robot>")
        # urdf_parser_py maps names to objects; reject duplicates before mapping.
        for tag in ("link", "joint"):
            names = [e.get("name") for e in root.findall(tag)]
            if not names or any(not n for n in names) or len(set(names)) != len(names):
                raise ModelError(f"URDF requires nonempty unique {tag} names")
        robot = URDF.from_xml_string(xml)
    except ModelError:
        raise
    except Exception as exc:
        raise ModelError(f"Cannot parse standard URDF: {exc}") from exc

    links = set(robot.link_map)
    parents, children, specs = {}, {}, {}
    for joint in robot.joints:
        if joint.parent not in links or joint.child not in links:
            raise ModelError(f"{joint.name}: parent/child link does not exist")
        if joint.child in parents or joint.parent == joint.child:
            raise ModelError(f"{joint.name}: invalid tree (multiple parents or self-loop)")
        if joint.type not in ("fixed", "revolute"):
            raise ModelError(f"{joint.name}: unsupported joint type {joint.type!r}; "
                             "G1-A supports fixed and bounded revolute joints")
        origin = joint.origin
        xyz = _vector(origin.xyz if origin is not None else (0, 0, 0), joint.name + ".xyz")
        rpy = _vector(origin.rpy if origin is not None else (0, 0, 0), joint.name + ".rpy")
        axis, lower, upper = None, None, None
        if joint.type == "revolute":
            # URDF's default axis is +X when <axis> is omitted.
            axis = _vector(joint.axis if joint.axis is not None else (1, 0, 0),
                           joint.name + ".axis")
            norm = math.hypot(*axis)
            if not math.isfinite(norm) or norm == 0:
                raise ModelError(f"{joint.name}: axis must have a finite nonzero norm")
            if joint.limit is None:
                raise ModelError(f"{joint.name}: revolute joint requires URDF limits")
            lower = _number(joint.limit.lower, joint.name + ".lower")
            upper = _number(joint.limit.upper, joint.name + ".upper")
            if lower > upper:
                raise ModelError(f"{joint.name}: lower limit exceeds upper limit")
        mimic = joint.mimic
        if mimic is not None and joint.type == "fixed":
            raise ModelError(f"{joint.name}: fixed joint cannot be a mimic joint")
        if mimic is not None and (not isinstance(mimic.joint, str) or not mimic.joint):
            raise ModelError(f"{joint.name}: mimic target must be a nonempty joint name")
        specs[joint.name] = JointSpec(
            joint.name, joint.type, joint.parent, joint.child, xyz, rpy, axis, lower, upper,
            mimic.joint if mimic else None,
            _number(mimic.multiplier if mimic and mimic.multiplier is not None else 1.0,
                    joint.name + ".multiplier"),
            _number(mimic.offset if mimic and mimic.offset is not None else 0.0,
                    joint.name + ".offset"))
        parents[joint.child] = joint.parent
        children.setdefault(joint.parent, []).append(joint.child)

    roots = links - set(parents)
    if len(roots) != 1:
        raise ModelError("URDF must be one connected tree with exactly one root")
    seen, pending = set(), list(roots)
    while pending:
        link = pending.pop()
        if link in seen:
            raise ModelError("URDF contains a cycle")
        seen.add(link)
        pending.extend(children.get(link, []))
    if seen != links:
        raise ModelError("URDF contains disconnected links or a cycle")

    checked, visiting = set(), set()

    def check_mimic(name):
        if name in visiting:
            raise ModelError(f"{name}: cyclic mimic dependency")
        if name in checked:
            return
        visiting.add(name)
        target = specs[name].mimic_joint
        if target:
            if target not in specs or specs[target].kind != "revolute":
                raise ModelError(f"{name}: missing or non-movable mimic target {target!r}")
            check_mimic(target)
        visiting.remove(name)
        checked.add(name)

    for name in specs:
        check_mimic(name)
    if base_frame not in links or ee_frame not in links:
        raise ModelError(f"Missing base/EE link: {base_frame!r} -> {ee_frame!r}")
    try:
        chain_names = robot.get_chain(base_frame, ee_frame, joints=True, links=False, fixed=True)
    except KeyError as exc:
        raise ModelError(f"{base_frame!r} is not an ancestor of {ee_frame!r}") from exc
    chain = tuple(specs[name] for name in chain_names)
    if any(j.mimic_joint for j in chain):
        raise ModelError("Selected chain contains a mimic joint; expected independent arm joints")
    actual = tuple(j.name for j in chain if j.kind != "fixed")
    if actual != tuple(joint_order):
        raise ModelError(f"Joint order mismatch: URDF chain {actual}, configured {tuple(joint_order)}")
    model = KinematicModel(xml, base_frame, ee_frame, chain, actual, specs,
                           _mapping({} if auxiliary_positions is None else auxiliary_positions,
                                    "auxiliary_positions"),
                           source_path, source_hashes or {}, xacro_environment or {})
    # Validate auxiliary keys, values and mimic limits without assuming q=0 is legal.
    model.joint_positions(model.limits.mean(axis=1))
    return model


def _expand_file(path, environment):
    try:
        text = path.read_text(encoding="utf-8")
        sources = {path}
        if path.suffix.lower() == ".xacro" or "wiki/xacro" in text:
            if set(environment) != {"MACHINE_TYPE", "LIDAR_TYPE"}:
                raise ModelError("Xacro loading requires explicit MACHINE_TYPE and LIDAR_TYPE "
                                 "in the configuration, rather than inherited shell values")
            import xacro
            # xacro and its include bookkeeping use global state. Serialize our
            # calls, isolate the include list, and always restore shell values.
            with _XACRO_LOCK:
                previous = {k: os.environ.get(k) for k in environment}
                old_includes = xacro.all_includes
                xacro.all_includes = []
                try:
                    os.environ.update(environment)
                    text = xacro.process_file(str(path)).toxml()
                    sources.update(Path(p).resolve() for p in xacro.all_includes)
                finally:
                    xacro.all_includes = old_includes
                    for key, value in previous.items():
                        if value is None:
                            os.environ.pop(key, None)
                        else:
                            os.environ[key] = value
            used_environment = environment
        else:
            used_environment = {}
        hashes = {str(p): sha256(p.read_bytes()).hexdigest() for p in sorted(sources)}
        return text, hashes, used_environment
    except Exception as exc:
        raise ModelError(f"Cannot load/expand model {path}: {exc}. "
                         "For package:// / $(find ...) resources, source the ROS overlay.") from exc


def load_model(config_path, *, model_path=None):
    """Load YAML and a .urdf/.xacro. No ROS node or controller is started.

    Config model.path is relative to the YAML; an explicit model_path override
    is relative to the caller's CWD. Without either, use the ament share index.
    need_compile is deliberately neither read nor required.
    """
    config_path = Path(config_path).expanduser().resolve()
    try:
        data = _mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")), "config")
        if set(data) - {"model", "xacro", "auxiliary_positions"}:
            raise ModelError("Unknown top-level configuration key")
        settings = _mapping(data.get("model"), "model")
        allowed = {"description_package", "xacro_relative_path", "path", "base_frame",
                   "ee_frame", "joint_order"}
        if set(settings) - allowed:
            raise ModelError("Unknown model configuration key")
        environment = _mapping(data.get("xacro", {}), "xacro")
        if set(environment) - {"MACHINE_TYPE", "LIDAR_TYPE"}:
            raise ModelError("xacro accepts MACHINE_TYPE and LIDAR_TYPE only; "
                             "need_compile is a Launch path switch, not an FK parameter")
        if any(not isinstance(v, str) or not v for v in environment.values()):
            raise ModelError("Xacro environment values must be nonempty strings")
        if model_path is not None:
            path = Path(model_path).expanduser().resolve()
        elif settings.get("path"):
            path = Path(settings["path"]).expanduser()
            path = (path if path.is_absolute() else config_path.parent / path).resolve()
        else:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory(settings.get("description_package", "jetrover_description"))
            path = (Path(share) / settings.get("xacro_relative_path", "urdf/jetrover.xacro")).resolve()
        xml, hashes, used_environment = _expand_file(path, environment)
        return model_from_xml(
            xml, base_frame=settings.get("base_frame", "base_link"),
            ee_frame=settings.get("ee_frame", "end_effector_link"),
            joint_order=settings.get("joint_order", JOINT_ORDER),
            auxiliary_positions=data.get("auxiliary_positions", {}),
            source_path=str(path), source_hashes=hashes, xacro_environment=used_environment)
    except ModelError:
        raise
    except Exception as exc:
        raise ModelError(f"Cannot load configuration {config_path}: {exc}") from exc
