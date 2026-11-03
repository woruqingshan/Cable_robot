"""G1-B integration support. Importing this module starts no ROS nodes.

Each case owns a new RSP process, private topics and a new, filtered TF Buffer.
G1-A model loading/FK are deliberately not modified here.
"""

from contextlib import contextmanager
from hashlib import sha256
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


CASES = (
    ("Zero", (0.0, 0.0, 0.0, 0.0, 0.0)),
    ("Nonzero", (0.35, -0.60, 0.80, -0.40, 0.20)),
    ("Near-limit", (2.08, -0.40, 0.50, -0.30, 0.25)),
)
POSITION_TOLERANCE = 1e-5
ROTATION_TOLERANCE = 1e-4


class TFValidationError(RuntimeError):
    """Model identity, publisher identity, synchronization or process failure."""


class TFTimeoutError(TFValidationError):
    """A bounded discovery, parameter or TF wait did not complete."""


class EndpointDiscoveryPending(TFValidationError):
    """A graph observation is incomplete, not evidence of a foreign endpoint."""

    def __init__(self, message, state="UNRESOLVED_DISCOVERY"):
        super().__init__(message)
        self.state = state


class EndpointIdentityUnresolved(TFTimeoutError):
    code = "ENDPOINT_IDENTITY_UNRESOLVED"


def choose_domain(requested=None, inherited=None):
    """Conservative Linux domain range; always differ from the caller's domain."""
    try:
        current = int(inherited if inherited is not None else "0")
    except (TypeError, ValueError) as exc:
        raise TFValidationError("Inherited ROS_DOMAIN_ID must be an integer") from exc
    selected = (81 if current != 81 else 82) if requested is None else requested
    if isinstance(selected, bool) or not isinstance(selected, int) or not 0 <= selected <= 101:
        raise TFValidationError("Use an integer domain ID in 0..101")
    if selected == current:
        raise TFValidationError("Test domain must differ from inherited ROS_DOMAIN_ID")
    return selected


@contextmanager
def temporary_environment(values):
    previous = {key: os.environ.get(key) for key in values}
    try:
        os.environ.update(values)
        yield
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


def wait_until(label, predicate, spin, guard, timeout, *, detail=lambda: "", clock=time.monotonic,
               on_timeout=None):
    """Completion is event/data driven, never inferred from elapsed sleep."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise TFValidationError("timeout must be a finite positive number")
    deadline = clock() + timeout
    while True:
        allowed = guard()
        if allowed is not False and predicate():
            return
        left = deadline - clock()
        if left <= 0:
            if on_timeout is not None:
                on_timeout()
            raise TFTimeoutError(f"{label} timed out after {timeout:g}s: {detail()}")
        spin(min(0.05, left))


def verify_rsp_parameters(values, expected_xml):
    actual = values.get("robot_description")
    if actual != expected_xml:
        raise TFValidationError("RSP robot_description differs from the exact G1-A URDF string")
    if values.get("use_sim_time") is not False or values.get("frame_prefix") != "":
        raise TFValidationError("RSP must use system time and an empty frame_prefix")
    if values.get("ignore_timestamp") is not True:
        raise TFValidationError("RSP retry configuration ignore_timestamp must be true")
    return sha256(actual.encode("utf-8")).hexdigest()


def _unknown(value):
    return not value or value in ("_NODE_NAME_UNKNOWN_", "_NODE_NAMESPACE_UNKNOWN_")


def _enum_name(value):
    return getattr(value, "name", str(value) if value is not None else "UNKNOWN")


def endpoint_diagnostic(endpoint):
    qos = getattr(endpoint, "qos_profile", None)
    profile = {key: _enum_name(getattr(qos, key, None)) for key in
               ("reliability", "durability", "history", "liveliness")}
    profile["depth"] = getattr(qos, "depth", None)
    for key in ("deadline", "lifespan", "liveliness_lease_duration"):
        profile[key + "_ns"] = getattr(getattr(qos, key, None), "nanoseconds", None)
    return {"node_name": getattr(endpoint, "node_name", None),
            "node_namespace": getattr(endpoint, "node_namespace", None),
            "topic_type": getattr(endpoint, "topic_type", None),
            "endpoint_type": _enum_name(getattr(endpoint, "endpoint_type", None)),
            "gid": bytes(getattr(endpoint, "endpoint_gid", [])).hex(), "qos": profile}


def require_single_endpoint(endpoints, namespace, name, label, *, topic_type=None,
                            role=None, qos=None, node_topics=None, topic=None, node_visible=True):
    if not endpoints:
        raise EndpointDiscoveryPending(f"{label}: endpoint not discovered", "NOT_DISCOVERED")
    if len(endpoints) != 1:
        raise TFValidationError(f"{label}: expected one endpoint, found {len(endpoints)}")
    endpoint = endpoints[0]
    # A known contradictory field is reliable rejection evidence even if the
    # other identity field is still UNKNOWN.
    if ((not _unknown(endpoint.node_namespace) and endpoint.node_namespace != namespace)
            or (not _unknown(endpoint.node_name) and endpoint.node_name != name)):
        raise TFValidationError(f"{label}: unexpected endpoint {endpoint.node_namespace}/{endpoint.node_name}")
    if topic_type and getattr(endpoint, "topic_type", None) not in (None, "", topic_type):
        raise TFValidationError(f"{label}: unexpected topic type {endpoint.topic_type}")
    if _unknown(endpoint.node_namespace) or _unknown(endpoint.node_name):
        raise EndpointDiscoveryPending(f"{label}: node identity is UNKNOWN")
    gid = bytes(endpoint.endpoint_gid)
    if not gid or not any(gid):
        raise EndpointDiscoveryPending(f"{label}: endpoint GID is unresolved")
    if topic_type and not getattr(endpoint, "topic_type", None):
        raise EndpointDiscoveryPending(f"{label}: topic type is unresolved")
    if role:
        actual_role = _enum_name(getattr(endpoint, "endpoint_type", None))
        if actual_role in ("UNKNOWN", "INVALID"):
            raise EndpointDiscoveryPending(f"{label}: endpoint role is unresolved")
        if actual_role != role:
            raise TFValidationError(f"{label}: unexpected endpoint role {actual_role}")
    if qos is not None:
        profile = getattr(endpoint, "qos_profile", None)
        for key, accepted in qos.items():
            value = _enum_name(getattr(profile, key, None))
            if value in ("UNKNOWN", "SYSTEM_DEFAULT"):
                raise EndpointDiscoveryPending(f"{label}: QoS {key} is unresolved")
            if value not in accepted:
                raise TFValidationError(f"{label}: incompatible QoS {key}={value}")
        # History/depth are diagnostic, not offered/requested matching policies;
        # Fast DDS may not expose them for a remote endpoint. Reliability and
        # durability above are the relevant matching constraints for this test.
    if not node_visible:
        raise EndpointDiscoveryPending(f"{label}: expected node absent from node graph")
    if node_topics is not None:
        types = dict(node_topics).get(topic)
        if not types:
            raise EndpointDiscoveryPending(f"{label}: per-node topic graph not ready")
        if set(types) != {topic_type}:
            raise TFValidationError(f"{label}: endpoint disagrees with per-node topic graph")
    return gid.hex()


class EndpointMonitor:
    """Bounded re-observation of graph identity, with retained diagnostics.

    Strict policy only. UNKNOWN never becomes an implicit isolated-topic PASS.
    Runtime gaps also block acceptance and message ingestion until reverified.
    """

    def __init__(self, node, namespace, timeout, *, clock=time.monotonic):
        self.node, self.namespace, self.timeout, self.clock = node, namespace, timeout, clock
        self.records, self.locked_gids, self.pending_since = {}, {}, {}

    def check(self, label, topic, name, role, topic_type, qos):
        getter = (self.node.get_publishers_info_by_topic if role == "PUBLISHER"
                  else self.node.get_subscriptions_info_by_topic)
        endpoints = getter(topic)
        nodes = self.node.get_node_names_and_namespaces()
        matches = nodes.count((name, self.namespace))
        query = (self.node.get_publisher_names_and_types_by_node if role == "PUBLISHER"
                 else self.node.get_subscriber_names_and_types_by_node)
        node_topics, graph_error = [], None
        if matches == 1:
            try:
                node_topics = query(name, self.namespace)
            except Exception as exc:
                if type(exc).__name__ != "NodeNameNonExistentError":
                    raise
                graph_error = str(exc)
        record = self.records.setdefault(label, {"first_seen_monotonic": self.clock(), "transitions": []})
        record.update(topic=topic, expected_node=[name, self.namespace], role=role,
                      expected_topic_type=topic_type, graph_nodes=nodes[:64], node_topics=node_topics,
                      graph_error=graph_error, endpoints=[endpoint_diagnostic(e) for e in endpoints])
        try:
            if matches > 1:
                raise TFValidationError(f"{label}: multiple graph nodes have the expected identity")
            gid = require_single_endpoint(endpoints, self.namespace, name, label, topic_type=topic_type,
                                          role=role, qos=qos, node_topics=node_topics, topic=topic,
                                          node_visible=matches == 1)
            if label in self.locked_gids and self.locked_gids[label] != gid:
                raise TFValidationError(f"{label}: endpoint GID changed after verification")
        except EndpointDiscoveryPending as exc:
            self._record(label, exc.state, str(exc))
            since = self.pending_since.setdefault(label, self.clock())
            if self.clock() - since >= self.timeout:
                self.raise_pending_timeout()
            return None
        except TFValidationError as exc:
            self._record(label, "REJECTED", str(exc))
            raise
        self.locked_gids[label] = gid
        self._record(label, "VERIFIED", "endpoint, QoS and per-node graph agree")
        self.pending_since.pop(label, None)
        return gid

    def _record(self, label, state, reason):
        record = self.records[label]
        now = self.clock()
        if record.get("state") != state:
            record["transitions"].append({"state": state, "reason": reason, "monotonic": now,
                                          "endpoints": record["endpoints"], "graph_nodes": record["graph_nodes"]})
            record["transitions"] = record["transitions"][-32:]
        record.update(state=state, reason=reason, last_seen_monotonic=now,
                      wait_elapsed_s=now - self.pending_since.get(label, now))

    def raise_pending_timeout(self):
        if self.pending_since:
            unresolved = any(self.records[k]["state"] == "UNRESOLVED_DISCOVERY" for k in self.pending_since)
            code = "ENDPOINT_IDENTITY_UNRESOLVED" if unresolved else "ENDPOINT_NOT_DISCOVERED"
            error = EndpointIdentityUnresolved if unresolved else TFTimeoutError
            raise error(f"{code}: {self.summary()}")

    def summary(self):
        return {key: {"state": value["state"], "reason": value["reason"],
                      "wait_elapsed_s": max(value["wait_elapsed_s"],
                                            self.clock() - self.pending_since.get(key, self.clock()))}
                for key, value in self.records.items()}

    def diagnostics(self):
        return {key: {**value, **self.summary()[key]} for key, value in self.records.items()}


def wait_for_rsp_initialization(node, client, spin, guard, timeout, read_parameters, progress):
    """No endpoint identity check may run before the constructor barrier."""
    last_nodes = []
    def node_visible():
        nodes = node.get_node_names_and_namespaces()
        last_nodes[:] = nodes
        expected = ("rsp", node.get_namespace())
        if nodes.count(expected) > 1:
            raise TFValidationError("Multiple RSP nodes in the private namespace")
        return expected in nodes
    wait_until("RSP node discovery", node_visible, spin, guard, timeout,
               detail=lambda: f"node graph: {last_nodes}")
    progress("ROS_NODE_DISCOVERED")
    wait_until("RSP parameter service", client.service_is_ready, spin, guard, timeout)
    digest = read_parameters()
    progress("MODEL_PARAMETERS_VERIFIED")
    return digest


class TransformAudit:
    """TF endpoint/freshness gate, independent of Buffer interpolation.

    Only exact-stamp dynamic transforms on a private topic with the sole
    discovered RSP endpoint may enter the Buffer. Humble's callback does not
    expose per-message GIDs; source IDs here are graph endpoint observations.
    All model edges (including gripper/mimic edges) must be observed.
    """

    def __init__(self, edges, dynamic_gid, static_gid):
        self.edges = dict(edges)  # {(parent, child): "fixed" or "revolute"}
        self.sources = {False: dynamic_gid, True: static_gid}
        self.expected_stamp_ns = None
        self.dynamic, self.static = {}, {}
        self.discarded_count = 0
        self.discarded_stamps = set()

    def arm(self, stamp_ns):
        if self.expected_stamp_ns is not None or stamp_ns <= 0:
            raise TFValidationError("Each fresh audit accepts exactly one positive request stamp")
        self.expected_stamp_ns = stamp_ns

    def observe(self, parent, child, stamp_ns, publisher_gid, *, static):
        if publisher_gid != self.sources[static]:
            raise TFValidationError("TF publisher GID differs from the discovered RSP publisher")
        edge = (parent, child)
        if edge not in self.edges or (self.edges[edge] == "fixed") != static:
            raise TFValidationError(f"Unexpected TF edge/type: {parent} -> {child}, static={static}")
        if not static and (self.expected_stamp_ns is None or stamp_ns != self.expected_stamp_ns):
            self.discarded_count += 1
            if len(self.discarded_stamps) < 16:
                self.discarded_stamps.add(stamp_ns)
            return False
        (self.static if static else self.dynamic)[edge] = stamp_ns
        return True

    def missing(self, static):
        observed = self.static if static else self.dynamic
        return sorted(edge for edge, kind in self.edges.items()
                      if (kind == "fixed") == static and edge not in observed)

    def complete(self):
        return self.expected_stamp_ns is not None and not self.missing(True) and not self.missing(False)

    def evidence(self):
        return {
            "request_stamp_ns": self.expected_stamp_ns,
            "source_check": "exclusive_private_topic_endpoint_graph",
            "per_message_gid_available": False,
            "dynamic_endpoint_gid": self.sources[False], "static_endpoint_gid": self.sources[True],
            "dynamic_edges": [{"parent": p, "child": c, "stamp_ns": stamp}
                              for (p, c), stamp in sorted(self.dynamic.items())],
            "static_edges": [{"parent": p, "child": c, "stamp_ns": stamp}
                             for (p, c), stamp in sorted(self.static.items())],
            "discarded_dynamic_count": self.discarded_count,
            "discarded_stamp_examples_ns": sorted(self.discarded_stamps),
            "missing_dynamic_edges": self.missing(False), "missing_static_edges": self.missing(True),
        }


def matrix_from_tf(message):
    transform = message.transform
    q = np.array([transform.rotation.x, transform.rotation.y,
                  transform.rotation.z, transform.rotation.w], dtype=float)
    position = np.array([transform.translation.x, transform.translation.y,
                         transform.translation.z], dtype=float)
    if not np.isfinite(q).all() or not np.isfinite(position).all() or abs(np.linalg.norm(q) - 1) > 1e-6:
        raise TFValidationError("TF returned a non-finite pose or non-unit quaternion")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(q).as_matrix()
    matrix[:3, 3] = position
    return matrix


def compare_poses(kdl_matrix, tf_matrix):
    for matrix in (kdl_matrix, tf_matrix):
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-10, rtol=0)
                or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-10):
            raise TFValidationError("Invalid homogeneous transform in pose comparison")
    position = float(np.linalg.norm(kdl_matrix[:3, 3] - tf_matrix[:3, 3]))
    angle = float(Rotation.from_matrix(kdl_matrix[:3, :3].T @ tf_matrix[:3, :3]).magnitude())
    return {"position_error_m": position, "rotation_error_rad": angle,
            "numerical_pass": position <= POSITION_TOLERANCE and angle <= ROTATION_TOLERANCE}


class ManagedRSP:
    """Own one direct RSP child/session. Never signal a name-matched process."""

    def __init__(self, command, environment, log_path):
        self.command, self.environment, self.log_path = command, environment, Path(log_path)
        self.process, self.log = None, None
        self.initialized = False
        self.prepare_stop = None

    def start(self):
        try:
            self.log = self.log_path.open("w", encoding="utf-8")
            self.process = subprocess.Popen(
                self.command, env=self.environment, stdout=self.log, stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL, start_new_session=True)
        except Exception as exc:
            if self.log is not None:
                self.log.close()
            raise TFValidationError(f"RSP launch failed: {exc}") from exc

    def check_alive(self):
        if self.process is None or self.process.poll() is not None:
            code = None if self.process is None else self.process.returncode
            raise TFValidationError(f"RSP exited before verification completed (code={code})")

    def close(self):
        outcome = {"pid": None, "reaped": True, "process_group_gone": True, "signals": [], "errors": []}
        try:
            if self.process is not None:
                outcome["pid"] = self.process.pid
                for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
                    try:
                        os.killpg(self.process.pid, 0)
                    except ProcessLookupError:
                        break
                    try:
                        os.killpg(self.process.pid, sig)
                    except ProcessLookupError:
                        break
                    outcome["signals"].append(sig.name)
                    try:
                        self.process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        continue
                    # Direct RSP has no wrapper/expected children. Check the
                    # entire owned session too, rather than only the parent PID.
                self.process.wait(timeout=2.0)
                outcome["returncode"] = self.process.returncode
                outcome["reaped"] = self.process.poll() is not None
                try:
                    os.killpg(self.process.pid, 0)
                    outcome["process_group_gone"] = False
                except ProcessLookupError:
                    pass
        except Exception as exc:
            outcome["errors"].append(f"RSP cleanup: {type(exc).__name__}: {exc}")
            outcome["reaped"] = False
        finally:
            if self.log is not None:
                self.log.close()
            if self.log_path.exists():
                with self.log_path.open("rb") as stream:
                    stream.seek(max(0, self.log_path.stat().st_size - 16000))
                    outcome["rsp_log_tail"] = stream.read().decode("utf-8", errors="replace")
        outcome["ok"] = outcome["reaped"] and outcome["process_group_gone"] and not outcome["errors"]
        outcome.update(cleanup_resources_released=outcome["ok"],
                       rsp_exit_code=outcome.get("returncode"),
                       rsp_exit_normal=(outcome.get("returncode") == 0 if self.process is not None else None),
                       initialization_barrier_reached=self.initialized)
        return outcome


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def close_resources(child, executor, node, context):
    """Finish owned cleanup even after a second Ctrl+C/SIGTERM (not SIGKILL)."""
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, signal.SIG_IGN)
    try:
        readiness_error = None
        if child is not None and getattr(child, "prepare_stop", None) is not None:
            try:
                child.prepare_stop()
            except Exception as exc:
                # A bounded grace period is exhausted: stop the owned child,
                # retain the failure, and never describe it as normal readiness.
                readiness_error = f"{type(exc).__name__}: {exc}"
        try:
            outcome = child.close() if child is not None else {"ok": True, "pid": None, "errors": []}
        except Exception as exc:
            outcome = {"ok": False, "errors": [f"RSP cleanup: {exc}"]}
        for label, action in (
            ("executor", lambda: executor.shutdown(timeout_sec=2.0) if executor else None),
            ("node", lambda: node.destroy_node() if node else None),
            ("context", lambda: context.shutdown() if context and context.ok() else None),
        ):
            try:
                if action() is False:
                    raise TFValidationError("resource shutdown returned false")
            except Exception as exc:
                outcome["errors"].append(f"{label}: {exc}")
        outcome["ok"] = outcome["ok"] and not outcome["errors"]
        outcome["cleanup_resources_released"] = outcome["ok"]
        outcome["shutdown_readiness_error"] = readiness_error
        return outcome
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def _get_parameters(node, executor, client, guard, timeout, xml, *, on_response=None, on_timeout=None):
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType

    request = GetParameters.Request()
    request.names = ["robot_description", "use_sim_time", "frame_prefix", "ignore_timestamp"]
    future = client.call_async(request)
    wait_until("RSP get_parameters", future.done, lambda t: executor.spin_once(timeout_sec=t), guard, timeout,
               on_timeout=on_timeout)
    response = future.result()
    if response is None or len(response.values) != len(request.names):
        raise TFValidationError("RSP get_parameters returned an incomplete response")
    if on_response is not None:
        on_response()
    values = {}
    for name, value in zip(request.names, response.values):
        if value.type == ParameterType.PARAMETER_STRING:
            values[name] = value.string_value
        elif value.type == ParameterType.PARAMETER_BOOL:
            values[name] = value.bool_value
        else:
            raise TFValidationError(f"Unexpected RSP parameter type for {name}")
    return verify_rsp_parameters(values, xml)


def run_case(model, fk, case, row, *, domain_id, timeout, namespace):
    """Run one isolated case, always populate cleanup information in row."""
    # Invalid q/frame fails BEFORE a Context or child process can be created.
    name, q = case
    positions = model.joint_positions(q)
    if model.base_frame not in {j.parent for j in model.chain} or model.ee_frame != model.chain[-1].child:
        raise TFValidationError("Missing Base/EE frame in selected model chain")
    kdl_matrix = fk(q)
    independent = [j.name for j in model.joints.values() if j.kind != "fixed" and not j.mimic_joint]
    row.update(name=name, q_rad=list(q), status="RUNNING", passed=False, namespace=namespace,
               published_joint_names=independent, published_positions_rad=[positions[n] for n in independent])

    # ROS imports are lazy: unit tests and static inspection start no DDS context.
    import rclpy
    from rclpy.context import Context
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.time import Time
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException
    from ament_index_python.packages import get_package_prefix

    context, node, executor, child, audit, monitor = None, None, None, None, None, None
    callback_errors = []
    with tempfile.TemporaryDirectory(prefix="g1b_tf_") as temporary:
        with temporary_environment({"ROS_DOMAIN_ID": str(domain_id), "ROS_LOCALHOST_ONLY": "1",
                                    "ROS_LOG_DIR": str(Path(temporary) / "ros_logs")}):
            try:
                prefix = get_package_prefix("robot_state_publisher")
                executable = Path(prefix) / "lib/robot_state_publisher/robot_state_publisher"
                if not executable.is_file() or not os.access(executable, os.X_OK):
                    raise TFValidationError(f"RSP executable missing/not executable: {executable}")
                topics = {key: f"{namespace}/{key}" for key in ("joint_states", "tf", "tf_static")}
                context = Context()
                context.init(args=[], domain_id=domain_id)
                from rclpy.utilities import get_rmw_implementation_identifier
                row["rmw_identifier"] = get_rmw_implementation_identifier()
                node = rclpy.create_node("probe", namespace=namespace, context=context,
                                         use_global_arguments=False, enable_rosout=False,
                                         parameter_overrides=[Parameter("use_sim_time", value=False)])
                executor = SingleThreadedExecutor(context=context)
                executor.add_node(node)
                buffer = Buffer(cache_time=Duration(seconds=max(10.0, timeout * 2)))
                reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
                static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
                publisher = node.create_publisher(JointState, topics["joint_states"], reliable)
                client = node.create_client(GetParameters, f"{namespace}/rsp/get_parameters")
                parameters = {"/**": {"ros__parameters": {
                    "robot_description": model.urdf_xml, "use_sim_time": False,
                    "frame_prefix": "", "publish_frequency": 100.0,
                    # Re-send the SAME request stamp until receipt is proven.
                    # This disables RSP throttling only; our gate checks stamps.
                    "ignore_timestamp": True}}}
                params_path = Path(temporary) / "rsp.yaml"
                params_path.write_text(yaml.safe_dump(parameters), encoding="utf-8")
                command = [str(executable), "--ros-args", "--params-file", str(params_path),
                           "-r", "__node:=rsp", "-r", f"__ns:={namespace}",
                           "-r", f"joint_states:={topics['joint_states']}",
                           "-r", f"/tf:={topics['tf']}", "-r", f"/tf_static:={topics['tf_static']}"]
                child = ManagedRSP(command, os.environ.copy(), Path(temporary) / "rsp.log")
                child.start()
                row.update(rsp_pid=child.process.pid, rsp_executable=str(executable), topics=topics,
                           ros_domain_id=domain_id, use_sim_time=False,
                           strict_endpoint_identity_verified=False, isolated_topic_source_checked=False,
                           readiness_stages=[])

                def progress(stage):
                    row["readiness_stages"].append({"stage": stage, "monotonic": time.monotonic()})
                    row["readiness_stage"] = stage

                progress("PROCESS_STARTED")

                def guard():
                    child.check_alive()
                    if not context.ok():
                        raise TFValidationError("ROS context stopped unexpectedly")
                    if callback_errors:
                        raise callback_errors[0]

                spin = lambda t: executor.spin_once(timeout_sec=t)

                def mark_constructor_complete():
                    child.initialized = True

                def prepare_stop():
                    if child.initialized or child.process.poll() is not None or not context.ok():
                        return
                    grace = min(timeout, 2.0)
                    wait_until("RSP shutdown constructor barrier", client.service_is_ready, spin,
                               child.check_alive, grace)
                    _get_parameters(node, executor, client, child.check_alive, grace, model.urdf_xml,
                                    on_response=mark_constructor_complete)

                child.prepare_stop = prepare_stop
                row["model_sha256_readback_before"] = wait_for_rsp_initialization(
                    node, client, spin, guard, timeout,
                    lambda: _get_parameters(node, executor, client, guard, timeout, model.urdf_xml,
                                            on_response=mark_constructor_complete), progress)
                monitor = EndpointMonitor(node, namespace, timeout)
                specs = {
                    "tf": (topics["tf"], "rsp", "PUBLISHER", "tf2_msgs/msg/TFMessage",
                           {"reliability": {"RELIABLE"}, "durability": {"VOLATILE"}}),
                    "tf_static": (topics["tf_static"], "rsp", "PUBLISHER", "tf2_msgs/msg/TFMessage",
                                  {"reliability": {"RELIABLE"}, "durability": {"TRANSIENT_LOCAL"}}),
                    "joint_state_subscriber": (topics["joint_states"], "rsp", "SUBSCRIPTION", "sensor_msgs/msg/JointState",
                                               {"reliability": {"BEST_EFFORT", "RELIABLE"}, "durability": {"VOLATILE"}}),
                    "joint_state_publisher": (topics["joint_states"], "probe", "PUBLISHER", "sensor_msgs/msg/JointState",
                                              {"reliability": {"RELIABLE"}, "durability": {"VOLATILE"}}),
                }

                def verified_labels(labels):
                    observed = [monitor.check(key, *specs[key]) for key in labels]
                    return all(gid is not None for gid in observed)

                wait_until("Strict RSP endpoint discovery", lambda: verified_labels(tuple(specs)), spin,
                           guard, timeout, detail=monitor.summary, on_timeout=monitor.raise_pending_timeout)
                progress("PUBLISHERS_AND_JOINTSTATE_ENDPOINTS_VERIFIED")
                sources = {key: monitor.locked_gids[key] for key in ("tf", "tf_static")}
                audit = TransformAudit({(j.parent, j.child): j.kind for j in model.joints.values()},
                                        sources["tf"], sources["tf_static"])

                def source_gid(key):
                    gid = monitor.check(key, *specs[key])
                    if gid is not None and gid != sources[key]:
                        raise TFValidationError("RSP TF endpoint identity changed during the case")
                    return gid

                # Preserve the latched static message if graph identity becomes
                # temporarily unresolved in its callback. Two bounded slots;
                # no unverified data enters the Buffer.
                pending_messages = {"tf": None, "tf_static": None}

                def drain_pending():
                    for key, message in list(pending_messages.items()):
                        if message is None:
                            continue
                        gid = source_gid(key)
                        if gid is None:
                            continue
                        static = key == "tf_static"
                        for transform in message.transforms:
                            accepted = audit.observe(transform.header.frame_id, transform.child_frame_id,
                                                     _stamp_ns(transform.header.stamp), gid, static=static)
                            if accepted:
                                if static:
                                    buffer.set_transform_static(transform, sources["tf_static"])
                                else:
                                    buffer.set_transform(transform, sources["tf"])
                        pending_messages[key] = None

                def callback(static):
                    def receive(message):
                        try:
                            # Installed Humble rclpy callbacks receive ONLY the
                            # message. Check the graph, not a nonexistent info API.
                            pending_messages["tf_static" if static else "tf"] = message
                            drain_pending()
                        except Exception as exc:
                            callback_errors.append(exc if isinstance(exc, TFValidationError) else
                                                   TFValidationError(f"TF subscription: {type(exc).__name__}: {exc}"))
                    return receive

                node.create_subscription(TFMessage, topics["tf"], callback(False), reliable)
                node.create_subscription(TFMessage, topics["tf_static"], callback(True), static_qos)
                specs.update({
                    "tf_listener": (topics["tf"], "probe", "SUBSCRIPTION", "tf2_msgs/msg/TFMessage",
                                    {"reliability": {"RELIABLE"}, "durability": {"VOLATILE"}}),
                    "tf_static_listener": (topics["tf_static"], "probe", "SUBSCRIPTION", "tf2_msgs/msg/TFMessage",
                                           {"reliability": {"RELIABLE"}, "durability": {"TRANSIENT_LOCAL"}}),
                })

                def provenance_guard():
                    guard()
                    return verified_labels(tuple(specs))

                def listeners_ready():
                    drain_pending()
                    return not monitor.pending_since and not audit.missing(True)

                wait_until("TF static listener readiness", listeners_ready, spin,
                           provenance_guard, timeout, detail=lambda: f"{monitor.summary()}; missing fixed edges: {audit.missing(True)}",
                           on_timeout=monitor.raise_pending_timeout)
                progress("READY_TO_PUBLISH_JOINTSTATE")
                stamp = node.get_clock().now()
                audit.arm(stamp.nanoseconds)
                message = JointState()
                message.header.stamp = stamp.to_msg()
                message.name = independent
                message.position = [positions[n] for n in independent]
                published = [0]
                next_publish = [0.0]
                found = []
                last_lookup_error = [""]

                def ready():
                    drain_pending()
                    if monitor.pending_since:
                        return False
                    now = time.monotonic()
                    if now >= next_publish[0]:
                        try:
                            publisher.publish(message)
                        except Exception as exc:
                            raise TFValidationError(f"JointState publish failed: {exc}") from exc
                        published[0] += 1
                        next_publish[0] = now + 0.2
                    if not audit.complete():
                        return False
                    try:
                        transform = buffer.lookup_transform(model.base_frame, model.ee_frame,
                                                            Time(nanoseconds=stamp.nanoseconds))
                    except TransformException as exc:
                        last_lookup_error[0] = str(exc)
                        return False
                    if _stamp_ns(transform.header.stamp) != stamp.nanoseconds:
                        raise TFValidationError("Composed TF timestamp does not match this case's request")
                    if transform.header.frame_id != model.base_frame or transform.child_frame_id != model.ee_frame:
                        raise TFValidationError("TF lookup returned unexpected Base/EE frame names")
                    found.append(transform)
                    return True

                wait_until("Exact-stamp TF lookup", ready, spin, provenance_guard, timeout,
                           detail=lambda: f"missing dynamic: {audit.missing(False)}; missing static: "
                           f"{audit.missing(True)}; Frame/lookup error: {last_lookup_error[0]}; {monitor.summary()}",
                           on_timeout=monitor.raise_pending_timeout)
                row["model_sha256_readback_after"] = _get_parameters(node, executor, client, provenance_guard,
                                                                     timeout, model.urdf_xml,
                                                                     on_timeout=monitor.raise_pending_timeout)
                wait_until("Final endpoint identity confirmation", lambda: True, spin, provenance_guard,
                           timeout, detail=monitor.summary, on_timeout=monitor.raise_pending_timeout)
                tf_matrix = matrix_from_tf(found[0])
                row.update(compare_poses(kdl_matrix, tf_matrix))
                for label, matrix in (("kdl", kdl_matrix), ("tf", tf_matrix)):
                    row[label] = {"position_m": matrix[:3, 3].tolist(), "rotation": matrix[:3, :3].tolist(),
                                  "quaternion_xyzw": Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist(),
                                  "T_base_to_ee": matrix.tolist()}
                row.update(tf_stamp_ns=_stamp_ns(found[0].header.stamp), joint_state_publish_count=published[0],
                           sync_complete=True, status="COMPARISON_COMPLETE",
                           strict_endpoint_identity_verified=True, isolated_topic_source_checked=True)
            except BaseException as exc:
                row.update(status="ERROR", passed=False, error=f"{type(exc).__name__}: {exc}")
                row["error_code"] = getattr(exc, "code", type(exc).__name__)
                row["strict_endpoint_identity_verified"] = False
                raise
            finally:
                if monitor is not None:
                    row["endpoint_diagnostics"] = monitor.diagnostics()
                if audit is not None:
                    row["synchronization"] = audit.evidence()
                outcome = close_resources(child, executor, node, context)
                row["cleanup"] = outcome
                row.update(cleanup_resources_released=outcome["cleanup_resources_released"],
                           rsp_exit_normal=outcome.get("rsp_exit_normal"), rsp_exit_code=outcome.get("rsp_exit_code"))
                row["passed"] = (row.get("numerical_pass", False) and row.get("sync_complete", False)
                                  and row.get("strict_endpoint_identity_verified", False)
                                  and outcome["ok"] and outcome.get("rsp_exit_normal") is True and "error" not in row)
                if "error" not in row:
                    row["status"] = "PASS" if row["passed"] else "FAIL"
                if not outcome["ok"]:
                    raise TFValidationError(f"Owned resource cleanup failed: {outcome['errors']}")


def finalize_report(report):
    rows = report.get("cases", [])
    passed = (not report.get("error") and len(rows) == len(CASES)
              and [r.get("name") for r in rows] == [c[0] for c in CASES]
              and all(r.get("passed") is True and r.get("numerical_pass") is True
                      and r.get("sync_complete") is True and r.get("status") == "PASS"
                      and r.get("cleanup", {}).get("ok") is True
                      and r.get("cleanup", {}).get("rsp_exit_normal") is not False for r in rows))
    if report.get("report_version", 0) >= 3:
        passed = passed and all(
            r.get("strict_endpoint_identity_verified") is True
            and r.get("isolated_topic_source_checked") is True
            and r.get("cleanup", {}).get("cleanup_resources_released") is True
            and r.get("cleanup", {}).get("rsp_exit_normal") is True
            and r.get("cleanup", {}).get("rsp_exit_code") == 0 for r in rows)
    report.update(status="PASS" if passed else "FAIL", hardware_checked=False,
                  stage_acceptance="pending_user_review",
                  tf_checked=len(rows) == len(CASES) and all(r.get("sync_complete", False) for r in rows))
    return passed
