"""No-DDS unit tests for G1-B gates, exceptions and resource ownership.

Real three-case integration is performed ONLY by the user-run validate_tf.py.
No test in this file creates a ROS Context or starts an RSP process.
"""

from types import SimpleNamespace
import signal
import subprocess

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from workspace_analysis.robot.model import ModelError, model_from_xml
from workspace_analysis.experiments.tf_validation import (
    CASES, ManagedRSP, TFTimeoutError, TFValidationError, TransformAudit,
    choose_domain, close_resources, compare_poses, finalize_report, matrix_from_tf,
    require_single_endpoint, run_case, temporary_environment, verify_rsp_parameters, wait_until,
    EndpointMonitor, EndpointIdentityUnresolved, wait_for_rsp_initialization,
)


EDGES = {("base", "link"): "revolute", ("link", "tip"): "fixed"}


def gate():
    audit = TransformAudit(EDGES, "dynamic", "static")
    audit.arm(123_456_789)
    audit.observe("link", "tip", 10, "static", static=True)
    return audit


@pytest.mark.parametrize("old_stamp", [0, 123_456_788, 123_456_790])
def test_old_or_different_stamp_never_completes_case(old_stamp):
    audit = gate()
    assert not audit.observe("base", "link", old_stamp, "dynamic", static=False)
    assert not audit.complete()
    assert audit.dynamic == {}
    assert audit.discarded_count == 1
    assert audit.observe("base", "link", 123_456_789, "dynamic", static=False)
    assert audit.complete()


def test_empty_buffer_context_cannot_reuse_previous_case_evidence():
    first = gate()
    first.observe("base", "link", 123_456_789, "dynamic", static=False)
    assert first.complete()
    second = TransformAudit(EDGES, "dynamic", "static")
    second.arm(223_456_789)
    assert not second.observe("base", "link", 123_456_789, "dynamic", static=False)
    assert not second.complete()
    assert second.missing(True) == [("link", "tip")]


def test_foreign_publisher_wrong_parent_and_static_type_rejected():
    audit = gate()
    for parent, child, gid, static in [
        ("base", "link", "foreign", False), ("wrong", "link", "dynamic", False),
        ("base", "link", "static", True),
    ]:
        with pytest.raises(TFValidationError):
            audit.observe(parent, child, 123_456_789, gid, static=static)
    with pytest.raises(TFValidationError):
        audit.arm(223_456_789)


def test_missing_frame_timeout_is_explicit():
    audit = gate()
    current = [0.0]
    with pytest.raises(TFTimeoutError, match="missing.*base.*link"):
        wait_until("Missing Frame TF", audit.complete, lambda dt: current.__setitem__(0, current[0] + dt),
                   lambda: None, 0.2, detail=lambda: f"missing frames: {audit.missing(False)}",
                   clock=lambda: current[0])


def test_cached_result_does_not_bypass_freshness_predicate():
    audit = gate()
    cached_transform_available = True
    current = [0.0]
    with pytest.raises(TFTimeoutError):
        wait_until("old cache", lambda: cached_transform_available and audit.complete(),
                   lambda dt: current.__setitem__(0, current[0] + dt), lambda: None,
                   0.1, clock=lambda: current[0])


def test_wait_returns_on_data_and_propagates_process_failure():
    observed = [False]
    wait_until("ready", lambda: observed[0], lambda dt: observed.__setitem__(0, True), lambda: None, 1)
    def dead():
        raise TFValidationError("RSP exited")
    with pytest.raises(TFValidationError, match="RSP exited"):
        wait_until("startup", lambda: False, lambda dt: None, dead, 1)


def test_model_parameter_identity_and_clock_flags():
    values = {"robot_description": "<robot/>", "use_sim_time": False,
              "frame_prefix": "", "ignore_timestamp": True}
    assert len(verify_rsp_parameters(values, "<robot/>")) == 64
    for key, value in [("robot_description", "different"), ("use_sim_time", True),
                       ("frame_prefix", "other/"), ("ignore_timestamp", False)]:
        with pytest.raises(TFValidationError):
            verify_rsp_parameters({**values, key: value}, "<robot/>")


def test_endpoint_identity_and_domain_isolation():
    endpoint = SimpleNamespace(node_name="rsp", node_namespace="/private", endpoint_gid=[1, 2])
    assert require_single_endpoint([endpoint], "/private", "rsp", "tf") == "0102"
    for endpoints in [[], [endpoint, endpoint]]:
        with pytest.raises(TFValidationError):
            require_single_endpoint(endpoints, "/private", "rsp", "tf")
    with pytest.raises(TFValidationError):
        require_single_endpoint([endpoint], "/other", "rsp", "tf")
    assert choose_domain(None, "0") == 81
    assert choose_domain(None, "81") == 82
    for requested in [-1, 102, 0, True]:
        with pytest.raises(TFValidationError):
            choose_domain(requested, "0")


def test_environment_restoration_on_exception(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "7")
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    import os
    with pytest.raises(RuntimeError):
        with temporary_environment({"ROS_DOMAIN_ID": "81", "ROS_LOCALHOST_ONLY": "1"}):
            raise RuntimeError("stop")
    assert os.environ["ROS_DOMAIN_ID"] == "7"
    assert "ROS_LOCALHOST_ONLY" not in os.environ


def test_pose_comparison_checks_both_tolerances_and_rotation_validity():
    baseline = np.eye(4)
    assert compare_poses(baseline, baseline)["numerical_pass"]
    moved = baseline.copy()
    moved[0, 3] = 2e-5
    assert not compare_poses(baseline, moved)["numerical_pass"]
    turned = baseline.copy()
    turned[:3, :3] = Rotation.from_rotvec([0, 0, 2e-4]).as_matrix()
    assert not compare_poses(baseline, turned)["numerical_pass"]
    with pytest.raises(TFValidationError):
        compare_poses(baseline, np.zeros((4, 4)))


def test_tf_quaternion_validation_and_conversion():
    message = SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=0.1, y=0.2, z=0.3),
        rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))
    np.testing.assert_allclose(matrix_from_tf(message)[:3, 3], [0.1, 0.2, 0.3])
    message.transform.rotation.w = 0.0
    with pytest.raises(TFValidationError):
        matrix_from_tf(message)


def test_invalid_joint_input_refused_before_ros_start(monkeypatch):
    xml = '<robot name="unit"><link name="base"/><link name="tip"/>' \
          '<joint name="j" type="revolute"><parent link="base"/><child link="tip"/>' \
          '<limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>'
    model = model_from_xml(xml, base_frame="base", ee_frame="tip", joint_order=["j"])
    with pytest.raises(ModelError):
        run_case(model, None, ("invalid", [2.0]), {}, domain_id=81, timeout=1, namespace="/unit")


def test_missing_target_frame_refused_before_ros_start():
    model = SimpleNamespace(joint_positions=lambda q: {}, base_frame="base", ee_frame="missing",
                            chain=[SimpleNamespace(parent="base", child="tip")])
    with pytest.raises(TFValidationError, match="Missing Base/EE"):
        run_case(model, None, ("invalid", []), {}, domain_id=81, timeout=1, namespace="/unit")


def test_rsp_launch_and_early_exit_errors(tmp_path, monkeypatch):
    def missing(*args, **kwargs):
        raise FileNotFoundError("missing binary")
    monkeypatch.setattr(subprocess, "Popen", missing)
    child = ManagedRSP(["missing"], {}, tmp_path / "rsp.log")
    with pytest.raises(TFValidationError, match="launch failed"):
        child.start()
    assert child.close()["ok"]
    child.process = SimpleNamespace(poll=lambda: 5, returncode=5)
    with pytest.raises(TFValidationError, match="code=5"):
        child.check_alive()


@pytest.mark.parametrize("failure", [None, RuntimeError("case failure"), KeyboardInterrupt()])
def test_owned_process_cleanup_on_success_exception_or_interrupt(tmp_path, monkeypatch, failure):
    alive = [True]
    signals = []
    class FakeProcess:
        pid = 987654
        returncode = None
        def poll(self):
            return self.returncode
        def wait(self, timeout):
            self.returncode = 0
            alive[0] = False
            return 0
    process = FakeProcess()
    def spawn(command, **kwargs):
        assert kwargs["start_new_session"] is True
        return process
    def killpg(pid, sig):
        assert pid == process.pid  # never any pre-existing process/group
        if not alive[0]:
            raise ProcessLookupError()
        if sig:
            signals.append(sig)
    monkeypatch.setattr(subprocess, "Popen", spawn)
    monkeypatch.setattr("os.killpg", killpg)
    child = ManagedRSP(["rsp"], {}, tmp_path / "rsp.log")
    child.start()
    try:
        if failure:
            raise failure
    except BaseException:
        pass
    finally:
        cleanup = child.close()
    assert cleanup["ok"] and cleanup["reaped"] and cleanup["process_group_gone"]
    assert signals == [signal.SIGINT]
    assert not alive[0]


def test_rsp_cleanup_escalation(tmp_path, monkeypatch):
    alive = [True]
    sent = []
    class StubbornProcess:
        pid = 987655
        returncode = None
        def poll(self):
            return self.returncode
        def wait(self, timeout):
            if signal.SIGKILL not in sent:
                raise subprocess.TimeoutExpired("rsp", timeout)
            alive[0] = False
            self.returncode = -9
            return -9
    def killpg(pid, sig):
        assert pid == 987655
        if not alive[0]:
            raise ProcessLookupError()
        if sig:
            sent.append(sig)
    monkeypatch.setattr("os.killpg", killpg)
    child = ManagedRSP(["rsp"], {}, tmp_path / "rsp.log")
    child.process = StubbornProcess()
    assert child.close()["ok"]
    assert sent == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]


def test_other_resources_still_close_after_one_cleanup_failure():
    closed = []
    executor = SimpleNamespace(shutdown=lambda **kw: False)
    node = SimpleNamespace(destroy_node=lambda: closed.append("node"))
    context = SimpleNamespace(ok=lambda: True, shutdown=lambda: closed.append("context"))
    outcome = close_resources(None, executor, node, context)
    assert not outcome["ok"]
    assert closed == ["node", "context"]
    assert "executor" in outcome["errors"][0]


def test_report_cannot_pass_with_missing_failed_or_unclean_case():
    good = [{"name": name, "passed": True, "numerical_pass": True, "sync_complete": True,
             "status": "PASS", "cleanup": {"ok": True}}
            for name, _ in CASES]
    report = {"cases": good}
    assert finalize_report(report)
    assert report["stage_acceptance"] == "pending_user_review"
    assert report["hardware_checked"] is False
    for rows in [[], good[:2], [{**good[0], "passed": False}, *good[1:]],
                 [{**good[0], "numerical_pass": False}, *good[1:]],
                 [{**good[0], "sync_complete": False}, *good[1:]],
                 [{**good[0], "cleanup": {"ok": False}}, *good[1:]]]:
        report = {"cases": rows}
        assert not finalize_report(report)
        assert report["status"] == "FAIL"
    assert not finalize_report({"cases": good, "error": "startup/cleanup failure"})


# Hotfix tests keep all original tests above unchanged.
PRIVATE = "/private"
TOPIC = "/private/tf"
TYPE = "tf2_msgs/msg/TFMessage"
QOS = {"reliability": {"RELIABLE"}, "durability": {"VOLATILE"}}


def endpoint(name="rsp", namespace=PRIVATE, gid=(1, 2, 3), topic_type=TYPE,
             reliability="RELIABLE", durability="VOLATILE", role="PUBLISHER"):
    enum = lambda name: SimpleNamespace(name=name)
    return SimpleNamespace(
        node_name=name, node_namespace=namespace, endpoint_gid=gid, topic_type=topic_type,
        endpoint_type=enum(role), qos_profile=SimpleNamespace(
            reliability=enum(reliability), durability=enum(durability), history=enum("UNKNOWN"),
            depth=0, liveliness=enum("AUTOMATIC")))


def unknown_endpoint(**kwargs):
    return endpoint(name="_NODE_NAME_UNKNOWN_", namespace="_NODE_NAMESPACE_UNKNOWN_", **kwargs)


class FakeGraph:
    def __init__(self, observations, *, visible=True):
        self.observations, self.index, self.now = observations, 0, 0.0
        self.visible = visible
        self.per_node_type = TYPE
    def get_namespace(self):
        return PRIVATE
    def get_publishers_info_by_topic(self, topic):
        return self.observations[self.index]
    def get_subscriptions_info_by_topic(self, topic):
        return self.observations[self.index]
    def get_node_names_and_namespaces(self):
        return [("rsp", PRIVATE)] if self.visible else []
    def get_publisher_names_and_types_by_node(self, name, namespace):
        return [(TOPIC, [self.per_node_type])]
    get_subscriber_names_and_types_by_node = get_publisher_names_and_types_by_node
    def spin(self, dt):
        self.now += dt
        self.index = min(self.index + 1, len(self.observations) - 1)


def await_graph(graph, timeout=0.2):
    monitor = EndpointMonitor(graph, PRIVATE, timeout, clock=lambda: graph.now)
    wait_until("endpoint", lambda: monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS) is not None,
               graph.spin, lambda: None, timeout, clock=lambda: graph.now,
               detail=monitor.summary, on_timeout=monitor.raise_pending_timeout)
    return monitor


def test_unknown_then_correct_identity_continues_and_retains_history():
    monitor = await_graph(FakeGraph([[unknown_endpoint()], [endpoint()]]))
    record = monitor.diagnostics()["tf"]
    assert record["state"] == "VERIFIED"
    assert [t["state"] for t in record["transitions"]] == ["UNRESOLVED_DISCOVERY", "VERIFIED"]
    unresolved = record["transitions"][0]
    assert unresolved["endpoints"][0]["gid"] == "010203"
    assert unresolved["endpoints"][0]["qos"]["reliability"] == "RELIABLE"


def test_unknown_forever_has_explicit_code_and_graph_qos_wait_diagnostics():
    graph = FakeGraph([[unknown_endpoint()]])
    monitor = EndpointMonitor(graph, PRIVATE, 0.2, clock=lambda: graph.now)
    with pytest.raises(EndpointIdentityUnresolved, match="ENDPOINT_IDENTITY_UNRESOLVED") as failure:
        wait_until("endpoint", lambda: monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS),
                   graph.spin, lambda: None, 0.2, clock=lambda: graph.now,
                   on_timeout=monitor.raise_pending_timeout)
    assert failure.value.code == "ENDPOINT_IDENTITY_UNRESOLVED"
    record = monitor.diagnostics()["tf"]
    assert record["wait_elapsed_s"] >= 0.2
    assert record["graph_nodes"] == [("rsp", PRIVATE)]
    assert record["endpoints"][0]["node_name"] == "_NODE_NAME_UNKNOWN_"
    assert record["endpoints"][0]["qos"]["durability"] == "VOLATILE"


def test_unknown_then_known_wrong_identity_fails_without_waiting_to_timeout():
    graph = FakeGraph([[unknown_endpoint()], [endpoint(name="foreign")]])
    with pytest.raises(TFValidationError, match="unexpected endpoint"):
        await_graph(graph)
    assert graph.now < 0.2


@pytest.mark.parametrize("endpoints", [[unknown_endpoint(), endpoint()], [endpoint(), endpoint(name="foreign")]])
def test_multiple_publishers_never_pass_even_if_one_is_unknown(endpoints):
    graph = FakeGraph([endpoints])
    with pytest.raises(TFValidationError, match="expected one endpoint"):
        await_graph(graph)
    assert graph.now == 0.0


def test_absent_endpoint_then_unknown_then_verified():
    monitor = await_graph(FakeGraph([[], [unknown_endpoint()], [endpoint()]]))
    assert [v["state"] for v in monitor.records["tf"]["transitions"]] == [
        "NOT_DISCOVERED", "UNRESOLVED_DISCOVERY", "VERIFIED"]


def test_absent_endpoint_timeout_is_distinct_from_unknown_identity():
    with pytest.raises(TFTimeoutError, match="ENDPOINT_NOT_DISCOVERED"):
        await_graph(FakeGraph([[]]))


@pytest.mark.parametrize("value,pattern", [
    (endpoint(topic_type="wrong/msg/Type"), "topic type"),
    (endpoint(reliability="BEST_EFFORT"), "incompatible QoS"),
    (endpoint(durability="TRANSIENT_LOCAL"), "incompatible QoS"),
    (endpoint(role="SUBSCRIPTION"), "endpoint role"),
    (endpoint(namespace="/other"), "unexpected endpoint"),
])
def test_resolved_contradictory_type_qos_role_or_namespace_is_rejected(value, pattern):
    with pytest.raises(TFValidationError, match=pattern):
        await_graph(FakeGraph([[value]]))


def test_unresolved_matching_qos_is_not_accepted_until_resolved():
    monitor = await_graph(FakeGraph([[endpoint(reliability="UNKNOWN")], [endpoint()]]))
    assert monitor.records["tf"]["state"] == "VERIFIED"


def test_endpoint_gid_cannot_change_after_strict_verification():
    graph = FakeGraph([[endpoint()], [endpoint(gid=(8, 9, 10))]])
    monitor = EndpointMonitor(graph, PRIVATE, 1, clock=lambda: graph.now)
    assert monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS)
    graph.spin(0.05)
    with pytest.raises(TFValidationError, match="GID changed"):
        monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS)


def test_node_graph_and_endpoint_graph_must_agree():
    graph = FakeGraph([[endpoint()]], visible=False)
    monitor = EndpointMonitor(graph, PRIVATE, 1, clock=lambda: graph.now)
    assert monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS) is None
    graph.visible = True
    graph.per_node_type = "wrong/msg/Type"
    with pytest.raises(TFValidationError, match="per-node topic graph"):
        monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS)


def test_startup_waits_for_node_then_parameter_barrier_before_any_endpoint_check(monkeypatch):
    events = []
    class StartingNode:
        def get_namespace(self):
            return PRIVATE
        def get_node_names_and_namespaces(self):
            return [("rsp", PRIVATE)] if "node_ready" in events else []
    class Client:
        def service_is_ready(self):
            assert "node_ready" in events
            return "service_ready" in events
    def spin(dt):
        events.append("node_ready" if "node_ready" not in events else "service_ready")
    def read_parameters():
        assert "service_ready" in events
        events.append("parameters_loaded")
        return "digest"
    def no_early_strict_check(*args, **kwargs):
        raise AssertionError("strict endpoint check before model initialization")
    monkeypatch.setattr("workspace_analysis.experiments.tf_validation.require_single_endpoint", no_early_strict_check)
    phases = []
    result = wait_for_rsp_initialization(StartingNode(), Client(), spin, lambda: None, 1,
                                         read_parameters, phases.append)
    assert result == "digest"
    assert phases == ["ROS_NODE_DISCOVERED", "MODEL_PARAMETERS_VERIFIED"]
    assert events == ["node_ready", "service_ready", "parameters_loaded"]


def test_runtime_unresolved_identity_blocks_cached_success():
    graph = FakeGraph([[endpoint()], [unknown_endpoint()], [endpoint()]])
    monitor = EndpointMonitor(graph, PRIVATE, 1, clock=lambda: graph.now)
    assert monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS)
    graph.spin(0.05)
    accepted_at = []
    wait_until("cached pose", lambda: accepted_at.append(graph.index) or True,
               graph.spin, lambda: monitor.check("tf", TOPIC, "rsp", "PUBLISHER", TYPE, QOS) is not None,
               1, clock=lambda: graph.now, on_timeout=monitor.raise_pending_timeout)
    assert accepted_at == [2]


def test_abnormal_rsp_exit_is_separate_from_successful_resource_release(tmp_path, monkeypatch):
    class Aborted:
        pid = 777777
        returncode = -6
        def wait(self, timeout):
            return self.returncode
        def poll(self):
            return self.returncode
    def gone(pid, sig):
        assert pid == 777777
        raise ProcessLookupError()
    monkeypatch.setattr("os.killpg", gone)
    child = ManagedRSP(["rsp"], {}, tmp_path / "rsp.log")
    child.process = Aborted()
    cleanup = child.close()
    assert cleanup["ok"] and cleanup["cleanup_resources_released"]
    assert cleanup["rsp_exit_code"] == -6
    assert cleanup["rsp_exit_normal"] is False
    rows = [{"name": name, "status": "PASS", "passed": True, "numerical_pass": True,
             "sync_complete": True, "cleanup": {"ok": True}} for name, _ in CASES]
    rows[-1]["cleanup"] = cleanup
    assert not finalize_report({"cases": rows})


def test_shutdown_readiness_barrier_precedes_child_signal():
    events = []
    child = SimpleNamespace(
        prepare_stop=lambda: events.append("initialization_barrier"),
        close=lambda: events.append("child_signal_and_reap") or
        {"ok": True, "errors": [], "rsp_exit_normal": True, "rsp_exit_code": 0})
    outcome = close_resources(child, None, None, None)
    assert events == ["initialization_barrier", "child_signal_and_reap"]
    assert outcome["rsp_exit_normal"] and outcome["cleanup_resources_released"]


def test_shutdown_barrier_timeout_is_recorded_and_owned_child_still_released():
    events = []
    def not_ready():
        raise TFTimeoutError("constructor not ready")
    child = SimpleNamespace(prepare_stop=not_ready, close=lambda: events.append("stopped") or
                            {"ok": True, "errors": [], "rsp_exit_normal": False, "rsp_exit_code": -6})
    outcome = close_resources(child, None, None, None)
    assert events == ["stopped"] and outcome["cleanup_resources_released"]
    assert outcome["rsp_exit_normal"] is False
    assert "constructor not ready" in outcome["shutdown_readiness_error"]


def test_v3_report_requires_explicit_strict_identity_and_normal_exit_evidence():
    rows = [{"name": name, "passed": True, "numerical_pass": True, "sync_complete": True,
             "status": "PASS", "strict_endpoint_identity_verified": True,
             "isolated_topic_source_checked": True,
             "cleanup": {"ok": True, "cleanup_resources_released": True,
                         "rsp_exit_normal": True, "rsp_exit_code": 0}} for name, _ in CASES]
    assert finalize_report({"report_version": 3, "cases": rows})
    isolated_only = [{**rows[0], "strict_endpoint_identity_verified": False}, *rows[1:]]
    assert not finalize_report({"report_version": 3, "cases": isolated_only})
    missing_exit = [{**rows[0], "cleanup": {"ok": True}}, *rows[1:]]
    assert not finalize_report({"report_version": 3, "cases": missing_exit})
