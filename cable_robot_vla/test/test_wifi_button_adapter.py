"""No GPIO device, ROS graph, serial, network reset, or halt is used in these tests."""
import ast
import builtins
from hashlib import sha256
import json
import os
from pathlib import Path
import struct
import sys
import threading
import time
from types import ModuleType, SimpleNamespace as NS

import pytest

from patches.jetrover_readback import wifi_button_adapter as proxy
from patches.jetrover_readback import prepare_button_stage as stage


def wait_for(predicate, timeout=1.):
    end = time.monotonic()+timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.002)
    assert predicate()


class FakeSink:
    def __init__(self, ready=True):
        self.ready, self.values, self.closed = ready, [], False
    def poll(self):
        time.sleep(.001)
        return self.ready
    def publish(self, values):
        self.values.append(values)
    def close(self):
        self.closed = True


@pytest.mark.parametrize("on_time,off_time", [
    (.2, .01), (.5, .01), (0., 0.), (.001, .009), (65.535, 1.2349), (.0001, .0005),
])
def test_float32_transport_preserves_original_wire_milliseconds(on_time, off_time):
    encoded = [proxy.duration_for_wire(t) for t in (on_time, off_time)]
    received = [struct.unpack("<f", struct.pack("<f", t))[0] for t in encoded]
    assert [int(t*1000) for t in received] == [int(on_time*1000), int(off_time*1000)]


@pytest.mark.parametrize("duration", [-1., float("nan"), float("inf"), 1e308, 65.536, True, "1"])
def test_invalid_buzzer_duration_rejected_without_io(duration):
    with pytest.raises(ValueError):
        proxy.duration_for_wire(duration)


def test_relay_publishes_once_and_cleans_only_owned_sink():
    sink = FakeSink()
    relay = proxy.BuzzerRelay(sink_factory=lambda: sink)
    try:
        assert relay.set_buzzer(1900, .2, .01, 1)
        wait_for(lambda: len(sink.values) == 1)
        assert sink.values[0][0] == 1900 and sink.values[0][3] == 1
        assert relay.status()["published"] == 1
        assert not relay.status()["serial_fallback"]
    finally:
        relay.close()
    assert sink.closed and not relay.status()["worker_alive"]
    assert relay.set_buzzer(1900, .2, .01) is False


def test_blocked_sink_never_blocks_button_callback_and_queue_is_bounded():
    entered, release = threading.Event(), threading.Event()
    class Blocked(FakeSink):
        def poll(self):
            entered.set()
            release.wait(.5)
            return False
    sink = Blocked()
    relay = proxy.BuzzerRelay(sink_factory=lambda: sink)
    try:
        assert entered.wait(.5)
        start = time.monotonic()
        accepted = [relay.set_buzzer(1900, .2, .01) for _ in range(5)]
        assert time.monotonic()-start < .2
        assert accepted == [True, True, True, True, False]
        assert relay.status()["queued"] == 4
    finally:
        release.set()
        relay.close()
    assert sink.values == [] and sink.closed


def test_unavailable_ros_drops_expired_beep_without_delayed_publish():
    now = [0.]
    sink = FakeSink(ready=False)
    relay = proxy.BuzzerRelay(sink_factory=lambda: sink, clock=lambda: now[0], ttl_s=.1)
    try:
        relay.set_buzzer(1900, .2, .01)
        now[0] = 1.
        wait_for(lambda: relay.status()["dropped"] == 1)
        sink.ready = True
        assert sink.values == []
    finally:
        relay.close()


def test_sink_failure_preserves_button_process_and_never_leaks_exception_payload():
    def fail():
        raise RuntimeError("offline-secret-sentinel")
    relay = proxy.BuzzerRelay(sink_factory=fail)
    try:
        wait_for(lambda: relay.status()["error"] is not None)
        assert relay.set_buzzer(1900, .2, .01) is False
        assert relay.status()["error"] == "BUZZER_SINK_RuntimeError"
        assert "offline-secret-sentinel" not in json.dumps(relay.status())
    finally:
        relay.close()


@pytest.fixture
def vendor_button():
    path = Path(proxy.BUTTON_PATH)
    if not path.is_file():
        pytest.skip("exact vendor button source required for preservation tests")
    assert sha256(path.read_bytes()).hexdigest() == proxy.BUTTON_SHA256
    return path


def test_inspection_cannot_import_gpio_ros_or_sdk(vendor_button, monkeypatch):
    original = builtins.__import__
    def forbidden(name, *args, **kwargs):
        if name.startswith(("Jetson", "rclpy", "ros_robot_controller")):
            raise AssertionError("hardware import attempted during inspection")
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, "__import__", forbidden)
    info = proxy.inspect_vendor(vendor_button)
    assert info["run_enabled"] is False and info["vendor_modified"] is False


def test_ast_changes_only_sdk_import_and_preserves_entire_button_logic(vendor_button):
    before = ast.parse(vendor_button.read_bytes())
    before.body = [node for node in before.body if not (
        isinstance(node, ast.ImportFrom)
        and node.module == "ros_robot_controller.ros_robot_controller_sdk")]
    assert ast.dump(proxy.prepare_vendor_tree(vendor_button)) == ast.dump(before)


class EndLegacyLoop(RuntimeError):
    pass


@pytest.mark.parametrize("mode,sleeps,expected_beep,expected_command_count", [
    ("idle", 1, None, 0), ("wifi", 3, (1900, .2, .01, 1), 2),
    ("halt", 31, (2500, .5, .01, 1), 1),
])
def test_preserved_legacy_paths_use_fake_gpio_and_intercept_all_system_commands(
        vendor_button, monkeypatch, mode, sleeps, expected_beep, expected_command_count):
    gpio = ModuleType("Jetson.GPIO")
    gpio.BCM, gpio.IN, gpio.LOW = 1, 2, 0
    gpio.setmode = lambda *args: None
    gpio.setup = lambda *args: None
    gpio.input = lambda pin: 0 if (mode == "wifi" and pin == 25 or mode == "halt" and pin == 4) else 1
    parent = ModuleType("Jetson");parent.__path__ = [];parent.GPIO = gpio
    monkeypatch.setitem(sys.modules, "Jetson", parent)
    monkeypatch.setitem(sys.modules, "Jetson.GPIO", gpio)
    commands, beeps, closed = [], [], []
    # No shell command is executed. No real GPIO module is imported.
    monkeypatch.setattr(os, "system", lambda command: commands.append(command))
    count = [0]
    def sleep(seconds):
        count[0] += 1
        if count[0] >= sleeps:
            raise EndLegacyLoop()
    monkeypatch.setattr(time, "sleep", sleep)
    relay = NS(set_buzzer=lambda *args: beeps.append(args), close=lambda: closed.append(True))
    with pytest.raises(EndLegacyLoop):
        proxy.run_vendor(vendor_button, relay_factory=lambda: relay)
    assert len(commands) == expected_command_count
    assert beeps == ([] if expected_beep is None else [expected_beep])
    if mode == "wifi":
        assert commands == [
            "sudo rm /etc/wifi/* -rf > /dev/null 2>&1",
            "sudo systemctl restart wifi.service > /dev/null 2>&1"]
    elif mode == "halt":
        assert commands == ["sudo halt"]
    assert closed == [True]
    assert sha256(vendor_button.read_bytes()).hexdigest() == proxy.BUTTON_SHA256


def test_modified_vendor_source_refuses_before_import_or_run(vendor_button, tmp_path):
    changed = tmp_path/"button.py"
    changed.write_bytes(vendor_button.read_bytes()+b"\n")
    with pytest.raises(ValueError, match="BUTTON_SOURCE_HASH_MISMATCH"):
        proxy.run_vendor(changed, relay_factory=lambda: pytest.fail("must not construct relay"))


@pytest.fixture
def backup_fixture(vendor_button, tmp_path):
    unit = Path(stage.UNIT_PATH)
    if not unit.exists():
        pytest.skip("exact vendor button unit required")
    assert sha256(unit.read_bytes()).hexdigest() == stage.UNIT_SHA256
    root = tmp_path/"backup";root.mkdir()
    records = []
    for source, filename in [(vendor_button, "button.py"), (unit, "button.service")]:
        p = root/filename;p.write_bytes(source.read_bytes())
        records.append({"source": str(source), "backup": filename,
                        "sha256": sha256(p.read_bytes()).hexdigest()})
    manifest = root/"manifest.json"
    manifest.write_text(json.dumps({"backup_root": str(root), "backup_verified": True, "files": records}))
    return manifest, sha256(manifest.read_bytes()).hexdigest(), unit


def test_review_package_never_changes_live_unit_script_or_installs_dropin(
        tmp_path, vendor_button, backup_fixture):
    manifest, digest, unit = backup_fixture
    output = tmp_path/"new-stage"
    result = stage.prepare_stage(output, backup_manifest=manifest, backup_sha256=digest, revision="a"*40)
    assert result["runtime_files_modified"] is False and result["services_restarted"] == []
    assert result["separate_approval_required"]
    text = (output/"button_scan.override.conf").read_text()
    assert "ExecStart=\n" in text and "--run --script" in text
    assert "start_app_node" not in text
    assert "source home/ubuntu/.zshrc;" in text
    assert sha256(unit.read_bytes()).hexdigest() == stage.UNIT_SHA256
    assert sha256(vendor_button.read_bytes()).hexdigest() == proxy.BUTTON_SHA256
    assert result["files"]["wifi_button_adapter.py"] == sha256(Path(proxy.__file__).read_bytes()).hexdigest()


def test_corrupt_backup_prevents_stage_creation(tmp_path, backup_fixture):
    manifest, digest, unit = backup_fixture
    (manifest.parent/"button.py").write_text("corrupted offline fixture")
    output = tmp_path/"new-stage"
    with pytest.raises(ValueError, match="BACKUP_COPY_HASH_MISMATCH"):
        stage.prepare_stage(output, backup_manifest=manifest, backup_sha256=digest, revision="a"*40)
    assert not output.exists()


@pytest.mark.parametrize("revision", ["main", "e0f151e", "", "g"*40])
def test_stage_requires_exact_commit_before_any_deployment_data(tmp_path, revision):
    with pytest.raises(ValueError, match="EXACT_COMMIT_REQUIRED"):
        stage.prepare_stage(tmp_path/"stage", backup_manifest="unused",
                            backup_sha256="unused", revision=revision)


@pytest.mark.parametrize("freq,on_time", [(1900,.2),(2500,.5)])
def test_relay_values_preserve_real_sdk_buzzer_packet_bytes(freq,on_time):
    source=Path("/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_sdk.py")
    if not source.exists():pytest.skip("exact SDK source required")
    raw=source.read_bytes()
    assert sha256(raw).hexdigest()=="c4dd48f3176a5dce1e8331846691ce995c0b3c2f79341cfc6c9e89dce1418a89"
    tree=ast.parse(raw)
    method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=="set_buzzer")
    namespace={"struct":struct,"PacketFunction":NS(PACKET_FUNC_BUZZER=2)}
    exec(compile(ast.Module(body=[method],type_ignores=[]),"offline_buzzer_method","exec"),namespace)
    packets=[];owner=NS(buf_write=lambda function,data:packets.append((function,data)))
    namespace["set_buzzer"](owner,freq,on_time,.01,1)
    on=struct.unpack("<f",struct.pack("<f",proxy.duration_for_wire(on_time)))[0]
    off=struct.unpack("<f",struct.pack("<f",proxy.duration_for_wire(.01)))[0]
    namespace["set_buzzer"](owner,freq,on,off,1)
    assert packets[0]==packets[1]


@pytest.mark.parametrize("fail_create", [False,True])
def test_ros_sink_owns_only_buzzer_publisher_and_private_context(monkeypatch,fail_create):
    calls=[]
    class Context:
        active=False
        def ok(self):return self.active
        def shutdown(self):self.active=False;calls.append("context_shutdown")
    context_module=ModuleType("rclpy.context");context_module.Context=Context
    class Executor:
        def __init__(self,context):assert context.ok()
        def add_node(self,node):pass
        def spin_once(self,timeout_sec):pass
        def shutdown(self,timeout_sec):calls.append("executor_shutdown")
    executor_module=ModuleType("rclpy.executors");executor_module.SingleThreadedExecutor=Executor
    publisher=NS(get_subscription_count=lambda:1,publish=lambda message:calls.append(message))
    node=NS(create_publisher=lambda msg,topic,depth:(calls.append((topic,depth)) or publisher),
        destroy_node=lambda:calls.append("destroy_node"))
    def create_node(*args,**kwargs):
        assert kwargs["namespace"]=="/idc_vla/real"
        assert kwargs["start_parameter_services"] is False
        if fail_create:raise RuntimeError("offline node construction failure")
        return node
    ros=ModuleType("rclpy");ros.__path__=[]
    ros.init=lambda context:setattr(context,"active",True)
    ros.create_node=create_node;ros.spin_once=lambda *args,**kwargs:None
    messages=ModuleType("ros_robot_controller_msgs.msg");messages.BuzzerState=NS
    parent=ModuleType("ros_robot_controller_msgs");parent.__path__=[];parent.msg=messages
    for name,module in [("rclpy",ros),("rclpy.context",context_module),("rclpy.executors",executor_module),
            ("ros_robot_controller_msgs",parent),("ros_robot_controller_msgs.msg",messages)]:
        monkeypatch.setitem(sys.modules,name,module)
    if fail_create:
        with pytest.raises(RuntimeError):proxy.RosBuzzerSink()
        assert calls==["context_shutdown"]
    else:
        sink=proxy.RosBuzzerSink();assert sink.poll()
        sink.publish((1900,.2,.01,1));sink.close()
        assert calls[0]==(proxy.BUZZER_TOPIC,1)
        assert calls[1].freq==1900 and calls[1].repeat==1
        assert calls[-2:]==["destroy_node","context_shutdown"]


@pytest.mark.skipif(os.environ.get("ROS_DOMAIN_ID")!="117",reason="real ROS lifecycle test requires isolated test domain117")
def test_real_ros_worker_context_lifecycle_without_publishing(monkeypatch):
    pytest.importorskip("rclpy")
    pytest.importorskip("ros_robot_controller_msgs.msg")
    created=threading.Event()
    def forbid_publish(*args):raise AssertionError("lifecycle test must never publish a command")
    monkeypatch.setattr(proxy.RosBuzzerSink,"publish",forbid_publish)
    def factory():
        sink=proxy.RosBuzzerSink();created.set();return sink
    relay=proxy.BuzzerRelay(sink_factory=factory)
    try:
        assert created.wait(3.),relay.status()
        assert relay.status()["published"]==0
    finally:relay.close()
    assert relay.status()["error"] is None
    assert not relay.status()["worker_alive"]
