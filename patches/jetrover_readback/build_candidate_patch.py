"""Build a review-only patch from exact audited vendor hashes. Never installs."""
import argparse
import ast
import difflib
from hashlib import sha256
import json
from pathlib import Path

SDK_SHA = "c4dd48f3176a5dce1e8331846691ce995c0b3c2f79341cfc6c9e89dce1418a89"
NODE_SHA = "4d37d84c6eff0629f5f73ddca2b140cc0af555175fe3f68f8a3359123a716627"
HERE = Path(__file__).resolve().parent


def replace_method(source, name, replacement):
    methods = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.FunctionDef) and n.name == name]
    if len(methods) != 1:
        raise ValueError("expected exactly one method: "+name)
    node = methods[0]
    lines = source.splitlines(keepends=True)
    body = "".join(" "*node.col_offset+line+"\n" if line else "\n" for line in replacement.strip("\n").splitlines())
    lines[node.lineno-1:node.end_lineno] = [body]
    return "".join(lines)


def candidate_sources(sdk, node):
    if sha256(sdk.encode()).hexdigest() != SDK_SHA or sha256(node.encode()).hexdigest() != NODE_SHA:
        raise ValueError("vendor source differs from reviewed baseline; do not apply")
    sdk = sdk.replace("import threading", "import threading\nfrom .guarded_readback import ReadbackGate", 1)
    sdk = sdk.replace("        self.pwm_servo_read_lock = threading.Lock()",
        "        self.pwm_servo_read_lock = threading.Lock()\n"
        "        self._bounded_bus_feedback = ReadbackGate(self.servo_read_lock)\n"
        "        self.last_bus_position_sample = None", 1)
    sdk = sdk.replace("threading.Thread(target=self.recv_task,", "threading.Thread(target=self._recv_task_guarded,", 1)
    sdk = replace_method(sdk, "buf_write", """
def buf_write(self, func, data, *, require_complete=False):
    buf = [0xAA, 0x55, int(func)]
    buf.append(len(data))
    buf.extend(data)
    buf.append(checksum_crc8(bytes(buf[2:])))
    written = self.port.write(buf)
    if require_complete and (type(written) is not int or written != len(buf)):
        raise RuntimeError("READBACK_TX_INCOMPLETE")
""")
    sdk = replace_method(sdk, "packet_report_serial_servo", """
def packet_report_serial_servo(self, data):
    self._bounded_bus_feedback.receive(data, crc_checked=True)
""")
    sdk = replace_method(sdk, "bus_servo_read_and_unpack", """
def bus_servo_read_and_unpack(self, servo_id, cmd, unpack):
    self.last_bus_position_sample = None
    sample = self._bounded_bus_feedback.read_position(
        servo_id, cmd, unpack,
        lambda: self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id], require_complete=True),
        timeout_s=0.1, write_timeout_s=self.port.write_timeout,
        reception_enabled=self.enable_recv)
    self.last_bus_position_sample = sample
    return [sample.raw_pulse]
""")
    sdk = sdk.replace("    def recv_task(self):", """    def _recv_task_guarded(self):
        try:
            self.recv_task()
        except Exception:
            self._bounded_bus_feedback.fail("RECEIVER_FAILED")
        else:
            self._bounded_bus_feedback.fail("RECEIVER_STOPPED")

    def recv_task(self):""", 1)
    node = node.replace("from ros_robot_controller.ros_robot_controller_sdk import Board",
        "from ros_robot_controller.ros_robot_controller_sdk import Board\n"
        "from ros_robot_controller.guarded_readback import ReadbackFault", 1)
    node = node.replace("    def get_bus_servo_state(self, request, response):",
        "    def _get_bus_servo_state_legacy(self, request, response):", 1)
    anchor = "    def _get_bus_servo_state_legacy(self, request, response):"
    wrapper = """    def get_bus_servo_state(self, request, response):
        response.state = []
        response.success = False
        ids = [item.id for item in request.cmd]
        flags = ("get_id", "get_offset", "get_voltage", "get_temperature",
                 "get_position_limit", "get_voltage_limit",
                 "get_max_temperature_limit", "get_torque_state")
        if not 1 <= len(ids) <= 6 or len(set(ids)) != len(ids):
            return response
        if any(i not in (1, 2, 3, 4, 5, 10) for i in ids):
            return response
        if any(item.get_position != 1 or any(getattr(item, f) for f in flags)
               for item in request.cmd):
            return response
        try:
            result = self._get_bus_servo_state_legacy(request, response)
        except ReadbackFault as error:
            response.state = []
            response.success = False
            self.get_logger().error("Readback refused: " + error.code)
            return response
        except Exception:
            self.board._bounded_bus_feedback.fail("SERVICE_READ_FAILED")
            response.state = []
            response.success = False
            self.get_logger().error("Readback refused: SERVICE_READ_FAILED")
            return response
        if not result.success or len(result.state) != len(ids) or any(len(item.position) != 1 for item in result.state):
            result.state = []
            result.success = False
        return result

"""
    node = node.replace(anchor, wrapper+anchor, 1)
    sdk, node = maintenance_sources(sdk, node)
    sdk, node = probe_sources(sdk, node)
    ast.parse(sdk)
    ast.parse(node)
    return sdk, node



def maintenance_sources(sdk, node):
    """Opt-in transport sandbox; never arms reads or installs runtime files."""
    sdk = sdk.replace(
        "from .guarded_readback import ReadbackGate",
        "from .guarded_readback import (ReadbackGate, MaintenanceFrameWriter,\n"
        "    claim_linux_exclusive, ReadbackFault)\nimport math", 1)
    sdk = sdk.replace(
        'def __init__(self, device="/dev/rrc", baudrate=1000000, timeout=5):',
        'def __init__(self, device="/dev/rrc", baudrate=1000000, timeout=5, *,\n'
        '                 maintenance_tx_timeout_s=None):', 1)
    sdk = sdk.replace("        self.enable_recv = False",
        "        self._maintenance_writer = None\n"
        "        options = {}\n"
        "        if maintenance_tx_timeout_s is not None:\n"
        "            if (type(maintenance_tx_timeout_s) not in (int, float)\n"
        "                    or not math.isfinite(maintenance_tx_timeout_s)\n"
        "                    or not 0 < maintenance_tx_timeout_s <= .05):\n"
        "                raise ValueError('invalid maintenance TX timeout')\n"
        "            options['write_timeout'] = maintenance_tx_timeout_s\n"
        "        self.enable_recv = False", 1)
    sdk = sdk.replace("serial.Serial(None, baudrate, timeout=timeout)",
        "serial.Serial(None, baudrate, timeout=timeout, **options)", 1)
    sdk = sdk.replace("        self.port.open()",
        "        self.port.open()\n"
        "        if maintenance_tx_timeout_s is not None:\n"
        "            try:\n"
        "                claim_linux_exclusive(self.port)\n"
        "            except Exception:\n"
        "                self.port.close()\n"
        "                raise\n"
        "            self._maintenance_writer = MaintenanceFrameWriter(\n"
        "                self.port, checksum_crc8)", 1)
    sdk = sdk.replace("def buf_write(self, func, data, *, require_complete=False):",
        "def buf_write(self, func, data, *, require_complete=False, deadline_ns=None):", 1)
    sdk = replace_method(sdk, "buf_write", """
def buf_write(self, func, data, *, require_complete=False, deadline_ns=None):
    buf = [0xAA, 0x55, int(func)]
    buf.append(len(data))
    buf.extend(data)
    buf.append(checksum_crc8(bytes(buf[2:])))
    writer = getattr(self, '_maintenance_writer', None)
    if writer is None:
        written = self.port.write(buf)
    else:
        try:
            written = writer.write(buf, deadline_ns=deadline_ns)
        except ReadbackFault as error:
            self._bounded_bus_feedback.fail(error.code)
            raise
    if require_complete and (type(written) is not int or written != len(buf)):
        raise RuntimeError("READBACK_TX_INCOMPLETE")
""")
    sdk = sdk.replace(
        "lambda: self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id], require_complete=True),",
        "lambda deadline: self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,\n"
        "            [cmd, servo_id], require_complete=True, deadline_ns=deadline),", 1)
    sdk = sdk.replace("reception_enabled=self.enable_recv)",
        "reception_enabled=self.enable_recv, send_with_deadline=True)", 1)
    node = node.replace("        self.board = Board()",
        "        self.declare_parameter('readback_maintenance', False)\n"
        "        self.readback_maintenance = self.get_parameter('readback_maintenance').value\n"
        "        if type(self.readback_maintenance) is not bool:\n"
        "            raise ValueError('readback_maintenance must be bool')\n"
        "        self.board = (Board(maintenance_tx_timeout_s=.05)\n"
        "            if self.readback_maintenance else Board())", 1)
    # Preserve the buzzer only. Do not expose any actuator or PWM read callback.
    lines = node.splitlines(keepends=True)
    selected = []
    for line in lines:
        gated = (("self.create_subscription(" in line and "'~/set_buzzer'" not in line)
                 or "self.create_service(GetPWMServoState," in line
                 or "self.board.pwm_servo_set_offset(1, 0)" in line
                 or "self.board.set_motor_speed([[1, 0]" in line)
        if gated:
            selected.append(" "*(len(line)-len(line.lstrip()))+
                            "if not self.readback_maintenance:\n")
            selected.append("    "+line)
        else:
            selected.append(line)
    node = "".join(selected)
    node = node.replace(
        "        node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])",
        "        if not node.readback_maintenance:\n"
        "            node.board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])", 1)
    node = replace_method(node, "set_buzzer_state", """
def set_buzzer_state(self, msg):
    try:
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)
    except ReadbackFault as error:
        if not self.readback_maintenance:
            raise
        self.get_logger().error("Maintenance TX refused: " + error.code)
""")
    return sdk, node



def probe_sources(sdk, node):
    """Add one-shot unverified capture; never enable trusted/continuous reads."""
    sdk = sdk.replace(
        'def bus_servo_read_and_unpack(self, servo_id, cmd, unpack):',
        'def bus_servo_read_and_unpack(self, servo_id, cmd, unpack, *, diagnostic=False):', 1)
    sdk = sdk.replace(
        'reception_enabled=self.enable_recv, send_with_deadline=True)',
        'reception_enabled=self.enable_recv, send_with_deadline=True, diagnostic=diagnostic)', 1)
    # Capture actual validated RX bytes, including the received CRC, in the existing parser.
    anchor = '                                data = bytes(self.frame[2:])'
    sdk = sdk.replace(anchor,
        '                                if func == PacketFunction.PACKET_FUNC_BUS_SERVO:\n'
        '                                    self._last_bus_rx_frame = (b"\\xaa\\x55"\n'
        '                                        + bytes(self.frame) + bytes([dat]))\n'+anchor, 1)
    anchor = "        writer = getattr(self, '_maintenance_writer', None)"
    sdk = sdk.replace(anchor,
        '        if int(func) == 5 and len(data) == 2 and data[0] == 5:\n'
        '            self._last_bus_tx_frame = bytes(buf)\n'+anchor, 1)
    anchor = '    def bus_servo_read_and_unpack('
    method = '''    def probe_bus_servo_id1_once(self, reference):
        """Capture through the official position path; source stays unknown."""
        capture_started = time.monotonic_ns()
        armed_here = False
        report = dict(servo_id=1, raw_pulse=None, source_type="unknown",
            canonical_observation_eligible=False, inner_association_verified=False,
            device_sample_time_ns=None, outer_frame_validated=False,
            read_started_at_ns=None, response_received_at_ns=None, read_latency_ns=None,
            tx_frame_hex=None, rx_frame_hex=None, error=None)
        try:
            if getattr(self, '_maintenance_writer', None) is None:
                raise ReadbackFault("MAINTENANCE_REQUIRED")
            self._bounded_bus_feedback.arm_diagnostic_once(reference)
            armed_here = True
            self._last_bus_tx_frame = self._last_bus_rx_frame = None
            self.bus_servo_read_and_unpack(1, 0x05, "<BBbh", diagnostic=True)
            sample = self.last_bus_position_sample
            report.update(raw_pulse=sample.raw_pulse,
                read_started_at_ns=sample.read_started_at_ns,
                response_received_at_ns=sample.response_received_at_ns,
                read_latency_ns=sample.response_received_at_ns-sample.read_started_at_ns,
                outer_frame_validated=True)
        except ReadbackFault as error:
            report["error"] = error.code
        except Exception:
            self._bounded_bus_feedback.fail("UNEXPECTED_PROBE_FAILURE")
            report["error"] = "UNEXPECTED_PROBE_FAILURE"
        for key, attr in (("tx_frame_hex", "_last_bus_tx_frame"),
                          ("rx_frame_hex", "_last_bus_rx_frame")):
            value = getattr(self, attr, None) if armed_here else None
            report[key] = value.hex() if value is not None else None
        report["gate_status"] = self._bounded_bus_feedback.status()
        report["capture_started_at_ns"] = capture_started
        report["capture_finished_at_ns"] = time.monotonic_ns()
        report["capture_elapsed_ns"] = report["capture_finished_at_ns"]-capture_started
        report["host_report_system_time_ns"] = time.time_ns()
        return report

'''
    sdk = sdk.replace(anchor, method+anchor, 1)
    node = node.replace('import math', 'import math\nimport json', 1)
    anchor = '        self.board = (Board(maintenance_tx_timeout_s=.05)'
    node = node.replace(anchor,
        "        self.declare_parameter('readback_probe_reference', '')\n"
        "        self.readback_probe_reference = self.get_parameter('readback_probe_reference').value\n"
        "        ref = self.readback_probe_reference\n"
        "        if not isinstance(ref, str) or len(ref) > 512:\n"
        "            raise ValueError('invalid probe reference')\n"
        "        if ref and (not ref.strip() or not self.readback_maintenance):\n"
        "            raise ValueError('probe requires explicit maintenance mode')\n"+anchor, 1)
    anchor = "        self.create_service(Trigger, '~/init_finish', self.get_node_state)"
    node = node.replace(anchor,
        "        if self.readback_probe_reference:\n"
        "            self.create_service(Trigger, '~/bus_servo/probe_id1_once', self.probe_id1_once)\n"+anchor, 1)
    method = '''    def probe_id1_once(self, request, response):
        report = self.board.probe_bus_servo_id1_once(self.readback_probe_reference)
        response.success = report["outer_frame_validated"]
        response.message = json.dumps(report, sort_keys=True)
        return response

'''
    node = node.replace('    def get_node_state(self, request, response):',
                        method+'    def get_node_state(self, request, response):', 1)
    # Do not announce controller readiness to a waiting motion initializer.
    node = node.replace('        response.success = True\n        return response',
                        '        response.success = not self.readback_maintenance\n        return response', 1)
    return sdk, node



def build_patch(sdk, node):
    modified = candidate_sources(sdk, node)
    chunks = []
    for name, before, after in zip(
            ("ros_robot_controller_sdk.py", "ros_robot_controller_node.py"),
            (sdk, node), modified):
        chunks.extend(difflib.unified_diff(before.splitlines(keepends=True), after.splitlines(keepends=True),
            fromfile="a/ros_robot_controller/"+name, tofile="b/ros_robot_controller/"+name))
    helper = (HERE/"guarded_readback.py").read_text()
    chunks.extend(difflib.unified_diff([], helper.splitlines(keepends=True), fromfile="/dev/null",
        tofile="b/ros_robot_controller/guarded_readback.py"))
    return "".join(chunks)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vendor-directory", required=True)
    args = parser.parse_args()
    vendor = Path(args.vendor_directory).resolve()
    sdk = (vendor/"ros_robot_controller_sdk.py").read_text()
    node = (vendor/"ros_robot_controller_node.py").read_text()
    patch = build_patch(sdk, node)
    path = HERE/"sdk_bounded_readback_candidate.patch"
    if path.exists() and path.read_text() != patch:
        raise RuntimeError("refuse to overwrite a different candidate patch")
    path.write_text(patch)
    metadata = {"schema_version":1, "sdk_sha256":SDK_SHA, "node_sha256":NODE_SHA,
        "patch_sha256":sha256(patch.encode()).hexdigest(), "applied_to_vendor":False,
        "default_state":"DISARMED", "continuous_readback_supported":False,
        "diagnostic_policy":"ID1_ONCE_UNVERIFIED", "diagnostic_source_type":"unknown",
        "maintenance_mode_default":False, "maintenance_candidate_hardware_validated":False,
        "review_required":["clean/exclusive epoch evidence","actual response echo contract",
            "finite serial write bound","verified inner servo ID/command/transaction matching",
            "separate live-test authorization"]}
    (HERE/"baseline.json").write_text(json.dumps(metadata,indent=2)+"\n")
    print(json.dumps({"patch":str(path),"patch_sha256":metadata["patch_sha256"],"vendor_modified":False}))


if __name__ == "__main__":
    main()
