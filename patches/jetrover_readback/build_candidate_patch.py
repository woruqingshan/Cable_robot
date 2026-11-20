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
    sdk = replace_method(sdk, "packet_report_serial_servo", """
def packet_report_serial_servo(self, data):
    self._bounded_bus_feedback.receive(data, crc_checked=True)
""")
    sdk = replace_method(sdk, "bus_servo_read_and_unpack", """
def bus_servo_read_and_unpack(self, servo_id, cmd, unpack):
    self.last_bus_position_sample = None
    sample = self._bounded_bus_feedback.read_position(
        servo_id, cmd, unpack,
        lambda: self.buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id]),
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
        if len(result.state) != len(ids) or any(len(item.position) != 1 for item in result.state):
            result.state = []
            result.success = False
        return result

"""
    node = node.replace(anchor, wrapper+anchor, 1)
    ast.parse(sdk)
    ast.parse(node)
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
        "review_required":["clean/exclusive epoch evidence","actual response echo contract",
            "finite serial write bound","separate live-test authorization"]}
    (HERE/"baseline.json").write_text(json.dumps(metadata,indent=2)+"\n")
    print(json.dumps({"patch":str(path),"patch_sha256":metadata["patch_sha256"],"vendor_modified":False}))


if __name__ == "__main__":
    main()
