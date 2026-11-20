"""Hash-pinned legacy button runner. Inspect by default; no serial SDK fallback."""
import argparse
import ast
from collections import deque
from hashlib import sha256
import json
import math
from pathlib import Path
import struct
import threading
import time

BUTTON_SHA256 = "ea04002b6b5095c4224706639ad5da0f821fdf3c22ea7a4ecd018acbe9b7e4dc"
BUTTON_PATH = "/home/ubuntu/wifi_manager/button_scan.py"
BUZZER_TOPIC = "/ros_robot_controller/set_buzzer"


def duration_for_wire(seconds):
    """Preserve SDK integer milliseconds across ROS float32 serialization."""
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 0 <= seconds < 65.536:
        raise ValueError("INVALID_BUZZER_DURATION")
    milliseconds = int(seconds*1000)
    if milliseconds > 65535:
        raise ValueError("INVALID_BUZZER_DURATION")
    bits = struct.unpack("<I", struct.pack("<f", milliseconds/1000))[0]
    value = struct.unpack("<f", struct.pack("<I", bits))[0]
    if int(value*1000) < milliseconds:
        value = struct.unpack("<f", struct.pack("<I", bits+1))[0]
    if int(value*1000) != milliseconds:
        raise ValueError("UNREPRESENTABLE_BUZZER_DURATION")
    return value


class RosBuzzerSink:
    """The existing buzzer topic only; private context, no hardware SDK import."""
    def __init__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from ros_robot_controller_msgs.msg import BuzzerState
        self.rclpy, self.message_type = rclpy, BuzzerState
        self.context, self.node, self.executor = Context(), None, None
        try:
            rclpy.init(context=self.context)
            self.node = rclpy.create_node("wifi_button_buzzer_proxy", namespace="/idc_vla/real",
                context=self.context, start_parameter_services=False, enable_rosout=False)
            self.publisher = self.node.create_publisher(BuzzerState, BUZZER_TOPIC, 1)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
        except Exception:
            self.close()
            raise

    def poll(self):
        self.executor.spin_once(timeout_sec=.02)
        return self.publisher.get_subscription_count() > 0

    def publish(self, values):
        freq, on_time, off_time, repeat = values
        message = self.message_type()
        message.freq, message.on_time = freq, on_time
        message.off_time, message.repeat = off_time, repeat
        self.publisher.publish(message)

    def close(self):
        try:
            if self.executor is not None:
                self.executor.shutdown(timeout_sec=1.)
        finally:
            self.executor = None
            try:
                if self.node is not None:
                    self.node.destroy_node()
                    self.node = None
            finally:
                if self.context.ok():
                    self.context.shutdown()


class BuzzerRelay:
    """Bounded asynchronous beeps; unavailable ROS never falls back to serial."""
    def __init__(self, *, sink_factory=RosBuzzerSink, clock=time.monotonic, ttl_s=2.,
                 queue_size=4):
        if not 0 < ttl_s <= 5 or type(queue_size) is not int or not 1 <= queue_size <= 8:
            raise ValueError("INVALID_RELAY_LIMIT")
        self._factory, self._clock, self._ttl = sink_factory, clock, ttl_s
        self._queue, self._capacity = deque(), queue_size
        self._lock, self._stop = threading.Lock(), threading.Event()
        self._published, self._dropped, self._error = 0, 0, None
        self._closed = False
        self._thread = threading.Thread(target=self._worker, name="wifi-buzzer-relay", daemon=True)
        self._thread.start()

    def set_buzzer(self, freq, on_time, off_time, repeat=1):
        if (type(freq) is not int or not 0 <= freq <= 65535
                or type(repeat) is not int or not 0 <= repeat <= 65535):
            raise ValueError("INVALID_BUZZER_ARGUMENT")
        values = (freq, duration_for_wire(on_time), duration_for_wire(off_time), repeat)
        with self._lock:
            if self._closed or self._error or len(self._queue) >= self._capacity:
                self._dropped += 1
                return False
            self._queue.append((self._clock(), values))
            return True

    def _worker(self):
        sink = None
        try:
            sink = self._factory()
            while not self._stop.is_set():
                ready = sink.poll()
                event = None
                with self._lock:
                    if self._queue:
                        started, values = self._queue[0]
                        if self._clock()-started > self._ttl:
                            self._queue.popleft()
                            self._dropped += 1
                        elif ready and not self._closed:
                            event = self._queue.popleft()
                if event is not None and not self._stop.is_set():
                    sink.publish(event[1])
                    with self._lock:
                        self._published += 1
        except Exception as error:
            with self._lock:
                # Error class only: no environment, credentials or exception payload.
                self._error = "BUZZER_SINK_" + type(error).__name__
        finally:
            with self._lock:
                self._dropped += len(self._queue)
                self._queue.clear()
            if sink is not None:
                try:
                    sink.close()
                except Exception:
                    with self._lock:
                        self._error = self._error or "BUZZER_CLEANUP_FAILED"

    def status(self):
        with self._lock:
            return {"published": self._published, "dropped": self._dropped,
                    "queued": len(self._queue), "error": self._error,
                    "worker_alive": self._thread.is_alive(), "closed": self._closed,
                    "serial_fallback": False}

    def close(self):
        with self._lock:
            self._closed = True
        self._stop.set()
        self._thread.join(timeout=1.)
        if self._thread.is_alive():
            with self._lock:
                self._error = self._error or "BUZZER_SHUTDOWN_INCOMPLETE"


def prepare_vendor_tree(path):
    """Only the exact reviewed SDK import is removed; every other AST node remains."""
    raw = Path(path).read_bytes()
    if sha256(raw).hexdigest() != BUTTON_SHA256:
        raise ValueError("BUTTON_SOURCE_HASH_MISMATCH")
    tree = ast.parse(raw, filename=str(path))
    imports = [node for node in tree.body if isinstance(node, ast.ImportFrom)
               and node.module == "ros_robot_controller.ros_robot_controller_sdk"]
    if len(imports) != 1 or [(a.name, a.asname) for a in imports[0].names] != [("Board", None)]:
        raise ValueError("BUTTON_IMPORT_CONTRACT_CHANGED")
    tree.body.remove(imports[0])
    return tree


def inspect_vendor(path):
    tree = prepare_vendor_tree(path)
    return {"script": str(Path(path).resolve()), "source_sha256": BUTTON_SHA256,
            "transformed_ast_sha256": sha256(ast.dump(tree).encode()).hexdigest(),
            "replaced_interface": "Board.set_buzzer via existing ROS BuzzerState",
            "buzzer_topic": BUZZER_TOPIC, "vendor_modified": False,
            "run_enabled": False, "serial_device_opened": False}


def run_vendor(path, *, relay_factory=BuzzerRelay):
    """Deployment approval is required: legacy GPIO/Wi-Fi/halt behavior is preserved."""
    tree = prepare_vendor_tree(path)
    owned = []
    def board_factory():
        relay = relay_factory()
        owned.append(relay)
        return relay
    namespace = {"__name__": "__main__", "__file__": str(path), "Board": board_factory}
    try:
        exec(compile(tree, str(path), "exec"), namespace)
    finally:
        for relay in owned:
            relay.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--script", default=BUTTON_PATH)
    parser.add_argument("--run", action="store_true",
                        help="Requires separate approval; executes preserved GPIO/Wi-Fi/halt logic")
    args = parser.parse_args(argv)
    if args.run:
        run_vendor(args.script)
    else:
        print(json.dumps(inspect_vendor(args.script), indent=2))


if __name__ == "__main__":
    main()
