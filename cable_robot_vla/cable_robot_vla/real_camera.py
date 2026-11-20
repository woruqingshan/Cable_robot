"""Camera evidence integrated with the real collector; depth stays outside VLA RGB."""
from collections import deque
import copy
from hashlib import sha256
import json
from time import monotonic
from types import SimpleNamespace

from vla_protocol.png import encode_rgb


def stamp_ns(message):
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1000000000 or not message.header.frame_id:
        raise ValueError("invalid source header")
    return stamp.sec*1000000000+stamp.nanosec


class StreamStats:
    def __init__(self, clock=monotonic):
        self.clock = clock
        self.count = self.duplicates = self.reversed = 0
        self.first = self.last = self.first_stamp = self.last_stamp = None
        self.max_gap_ns = 0
        self.gaps = deque(maxlen=128)
        self.large_gaps = 0

    def observe(self, stamp):
        now = self.clock()
        self.count += 1
        if self.first is None:
            self.first, self.first_stamp = now, stamp
        if self.last_stamp is not None:
            delta = stamp-self.last_stamp
            self.duplicates += delta == 0
            self.reversed += delta < 0
            self.max_gap_ns = max(self.max_gap_ns, delta)
            if delta > 0:
                if len(self.gaps) >= 5 and delta > 2.5*sorted(self.gaps)[len(self.gaps)//2]:
                    self.large_gaps += 1
                self.gaps.append(delta)
        self.last, self.last_stamp = now, stamp

    def status(self):
        duration = 0 if self.first is None else self.last-self.first
        return {"received": self.count, "receive_hz": (self.count-1)/duration if duration > 0 else None,
            "duplicate_stamps": self.duplicates, "backward_stamps": self.reversed,
            "max_source_gap_ns": self.max_gap_ns, "large_source_gaps": self.large_gaps,
            "exact_driver_drop_count": None,
            "last_receive_age_s": None if self.last is None else self.clock()-self.last}


class CameraRecordAdapter:
    def __init__(self, recorder, *, sample_hz=2.0, bridge=None, clock=monotonic):
        if not 0 < sample_hz <= 30:
            raise ValueError("sample_hz must be in (0,30]")
        if bridge is None:
            from cv_bridge import CvBridge
            bridge = CvBridge()
        self.bridge, self.recorder, self.clock = bridge, recorder, clock
        self.interval = 1/sample_hz
        self.stats = {kind: StreamStats(clock) for kind in ("rgb", "depth")}
        self.last_saved = {"rgb": None, "depth": None}
        self.sequence = {"rgb": 0, "depth": 0}
        self.info = {}
        self.provenance = None
        for name in ("camera", "depth"):
            (recorder.path/name).mkdir()
        self.samples_skipped = {"rgb": 0, "depth": 0}

    def _accept(self, kind, message):
        stamp = stamp_ns(message)
        self.stats[kind].observe(stamp)
        now = self.clock()
        last = self.last_saved[kind]
        if last is not None and now-last < self.interval:
            self.samples_skipped[kind] += 1
            return False
        if not 0 < message.width <= 1024 or not 0 < message.height <= 1024:
            raise ValueError("camera dimensions exceed protocol bounds")
        return True

    def _save(self, kind, message, raw, converted_encoding):
        self.sequence[kind] += 1
        folder = "camera" if kind == "rgb" else "depth"
        relative = f"{folder}/{kind}_{self.sequence[kind]:08d}.png"
        with (self.recorder.path/relative).open("xb") as stream:
            stream.write(raw)
        provenance = {"original_encoding": message.encoding, "original_step": message.step,
            "source_stamp_ns": stamp_ns(message), "frame_id": message.header.frame_id,
            "width": message.width, "height": message.height, "png_path": relative,
            "png_sha256": sha256(raw).hexdigest(), "converted_encoding": converted_encoding,
            "receive_monotonic_s": self.clock(), "valid_complete_observation": False}
        self.recorder.event("sensor_sample", sensor=kind, **provenance)
        self.last_saved[kind] = self.clock()
        return provenance

    def rgb(self, message):
        if not self._accept("rgb", message):
            return None
        if message.encoding not in ("rgb8", "bgr8") or message.is_bigendian != 0:
            raise ValueError("unsupported RGB encoding")
        # Reuse the user's verified CvBridge path, then explicitly transport RGB8.
        bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        if bgr.dtype.name != "uint8" or tuple(bgr.shape) != (message.height, message.width, 3):
            raise ValueError("CvBridge RGB dimensions/dtype mismatch")
        rgb = bgr[:, :, ::-1].copy().tobytes()
        self.provenance = self._save("rgb", message, encode_rgb(message.width, message.height, rgb), "rgb8")
        self.provenance["conversion"] = "cv_bridge_bgr8_then_rgb8"
        return SimpleNamespace(header=copy.deepcopy(message.header), encoding="rgb8", is_bigendian=0,
            width=message.width, height=message.height, step=message.width*3, data=rgb)

    def depth(self, message):
        if not self._accept("depth", message):
            return
        if message.encoding != "16UC1":
            raise ValueError("depth requires explicit 16UC1 encoding")
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        if image.dtype.name != "uint16" or tuple(image.shape) != (message.height, message.width):
            raise ValueError("CvBridge depth dimensions/dtype mismatch")
        import cv2
        success, png = cv2.imencode(".png", image)
        if not success:
            raise ValueError("depth PNG encoding failed")
        self._save("depth", message, png.tobytes(), "16UC1")

    def camera_info(self, kind, message):
        payload = {"source_stamp_ns": stamp_ns(message), "frame_id": message.header.frame_id,
            "width": message.width, "height": message.height, "distortion_model": message.distortion_model,
            "k": list(message.k), "d": list(message.d), "r": list(message.r), "p": list(message.p)}
        previous = self.info.get(kind)
        self.info[kind] = payload
        if previous is None or any(previous[k] != payload[k] for k in payload if k != "source_stamp_ns"):
            self.recorder.event("camera_info", sensor=kind, **payload)

    def status(self):
        return {"streams": {kind: stats.status() for kind, stats in self.stats.items()},
            "saved_samples": dict(self.sequence), "intentional_sampling_skips": dict(self.samples_skipped),
            "camera_info": self.info, "rgb_depth_synchronization": "not_asserted"}
