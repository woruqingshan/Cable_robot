"""Raw readback diagnostics only; no serial access, ROS clients, or rad conversion."""
from copy import deepcopy
import struct
from time import monotonic_ns

SERVO_IDS = (1, 2, 3, 4, 5, 10)
JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "r_joint")
MAX_RECEIVE_AGE_NS = 500_000_000
SYNC_LIMIT_NS = 20_000_000


def _integer(value):
    return type(value) is int and value >= 0


class RawFeedbackAdapter:
    """Validate complete candidate batches without promoting them to measured state.

    Input is the existing SDK candidate's PositionSample interface. Payload checks
    are record consistency checks, not a second serial parser or CRC verifier.
    Local sequences and monotonicity reject record replay; they do NOT establish
    wire-level transaction identity or make the SDK safe for repeated reads.
    """

    def __init__(self, clock_ns=monotonic_ns):
        self.clock_ns = clock_ns
        self._channels = None
        self._sequence = -1
        self._last_response = {}
        self.error = "WAITING_FOR_RAW_READBACK"

    def reject(self, reason):
        self._channels = None
        self.error = str(reason)

    def offer(self, samples, *, sequence):
        self._channels = None
        try:
            if not _integer(sequence) or sequence <= self._sequence:
                raise ValueError("RAW_SEQUENCE_REUSED")
            # Consume the local batch identifier even when the batch is rejected.
            self._sequence = sequence
            if not isinstance(samples, (tuple, list)) or len(samples) != 6:
                raise ValueError("INCOMPLETE_RAW_SNAPSHOT")
            now = self.clock_ns()
            if not _integer(now):
                raise ValueError("INVALID_HOST_CLOCK")
            channels = {}
            for sample in samples:
                servo_id = sample.servo_id
                pulse = sample.raw_pulse
                started = sample.read_started_at_ns
                received = sample.response_received_at_ns
                if type(servo_id) is not int or servo_id not in SERVO_IDS or servo_id in channels:
                    raise ValueError("INVALID_OR_DUPLICATE_SERVO_ID")
                if type(pulse) is not int or not 0 <= pulse <= 1000:
                    raise ValueError("INVALID_RAW_PULSE")
                if (not _integer(started) or not _integer(received)
                        or not started <= received <= now):
                    raise ValueError("INVALID_HOST_TIME_ORDER")
                if now - started > MAX_RECEIVE_AGE_NS:
                    raise ValueError("STALE_HOST_TRANSACTION")
                if started <= self._last_response.get(servo_id, -1):
                    raise ValueError("RAW_RECORD_REPLAY")
                if (sample.clock_domain != "host_monotonic"
                        or sample.device_sample_time_ns is not None):
                    raise ValueError("UNREVIEWED_SAMPLE_TIME")
                if sample.source_type != "unknown":
                    raise ValueError("SOURCE_PROMOTION_REQUIRES_REVIEW")
                payload = sample.raw_payload
                if not isinstance(payload, (bytes, bytearray)) or len(payload) != 5:
                    raise ValueError("INVALID_REPLY_RECORD")
                reply_id, command, status, reply_pulse = struct.unpack("<BBbh", payload)
                if (reply_id, command, status, reply_pulse) != (servo_id, 5, 0, pulse):
                    raise ValueError("REPLY_RECORD_MISMATCH")
                channels[servo_id] = {
                    "servo_id": servo_id, "joint_name": JOINT_NAMES[SERVO_IDS.index(servo_id)],
                    "raw_pulse": pulse, "units": "pulse", "source_type": "unknown",
                    "request_command": 5, "response_id": reply_id,
                    "response_command": command, "response_status": status,
                    "read_started_at_ns": started, "response_received_at_ns": received,
                    "host_read_latency_ms": (received-started)/1e6,
                    "clock_domain": "host_monotonic",
                    "timestamp_semantics": "host_transaction_start_and_parser_callback",
                    "tx_start_monotonic_ns": None, "device_sample_time_ns": None,
                    "sample_interval_ns": None, "ros_publication_time_ns": None,
                    "raw_reply_payload_hex": bytes(payload).hex(),
                    "raw_frame_evidence": None,
                }
            if set(channels) != set(SERVO_IDS):
                raise ValueError("INCOMPLETE_RAW_SNAPSHOT")
            ordered = sorted(channels.values(), key=lambda channel: channel["read_started_at_ns"])
            if any(a["response_received_at_ns"] > b["read_started_at_ns"]
                   for a, b in zip(ordered, ordered[1:])):
                raise ValueError("OVERLAPPING_BUS_TRANSACTIONS")
            self._channels = [channels[i] for i in SERVO_IDS]
            self._last_response = {i: channels[i]["response_received_at_ns"] for i in SERVO_IDS}
            self.error = None
        except Exception as error:
            self.reject(str(error))
            raise ValueError(self.error) from error

    def snapshot(self):
        channels = deepcopy(self._channels)
        if channels:
            now = self.clock_ns()
            if (not _integer(now) or any(now < c["response_received_at_ns"]
                    or now-c["read_started_at_ns"] > MAX_RECEIVE_AGE_NS for c in channels)):
                self.reject("STALE_OR_INVALID_HOST_CLOCK")
                channels = None
        span = None if not channels else max(c["response_received_at_ns"] for c in channels)-min(
            c["read_started_at_ns"] for c in channels)
        return {
            "raw_snapshot_valid": channels is not None,
            "sequence": self._sequence, "channels": channels or [],
            "source_type": "unknown", "units": "pulse",
            "host_transaction_span_ns": span,
            "host_read_span_exceeds_20ms": None if span is None else span > SYNC_LIMIT_NS,
            "device_sample_span_ns": None, "image_state_sync_skew_ns": None,
            "validity": "diagnostic_only", "canonical_observation_eligible": False,
            "error_reason": self.error,
            "canonical_blockers": ["HARDWARE_SOURCE_NOT_VERIFIED", "PULSE_TO_RAD_NOT_VERIFIED",
                                   "DEVICE_ACQUISITION_TIME_UNKNOWN"],
        }
