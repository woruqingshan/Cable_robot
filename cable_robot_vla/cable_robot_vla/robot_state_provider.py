"""Passive state quality. Never constructs ArmInterface or an actuator client."""
from time import monotonic

from .hardware_feedback import RawFeedbackAdapter, JointFeedbackConversion, samples_from_diagnostic_report

SDK_BLOCK = "SDK_UNBOUNDED_QUEUE: bus_servo_read_and_unpack uses queue.get(block=True) under servo_read_lock; client timeout cannot release server"
IDS = (1, 2, 3, 4, 5, 10)
NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "r_joint")


class RobotStateProvider:
    def __init__(self, clock=monotonic):
        self.clock = clock
        self.raw_feedback = RawFeedbackAdapter(clock_ns=lambda: int(self.clock()*1e9))
        self.conversion = None
        self.cache = {}
        self.stamp_ns = None
        self.received = None
        self.error = "WAITING_FOR_CONTROLLER_DIAGNOSTICS"

    def offer_controller_cache(self, message):
        values = {}
        for state in message.servo_state:
            if state.id in values or not 0 <= state.position <= 1000:
                self.cache = {}
                self.error = "INVALID_CONTROLLER_CACHE"
                raise ValueError(self.error)
            values[int(state.id)] = int(state.position)
        self.cache = {key: values[key] for key in IDS if key in values}
        self.stamp_ns = message.header.stamp.sec*10**9 + message.header.stamp.nanosec
        self.received = self.clock()
        self.error = "CONTROLLER_CACHE_NOT_MEASURED"

    def _read(self, ids):
        age = None if self.received is None else self.clock()-self.received
        return {"joint_names": [NAMES[IDS.index(i)] for i in ids],
            "servo_ids": list(ids), "raw_values": [self.cache.get(i) for i in ids],
            "units": "pulse", "source_type": "controller_cached" if self.cache else "unknown",
            "timestamp_ns": self.stamp_ns, "timestamp_semantics": "controller_publication",
            "sample_interval_ns": None, "receive_age_s": age, "read_latency_s": None,
            "validity": "diagnostic_only", "canonical_observation_eligible": False,
            "missing_ids": [i for i in ids if i not in self.cache], "error_reason": self.error}

    def read_arm_state(self):
        return self._read(IDS[:5])

    def read_gripper_state(self):
        return self._read((10,))

    def read_state_snapshot(self):
        return self._read(IDS)

    def offer_raw_readback_snapshot(self, samples, *, sequence):
        """Explicit diagnostic ingestion; never replaces cache with claimed measurements."""
        self.raw_feedback.offer(samples, sequence=sequence)

    def read_raw_feedback(self):
        return self.raw_feedback.snapshot()

    def configure_raw_conversion(self, configuration_path, *, fk=None, fk_unavailable_reason=None):
        self.conversion = JointFeedbackConversion.from_vendor_file(configuration_path, fk=fk, fk_unavailable_reason=fk_unavailable_reason)

    def offer_diagnostic_report(self, report):
        self.offer_raw_readback_snapshot(samples_from_diagnostic_report(report), sequence=report["sequence"])
        return self.read_converted_feedback()

    def read_converted_feedback(self):
        raw = self.read_raw_feedback()
        if self.conversion is None or not raw["raw_snapshot_valid"]:
            return {"validity":"unavailable", "canonical_observation_eligible":False,
                    "error":raw["error_reason"] or "CONVERSION_NOT_CONFIGURED"}
        try:
            return self.conversion.convert(raw)
        except Exception as error:
            return {"validity":"unavailable", "canonical_observation_eligible":False,
                    "error":str(error)}

    def get_source_status(self):
        configured = self.conversion is not None
        return {"hardware_readback": "DIAGNOSTIC_ONLY" if configured else "BLOCKED",
            "reason": "INNER_ASSOCIATION_AND_CALIBRATION_UNVERIFIED" if configured else SDK_BLOCK,
            "rpc_requests_sent": 0, "controller": self.read_state_snapshot(),
            "raw_feedback": self.read_raw_feedback(),
            "converted_diagnostics": self.read_converted_feedback(),
            "kinematics_pose": "not_measured_without_input_provenance"}
