"""Offline intake tests. No ROS node, serial device, or hardware request exists."""
from dataclasses import replace
import json
from pathlib import Path
import struct
from types import SimpleNamespace as NS

import pytest

from cable_robot_vla.hardware_feedback import RawFeedbackAdapter
from cable_robot_vla.robot_state_provider import RobotStateProvider
from patches.jetrover_readback.guarded_readback import PositionSample
from patches.jetrover_readback.audit_environment import serial_openers


def samples(start=1_900_000_000, spacing=2_000_000):
    return [PositionSample(i, 400+i, start+k*spacing, start+k*spacing+1_000_000,
                           struct.pack("<BBbh", i, 5, 0, 400+i))
            for k, i in enumerate([1, 2, 3, 4, 5, 10])]


def adapter():
    return RawFeedbackAdapter(clock_ns=lambda: 2_000_000_000)


def test_complete_raw_batch_preserves_pulse_and_unknown_time_without_promotion():
    a = adapter()
    a.offer(samples(), sequence=1)
    state = a.snapshot()
    assert state["raw_snapshot_valid"] is True
    assert [c["joint_name"] for c in state["channels"]] == [
        "joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"]
    assert [c["raw_pulse"] for c in state["channels"]] == [401, 402, 403, 404, 405, 410]
    assert state["units"] == "pulse" and state["source_type"] == "unknown"
    assert state["canonical_observation_eligible"] is False
    assert state["host_transaction_span_ns"] == 11_000_000
    assert state["image_state_sync_skew_ns"] is None
    for c in state["channels"]:
        assert c["device_sample_time_ns"] is None
        assert c["sample_interval_ns"] is None
        assert c["tx_start_monotonic_ns"] is None
        assert c["raw_frame_evidence"] is None
        assert c["host_read_latency_ms"] == 1.


@pytest.mark.parametrize("change", [
    {"servo_id": True}, {"servo_id": 6}, {"servo_id": 10},
    {"raw_pulse": True}, {"raw_pulse": 1.5}, {"raw_pulse": 1001},
    {"clock_domain": "ros_system"}, {"device_sample_time_ns": 1},
    {"read_started_at_ns": -1}, {"read_started_at_ns": True},
    {"response_received_at_ns": 1_899_999_999},
    {"response_received_at_ns": 2_000_000_001},
    {"raw_payload": b"bad"},
    {"raw_payload": struct.pack("<BBbh", 2, 5, 0, 401)},
    {"raw_payload": struct.pack("<BBbh", 1, 7, 0, 401)},
    {"raw_payload": struct.pack("<BBbh", 1, 5, -1, 401)},
    {"source_type": "controller_cached"}, {"source_type": "hardware_measured"},
])
def test_invalid_records_clear_previous_complete_snapshot(change):
    a = adapter()
    a.offer(samples(), sequence=1)
    batch = samples(start=1_930_000_000)
    batch[0] = replace(batch[0], **change)
    with pytest.raises(ValueError):
        a.offer(batch, sequence=2)
    assert a.snapshot()["channels"] == []
    assert a.snapshot()["raw_snapshot_valid"] is False


@pytest.mark.parametrize("count", [0, 5, 7])
def test_missing_or_extra_actuators_never_form_snapshot(count):
    batch = samples()
    if count == 7:
        batch.append(batch[0])
    else:
        batch = batch[:count]
    a = adapter()
    with pytest.raises(ValueError, match="INCOMPLETE_RAW_SNAPSHOT"):
        a.offer(batch, sequence=1)
    assert a.snapshot()["canonical_observation_eligible"] is False


def test_sequence_and_per_channel_time_replay_rejected_but_fresh_raw_records_remain_diagnostic():
    a = adapter()
    a.offer(samples(), sequence=1)
    with pytest.raises(ValueError, match="RAW_SEQUENCE_REUSED"):
        a.offer(samples(), sequence=1)
    with pytest.raises(ValueError, match="RAW_RECORD_REPLAY"):
        a.offer(samples(), sequence=2)
    a.offer(samples(start=1_930_000_000), sequence=3)
    assert a.snapshot()["raw_snapshot_valid"]
    assert not a.snapshot()["canonical_observation_eligible"]


def test_expiration_does_not_fallback_to_prior_state_or_restamp_it():
    now = [2_000_000_000]
    a = RawFeedbackAdapter(clock_ns=lambda: now[0])
    a.offer(samples(), sequence=1)
    original = a.snapshot()
    now[0] += 500_000_001
    assert a.snapshot()["channels"] == []
    assert a.snapshot()["error_reason"] == "STALE_OR_INVALID_HOST_CLOCK"
    assert original["channels"][0]["read_started_at_ns"] == 1_900_000_000


def test_invalid_bus_overlap_and_long_span_are_not_disguised_as_single_sample_time():
    a = adapter()
    with pytest.raises(ValueError, match="OVERLAPPING_BUS_TRANSACTIONS"):
        a.offer(samples(spacing=500_000), sequence=1)
    a.offer(samples(spacing=5_000_000), sequence=2)
    state = a.snapshot()
    assert state["host_read_span_exceeds_20ms"] is True
    assert state["device_sample_span_ns"] is None
    assert state["image_state_sync_skew_ns"] is None
    assert not state["canonical_observation_eligible"]


def test_snapshot_returns_detached_data_and_explicit_failure_invalidates_it():
    a = adapter()
    a.offer(samples(), sequence=1)
    copy = a.snapshot()
    copy["channels"][0]["raw_pulse"] = 999
    assert a.snapshot()["channels"][0]["raw_pulse"] == 401
    a.reject("READBACK_FAULT")
    assert a.snapshot()["channels"] == []
    assert a.snapshot()["error_reason"] == "READBACK_FAULT"


def test_provider_keeps_controller_cache_separate_and_r2_blocked():
    p = RobotStateProvider(clock=lambda: 2.)
    message = NS(header=NS(stamp=NS(sec=1, nanosec=0)),
                 servo_state=[NS(id=i, position=500) for i in [1, 2, 3, 4, 5, 10]])
    p.offer_controller_cache(message)
    p.offer_raw_readback_snapshot(samples(), sequence=1)
    status = p.get_source_status()
    assert status["controller"]["source_type"] == "controller_cached"
    assert status["raw_feedback"]["source_type"] == "unknown"
    assert status["raw_feedback"]["raw_snapshot_valid"]
    assert status["hardware_readback"] == "BLOCKED" and status["rpc_requests_sent"] == 0
    assert p.read_state_snapshot()["canonical_observation_eligible"] is False


def process(proc, pid, target, *, secret="do-not-emit"):
    root = proc/str(pid)
    (root/"fd").mkdir(parents=True)
    (root/"fd"/"4").symlink_to(target)
    (root/"comm").write_text("python3\n")
    (root/"cmdline").write_bytes(b"python3\0/official/button_scan.py\0--token\0"+secret.encode()+b"\0")


def test_metadata_inventory_detects_multiple_openers_without_leaking_arguments(tmp_path):
    device = tmp_path/"fake_tty"
    device.write_bytes(b"")
    proc = tmp_path/"proc";proc.mkdir()
    process(proc, 1, device);process(proc, 2, device)
    result = serial_openers(device, proc)
    assert result["ownership_assessment"] == "MULTIPLE_OPENERS"
    assert result["safe_to_read"] is False and result["visibility_complete"] is False
    assert "do-not-emit" not in json.dumps(result)


def test_one_visible_opener_is_not_proof_of_exclusivity(tmp_path):
    device = tmp_path/"fake_tty";device.write_bytes(b"")
    proc = tmp_path/"proc";proc.mkdir();process(proc, 1, device)
    result = serial_openers(device, proc)
    assert result["ownership_assessment"] == "NOT_PROVEN_EXCLUSIVE"
    assert result["safe_to_read"] is False


def test_inaccessible_process_metadata_remains_unknown(tmp_path, monkeypatch):
    device = tmp_path/"fake_tty";device.write_bytes(b"")
    proc = tmp_path/"proc";proc.mkdir();process(proc, 1, device)
    original = Path.iterdir
    def denied(path):
        if path == proc/"1"/"fd":
            raise PermissionError("offline permission fixture")
        return original(path)
    monkeypatch.setattr(Path, "iterdir", denied)
    result = serial_openers(device, proc)
    assert result["inaccessible_process_fd_directories"] == 1
    assert result["safe_to_read"] is False
