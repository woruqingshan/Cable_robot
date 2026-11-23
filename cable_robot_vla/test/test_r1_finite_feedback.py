"""Finite diagnostics only: no physical serial device or robot service requests."""
import json
import time
from pathlib import Path
import pytest
from test_r1_vendor_patch import (
    audited_sources, patched_sdk, board_fixture, wire_response, StopFixture)


def plan(**updates):
    value=dict(schema_version=1, prior_used_ids=[1],
        prior_evidence_sha256="a"*64, d2_ids=[2,3,4,5,10],
        d3_cycles=3, cycle_interval_s=1.0, reference="offline-review",
        source_sha="a"*40)
    value.update(updates)
    return value


def session(patched_sdk, bad_id=None, duplicate=False):
    sdk,helper,_,_=patched_sdk
    board=board_fixture(sdk,helper,arm=False)
    board._maintenance_writer=helper.MaintenanceFrameWriter(board.port,sdk.checksum_crc8)
    def respond(raw):
        servo_id=raw[5]
        if servo_id==bad_id:return
        wire=wire_response(sdk,servo_id)
        board.port.incoming=list(wire*(2 if duplicate else 1))
        try:board.recv_task()
        except StopFixture:pass
    board.port.on_write=respond
    ticks=[time.monotonic_ns()]
    sleeps=[]
    def sleep(seconds):
        sleeps.append(seconds);ticks[0]+=int(seconds*1e9)
    callbacks=[]
    instance=helper.BoundedDiagnosticSession(board,plan(),clock_ns=lambda:ticks[0],
        sleep=sleep,on_cycle=callbacks.append)
    return instance,board,helper,callbacks,sleeps


def test_d2_only_queries_five_previously_unused_ids(patched_sdk):
    instance,board,helper,callbacks,_=session(patched_sdk)
    result=instance.run("D2")
    assert result["complete"] and result["phase"]=="D2_COMPLETE"
    assert [f[5] for f in board.port.frames]==[2,3,4,5,10]
    assert (1,5) in board._bounded_bus_feedback.used
    assert [r["raw_pulse"] for r in result["batches"][0]["channels"]]==[602,603,604,605,610]
    assert not result["canonical_observation_eligible"]
    assert not instance.run("D2")["complete"] and len(board.port.frames)==5


def test_d3_bounded_repeats_are_never_trusted_feedback(patched_sdk):
    instance,board,helper,callbacks,sleeps=session(patched_sdk)
    assert not instance.run("D3")["complete"] and board.port.frames==[]
    assert instance.run("D2")["complete"]
    result=instance.run("D3")
    assert result["complete"] and result["phase"]=="COMPLETE_UNVERIFIED"
    assert [f[5] for f in board.port.frames]==[2,3,4,5,10]+[1,2,3,4,5,10]*3
    assert len(result["batches"])==3 and len(callbacks)==4 and len(sleeps)==2
    assert result["association_guarantee"]=="UNPROVEN_REPEATED_RESPONSES"
    assert not result["canonical_observation_eligible"]
    assert not instance.run("D3")["complete"] and len(board.port.frames)==23
    with pytest.raises(helper.ReadbackFault) as error:
        board.bus_servo_read_and_unpack(1,5,"<BBbh",diagnostic=True)
    assert error.value.code=="DIAGNOSTIC_BUDGET_EXHAUSTED"
    assert len(board.port.frames)==23


def test_d2_failure_stops_partial_batch_and_d3(patched_sdk):
    instance,board,_,_,_=session(patched_sdk,bad_id=4)
    result=instance.run("D2")
    assert not result["complete"] and result["error"]=="READ_TIMEOUT"
    assert [f[5] for f in board.port.frames]==[2,3,4]
    assert result["batches"][0]["channels"][-1]["raw_pulse"] is None
    assert not instance.run("D3")["complete"]
    assert len(board.port.frames)==3


def test_duplicate_frame_fault_stops_finite_session(patched_sdk):
    instance,board,_,_,_=session(patched_sdk,duplicate=True)
    result=instance.run("D2")
    assert result["error"]=="DUPLICATE_REPLY" and len(board.port.frames)==1


@pytest.mark.parametrize("updates",[
    {"prior_used_ids":[]},{"d2_ids":[1,2,3,4,5,10]},
    {"d3_cycles":4},{"d3_cycles":True},{"cycle_interval_s":.01},
    {"cycle_interval_s":float("nan")},{"reference":""},
    {"prior_evidence_sha256":""}])
def test_unbounded_or_reordered_plan_rejected(updates):
    from patches.jetrover_readback.guarded_readback import BoundedDiagnosticSession
    with pytest.raises(ValueError):BoundedDiagnosticSession.validate_plan(plan(**updates))


def test_normal_trusted_key_reuse_protection_remains(patched_sdk):
    sdk,helper,_,_=patched_sdk
    board=board_fixture(sdk,helper)
    def respond(raw):
        board.port.incoming=list(wire_response(sdk))
        try:board.recv_task()
        except StopFixture:pass
    board.port.on_write=respond
    board.bus_servo_read_position(1)
    with pytest.raises(helper.ReadbackFault) as error:board.bus_servo_read_position(1)
    assert error.value.code=="KEY_REUSE_UNSAFE" and len(board.port.frames)==1


def test_staged_plan_requires_hashed_d1_history(tmp_path,audited_sources):
    from hashlib import sha256
    from patches.jetrover_readback.prepare_sdk_stage import prepare
    vendor=tmp_path/"vendor";vendor.mkdir()
    for name,text in zip(["ros_robot_controller_sdk.py","ros_robot_controller_node.py"],audited_sources):
        (vendor/name).write_text(text)
    old=tmp_path/"old";old.mkdir()
    capture=old/"d1_response.json"
    capture.write_text(json.dumps(dict(diagnostic_service_requests_sent=1,
        diagnostic=dict(gate_status={"used_keys":[[1,5]]},outer_frame_validated=True))))
    old_manifest=old/"evidence_manifest.json"
    old_manifest.write_text(json.dumps(dict(
        source_sha="b020afb16f8112b479c2104f9af7f52e676ec891",
        files={"d1_response.json":sha256(capture.read_bytes()).hexdigest()})))
    output=tmp_path/"new"
    result=prepare(output,vendor,"a"*40,"D2-D4-review",capture)
    assert result["finite_diagnostic_plan"]["prior_used_ids"]==[1]
    config=json.loads((output/"maintenance.json").read_text())
    params=config["ros_robot_controller"]["ros__parameters"]
    assert params["readback_probe_reference"]=="" and params["readback_maintenance"]
    assert Path(params["readback_diagnostic_plan_path"])==output/"diagnostic_plan.json"
    capture.write_text(capture.read_text()+"\n")
    with pytest.raises(ValueError):prepare(tmp_path/"rejected",vendor,"a"*40,"ref",capture)
    assert not (tmp_path/"rejected").exists()

