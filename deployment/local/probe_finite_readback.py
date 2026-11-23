"""Explicit finite R1 diagnostic RPCs. No actuator interface, retries or background loop."""
import argparse
import json
from pathlib import Path
import time


def validate_previous(stage, expected_sha, prior_path):
    if len(expected_sha) != 40 or any(c not in "0123456789abcdef" for c in expected_sha):
        raise ValueError("FULL_EXPECTED_SOURCE_SHA_REQUIRED")
    if stage == "D3":
        if prior_path is None:
            raise ValueError("SUCCESSFUL_D2_EVIDENCE_REQUIRED")
        previous = json.loads(Path(prior_path).read_text())
        if (previous.get("expected_source_sha") != expected_sha
                or previous.get("client_status") != "RESPONSE_RECEIVED"
                or previous.get("response_success") is not True
                or previous.get("diagnostic", {}).get("stage") != "D2"
                or previous["diagnostic"].get("complete") is not True):
            raise ValueError("D2_DID_NOT_PASS_ON_THIS_SOURCE")


def validate_response(report, expected_sha):
    if not report.get("complete"):
        return False
    if (report.get("canonical_observation_eligible") is not False
            or report.get("source_type") != "unknown"):
        raise ValueError("UNREVIEWED_DIAGNOSTIC_PROMOTION")
    stage = report.get("stage")
    if stage not in ("D2", "D3") or len(report["batches"]) != (1 if stage=="D2" else 3):
        raise ValueError("INCOMPLETE_DIAGNOSTIC_STAGE")
    expected_ids = [2,3,4,5,10] if stage=="D2" else [1,2,3,4,5,10]
    for batch in report["batches"]:
        if (not batch.get("complete")
                or [r["servo_id"] for r in batch["channels"]] != expected_ids
                or any(r.get("error") or not r.get("outer_frame_validated") for r in batch["channels"])):
            raise ValueError("INVALID_DIAGNOSTIC_CHANNELS")
        if batch.get("runtime_source_sha") != expected_sha:
            raise ValueError("RUNTIME_SOURCE_MISMATCH")
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=["D2","D3"], required=True)
    parser.add_argument("--expected-source-sha", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prior-d2", type=Path)
    args = parser.parse_args(argv)
    validate_previous(args.stage, args.expected_source_sha, args.prior_d2)
    # Reserve the destination before starting ROS or sending any RPC.
    with args.output.open("x") as output:
        record = dict(stage=args.stage, expected_source_sha=args.expected_source_sha,
            diagnostic_service_requests_sent=0, automatic_retry=False, client_status="NOT_SENT")
        output.write(json.dumps(record)+"\n"); output.flush()
        import rclpy
        from std_srvs.srv import Trigger
        rclpy.init()
        node = rclpy.create_node("idc_r1_finite_capture", namespace="/idc_vla/real")
        try:
            service = "/ros_robot_controller/bus_servo/diagnostic_"+args.stage.lower()
            client = node.create_client(Trigger, service)
            if not client.wait_for_service(timeout_sec=5):
                record["client_status"] = "SERVICE_UNAVAILABLE_STOP"
            else:
                record["client_start_monotonic_ns"] = time.monotonic_ns()
                record["client_start_system_time_ns"] = time.time_ns()
                future = client.call_async(Trigger.Request())
                record["diagnostic_service_requests_sent"] = 1
                rclpy.spin_until_future_complete(node, future, timeout_sec=15)
                record["client_rtt_ns"] = time.monotonic_ns()-record["client_start_monotonic_ns"]
                if not future.done():
                    record["client_status"] = "CLIENT_TIMEOUT_OUTCOME_UNKNOWN_STOP"
                else:
                    response = future.result()
                    record.update(client_status="RESPONSE_RECEIVED",
                        response_success=response.success,
                        diagnostic=json.loads(response.message))
                    record["validated_complete"] = validate_response(
                        record["diagnostic"], args.expected_source_sha)
                    from cable_robot_vla.readback_metrics import diagnostic_statistics
                    record["statistics"] = diagnostic_statistics(record["diagnostic"])
        except Exception as error:
            record.update(client_status="CLIENT_ERROR_STOP", exception_type=type(error).__name__,
                          exception_message=str(error))
        finally:
            output.seek(0); output.truncate()
            output.write(json.dumps(record, indent=2)+"\n")
            node.destroy_node()
            rclpy.shutdown()
        print(json.dumps(record, indent=2))
        return 0 if record.get("validated_complete") and record.get("response_success") else 2


if __name__ == "__main__":
    raise SystemExit(main())
