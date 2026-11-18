#!/usr/bin/env python3
"""Ubuntu ROS mock observations -> real cloud GPU -> rejection of unmapped actions."""
import argparse
import json
import os
from pathlib import Path
import sys
from time import monotonic

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "vla_protocol/src"), str(ROOT / "vla_server/src"), str(ROOT / "cable_robot_vla")]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:18000")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--domain", type=int, default=83)
    args = parser.parse_args()
    from urllib.parse import urlparse
    url = urlparse(args.url)
    if url.hostname != "127.0.0.1" or url.scheme != "http" or url.username or url.password:
        parser.error("use the loopback SSH tunnel")
    if not 1 <= args.domain <= 101 or not 2 <= args.cycles <= 20:
        parser.error("domain must be 1..101 and cycles 2..20")
    out = Path(args.output_dir).resolve()
    if out.is_relative_to(ROOT):
        parser.error("save experiment outputs outside source")
    out.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(args.domain), ROS_LOG_DIR=str(out / "ros"),
        PYTHONDONTWRITEBYTECODE="1")
    import httpx
    health = httpx.get(args.url + "/health", timeout=5, trust_env=False).json()
    if not (health["policy_ready"] and health["gpu_model_ready"] and health["model_type"] == "smolvla"):
        raise RuntimeError("require the real ready SmolVLA service")
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from cable_robot_vla.mock_robot_node import MockRobotNode
    from cable_robot_vla.vla_node import VLANode
    report = {"status": "FAIL", "test_kind": "real_ros_dds_http_gpu_synthetic_inspection",
        "simulation_only": True, "robot_actuation": False, "url": args.url, "ros_domain_id": args.domain,
        "server_revision": health["source_revision"]}
    robot = pipeline = monitor = executor = None
    outcomes, commands = [], []
    rclpy.init()
    try:
        robot = MockRobotNode()
        pipeline = VLANode(parameter_overrides=[Parameter("server_url", value=args.url),
            Parameter("enable_native_protocol", value=True), Parameter("max_horizon", value=64),
            Parameter("request_timeout_s", value=30.0), Parameter("inference_interval_s", value=1.0),
            Parameter("advance_simulation", value=False), Parameter("allow_unvalidated_simulation", value=False),
            Parameter("log_dir", value=str(out / "loops"))])
        monitor = rclpy.create_node("smolvla_cloud_inspection_monitor", namespace="/idc_vla/mock")
        subscriptions = [monitor.create_subscription(String, "/idc_vla/mock/outcomes", lambda msg: outcomes.append(json.loads(msg.data)), 30),
            monitor.create_subscription(String, "/idc_vla/mock/execute", lambda msg: commands.append(msg.data), 10)]
        executor = SingleThreadedExecutor()
        for node in (robot, pipeline, monitor):
            executor.add_node(node)
        deadline = monotonic() + 90
        while monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
            rejected = [e for e in outcomes if e.get("event") == "candidate_rejected"]
            if len(rejected) >= args.cycles:
                break
        candidates = [e for e in outcomes if e.get("event") == "candidate_received"]
        rejected = [e for e in outcomes if e.get("event") == "candidate_rejected"]
        assert len(candidates) >= args.cycles and len(rejected) >= args.cycles, outcomes
        assert all(e["policy_action"]["shape"] == [50, 6] and not e["policy_action"]["synthetic"]
            and e["policy_action"]["policy"]["policy_type"] == "smolvla"
            and e["policy_action"]["semantics"] == "unknown" for e in candidates)
        assert all(e["error"]["code"] == "UNSUPPORTED_ACTION_PROFILE" for e in rejected), rejected
        assert not commands and robot.state.q == [0.] * 5 and robot.state.state_revision == 0
        assert all(not p.topic_name.startswith("/servo") for p in pipeline.publishers)
        assert len(list(pipeline.clients)) == 0
        report.update(status="PASS", received=len(candidates), rejected=len(rejected), execution_messages=len(commands),
            final_positions=robot.state.q, final_state_revision=robot.state.state_revision,
            round_trip_s=[e["network_round_trip_s"] for e in candidates],
            policy_s=[e["policy_action"]["inference_duration_s"] for e in candidates],
            action_shape=[50, 6], rejection_code="UNSUPPORTED_ACTION_PROFILE")
    except Exception as exc:
        import traceback
        report["error"] = traceback.format_exc()
    finally:
        if executor:
            executor.shutdown()
        for node in (pipeline, robot, monitor):
            if node:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        report["clean_shutdown"] = True
        (out / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
