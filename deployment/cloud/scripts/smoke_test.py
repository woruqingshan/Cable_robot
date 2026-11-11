"""Actual HTTP smoke test; never loads a model or requests motor execution."""
import argparse
import base64
from hashlib import sha256
import json
import sys
from time import time_ns
from uuid import uuid4
from urllib.parse import urlparse
import httpx
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.observation import Observation
from vla_protocol.png import encode_rgb
from vla_protocol.profiles import JOINT_NAMES, RuntimeLimits


def smoke_observation():
    stamp = {"nanoseconds": time_ns(), "clock_domain": "ros_system"}
    png = encode_rgb(2, 2, bytes([20, 40, 60] * 4))
    return Observation(schema_version=1, request_id=str(uuid4()), session_id=str(uuid4()), observation_seq=0,
        task_revision=0, state_revision=0, robot_id="jetrover_mock", robot_profile_id="jetrover_joint_abs_rad_v1",
        scene_id="stage1_synthetic", scene_sha256="0" * 64, model_id="synthetic_joint_state_v1",
        instruction="Hold the synthetic arm", joints={"names": list(JOINT_NAMES), "positions": [0.] * 5, "unit": "rad", "stamp": stamp},
        gripper={"name": "r_joint", "position": 0.3, "unit": "rad", "stamp": stamp, "simulated": True, "calibrated": False},
        images=[{"camera_id": "mock_rgb", "frame_id": "mock_camera_optical_frame", "stamp": stamp, "width": 2, "height": 2,
            "codec": "png", "pixel_encoding": "rgb8", "sha256": sha256(png).hexdigest(), "data_base64": base64.b64encode(png).decode()}],
        simulated=True, calibration_status="synthetic_uncalibrated", capabilities=["native_actions_v1", "session_state_v1"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8000")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--allow-real-model", action="store_true", help="Stage 2 only; normal smoke rejects genuine model selection")
    args = parser.parse_args()
    parsed = urlparse(args.url)
    if parsed.hostname not in ("127.0.0.1", "localhost", "::1") or parsed.scheme not in ("http", "https") or parsed.username or parsed.password:
        parser.error("use a localhost endpoint, including an SSH-forwarded remote service")
    try:
        with httpx.Client(timeout=args.timeout, trust_env=False) as client:
            health_response = client.get(args.url.rstrip("/") + "/health")
            health_response.raise_for_status()
            health = health_response.json()
            if not health["http_service_available"] or not health["policy_ready"]:
                raise ValueError("policy is not ready: " + str(health.get("startup_error")))
            if health["model_type"] == "smolvla" and not args.allow_real_model:
                raise ValueError("Stage 1.5 smoke never invokes a real model; use the explicit Stage 2 flag after model validation")
            obs = smoke_observation()
            response = client.post(args.url.rstrip("/") + "/v1/infer", content=codec.dumps(obs), headers={"Content-Type": "application/json"})
            response.raise_for_status()
            limits = RuntimeLimits.model_validate(health["limits"])
            action = codec.loads(response.content, PolicyActionChunk, limits)
            assert (action.request_id, action.session_id, action.observation_seq) == (obs.request_id, obs.session_id, obs.observation_seq)
            if health["model_type"] == "mock":
                assert action.values == [obs.joints.positions] and action.synthetic
            bad = client.post(args.url.rstrip("/") + "/v1/infer", json={"schema_version": 999})
            assert bad.status_code == 422 and bad.json()["code"] == "UNSUPPORTED_VERSION"
            print(json.dumps({"status": "PASS", "model_type": health["model_type"], "shape": action.shape,
                "native_action_space": action.native_action_space, "synthetic": action.synthetic,
                "gpu_inference_verified": health["genuine_inference_verified"], "error_handling": "PASS"}, indent=2))
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": type(exc).__name__ + ": " + str(exc)}), file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
