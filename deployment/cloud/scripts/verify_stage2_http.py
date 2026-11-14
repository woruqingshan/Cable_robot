#!/usr/bin/env python3
"""Real loopback HTTP acceptance with owned server lifecycle; no robot execution."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
from time import monotonic, perf_counter, sleep
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / "vla_protocol/src"), str(ROOT / "vla_server/src"), str(ROOT / "cable_robot_vla")]
from urllib.request import Request, build_opener, ProxyHandler
from urllib.error import HTTPError, URLError
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_server.config import load_config
from smoke_test import smoke_observation
from verify_smolvla_inference import external_report


def observation(config, session=None, seq=0, all_cameras=True):
    data = smoke_observation().model_dump()
    template = data["images"][0]
    data.update(session_id=session or str(uuid4()), observation_seq=seq, model_id=config.policy.feature_spec.model_id)
    data["images"] = [{**template, "camera_id": f.camera_id} for f in config.policy.feature_spec.cameras if all_cameras or f.required]
    return data


def checked_action(response, obs, config):
    assert response.status_code == 200, response.text
    action = codec.loads(response.content, PolicyActionChunk, config.limits)
    assert action.shape == [50, 6] and not action.synthetic
    assert action.policy.policy_type == "smolvla" and action.semantics == "unknown"
    assert action.request_id == obs["request_id"] and action.session_id == obs["session_id"]
    assert action.horizon.generated == action.horizon.transmitted == 50
    return action


class HTTPClient:
    """Bounded loopback test transport; no cloud-only client dependency needed."""
    def __init__(self, base_url="", timeout=30, **_):
        self.base_url, self.timeout = base_url, timeout
        self.opener = build_opener(ProxyHandler({}))

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def request(self, method, path, *, json=None, content=None, headers=None):
        import json as json_module
        from types import SimpleNamespace
        headers = dict(headers or {})
        if json is not None:
            content = json_module.dumps(json, allow_nan=False).encode()
            headers["Content-Type"] = "application/json"
        request = Request(self.base_url + path, data=content, headers=headers, method=method)
        try:
            response = self.opener.open(request, timeout=self.timeout)
        except HTTPError as error:
            response = error
        with response:
            body = response.read(6_000_001)
            if len(body) > 6_000_000:
                raise ValueError("response exceeds protocol ceiling")
            return SimpleNamespace(status_code=response.code, content=body, text=body.decode(),
                json=lambda: json_module.loads(body), headers=response.headers)

    def get(self, path):
        return self.request("GET", path)

    def post(self, path, **kwargs):
        return self.request("POST", path, **kwargs)


def gpu_fault_probes(config):
    """Isolated ASGI fault injection with real GPU forwards; no HTTPX dependency."""
    import asyncio
    from vla_server.app import create_app
    from vla_server.policies.smolvla import SmolVLAPolicyAdapter

    async def request(app, data):
        body = json.dumps(data).encode()
        sent = False
        messages = []
        async def receive():
            nonlocal sent
            if not sent:
                sent = True
                return {"type": "http.request", "body": body, "more_body": False}
            await asyncio.Event().wait()
        async def send(message):
            messages.append(message)
        await app({"type": "http", "asgi": {"version": "3.0"}, "http_version": "1.1",
            "method": "POST", "scheme": "http", "path": "/v1/infer", "raw_path": b"/v1/infer",
            "root_path": "", "query_string": b"", "headers": [(b"content-type", b"application/json")],
            "client": ("127.0.0.1", 12345), "server": ("127.0.0.1", 8000)}, receive, send)
        from types import SimpleNamespace
        status = next(m["status"] for m in messages if m["type"] == "http.response.start")
        raw = b"".join(m.get("body", b"") for m in messages if m["type"] == "http.response.body")
        return SimpleNamespace(status_code=status, content=raw, text=raw.decode(), json=lambda: json.loads(raw))

    async def run():
        probes = []
        policy = SmolVLAPolicyAdapter(config.policy)
        app = create_app(policy, config=config)
        async with app.router.lifespan_context(app):
            assert app.state.started and policy.gpu_loaded
            original = policy.postprocess
            forwards = [0]
            def injected_postprocess(actions):
                forwards[0] += 1
                raise RuntimeError("explicit test-only failure after genuine GPU forward pass")
            policy.postprocess = injected_postprocess
            response = await request(app, observation(config))
            assert response.status_code == 500 and response.json()["code"] == "POLICY_ERROR"
            assert forwards[0] == 1
            probes.append({"name": "injected_postprocessor_failure", "status": "PASS", "http_status": 500,
                "kind": "ASGI with real GPU forward and explicitly injected processor exception"})
            policy.postprocess = original
            obs = observation(config)
            checked_action(await request(app, obs), obs, config)
            probes.append({"name": "recovery_after_failure", "status": "PASS", "kind": "ASGI real GPU inference"})
        assert policy.model is None and not policy.gpu_loaded
        short = config.model_copy(update={"timing": config.timing.model_copy(update={"inference_timeout_s": 0.05})})
        policy = SmolVLAPolicyAdapter(short.policy)
        app = create_app(policy, config=short)
        timeout_forwards = [0]
        async with app.router.lifespan_context(app):
            original_predict = policy.model.predict_action_chunk
            def tracked_predict(*args, **kwargs):
                output = original_predict(*args, **kwargs)
                timeout_forwards[0] += 1
                return output
            policy.model.predict_action_chunk = tracked_predict
            response = await request(app, observation(short))
            assert response.status_code == 504 and response.json()["code"] == "TIMEOUT"
        assert timeout_forwards[0] == 1 and policy.model is None
        probes.append({"name": "timeout_and_serial_cleanup", "status": "PASS", "http_status": 504,
            "kind": "ASGI real GPU forward; 50 ms deadline; late result discarded and worker drained"})
        bad = config.policy.model_copy(update={"checkpoint_path": str(Path(config.policy.checkpoint_path) / "deliberately_missing")})
        policy = SmolVLAPolicyAdapter(bad)
        app = create_app(policy, config=config.model_copy(update={"policy": bad}))
        async with app.router.lifespan_context(app):
            assert not app.state.started and not policy.genuine_inference_verified
            response = await request(app, observation(config))
            assert response.status_code == 503 and response.json()["code"] == "POLICY_NOT_READY"
        probes.append({"name": "unavailable_model", "status": "PASS", "http_status": 503,
            "kind": "ASGI real adapter startup with deliberately missing path; no forward pass"})
        return probes
    return asyncio.run(run())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--manifest", help="required when owning remote server")
    parser.add_argument("--report", required=True)
    parser.add_argument("--url", default="http://127.0.0.1:8000")
    parser.add_argument("--check-local-client", action="store_true", help="exercise installed Ubuntu HTTPX client and unmapped-action rejection")
    parser.add_argument("--connect-only", action="store_true", help="test an existing service/tunnel, do not stop it")
    parser.add_argument("--gpu-fault-probes", action="store_true", help="after clean shutdown, run isolated explicit fault tests")
    args = parser.parse_args()
    from urllib.parse import urlparse
    parsed = urlparse(args.url)
    if parsed.scheme != "http" or parsed.hostname != "127.0.0.1" or parsed.username or parsed.password or parsed.path not in ("", "/"):
        parser.error("require plain loopback HTTP, including an SSH-forwarded endpoint")
    if not args.connect_only and (not args.manifest or parsed.port != 8000):
        parser.error("owned GPU server requires --manifest and port 8000")
    if args.connect_only and args.gpu_fault_probes:
        parser.error("fault probes require ownership and clean shutdown of the GPU server")
    cfg = load_config(args.config)
    if cfg.policy.type != "smolvla":
        parser.error("this acceptance test requires the real SmolVLA policy")
    path = external_report(args.report, ROOT)
    report = {"schema_version": 1, "status": "FAIL", "test_kind": "real_http_gpu_inference",
        "started_utc": datetime.now(timezone.utc).isoformat(), "source_revision": cfg.source_revision,
        "synthetic_observation": True, "robot_actuation": False, "url": args.url, "tests": []}
    process = log = None
    def record(name, kind, **data):
        report["tests"].append({"name": name, "kind": kind, "status": "PASS", **data})
    try:
        if not args.connect_only:
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 8000))
            env = dict(os.environ, VLA_PYTHON=sys.executable, PYTHONDONTWRITEBYTECODE="1",
                HF_HOME=cfg.policy.hf_home, HF_HUB_CACHE=str(Path(cfg.policy.hf_home) / "hub"),
                HF_HUB_OFFLINE="1", TRANSFORMERS_OFFLINE="1", TOKENIZERS_PARALLELISM="false")
            log = path.with_suffix(".server.log").open("x")
            process = subprocess.Popen(["bash", str(ROOT / "deployment/cloud/scripts/start_server.sh"),
                "--manifest", args.manifest, "--config", args.config], env=env, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
        with HTTPClient(base_url=args.url.rstrip("/"), timeout=30, trust_env=False) as client:
            deadline = monotonic() + 150
            while True:
                if process is not None and process.poll() is not None:
                    raise RuntimeError("server exited; inspect " + str(path.with_suffix(".server.log")))
                try:
                    health = client.get("/health").json()
                    if not health["policy_ready"]:
                        raise RuntimeError("model startup failed: " + str(health.get("startup_error")))
                    break
                except (URLError, ConnectionError):
                    if monotonic() >= deadline:
                        raise
                    sleep(0.1)
            assert health["model_type"] == "smolvla" and health["gpu_model_ready"]
            assert health["source_revision"] == cfg.source_revision
            report["initial_health"] = health
            record("health", "actual HTTP readiness, no forward pass required")
            session = str(uuid4())
            first_obs = observation(cfg, session, 0)
            generations = []
            for index in range(3):
                obs = first_obs if index == 0 else observation(cfg, session, index)
                begin = perf_counter()
                response = client.post("/v1/infer", json=obs)
                elapsed = perf_counter() - begin
                action = checked_action(response, obs, cfg)
                runtime = client.get("/health").json()["runtime"]
                generations.append(action.session_generation)
                record(f"inference_{index}", "actual HTTP real GPU forward", shape=action.shape,
                    round_trip_s=elapsed, adapter_s=action.inference_duration_s,
                    server_timing=response.headers.get("server-timing"), runtime=runtime)
                if index == 0:
                    report["first_action_values"] = action.values
            # Cache the most recent request; never claim idempotent replay as another forward.
            response = client.post("/v1/infer", json=obs)
            assert checked_action(response, obs, cfg).values == action.values
            record("idempotent_retry", "actual HTTP cached genuine output; no new forward")
            assert len(set(generations)) == 1
            new_obs = observation(cfg, str(uuid4()), 0, all_cameras=False)
            action = checked_action(client.post("/v1/infer", json=new_obs), new_obs, cfg)
            assert action.session_generation > generations[-1]
            record("session_reset_single_camera", "actual HTTP real GPU forward", generation=action.session_generation, shape=action.shape)
            changed = observation(cfg, new_obs["session_id"], 1, all_cameras=False)
            changed["task_revision"] = 1
            action = checked_action(client.post("/v1/infer", json=changed), changed, cfg)
            record("task_reset", "actual HTTP real GPU forward", generation=action.session_generation)
            invalid = [
                ("malformed_json", b"{", 422),
                ("wrong_version", {"schema_version": 999}, 422),
                ("missing_state", {k: v for k, v in observation(cfg).items() if k != "joints"}, 422),
                ("wrong_state_shape", {**observation(cfg), "joints": {**first_obs["joints"], "positions": [0.] * 4}}, 422),
                ("missing_all_cameras", {**observation(cfg), "images": []}, 422),
                ("missing_required_camera", {**observation(cfg), "images": [{**first_obs["images"][1], "camera_id": "camera2"}]}, 422),
                ("unknown_camera", {**observation(cfg), "images": [{**first_obs["images"][0], "camera_id": "unknown"}]}, 422),
                ("blank_task", {**observation(cfg), "instruction": "  "}, 422),
            ]
            for name, payload, status in invalid:
                response = client.post("/v1/infer", content=payload, headers={"Content-Type": "application/json"}) if isinstance(payload, bytes) else client.post("/v1/infer", json=payload)
                assert response.status_code == status, (name, response.text)
                assert "code" in response.json()
                record(name, "actual HTTP input validation; no model forward", http_status=status, code=response.json()["code"])
            def concurrent(_):
                obs = observation(cfg)
                with HTTPClient(timeout=30, trust_env=False) as worker:
                    return checked_action(worker.post(args.url.rstrip("/") + "/v1/infer", json=obs), obs, cfg).shape
            with ThreadPoolExecutor(max_workers=2) as pool:
                assert list(pool.map(concurrent, range(2))) == [[50, 6], [50, 6]]
            record("concurrent_requests", "actual HTTP two real GPU forwards, serialized worker")
            report["final_health"] = client.get("/health").json()
            assert report["final_health"]["genuine_inference_verified"]
            if args.check_local_client:
                # Exercise the unchanged ROS-independent client and fail-closed action registry.
                from cable_robot_vla.policy_client import PolicyClient
                from cable_robot_vla.action_adapter import adapt_action
                from vla_protocol.errors import VLAError
                from vla_protocol.observation import Observation
                policy_client = PolicyClient(args.url, limits=cfg.limits)
                try:
                    policy_client.submit(Observation.model_validate(observation(cfg)))
                    deadline = monotonic() + 30
                    result = None
                    while result is None and monotonic() < deadline:
                        result = policy_client.poll()
                        sleep(0.01)
                    assert result is not None and result.candidate.shape == [50, 6]
                    try:
                        adapt_action(result.candidate)
                    except VLAError as exc:
                        assert exc.code == "UNSUPPORTED_ACTION_PROFILE"
                    else:
                        raise AssertionError("unknown real model output unexpectedly executable")
                    record("local_policy_client_and_action_boundary", "actual HTTP real GPU forward; unknown action mapping rejected",
                        round_trip_s=result.round_trip_s)
                finally:
                    policy_client.close()
        report["status"] = "PASS"
    except Exception as exc:
        import traceback
        report["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
    finally:
        if process is not None:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=60)
                report["server_exit_code"] = process.returncode
                if process.returncode != 0:
                    report["status"] = "FAIL"
            except subprocess.TimeoutExpired:
                report["status"] = "FAIL"
                report["shutdown_error"] = "owned server did not exit within 60 s; no forced kill issued"
            if log is not None:
                log.close()
        if args.gpu_fault_probes and report["status"] == "PASS":
            try:
                report["gpu_fault_probes"] = gpu_fault_probes(cfg)
            except Exception as exc:
                import traceback
                report["status"] = "FAIL"
                report["gpu_fault_error"] = traceback.format_exc()
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(json.dumps({"status": report["status"], "report": str(path), "error": report.get("error"),
            "gpu_fault_error": report.get("gpu_fault_error")}))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
