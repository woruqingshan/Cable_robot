"""Bounded request-response service. No ROS controller dependency."""
import argparse
import asyncio
from concurrent.futures import ThreadPoolExecutor
from contextlib import asynccontextmanager
import logging
from time import perf_counter
from time import monotonic

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, Response
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.errors import VLAError
from vla_protocol.observation import Observation, identity_fields
from vla_protocol.profiles import RuntimeLimits, ROBOT_PROFILE
from .policy_factory import create_policy
from .config import ServerConfig, load_config
from .session import SerialPolicyRunner, RequestToken

LOG = logging.getLogger(__name__)


def create_app(policy=None, limits: RuntimeLimits | None = None, config=None) -> FastAPI:
    config = config or ServerConfig()
    policy = policy or create_policy(config.policy)
    limits = limits or config.limits
    inference_timeout = config.timing.inference_timeout_s or limits.request_timeout_s
    runner = SerialPolicyRunner(policy, limits)

    @asynccontextmanager
    async def lifespan(app):
        # A timed-out HTTP request cannot stop a Python inference thread. Keep
        # start/predict/close on ONE owned worker so cancellation never permits
        # concurrent model calls or teardown while an inference is still running.
        worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla_policy")
        app.state.policy_worker = worker
        loop = asyncio.get_running_loop()
        app.state.started = False
        app.state.startup_error = None
        app.state.warmup_verified = False
        phase = "startup"
        try:
            try:
                await asyncio.wait_for(loop.run_in_executor(worker, policy.start), config.timing.startup_timeout_s)
                if config.timing.warmup_observation_path:
                    phase = "warmup"
                    from pathlib import Path
                    warmup = codec.loads(Path(config.timing.warmup_observation_path).read_bytes(), Observation, limits)
                    await asyncio.wait_for(loop.run_in_executor(worker, runner.warmup, warmup), config.timing.warmup_timeout_s)
                    app.state.warmup_verified = True
                app.state.started = True
            except Exception as exc:
                error = (VLAError("POLICY_STARTUP_TIMEOUT" if phase == "startup" else "WARMUP_TIMEOUT", phase + " deadline exceeded")
                         if isinstance(exc, asyncio.TimeoutError) else exc if isinstance(exc, VLAError)
                         else VLAError("POLICY_STARTUP_FAILED", type(exc).__name__ + ": " + str(exc)))
                app.state.startup_error = error.report()
                LOG.error("policy unavailable: %s", app.state.startup_error)
            yield
        finally:
            app.state.started = False
            try:
                await loop.run_in_executor(worker, policy.close)
            finally:
                worker.shutdown(wait=True, cancel_futures=True)

    app = FastAPI(title="IDC VLA inference service", version="0.1.0", lifespan=lifespan)
    lock = asyncio.Lock()

    @app.exception_handler(VLAError)
    async def protocol_error(request, exc):
        status = 413 if exc.code == "PAYLOAD_TOO_LARGE" else 504 if exc.code == "TIMEOUT" else 422
        return JSONResponse(exc.report(), status_code=status)

    @app.get("/health")
    def health():
        ready = getattr(app.state, "started", False)
        return {"schema_version": 1, "service": "mock_policy_server" if policy.model_type == "mock" else "vla_policy_server", "status": "ready" if ready else "not_ready",
                "http_service_available": True, "policy_ready": ready, "model_type": policy.model_type,
                "policy": policy.identity.model_dump(), "gpu_model_ready": bool(ready and policy.gpu_loaded),
                "model_loaded": bool(policy.model_loaded), "genuine_inference_verified": bool(policy.genuine_inference_verified),
                "warmup_verified": getattr(app.state, "warmup_verified", False),
                "startup_error": getattr(app.state, "startup_error", None), "source_revision": config.source_revision,
                "session_generation": runner.generation, "inference_timeout_s": inference_timeout,
                "protocol_extensions": ["native_actions_v1", "source_profiles_v1", "session_state_v1"],
                "supported_native_action_spaces": policy.native_action_spaces,
                "supported_robot_profiles": [ROBOT_PROFILE], "limits": limits.model_dump()}

    async def read_body(request):
        content_type = request.headers.get("content-type", "").split(";")[0].strip().lower()
        if content_type != "application/json":
            raise VLAError("INVALID_CONTENT_TYPE", "application/json required")
        length = request.headers.get("content-length")
        if length is not None:
            try:
                length = int(length)
                if length < 0:
                    raise ValueError()
            except ValueError as exc:
                raise VLAError("INVALID_HTTP", "invalid Content-Length") from exc
            if length > limits.max_body_bytes:
                raise VLAError("PAYLOAD_TOO_LARGE", "request body exceeds limit")
        chunks, total = [], 0
        async for block in request.stream():
            total += len(block)
            if total > limits.max_body_bytes:
                raise VLAError("PAYLOAD_TOO_LARGE", "streamed request exceeds limit")
            chunks.append(block)
        return b"".join(chunks)

    @app.post("/v1/infer")
    async def infer(request: Request):
        try:
            data = await asyncio.wait_for(read_body(request), limits.request_timeout_s)
        except asyncio.TimeoutError as exc:
            raise VLAError("TIMEOUT", "request body deadline exceeded") from exc
        observation = codec.loads(data, Observation, limits)
        request_id = observation.request_id
        if not app.state.started:
            return JSONResponse(VLAError("POLICY_NOT_READY", "selected policy failed startup/readiness", request_id=request_id).report(), status_code=503)
        token = RequestToken(monotonic() + inference_timeout)
        try:
            async def predict():
                async with lock:
                    future = asyncio.get_running_loop().run_in_executor(app.state.policy_worker, runner.predict, observation, token)
                    # A disconnected/timed-out HTTP handler may stop awaiting it.
                    # Drain discarded exceptions while the owned worker finishes.
                    future.add_done_callback(lambda done: None if done.cancelled() else done.exception())
                    while not future.done():
                        if await request.is_disconnected():
                            token.cancelled.set()
                            raise VLAError("REQUEST_INVALIDATED", "HTTP client disconnected")
                        await asyncio.sleep(0.005)
                    return await future
            result = await asyncio.wait_for(predict(), inference_timeout)
            response_limits = limits.model_copy(update={"max_body_bytes": limits.max_response_bytes})
            return Response(codec.dumps(result, response_limits), media_type="application/json")
        except asyncio.TimeoutError:
            token.cancelled.set()
            return JSONResponse(VLAError("TIMEOUT", "policy deadline exceeded", request_id=request_id).report(), status_code=504)
        except asyncio.CancelledError:
            token.cancelled.set()
            raise
        except VLAError as exc:
            token.cancelled.set()
            if exc.code in ("REQUEST_INVALIDATED", "SESSION_RETIRED", "STALE_OBSERVATION_SEQUENCE", "REQUEST_ID_REUSE", "PROTOCOL_EXTENSION_REQUIRED"):
                exc.request_id = request_id
                return JSONResponse(exc.report(), status_code=409)
            return JSONResponse(VLAError("POLICY_ERROR", "policy failed to produce a valid correlated candidate", request_id=request_id).report(), status_code=500)
        except Exception:
            token.cancelled.set()
            LOG.exception("policy failed for request %s", request_id)
            return JSONResponse(VLAError("POLICY_ERROR", "policy failed to produce a valid correlated candidate", request_id=request_id).report(), status_code=500)

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config")
    parser.add_argument("--host", choices=["127.0.0.1", "::1"])
    parser.add_argument("--port", type=int)
    parser.add_argument("--max-body-bytes", type=int)
    parser.add_argument("--max-image-bytes", type=int)
    parser.add_argument("--max-horizon", type=int)
    parser.add_argument("--request-timeout-s", type=float)
    args = parser.parse_args()
    config = load_config(args.config) if args.config else ServerConfig()
    data = config.model_dump()
    for name in ("max_body_bytes", "max_image_bytes", "max_horizon", "request_timeout_s"):
        if getattr(args, name) is not None:
            data["limits"][name] = getattr(args, name)
    for name in ("host", "port"):
        if getattr(args, name) is not None:
            data[name] = getattr(args, name)
    config = ServerConfig.model_validate(data)
    import uvicorn
    uvicorn.run(create_app(config=config), host=config.host, port=config.port, log_level="info")


if __name__ == "__main__":
    main()
