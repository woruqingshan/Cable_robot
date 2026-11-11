"""Bounded request-response service. No ROS controller dependency."""
import argparse
import asyncio
from contextlib import asynccontextmanager
import logging
from time import perf_counter

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, Response
from starlette.concurrency import run_in_threadpool
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.errors import VLAError
from vla_protocol.observation import Observation, identity_fields
from vla_protocol.profiles import RuntimeLimits, ROBOT_PROFILE
from .policies.mock import HoldPositionPolicy

LOG = logging.getLogger(__name__)


def create_app(policy=None, limits: RuntimeLimits | None = None) -> FastAPI:
    policy = policy or HoldPositionPolicy()
    limits = limits or RuntimeLimits()

    @asynccontextmanager
    async def lifespan(app):
        policy.start()
        app.state.started = True
        try:
            yield
        finally:
            app.state.started = False
            policy.close()

    app = FastAPI(title="IDC VLA Stage 1", version="0.1.0", lifespan=lifespan)
    lock = asyncio.Lock()

    @app.exception_handler(VLAError)
    async def protocol_error(request, exc):
        status = 413 if exc.code == "PAYLOAD_TOO_LARGE" else 504 if exc.code == "TIMEOUT" else 422
        return JSONResponse(exc.report(), status_code=status)

    @app.get("/health")
    def health():
        return {"schema_version": 1, "service": "mock_policy_server", "status": "ready",
                "policy": policy.identity.model_dump(), "gpu_model_ready": False,
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
        try:
            async def predict():
                async with lock:
                    begin = perf_counter()
                    result = await run_in_threadpool(policy.predict, observation)
                    result = codec.validate(result.model_dump(), PolicyActionChunk, limits)
                    if identity_fields(result) != identity_fields(observation):
                        raise VLAError("POLICY_OUTPUT_INVALID", "policy changed request identity")
                    return codec.validate({**result.model_dump(), "inference_duration_s": perf_counter() - begin}, PolicyActionChunk, limits)
            result = await asyncio.wait_for(predict(), limits.request_timeout_s)
            return Response(codec.dumps(result, limits), media_type="application/json")
        except asyncio.TimeoutError:
            return JSONResponse(VLAError("TIMEOUT", "policy deadline exceeded", request_id=request_id).report(), status_code=504)
        except Exception:
            LOG.exception("policy failed for request %s", request_id)
            return JSONResponse(VLAError("POLICY_ERROR", "policy failed to produce a valid correlated candidate", request_id=request_id).report(), status_code=500)

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--max-body-bytes", type=int, default=6_000_000)
    parser.add_argument("--max-image-bytes", type=int, default=1_048_576)
    parser.add_argument("--max-horizon", type=int, default=32)
    parser.add_argument("--request-timeout-s", type=float, default=2.0)
    args = parser.parse_args()
    limits = RuntimeLimits(max_body_bytes=args.max_body_bytes, max_image_bytes=args.max_image_bytes,
                           max_horizon=args.max_horizon, request_timeout_s=args.request_timeout_s)
    import uvicorn
    uvicorn.run(create_app(limits=limits), host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
