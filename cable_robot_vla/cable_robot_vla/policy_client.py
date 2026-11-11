"""One-flight HTTP worker; ROS callbacks only submit, invalidate and poll."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from time import monotonic, perf_counter
from urllib.parse import urlparse
import httpx
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.errors import VLAError
from vla_protocol.observation import Observation, identity_fields
from vla_protocol.profiles import RuntimeLimits


@dataclass(frozen=True)
class InferenceResult:
    observation: Observation
    candidate: PolicyActionChunk
    submitted_at: float
    encoding_s: float
    round_trip_s: float


class PolicyClient:
    def __init__(self, endpoint="http://127.0.0.1:8000", *, limits=None, transport=None, clock=monotonic):
        url = urlparse(endpoint)
        if url.scheme not in ("http", "https") or not url.hostname or url.username or url.password or url.query or url.fragment:
            raise VLAError("INVALID_ENDPOINT", "require an HTTP(S) server URL without credentials/query")
        self.endpoint = endpoint.rstrip("/")
        self.limits = limits or RuntimeLimits()
        self.clock = clock
        self.http = httpx.Client(timeout=self.limits.request_timeout_s, transport=transport,
                                 trust_env=False, follow_redirects=False)
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vla_http")
        self.future = None
        self.pending = None
        self.invalidated = None
        self.closed = False
        self.seen = deque(maxlen=1024)

    @property
    def busy(self):
        return self.future is not None

    def correlate(self, observation, candidate):
        if candidate.request_id in self.seen:
            raise VLAError("DUPLICATE_RESPONSE", "response already consumed", request_id=candidate.request_id)
        expected, actual = identity_fields(observation), identity_fields(candidate)
        for field in expected:
            if expected[field] != actual[field]:
                raise VLAError("IDENTITY_MISMATCH", "response differs at " + field, request_id=observation.request_id)

    def _infer(self, observation, submitted):
        begin = perf_counter()
        body = codec.dumps(observation, self.limits)
        encoding_s = perf_counter() - begin
        request_id = observation.request_id
        try:
            begin = perf_counter()
            with self.http.stream("POST", self.endpoint + "/v1/infer", content=body,
                                  headers={"Content-Type": "application/json", "Accept-Encoding": "identity"}) as response:
                if response.headers.get("content-encoding", "identity") != "identity":
                    raise VLAError("MALFORMED_RESPONSE", "compressed HTTP responses are unsupported; PNG already compresses images", request_id=request_id)
                declared = response.headers.get("content-length")
                if declared is not None:
                    try:
                        if not 0 <= int(declared) <= self.limits.max_response_bytes:
                            raise VLAError("PAYLOAD_TOO_LARGE", "declared response exceeds limit", request_id=request_id)
                    except ValueError as exc:
                        if isinstance(exc, VLAError):
                            raise
                        raise VLAError("MALFORMED_RESPONSE", "invalid response Content-Length", request_id=request_id) from exc
                blocks, total = [], 0
                for block in response.iter_bytes(chunk_size=65536):
                    total += len(block)
                    if total > self.limits.max_response_bytes:
                        raise VLAError("PAYLOAD_TOO_LARGE", "response exceeds configured limit", request_id=request_id)
                    if self.clock() - submitted > self.limits.request_timeout_s:
                        raise VLAError("TIMEOUT", "total response deadline exceeded", request_id=request_id)
                    blocks.append(block)
                raw = b"".join(blocks)
                if response.status_code != 200:
                    from vla_protocol.errors import ErrorResponse
                    error = codec.loads(raw, ErrorResponse, self.limits)
                    raise VLAError("SERVER_ERROR", error.code + ": " + error.message, request_id=request_id)
                if response.headers.get("content-type", "").split(";")[0] != "application/json":
                    raise VLAError("MALFORMED_RESPONSE", "server did not return JSON", request_id=request_id)
                response_limits = self.limits.model_copy(update={"max_body_bytes": self.limits.max_response_bytes})
                candidate = codec.loads(raw, PolicyActionChunk, response_limits)
                return InferenceResult(observation, candidate, submitted, encoding_s, perf_counter() - begin)
        except httpx.TimeoutException as exc:
            raise VLAError("TIMEOUT", "HTTP operation timed out", request_id=request_id) from exc
        except httpx.HTTPError as exc:
            raise VLAError("NETWORK_ERROR", type(exc).__name__, request_id=request_id) from exc

    def submit(self, observation: Observation):
        if self.closed or self.busy:
            raise VLAError("CLIENT_BUSY", "client closed or a request is still outstanding")
        observation = codec.validate(observation.model_dump(), Observation, self.limits)
        if observation.request_id in self.seen:
            raise VLAError("DUPLICATE_REQUEST", "request ID was already consumed")
        self.pending = observation
        self.submitted_at = self.clock()
        self.invalidated = None
        self.future = self.worker.submit(self._infer, observation, self.submitted_at)

    def invalidate(self, reason="REQUEST_INVALIDATED"):
        if self.busy:
            self.invalidated = reason

    def poll(self):
        if not self.busy:
            return None
        if self.clock() - self.submitted_at > self.limits.request_timeout_s:
            self.invalidated = self.invalidated or "STALE_RESPONSE"
        if not self.future.done():
            return None
        future, observation, invalidated = self.future, self.pending, self.invalidated
        self.future = self.pending = self.invalidated = None
        try:
            result = future.result()
            if invalidated:
                raise VLAError(invalidated, "request invalidated before consumption", request_id=observation.request_id)
            self.correlate(observation, result.candidate)
            return result
        finally:
            self.seen.append(observation.request_id)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.invalidate()
        self.worker.shutdown(wait=True, cancel_futures=True)
        self.http.close()
