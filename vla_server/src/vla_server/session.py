"""Single-owner temporal state, idempotent requests and cancellation tokens."""
from collections import OrderedDict, deque
from dataclasses import dataclass, field
from hashlib import sha256
import json
from threading import Event
from time import monotonic, perf_counter
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.observation import identity_fields
from vla_protocol.errors import VLAError


@dataclass
class RequestToken:
    deadline: float
    cancelled: Event = field(default_factory=Event)

    def invalid(self):
        return self.cancelled.is_set() or monotonic() > self.deadline


class SerialPolicyRunner:
    """All methods run on the SAME owned policy thread, never ROS callbacks."""
    def __init__(self, policy, limits):
        self.policy, self.limits = policy, limits
        self.context = None
        self.generation = 0
        self.last_seq = -1
        self.last_token = None
        self.retired = deque(maxlen=256)
        self.cache = OrderedDict()
        self.dirty = False

    def _transition(self, observation):
        session = (observation.robot_id, observation.session_id)
        source = observation.source_profile.model_dump() if observation.source_profile else {"profile_id": "synthetic_uncalibrated_v1"}
        context = (session, observation.task_revision, observation.scene_id, observation.scene_sha256,
                   observation.model_id, observation.robot_profile_id, json.dumps(source, sort_keys=True))
        if session in self.retired:
            raise VLAError("SESSION_RETIRED", "start a new session after another session has taken ownership")
        if self.context is not None and session == self.context[0] and observation.task_revision < self.context[1]:
            raise VLAError("REQUEST_INVALIDATED", "task revision predates current model state")
        cancelled = self.last_token is not None and self.last_token.cancelled.is_set()
        if context != self.context or self.dirty or cancelled:
            reason = "late_or_invalidated_request" if self.dirty or cancelled else "new_session" if self.context is None or session != self.context[0] else "context_change"
            if self.context is not None and session != self.context[0]:
                self.retired.append(self.context[0])
            self.policy.reset(context, reason=reason)
            self.context, self.last_seq, self.last_token = context, -1, None
            self.generation += 1
            self.dirty = False

    def predict(self, observation, token):
        if token.invalid():
            raise VLAError("TIMEOUT", "request expired before entering the policy worker")
        self._transition(observation)
        digest = sha256(codec.dumps(observation, self.limits)).hexdigest()
        cached = self.cache.get(observation.request_id)
        if cached is not None:
            if cached[0] != digest:
                raise VLAError("REQUEST_ID_REUSE", "request identity was reused with different input")
            if cached[2] != self.generation:
                raise VLAError("REQUEST_INVALIDATED", "cached request belongs to invalidated temporal state")
            return codec.validate(cached[1], PolicyActionChunk, self.limits)
        if observation.observation_seq <= self.last_seq:
            raise VLAError("STALE_OBSERVATION_SEQUENCE", "model-side sequence must increase within one context")
        self.last_token = token
        begin = perf_counter()
        try:
            result = self.policy.predict(observation)
            if token.invalid():
                raise VLAError("TIMEOUT", "late policy result discarded; temporal state will reset")
            result = codec.validate(result.model_dump(), PolicyActionChunk, self.limits)
            if result.horizon is not None and "native_actions_v1" not in observation.capabilities:
                raise VLAError("PROTOCOL_EXTENSION_REQUIRED", "model-native horizon metadata requires explicit native_actions_v1")
            if identity_fields(result) != identity_fields(observation):
                raise VLAError("POLICY_OUTPUT_INVALID", "policy changed request identity")
            result = codec.validate({**result.model_dump(), "session_generation": self.generation if "session_state_v1" in observation.capabilities else None,
                "inference_duration_s": perf_counter() - begin}, PolicyActionChunk, self.limits)
            self.last_seq = observation.observation_seq
            self.cache[observation.request_id] = (digest, result.model_dump(), self.generation)
            if len(self.cache) > 128:
                self.cache.popitem(last=False)
            return result
        except BaseException:
            self.dirty = True
            raise

    def warmup(self, observation):
        self.policy.reset(None, reason="warmup_start")
        try:
            result = self.policy.predict(observation)
            result = codec.validate(result.model_dump(), PolicyActionChunk, self.limits)
            if identity_fields(result) != identity_fields(observation):
                raise VLAError("POLICY_OUTPUT_INVALID", "warmup changed observation identity")
        finally:
            self.policy.reset(None, reason="warmup_end")
            self.context = None
