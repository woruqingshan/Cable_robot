import json
import time
import pytest
from fastapi.testclient import TestClient
from vla_protocol import codec
from vla_protocol.action import PolicyActionChunk
from vla_protocol.profiles import RuntimeLimits
from vla_server.app import create_app
from vla_server.policies.mock import HoldPositionPolicy
from deployment.local.test_support import observation


def test_health_roundtrip_determinism_and_lifecycle():
    class Tracked(HoldPositionPolicy):
        started = closed = False
        def start(self): self.started = True
        def close(self): self.closed = True
    policy = Tracked()
    with TestClient(create_app(policy)) as client:
        assert policy.started
        health = client.get("/health").json()
        assert health["gpu_model_ready"] is False
        assert health["supported_robot_profiles"] == ["jetrover_joint_abs_rad_v1"]
        obs = observation([0.1, 0.2, 0.3, 0.4, 0.5])
        results = [client.post("/v1/infer", content=codec.dumps(obs), headers={"Content-Type": "application/json"}) for _ in range(2)]
        assert [r.status_code for r in results] == [200, 200]
        actions = [codec.loads(r.content, PolicyActionChunk) for r in results]
        assert actions[0].values == [obs.joints.positions]
        assert actions[0].action_id == actions[1].action_id
        assert actions[0].request_id == obs.request_id
    assert policy.closed


@pytest.mark.parametrize("body,content_type,code", [
    (b"{", "application/json", "INVALID_JSON"),
    (b"{}", "text/plain", "INVALID_CONTENT_TYPE"),
    (b'{"schema_version":2}', "application/json", "UNSUPPORTED_VERSION"),
    (b'{"schema_version":1,"schema_version":1}', "application/json", "INVALID_JSON"),
])
def test_bad_requests(body, content_type, code):
    with TestClient(create_app()) as client:
        result = client.post("/v1/infer", content=body, headers={"Content-Type": content_type})
        assert result.status_code == 422
        assert result.json()["code"] == code


def test_size_limits_apply_to_stream_without_content_length():
    with TestClient(create_app(limits=RuntimeLimits(max_body_bytes=1024))) as client:
        response = client.post("/v1/infer", content=iter([b" " * 700, b" " * 700]), headers={"Content-Type": "application/json"})
        assert response.status_code == 413
        assert response.json()["code"] == "PAYLOAD_TOO_LARGE"


@pytest.mark.parametrize("behavior", ["exception", "wrong_identity", "malformed"])
def test_policy_failures_are_explicit_and_correlated(behavior):
    class Broken(HoldPositionPolicy):
        def predict(self, observation):
            if behavior == "exception": raise RuntimeError("private internal failure")
            if behavior == "malformed": return {"bad": True}
            return super().predict(observation).model_copy(update={"request_id": "wrong"})
    obs = observation()
    with TestClient(create_app(Broken())) as client:
        response = client.post("/v1/infer", json=obs.model_dump())
        assert response.status_code == 500
        assert response.json()["request_id"] == obs.request_id
        assert response.json()["code"] == "POLICY_ERROR"
        assert "private internal" not in response.text


def test_policy_deadline_returns_explicit_timeout():
    class Slow(HoldPositionPolicy):
        active = False
        closed = False
        def predict(self, obs):
            self.active = True
            try:
                time.sleep(0.05)
                return super().predict(obs)
            finally:
                self.active = False
        def close(self):
            assert not self.active, "policy teardown must wait for timed-out worker inference"
            self.closed = True
    obs = observation()
    policy = Slow()
    with TestClient(create_app(policy, limits=RuntimeLimits(request_timeout_s=0.01))) as client:
        response = client.post("/v1/infer", json=obs.model_dump())
        assert response.status_code == 504 and response.json()["code"] == "TIMEOUT"
        assert response.json()["request_id"] == obs.request_id
    assert policy.closed
