"""Unit fakes exercise adapter delegation/failures, not Native integration."""
from types import SimpleNamespace
import pytest
from cable_robot_vla.adapters.p2 import P2Adapter, P2Config
from cable_robot_vla.action_adapter import adapt_action
from deployment.local.test_support import observation, moved_candidate


def adapter_with_checker(monkeypatch, obs, mode):
    adapter = P2Adapter(P2Config(mode="required"))
    adapter.model = SimpleNamespace(metadata=lambda: {"expanded_urdf_sha256": "a" * 64}, validate_q=lambda q: q)
    adapter.scene = SimpleNamespace(scene_id=obs.scene_id, canonical_scene_sha256=obs.scene_sha256)
    adapter.evaluator = SimpleNamespace(identity={"test_fake": True})
    adapter.task_identity = None
    def path(waypoints, resolution, **options):
        adapter.received = waypoints
        assert options["fresh"] is True and options["stop_first"] is True and options["deadline"] > 0
        if mode == "error": raise RuntimeError("backend unavailable")
        return {"valid": mode == "pass", "complete": mode == "pass", "validation": "DISCRETE_ONLY",
            "checked_samples": 3, "expected_samples": 10, "query_statistics": {}, "elapsed_s": 0.,
            "first_failure": None if mode == "pass" else {"reasons": ["CLEARANCE_OR_COLLISION_UNVERIFIED"]}}
    adapter.checker = SimpleNamespace(path=path)
    monkeypatch.setattr(adapter, "_load", lambda gripper: None)
    # Isolate the optional P2 path-sampling dependency in these unit tests.
    import sys
    fake = SimpleNamespace(sample_path=lambda path, res: (path, []))
    monkeypatch.setitem(sys.modules, "workspace_analysis.planning.joint_path", fake)
    return adapter


@pytest.mark.parametrize("mode,status", [("pass", "PASS_RESEARCH_ONLY"), ("unknown", "REJECTED"), ("error", "ERROR")])
def test_required_mode_delegates_initial_and_all_subsequent_segments(monkeypatch, mode, status):
    obs = observation(model_id="urdf:" + "a" * 64)
    action = adapt_action(moved_candidate(obs, [[0.01] * 5, [0.02] * 5]))
    adapter = adapter_with_checker(monkeypatch, obs, mode)
    result = adapter.validate(action, obs)
    assert result.status == status
    assert adapter.received == [obs.joints.positions, *action.values]
    assert (result.trajectory is not None) == (mode == "pass")
    if mode == "pass": assert result.discrete_only and not result.hardware_validated


@pytest.mark.parametrize("field,value", [("scene_id", "other"), ("scene_sha256", "f" * 64), ("model_id", "other")])
def test_scene_model_mismatch_blocks_before_checker(monkeypatch, field, value):
    obs = observation(model_id="urdf:" + "a" * 64)
    adapter = adapter_with_checker(monkeypatch, obs, "pass")
    data = obs.model_dump()
    data[field] = value
    changed = type(obs).model_validate(data)
    action = adapt_action(moved_candidate(changed, [[0.] * 5]))
    result = adapter.validate(action, changed)
    assert result.status == "ERROR" and result.failure_reasons == ["P2_IDENTITY_MISMATCH"]
    assert not hasattr(adapter, "received")


def test_missing_native_executable_blocks_before_importing_p2(tmp_path):
    root = tmp_path / "workspace_analysis"
    marker = root / "src/workspace_analysis/planning/validity.py"
    marker.parent.mkdir(parents=True)
    marker.write_text("# deliberately not importable fake resource marker\n")
    for name in ("model.yaml", "scene.yaml", "semantic.srdf"):
        (tmp_path / name).write_text("unused: resource preflight must fail first\n")
    adapter = P2Adapter(P2Config(mode="required", workspace_root=str(root),
        model_config_path=str(tmp_path / "model.yaml"), scene_path=str(tmp_path / "scene.yaml"),
        srdf_path=str(tmp_path / "semantic.srdf"), backend_path=str(tmp_path / "missing_native")))
    obs = observation()
    result = adapter.validate(adapt_action(moved_candidate(obs, [[0.] * 5])), obs)
    assert result.status == "ERROR" and result.failure_reasons == ["P2_RESOURCES_MISSING"]
    assert "backend_path" in result.provenance["error"]
    assert adapter.evaluator is None
