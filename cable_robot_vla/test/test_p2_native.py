"""Opt-in real P2/FCL integration. Missing explicit resources fail, never fake-pass."""
import os
from pathlib import Path
import sys
import pytest
from cable_robot_vla.adapters.p2 import P2Adapter, P2Config
from cable_robot_vla.action_adapter import adapt_action
from deployment.local.test_support import observation, moved_candidate


@pytest.mark.parametrize("workspace_enabled", [False, True])
def test_real_model_native_hold_and_joint_limit_rejection(workspace_enabled):
    assert os.environ.get("IDC_P2_ROOT"), "Set IDC_P2_ROOT to the actual workspace_analysis directory"
    assert os.environ.get("C1_BACKEND"), "Set C1_BACKEND to the actual Native executable"
    root = Path(os.environ["IDC_P2_ROOT"]).resolve()
    adapter = P2Adapter(P2Config(mode="required", workspace_root=str(root),
        model_config_path=str(root / "config/workspace.yaml"),
        scene_path=str(root / "config/scenes/idc_prototype_v0.yaml"),
        srdf_path=str(root / "config/collision/semantic/standard_moveit_candidate_v0/standard_moveit_candidate_v0.srdf"),
        backend_path=os.environ["C1_BACKEND"], allow_candidate_semantics=True,
        workspace_enabled=workspace_enabled, task_path=str(root / "config/tasks/p22_research_v0.yaml")))
    try:
        adapter._load(0.3)
        from workspace_analysis.workspace.sampling import sample_joint_positions
        samples = sample_joint_positions(adapter.model.limits, 100, 42)
        good = next((q.tolist() for q in samples if adapter.checker.check(q, fresh=True)["valid"]), None)
        assert good is not None, "No valid candidate in the fixed 100-state probe"
        obs = observation(good, scene_id=adapter.scene.scene_id, scene_sha256=adapter.scene.canonical_scene_sha256,
                          model_id="urdf:" + adapter.model.metadata()["expanded_urdf_sha256"])
        hold = adapt_action(moved_candidate(obs, [good]))
        result = adapter.validate(hold, obs)
        assert result.status == "PASS_RESEARCH_ONLY", result.model_dump()
        assert result.provenance["checked_samples"] >= 2
        assert result.provenance["query_statistics"]["cache_hits"] == 0
        assert result.discrete_only and result.hardware_validated is False
        assert result.provenance["workspace_checked"] is workspace_enabled
        invalid = list(good)
        invalid[0] = float(adapter.model.limits[0, 1] + 1)
        bad_obs = observation(invalid, scene_id=obs.scene_id, scene_sha256=obs.scene_sha256, model_id=obs.model_id)
        rejected = adapter.validate(adapt_action(moved_candidate(bad_obs, [good])), bad_obs)
        assert rejected.status == "ERROR" and rejected.trajectory is None
        # Every accepted candidate still requires physical calibration and reviewed semantics.
        assert adapter.evaluator.identity["hardware_validated"] is False
    finally:
        adapter.close()


def test_real_p2_interpolation_rejects_interior_of_initial_and_later_segments():
    root = Path(os.environ["IDC_P2_ROOT"])
    sys.path.insert(0, str(root / "src"))
    import numpy as np
    from workspace_analysis.planning.validity import ConfigurationValidity
    class Model:
        def validate_q(self, q): return np.asarray(q)
        def metadata(self): return {"expanded_urdf_sha256": "a" * 64}
    class Evaluator:
        semantic = object()
        model = Model()
        identity = {"test_fake_collision_oracle": True}
        last_native = None
        def evaluate(self, values):
            q = values[0]
            blocked = bool(abs(q[0]) < 0.04)
            return [{"model_configuration_valid": not blocked,
                "semantic_self_collision": blocked, "environment_collision": False,
                "configuration_decision": {"planning_approved": False, "hardware_validated": False},
                "semantic_blocking_pairs": [], "raw_collision_pairs": []}]
    evaluator = Evaluator()
    for initial, waypoints in [([-0.1, 0., 0., 0., 0.], [[0.1, 0., 0., 0., 0.]]),
                               ([-0.1, 0., 0., 0., 0.], [[-0.09, 0., 0., 0., 0.], [0.1, 0., 0., 0., 0.]])]:
        adapter = P2Adapter(P2Config(mode="required"))
        adapter.model, adapter.evaluator, adapter.checker = evaluator.model, evaluator, ConfigurationValidity(evaluator)
        adapter.scene = type("Scene", (), {"scene_id": "stage1_synthetic", "canonical_scene_sha256": "0" * 64})()
        adapter.task_identity = None
        adapter._load = lambda _: None
        obs = observation(initial, model_id="urdf:" + "a" * 64)
        result = adapter.validate(adapt_action(moved_candidate(obs, waypoints)), obs)
        assert result.status == "REJECTED" and result.trajectory is None
        assert result.provenance["first_failure"]["path_sample_index"] > 0
        assert result.failure_reasons == ["SEMANTIC_SELF_COLLISION"]
