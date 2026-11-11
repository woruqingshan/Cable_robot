"""Borrow P2's sampled path checker without copying its algorithms."""
import json
from pathlib import Path
import sys
from threading import Lock
from time import perf_counter
from typing import Literal
from pydantic import Field
from vla_protocol.action import ValidatedTrajectory
from vla_protocol.profiles import StrictModel
from vla_protocol.errors import VLAError


class P2Config(StrictModel):
    mode: Literal["disabled", "required"] = "disabled"
    workspace_root: str = ""
    model_config_path: str = ""
    scene_path: str = ""
    srdf_path: str = ""
    backend_path: str = ""
    allow_candidate_semantics: bool = False
    resolution_rad: float = Field(default=0.01, gt=0, le=0.02)
    max_path_samples: int = Field(default=4096, ge=2, le=100000)
    backend_timeout_s: float = Field(default=5.0, gt=0, le=30)
    max_validation_time_s: float = Field(default=2.0, gt=0, le=30)
    self_clearance_m: float = Field(default=0.0, ge=0)
    environment_clearance_m: float = Field(default=0.0, ge=0)
    workspace_enabled: bool = False
    task_path: str = ""
    world_from_base: list[list[float]] = Field(default_factory=lambda: [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])


class P2Adapter:
    """Owned Native session, serial access, lazy initialization and fail-closed errors."""
    def __init__(self, config=None):
        self.config = config or P2Config()
        self.evaluator = self.checker = self.model = self.scene = None
        self.lock = Lock()
        self.gripper = None

    def _load(self, gripper):
        if self.checker is not None:
            if gripper != self.gripper:
                raise VLAError("GRIPPER_IDENTITY_MISMATCH", "P2 session gripper changed")
            return
        c = self.config
        for field in ("workspace_root", "model_config_path", "scene_path", "srdf_path", "backend_path"):
            if not getattr(c, field):
                raise VLAError("P2_RESOURCES_MISSING", "required configuration missing: " + field)
        root = Path(c.workspace_root).resolve()
        if not (root / "src/workspace_analysis/planning/validity.py").is_file():
            raise VLAError("P2_RESOURCES_MISSING", "workspace_root must identify the existing workspace_analysis directory")
        for field in ("model_config_path", "scene_path", "srdf_path", "backend_path"):
            if not Path(getattr(c, field)).is_file():
                raise VLAError("P2_RESOURCES_MISSING", "missing " + field)
        # Explicit source-root opt-in is needed because P2 is not a pip package.
        source = str(root / "src")
        if source not in sys.path:
            sys.path.insert(0, source)
        from workspace_analysis.robot.model import load_model
        from workspace_analysis.scene.schema import load_scene
        from workspace_analysis.collision.semantic import SemanticConfig
        from workspace_analysis.collision.evaluator import Evaluator
        from workspace_analysis.planning.validity import ConfigurationValidity
        import workspace_analysis
        if Path(workspace_analysis.__file__).resolve().parent != root / "src/workspace_analysis":
            raise VLAError("P2_IDENTITY_MISMATCH", "another workspace_analysis source is already imported")
        model = load_model(c.model_config_path)
        scene = load_scene(c.scene_path)
        semantic = SemanticConfig.load(c.srdf_path, model, allow_candidate=c.allow_candidate_semantics)
        workspace = None
        self.task_identity = None
        if c.workspace_enabled:
            if not c.task_path:
                raise VLAError("P2_RESOURCES_MISSING", "TCP workspace requires explicit task_path")
            from workspace_analysis.task.spec import load_task
            from workspace_analysis.task.targets import build_targets
            from workspace_analysis.planning.workspace_constraint import WorkspaceConstraint, TCPWorkspaceCheck
            task = load_task(c.task_path)
            build_targets(task, scene)
            workspace = TCPWorkspaceCheck(WorkspaceConstraint(), c.world_from_base, task.T_ee_tcp,
                                          {"source": "explicit_p3_configuration"})
            self.task_identity = task.sha256
        evaluator = Evaluator(model, scene, None, c.backend_path, semantic=semantic, gripper=gripper,
            clearance={"self_m": c.self_clearance_m, "environment_m": c.environment_clearance_m}, timeout=c.backend_timeout_s)
        try:
            checker = ConfigurationValidity(evaluator, workspace_check=workspace)
        except BaseException:
            evaluator.close()
            raise
        self.model, self.scene, self.evaluator, self.checker = model, scene, evaluator, checker
        self.gripper = gripper

    def validate(self, action, observation):
        from vla_protocol.observation import identity_fields
        base = dict(schema_version=1, source_action_id=action.action_id,
            starting_joints=observation.joints, starting_gripper=observation.gripper,
            scene_id=action.scene_id, scene_sha256=action.scene_sha256, model_id=action.model_id,
            validation_config=self.config.model_dump(), provenance={}, discrete_only=False)
        if identity_fields(action) != identity_fields(observation):
            return ValidatedTrajectory(**base, status="ERROR", trajectory=None, failure_reasons=["P2_IDENTITY_MISMATCH"])
        if self.config.mode == "disabled":
            return ValidatedTrajectory(**base, status="NOT_CHECKED", trajectory=action, failure_reasons=[])
        with self.lock:
            try:
                self._load(observation.gripper.position)
                model_id = "urdf:" + self.model.metadata()["expanded_urdf_sha256"]
                if (action.scene_id, action.scene_sha256, action.model_id) != (self.scene.scene_id, self.scene.canonical_scene_sha256, model_id):
                    raise VLAError("P2_IDENTITY_MISMATCH", "observation scene/model identity differs from loaded P2 context")
                waypoints = [list(observation.joints.positions), *[list(q) for q in action.values]]
                for q in waypoints:
                    self.model.validate_q(q)
                from workspace_analysis.planning.joint_path import sample_path
                states, _ = sample_path(waypoints, self.config.resolution_rad)
                if len(states) > self.config.max_path_samples:
                    raise VLAError("P2_SAMPLE_LIMIT", "candidate path exceeds configured sample budget")
                result = self.checker.path(waypoints, self.config.resolution_rad, fresh=True, stop_first=True,
                                           deadline=perf_counter() + self.config.max_validation_time_s)
                base["discrete_only"] = True
                base["provenance"] = {"identity": self.evaluator.identity, "task_sha256": self.task_identity,
                    "task_pose_checked": False, "workspace_checked": self.config.workspace_enabled,
                    "validation": result["validation"], "continuous_collision_proven": False,
                    "checked_samples": result["checked_samples"], "expected_samples": result["expected_samples"],
                    "first_failure": result["first_failure"], "query_statistics": result["query_statistics"],
                    "elapsed_s": result["elapsed_s"]}
                if result["valid"] is not True or result["complete"] is not True:
                    failure = result.get("first_failure") or {}
                    reasons = failure.get("reasons") or ["P2_UNRESOLVED_RESULT"]
                    return ValidatedTrajectory(**base, status="REJECTED", trajectory=None, failure_reasons=reasons)
                return ValidatedTrajectory(**base, status="PASS_RESEARCH_ONLY", trajectory=action, failure_reasons=[])
            except Exception as exc:
                reason = exc.code if isinstance(exc, VLAError) else "P2_VALIDATION_ERROR"
                base["provenance"] = {"error": str(exc), "exception_type": type(exc).__name__}
                return ValidatedTrajectory(**base, status="ERROR", trajectory=None, failure_reasons=[reason])

    def close(self):
        with self.lock:
            if self.evaluator is not None:
                self.evaluator.close()
                self.evaluator = self.checker = None
