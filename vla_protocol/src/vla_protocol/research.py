"""Explicit inspection input; never a canonical Observation or controller input."""
from typing import Literal
from pydantic import Field, model_validator
from .profiles import StrictModel
from .observation import Identity, JointPositions, GripperState, ImagePayload

class ResearchSource(StrictModel):
    profile_id: Literal["real_research_inspection_v1"] = "real_research_inspection_v1"

class ResearchObservation(Identity):
    source_profile: ResearchSource = Field(default_factory=ResearchSource)
    capture_mode: Literal["live", "recorded_replay"] = "live"
    replay_reference: str | None = None
    simulated: Literal[False] = False
    physical_calibrated: Literal[False] = False
    execution_enabled: Literal[False] = False
    canonical_observation_eligible: Literal[False] = False
    instruction: str = Field(min_length=1,max_length=2048)
    source_profile: ResearchSource = Field(default_factory=ResearchSource)
    simulated: Literal[False] = False
    physical_calibrated: Literal[False] = False
    canonical_observation_eligible: Literal[False] = False
    execution_allowed: Literal[False] = False
    joints: JointPositions
    gripper: GripperState
    images: list[ImagePayload] = Field(min_length=1,max_length=1)
    feedback_report: dict
    conversion_evidence: dict
    timing_evidence: dict
    capabilities: list[str] = Field(default_factory=lambda:["native_actions_v1","source_profiles_v1","session_state_v1"])

    @model_validator(mode="after")
    def inspection_only(self):
        if self.capture_mode=="recorded_replay" and not self.replay_reference:raise ValueError("REPLAY_REFERENCE_REQUIRED")
        r=self.feedback_report;c=r.get("channels",[])
        if (not r.get("complete") or r.get("source_type")!="unknown" or r.get("canonical_observation_eligible") is not False
            or [x.get("servo_id") for x in c]!=[1,2,3,4,5,10]
            or any(x.get("error") or not x.get("outer_frame_validated") or x.get("source_type")!="unknown" for x in c)):
            raise ValueError("COMPLETE_RAW_DIAGNOSTIC_REQUIRED")
        if self.gripper.simulated or self.gripper.calibrated:raise ValueError("UNCALIBRATED_REAL_GRIPPER_REQUIRED")
        if self.conversion_evidence.get("physical_calibrated") is not False:raise ValueError("NOMINAL_CONVERSION_REQUIRED")
        if self.joints.positions+self.gripper_positions()!=self.conversion_evidence.get("positions"):raise ValueError("CONVERSION_VALUE_MISMATCH")
        if [x["raw_pulse"] for x in c]!=self.conversion_evidence.get("raw_pulses"):raise ValueError("RAW_VALUE_MISMATCH")
        for name in ("feedback_receive_age_s","image_receive_age_s"):
            v=self.timing_evidence.get(name)
            if type(v) not in (float,int) or not 0<=v<=.5:raise ValueError("STALE_INSPECTION_INPUT")
        return self

    def gripper_positions(self):return [self.gripper.position]
