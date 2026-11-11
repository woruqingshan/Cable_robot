"""Explicit semantic decision for downstream consumers; no legacy field inference."""
from .errors import CollisionError


def semantic_configuration_decision(valid,review_status):
    """Classify under an explicit model; no research candidate is planning approval."""
    if valid is not None and type(valid) is not bool:raise CollisionError('semantic validity must be bool or None')
    if review_status not in {'model_only_partial','candidate_baseline_unverified'}:raise CollisionError('unsupported semantic decision review status')
    status=('UNVERIFIED' if valid is None else 'REJECTED' if valid is False else
            'RESEARCH_CANDIDATE' if review_status=='candidate_baseline_unverified' else 'VALID_MODEL_ONLY')
    return {'schema_version':1,'basis':'independent_SRDF_FCL_and_clearance','status':status,
            'model_configuration_valid':valid,'semantic_review_status':review_status,
            'usage':'research_only' if review_status=='candidate_baseline_unverified' else 'model_only_partial',
            'planning_approved':False,'hardware_validated':False,
            'authoritative_fields':['semantic_self_collision','environment_collision','semantic_clearance'],
            'legacy_fields_do_not_drive_this_decision':['final_decision','model_scope_collision_free']}
