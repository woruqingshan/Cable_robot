# R2-A/B test dependency audit

Baseline: 641 cases, 305 original test functions. All original function ASTs/decorators/assertions are preserved after only resource-root normalization.

The explicit machine-readable declaration is `tests/dependencies.json`. Counts below are functions, not parametrized cases.

| Test module | Core | Integration | Artifact |
| --- | ---: | ---: | ---: |
| integration/test_saved_viewer_artifacts.py | 0 | 0 | 2 |
| test_architecture.py | 8 | 2 | 1 |
| test_candidate_baseline.py | 1 | 7 | 1 |
| test_cartesian_approach.py | 2 | 9 | 2 |
| test_collision_client.py | 2 | 5 | 0 |
| test_collision_policy.py | 0 | 3 | 1 |
| test_collision_protocol.py | 4 | 1 | 1 |
| test_collision_visualization.py | 1 | 0 | 3 |
| test_constrained_evaluator.py | 0 | 0 | 7 |
| test_geometry_profile.py | 2 | 6 | 2 |
| test_idc_prototype.py | 6 | 0 | 1 |
| test_model_fk.py | 4 | 12 | 0 |
| test_path_planning.py | 12 | 0 | 1 |
| test_repro_doctor.py | 7 | 0 | 0 |
| test_repro_groups.py | 1 | 0 | 1 |
| test_research_workspace.py | 4 | 2 | 2 |
| test_sampling_results.py | 6 | 22 | 0 |
| test_scene_model.py | 17 | 0 | 0 |
| test_scene_visualization.py | 11 | 0 | 5 |
| test_semantic.py | 3 | 6 | 1 |
| test_task_ik.py | 0 | 8 | 2 |
| test_task_scope.py | 0 | 6 | 2 |
| test_task_targets.py | 6 | 0 | 0 |
| test_tf_validation.py | 34 | 0 | 0 |
| test_transforms.py | 11 | 0 | 0 |
| test_viewer_replay.py | 6 | 0 | 0 |
| test_visualization.py | 13 | 0 | 6 |
| test_workspace_validation.py | 9 | 13 | 2 |

## Declared external artifact inputs

- `integration/test_saved_viewer_artifacts.py::test_saved_t3_replay_keeps_endpoint_only_semantics_and_mesh_updates`: `task_planning/p22_20261104_02/figures/q0_linear.html`. Requirements: Python/JS only.
- `integration/test_saved_viewer_artifacts.py::test_saved_t4_native_data_all_26_frames_slider_and_play`: `task_approach/p22_20261104_03/figures/q0_approach.html`, `task_approach/p22_20261104_03/figures/q1_approach.html`. Requirements: Python/JS only.
- `test_architecture.py::test_pure_scene_load_and_existing_report_read_work_without_robot_or_plotly`: `g4_run01`. Requirements: Python/JS only.
- `test_candidate_baseline.py::test_real_native_three_semantics_known_cases_and_geometry_unchanged`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_cartesian_approach.py::test_complete_continuity_failure_experiment_is_reproducible_and_verifiable`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`, `task_reachability/p22_20261104_02/ik_results.json`, `task_reachability/p22_20261104_02`, `task_planning/p22_20261104_02`. Requirements: model, native.
- `test_cartesian_approach.py::test_real_both_branches_and_native_collision_failure_keep_complete_replay`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`, `task_reachability/p22_20261104_02/ik_results.json`, `task_reachability/p22_20261104_02`, `task_planning/p22_20261104_02`. Requirements: model, native.
- `test_collision_policy.py::test_real_policy_switch_baseline_restore_env_and_gripper`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_collision_protocol.py::test_init_failure_retains_all_100_indices_and_refuses_overwrite`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_collision_visualization.py::test_all_robot_surfaces_poses_distinct_and_cabinet_fixed`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_collision_visualization.py::test_offline_pair_selector_and_export_refuses_overwrite`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_collision_visualization.py::test_real_contacts_and_pairs_correspond_to_displayed_objects`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_constrained_evaluator.py::test_batch_size_order_consistency_clearance_and_scope`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_constrained_evaluator.py::test_directed_native_motion_mimic_and_fixed_assembly`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_constrained_evaluator.py::test_invalid_threshold_rejected_before_backend`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_constrained_evaluator.py::test_pilot_no_overwrite_and_configuration_npz_roundtrip`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_constrained_evaluator.py::test_positive_clearance_model_scope_remains_physically_unverified`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_constrained_evaluator.py::test_thousand_source_rows_preserved`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`, `g4_run01/experiments/workspace_n1000_seed42.npz`, `g4_run01/experiments/workspace_n1000_seed42.json`. Requirements: model, native.
- `test_constrained_evaluator.py::test_v1_only_adds_two_fixed_pairs_and_preserves_unresolved`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_geometry_profile.py::test_d1_paired_experiment_preserves_arrays_identity_and_robot_only`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: native, model.
- `test_geometry_profile.py::test_jetrover_profiles_fk_mimic_environment_and_non_target_invariance`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_idc_prototype.py::test_existing_100_points_overlay_without_resampling`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_path_planning.py::test_real_native_semantics_limits_clearance_environment_and_full_mesh_replay`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`, `task_reachability/p22_20261104_02/ik_results.json`. Requirements: model, native.
- `test_repro_groups.py::test_saved_viewers_can_use_an_independent_external_artifact_root`: `task_approach/p22_20261104_03/figures/q0_approach.html`, `task_approach/p22_20261104_03/figures/q1_approach.html`, `task_planning/p22_20261104_02/figures/q0_linear.html`. Requirements: Python/JS only.
- `test_research_workspace.py::test_existing_output_and_large_dataset_are_rejected_before_evaluation`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`, `workspace_n10000_seed42.npz`, `workspace_n10000_seed42.json`. Requirements: model, native, vendor.
- `test_research_workspace.py::test_real_evaluator_semantic_stats_use_new_fields_not_legacy_raw`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_scene_visualization.py::test_cli_100_point_overlay_preserves_yaml_npz_json_and_errors_nonzero`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_scene_visualization.py::test_corrupt_or_incompatible_g2_inputs_rejected`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_scene_visualization.py::test_display_limit_keeps_deterministic_original_indices`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_scene_visualization.py::test_reference_frame_mismatch_is_rejected`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_scene_visualization.py::test_workspace_uses_g3_loader_without_resampling_and_preserves_all_rows`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_semantic.py::test_jetrover_semantics_preserves_raw_state_fk_mimic_and_known_rejections`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_task_ik.py::test_complete_failure_run_keeps_best_viewer_and_skips_task_collision_screening`: `g4_run01/experiments/workspace_n1000_seed42.npz`, `g4_run01/experiments/workspace_n1000_seed42.json`. Requirements: model, native.
- `test_task_ik.py::test_seeds_use_tcp_positions_and_record_original_g4_indices_deterministically`: `g4_run01/experiments/workspace_n1000_seed42.npz`, `g4_run01/experiments/workspace_n1000_seed42.json`. Requirements: model.
- `test_task_scope.py::test_d2_original_arrays_statistical_conservation_and_no_overwrite`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: native, model.
- `test_task_scope.py::test_jetrover_scope_topology_gripper_scans_and_real_rejections`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model, native.
- `test_visualization.py::test_cli_missing_plotly_does_not_claim_pass`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_visualization.py::test_g2_file_validation_errors_stop_before_plotting`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_visualization.py::test_g2_loader_is_reused_with_model_and_real_100_row_pair`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_visualization.py::test_loading_does_not_change_source_npz_json`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_visualization.py::test_model_mismatch_is_rejected`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_visualization.py::test_real_cli_keeps_source_npz_json_unchanged`: `workspace_n100_seed42.npz`, `workspace_n100_seed42.json`. Requirements: model.
- `test_workspace_validation.py::test_cli_small_diagnostic_from_other_cwd_and_error_exit`: `g1a_fk.json`, `g1b_tf_v3.json`. Requirements: Python/JS only.
- `test_workspace_validation.py::test_prior_evidence_is_cited_without_tf_rerun`: `g1a_fk.json`, `g1b_tf_v3.json`. Requirements: model.

## Environment / path findings

- Official model tests use the existing ament package index and external official description. They are Integration unless they also consume saved artifacts.
- Native is selected by `--native-backend` / `C1_BACKEND`, never auto-discovered inside history. Synthetic Native geometry tests still require an actual executable.
- Vendor DH/controller evidence uses `--system-source` / `JETROVER_SYSTEM_SOURCE`, preserving all mechanical assertions.
- G2/G4 and T2/T3/T4 inputs use `--artifact-root` / `WORKSPACE_ARTIFACT_ROOT`. Legacy G3 input overrides remain test-specific; prefer the unified root.
- Shared legacy fixtures that read G2 even for small policy assertions remain explicitly Artifact; no fixture is silently replaced with invented data.
- Real saved HTML tests moved to `tests/integration/test_saved_viewer_artifacts.py`; Core retains all synthetic JS controls and adds a synthetic T3 mode case.
- Core guard rejects history/official model access, and the isolated runner also guards original history in normal Python subprocesses. Isolated-mode subprocesses ignore PYTHONPATH but receive only copied source and no history.
- Model assets and toolchain/environment snapshots are observational metadata, not automatic installation, artifact recovery or hardware validation.
