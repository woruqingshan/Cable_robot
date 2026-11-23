# D2-D4 integration execution plan (2026-11-23)

This is one development run, with code checkpoints rather than separate task handovers.
Current maintenance source: b020afb; current maintenance PID9714; parent PID799.
Recheck all process and Git identities before changes. Existing Restart=no stays in force.

## Software and data boundaries

- D2 sends exactly one sequential query per ID2,3,4,5,10.
- Prior ID1 use is imported from the hashed D1 evidence; process replacement is not a clean epoch.
- D3 is at most three complete ID1,2,3,4,5,10 cycles, target1Hz.
  Only continue each cycle if the previous cycle had no error.
  No frequency increase, no retry, no endless collection.
- Repeated responses stay diagnostic/unknown. The protocol has no nonce and the
  running firmware inner association is unproven; numerically stable repeats
  do not prove reliable closed-loop feedback. Trusted KEY_REUSE_UNSAFE remains.
- A failure, unsolicited/duplicate/late reply or stale host batch latches a stop.
- Preserve every outer TX/RX, raw Pulse and host interval. Report mean/P95/max,
  empirical cadence and host six-channel span, not fabricated device sample rate.
- D4 uses the actually read controller parameter mapping (metadata only):
  joint1-5 init500, gripper ID10 init700; all directions min1000/max0.
  q=(init-pulse)*radians(240)/1000. This is a vendor configuration candidate,
  not physical zero/direction/internal-offset calibration.
- Reuse RobotStateProvider, RawFeedbackAdapter and RealObservationNode.
  /idc_vla/real/raw_feedback feeds only diagnostic conversion.
  /idc_vla/real/diagnostic_joint_states has unset acquisition stamp and is never
  forwarded into canonical Observation. No new HTTP or serial architecture.
- Existing URDF FK integration is implemented; urdf_parser_py/PyKDL are absent.
  Expose that exact dependency error, do not invent geometry or claim FK validation.
- Real RGB/inference are not enabled with unknown provenance/calibration/timing.

## One necessary confirmation: maintenance owner substitution

The user authorized controlled position experiments this round. The remaining
extra confirmation is stopping the existing ID1-only maintenance owner and
reopening the same port with the pinned finite-session package.

On approval:
1. Verify package manifest, current one-owner inventory and parent Restart=no;
   save backups/evidence. Original vendor files stay unchanged.
2. Stop only idc-r1-readback-b020afb.service via SIGTERM; no signal to the parent
   bringup and no normal-controller restore. Confirm zero serial openers.
3. Start one new versioned maintenance unit, Restart=no, physical domain0.
   No overlap of owners, no startup PWM/motor/Torque commands; no initialization launch.
4. Verify actual parameters, intended services, no actuator subscriptions and sole
   serial owner. Startup performs no read query.
5. Read-only RealObservationNode recording and a diagnostic subscriber can run
   alongside the driver; neither publishes actuator commands.
6. Execute D2 once. Save evidence and Git result checkpoint.
   On any communication failure stop here; no D3 request.
7. Execute D3 once only after verified successful D2 on the same runtime SHA.
   The three cycles are internally bounded and stop on errors. Save all results.
8. Verify D4 live private JointState/metadata, source separation and zero valid
   canonical Observations. Stop only the owned recorder after evidence is saved.
9. Commit final results. Keep maintenance mode and Restart=no; normal control
   restore and any further query budget require explicit follow-up.

## Effects and risks to confirm

Normal control is already disabled and stays disabled. The parent robot service
is not restarted. Serial close/reopen may reset the control board; actual
DTR/RTS/reset behavior and firmware identity remain unknown. Confirm the robot
is static and someone is supervising before substitution.

No physical motion, Torque, flashing, second driver, automatic retry, automatic
recovery or production/cloud deployment is included. If the new driver fails
to start, stop without reopening again or restoring an original initializer.

## 最新执行终点：USB控制面打开失败

切换已单独获批。旧维护进程退出后，新ede2020驱动在open期间的DTR ioctl Errno110失败。
未运行D2/D3，未自动重试或恢复。只读观察器已停止，当前无串口驱动/正常控制；
整车父服务仍运行且Restart=no。D4实机反馈/FK和R2不声称通过。
代码与证据保留，详见R1_D2_D4_RESULT_20261123.md。
