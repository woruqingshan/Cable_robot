# R1 review-only vendor patch candidate

This bundle is NOT installed and is NOT a continuous-feedback implementation.
The production observation node does not import this helper. The live vendor
files and services must remain unchanged until separate explicit authorization.

- guarded_readback.py: standard-library-only state machine, no serial driver.
- build_candidate_patch.py: reads exact audited vendor files and writes only this bundle.
- sdk_bounded_readback_candidate.patch: SDK, service wrapper, and support module candidate.
- baseline.json: source fingerprints and generated patch fingerprint.

## What this candidate can and cannot establish

The audited reply body has ID, command, result and pulse, without a transaction
nonce. ID/command checks alone cannot distinguish a delayed duplicate of the
same request key. The helper therefore refuses reuse of every (ID, command)
key even after success, and latches faults after uncertainty. It has no reset
or automatic rearm path. At most one sequential six-ID snapshot is contemplated
per independently established epoch.

The default is DISARMED. arm() only accepts external assertions/reference; it
does NOT prove a clean UART epoch, exclusive ownership, correct firmware echo
semantics, bounded serial writes, or physical safety. Tests use clearly labeled
OFFLINE_FIXTURE_ONLY assertions. Do not copy these assertions into a robot run.

The existing SDK does not configure serial write_timeout. Even an armed
candidate refuses to transmit if this bound is absent. This bundle does not
change the shared port settings or existing actuator command bytes. A reviewed
transport/firmware guarantee is still needed before any live application.

The existing PWM read path is outside this patch and is also unbounded; it
shares servo_read_lock. Whole-driver availability is NOT certified here.

## Reproduce offline checks on JetRover

Run from the Cable_robot root, with the repository root on PYTHONPATH:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 patches/jetrover_readback/build_candidate_patch.py \
  --vendor-directory /home/ubuntu/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller

PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD:$PWD/cable_robot_vla:$PWD/vla_protocol/src" \
  python3 -m pytest -q -p no:cacheprovider \
  cable_robot_vla/test/test_r1_readback_guard.py \
  cable_robot_vla/test/test_r1_vendor_patch.py
```

Tests apply/reverse the patch exclusively in pytest temporary copies, replace
serial.Serial with a constructor that raises, and use FakePort for all bytes.
They do not instantiate Board on hardware, call ROS services, restart drivers,
or perform motion. Tests requiring exact vendor source files skip off-device;
the pure guard tests remain independent of ROS and serial.

No live installer, arming service, reset service or automatic recovery is
provided. Application, package rebuild, supervised arming and rollback are
separate future maintenance actions. Read the full constraints and remaining
blockers in docs/real_operation/SERVO_READBACK_DESIGN.md before considering them.
