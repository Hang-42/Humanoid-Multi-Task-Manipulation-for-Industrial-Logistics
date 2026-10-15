# PI05 kd12 deployment verification — 2026-10-15

## Result and scope

Latest trial: the opt-in synchronized-grasp experiment at 23:27 stopped on its
conservative right-hand position-completion gate, requested arm hold successfully,
and did not execute a post-close lift. The onsite operator saw opposing-finger
contact around the package, but lifting/retention was not tested. The completed
120-second run described first below is an earlier experiment, not this latest one.

The 120-second live deployment completed with exit code 0. This verifies the
deployment/control loop, not successful execution of the package task.

The onsite operator reported that the left hand repeatedly attempted grasping
and did not reach the handover. Logs agree: no completed chunk had a right-thumb
target of 50 or more. Therefore the previously reported post-handover arm-tracking
failure was not reproduced or explained by this run.

## Verified run

```bash
./src/deploy_pi05.sh --pi05-live --model kd12 --steps 10 --duration 120
```

- Robot: v53_qi / Revo1; profile hash
  `932e835673c5c0f50207b521b7b03cf14e6aad628cdd0503fd0f93a3f5a0a7fc`.
- Boot: `336cb333-14e1-48a9-be64-f055b96a9d49`.
- Lower release: `/home/lab/kuavo_pi05_lower/releases/d09ec95dd580bcf6/lower`.
- Both hands: verified protection currents `[600, 500, 600, 600, 600, 600]` mA.
  These are protection thresholds, not measured grip force.
- 88 completed execution windows, 871 model steps; the last window was shortened
  by the requested time limit.
- Largest logged arm endpoint error: 7.9662 degrees. This is an endpoint metric,
  not the maximum tracking error at every intermediate instant.
- No `ARM_TRACKING_LIMIT` or `LIVE_RUN_STOPPED` event.
- Final events: `LIVE_RUN_COMPLETED`, `ARM_HOLD_REQUESTED` with `accepted=true`,
  and `VLA_HOLD_MARKER_SAVED`.
- 334 `HAND_TRACKING_WARNING` events remain. The pre-existing warning-only
  behavior was retained; hand tracking/task quality is not certified by this test.
- Full log on Qi, relative to its project root:
  `pi05/envs/logs/diagnostic-kd12-steps10-fixed-20261015-221847.typescript`.

## Fix and regression checks

The profile-based planner can return integral-valued floats, including retained
AUX channels. Direct assignment to `robotHandPosition` fails serialization because
its hand arrays require unsigned integers.

`robot_bridge.hand_position_command` validates twelve finite values in 0..100,
then rounds to built-in Python integers. All hand samples are validated before
any arm trajectory is published. Invalid samples are rejected rather than clipped;
existing finger caps and retained channels remain unchanged.

Local and Qi unit runs: 15 tests, one ROS-only test skipped where kuavo_msgs is
unavailable. On the lower computer, all three hand-message tests passed using
actual kuavo_msgs, including in-memory ROS serialization. These tests do not
create a ROS node or publish commands.

## Remaining work

Investigate why the policy repeats the left-hand phase before attempting another
handover test. Distinguish actual grasp/scale placement from empty or failed
grasping using onsite observation and synchronized images/state. Do not disable
arm tracking protection or claim the original handover fault has been resolved.

No model weights were changed and no Git commit or push was performed.

## Follow-up: recovery and reproduced right-arm tracking fault

After onsite clearance/empty-hand confirmation, the managed recovery completed
with maximum arm error 1.9042 degrees, hand targets unchanged and arm hold mode
confirmed. A second 120-second-limited run used the same kd12/steps=10/600 mA
settings, but stopped early after 33 completed chunks. Five completed chunks
included a right-hand closing target, so this run did reach the right-hand phase.

The original arm-tracking protection was reproduced without changing its limits:

- Joint: `zarm_r4_joint` (right arm joint 4).
- Target: -67.2571 degrees; actual: -77.5476 degrees.
- Absolute error: 10.2905 degrees, over-limit interval 0.2030 seconds.
- Current trajectory duration: 1.0 second; controller elapsed time: 1.054 seconds.
- Right-hand command: `[100,97,100,100,100,100]`; feedback at fault:
  `[51,96,40,40,40,40]`. This is a hand-tracking gap, not evidence by itself of
  a dexterous-hand current-protection trip.
- `LIVE_RUN_STOPPED` was followed by `ARM_HOLD_REQUESTED accepted=true`.
  Unlike the normal completed run, this failed run has no success hold marker.

Offline analysis of the already-closed lower-computer bag
`/root/.ros/2026-10-15-22-25-47_36.bag` captured the fault timestamp
1792074378.6138. In the extracted fault window there were no nonzero body-motor
error-code messages. Right-arm joint 4 was moving, but behind the target; its
velocity near the fault was approximately 10.34 degrees/second.

The controller acknowledgement exposed a cross-window trajectory discontinuity:
its initial right-joint-4 reference was the previous target, -90.4819 degrees,
but the next first waypoint was -84.7977 degrees at +0.1 second. That reference
segment implies approximately 56.84 degrees/second, although subsequent planner
increments are limited to 2 degrees per 0.1 second. The planner reanchors each
window to measured feedback while the timed controller begins at its previous
reference; limiting only the new window's measured-origin increments does not
bound the acknowledged boundary segment. Continuous following lag also grew
during the subsequent approximately 20-degree/second target motion.

Recommended next change: enforce trajectory continuity/speed limits against the
acknowledged controller reference, handle direction changes with bounded
acceleration, and retest at a lower arm speed. Retain the 10-degree/0.2-second
tracking stop; do not change protection current or disable the guard to hide the
fault. The exact physical contribution of payload/contact/controller filtering
still needs investigation. No such tuning or additional motion was performed.

Evidence:

- Qi: `pi05/envs/logs/kd12-after-recover-20261015-222716.typescript`.
- Lower: `initialize/logs/336cb333-14e1-48a9-be64-f055b96a9d49/live-arm-tracking-1792074378613949536.json`.
- Copies are preserved in the local task workspace's
  `tmp/revo1-force-sdk-20261015` audit directory.

## Implemented continuity fix (initial offline verification)

The user authorized the continuity/lower-speed change. The bridge now requires
the timed controller's fresh one-point mode-switch receipt before its first arm
trajectory; a missing/invalid receipt fails closed without guessing from sensors.
Every subsequent plan starts from the previous fully acknowledged endpoint.
Both bounded and retimed planners apply a 10-degree/second execution ceiling
(or the profile limit if lower), including the first segment. The hardware/model
profile itself was not edited, so its hash and existing initialization identity
remain unchanged.

Every received execution acknowledgement is checked against all planned points
and times, including its controller-selected initial state and segment speeds.
Mismatch is logged as `ARM_ACK_REJECTED`; the bridge raises and its existing
finally block requests hold. It does not resend. Tracking now compares measured
positions against the validated actual acknowledgement, not a reconstructed
trajectory with a measured-feedback initial point. Existing 10-degree/0.2-second
tracking protection and hand-command logic remain in place.

- Installed lower release: `/home/lab/kuavo_pi05_lower/releases/34dc62e72d0bf0c2/lower`.
- Local and Qi: 25 tests pass with one ROS serialization test skipped where ROS
  messages are unavailable. Lower computer: all 25 tests pass with actual messages.
- Tests cover the recorded -90.4819/-86.7977 reference/measurement discrepancy,
  both planners, two-window direction changes, bad references, full-ack mismatches,
  missing/stale mode-switch receipts, no resend, and integer hand serialization.
- These tests use fake in-memory publishers and perform no real ROS service call
  or motion publication. Syntax and whitespace checks also pass.
- Diagnostics: `ARM_REFERENCE_INITIALIZED` and execution planning logs include
  the controller reference and actual speed ceiling.
- At this checkpoint, no live inference/recovery had been started after the code
  change. The subsequent battery-swap initialization and live verification are
  recorded below. No model-weight change, Git commit or push was performed.

At that checkpoint, before a new live test, inspect/clear the scene and recover the arms through the
managed recovery flow; the prior fault left an interrupted-run marker. Do not
remove that marker or use `--resume-current` to bypass onsite recovery checks.

## Live verification after battery swap: continuity fix

The onsite operator confirmed full suspension, a cleared workspace and emergency
stop supervision, then explicitly confirmed that the robot had been lowered for
standing. The managed initialization completed on the new boot, followed by one
bounded 120-second deployment test. No interrupted-run marker was bypassed.

- Boot: `a78a9092-1a15-4569-993f-abeef6422fa5`.
- Initialization result: `POSE_RESTORED`; stand and arm/hand initialization
  completed. This initialization result is historical, not a claim that the arms
  remained in their initialization pose after inference.
- Current-boot hand setup was verified before the full-body controller started:
  both arrays `[600,500,600,600,600,600]` mA. AUX remains 500 mA. These are
  protection settings, not measured gripping force; no exclusive CAN test was
  performed while the full-body controller was running.
- Command: `./src/deploy_pi05.sh --pi05-live --model kd12 --steps 10 --duration 120`.
- Lower release: `/home/lab/kuavo_pi05_lower/releases/34dc62e72d0bf0c2/lower`.
- Actual execution ceiling: 10 degrees/second; the unchanged profile hash is
  `932e835673c5c0f50207b521b7b03cf14e6aad628cdd0503fd0f93a3f5a0a7fc`.
- `ARM_REFERENCE_INITIALIZED` used `fresh_mode_switch_ack`; all 88 execution
  windows logged `acknowledged_controller_endpoint` as their reference source.
- 88 completed windows / 880 model steps. Twenty-four completed windows had
  a right-thumb target of at least 50, so this run included right-hand closing
  commands. This does not establish an actual successful grasp or handover.
- Largest logged arm endpoint error: 8.6380 degrees. This endpoint statistic
  does not bound every intermediate tracking error.
- No `ARM_ACK_REJECTED`, `ARM_TRACKING_LIMIT` or `LIVE_RUN_STOPPED` event.
  The existing 10-degree/0.2-second tracking stop was retained.
- Normal ending: `LIVE_RUN_COMPLETED`, `ARM_HOLD_REQUESTED accepted=true`,
  arms settled, then `VLA_HOLD_MARKER_SAVED`. Script exit code 0 at
  2026-10-15 23:08:03 +08:00.
- The saved current-boot marker reports `HOLD_CONFIRMED`; the full-body controller
  was still running when checked after the run. No additional motion was started.
- 349 `HAND_TRACKING_WARNING` events remain, with a largest sampled target/feedback
  gap of 100 command units. Their existing warning-only behavior was not changed.
  Such gaps can include transient opening/closing lag or contact; these events
  alone do not prove either a current-protection trip or sufficient grip force.

This run verifies normal completion of the deployment loop, including right-hand
closing commands, with the continuity/speed fix. It does not yet establish the
package-task success rate or eliminate all causes of hand tracking/grasp failure.
The onsite operator confirmed that the robot entered the right-hand phase but
lifted the arm before securing the package. This is a completed deployment test,
not a successful package-task trial.

In all 24 completed windows with right-thumb targets of at least 50, the
right-index target was 77..100 but its endpoint feedback was only 2..59. For
example, windows 26..29 retained feedback near thumb 76 / index 59 while the
other three gripping fingers were near 99. This is evidence of a position
target/feedback gap, not a force measurement. Contact with the package, motion
lag and mechanical/current constraints cannot be distinguished from these
positions alone; do not infer a protection trip or raise current on this basis.
No further motion or parameter change was performed after this onsite report.

Evidence:

- Qi terminal log: `pi05/envs/logs/kd12-continuity-10deg-20261015-230510.typescript`.
- Lower hold marker:
  `initialize/logs/a78a9092-1a15-4569-993f-abeef6422fa5/pi05_last_hold.json`.
- Copies in the local `tmp/revo1-force-sdk-20261015` audit directory:
  `kd12-continuity-10deg-20261015-230510.typescript` and
  `pi05_last_hold-a78a9092.json`.

No model weights, body-motor safety limits or hand-tracking stop behavior were
changed during this live verification. No Git commit or push was performed.

## Opt-in synchronized-grasp experiment, 23:27

The operator authorized code changes, empty-hand recovery and one bounded live
test after confirming stable standing, cleared paths, package-drop support and
emergency-stop supervision. Managed recovery passed its existing geometry checks
and completed with maximum endpoint error 2.5941 degrees; hands were not commanded.

The new `--sync-grasp` mode preserves model arm goals and synchronizes hand-target
changes to retimed model-pose endpoints. It splits before the first right closing
edge; a stronger close in an already-closing episode holds the current arm
reference instead of executing a later lift pose. No remaining post-close arm
trajectory is submitted while closure feedback is pending. The mode is opt-in,
Revo1-only and incompatible with retime/single-step/approach-only modes.

- Wait for measured arms within 5 degrees of the reference and below 5 degrees/s
  for 0.3 seconds, without changing the existing tracking guard.
- Send the model's exact capped right-hand target, not a forced full close.
- Require fresh right-finger positions within 10 units of target, stable within
  2 units for 0.4 seconds, and at least 0.5 seconds since command publication.
- A 3-second timeout saves fresh head/left-wrist/right-wrist images and stops.
- Position completion is explicitly logged as `grasp_verified=false`; it is not
  a contact/force/object-retention detector. Fresh inference follows a successful
  wait, not the discarded remainder of the old prediction.

Command tested:

```bash
./src/deploy_pi05.sh --pi05-live --model kd12 --steps 10 --sync-grasp --duration 120
```

Results for live release `/home/lab/kuavo_pi05_lower/releases/bf8425d173183731/lower`:

- Current boot/profile/current settings unchanged from the preceding run.
- Before motion: local/Qi 40 tests, one ROS-only skip; all 40 passed on the lower
  computer with actual ROS messages. Tests use in-memory fake publishers only.
- Four completed playback windows, 24 model frames / 87 controller samples.
  Largest logged arm endpoint error: 6.3061 degrees. No arm-tracking-limit or
  acknowledgement-rejection event occurred.
- The model first requested right-finger positions 51; feedback reached
  `[51,50,50,50,50]` (thumb/index/middle/ring/pinky). Position completion was
  accepted after 1.0580 seconds; a new observation/prediction was obtained.
- The new model request increased those targets to 100. The bridge held the
  same arm reference instead of playing the new arm movement. After 3.0199
  seconds, feedback was `[90,50,51,99,50]`, so `GRASP_WAIT_TIMEOUT` stopped the run.
  This is a position-completion timeout, not evidence of a current-protection trip.
- `post_close_motion_sent=false`, then `LIVE_RUN_STOPPED` and
  `ARM_HOLD_REQUESTED accepted=true`; exit code 1. The run ended early, not at
  the 120-second normal limit. No successful-completion hold marker was written.
- The session marker remains `ACTIVE` intentionally after the interrupted run.
  Do not delete it or bypass it with resume-current. The body controller remained
  running; the bridge exited and did not automatically open the hand or retry.
- Head/right-wrist images show the black package at the scale and hand/package
  contact/deformation. The onsite operator confirmed opposing-finger contact
  around the package, but no lift was performed. Therefore retention/force and
  successful transfer are still unverified. The strict position gate may reject
  normal object contact; do not treat this timeout as proof of failed grasp.

After the live run, an additional offline regression covered slow cumulative
finger motion: individually small increments must not be accepted as stability.
The source gate was strengthened to bound the total position range during the
stable interval, rather than only consecutive-sample differences. Final local/Qi
regression: 41 tests, one ROS-only skip; lower computer: all 41 pass with actual
ROS messages. This stricter source revision was synchronized to Qi but was not
installed over the immutable live-tested release or live retested. No additional
robot motion was started after the timeout.

Evidence on Qi:

- `pi05/envs/logs/kd12-sync-grasp-20261015-232646.typescript`.
- Lower current-boot image directories: `grasp_evidence_1792077919765474828`,
  `grasp_evidence_1792077920830735453`, `grasp_evidence_1792077921619869359`,
  `grasp_evidence_1792077924660052119`.
- Terminal log and all four three-camera evidence bundles were copied to the
  local `tmp/revo1-force-sdk-20261015` audit directory.

Next work requires a defensible contact/holding criterion and separately
authorized, bounded retention testing. Do not simply raise protection current,
disable the arm guard, require a contacted hand to reach 100, or relax the new
gate to force the task to continue. No Git commit or push was performed.
