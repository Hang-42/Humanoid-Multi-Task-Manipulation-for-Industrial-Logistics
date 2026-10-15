# Right-hand closure wait revision — 2026-10-16

## Scope and outcome

Code and README were updated on Qi and the local workspace. A new immutable
lower release was installed at:

`/home/lab/kuavo_pi05_lower/releases/1129601bb8bf9162/lower`

This request involved code changes and offline verification only. No arm/hand
motion, live inference, recovery, controller restart, current-setting write,
model-weight change, Git commit or push was performed. The new criterion has
not been live validated and does not certify package retention/task success.

## Original failure

Release `5565a8dce9db0af6` required every right gripping finger to settle within
10 position units of its model target before continuing. A contacted package
can prevent reaching an empty-hand closure target, so this rejects some valid
contact configurations. Current-boot saved feedback included
`[61,53,68,97,97]` for thumb/index/middle/ring/pinky. Position feedback alone
cannot distinguish package contact, a mechanical limit or drive constraints.

The reported exception is the software closure-position timeout, not proof of
a dexterous-hand peak-current trip or the separate arm-tracking guard.

## Revised behavior (only with --sync-grasp)

Model hand targets are unchanged. Arms still stop at the closing pose; no old
post-close arm trajectory is submitted while the bounded wait is pending.

The wait now requires at least 1.0 second since the hand command, fresh feedback,
at least three new samples and a total position range of at most 2 units over
0.4 seconds. It then distinguishes:

- `target_reached`: every right gripping finger is within 10 units of target.
- `closure_motion_settled`: the thumb and at least two opposing fingers have
  each demonstrated at least 8 units of closing progress during the current
  closure episode, and no finger exceeds its target by more than 10 units.
  A stable off-target position is allowed, but is NOT called contact or grasp.

An increased model closing target retains only the same episode's initial
feedback reference. Reopening clears it, including reopening within a prediction
prefix. A new episode cannot inherit old progress to excuse a nonresponsive hand.

Feedback gaps over 0.25 seconds reset the stability interval. A finger expected
to close that reverses more than 5 units from its observed peak is rejected.
Stale, invalid or nonfinite data are rejected. No qualifying completion within
3 seconds, or reaching the overall run deadline, saves evidence and stops; a
completion arriving at the deadline cannot override the timeout.

After either completion, the old prediction remainder is discarded. All three
camera images, joint state and hand state must have receipt times after the
wait ended before the next observation is sent. This has a separate one-second
deadline and runs the existing guards throughout. Fresh inference then decides
the next action; the wait itself never directly emits a lift command.

Logs always retain `contact_verified=false` and `grasp_verified=false`. This is
motion-wait completion, not a grasp sensor. Empty closing and mechanical stops
remain possible; actual retention/contact must be evaluated independently.

## Verification

- Local and Qi: 54 tests, with one actual-ROS serialization test skipped where
  kuavo_msgs is unavailable.
- Lower computer: all 54 tests pass using actual ROS messages; fake in-memory
  publishers only, no ROS node or control service/action publication.
- Regressions include recorded package plateaus, no measured motion, insufficient
  opposing-finger progress, overshoot/reversal, cumulative slow movement, duplicate
  or missing feedback, episode reset/retention, duration deadlines, no replay of
  old lift frames, and post-wait observation freshness/fail-closed timeout.
- Syntax and whitespace checks passed. README explains the new conditions and
  limitations; the CLI remains `--sync-grasp`.

Core source hashes:

```text
robot_bridge.py  9482a1d5358f14f5fe20e8f3a1af5881f033a459d4a16c0a8df0c1794396fe58
grasp_sync.py    8eb3904920dc5a68330d1aed3a56895bd4cb209cb2e23db0e40f84663635a45a
test_grasp_sync.py 85a3ab1bd8c10d9d585d4739bea753fc3d546f9da489db458f16d37c804a49cd
```

Offline tests on the lower computer are preserved outside the immutable runtime
at `/home/lab/kuavo_pi05_lower/offline_tests/1129601bb8bf9162`.

Before another live run, inspect the scene, use the managed safe recovery flow
if necessary, and confirm the current boot has no motor fault. Do not clear an
interrupted-run marker or weaken recovery geometry checks to force resumption.
