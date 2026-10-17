"""Opt-in synchronized playback and bilateral closure-motion waiting.

Neither position completion nor motion settling proves contact or a grasp.
No ROS calls are made by this module.
"""
import numpy as np

from action_safety import (ARM_COLUMNS, ARM_JOINT_NAMES, bounded_plan, arm_command_limits_deg,
                           arm_speed_limit_deg_s, ROBOT_PROFILE, HAND_BY_SIDE)

SIDE_NAMES = ('left', 'right')
GRIP_INDICES = HAND_BY_SIDE
RIGHT_GRIP_INDICES = GRIP_INDICES[1]
GRASP_TIMEOUT_SECONDS = 3.0
GRASP_MIN_WAIT_SECONDS = 1.0
GRASP_STABLE_SECONDS = .4
GRASP_POSITION_TOLERANCE = 10.
GRASP_PROGRESS_UNITS = 8.
GRASP_MAX_REVERSE_UNITS = 5.
GRASP_MAX_SAMPLE_GAP_SECONDS = .25
MAX_SYNC_SAMPLES = 50  # At most five seconds before fresh inference.
GRASP_TARGET_DEADBAND_UNITS = 5


def grasp_execution_window(actions, steps, lookahead=0, armed_sides=(True, True)):
    """Optionally retain a nearby MODEL close, never synthesize a close.

    An explicit lookahead extends only to the first new closing edge, by at
    most ten model frames. Playback still splits there and obeys its own
    five-second budget. With lookahead=0 the requested prefix is unchanged.
    """
    a = np.asarray(actions, dtype=float)
    if (a.shape != (50, 16) or not np.isfinite(a).all() or
            isinstance(steps, (bool, np.bool_)) or not isinstance(steps, (int, np.integer)) or
            not 1 <= steps <= 50 or isinstance(lookahead, (bool, np.bool_)) or
            not isinstance(lookahead, (int, np.integer)) or not 0 <= lookahead <= 10 or
            len(armed_sides) != 2):
        raise RuntimeError('Invalid synchronized grasp window')
    armed = list(armed_sides)
    for index, row in enumerate(a[:min(50, steps+lookahead)]):
        closing = [SIDE_NAMES[s] for s, column in enumerate((7, 15))
                   if armed[s] and row[column] >= .5]
        if closing:
            return max(steps, index+1), closing
        for side, column in enumerate((7, 15)):
            if row[column] <= .2:
                armed[side] = True
    return steps, []


def synchronized_plan(actions, measured_deg, hand_feedback, fk, steps,
                      held_hand, reference_deg, last_hand=None, right_armed=True,
                      limit_events=None, left_armed=True):
    """Reach each retained model pose before changing its hand target.

    Split at either hand's first closing edge/increase; later arm actions never
    enter the submitted trajectory. The closing target is returned
    separately for publication only after measured arms have settled.
    """
    _, goals = bounded_plan(actions, measured_deg, hand_feedback, fk, steps,
                            held_hand=held_hand, reference_deg=reference_deg,
                            limit_events=limit_events)
    q = np.asarray(reference_deg, dtype=float).copy()
    previous = np.asarray(hand_feedback if last_hand is None else last_hand, dtype=float)
    if (previous.shape != (ROBOT_PROFILE.hand_size,) or not np.isfinite(previous).all() or
            np.any(previous < 0) or np.any(previous > 100)):
        raise RuntimeError('Invalid last synchronized hand target')
    previous = np.rint(previous).astype(int)
    lower, upper = arm_command_limits_deg(fk)
    increment = arm_speed_limit_deg_s() / 10.
    frames, hands = [], []
    close_target = None
    close_is_new_episode = False
    closing_sides = []
    close_new_episodes = [False, False]
    armed = [left_armed, right_armed]
    consumed = 0
    for index, (row, native) in enumerate(zip(np.asarray(actions)[:steps], goals)):
        target = np.rint(native).astype(int)
        closing = []
        for side, indices in enumerate(GRIP_INDICES):
            # Keep the last applied model goal across small prediction jitter.
            # Do not accumulate 1-unit increases into an unguarded large close.
            if (not armed[side] and target[indices[0]] >= 50 and
                    np.max(np.abs(target[list(indices)]-previous[list(indices)])) <
                    GRASP_TARGET_DEADBAND_UNITS):
                target[list(indices)] = previous[list(indices)]
            if target[indices[0]] >= 50 and (armed[side] or
                    np.max(target[list(indices)]-previous[list(indices)]) >=
                    GRASP_TARGET_DEADBAND_UNITS):
                closing.append(side)
        goal = np.clip(np.degrees(row[ARM_COLUMNS]), lower, upper)
        if closing and all(not armed[side] for side in closing):
            # A stronger model close must complete before any subsequent lift.
            goal = q.copy()
        samples = max(1, int(np.ceil(np.max(np.abs(goal-q))/increment)))
        if len(frames)+samples > MAX_SYNC_SAMPLES:
            if frames:
                break
            raise RuntimeError('One synchronized model pose exceeds five seconds')
        for sample in range(1, samples+1):
            frame = q + (goal-q) * (sample/samples)
            fk.check_limits(frame)
            hand = previous.copy()
            if sample == samples:
                hand = target.copy()
                for side in closing:
                    indices = list(GRIP_INDICES[side])
                    hand[indices] = previous[indices]
            frames.append(frame.tolist())
            hands.append(hand.tolist())
        q = goal
        consumed = index+1
        if closing:
            close_target = target.tolist()
            closing_sides = [SIDE_NAMES[side] for side in closing]
            for side in closing:
                close_new_episodes[side] = bool(armed[side])
                armed[side] = False
            close_is_new_episode = close_new_episodes[1]
            break
        for side, indices in enumerate(GRIP_INDICES):
            if target[indices[0]] <= 20:
                armed[side] = True
        previous = target
    return dict(frames=frames, hands=hands, model_steps=consumed,
                close_target=close_target, right_armed=armed[1], left_armed=armed[0],
                closing_sides=closing_sides, close_new_episodes=close_new_episodes,
                close_is_new_episode=close_is_new_episode)


class ArmSettlingGate:
    """Same 5-degree / 5-degree-per-second limits, with a paired wall clock.

    Sensor positions and the MPC clock arrive independently. Dividing one
    position update by an unrelated tiny MPC-clock increment amplifies jitter.
    Use a >=120-ms position total-variation window, not that unpaired denominator.
    No target rebasing, tolerance increase or corrective motion is performed.
    """
    def __init__(self, reference):
        self.reference = np.asarray(reference, dtype=float)
        if self.reference.shape != (14,) or not np.isfinite(self.reference).all():
            raise RuntimeError('Invalid grasp arm reference')
        self.samples = []
        self.last_clock = None
        self.last_now = None
        self.stable_since = None
        self.error = np.zeros(14)
        self.speed = None
        self.near = self.slow = False
        self.feedback = self.reference.copy()

    def observe(self, now, controller_clock, positions):
        q = np.asarray(positions, dtype=float)
        if (q.shape != (14,) or not np.isfinite(q).all() or
                not np.isfinite((now, controller_clock)).all() or
                self.last_now is not None and now < self.last_now):
            raise RuntimeError('Invalid grasp arm settling sample')
        if self.last_clock is not None and controller_clock <= self.last_clock:
            return False
        if self.last_now is not None and now-self.last_now > .25:
            self.samples = []
            self.stable_since = None
        self.last_clock, self.last_now = controller_clock, now
        self.feedback = q.copy()
        self.error = np.abs(q-self.reference)
        self.near = bool(np.max(self.error) <= 5.)
        self.samples.append((now, q.copy()))
        while len(self.samples) > 2 and self.samples[1][0] <= now-.12:
            self.samples.pop(0)
        span = now-self.samples[0][0]
        self.speed = None
        self.slow = False
        if span >= .12-1e-9:
            values = np.asarray([sample[1] for sample in self.samples])
            self.speed = np.abs(np.diff(values,axis=0)).sum(axis=0)/span
            self.slow = bool(np.max(self.speed) <= 5.)
        if self.near and self.slow:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        return self.stable_since is not None and now-self.stable_since >= .3

    def diagnostic(self):
        worst = int(np.argmax(self.error))
        return dict(arm_reference_deg=self.reference.tolist(),
                    arm_feedback_deg=self.feedback.tolist(),
                    absolute_error_deg=self.error.tolist(),
                    worst_joint=ARM_JOINT_NAMES[worst],
                    worst_error_deg=float(self.error[worst]),
                    position_speed_deg_s=None if self.speed is None else self.speed.tolist(),
                    near_target=self.near, slow=self.slow,
                    stable_seconds=0. if self.stable_since is None else self.last_now-self.stable_since,
                    position_tolerance_deg=5., speed_limit_deg_s=5.,
                    speed_clock='paired_monotonic_position_total_variation')


def _hand_vector(values, label):
    result = np.asarray(values, dtype=float)
    if (result.shape != (ROBOT_PROFILE.hand_size,) or not np.isfinite(result).all() or
            np.any(result < 0) or np.any(result > 100)):
        raise RuntimeError('Invalid closure '+label)
    return result


class ClosureFeedbackGate:
    """Wait for target arrival OR demonstrated closing motion to settle.

    A stable off-target plateau can be contact OR a mechanical/drive constraint.
    Returning True only completes the bounded wait; callers must obtain fresh
    observations/inference and must never label this as a verified grasp.
    """
    def __init__(self, target, sent_at, initial, episode_start=None, side='right'):
        if side not in SIDE_NAMES:
            raise RuntimeError('Invalid closure side')
        self.side = side
        self.indices = list(GRIP_INDICES[SIDE_NAMES.index(side)])
        self.target = _hand_vector(target,'target')[self.indices]
        self.initial = _hand_vector(initial,'initial feedback')[self.indices]
        origin = initial if episode_start is None else episode_start
        self.episode_start = _hand_vector(origin,'episode start')[self.indices]
        if not np.isfinite(sent_at):
            raise RuntimeError('Invalid closure command time')
        self.sent_at = sent_at
        self.last_stamp = sent_at
        self.last = None
        self.stable_since = None
        self.stable_min = None
        self.stable_max = None
        self.max_error = None
        self.progress = np.zeros(len(self.indices))
        self.progress_sufficient = False
        self.target_reached = False
        self.completion_reason = None
        self.fresh_samples = 0
        self.peak = self.initial.copy()
        self.expected_closing = self.target-self.initial >= GRASP_MAX_REVERSE_UNITS

    def diagnostic(self):
        stable = (0. if self.stable_since is None else
                  max(0.,self.last_stamp-self.stable_since))
        result = dict(completion_reason=self.completion_reason,
                    target_reached=bool(self.target_reached),
                    finger_progress_units=self.progress.tolist(),
                    closing_progress_sufficient=bool(self.progress_sufficient),
                    stable_feedback_seconds=float(stable),
                    fresh_feedback_samples=self.fresh_samples)
        result[self.side+'_finger_progress_units'] = self.progress.tolist()
        return result

    def observe(self, now, feedback_stamp, actual):
        actual = _hand_vector(actual,'feedback')
        if not np.isfinite((now,feedback_stamp)).all():
            raise RuntimeError('Invalid closure feedback clock')
        if feedback_stamp <= self.last_stamp:
            return False
        if feedback_stamp > now+.01 or now-feedback_stamp >= .5:
            raise RuntimeError('Stale or future closure feedback')
        values = actual[self.indices]
        gap = feedback_stamp-self.last_stamp
        self.fresh_samples += 1
        self.max_error = float(np.max(np.abs(values-self.target)))
        near = self.max_error <= GRASP_POSITION_TOLERANCE
        self.target_reached = bool(near)
        self.peak = np.maximum(self.peak,values)
        if np.any(self.expected_closing & (self.peak-values>GRASP_MAX_REVERSE_UNITS)):
            raise RuntimeError(self.side.capitalize()+' finger reversed during requested closure')
        self.progress = values-self.episode_start
        self.progress_sufficient = bool(
            self.progress[0]>=GRASP_PROGRESS_UNITS and
            (len(self.indices) == 1 or np.count_nonzero(self.progress[1:]>=GRASP_PROGRESS_UNITS)>=2) and
            np.all(values<=self.target+GRASP_POSITION_TOLERANCE))
        if self.stable_since is None or gap>GRASP_MAX_SAMPLE_GAP_SECONDS:
            self.stable_since = feedback_stamp
            self.stable_min = self.stable_max = values.copy()
        else:
            self.stable_min = np.minimum(self.stable_min,values)
            self.stable_max = np.maximum(self.stable_max,values)
            if np.max(self.stable_max-self.stable_min)>2:
                # Consecutive small increments must not disguise continued motion.
                self.stable_since = feedback_stamp
                self.stable_min = self.stable_max = values.copy()
        self.last, self.last_stamp = values.copy(), feedback_stamp
        settled = (now-self.sent_at >= GRASP_MIN_WAIT_SECONDS and
                   self.fresh_samples>=3 and
                   feedback_stamp-self.stable_since>=GRASP_STABLE_SECONDS)
        self.completion_reason = None
        if settled and near:
            self.completion_reason = 'target_reached'
        elif settled and self.progress_sufficient:
            self.completion_reason = 'closure_motion_settled'
        return self.completion_reason is not None
