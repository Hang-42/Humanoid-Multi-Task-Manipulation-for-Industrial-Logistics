"""Convert the KD12 16-vector into speed-bounded arm/whole-hand commands."""
import numpy as np

MAX_EXECUTION_STEPS = 50

ARM_COLUMNS = list(range(7)) + list(range(8, 15))
HAND_COLUMNS = [7, 15]
HAND_BY_SIDE = ((0, 2, 3, 4, 5), (6, 8, 9, 10, 11))
CONTROLLED_HAND_INDICES = HAND_BY_SIDE[0] + HAND_BY_SIDE[1]
# Free-air S53 measurements on 2026-10-12: thumb/index follow 60 but stall
# above it; the other fingers follow 75. Clip requested positions to tested
# targets instead of treating the unavoidable thumb/index gap as tracking loss.
HAND_COMMAND_CAPS = np.array([60, 60, 75, 75, 75, 100, 100, 100, 100, 100], dtype=float)
HELD_HAND_INDICES = (1, 7)  # Thumb auxiliary remains at its verified startup target.
ARM_JOINT_NAMES = tuple('zarm_%s%d_joint' % (side, i)
                        for side in ('l', 'r') for i in range(1, 8))


def arm_command_limits_deg(fk):
    """Use the running robot's URDF limits with one degree of clearance."""
    lower, upper = [], []
    for name in ARM_JOINT_NAMES:
        limit = fk.joints[name].find('limit')
        low = np.degrees(float(limit.get('lower'))) + 1
        high = np.degrees(float(limit.get('upper'))) - 1
        if not np.isfinite((low, high)).all() or low >= high:
            raise RuntimeError('Invalid arm limit in robot URDF: ' + name)
        lower.append(low)
        upper.append(high)
    return np.asarray(lower), np.asarray(upper)


def bounded_plan(actions, measured_deg, hand_feedback, fk, steps=10, held_hand=None,
                 limit_events=None):
    """Validate the full chunk structure, then bound only its executed prefix."""
    a = np.asarray(actions, dtype=float)
    q = np.asarray(measured_deg, dtype=float)
    h = np.asarray(hand_feedback, dtype=float)
    if a.shape != (50,16) or not np.isfinite(a).all():
        raise RuntimeError('Expected a finite 50x16 action chunk')
    if q.shape != (14,) or h.shape != (12,) or not np.isfinite(q).all() or not np.isfinite(h).all():
        raise RuntimeError('Invalid robot feedback')
    if (np.any(h < 0) or np.any(h > 100) or
            isinstance(steps, (bool, np.bool_)) or
            not isinstance(steps, (int, np.integer)) or
            not 1 <= steps <= MAX_EXECUTION_STEPS):
        raise RuntimeError('Invalid hand feedback or rollout length')
    hold = h if held_hand is None else np.asarray(held_hand, dtype=float)
    if hold.shape != (12,) or not np.isfinite(hold).all() or np.any(hold < 0) or np.any(hold > 100):
        raise RuntimeError('Invalid retained hand targets')
    # Only this prefix becomes commands; later predictions are discarded when
    # fresh observations trigger another inference. Match the arm checks below.
    # Slight normalized noise at the endpoints is clipped, larger errors stop.
    predicted_hands = a[:steps, HAND_COLUMNS]
    invalid = np.argwhere((predicted_hands < -.02) | (predicted_hands > 1.02))
    if len(invalid):
        row, hand = invalid[0]
        full_hands = a[:, HAND_COLUMNS]
        raise RuntimeError(
            'Predicted hand scalar outside [0,1] (tolerance 0.02): '
            'step_index=%d, action_column=%d, value=%.8f, executed_steps=%d, '
            'prefix_range=[%.8f,%.8f], full_chunk_range=[%.8f,%.8f]' %
            (row, HAND_COLUMNS[hand], predicted_hands[row, hand], steps,
             predicted_hands.min(), predicted_hands.max(), full_hands.min(), full_hands.max()))
    frames, hands = [], []
    # Keep one startup reference across chunks. Reusing measured feedback as
    # a new target each time would write feedback drift back into commands.
    fixed = np.rint(hold).astype(int)
    controlled = h[list(CONTROLLED_HAND_INDICES)].copy()
    lower, upper = arm_command_limits_deg(fk)
    for step_index, row in enumerate(a[:steps]):
        predicted = np.degrees(row[ARM_COLUMNS])
        desired = np.clip(predicted, lower, upper)
        if limit_events is not None:
            for joint_index in np.flatnonzero(np.abs(predicted-desired) > 1e-6):
                limit_events.append(dict(step_index=step_index,
                    joint=ARM_JOINT_NAMES[joint_index],
                    predicted_deg=float(predicted[joint_index]),
                    commanded_target_deg=float(desired[joint_index])))
        fk.check_limits(desired)
        # 10 Hz rollout, at most 20 deg/s per arm joint and 20 units/s per finger.
        q = q + np.clip(desired-q, -2, 2)
        fk.check_limits(q)
        for side, action_column in enumerate(HAND_COLUMNS):
            desired_hand = np.clip(row[action_column], 0, 1) * 100
            lo, hi = side * 5, (side + 1) * 5
            desired_fingers = np.minimum(desired_hand, HAND_COMMAND_CAPS[lo:hi])
            controlled[lo:hi] += np.clip(desired_fingers - controlled[lo:hi], -2, 2)
        hand = fixed.copy()
        hand[list(CONTROLLED_HAND_INDICES)] = np.rint(controlled).astype(int)
        assert np.array_equal(hand[list(HELD_HAND_INDICES)], fixed[list(HELD_HAND_INDICES)])
        frames.append(q.tolist())
        hands.append(hand.tolist())
    return frames, hands


def check_initialization(result, boot, expected_thumb=15, expected_aux=None, expected_hand=None):
    if result.get('boot_id') != boot or result.get('status') != 'POSE_RESTORED':
        raise RuntimeError('Complete robot initialization on this boot before running SmolVLA')
    reset = result.get('thumb_reset',{})
    if reset.get('status') != 'completed' or len(reset.get('target',[])) != 12:
        raise RuntimeError('Initialization did not record the thumb reset')
    if any(reset['target'][i] != expected_thumb for i in (0,6)):
        raise RuntimeError('Run initialization with the current thumb profile: %s' % expected_thumb)
    if expected_aux is not None and any(reset['target'][i] != expected_aux for i in (1,7)):
        raise RuntimeError('Run initialization with the current thumb auxiliary profile: %s' % expected_aux)
    if expected_hand is not None and list(reset['target']) != list(expected_hand):
        raise RuntimeError('Run initialization with the current saved hand profile')


def retimed_plan(actions, measured_deg, hand_feedback, fk, steps=5,
                 held_hand=None, limit_events=None):
    """Keep each predicted goal; insert 10 Hz samples to respect existing caps."""
    # Reuse the original shape, range, hand and URDF validation.
    bounded_plan(actions, measured_deg, hand_feedback, fk, steps,
                 held_hand=held_hand, limit_events=limit_events)
    a = np.asarray(actions, dtype=float)
    q = np.asarray(measured_deg, dtype=float).copy()
    h = np.asarray(hand_feedback, dtype=float)
    fixed = np.rint(h if held_hand is None else held_hand).astype(int)
    controlled = h[list(CONTROLLED_HAND_INDICES)].copy()
    lower, upper = arm_command_limits_deg(fk)
    frames, hands = [], []
    for row in a[:steps]:
        goal = np.clip(np.degrees(row[ARM_COLUMNS]), lower, upper)
        finger_goal = np.concatenate([
            np.minimum(np.clip(row[column], 0, 1) * 100,
                       HAND_COMMAND_CAPS[side*5:(side+1)*5])
            for side, column in enumerate(HAND_COLUMNS)])
        samples = max(1, int(np.ceil(np.max(np.abs(goal-q))/2)),
                      int(np.ceil(np.max(np.abs(finger_goal-controlled))/2)))
        if len(frames)+samples > 300:
            raise RuntimeError('Retimed window exceeds 30 seconds; use a smaller --steps')
        for index in range(1, samples+1):
            fraction = index / samples
            frame = q + fraction * (goal-q)
            fk.check_limits(frame)
            hand = fixed.copy()
            hand[list(CONTROLLED_HAND_INDICES)] = np.rint(
                controlled + fraction * (finger_goal-controlled)).astype(int)
            frames.append(frame.tolist())
            hands.append(hand.tolist())
        q, controlled = goal, finger_goal
    return frames, hands


def grasp_window(actions, steps, confirmed_sides=()):
    """Stop the prediction prefix at its first unconfirmed closing target.

    This detects a requested close, not successful object contact. Success
    must be checked onsite before permitting the remaining predictions.
    """
    known = set(confirmed_sides)
    for index, row in enumerate(np.asarray(actions)[:steps]):
        pending = []
        for side, column in enumerate(HAND_COLUMNS):
            if row[column] <= .2:
                known.discard(side)
            elif row[column] >= .5 and side not in known:
                pending.append(side)
        if pending:
            return index+1, pending
    return steps, []


def approach_window(actions, steps, extra_steps=0):
    """Extend an open-hand approach a bounded number of model frames.

    The first unconfirmed closing action remains the reference pose, while
    the caller keeps all hand channels fixed at their measured startup
    values.  The result never exceeds the requested execution prefix.
    """
    if (isinstance(extra_steps, (bool, np.bool_)) or
            not isinstance(extra_steps, (int, np.integer)) or
            not 0 <= extra_steps <= 10):
        raise RuntimeError('Approach extra steps must be an integer from 0 to 10')
    closing_step, pending = grasp_window(actions, steps, ())
    if not pending:
        return steps, None, 0
    execution_steps = min(steps, closing_step + int(extra_steps))
    return execution_steps, closing_step, execution_steps - closing_step
