"""Pure, conservative chest-front return planner; never commands hardware.

The S53 installation has no validated full-workspace collision model.  The
``full`` scope is intentionally unavailable until such a model and an
obstacle-aware planner are supplied and tested on the real robot.
"""

import math

import numpy as np


RATE_HZ = 10
MAX_SPEED_DEG_S = 10.0
MAX_JOINT_DEVIATION_DEG = 70.0
MAX_POINT_DISPLACEMENT_M = {'elbow': .35, 'wrist': .40, 'modeled_hand_tip': .45}
MIN_CENTERLINE_CLEARANCE_M = .08
MAX_BELOW_TARGET_M = .35
MAX_SAMPLES = 250


def _angles(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (14,) or not np.isfinite(result).all():
        raise RuntimeError('%s must contain 14 finite arm angles in degrees' % name)
    return result


def _point(points, key):
    value = np.asarray(points[key], dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise RuntimeError('Invalid FK point: '+key)
    return value


def plan_recovery(scope, fk, current_deg, target_deg, base_xyz, base_ypr, waist_rad):
    """Plan from measured feedback.  Unknown/full scopes fail closed.

    Joint limits and sampled FK are *not* full-body, finger, or environment
    collision checks.  A person must inspect the real scene before execution.
    """
    if scope == 'full':
        raise RuntimeError('Full-workspace recovery is disabled: S53 collision geometry, '
                           'obstacle model and validated planner are not available')
    if scope != 'chest':
        raise RuntimeError('Unknown recovery scope: '+str(scope))
    current = _angles(current_deg, 'current_deg')
    target = _angles(target_deg, 'target_deg')
    xyz = np.asarray(base_xyz, dtype=float)
    ypr = np.asarray(base_ypr, dtype=float)
    if xyz.shape != (3,) or ypr.shape != (3,) or not np.isfinite(np.r_[xyz,ypr,waist_rad]).all():
        raise RuntimeError('Invalid base or waist feedback')
    delta = float(np.max(np.abs(target-current)))
    if delta > MAX_JOINT_DEVIATION_DEG:
        raise RuntimeError('Arms are outside the chest recovery joint envelope: %.2f > %.2f deg'
                           % (delta, MAX_JOINT_DEVIATION_DEG))
    fk.check_limits(current)
    fk.check_limits(target)
    # Quintic time scaling peaks at 1.875 times its mean speed.
    moving_seconds = max(3.0, 1.875*delta/MAX_SPEED_DEG_S)
    steps = int(math.ceil(moving_seconds*RATE_HZ))
    if steps+2 > MAX_SAMPLES:
        raise RuntimeError('Recovery trajectory exceeds the audited sample limit')
    moving_seconds = steps/RATE_HZ
    times = [.5 + i/RATE_HZ for i in range(steps+1)]
    frames = []
    for i in range(steps+1):
        u = i/steps
        s = 10*u**3-15*u**4+6*u**5
        frames.append((current+s*(target-current)).tolist())
    times.append(times[-1]+1.0)
    frames.append(target.tolist())

    target_fk = {side:fk.points(target[offset:offset+7],side,xyz,ypr,waist_rad)
                 for side,offset in (('l',0),('r',7))}
    left_shoulder = _point(target_fk['l'],'shoulder')
    right_shoulder = _point(target_fk['r'],'shoulder')
    lateral = left_shoulder-right_shoulder
    norm = float(np.linalg.norm(lateral))
    if norm < .3 or norm > .8:
        raise RuntimeError('Unexpected shoulder geometry; S53 chest envelope cannot be checked')
    lateral /= norm
    center = (left_shoulder+right_shoulder)/2
    closest_centerline = float('inf')
    largest_displacement = {key:0.0 for key in MAX_POINT_DISPLACEMENT_M}
    lowest_relative_height = float('inf')
    for frame in frames:
        fk.check_limits(frame)
        for side,offset,sign in (('l',0,1),('r',7,-1)):
            points = fk.points(frame[offset:offset+7],side,xyz,ypr,waist_rad)
            for key in ('elbow','wrist','modeled_hand_tip'):
                p = _point(points,key)
                reference = _point(target_fk[side],key)
                clearance = float(sign*np.dot(p-center,lateral))
                closest_centerline = min(closest_centerline,clearance)
                if clearance < MIN_CENTERLINE_CLEARANCE_M:
                    raise RuntimeError('%s %s crosses the conservative torso centerline envelope'
                                       % (side,key))
                height = float(p[2]-reference[2])
                lowest_relative_height = min(lowest_relative_height,height)
                if height < -MAX_BELOW_TARGET_M:
                    raise RuntimeError('%s %s dips below the chest recovery height envelope'
                                       % (side,key))
                displacement = float(np.linalg.norm(p-reference))
                largest_displacement[key] = max(largest_displacement[key],displacement)
                if displacement > MAX_POINT_DISPLACEMENT_M[key]:
                    raise RuntimeError('%s %s leaves the chest recovery FK envelope'
                                       % (side,key))
    return {'scope':'chest', 'initial_deg':current.tolist(), 'target_deg':target.tolist(),
            'planning_context':{'base_xyz':xyz.tolist(), 'base_ypr':ypr.tolist(),
                                'waist_rad':float(waist_rad)},
            'times':times, 'frames':frames, 'duration_seconds':times[-1],
            'moving_seconds':moving_seconds, 'peak_speed_deg_s':1.875*delta/moving_seconds,
            'max_joint_deviation_deg':delta, 'minimum_centerline_clearance_m':closest_centerline,
            'maximum_fk_displacement_m':largest_displacement,
            'lowest_fk_height_relative_to_target_m':lowest_relative_height,
            'already_at_target':delta <= 2.0,
            'collision_check':'Joint/FK envelope only; no self, finger, object or scene collision guarantee.'}
