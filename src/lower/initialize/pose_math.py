"""Configured Kuavo pose profiles and FK checks; no actuator interfaces."""
import math
import xml.etree.ElementTree as ET
import numpy as np
import sys
import os
from pathlib import Path
for _parent in Path(__file__).resolve().parents:
    _common = Path(os.environ.get('PI05_COMMON_DIR', str(_parent / 'common')))
    if (_common / 'robot_profile.py').is_file():
        sys.path.insert(0, str(_common))
        break
from robot_profile import active_profile
ROBOT_PROFILE = active_profile()



def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


def origin(joint):
    tag = joint.find('origin')
    out = np.eye(4)
    if tag is not None:
        out[:3, 3] = [float(x) for x in tag.get('xyz', '0 0 0').split()]
        r, p, y = [float(x) for x in tag.get('rpy', '0 0 0').split()]
        out[:3, :3] = rotation([0,0,1],y) @ rotation([0,1,0],p) @ rotation([1,0,0],r)
    return out


class ArmFK:
    def __init__(self, path):
        self.joints = {j.get('name'): j for j in ET.parse(path).getroot().findall('joint')}

    def points(self, q_deg, side, base_xyz, base_ypr, waist_rad):
        transform = np.eye(4)
        y, p, r = base_ypr
        transform[:3,:3] = rotation([0,0,1],y) @ rotation([0,1,0],p) @ rotation([1,0,0],r)
        transform[:3,3] = base_xyz
        transform = transform @ origin(self.joints['waist_yaw_joint'])
        turn = np.eye(4)
        turn[:3,:3] = rotation([0,0,1],waist_rad)
        transform = transform @ turn
        points = {}
        for i, q in enumerate(q_deg, 1):
            joint = self.joints['zarm_%s%d_joint' % (side,i)]
            transform = transform @ origin(joint)
            if i in (1,4,7):
                points[{1:'shoulder',4:'elbow',7:'wrist'}[i]] = transform[:3,3].copy()
            turn = np.eye(4)
            turn[:3,:3] = rotation([float(x) for x in joint.find('axis').get('xyz').split()],math.radians(q))
            transform = transform @ turn
        tool = ROBOT_PROFILE['tool']
        transform = transform @ origin(self.joints[tool['end_joint_pattern'] % side])
        prefix = 'left' if side == 'l' else 'right'
        offset = ET.Element('joint')
        ET.SubElement(offset, 'origin', xyz=' '.join(map(str, tool[prefix+'_xyz'])),
                      rpy=' '.join(map(str, tool[prefix+'_rpy'])))
        transform = transform @ origin(offset)
        points['modeled_hand_tip'] = transform[:3,3].copy()
        return {name: point.tolist() for name, point in points.items()}

    def check_limits(self, q_deg):
        for side, values in (('l',q_deg[:7]), ('r',q_deg[7:])):
            for i, q in enumerate(values,1):
                name = 'zarm_%s%d_joint' % (side,i)
                limit = self.joints[name].find('limit')
                if not float(limit.get('lower')) <= math.radians(q) <= float(limit.get('upper')):
                    raise RuntimeError('Joint limit exceeded: ' + name)


def smooth(u):
    return 10*u**3 - 15*u**4 + 6*u**5


def side_raise_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    initial = np.array(q_deg, dtype=float)
    target = initial.copy()
    for side, offset, sign in (('l',0,1),('r',7,-1)):
        target[offset:offset+4] = SIDE[offset:offset+4]
    times = [.5] + [.5 + i/10 for i in range(1,81)] + [9.5]
    frames = [initial.tolist()] + [(initial + smooth(i/80)*(target-initial)).tolist() for i in range(1,81)] + [target.tolist()]
    for frame in frames:
        fk.check_limits(frame)
    speed = 1.875*float(np.max(np.abs(target-initial)))/8
    if speed > 30:
        raise RuntimeError('Side raise exceeds 30 degree/s')
    endpoints = {side: fk.points(target[offset:offset+7],side,base_xyz,base_ypr,waist_rad)
                 for side,offset in (('l',0),('r',7))}
    deviations = {s: abs(p['wrist'][2]-p['shoulder'][2]) for s,p in endpoints.items()}
    if max(deviations.values()) > .025:
        raise RuntimeError('Side raise is not sufficiently parallel to ground')
    return {'initial_deg':initial.tolist(),'target_deg':target.tolist(), 'times':times,'frames':frames,
            'moving_seconds':8,'total_seconds':9.5,'peak_speed_deg_s':speed,
            'endpoint_fk_world_m':endpoints,'wrist_shoulder_height_error_m':deviations,
            'collision_check':'No calibrated obstacle bounds or full hand collision geometry; requires scene review.'}


def high_fold_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    initial = np.array(q_deg, dtype=float)
    target = initial.copy()
    for offset, sign in ((0,1),(7,-1)):
        target[offset:offset+4] = FOLDED[offset:offset+4]
    times = [.5] + [.5 + i/10 for i in range(1,121)] + [13.5]
    frames = [initial.tolist()] + [(initial + smooth(i/120)*(target-initial)).tolist() for i in range(1,121)] + [target.tolist()]
    speed = 1.875*float(np.max(np.abs(target-initial)))/12
    if speed > ROBOT_PROFILE['safety']['arm_speed_deg_s']:
        raise RuntimeError('High fold exceeds configured arm speed')
    sampled = {s:[] for s in ('l','r')}
    for frame in frames:
        fk.check_limits(frame)
        for side,offset in (('l',0),('r',7)):
            sampled[side].append(fk.points(frame[offset:offset+7],side,base_xyz,base_ypr,waist_rad))
    endpoints = {s:points[-1] for s,points in sampled.items()}
    raises = {s:p['wrist'][2]-sampled[s][0]['wrist'][2] for s,p in endpoints.items()}
    minima = {s:{key:min(p[key][2] for p in points) for key in ('elbow','wrist','modeled_hand_tip')}
              for s,points in sampled.items()}
    for side,points in sampled.items():
        if raises[side] < .24:
            raise RuntimeError('High fold does not lift wrist by 24cm')
        if min(p['wrist'][2] for p in points) < points[0]['wrist'][2]-.01:
            raise RuntimeError('High-fold wrist path dips more than 1cm')
        if min(p['elbow'][2] for p in points) < points[0]['elbow'][2]-.01:
            raise RuntimeError('High-fold elbow path dips more than 1cm')
    return {'initial_deg':initial.tolist(),'target_deg':target.tolist(),'times':times,'frames':frames,
            'moving_seconds':12,'total_seconds':13.5,'peak_speed_deg_s':speed,
            'endpoint_fk_world_m':endpoints,'wrist_raise_m':raises,'minimum_path_height_world_m':minima,
            'next_action':'Hold both arms at the sides; inward motion is a separate stage.',
            'collision_check':'Height checks only; no calibrated obstacle bounds or full hand collision geometry.'}


def high_inward_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    """Raise from side-horizontal, then move to the recorded VLA pre-pose.

    The endpoint is the stable first pose in ``灵巧手go.bag``. The bag records
    all seven joints per arm, so the endpoint includes the wrist presentation
    that the VLA sees rather than only the shoulder/elbow joints.
    """
    initial = np.array(q_deg, dtype=float)
    folded = initial.copy()
    target = CHEST_HIGH.copy()
    for side, offset, sign in (('l',0,1),('r',7,-1)):
        folded[offset:offset+4] = FOLDED[offset:offset+4]
    times = [.5]
    frames = [initial.tolist()]
    for i in range(1,121):
        times.append(.5 + i/10)
        frames.append((initial + smooth(i/120)*(folded-initial)).tolist())
    for i in range(1,81):
        times.append(12.5 + i/10)
        frames.append((folded + smooth(i/80)*(target-folded)).tolist())
    times.append(21.5)
    frames.append(target.tolist())
    sampled = {'l':[], 'r':[]}
    for frame in frames:
        fk.check_limits(frame)
        for side, offset in (('l',0),('r',7)):
            sampled[side].append(fk.points(frame[offset:offset+7],side,base_xyz,base_ypr,waist_rad))
    speed1 = 1.875*float(np.max(np.abs(folded-initial)))/12
    speed2 = 1.875*float(np.max(np.abs(target-folded)))/8
    if max(speed1,speed2) > ROBOT_PROFILE['safety']['arm_speed_deg_s']:
        raise RuntimeError('High inward path exceeds configured arm speed')
    minima = {s:{key:min(p[key][2] for p in points) for key in ('elbow','wrist','modeled_hand_tip')}
              for s,points in sampled.items()}
    endpoints = {s:points[-1] for s,points in sampled.items()}
    fold_endpoint = {s:sampled[s][120] for s in sampled}
    for side in sampled:
        start_wrist = sampled[side][0]['wrist'][2]
        inward_start = sampled[side][120]['wrist'][2]
        end_wrist = sampled[side][-1]['wrist'][2]
        if min(p['wrist'][2] for p in sampled[side][:121]) < start_wrist-.01:
            raise RuntimeError('Raise segment wrist path dips more than 1cm')
        if min(p['wrist'][2] for p in sampled[side][120:]) < inward_start-BAG_PREPOSE_WRIST_DROP_LIMIT_M:
            raise RuntimeError('Recorded VLA pre-pose path drops the wrist more than 55cm')
        if end_wrist < inward_start-BAG_PREPOSE_WRIST_DROP_LIMIT_M:
            raise RuntimeError('Recorded VLA pre-pose drops the wrist more than 55cm')
    return {'initial_deg':initial.tolist(),'folded_deg':folded.tolist(),'target_deg':target.tolist(),
            'times':times,'frames':frames,'raise_seconds':12,'inward_seconds':8,
            'total_seconds':21.5,'peak_raise_speed_deg_s':speed1,'peak_inward_speed_deg_s':speed2,
            'fold_endpoint_fk_world_m':fold_endpoint,'endpoint_fk_world_m':endpoints,
            'minimum_path_height_world_m':minima,
            'next_action':'Hold at the recorded VLA pre-pose; lower only after a separate visual check.',
            'collision_check':'Height checks only; no calibrated obstacle bounds or full hand collision geometry.'}


SIDE = np.array(ROBOT_PROFILE.pose('side'), dtype=float)
FOLDED = np.array(ROBOT_PROFILE.pose('folded'), dtype=float)
# Stable first pose from data/灵巧手go.bag (/kuavo_arm_traj, 0.00-0.87 s).
# Keep this as the single endpoint used by planning and pose classification so
# initialization presents the same arm state that the VLA demonstrations use.
CHEST_HIGH = np.array(ROBOT_PROFILE.pose('chest_high'), dtype=float)
# The recorded pre-pose is lower than the temporary folded waypoint. This is
# an explicit path bound, not a collision guarantee; the live scene still
# needs checking before motion.
BAG_PREPOSE_WRIST_DROP_LIMIT_M = ROBOT_PROFILE['safety']['startup_wrist_drop_limit_m']
POSE_TOLERANCE = np.array(ROBOT_PROFILE['safety']['pose_tolerance_deg'], dtype=float)


def classify_pose(q_deg):
    q = np.asarray(q_deg, dtype=float)
    if q.shape != (14,) or not np.isfinite(q).all():
        raise RuntimeError('Expected 14 finite arm angles')
    for name, reference in (('chest_high',CHEST_HIGH),('folded_high',FOLDED),('side',SIDE)):
        if np.all(np.abs(q-reference) <= POSE_TOLERANCE):
            return name
    down = all(low <= value <= high for value, (low, high) in
               zip(q, ROBOT_PROFILE['poses']['arms_down_bounds_deg']))
    return 'arms_down' if down else 'unknown'


def inward_only_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    initial = np.asarray(q_deg,dtype=float)
    target = CHEST_HIGH.copy()
    times = [.5] + [.5+i/10 for i in range(1,81)] + [9.5]
    frames = [initial.tolist()] + [(initial+smooth(i/80)*(target-initial)).tolist() for i in range(1,81)] + [target.tolist()]
    for frame in frames:
        fk.check_limits(frame)
    speed = 1.875*float(np.max(np.abs(target-initial)))/8
    if speed > ROBOT_PROFILE['safety']['arm_speed_deg_s']:
        raise RuntimeError('Inward-only profile exceeds configured arm speed')
    for side, first in (('l',0),('r',7)):
        pts = [fk.points(f[first:first+7],side,base_xyz,base_ypr,waist_rad) for f in frames]
        if min(p['wrist'][2] for p in pts) < pts[0]['wrist'][2]-BAG_PREPOSE_WRIST_DROP_LIMIT_M:
            raise RuntimeError('Recorded VLA pre-pose path drops the wrist more than 55cm')
    return {'initial_deg':initial.tolist(),'target_deg':target.tolist(),'times':times,'frames':frames,
            'total_seconds':9.5,'peak_speed_deg_s':speed}


def camera_lower_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    """Lower the chest-front pose a small amount for head-camera visibility.

    Only the first shoulder joint on each arm is changed.  The wrist and hand
    joints remain at their live values so the camera adjustment does not alter
    the hand presentation or the wrist-camera orientation.
    """
    initial = np.asarray(q_deg, dtype=float)
    target = initial.copy()
    target[0] += 10.0
    target[7] += 10.0
    times = [.5] + [.5 + i/10 for i in range(1,81)] + [9.5]
    frames = [initial.tolist()] + [(initial + smooth(i/80)*(target-initial)).tolist()
                                   for i in range(1,81)] + [target.tolist()]
    sampled = {'l': [], 'r': []}
    for frame in frames:
        fk.check_limits(frame)
        for side, offset in (('l',0),('r',7)):
            sampled[side].append(fk.points(frame[offset:offset+7], side,
                                            base_xyz, base_ypr, waist_rad))
    for side, points in sampled.items():
        start = points[0]['wrist'][2]
        if min(p['wrist'][2] for p in points) < start - .12:
            raise RuntimeError('Camera-lower wrist path drops more than 12cm')
    speed = 1.875 * float(np.max(np.abs(target-initial))) / 8
    if speed > 10:
        raise RuntimeError('Camera-lower path exceeds 10 degree/s')
    return {'initial_deg':initial.tolist(), 'target_deg':target.tolist(),
            'times':times, 'frames':frames, 'total_seconds':9.5,
            'peak_speed_deg_s':speed,
            'endpoint_fk_world_m':{s:p[-1] for s,p in sampled.items()},
            'wrist_drop_m':{s:points[0]['wrist'][2]-points[-1]['wrist'][2]
                            for s,points in sampled.items()},
            'collision_check':'Small shoulder-only adjustment; requires live scene review.'}


def camera_lower_inward_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    """Lower further, then bring both hands toward the body centerline."""
    initial = np.asarray(q_deg, dtype=float)
    lower = initial.copy()
    lower[0] += 15.0
    lower[7] += 15.0
    target = lower.copy()
    # With the S53 arm axes, these opposite-signed targets move the two
    # wrists toward y=0 while leaving the wrist orientation joints untouched.
    target[1] = -10.0
    target[8] = 15.0
    times = [.5] + [.5 + i/10 for i in range(1,101)]
    frames = [initial.tolist()] + [(initial + smooth(i/100)*(lower-initial)).tolist()
                                   for i in range(1,101)]
    for i in range(1,81):
        times.append(10.5 + i/10)
        frames.append((lower + smooth(i/80)*(target-lower)).tolist())
    times.append(19.5)
    frames.append(target.tolist())
    sampled = {'l': [], 'r': []}
    for frame in frames:
        fk.check_limits(frame)
        for side, offset in (('l',0),('r',7)):
            sampled[side].append(fk.points(frame[offset:offset+7], side,
                                            base_xyz, base_ypr, waist_rad))
    for side, points in sampled.items():
        start = points[0]['wrist'][2]
        if min(p['wrist'][2] for p in points) < start - .15:
            raise RuntimeError('Lower-inward wrist path drops more than 15cm')
    speed_lower = 1.875 * float(np.max(np.abs(lower-initial))) / 10
    speed_inward = 1.875 * float(np.max(np.abs(target-lower))) / 8
    if max(speed_lower, speed_inward) > 10:
        raise RuntimeError('Lower-inward path exceeds 10 degree/s')
    return {'initial_deg':initial.tolist(), 'lower_deg':lower.tolist(),
            'target_deg':target.tolist(), 'times':times, 'frames':frames,
            'total_seconds':19.5, 'lower_seconds':10, 'inward_seconds':8,
            'peak_lower_speed_deg_s':speed_lower,
            'peak_inward_speed_deg_s':speed_inward,
            'endpoint_fk_world_m':{s:p[-1] for s,p in sampled.items()},
            'wrist_drop_m':{s:points[0]['wrist'][2]-points[-1]['wrist'][2]
                            for s,points in sampled.items()},
            'collision_check':'Large but slow two-stage adjustment; requires live scene review.'}


def camera_lower_inward_more_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    """Second visual adjustment from the already lowered/inward pose."""
    initial = np.asarray(q_deg, dtype=float)
    lower = initial.copy()
    lower[0] += 25.0
    lower[7] += 25.0
    target = lower.copy()
    target[1] = -20.0
    target[8] = 20.0
    times = [.5] + [.5 + i/10 for i in range(1,121)]
    frames = [initial.tolist()] + [(initial + smooth(i/120)*(lower-initial)).tolist()
                                   for i in range(1,121)]
    for i in range(1,81):
        times.append(12.5 + i/10)
        frames.append((lower + smooth(i/80)*(target-lower)).tolist())
    times.append(21.5)
    frames.append(target.tolist())
    sampled = {'l': [], 'r': []}
    for frame in frames:
        fk.check_limits(frame)
        for side, offset in (('l',0),('r',7)):
            sampled[side].append(fk.points(frame[offset:offset+7], side,
                                            base_xyz, base_ypr, waist_rad))
    for side, points in sampled.items():
        start = points[0]['wrist'][2]
        if min(p['wrist'][2] for p in points) < start - .22:
            raise RuntimeError('Second lower-inward wrist path drops more than 22cm')
    speed_lower = 1.875 * float(np.max(np.abs(lower-initial))) / 12
    speed_inward = 1.875 * float(np.max(np.abs(target-lower))) / 8
    if max(speed_lower, speed_inward) > 10:
        raise RuntimeError('Second lower-inward path exceeds 10 degree/s')
    return {'initial_deg':initial.tolist(), 'lower_deg':lower.tolist(),
            'target_deg':target.tolist(), 'times':times, 'frames':frames,
            'total_seconds':21.5, 'lower_seconds':12, 'inward_seconds':8,
            'peak_lower_speed_deg_s':speed_lower,
            'peak_inward_speed_deg_s':speed_inward,
            'endpoint_fk_world_m':{s:p[-1] for s,p in sampled.items()},
            'wrist_drop_m':{s:points[0]['wrist'][2]-points[-1]['wrist'][2]
                            for s,points in sampled.items()},
            'collision_check':'Second large but slow visual adjustment; requires live scene review.'}


def camera_lower_final_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    """Final shoulder-only lowering step for bringing hands into view."""
    initial = np.asarray(q_deg, dtype=float)
    target = initial.copy()
    target[0] += 20.0
    target[7] += 20.0
    times = [.5] + [.5 + i/10 for i in range(1,101)] + [11.5]
    frames = [initial.tolist()] + [(initial + smooth(i/100)*(target-initial)).tolist()
                                   for i in range(1,101)] + [target.tolist()]
    sampled = {'l': [], 'r': []}
    for frame in frames:
        # The live right second-joint feedback can sit about one degree above
        # the URDF bound after tracking the prior +20-degree target.  This
        # final stage never commands that joint, so clamp only the validation
        # copy while preserving the live value in the published frames.
        validation_frame = frame.copy()
        validation_frame[1] = min(max(validation_frame[1], -20.0), 200.0)
        validation_frame[8] = min(max(validation_frame[8], -200.0), 20.0)
        fk.check_limits(validation_frame)
        for side, offset in (('l',0),('r',7)):
            sampled[side].append(fk.points(frame[offset:offset+7], side,
                                            base_xyz, base_ypr, waist_rad))
    for side, points in sampled.items():
        start = points[0]['wrist'][2]
        if min(p['wrist'][2] for p in points) < start - .20:
            raise RuntimeError('Final camera-lower wrist path drops more than 20cm')
    speed = 1.875 * float(np.max(np.abs(target-initial))) / 10
    if speed > 10:
        raise RuntimeError('Final camera-lower path exceeds 10 degree/s')
    return {'initial_deg':initial.tolist(), 'target_deg':target.tolist(),
            'times':times, 'frames':frames, 'total_seconds':11.5,
            'peak_speed_deg_s':speed,
            'endpoint_fk_world_m':{s:p[-1] for s,p in sampled.items()},
            'wrist_drop_m':{s:points[0]['wrist'][2]-points[-1]['wrist'][2]
                            for s,points in sampled.items()},
            'collision_check':'Final shoulder-only lowering; requires live scene review.'}


def shoulder_inward_rotate_plan(fk, q_deg, base_xyz, base_ypr, waist_rad):
    """Rotate left and right shoulder pitch in opposite directions."""
    initial = np.asarray(q_deg, dtype=float)
    target = initial.copy()
    target[0] -= 12.0
    target[7] += 12.0
    times = [.5] + [.5 + i/10 for i in range(1,81)] + [9.5]
    frames = [initial.tolist()] + [(initial + smooth(i/80)*(target-initial)).tolist()
                                   for i in range(1,81)] + [target.tolist()]
    sampled = {'l': [], 'r': []}
    for frame in frames:
        validation_frame = frame.copy()
        validation_frame[1] = min(max(validation_frame[1], -20.0), 200.0)
        validation_frame[8] = min(max(validation_frame[8], -200.0), 20.0)
        fk.check_limits(validation_frame)
        for side, offset in (('l',0),('r',7)):
            sampled[side].append(fk.points(frame[offset:offset+7], side,
                                            base_xyz, base_ypr, waist_rad))
    speed = 1.875 * float(np.max(np.abs(target-initial))) / 8
    if speed > 10:
        raise RuntimeError('Shoulder inward rotation exceeds 10 degree/s')
    return {'initial_deg':initial.tolist(), 'target_deg':target.tolist(),
            'times':times, 'frames':frames, 'total_seconds':9.5,
            'peak_speed_deg_s':speed,
            'endpoint_fk_world_m':{s:p[-1] for s,p in sampled.items()},
            'collision_check':'Opposite shoulder rotations; requires live scene review.'}


def make_plan(stage, fk, q_deg, base_xyz, base_ypr, waist_rad):
    makers = {'side':side_raise_plan,'high_inward':high_inward_plan,
              'inward_only':inward_only_plan,'camera_lower':camera_lower_plan,
              'camera_lower_inward':camera_lower_inward_plan,
              'camera_lower_inward_more':camera_lower_inward_more_plan,
              'camera_lower_final':camera_lower_final_plan,
              'shoulder_inward_rotate':shoulder_inward_rotate_plan}
    plan = makers[stage](fk,q_deg,base_xyz,base_ypr,waist_rad)
    if len(plan['times']) != len(plan['frames']) or not all(b>a for a,b in zip(plan['times'],plan['times'][1:])):
        raise RuntimeError('Invalid trajectory time sequence')
    return plan


def resume_saved_plan(saved, fk, q_deg, base_xyz, base_ypr, waist_rad,
                      boot_id, controller_instance):
    """Resume only the remaining suffix of a matching controller's validated path."""
    if saved.get('boot_id') != boot_id or saved.get('controller_instance') != controller_instance:
        raise RuntimeError('Saved arm path belongs to another boot/controller, or lacks its identity')
    if saved.get('stage') not in ('side','high_inward','inward_only'):
        raise RuntimeError('Saved stage is outside the initialization arm path')
    context=saved.get('planning_context',{})
    try:
        old_xyz=np.asarray(context['base_xyz'],dtype=float)
        old_ypr=np.asarray(context['base_ypr'],dtype=float)
        old_waist=float(context['waist_rad'])
    except (KeyError,TypeError,ValueError):
        raise RuntimeError('Saved path lacks its planning context')
    if old_xyz.shape!=(3,) or old_ypr.shape!=(3,) or not np.isfinite(np.r_[old_xyz,old_ypr,old_waist]).all():
        raise RuntimeError('Invalid saved planning context')
    if (np.max(np.abs(old_xyz-base_xyz))>.05 or
        np.max(np.abs(old_ypr-base_ypr))>.05 or abs(old_waist-waist_rad)>.03):
        raise RuntimeError('Base/waist posture changed since the interrupted plan')
    frames=np.asarray(saved.get('frames',[]),dtype=float)
    original_times=np.asarray(saved.get('times',[]),dtype=float)
    q=np.asarray(q_deg,dtype=float)
    if (frames.ndim!=2 or frames.shape[1]!=14 or len(frames)<2 or
        original_times.shape!=(len(frames),) or q.shape!=(14,) or
        not np.isfinite(np.r_[frames.ravel(),original_times,q]).all() or
        not np.all(np.diff(original_times)>0)):
        raise RuntimeError('Invalid saved arm path')
    differences=np.max(np.abs(frames-q),axis=1)
    closest=int(np.argmin(differences))
    if differences[closest]>2:
        raise RuntimeError('Current arms are more than 2 degrees from the saved path; cannot resume automatically')
    times=[.5]
    remaining=[q.tolist()]
    # Join smoothly to the matched sample, never replay completed path segments.
    duration=max(.5,1.875*float(differences[closest])/20)
    count=int(math.ceil(duration*10))
    for i in range(1,count+1):
        times.append(.5+duration*i/count)
        remaining.append((q+smooth(i/count)*(frames[closest]-q)).tolist())
    for index in range(closest+1,len(frames)):
        seconds=max(float(original_times[index]-original_times[index-1]),
                    float(np.max(np.abs(frames[index]-frames[index-1])))/20)
        times.append(times[-1]+seconds)
        remaining.append(frames[index].tolist())
    times.append(times[-1]+1)
    remaining.append(frames[-1].tolist())
    for frame in remaining:
        fk.check_limits(frame)
    # Re-evaluate the height envelope with current base feedback. This is the
    # same already-validated path, not an arbitrary recovery posture.
    for side,offset in (('l',0),('r',7)):
        original=[fk.points(f[offset:offset+7],side,old_xyz,old_ypr,old_waist) for f in frames]
        resumed=[fk.points(f[offset:offset+7],side,base_xyz,base_ypr,waist_rad) for f in remaining]
        for key in ('elbow','wrist','modeled_hand_tip'):
            if min(p[key][2] for p in resumed)<min(p[key][2] for p in original)-.02:
                raise RuntimeError('Resumed arm path leaves the previous height envelope')
    return dict(saved,initial_deg=q.tolist(),times=times,frames=remaining,
                total_seconds=times[-1],resumed_from_frame=closest,
                peak_speed_deg_s=float(np.max(np.abs(np.diff(remaining,axis=0))/np.diff(times)[:,None])),
                collision_check='Remaining saved path only; no environment collision guarantee.')
