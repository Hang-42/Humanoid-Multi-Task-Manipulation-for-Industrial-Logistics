"""Observation contract for the current 29-joint Kuavo and SmolVLA dataset.

Camera orientation is relative to the live ROS RGB topics, which can already
include driver rotation. Compare the final frames with training video frames.
"""
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



CAMERA_ROTATIONS = ROBOT_PROFILE['cameras']['rotations']
ARM_START_INDEX = 13  # 12 legs, 1 waist, 14 arms, 2 head joints.
SENSOR_JOINT_COUNT = ROBOT_PROFILE['joints']['sensor_count']


def orient_camera_rgb(rgb, rotation_degrees):
    rgb = np.asarray(rgb)
    if rgb.shape != (480, 848, 3) or rgb.dtype != np.uint8:
        raise ValueError('Expected uint8 RGB image of shape (480,848,3)')
    if rotation_degrees not in (0, 180):
        raise ValueError('Camera rotation must be 0 or 180 degrees')
    if rotation_degrees == 180:
        rgb = np.rot90(rgb, 2)
    return np.ascontiguousarray(rgb)


def make_model_state(joint_positions, hand_positions):
    q = np.asarray(joint_positions, dtype=np.float64)
    if q.shape != (SENSOR_JOINT_COUNT,) or not np.isfinite(q).all():
        raise ValueError('Expected 29 finite sensor joint positions')
    from end_effector import model_scalars
    if isinstance(hand_positions, dict):  # Legacy Revo1 callers.
        hands = [hand_positions['l_thumb']/100., hand_positions['r_thumb']/100.]
    else:
        hands = model_scalars(hand_positions, ROBOT_PROFILE)
    if not np.isfinite(hands).all() or np.any(np.asarray(hands) < 0) or np.any(np.asarray(hands) > 1):
        raise ValueError('Measured end-effector scalars must be in [0,1]')
    arms = q[ROBOT_PROFILE['joints']['arm_indices']]
    # Qiangnao's one-DOF training representation uses [0,1], while
    # /dexhand/state reports thumb positions in [0,100]. Arm angles are radians.
    return np.concatenate((arms[:7], [hands[0]], arms[7:], [hands[1]])).astype(np.float32)
