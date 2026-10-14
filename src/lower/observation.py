"""Observation contract for the current 29-joint Kuavo and SmolVLA dataset.

Camera orientation is relative to the live ROS RGB topics, which can already
include driver rotation. Compare the final frames with training video frames.
"""
import numpy as np


CAMERA_ROTATIONS = {'head_cam_h': 180, 'wrist_cam_l': 0, 'wrist_cam_r': 0}
ARM_START_INDEX = 13  # 12 legs, 1 waist, 14 arms, 2 head joints.
SENSOR_JOINT_COUNT = 29


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
    hands = np.array([hand_positions['l_thumb'], hand_positions['r_thumb']], dtype=np.float64)
    if not np.isfinite(hands).all() or np.any(hands < 0) or np.any(hands > 100):
        raise ValueError('Measured dexhand positions must be in [0,100]')
    arms = q[ARM_START_INDEX:ARM_START_INDEX + 14]
    # Qiangnao's one-DOF training representation uses [0,1], while
    # /dexhand/state reports thumb positions in [0,100]. Arm angles are radians.
    return np.concatenate((arms[:7], [hands[0]/100.0], arms[7:], [hands[1]/100.0])).astype(np.float32)
