#!/usr/bin/env python3
"""Latest ROS observations, image decoding and verified training preprocessing."""
from __future__ import annotations

import argparse
import io
import json
import socket
import struct
import threading
import time

import numpy as np
import rospy
from PIL import Image
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import JointState
from kuavo_msgs.msg import sensorsData
from observation import CAMERA_ROTATIONS, make_model_state, orient_camera_rgb
from camera_codec import decode_rgb_jpeg
from robot_profile import active_profile, STATE_CONTRACT
ROBOT_PROFILE = active_profile()


MAGIC = b"SVSHAD01"


class Latest:
    def __init__(self, camera_rotations=None):
        self.lock = threading.Lock()
        self.camera_rotations = dict(CAMERA_ROTATIONS if camera_rotations is None else camera_rotations)
        self.images: dict[str, tuple[float, np.ndarray]] = {}
        self.image_counts: dict[str, int] = {}
        self.joints: tuple[float, list[float]] | None = None
        self.hands: tuple[float, dict[str, float]] | None = None

    def compressed_image(self,key,msg):
        stamp=time.monotonic()
        try:
            arr=orient_camera_rgb(decode_rgb_jpeg(msg.data),self.camera_rotations[key])
        except (ValueError,OSError) as exc:
            rospy.logwarn_throttle(5,'Invalid compressed camera image %s: %s',key,exc)
            return
        with self.lock:
            self.image_counts[key]=self.image_counts.get(key,0)+1
            self.images[key]=(stamp,arr)

    def image(self, key: str, msg: RosImage) -> None:
        with self.lock:
            self.image_counts[key] = self.image_counts.get(key, 0) + 1
        encoding = msg.encoding.lower()
        rospy.loginfo_throttle(10.0, "IMAGE_CALLBACK: %s encoding=%s size=%dx%d step=%d",
                               key, encoding, msg.width, msg.height, msg.step)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if raw.size < expected:
            rospy.logwarn_throttle(10.0, "invalid image buffer for %s", key)
            return
        if encoding in ("rgb8", "bgr8"):
            if msg.step < msg.width * 3:
                rospy.logwarn_throttle(10.0, "invalid RGB image buffer for %s", key)
                return
            arr = raw[:expected].reshape(msg.height, msg.step)[:, : msg.width * 3]
            arr = arr.reshape(msg.height, msg.width, 3)
        elif encoding in ("yuyv", "yuy2"):
            if msg.step < msg.width * 2 or msg.width % 2:
                rospy.logwarn_throttle(10.0, "invalid YUYV image buffer for %s", key)
                return
            yuyv = raw[:expected].reshape(msg.height, msg.step)[:, : msg.width * 2]
            yuyv = yuyv.reshape(msg.height, msg.width // 2, 2, 2).astype(np.int32)
            y0, u, y1, v = yuyv[:, :, 0, 0], yuyv[:, :, 0, 1], yuyv[:, :, 1, 0], yuyv[:, :, 1, 1]
            def convert(y):
                c = y - 16
                d = u - 128
                e = v - 128
                return np.stack((298 * c + 409 * e, 298 * c - 100 * d - 208 * e, 298 * c + 516 * d), axis=-1)
            rgb0, rgb1 = convert(y0), convert(y1)
            arr = np.empty((msg.height, msg.width, 3), dtype=np.int32)
            arr[:, 0::2, :], arr[:, 1::2, :] = rgb0, rgb1
            arr = np.clip(arr // 256, 0, 255).astype(np.uint8)
        else:
            rospy.logwarn_throttle(10.0, "unsupported image encoding: %s", msg.encoding)
            return
        if encoding == "bgr8":
            arr = arr[:, :, ::-1]
        try:
            arr = orient_camera_rgb(arr, self.camera_rotations[key])
        except ValueError as exc:
            rospy.logwarn_throttle(10.0, "camera preprocessing failed for %s: %s", key, exc)
            return
        # Use local receipt time for freshness checks. The robot's camera clock
        # can be in a different ROS time domain after reboot.
        stamp = time.monotonic()
        with self.lock:
            self.images[key] = (stamp, arr.copy())

    def joint(self, msg: sensorsData) -> None:
        q = [float(v) for v in msg.joint_data.joint_q]
        if len(q) != ROBOT_PROFILE['joints']['sensor_count'] or not np.isfinite(q).all():
            rospy.logwarn_throttle(10.0, "unexpected joint_q: len=%d", len(q))
            return
        stamp = time.monotonic()
        with self.lock:
            self.joints = (stamp, q)

    def hand(self, msg: JointState) -> None:
        values = {str(name): float(value) for name, value in zip(msg.name, msg.position)}
        if "l_thumb" not in values or "r_thumb" not in values:
            rospy.logwarn_throttle(10.0, "thumb joints missing from /dexhand/state")
            return
        stamp = time.monotonic()
        with self.lock:
            self.hands = (stamp, values)

    def end_effector(self, values, stamp):
        with self.lock:
            self.hands = (stamp, list(values))

    def packet(self, max_age: float) -> bytes | None:
        with self.lock:
            required = ("head_cam_h", "wrist_cam_l", "wrist_cam_r")
            if self.joints is None or self.hands is None or any(k not in self.images for k in required):
                return None
            now = time.monotonic()
            image_values = {k: self.images[k] for k in required}
            joint_stamp, q = self.joints
            hand_stamp, hands = self.hands
        if any(now - stamp > max_age for stamp, _ in image_values.values()):
            return None
        if now - joint_stamp > max_age or now - hand_stamp > max_age:
            return None
        # Use the verified sensor arm segment q[13:27], and map measured
        # dexhand thumb units [0,100] to the training hand scalars [0,1].
        try:
            state = make_model_state(q, hands).tolist()
        except (ValueError, KeyError) as exc:
            rospy.logwarn_throttle(10.0, "state preprocessing failed: %s", exc)
            return None
        if len(state) != 16 or not np.isfinite(state).all():
            return None
        parts = []
        for key in required:
            # JPEG keeps the bridge bandwidth bounded while preserving the
            # model's expected RGB input dimensions. Encoding only the latest
            # frame here prevents the ROS callback threads from being blocked
            # by image compression at camera rate.
            with io.BytesIO() as out:
                Image.fromarray(image_values[key][1], mode="RGB").save(out, format="JPEG", quality=85)
                parts.append(out.getvalue())
        meta = {
            "state": state,
            "camera_rotations_degrees": self.camera_rotations,
            "state_contract": STATE_CONTRACT,
            "robot_identity": ROBOT_PROFILE.identity,
            "ages": {
                "head_cam_h": now - image_values["head_cam_h"][0],
                "wrist_cam_l": now - image_values["wrist_cam_l"][0],
                "wrist_cam_r": now - image_values["wrist_cam_r"][0],
                "state": max(now - joint_stamp, now - hand_stamp),
            },
            "time": now,
        }
        header = json.dumps(meta, separators=(",", ":")).encode("utf-8")
        return MAGIC + struct.pack("!I", len(header)) + header + b"".join(
            struct.pack("!I", len(part)) + part for part in parts
        )
