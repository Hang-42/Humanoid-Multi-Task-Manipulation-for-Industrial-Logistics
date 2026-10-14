#!/usr/bin/env python3
"""Three read-only ROS image subscriptions and compressed PI05 publications."""
import threading
import time

import rospy
from sensor_msgs.msg import CompressedImage, Image

from camera_codec import encode_rgb_jpeg, raw_rgb


CAMERAS = {
    "head_cam_h": "/head_camera/color/image_raw",
    "wrist_cam_l": "/left_wrist_camera/color/image_raw",
    "wrist_cam_r": "/right_wrist_camera/color/image_raw",
}


def main():
    rospy.init_node("pi05_camera_relay")
    quality = int(rospy.get_param("~jpeg_quality", 90))
    publish_hz = float(rospy.get_param("~publish_hz", 10))
    max_age = float(rospy.get_param("~max_frame_age_s", 0.3))
    if not 1 <= quality <= 100 or publish_hz <= 0 or max_age <= 0:
        raise ValueError("Invalid camera relay parameters")

    lock = threading.Lock()
    latest = {}
    sequence = {}
    sent = {}
    publishers = {
        key: rospy.Publisher("/pi05_camera/" + key + "/compressed", CompressedImage, queue_size=1)
        for key in CAMERAS
    }

    def store(key, message):
        arrival = time.monotonic()
        with lock:
            sequence[key] = sequence.get(key, 0) + 1
            latest[key] = (arrival, sequence[key], message)

    subscribers = [
        rospy.Subscriber(topic, Image, lambda message, name=key: store(name, message),
                         queue_size=1, buff_size=16777216)
        for key, topic in CAMERAS.items()
    ]
    rospy.loginfo("PI05_CAMERA_RELAY_READY JPEG=%s rate=%sHz", quality, publish_hz)
    rate = rospy.Rate(publish_hz)
    while not rospy.is_shutdown():
        with lock:
            current = dict(latest)
        for key, (arrival, number, source) in current.items():
            if sent.get(key) == number or time.monotonic() - arrival >= max_age:
                continue
            try:
                rgb = raw_rgb(source.data, source.width, source.height,
                              source.step, source.encoding)
                payload = encode_rgb_jpeg(rgb, quality)
            except ValueError as exc:
                rospy.logerr_throttle(5, "PI05 %s image rejected: %s", key, exc)
                sent[key] = number
                continue
            if time.monotonic() - arrival >= max_age:
                continue
            result = CompressedImage()
            result.header = source.header
            result.format = "rgb8; jpeg compressed rgb8"
            result.data = payload
            publishers[key].publish(result)
            sent[key] = number
        rate.sleep()


if __name__ == "__main__":
    main()
