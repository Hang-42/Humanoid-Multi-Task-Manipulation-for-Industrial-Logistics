"""Convert ROS raw color images to contiguous uint8 RGB and JPEG bytes."""
import io

import numpy as np
from PIL import Image as PILImage


def raw_rgb(data, width, height, step, encoding):
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}
    encoding = encoding.lower()
    if encoding not in channels:
        raise ValueError("Unsupported color image encoding: " + encoding)
    if width != 848 or height != 480:
        raise ValueError("Expected 848x480 color image")
    count = channels[encoding]
    if step < width * count:
        raise ValueError("Invalid camera row stride")
    raw = np.frombuffer(data, dtype=np.uint8)
    if raw.size < height * step:
        raise ValueError("Incomplete image buffer")
    pixels = raw[: height * step].reshape(height, step)[:, : width * count].reshape(height, width, count)
    if encoding in ("bgr8", "bgra8"):
        pixels = pixels[:, :, (2, 1, 0)]
    else:
        pixels = pixels[:, :, :3]
    return np.ascontiguousarray(pixels, dtype=np.uint8)


def encode_rgb_jpeg(rgb, quality=90):
    if rgb.shape != (480, 848, 3) or rgb.dtype != np.uint8:
        raise ValueError("Expected 480x848x3 uint8 RGB image")
    if not 1 <= quality <= 100:
        raise ValueError("JPEG quality must be 1..100")
    with io.BytesIO() as stream:
        PILImage.fromarray(rgb, mode="RGB").save(stream, format="JPEG", quality=quality)
        return stream.getvalue()
