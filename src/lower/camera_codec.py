"""Pure camera codec helpers; image orientation is applied by the receiver."""
import io
import numpy as np
from PIL import Image

IMAGE_SHAPE=(480,848,3)


def raw_rgb(data,width,height,step,encoding):
    if (height,width,3)!=IMAGE_SHAPE or encoding.lower() not in ('rgb8','bgr8'):
        raise ValueError('Expected 848x480 RGB/BGR camera image')
    raw=np.frombuffer(data,dtype=np.uint8)
    if step<width*3 or len(raw)<height*step:
        raise ValueError('Invalid camera row stride or buffer length')
    arr=raw[:height*step].reshape(height,step)[:,:width*3].reshape(height,width,3)
    if encoding.lower()=='bgr8': arr=arr[:,:,::-1]
    return np.ascontiguousarray(arr).copy()


def encode_rgb_jpeg(rgb):
    if rgb.shape!=IMAGE_SHAPE or rgb.dtype!=np.uint8:
        raise ValueError('Expected uint8 RGB image of shape (480,848,3)')
    with io.BytesIO() as out:
        Image.fromarray(rgb).save(out,format='JPEG',quality=90)
        return out.getvalue()


def decode_rgb_jpeg(payload):
    with Image.open(io.BytesIO(payload)) as image:
        if image.format!='JPEG' or image.size!=(848,480):
            raise ValueError('Expected 848x480 JPEG camera image')
        return np.array(image.convert('RGB'),dtype=np.uint8)
