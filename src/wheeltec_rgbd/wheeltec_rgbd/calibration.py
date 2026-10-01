"""Explicit local calibration loading, independent of the vendor metadata bug."""
import copy
from pathlib import Path
from urllib.parse import urlparse, unquote
import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo
from .core import FrameError


def load_camera_info(url):
    if not url:
        return None
    parsed = urlparse(url)
    if parsed.scheme not in ('', 'file') or parsed.netloc not in ('', 'localhost'):
        raise ValueError('Calibration must be a local path or file:// URL')
    path = Path(unquote(parsed.path)).expanduser()
    data = yaml.safe_load(path.read_text())
    msg = CameraInfo()
    msg.width, msg.height = int(data['image_width']), int(data['image_height'])
    def matrix(key, rows, cols):
        mat = data[key]
        values = mat['data']
        if mat['rows'] != rows or mat['cols'] != cols or len(values) != rows*cols:
            raise ValueError('Invalid calibration matrix: '+key)
        if not np.isfinite(values).all():
            raise ValueError('Nonfinite calibration: '+key)
        return [float(x) for x in values]
    msg.k = matrix('camera_matrix', 3, 3)
    msg.r = matrix('rectification_matrix', 3, 3)
    msg.p = matrix('projection_matrix', 3, 4)
    dist = data['distortion_coefficients']
    msg.d = matrix('distortion_coefficients', 1, int(dist['cols']))
    msg.distortion_model = str(data['distortion_model'])
    if msg.width <= 0 or msg.height <= 0:
        raise ValueError('Invalid calibration resolution')
    return msg


def overlay_calibration(pair, color_template=None, depth_template=None):
    result = list(pair)
    for index, template in ((2, color_template), (3, depth_template)):
        if template is None:
            continue
        image, raw_info = pair[index-2], pair[index]
        if ((template.width, template.height) != (image.width, image.height)
                or (raw_info.width, raw_info.height) != (image.width, image.height)):
            raise FrameError('CALIBRATION_RESOLUTION_MISMATCH')
        info = copy.deepcopy(template)
        # Keep actual frame/stamp: a file must never disguise unregistered or stale input.
        info.header = copy.deepcopy(raw_info.header)
        result[index] = info
    return tuple(result)
