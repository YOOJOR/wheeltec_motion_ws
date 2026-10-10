"""Bounded RGB-D pairing and image validation; no ROS runtime or hardware side effects."""
from collections import OrderedDict
from functools import lru_cache
from dataclasses import dataclass
import math
import numpy as np


class FrameError(ValueError):
    pass


def stamp_ns(msg):
    s = msg.header.stamp
    if s.sec < 0 or not 0 <= s.nanosec < 10**9:
        raise FrameError('INVALID_STAMP')
    return s.sec * 10**9 + s.nanosec


@dataclass(frozen=True)
class Settings:
    expected_frame: str = 'wheeltec_camera_color_optical_frame'
    frame_timeout: float = 0.5
    future_tolerance: float = 0.1
    sync_tolerance: float = 0.03
    depth_min: float = 0.4
    depth_max: float = 2.0

    def __post_init__(self):
        vals = (self.frame_timeout, self.future_tolerance, self.sync_tolerance,
                self.depth_min, self.depth_max)
        if (not self.expected_frame or not all(math.isfinite(x) for x in vals)
                or self.frame_timeout <= 0 or self.future_tolerance < 0
                or not 0 <= self.sync_tolerance < self.frame_timeout
                or not 0 < self.depth_min < self.depth_max):
            raise ValueError('Invalid RGB-D settings')

    def check_stamp(self, msg, now_ns):
        stamp = stamp_ns(msg)
        age = (now_ns - stamp) / 1e9
        if stamp <= 0 or age > self.frame_timeout or age < -self.future_tolerance:
            raise FrameError('STALE_OR_FUTURE_STAMP')
        return stamp


class PairBuffer:
    """At most four samples per stream; each source image can be used only once."""
    kinds = ('color', 'depth', 'color_info', 'depth_info')

    def __init__(self, settings):
        self.settings = settings
        self.samples = {k: OrderedDict() for k in self.kinds}
        self.last_seen = {k: 0 for k in self.kinds}
        self.last_used = {'color': 0, 'depth': 0}

    def clear(self):
        for samples in self.samples.values():
            samples.clear()

    def add(self, kind, msg, now_ns):
        stamp = self.settings.check_stamp(msg, now_ns)
        if stamp <= self.last_seen[kind]:
            raise FrameError('DUPLICATE_OR_OUT_OF_ORDER')
        self.last_seen[kind] = stamp
        samples = self.samples[kind]
        samples[stamp] = msg
        while len(samples) > 4:
            samples.popitem(last=False)
        # Prefer newest complete color observation, then closest available depth.
        for c in reversed(self.samples['color']):
            if c <= self.last_used['color'] or c not in self.samples['color_info']:
                continue
            depths = sorted(self.samples['depth'], key=lambda d: abs(c-d))
            for d in depths:
                if (d <= self.last_used['depth'] or d not in self.samples['depth_info']
                        or abs(c-d) > self.settings.sync_tolerance * 1e9):
                    continue
                pair = (self.samples['color'][c], self.samples['depth'][d],
                        self.samples['color_info'][c], self.samples['depth_info'][d])
                self.last_used.update(color=c, depth=d)
                for k in self.kinds:
                    used = c if k.startswith('color') else d
                    for t in list(self.samples[k]):
                        if t <= used:
                            del self.samples[k][t]
                return pair
        return None


def image_array(msg):
    formats = {'rgb8': (np.dtype('u1'), 3), 'bgr8': (np.dtype('u1'), 3),
               '16UC1': (np.dtype('u2'), 1), '32FC1': (np.dtype('f4'), 1)}
    if msg.encoding not in formats:
        raise FrameError('UNSUPPORTED_ENCODING')
    dtype, channels = formats[msg.encoding]
    if msg.width <= 0 or msg.height <= 0 or msg.is_bigendian not in (0, 1):
        raise FrameError('INVALID_IMAGE_LAYOUT')
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    row_size = msg.width * channels * dtype.itemsize
    if msg.step < row_size or len(msg.data) != msg.height * msg.step:
        raise FrameError('INVALID_IMAGE_LAYOUT')
    shape = (msg.height, msg.width, channels) if channels == 3 else (msg.height, msg.width)
    strides = ((msg.step, channels*dtype.itemsize, dtype.itemsize) if channels == 3
               else (msg.step, dtype.itemsize))
    return np.ndarray(shape, dtype=dtype, buffer=memoryview(msg.data), strides=strides)


def check_info(info, image, expected_frame):
    if (info.width != image.width or info.height != image.height
            or info.header.frame_id != expected_frame
            or stamp_ns(info) != stamp_ns(image)):
        raise FrameError('CAMERA_INFO_MISMATCH')
    k = np.asarray(info.k, dtype=float).reshape(3, 3)
    if (not np.isfinite(k).all() or not np.isfinite(info.d).all()
            or not np.isfinite(info.r).all() or not np.isfinite(info.p).all()
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not 0 <= k[0, 2] < image.width or not 0 <= k[1, 2] < image.height
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1]) > 1e-6
            or info.binning_x > 1 or info.binning_y > 1
            or any((info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height))):
        raise FrameError('INVALID_INTRINSICS')
    return k


def normalize(pair, settings, now_ns):
    color, depth, color_info, depth_info = pair
    for msg in pair:
        settings.check_stamp(msg, now_ns)
        if msg.header.frame_id != settings.expected_frame:
            raise FrameError('FRAME_MISMATCH_OR_UNREGISTERED_DEPTH')
    if (color.width, color.height) != (depth.width, depth.height):
        raise FrameError('RGB_DEPTH_SIZE_MISMATCH')
    delta = abs(stamp_ns(color)-stamp_ns(depth)) / 1e9
    if delta > settings.sync_tolerance:
        raise FrameError('UNSYNCHRONIZED_IMAGES')
    ck = check_info(color_info, color, settings.expected_frame)
    dk = check_info(depth_info, depth, settings.expected_frame)
    if not np.allclose(ck, dk, rtol=1e-4, atol=1e-3):
        raise FrameError('REGISTERED_INTRINSICS_MISMATCH')
    if color.encoding not in ('rgb8', 'bgr8') or depth.encoding not in ('16UC1', '32FC1'):
        raise FrameError('UNSUPPORTED_ENCODING')
    rgb = image_array(color)
    if color.encoding == 'bgr8':
        rgb = rgb[..., ::-1]
    rgb = np.ascontiguousarray(rgb).copy()
    metres = image_array(depth).astype(np.float32)
    if depth.encoding == '16UC1':
        # Division preserves exact metre boundaries (6000 mm -> 6.0), avoiding
        # float32 multiplication rounding an inclusive depth_max out of range.
        metres /= 1000.0
    valid = np.isfinite(metres) & (metres >= settings.depth_min) & (metres <= settings.depth_max)
    metres[~valid] = np.nan
    if not valid.any():
        raise FrameError('NO_USABLE_DEPTH')
    return rgb, metres, float(valid.mean()), delta


def prepare_rectification():
    """Load the image backend before accepting time-limited input frames."""
    import cv2
    return cv2


def rectify(rgb, metres, color_info, depth_info):
    """Rectify registered images using supplied calibration; no invented focal length."""
    import copy
    cv2 = prepare_rectification()
    ck = np.asarray(color_info.k).reshape(3,3)
    cd, dd = np.asarray(color_info.d), np.asarray(depth_info.d)
    cp, dp = np.asarray(color_info.p).reshape(3,4), np.asarray(depth_info.p).reshape(3,4)
    if (color_info.distortion_model not in ('', 'plumb_bob')
            or depth_info.distortion_model not in ('', 'plumb_bob')
            or cd.size not in (0,4,5,8) or dd.shape != cd.shape
            or not np.allclose(cd, dd, rtol=1e-4, atol=1e-6)
            or not np.allclose(cp, dp, rtol=1e-4, atol=1e-3)
            or not np.allclose(color_info.r, np.eye(3).ravel())
            or not np.allclose(depth_info.r, np.eye(3).ravel())
            or not np.allclose(cp[:,3], 0)):
        raise FrameError('UNSUPPORTED_REGISTERED_CALIBRATION')
    target = cp[:,:3]
    if (not np.isfinite(target).all() or target[0,0] <= 0 or target[1,1] <= 0
            or not 0 <= target[0,2] < rgb.shape[1] or not 0 <= target[1,2] < rgb.shape[0]
            or not np.allclose(target[2], [0,0,1])):
        raise FrameError('INVALID_PROJECTION')
    if np.any(cd) or not np.allclose(ck, target):
        try:
            maps = _rectification_maps(tuple(ck.ravel()), tuple(cd), tuple(target.ravel()),
                                       (rgb.shape[1], rgb.shape[0]))
            rgb = cv2.remap(rgb, *maps, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            metres = cv2.remap(metres, *maps, cv2.INTER_NEAREST,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=float('nan'))
        except cv2.error as exc:
            raise FrameError('RECTIFICATION_FAILED') from exc
    infos = []
    for source in (color_info, depth_info):
        info = copy.deepcopy(source)
        info.k = target.ravel().tolist()
        info.d = [0.0]*5
        info.r = np.eye(3).ravel().tolist()
        info.p = cp.ravel().tolist()
        info.distortion_model = 'plumb_bob'
        infos.append(info)
    return rgb, metres, infos[0], infos[1]


@lru_cache(maxsize=4)
def _rectification_maps(k, d, target, size):
    import cv2
    return cv2.initUndistortRectifyMap(np.array(k).reshape(3,3), np.array(d), np.eye(3),
                                     np.array(target).reshape(3,3), size, cv2.CV_32FC1)
