"""Latest-only consumer API. Callers must spin this node and request new observations."""
from dataclasses import dataclass
from types import SimpleNamespace
import threading
import time
import numpy as np
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from wheeltec_rgbd_interfaces.msg import RGBDFrame
from .core import FrameError, normalize, stamp_ns
from .node import latest_qos
from .profiles import camera_profile


class NoFreshFrame(RuntimeError):
    pass


@dataclass(frozen=True)
class Observation:
    sequence: int
    stamp_ns: int
    color_stamp_ns: int
    depth_stamp_ns: int
    calibration_source: str
    rgb: np.ndarray
    depth_m: np.ndarray
    color_info: object
    depth_info: object
    valid_depth_fraction: float
    pairing_delta_sec: float

    @property
    def cam_K(self):
        k = self.color_info.k
        return SimpleNamespace(fx=k[0], fy=k[4], cx=k[2], cy=k[5])

    def as_legacy_conav_observation(self):
        """Upstream ROS Agent divides depth by 1000: supply mm only at this boundary."""
        return {'rgb': self.rgb.copy(),
                'depth': np.nan_to_num(self.depth_m, nan=0.0)*1000.0,
                'cam_K': self.cam_K}


class RGBDReader(Node):
    def __init__(self, topic=None, status_topic=None, settings=None, camera='astra_s', **kwargs):
        profile = camera_profile(camera)
        super().__init__('rgbd_reader', **kwargs)
        self.settings = settings or profile.settings
        topic = profile.topic if topic is None else topic
        status_topic = profile.status_topic if status_topic is None else status_topic
        self.lock = threading.Lock()
        self.latest = None
        self.received_at = None
        self.last_stamp = 0
        self.cache_epoch = 0
        self.reason = 'WAITING'
        self.last_status = None
        self.sub_rgbd = self.create_subscription(RGBDFrame, topic, self.receive, latest_qos())
        self.sub_status = self.create_subscription(DiagnosticArray, status_topic, self.status, 1)

    def receive(self, msg):
        try:
            if (stamp_ns(msg) != max(stamp_ns(msg.color), stamp_ns(msg.depth))
                    or msg.header.frame_id != self.settings.expected_frame or not msg.rectified):
                raise FrameError('INVALID_BUNDLE_HEADER')
            rgb, depth, fraction, delta = normalize(
                (msg.color, msg.depth, msg.color_info, msg.depth_info), self.settings,
                self.get_clock().now().nanoseconds)
            with self.lock:
                if stamp_ns(msg) <= self.last_stamp:
                    raise FrameError('DUPLICATE_OR_OUT_OF_ORDER')
                rgb.flags.writeable = depth.flags.writeable = False
                self.latest = Observation(msg.sequence, stamp_ns(msg), stamp_ns(msg.color),
                                          stamp_ns(msg.depth), msg.calibration_source, rgb, depth, msg.color_info,
                                          msg.depth_info, fraction, delta)
                self.received_at = time.monotonic()
                self.last_stamp = stamp_ns(msg)
                self.reason = 'READY'
        except (FrameError, ValueError, TypeError, BufferError, OverflowError) as exc:
            with self.lock:
                self.latest = None
                self.cache_epoch += 1
                self.reason = str(exc)

    def status(self, msg):
        for item in msg.status:
            if item.name == 'wheeltec_camera/rgbd':
                self.last_status = item
        if any(s.name == 'wheeltec_camera/rgbd' and s.level == DiagnosticStatus.ERROR
               for s in msg.status):
            with self.lock:
                # Ignore a delayed error emitted before this observation.
                if self.latest is None or stamp_ns(msg) >= self.latest.stamp_ns:
                    self.latest = None
                    self.cache_epoch += 1
                    self.reason = 'ADAPTER_NOT_READY'

    def read_latest(self, after_stamp_ns=0):
        return self._read_latest_sample(after_stamp_ns)[0]

    def _read_latest_sample(self, after_stamp_ns=0):
        """Atomic observation/receipt/epoch for bounded pose matching."""
        with self.lock:
            obs = self.latest
            if obs is None:
                raise NoFreshFrame(self.reason)
            source_age = (self.get_clock().now().nanoseconds-
                          min(obs.color_stamp_ns, obs.depth_stamp_ns))/1e9
            if (time.monotonic()-self.received_at > self.settings.frame_timeout
                    or source_age > self.settings.frame_timeout
                    or source_age < -self.settings.future_tolerance):
                self.latest = None
                self.cache_epoch += 1
                raise NoFreshFrame('STALE')
            if obs.stamp_ns <= after_stamp_ns:
                raise NoFreshFrame('NO_NEW_OBSERVATION')
            return obs, self.received_at, self.cache_epoch
