"""Read-only RGB-D/FAST-LIO bridge; explicit extrinsics, no TF or motion output."""
from collections import deque
from dataclasses import dataclass
import threading
import time
import numpy as np
from nav_msgs.msg import Odometry
from .client import RGBDReader, NoFreshFrame
from .core import stamp_ns
from .node import latest_qos


def rigid(value):
    m = np.array(value, dtype=float, copy=True)
    if (m.shape != (4, 4) or not np.isfinite(m).all()
            or not np.allclose(m[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(m[:3, :3].T @ m[:3, :3], np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(m[:3, :3]), 1, atol=1e-7)):
        raise ValueError('INVALID_RIGID_TRANSFORM')
    m.flags.writeable = False
    return m


def transform(position, quaternion):
    p, q = np.asarray(position, float), np.asarray(quaternion, float)
    if (p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all()
            or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .1):
        raise ValueError('INVALID_POSE')
    x, y, z, w = q / np.linalg.norm(q)
    m = np.eye(4)
    m[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    m[:3, 3] = p
    return rigid(m)


class PoseWindow:
    """Bounded nearest-time matching, no extrapolation or invented interpolation.

    Any localization fault latches until reset; callers must also discard old maps.
    """
    def __init__(self, timeout=.5, tolerance=.05, future=.1,
                 position_jump=.5, angle_jump=.8):
        values = (timeout, tolerance, future, position_jump, angle_jump)
        if not all(np.isfinite(v) and v > 0 for v in values):
            raise ValueError('INVALID_POSE_SETTINGS')
        self.timeout, self.tolerance, self.future = timeout, tolerance, future
        self.position_jump, self.angle_jump = position_jump, angle_jump
        self.samples = deque(maxlen=16)
        self.reason = None

    def fail(self, reason):
        self.samples.clear()
        self.reason = reason
        raise NoFreshFrame(reason)

    def reset(self):
        self.samples.clear()
        self.reason = None

    def add(self, stamp, matrix, now_ns, monotonic):
        if self.reason:
            raise NoFreshFrame(self.reason)
        try:
            matrix = rigid(matrix)
        except ValueError:
            self.fail('INVALID_POSE')
        age = (now_ns-stamp)/1e9
        if stamp <= 0 or not -self.future <= age <= self.timeout:
            self.fail('POSE_TIME_INVALID')
        if self.samples:
            prev_stamp, prev, received = self.samples[-1]
            if stamp <= prev_stamp:
                self.fail('POSE_TIME_RESET_OR_OUT_OF_ORDER')
            angle = np.arccos(np.clip((np.trace(prev[:3, :3].T @ matrix[:3, :3])-1)/2, -1, 1))
            if ((stamp-prev_stamp)/1e9 > self.timeout or monotonic-received > self.timeout
                    or np.linalg.norm(matrix[:3, 3]-prev[:3, 3]) > self.position_jump
                    or angle > self.angle_jump):
                self.fail('POSE_DISCONTINUITY')
        self.samples.append((stamp, matrix, monotonic))

    def match(self, stamp, now_ns, monotonic):
        if self.reason or not self.samples:
            raise NoFreshFrame(self.reason or 'WAITING_FOR_POSE')
        latest = self.samples[-1]
        if (monotonic-latest[2] > self.timeout
                or not -self.future <= (now_ns-latest[0])/1e9 <= self.timeout):
            self.fail('POSE_STALE')
        sample = min(self.samples, key=lambda v: abs(v[0]-stamp))
        if abs(sample[0]-stamp)/1e9 > self.tolerance:
            raise NoFreshFrame('NO_TIME_MATCHED_POSE')
        if monotonic-sample[2] > self.timeout or (now_ns-sample[0])/1e9 > self.timeout:
            raise NoFreshFrame('MATCHED_POSE_STALE')
        return sample[0], sample[1]


@dataclass(frozen=True)
class PosedObservation:
    observation: object
    pose_stamp_ns: int
    world_from_base: np.ndarray
    world_from_camera: np.ndarray
    world_frame: str = 'camera_init'

    def camera_points_to_world(self, points):
        points = np.asarray(points, float)
        if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
            raise ValueError('EXPECTED_FINITE_NX3_POINTS')
        return points @ self.world_from_camera[:3, :3].T + self.world_from_camera[:3, 3]


class PosedRGBDReader(RGBDReader):
    def __init__(self, *, body_from_base, base_from_camera,
                 extrinsics_confirmed=False, odom_topic='/Odometry', **kwargs):
        if extrinsics_confirmed is not True:
            raise ValueError('CAMERA_EXTRINSICS_NOT_CONFIRMED')
        self.body_from_base = rigid(body_from_base)
        self.base_from_camera = rigid(base_from_camera)
        super().__init__(**kwargs)
        self.pose_lock = threading.Lock()
        self.poses = PoseWindow()
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.receive_pose, latest_qos())

    def receive_pose(self, msg):
        with self.pose_lock:
            try:
                if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
                    self.poses.fail('POSE_FRAME_MISMATCH')
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                matrix = transform([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                self.poses.add(stamp_ns(msg), matrix, self.get_clock().now().nanoseconds,
                               time.monotonic())
            except ValueError:
                self.poses.samples.clear()
                self.poses.reason = 'INVALID_POSE'
            except NoFreshFrame:
                pass

    def reset_localization(self):
        """Caller must discard its map/task before explicitly accepting a new origin."""
        with self.pose_lock:
            self.poses.reset()
            with self.lock:
                self.latest = None

    def read_latest(self, after_stamp_ns=0):
        with self.pose_lock:
            obs = super().read_latest(after_stamp_ns)
            stamp, world_body = self.poses.match(obs.depth_stamp_ns,
                self.get_clock().now().nanoseconds, time.monotonic())
            world_base = rigid(world_body @ self.body_from_base)
            return PosedObservation(obs, stamp, world_base,
                                    rigid(world_base @ self.base_from_camera))
