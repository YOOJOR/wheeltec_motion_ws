"""ROS-independent geometry and relative-motion controller (metres, radians, seconds)."""

from dataclasses import dataclass, fields
import math

MOVE_LINEAR, ROTATE = 1, 2
SUCCEEDED, CANCELED, POSE_INVALID, TIMEOUT, NO_PROGRESS, STOPPED, INTERNAL_ERROR = range(7)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


def quaternion_product(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def quaternion_from_rpy(roll, pitch, yaw):
    cr, cp, cy = (math.cos(v/2) for v in (roll, pitch, yaw))
    sr, sp, sy = (math.sin(v/2) for v in (roll, pitch, yaw))
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float


def chassis_pose(position, quaternion, translation, rpy):
    """T_world_base = T_world_body * T_body_base, before planar projection.

    translation is the base origin expressed in body; rpy rotates base into body.
    """
    if not all(math.isfinite(v) for v in (*position, *quaternion, *translation, *rpy)):
        raise ValueError('non-finite pose or extrinsic')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-8 or abs(norm - 1.0) > 0.1:
        raise ValueError('invalid orientation quaternion')
    q = tuple(v/norm for v in quaternion)
    rotated = quaternion_product(quaternion_product(q, (*translation, 0.0)),
                                 (-q[0], -q[1], -q[2], q[3]))
    x, y, z, w = quaternion_product(q, quaternion_from_rpy(*rpy))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return Pose(position[0]+rotated[0], position[1]+rotated[1], yaw)


@dataclass(frozen=True)
class Config:
    control_rate_hz: float = 20.0
    feedback_rate_hz: float = 5.0
    max_linear_speed: float = 0.15
    max_lateral_speed: float = 0.05
    max_angular_speed: float = 0.35
    linear_accel: float = 0.15
    angular_accel: float = 0.5
    distance_gain: float = 0.8
    heading_gain: float = 1.5
    lateral_gain: float = 0.6
    rotation_gain: float = 1.2
    distance_tolerance: float = 0.03
    cross_track_tolerance: float = 0.05
    angle_tolerance: float = 0.035
    settle_time: float = 0.4
    stopped_linear_speed: float = 0.025
    stopped_angular_speed: float = 0.04
    pose_timeout: float = 0.5
    future_stamp_tolerance: float = 0.1
    max_position_jump: float = 0.5
    max_yaw_jump: float = 0.8
    default_timeout: float = 45.0
    max_timeout: float = 180.0
    no_progress_timeout: float = 5.0
    linear_progress_epsilon: float = 0.005
    angular_progress_epsilon: float = 0.01
    max_distance: float = 5.0
    max_rotation: float = 2*math.pi
    max_control_gap: float = 0.3
    lateral_correction: bool = False

    def validate(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name == 'lateral_correction':
                if type(value) is not bool:
                    raise ValueError('lateral_correction must be bool')
            elif type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{field.name} must be finite and positive')
        if self.default_timeout > self.max_timeout:
            raise ValueError('default_timeout exceeds max_timeout')
        if self.feedback_rate_hz > self.control_rate_hz:
            raise ValueError('feedback_rate_hz exceeds control_rate_hz')
        if self.max_control_gap <= 1.0/self.control_rate_hz:
            raise ValueError('max_control_gap must exceed the control period')
        if self.max_yaw_jump >= math.pi:
            raise ValueError('max_yaw_jump must be below pi for angle unwrapping')


@dataclass(frozen=True)
class Goal:
    kind: int
    target: float
    max_speed: float = 0.0
    timeout: float = 0.0

    def validate(self, config):
        if self.kind not in (MOVE_LINEAR, ROTATE):
            raise ValueError('unsupported motion_type')
        if not all(math.isfinite(v) for v in (self.target, self.max_speed, self.timeout)):
            raise ValueError('goal values must be finite')
        if self.max_speed < 0 or self.timeout < 0:
            raise ValueError('speed and timeout must be nonnegative')
        bound = config.max_distance if self.kind == MOVE_LINEAR else config.max_rotation
        if abs(self.target) > bound:
            raise ValueError('target exceeds configured bound')
        limit = config.max_linear_speed if self.kind == MOVE_LINEAR else config.max_angular_speed
        if self.max_speed > limit:
            raise ValueError('goal speed exceeds configured limit')
        if self.timeout > config.max_timeout:
            raise ValueError('goal timeout exceeds configured maximum')


@dataclass(frozen=True)
class Command:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass(frozen=True)
class Outcome:
    code: int
    message: str


class Controller:
    """Caller provides a monotonic time; only observe() counts as a new pose."""

    def __init__(self, config):
        config.validate()
        self.config = config
        self.pose = None
        self.stamp = None
        self.received = None
        self.pose_error = 'no pose received'
        self.sequence = 0
        self.yaw_total = 0.0
        self.measured_linear = math.inf
        self.measured_angular = math.inf
        self.goal = None
        self.outcome = None
        self.command = Command()
        self.phase = 'IDLE'
        self.remaining = 0.0
        self.cross_track = 0.0

    def invalidate(self, reason):
        self.pose_error = reason
        if self.goal is not None and self.outcome is None:
            self.finish(POSE_INVALID, reason)

    def observe(self, pose, stamp, now):
        if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.yaw, stamp, now)):
            self.invalidate('non-finite pose')
            return False
        if self.stamp is not None:
            dt = stamp - self.stamp
            if dt <= 0:
                self.invalidate('pose timestamp did not advance; restart after clock reset')
                return False
            distance = math.hypot(pose.x-self.pose.x, pose.y-self.pose.y)
            dyaw = wrap(pose.yaw-self.pose.yaw)
            discontinuity = (distance > self.config.max_position_jump or
                             abs(dyaw) > self.config.max_yaw_jump)
            gap = self.received is not None and now-self.received > self.config.pose_timeout
            if discontinuity or gap:
                self.invalidate('pose jump detected' if discontinuity else 'pose stream interrupted')
                self.measured_linear = self.measured_angular = math.inf
            else:
                self.measured_linear = distance/dt
                self.measured_angular = abs(dyaw)/dt
            self.yaw_total += dyaw
        else:
            self.yaw_total = pose.yaw
        self.pose, self.stamp, self.received = pose, stamp, now
        self.sequence += 1
        # An interrupted goal remains terminal even when the next pose is valid.
        self.pose_error = ''
        return True

    def ready(self, now):
        return (self.pose is not None and not self.pose_error and
                0 <= now-self.received <= self.config.pose_timeout)

    def start(self, goal, now):
        goal.validate(self.config)
        if self.goal is not None and self.outcome is None:
            raise ValueError('controller busy')
        if not self.ready(now):
            raise ValueError(self.pose_error or 'pose is stale')
        self.goal, self.origin, self.origin_yaw = goal, self.pose, self.yaw_total
        self.started = self.last_tick = self.progress_at = now
        self.remaining, self.cross_track = goal.target, 0.0
        self.best_error = abs(goal.target)
        self.settle_since = None
        self.settle_sequence = self.sequence
        self.outcome, self.command, self.phase = None, Command(), 'RUNNING'

    def finish(self, code, message):
        if self.outcome is None:
            self.outcome = Outcome(code, message)
        self.command = Command()
        self.phase = 'SUCCEEDED' if self.outcome.code == SUCCEEDED else 'STOPPED'
        return self.command

    def step(self, now):
        if self.goal is None or self.outcome is not None:
            return Command()
        c, g = self.config, self.goal
        if not self.ready(now):
            return self.finish(POSE_INVALID, self.pose_error or 'pose timeout')
        dt = now-self.last_tick
        self.last_tick = now
        if dt < 0 or dt > c.max_control_gap+1e-9:
            return self.finish(INTERNAL_ERROR, 'control loop timing gap')
        if now-self.started >= (g.timeout or c.default_timeout):
            return self.finish(TIMEOUT, 'action timeout')
        yaw_error = wrap(self.origin.yaw-self.pose.yaw)
        if g.kind == MOVE_LINEAR:
            dx, dy = self.pose.x-self.origin.x, self.pose.y-self.origin.y
            cs, sn = math.cos(self.origin.yaw), math.sin(self.origin.yaw)
            self.remaining = g.target-(cs*dx+sn*dy)
            self.cross_track = -sn*dx+cs*dy
            error = max(abs(self.remaining), abs(self.cross_track))
            epsilon = c.linear_progress_epsilon
            within = (abs(self.remaining) <= c.distance_tolerance and
                      abs(self.cross_track) <= c.cross_track_tolerance and
                      abs(yaw_error) <= c.angle_tolerance)
            speed = clamp(c.distance_gain*self.remaining, g.max_speed or c.max_linear_speed)
            if c.lateral_correction:
                lateral = clamp(-c.lateral_gain*self.cross_track, c.max_lateral_speed)
                # Transform start-frame velocity into current chassis axes.
                vx = math.cos(yaw_error)*speed-math.sin(yaw_error)*lateral
                vy = math.sin(yaw_error)*speed+math.cos(yaw_error)*lateral
                vx, vy = clamp(vx, g.max_speed or c.max_linear_speed), clamp(vy, c.max_lateral_speed)
            else:
                vx, vy = speed, 0.0
            desired = Command(vx, vy, clamp(c.heading_gain*yaw_error, c.max_angular_speed))
        else:
            self.remaining = g.target-(self.yaw_total-self.origin_yaw)
            error, epsilon = abs(self.remaining), c.angular_progress_epsilon
            within = error <= c.angle_tolerance
            desired = Command(wz=clamp(c.rotation_gain*self.remaining,
                                       g.max_speed or c.max_angular_speed))
        if within:
            self.phase = 'SETTLING'
            self.command = Command()
            # Success must span distinct incoming samples, never repeated timer ticks.
            stopped = (self.measured_linear <= c.stopped_linear_speed and
                       self.measured_angular <= c.stopped_angular_speed)
            if not stopped:
                self.settle_since = None
            elif self.settle_since is None:
                self.settle_since, self.settle_sequence = now, self.sequence
            elif now-self.settle_since >= c.settle_time and self.sequence > self.settle_sequence:
                return self.finish(SUCCEEDED, 'target reached and settled')
            self.progress_at = now
            return self.command
        self.settle_since, self.phase = None, 'RUNNING'
        if error < self.best_error-epsilon:
            self.best_error, self.progress_at = error, now
        elif now-self.progress_at >= c.no_progress_timeout:
            return self.finish(NO_PROGRESS, 'no measurable progress')
        old = self.command
        self.command = Command(
            old.vx+clamp(desired.vx-old.vx, c.linear_accel*dt),
            old.vy+clamp(desired.vy-old.vy, c.linear_accel*dt),
            old.wz+clamp(desired.wz-old.wz, c.angular_accel*dt))
        return self.command
