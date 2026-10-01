"""Latest-request velocity control, with no path/heading controller or queue.

All receive/control times are monotonic seconds; source times are ROS nanoseconds.
The caller owns mode arbitration and pose validation. Every fault disarms navigation.
"""
import math

from .core import Command, clamp


class NavigationController:
    def __init__(self, config, frame):
        config.validate()
        self.config, self.frame = config, frame
        self.enabled = False
        self.session = 0
        self.enabled_stamp_ns = 0
        self.source_stamp_ns = None
        self.received = None
        self.target = self.command = Command()
        self.phase, self.reason = 'IDLE', 'navigation not enabled'
        self.last_tick = None

    def enable(self, now, ros_now_ns):
        # Idempotent: repeated enable cannot renew the watchdog or cached request.
        if self.enabled:
            return
        self.disable('navigation enabled; waiting for a new request')
        self.enabled = True
        self.session += 1
        self.enabled_stamp_ns = ros_now_ns
        self.last_tick = now
        self.phase = 'WAITING'

    def disable(self, reason):
        self.enabled = False
        self.target = self.command = Command()
        self.received = self.source_stamp_ns = None
        self.last_tick = None
        self.phase, self.reason = 'IDLE', reason
        return self.command

    def reject(self, reason):
        self.disable(reason)
        return False

    def accept(self, components, stamp_ns, frame, now, ros_now_ns):
        if not self.enabled:
            return False
        # A late callback must not hide a watchdog expiry before the next tick.
        if not 0 <= now-self.last_tick <= self.config.max_control_gap:
            return self.reject('navigation control scheduling gap')
        if self.received is not None:
            if not 0 <= now-self.received <= self.config.navigation_command_timeout:
                return self.reject('navigation receive timeout')
            age = (ros_now_ns-self.source_stamp_ns)*1e-9
            if age > self.config.navigation_command_timeout:
                return self.reject('navigation source timeout')
            if age < -self.config.navigation_future_stamp_tolerance:
                return self.reject('navigation source clock moved backwards')
        # Delayed previous-session packets and reordered/duplicate packets cannot
        # replace the latest request OR extend its lifetime, even if malformed.
        if (stamp_ns <= self.enabled_stamp_ns or
                (self.source_stamp_ns is not None and stamp_ns <= self.source_stamp_ns)):
            self.reason = 'ignored old or duplicate navigation request'
            return False
        if len(components) != 6 or not all(math.isfinite(v) for v in components):
            return self.reject('non-finite navigation velocity')
        vx, vy, vz, wx, wy, wz = components
        if any(v != 0.0 for v in (vy, vz, wx, wy)):
            return self.reject('unsupported navigation velocity component')
        if frame != self.frame:
            return self.reject('unexpected navigation control frame')
        age = (ros_now_ns-stamp_ns)*1e-9
        if age > self.config.navigation_command_timeout:
            return self.reject('navigation source timestamp is stale')
        if age < -self.config.navigation_future_stamp_tolerance:
            return self.reject('navigation source timestamp is in the future')
        if abs(vx) > self.config.max_linear_speed or abs(wz) > self.config.max_angular_speed:
            return self.reject('navigation velocity exceeds configured limit')
        self.target, self.source_stamp_ns, self.received = Command(vx, 0.0, wz), stamp_ns, now
        self.phase, self.reason = 'RUNNING', 'fresh navigation request accepted'
        # A zero request is an immediate stop, without leaving navigation mode.
        if self.target == Command():
            self.command = Command()
            self.phase = 'ZERO'
        return True

    def step(self, now, ros_now_ns, pose_ready, pose_reason='pose invalid or stale'):
        if not self.enabled:
            return Command()
        dt = now-self.last_tick
        self.last_tick = now
        if dt < 0 or dt > self.config.max_control_gap:
            return self.disable('navigation control scheduling gap')
        if not pose_ready:
            return self.disable(pose_reason)
        if self.received is None:
            return Command()
        if not 0 <= now-self.received <= self.config.navigation_command_timeout:
            return self.disable('navigation receive timeout')
        source_age = (ros_now_ns-self.source_stamp_ns)*1e-9
        if source_age > self.config.navigation_command_timeout:
            return self.disable('navigation source timeout')
        if source_age < -self.config.navigation_future_stamp_tolerance:
            return self.disable('navigation source clock moved backwards')
        c, old, target = self.config, self.command, self.target
        self.command = Command(
            old.vx+clamp(target.vx-old.vx, c.linear_accel*dt),
            0.0,
            old.wz+clamp(target.wz-old.wz, c.angular_accel*dt))
        return self.command
