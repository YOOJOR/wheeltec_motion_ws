"""Minimal Co-NavGPT2 adapter. No timer, decision replay or automatic re-enable.

Call synchronous methods from the upstream main loop, outside ROS callbacks and
outside another executor. Each update must correspond to a newly processed
observation/decision and carry its original ROS source time.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import SetBool, Trigger
from wheeltec_motion_interfaces.msg import MotionStatus


class ServiceRejected(RuntimeError):
    """Known rejection: the server did not grant navigation control."""


class ServiceUnavailable(RuntimeError):
    """No request was sent, so there is no uncertain enable to undo."""


def decision_velocity(decision, forward_speed=.12, turn_speed=.30):
    if (not all(math.isfinite(v) for v in (forward_speed, turn_speed)) or
            not 0 < forward_speed <= .15 or not 0 < turn_speed <= .35):
        raise ValueError('adapter speeds must fit this vehicle: <=0.15 m/s, <=0.35 rad/s')
    mapping = {'forward': (forward_speed, 0.), 'left': (0., turn_speed),
               'right': (0., -turn_speed), 'reverse': (-forward_speed, 0.)}
    if decision not in mapping:
        raise ValueError('use forward/left/right/reverse; stop uses the stop service')
    return mapping[decision]


class NavigationClient(Node):
    def __init__(self, prefix='/wheeltec_motion', frame='base_link',
                 forward_speed=.12, turn_speed=.30, command_timeout=.3,
                 status_timeout=.6):
        super().__init__('wheeltec_navigation_adapter')
        decision_velocity('forward', forward_speed, turn_speed)
        if not all(math.isfinite(v) and v > 0 for v in (command_timeout, status_timeout)):
            raise ValueError('client timeouts must be finite and positive')
        self.frame, self.forward_speed, self.turn_speed = frame, forward_speed, turn_speed
        self.command_timeout, self.status_timeout = command_timeout, status_timeout
        self.status = None
        self.status_received = None
        self.session = None
        self.last_decision_stamp_ns = 0
        self.publisher = self.create_publisher(TwistStamped, prefix+'/navigation/cmd_vel', 1)
        self.mode_client = self.create_client(SetBool, prefix+'/navigation/set_enabled')
        self.stop_client = self.create_client(Trigger, prefix+'/stop')
        self.subscription = self.create_subscription(
            MotionStatus, prefix+'/status', self.on_status,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_status(self, msg):
        self.status, self.status_received = msg, time.monotonic()
        if (msg.mode != msg.NAVIGATION or
                self.session != (msg.navigation_session, msg.navigation_enabled_stamp_ns)):
            self.session = None

    def poll(self):
        rclpy.spin_once(self, timeout_sec=0.)

    def call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=timeout):
            raise ServiceUnavailable('motion service unavailable; no request will be replayed')
        future = client.call_async(request)
        # Discovery can use its entire bounded window. A newly sent request
        # still needs a bounded response window; never send then wait zero time.
        # This does not change velocity freshness or retry the request.
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            raise RuntimeError('motion service response timeout; state is uncertain, stop before retry')
        result = future.result()
        if not result.success:
            raise ServiceRejected(result.message)
        return result

    def fresh_status(self):
        if self.status is None or self.status_received is None:
            return False
        age = (self.get_clock().now().nanoseconds -
               (self.status.stamp.sec*10**9+self.status.stamp.nanosec))*1e-9
        return (0 <= time.monotonic()-self.status_received <= self.status_timeout and
                -.1 <= age <= self.status_timeout)

    def enable(self, timeout=2.):
        req = SetBool.Request()
        req.data = True
        started_ns = self.get_clock().now().nanoseconds
        try:
            result = self.call(self.mode_client, req, timeout)
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.02)
                s = self.status
                if (self.fresh_status() and s.mode == s.NAVIGATION and
                        s.stamp.sec*10**9+s.stamp.nanosec >= started_ns):
                    session = (s.navigation_session, s.navigation_enabled_stamp_ns)
                    if session != self.session:
                        self.last_decision_stamp_ns = s.navigation_enabled_stamp_ns
                    self.session = session
                    return result
            raise RuntimeError('fresh navigation status unavailable after enable')
        except (ServiceRejected, ServiceUnavailable):
            self.session = None
            raise
        except Exception:
            self.session = None
            # An enable response may have been lost after the server accepted it.
            # Best effort explicit stop; even if unavailable, no velocities follow.
            try:
                self.stop(timeout=.3)
            except RuntimeError:
                pass
            raise

    def disable(self, timeout=2.):
        self.session = None
        req = SetBool.Request()
        req.data = False
        return self.call(self.mode_client, req, timeout)

    def stop(self, timeout=2.):
        self.session = None
        return self.call(self.stop_client, Trigger.Request(), timeout)

    def update(self, decision, decision_stamp_ns):
        """One publish for ONE new decision; never restamp a cached decision."""
        self.poll()
        s = self.status
        if (not self.fresh_status() or self.session is None or s.mode != s.NAVIGATION or
                not s.pose_ready or not s.control_enabled or not s.extrinsics_calibrated or
                self.session != (s.navigation_session, s.navigation_enabled_stamp_ns)):
            self.session = None
            raise RuntimeError('navigation status lost/exited; explicitly stop and re-enable after recovery')
        if type(decision_stamp_ns) is not int:
            raise ValueError('decision timestamp must be integer ROS nanoseconds')
        age = (self.get_clock().now().nanoseconds-decision_stamp_ns)*1e-9
        if (decision_stamp_ns <= self.last_decision_stamp_ns or
                not 0 <= age < self.command_timeout):
            raise ValueError('decision must have a new, fresh original ROS timestamp')
        vx, wz = decision_velocity(decision, self.forward_speed, self.turn_speed)
        msg = TwistStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(decision_stamp_ns, 10**9)
        msg.twist.linear.x, msg.twist.angular.z = vx, wz
        self.publisher.publish(msg)
        self.last_decision_stamp_ns = decision_stamp_ns


def main(args=None):
    parser = argparse.ArgumentParser(description='Navigation mode/status/stop; no motion publishing')
    parser.add_argument('operation', choices=('enable', 'disable', 'stop', 'status'))
    parser.add_argument('--prefix', default='/wheeltec_motion')
    opts, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = NavigationClient(prefix=opts.prefix)
    try:
        if opts.operation == 'status':
            deadline = time.monotonic()+2.
            while not node.fresh_status() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            if not node.fresh_status():
                raise RuntimeError('fresh motion status unavailable')
            print(node.status)
        else:
            print(getattr(node, opts.operation)().message)
    except (RuntimeError, ValueError) as exc:
        print(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
