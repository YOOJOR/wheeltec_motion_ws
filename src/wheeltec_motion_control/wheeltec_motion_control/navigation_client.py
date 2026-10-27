"""Minimal Co-NavGPT2 adapter. No timer, decision replay or automatic re-enable.

Call synchronous methods from the upstream main loop, outside ROS callbacks and
outside another executor. Each update must correspond to a newly processed
observation/decision and carry its original ROS source time.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from wheeltec_motion_interfaces.action import ExecuteMotion
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import SetBool, Trigger
from wheeltec_motion_interfaces.msg import MotionStatus, NavigationPath


class ServiceRejected(RuntimeError):
    """Known rejection: the server did not grant navigation control."""


class ServiceUnavailable(RuntimeError):
    """No request was sent, so there is no uncertain enable to undo."""


class NavigationClient(Node):
    def __init__(self, prefix='/wheeltec_motion', frame='camera_init',
                 forward_speed=.12, turn_speed=.30, command_timeout=3.0,
                 status_timeout=3.0):
        super().__init__('wheeltec_navigation_adapter')
        if not all(math.isfinite(v) and v > 0 for v in (forward_speed, turn_speed, command_timeout, status_timeout)):
            raise ValueError('speeds and timeouts must be finite and positive')
        self.frame, self.forward_speed, self.turn_speed = frame, forward_speed, turn_speed
        self.command_timeout, self.status_timeout = command_timeout, status_timeout
        self.status = None
        self.status_received = None
        self.session = None
        self.last_decision_stamp_ns = 0
        self.publisher = self.create_publisher(NavigationPath, prefix+'/navigation/path', 1)
        self.scan_client = ActionClient(self, ExecuteMotion, prefix+'/execute')
        self.scan_goal_future = self.scan_result_future = None
        self.scan_deadline = None
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

    def enable(self, timeout=5.):
        req = SetBool.Request()
        req.data = True
        started_ns = self.get_clock().now().nanoseconds
        previous_session = self.session
        self.session = None
        result = self.call(self.mode_client, req, timeout)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.02)
            s = self.status
            if (self.fresh_status() and s.mode == s.NAVIGATION and
                    s.stamp.sec*10**9+s.stamp.nanosec >= started_ns):
                self.session = (s.navigation_session, s.navigation_enabled_stamp_ns)
                if self.session != previous_session:
                    self.last_decision_stamp_ns = s.navigation_enabled_stamp_ns
                return result
        raise RuntimeError('fresh navigation status unavailable after enable')

    def disable(self, timeout=5.):
        self.session = None
        req = SetBool.Request()
        req.data = False
        return self.call(self.mode_client, req, timeout)

    def stop(self, timeout=5.):
        self.session = None
        return self.call(self.stop_client, Trigger.Request(), timeout)

    def start_scan(self, timeout=5.):
        if not self.scan_client.server_is_ready():
            raise ServiceUnavailable('scan action server unavailable')
        goal = ExecuteMotion.Goal()
        goal.motion_type, goal.target = goal.ROTATE, 2*math.pi
        goal.max_speed, goal.timeout_sec = self.turn_speed, 45.0
        self.scan_goal_future = self.scan_client.send_goal_async(goal)
        self.scan_deadline = time.monotonic()+timeout

    def scan_complete(self):
        """Poll acceptance/result without blocking the observation loop."""
        self.poll()
        if self.scan_result_future is None:
            if not self.scan_goal_future.done():
                if time.monotonic() > self.scan_deadline:
                    raise RuntimeError('scan goal response timeout')
                return False
            handle = self.scan_goal_future.result()
            if not handle.accepted:
                raise ServiceRejected('scan action rejected')
            self.scan_result_future = handle.get_result_async()
            self.scan_deadline = time.monotonic()+47.0
        if not self.scan_result_future.done():
            if time.monotonic() > self.scan_deadline:
                raise RuntimeError('scan result timeout')
            return False
        response = self.scan_result_future.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or response.result.code != response.result.SUCCEEDED:
            raise RuntimeError('scan failed: '+response.result.message)
        return True

    def update(self, points, decision_stamp_ns):
        """Publish one camera_init path for one newly processed observation."""
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
        msg = NavigationPath()
        msg.path.header.frame_id = self.frame
        msg.path.header.stamp.sec, msg.path.header.stamp.nanosec = divmod(decision_stamp_ns, 10**9)
        msg.max_linear_speed, msg.max_angular_speed = self.forward_speed, self.turn_speed
        for x, y, z in points:
            pose = PoseStamped()
            pose.header = msg.path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(x), float(y), float(z)
            pose.pose.orientation.w = 1.0
            msg.path.poses.append(pose)
        self.publisher.publish(msg)
        self.last_decision_stamp_ns = decision_stamp_ns

    def destroy_node(self):
        self.scan_client.destroy()
        return super().destroy_node()


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
