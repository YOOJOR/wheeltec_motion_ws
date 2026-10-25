"""运动控制：导航速度、相对直行/转角及 ROS 接口，单位 m、rad、s。"""
import math
import signal
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool
from wheeltec_motion_interfaces.action import ExecuteMotion
from wheeltec_motion_interfaces.msg import MotionStatus, NavigationPath

CONTROL_PERIOD, FEEDBACK_PERIOD, FUTURE_TOLERANCE = 0.05, 0.2, 0.1
ZERO = (0.0, 0.0)
DEFAULTS = {
    'control_enabled': False,
    'extrinsics_calibrated': False,
    'navigation_frame': 'base_link',
    'body_from_base_translation': [0.0, 0.0, 0.0],
    'body_from_base_rpy': [0.0, 0.0, 0.0],
    'max_linear_speed': 0.15,
    'max_angular_speed': 0.35,
    'linear_accel': 0.15,
    'angular_accel': 0.5,
    'distance_gain': 0.8,
    'heading_gain': 1.5,
    'rotation_gain': 1.2,
    'path_lookahead': 0.25,
    'distance_tolerance': 0.03,
    'angle_tolerance': 0.035,
    'pose_timeout': 0.5,
    'navigation_command_timeout': 0.3,
    'default_timeout': 45.0,
}


def clamp(value, limit):
    return max(-limit, min(limit, value))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_product(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def chassis_pose(position, quaternion, translation, rpy):
    """完整计算 T_world_body * T_body_base 后取平面位姿。"""
    if not all(math.isfinite(v) for v in (*position, *quaternion)):
        raise ValueError('non-finite odometry')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if abs(norm-1.0) > 0.1:
        raise ValueError('invalid orientation quaternion')
    q = tuple(v/norm for v in quaternion)
    offset = quaternion_product(quaternion_product(q, (*translation, 0.0)),
                                (-q[0], -q[1], -q[2], q[3]))
    cr, cp, cy = (math.cos(v/2) for v in rpy)
    sr, sp, sy = (math.sin(v/2) for v in rpy)
    rotation = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
    x, y, z, w = quaternion_product(q, rotation)
    return (position[0]+offset[0], position[1]+offset[1],
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


class MotionNode(Node):
    def __init__(self):
        super().__init__('wheeltec_motion_controller')
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.p = {}
        for name, default in DEFAULTS.items():
            value = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
            if isinstance(default, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive')
            if isinstance(default, list) and (len(value) != 3 or
                                               not all(math.isfinite(v) for v in value)):
                raise ValueError(f'{name} must contain three finite numbers')
            self.p[name] = value
        if self.p['navigation_frame'] != 'base_link':
            raise ValueError('navigation_frame must be base_link')
        self.pose = None
        self.pose_stamp = self.pose_received = None
        self.pose_error = 'no pose received'
        self.yaw_total = 0.0
        self.mode, self.phase, self.reason = MotionStatus.IDLE, 'IDLE', 'ready'
        self.active = self.goal = None
        self.outcome = None
        self.closing = False
        self.session = self.enabled_stamp_ns = 0
        self.request_stamp_ns = self.request_received = None
        self.path = []
        self.path_limits = ZERO
        self.target = self.output = ZERO
        self.remaining = self.cross_track = 0.0
        self.last_tick = time.monotonic()
        group = ReentrantCallbackGroup()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.odom_subscription = self.create_subscription(
            Odometry, '/Odometry', self.on_pose, qos, callback_group=group)
        self.navigation_subscription = self.create_subscription(
            NavigationPath, '/wheeltec_motion/navigation/path', self.on_navigation,
            qos, callback_group=group)
        self.status_publisher = self.create_publisher(
            MotionStatus, '/wheeltec_motion/status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.server = ActionServer(
            self, ExecuteMotion, '/wheeltec_motion/execute',
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, handle_accepted_callback=self.on_accepted,
            callback_group=group)
        self.stop_service = self.create_service(
            Trigger, '/wheeltec_motion/stop', self.on_stop, callback_group=group)
        self.mode_service = self.create_service(
            SetBool, '/wheeltec_motion/navigation/set_enabled', self.on_navigation_mode,
            callback_group=group)
        self.timer = self.create_timer(CONTROL_PERIOD, self.tick, callback_group=group)
        self.feedback_timer = self.create_timer(FEEDBACK_PERIOD, self.feedback,
                                                callback_group=group)
        self.publish_status()
        self.get_logger().info('Motion ready; YAML parameters require restart.')

    def pose_ready(self, now, ros_now_ns):
        return (self.pose is not None and not self.pose_error and
                now-self.pose_received <= self.p['pose_timeout'] and
                -FUTURE_TOLERANCE <= (ros_now_ns-self.pose_stamp)*1e-9 <= self.p['pose_timeout'])

    def on_pose(self, msg):
        with self.lock:
            try:
                if msg.header.frame_id != 'camera_init' or msg.child_frame_id != 'body':
                    raise ValueError('expected camera_init/body odometry')
                if msg.header.stamp.sec < 0 or not 0 <= msg.header.stamp.nanosec < 10**9:
                    raise ValueError('invalid odometry timestamp')
                stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                if self.pose_stamp is not None and stamp <= self.pose_stamp:
                    raise ValueError('odometry timestamp did not advance')
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                pose = chassis_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                                    self.p['body_from_base_translation'],
                                    self.p['body_from_base_rpy'])
            except ValueError as exc:
                self.pose_error = str(exc)
                self.finish(ExecuteMotion.Result.POSE_INVALID, self.pose_error)
                return
            self.yaw_total += wrap(pose[2]-self.pose[2]) if self.pose is not None else pose[2]
            self.pose, self.pose_stamp = pose, stamp
            self.pose_received, self.pose_error = time.monotonic(), ''

    def publish(self, command):
        self.output = command if self.p['control_enabled'] else ZERO
        if self.p['control_enabled']:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.output
            self.publisher.publish(msg)

    def finish(self, code, reason):
        """结束当前任务；Action 保持占用至结果发出，避免新请求覆盖结果。"""
        self.path = []
        self.target = ZERO
        self.request_stamp_ns = self.request_received = None
        if self.mode == MotionStatus.ACTION:
            if self.outcome is None:
                self.outcome = (code, reason)
                self.phase = 'SUCCEEDED' if code == ExecuteMotion.Result.SUCCEEDED else 'STOPPED'
                self.reason = reason
            self.done.set()
        else:
            self.mode, self.phase, self.reason = MotionStatus.IDLE, 'IDLE', reason
        self.publish(ZERO)
        self.publish_status()

    def on_navigation_mode(self, request, response):
        with self.lock:
            now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
            if self.mode == MotionStatus.ACTION or self.closing:
                response.success, response.message = False, 'action active or shutting down'
                return response
            if not request.data:
                self.finish(ExecuteMotion.Result.STOPPED, 'navigation disabled')
            elif not (self.p['control_enabled'] and self.p['extrinsics_calibrated'] and
                      self.pose_ready(now, ros_now)):
                response.success, response.message = False, 'control disabled or pose unavailable'
                return response
            elif self.mode != MotionStatus.NAVIGATION:
                self.mode, self.phase, self.reason = MotionStatus.NAVIGATION, 'WAITING_PATH', 'waiting for path'
                self.session += 1
                self.enabled_stamp_ns = ros_now
                self.last_tick = now
                self.publish(ZERO)
                self.publish_status()
            response.success, response.message = True, self.reason
            return response

    def on_navigation(self, msg):
        with self.lock:
            if self.mode != MotionStatus.NAVIGATION:
                return
            header = msg.path.header
            stamp = header.stamp.sec*10**9+header.stamp.nanosec
            if stamp <= (self.request_stamp_ns or self.enabled_stamp_ns):
                return
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            limits = (msg.max_linear_speed, msg.max_angular_speed)
            points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.path.poses]
            if (header.stamp.sec < 0 or not 0 <= header.stamp.nanosec < 10**9 or
                    header.frame_id != 'camera_init' or
                    not -FUTURE_TOLERANCE <= age <= self.p['navigation_command_timeout'] or
                    not all(math.isfinite(v) and v > 0 for v in limits) or
                    any(pose.header.frame_id != 'camera_init' or
                        not all(math.isfinite(v) for v in (pose.pose.position.x,
                            pose.pose.position.y, pose.pose.position.z)) for pose in msg.path.poses)):
                self.finish(ExecuteMotion.Result.STOPPED, 'invalid navigation path')
                return
            self.path = []
            for point in points:
                if not self.path or point != self.path[-1]:
                    self.path.append(point)
            self.path_limits = (min(limits[0], self.p['max_linear_speed']),
                                min(limits[1], self.p['max_angular_speed']))
            self.request_stamp_ns, self.request_received = stamp, time.monotonic()
            self.phase, self.reason = 'TRACKING', 'new path accepted'
            if not self.path:
                self.target = ZERO
                self.phase, self.reason = 'WAITING_PATH', 'empty path'
                self.publish(ZERO)
            self.publish_status()

    def follow_path(self):
        """Project onto the polyline, then track a point ahead in arc length."""
        x, y, yaw = self.pose
        end_distance = math.hypot(self.path[-1][0]-x, self.path[-1][1]-y)
        if end_distance <= self.p['distance_tolerance']:
            self.phase, self.reason = 'REACHED', 'path endpoint reached'
            return ZERO
        nearest, segment, fraction = math.inf, 0, 0.0
        for i, (a, b) in enumerate(zip(self.path, self.path[1:])):
            dx, dy = b[0]-a[0], b[1]-a[1]
            t = max(0.0, min(1.0, ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
            distance = math.hypot(a[0]+t*dx-x, a[1]+t*dy-y)
            if distance < nearest:
                nearest, segment, fraction = distance, i, t
        target = self.path[-1]
        remaining, ahead = 0.0, self.p['path_lookahead']
        for i in range(segment, len(self.path)-1):
            a, b = self.path[i], self.path[i+1]
            t = fraction if i == segment else 0.0
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            available = (1-t)*length
            remaining += available
            if ahead > 0:
                if ahead <= available:
                    t += ahead/length
                    target = (a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1]))
                    ahead = 0.0
                else:
                    ahead -= available
        heading = wrap(math.atan2(target[1]-y, target[0]-x)-yaw)
        speed = min(self.path_limits[0], self.p['distance_gain']*max(remaining, end_distance))
        vx = 0.0 if abs(heading) >= math.pi/3 else speed*max(0.0, math.cos(heading))
        return (vx, clamp(self.p['heading_gain']*heading, self.path_limits[1]))

    def on_goal(self, request):
        with self.lock:
            now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
            valid = (request.motion_type in (request.MOVE_LINEAR, request.ROTATE) and
                     all(math.isfinite(v) for v in (request.target, request.max_speed, request.timeout_sec)) and
                     request.max_speed >= 0 and request.timeout_sec >= 0)
            if (not valid or self.mode != MotionStatus.IDLE or self.closing or
                    not self.p['control_enabled'] or not self.p['extrinsics_calibrated'] or
                    not self.pose_ready(now, ros_now)):
                self.get_logger().warning('Goal rejected: invalid request, busy, disabled or pose unavailable')
                return GoalResponse.REJECT
            self.mode, self.phase, self.reason = MotionStatus.ACTION, 'ACCEPTING', 'relative action accepted'
            self.goal, self.origin, self.origin_yaw = request, self.pose, self.yaw_total
            self.started = self.last_tick = now
            self.remaining, self.cross_track = request.target, 0.0
            self.outcome = None
            self.done.clear()
            self.publish_status()
            return GoalResponse.ACCEPT

    def on_accepted(self, handle):
        with self.lock:
            self.active = handle
            if self.outcome is None:
                self.phase = 'RUNNING'
            self.publish_status()
        handle.execute()

    def on_cancel(self, handle):
        with self.lock:
            if handle is self.active and self.outcome is None:
                return CancelResponse.ACCEPT
            return CancelResponse.REJECT

    def on_stop(self, request, response):
        with self.lock:
            self.finish(ExecuteMotion.Result.STOPPED, 'stop requested')
            response.success, response.message = True, self.reason
            return response

    def tick(self):
        with self.lock:
            now, ros_now = time.monotonic(), self.get_clock().now().nanoseconds
            dt, self.last_tick = now-self.last_tick, now
            if self.mode == MotionStatus.IDLE or (self.mode == MotionStatus.ACTION and self.outcome is not None):
                self.publish(ZERO)
                return
            if not self.pose_ready(now, ros_now):
                self.finish(ExecuteMotion.Result.POSE_INVALID, self.pose_error or 'pose timeout')
                return
            p = self.p
            if self.mode == MotionStatus.NAVIGATION:
                if self.request_received is None:
                    self.publish(ZERO)
                    return
                age = (ros_now-self.request_stamp_ns)*1e-9
                if (now-self.request_received > p['navigation_command_timeout'] or
                        not -FUTURE_TOLERANCE <= age <= p['navigation_command_timeout']):
                    self.finish(ExecuteMotion.Result.TIMEOUT, 'navigation request timeout')
                    return
                if not self.path or self.phase == 'REACHED':
                    self.publish(ZERO)
                    return
                self.target = desired = self.follow_path()
                if self.phase == 'REACHED':
                    self.publish(ZERO)
                    self.publish_status()
                    return
            else:
                if self.active is None:
                    return
                if self.active.is_cancel_requested:
                    self.finish(ExecuteMotion.Result.CANCELED, 'action canceled')
                    return
                g = self.goal
                if now-self.started >= (g.timeout_sec or p['default_timeout']):
                    self.finish(ExecuteMotion.Result.TIMEOUT, 'action timeout')
                    return
                if g.motion_type == g.MOVE_LINEAR:
                    dx, dy = self.pose[0]-self.origin[0], self.pose[1]-self.origin[1]
                    cs, sn = math.cos(self.origin[2]), math.sin(self.origin[2])
                    self.remaining = g.target-(cs*dx+sn*dy)
                    self.cross_track = -sn*dx+cs*dy
                    heading = wrap(self.origin[2]-self.pose[2])
                    reached = abs(self.remaining) <= p['distance_tolerance'] and abs(heading) <= p['angle_tolerance']
                    limit = min(g.max_speed or p['max_linear_speed'], p['max_linear_speed'])
                    desired = (clamp(p['distance_gain']*self.remaining, limit),
                               clamp(p['heading_gain']*heading, p['max_angular_speed']))
                else:
                    self.remaining = g.target-(self.yaw_total-self.origin_yaw)
                    reached = abs(self.remaining) <= p['angle_tolerance']
                    limit = min(g.max_speed or p['max_angular_speed'], p['max_angular_speed'])
                    desired = (0.0, clamp(p['rotation_gain']*self.remaining, limit))
                if reached:
                    self.finish(ExecuteMotion.Result.SUCCEEDED, 'target reached')
                    return
            vx, wz = self.output
            linear = vx+clamp(desired[0]-vx, p['linear_accel']*dt)
            if self.mode == MotionStatus.NAVIGATION and desired[0] == 0.0:
                linear = 0.0
            angular = wz+clamp(desired[1]-wz, p['angular_accel']*dt)
            if self.mode == MotionStatus.NAVIGATION:
                linear = clamp(linear, self.path_limits[0])
                angular = clamp(angular, self.path_limits[1])
            self.publish((linear, angular))

    def publish_status(self):
        msg = MotionStatus()
        ros_now = self.get_clock().now()
        msg.stamp = ros_now.to_msg()
        msg.mode, msg.phase, msg.reason = self.mode, self.phase, self.reason
        msg.control_enabled = self.p['control_enabled']
        msg.extrinsics_calibrated = self.p['extrinsics_calibrated']
        msg.pose_ready = self.pose_ready(time.monotonic(), ros_now.nanoseconds)
        msg.navigation_session, msg.navigation_enabled_stamp_ns = self.session, self.enabled_stamp_ns
        msg.navigation_request_age_sec = -1.0 if self.request_received is None else time.monotonic()-self.request_received
        msg.requested_linear_x, msg.requested_angular_z = self.target
        msg.output_linear_x, msg.output_angular_z = self.output
        self.status_publisher.publish(msg)

    def feedback(self):
        with self.lock:
            self.publish_status()
            if self.active is None or self.outcome is not None:
                return
            msg = ExecuteMotion.Feedback()
            msg.phase = self.phase
            msg.current_x, msg.current_y, msg.current_yaw = self.pose
            msg.remaining, msg.cross_track_error = self.remaining, self.cross_track
            msg.elapsed_sec = time.monotonic()-self.started
            self.active.publish_feedback(msg)

    def execute(self, handle):
        while rclpy.ok() and not self.done.wait(0.05):
            pass
        with self.lock:
            if self.outcome is None:
                self.finish(ExecuteMotion.Result.STOPPED, 'node shutting down')
            result = ExecuteMotion.Result()
            result.code, result.message = self.outcome
            result.final_x, result.final_y, result.final_yaw = self.pose
            result.remaining = self.remaining
            result.elapsed_sec = time.monotonic()-self.started
            if result.code == result.SUCCEEDED:
                handle.succeed()
            elif result.code == result.CANCELED:
                handle.canceled()
            else:
                handle.abort()
            self.active = self.goal = None
            self.mode, self.phase = MotionStatus.IDLE, 'IDLE'
            self.publish_status()
            return result

    def close(self):
        with self.lock:
            self.closing = True
            self.finish(ExecuteMotion.Result.STOPPED, 'node shutting down')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = MotionNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.close()
        executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
