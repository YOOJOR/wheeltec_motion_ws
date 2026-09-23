"""ROS 2 Humble action server. No serial access and no TF broadcasting."""

from dataclasses import fields
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from wheeltec_motion_interfaces.action import ExecuteMotion

from .core import (Config, Controller, Goal, Command, chassis_pose, SUCCEEDED,
                   CANCELED, STOPPED, INTERNAL_ERROR)


SETTINGS = {
    'odom_topic': '/Odometry',
    'cmd_vel_topic': '/cmd_vel',
    'action_name': '/wheeltec_motion/execute',
    'stop_service': '/wheeltec_motion/stop',
    'world_frame': 'camera_init',
    'body_frame': 'body',
    'control_enabled': False,
    'extrinsics_calibrated': False,
    'body_from_base_translation': [0.0, 0.0, 0.0],
    'body_from_base_rpy': [0.0, 0.0, 0.0],
}
RESTART_ONLY = {'odom_topic', 'cmd_vel_topic', 'action_name', 'stop_service',
                'control_rate_hz', 'feedback_rate_hz', 'use_sim_time'}


class MotionNode(Node):
    def __init__(self):
        super().__init__('wheeltec_motion_controller')
        self.lock = threading.RLock()
        self.active = None
        self.reserved = False
        self.closing = False
        self.stop_pending = False
        self.done = threading.Event()
        defaults = {f.name: getattr(Config(), f.name) for f in fields(Config)}
        defaults.update(SETTINGS)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.values = {key: self.get_parameter(key).value for key in defaults}
        self.validate_values(self.values)
        self.core = Controller(self.make_config(self.values))
        self.publisher = self.create_publisher(Twist, self.values['cmd_vel_topic'], 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(
            Odometry, self.values['odom_topic'], self.on_pose, qos)
        self.action_group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self, ExecuteMotion, self.values['action_name'],
            execute_callback=self.execute, goal_callback=self.on_goal,
            cancel_callback=self.on_cancel, handle_accepted_callback=self.on_accepted,
            callback_group=self.action_group)
        self.stop_service = self.create_service(
            Trigger, self.values['stop_service'], self.on_stop,
            callback_group=self.action_group)
        self.timer = self.create_timer(1/self.core.config.control_rate_hz, self.tick)
        self.feedback_timer = self.create_timer(
            1/self.core.config.feedback_rate_hz, self.feedback)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.get_logger().info(
            f"Motion server ready; output enabled={self.values['control_enabled']}; "
            f"extrinsics calibrated={self.values['extrinsics_calibrated']}. "
            'No TF or serial port is owned by this node.')

    @staticmethod
    def make_config(values):
        return Config(**{f.name: values[f.name] for f in fields(Config)})

    @classmethod
    def validate_values(cls, values):
        cls.make_config(values).validate()
        for name in ('body_from_base_translation', 'body_from_base_rpy'):
            value = values[name]
            if len(value) != 3 or not all(math.isfinite(v) for v in value):
                raise ValueError(f'{name} must contain three finite numbers')
        for name in ('control_enabled', 'extrinsics_calibrated'):
            if type(values[name]) is not bool:
                raise ValueError(f'{name} must be bool')
        for name in ('odom_topic', 'cmd_vel_topic', 'action_name', 'stop_service',
                     'world_frame', 'body_frame'):
            if not isinstance(values[name], str) or not values[name].strip():
                raise ValueError(f'{name} must be nonempty')

    def on_parameters(self, params):
        with self.lock:
            try:
                if self.reserved:
                    raise ValueError('cancel/stop the action before changing parameters')
                updated = dict(self.values)
                for p in params:
                    if p.name in RESTART_ONLY:
                        raise ValueError(f'{p.name} requires node restart')
                    if p.name not in updated:
                        raise ValueError(f'unknown controller parameter: {p.name}')
                    updated[p.name] = p.value
                self.validate_values(updated)
                if self.values['control_enabled'] and not updated['control_enabled']:
                    self.publish(Command())
                self.values = updated
                # Changing extrinsics/thresholds requires a fresh observation.
                self.core = Controller(self.make_config(updated))
                self.get_logger().info('Parameters updated; waiting for fresh pose.')
                return SetParametersResult(successful=True)
            except (ValueError, TypeError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))

    def on_pose(self, msg):
        with self.lock:
            try:
                if msg.header.frame_id != self.values['world_frame']:
                    raise ValueError('unexpected odometry world frame')
                if msg.child_frame_id != self.values['body_frame']:
                    raise ValueError('unexpected odometry child frame')
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
                age = self.get_clock().now().nanoseconds*1e-9-stamp
                if age > self.core.config.pose_timeout:
                    raise ValueError('odometry source timestamp is stale')
                if age < -self.core.config.future_stamp_tolerance:
                    raise ValueError('odometry source timestamp is in the future')
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                pose = chassis_pose(
                    (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                    self.values['body_from_base_translation'],
                    self.values['body_from_base_rpy'])
                self.core.observe(pose, stamp, time.monotonic())
            except (ValueError, TypeError) as exc:
                self.core.invalidate(str(exc))
            if self.active is not None and self.core.outcome is not None:
                self.publish(Command())
                self.done.set()

    @staticmethod
    def goal_from_request(request):
        return Goal(request.motion_type, request.target, request.max_speed, request.timeout_sec)

    def on_goal(self, request):
        with self.lock:
            try:
                self.goal_from_request(request).validate(self.core.config)
                if self.reserved or self.closing:
                    raise ValueError('controller busy or shutting down')
                if not self.values['control_enabled']:
                    raise ValueError('control_enabled is false')
                if not self.values['extrinsics_calibrated']:
                    raise ValueError('extrinsics_calibrated is false')
                if not self.core.ready(time.monotonic()):
                    raise ValueError(self.core.pose_error or 'pose is stale')
                self.reserved = True
                self.stop_pending = False
                self.done.clear()
                return GoalResponse.ACCEPT
            except (ValueError, TypeError) as exc:
                self.get_logger().warning(f'Goal rejected: {exc}')
                return GoalResponse.REJECT

    def on_accepted(self, handle):
        with self.lock:
            self.active = handle
            try:
                self.core.start(self.goal_from_request(handle.request), time.monotonic())
                if self.stop_pending:
                    self.core.finish(STOPPED, 'stop requested during goal acceptance')
                    self.publish(Command())
                    self.done.set()
                self.get_logger().info(
                    f'Goal started: type={handle.request.motion_type}, target={handle.request.target}')
            except Exception as exc:
                self.core.outcome = None
                self.core.started = time.monotonic()
                self.core.finish(INTERNAL_ERROR, str(exc))
                self.publish(Command())
                self.done.set()
        handle.execute()

    def on_cancel(self, handle):
        with self.lock:
            if handle is self.active and self.core.outcome is None:
                return CancelResponse.ACCEPT
            return CancelResponse.REJECT

    def on_stop(self, request, response):
        del request
        with self.lock:
            if self.reserved:
                self.stop_pending = True
            if self.active is not None and self.core.outcome is None:
                self.core.finish(STOPPED, 'stop service requested')
                self.done.set()
            self.publish(Command())
            response.success = True
            response.message = 'Stop requested; this response is not a physical stop measurement.'
            self.get_logger().info(response.message)
            return response

    def publish(self, command):
        # Disabled means no messages, including idle zero messages.
        if not self.values['control_enabled']:
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = command.vx, command.vy, command.wz
        self.publisher.publish(msg)

    def tick(self):
        with self.lock:
            try:
                if self.active is not None and self.active.is_cancel_requested:
                    self.core.finish(CANCELED, 'action canceled')
                command = self.core.step(time.monotonic()) if self.active else Command()
                self.publish(command)
                if self.active is not None and self.core.outcome is not None:
                    self.done.set()
            except Exception as exc:
                self.get_logger().error(f'Controller exception: {exc}')
                self.core.finish(INTERNAL_ERROR, str(exc))
                self.publish(Command())
                self.done.set()

    def feedback(self):
        with self.lock:
            if self.active is None or self.core.outcome is not None:
                return
            msg = ExecuteMotion.Feedback()
            msg.phase = self.core.phase
            p = self.core.pose
            msg.current_x, msg.current_y, msg.current_yaw = p.x, p.y, p.yaw
            msg.remaining, msg.cross_track_error = self.core.remaining, self.core.cross_track
            msg.elapsed_sec = time.monotonic()-self.core.started
            self.active.publish_feedback(msg)

    def execute(self, handle):
        # Executor has spare threads for pose updates, timers, stop and cancellation.
        while rclpy.ok() and not self.done.wait(0.05):
            pass
        with self.lock:
            if self.core.outcome is None:
                self.core.finish(STOPPED, 'node shutting down')
            result = ExecuteMotion.Result()
            result.code, result.message = self.core.outcome.code, self.core.outcome.message
            p = self.core.pose
            if p is not None:
                result.final_x, result.final_y, result.final_yaw = p.x, p.y, p.yaw
            result.remaining = self.core.remaining
            result.elapsed_sec = time.monotonic()-self.core.started
            if handle.is_active:
                if result.code == SUCCEEDED:
                    handle.succeed()
                elif result.code == CANCELED and handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
            self.get_logger().info(f'Goal finished: code={result.code}, {result.message}')
            self.active, self.reserved = None, False
            self.core.goal = None
            return result

    def close(self):
        with self.lock:
            self.closing = True
            if self.active is not None:
                self.core.finish(STOPPED, 'node shutting down')
            self.done.set()
            if self.context.ok():
                self.publish(Command())


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
        if node is not None:
            node.close()
        executor.shutdown(timeout_sec=2.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
