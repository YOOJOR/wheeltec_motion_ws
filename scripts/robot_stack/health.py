"""Passive ROS readiness checks; no Twist publisher and no action goals."""
import math
import struct
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from livox_ros_driver2.msg import CustomMsg
from wheeltec_motion_interfaces.action import ExecuteMotion


class Health:
    def __init__(self, config, params):
        self.config, self.params = config, params
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('wheeltec_stack_monitor')
        self.received, self.errors = {}, {}
        self.odom_stamp, self.odom_good = None, 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscriptions = [
            self.node.create_subscription(Float32, config['base_voltage_topic'], self.base, qos),
            self.node.create_subscription(Imu, config['imu_topic'], lambda data: self.raw('imu', data), qos, raw=True),
            self.node.create_subscription(CustomMsg, config['lidar_topic'], lambda data: self.raw('lidar', data), qos, raw=True),
            self.node.create_subscription(Odometry, params.get('odom_topic', '/Odometry'), self.odom, qos),
            self.node.create_subscription(Twist, params.get('cmd_vel_topic', '/cmd_vel'),
                                          lambda msg: self.record('motion'), qos)]
        self.action = ActionClient(self.node, ExecuteMotion, params.get('action_name', '/wheeltec_motion/execute'))

    def record(self, name):
        self.received[name] = time.monotonic()
        self.errors.pop(name, None)

    def base(self, msg):
        if math.isfinite(msg.data):
            self.record('base')

    def check_stamp(self, name, stamp):
        age = self.node.get_clock().now().nanoseconds*1e-9-stamp
        timeout = self.params.get('pose_timeout', 0.5)
        future = self.params.get('future_stamp_tolerance', 0.1)
        if not math.isfinite(age) or age > timeout or age < -future:
            self.errors[name] = f'source timestamp age={age:.3f}s'
            return False
        return True

    def raw(self, name, data):
        try:
            if len(data) < 12 or data[:2] not in (b'\x00\x01', b'\x00\x00'):
                raise ValueError('unsupported serialized header')
            sec, ns = struct.unpack_from(('<' if data[1] else '>')+'iI', data, 4)
            if self.check_stamp(name, sec+ns*1e-9):
                self.record(name)
        except (ValueError, struct.error) as exc:
            self.errors[name] = str(exc)

    def odom(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        reason = None
        if msg.header.frame_id != self.params.get('world_frame', 'camera_init'):
            reason = 'wrong world frame'
        elif msg.child_frame_id != self.params.get('body_frame', 'body'):
            reason = 'wrong body frame'
        elif not self.check_stamp('fastlio', stamp):
            reason = self.errors['fastlio']
        elif self.odom_stamp is not None and stamp <= self.odom_stamp:
            reason = 'odometry timestamp did not advance'
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or abs(math.sqrt(sum(v*v for v in values[3:]))-1) > 0.1:
            reason = 'invalid pose/quaternion'
        if reason:
            self.errors['fastlio'] = reason
            self.odom_good = 0
            return
        self.odom_stamp = stamp
        self.odom_good += 1
        self.record('fastlio')

    def reset(self, name):
        keys = ('imu', 'lidar') if name == 'livox' else (name,)
        for key in keys:
            self.received.pop(key, None)
            self.errors.pop(key, None)
        if name == 'fastlio':
            self.odom_stamp, self.odom_good = None, 0

    def stream(self, name, timeout):
        if name in self.errors:
            return False, self.errors[name]
        elapsed = time.monotonic()-self.received.get(name, -math.inf)
        if elapsed > timeout:
            return False, name+' stream absent/stale'
        return True, ''

    def healthy(self, name):
        timeout = self.config['health_timeout_sec']
        if name == 'livox':
            imu = self.stream('imu', timeout)
            return self.stream('lidar', timeout) if imu[0] else imu
        if name == 'fastlio':
            fresh = self.stream(name, self.params.get('pose_timeout', 0.5))
            if not fresh[0]:
                return fresh
            if self.odom_good < self.config['odom_ready_samples']:
                return False, 'waiting for consecutive valid odometry samples'
            return True, ''
        if name == 'motion':
            if not self.action.server_is_ready():
                return False, 'action server not ready'
            publishers = self.node.get_publishers_info_by_topic(self.params.get('cmd_vel_topic', '/cmd_vel'))
            if len(publishers) != 1 or publishers[0].node_name != 'wheeltec_motion_controller':
                return False, 'cmd_vel publisher conflict or missing controller'
        return self.stream(name, timeout)

    def spin(self, timeout):
        rclpy.spin_once(self.node, timeout_sec=timeout)

    def close(self):
        self.action.destroy()
        self.node.destroy_node()
        rclpy.shutdown()
