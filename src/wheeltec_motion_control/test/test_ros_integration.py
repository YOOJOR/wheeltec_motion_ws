"""Real Humble Action/Topic/Service tests using a synthetic planar chassis.

Runs in an isolated ROS domain. Never uses /cmd_vel or live FAST-LIO data.
"""
import math
import threading
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from wheeltec_motion_interfaces.action import ExecuteMotion
from wheeltec_motion_control.node import MotionNode


class Plant(Node):
    def __init__(self):
        super().__init__('motion_test_plant')
        self.publisher = self.create_publisher(Odometry, '/motion_test/odom', 10)
        self.sub = self.create_subscription(Twist, '/motion_test/velocity', self.command, 10)
        self.vx = self.vy = self.wz = self.x = self.y = self.yaw = 0.0
        self.commands = []
        self.publish_pose = True
        self.freeze_motion = False
        self.frame = 'camera_init'
        self.stamp_offset = 0.0
        self.previous = time.monotonic()
        self.timer = self.create_timer(0.02, self.tick)

    def command(self, msg):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z
        self.commands.append((self.vx, self.vy, self.wz))

    def tick(self):
        now = time.monotonic()
        dt = now-self.previous
        self.previous = now
        if not self.freeze_motion:
            self.x += (math.cos(self.yaw)*self.vx-math.sin(self.yaw)*self.vy)*dt
            self.y += (math.sin(self.yaw)*self.vx+math.cos(self.yaw)*self.vy)*dt
            self.yaw += self.wz*dt
        if self.publish_pose:
            msg = Odometry()
            stamp = self.get_clock().now().nanoseconds+int(self.stamp_offset*1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.header.frame_id, msg.child_frame_id = self.frame, 'body'
            msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(self.yaw/2), math.cos(self.yaw/2)
            self.publisher.publish(msg)


class RosIntegrationTests(unittest.TestCase):
    def setUp(self):
        rclpy.init(args=[
            '--ros-args', '-p', 'control_enabled:=true', '-p', 'extrinsics_calibrated:=true',
            '-p', 'odom_topic:=/motion_test/odom', '-p', 'cmd_vel_topic:=/motion_test/velocity',
            '-p', 'action_name:=/motion_test/execute', '-p', 'stop_service:=/motion_test/stop',
            '-p', 'settle_time:=0.12', '-p', 'pose_timeout:=0.25',
            '-p', 'no_progress_timeout:=0.6', '-p', 'max_control_gap:=0.8',
        ])
        self.controller = MotionNode()
        self.plant = Plant()
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.controller)
        self.executor.add_node(self.plant)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.client = ActionClient(self.plant, ExecuteMotion, '/motion_test/execute')
        self.assertTrue(self.client.wait_for_server(timeout_sec=5))
        self.wait(lambda: self.controller.core.ready(time.monotonic()))

    def tearDown(self):
        self.controller.close()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.client.destroy()
        self.plant.destroy_node()
        self.controller.destroy_node()
        rclpy.shutdown()

    def wait(self, predicate, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(0.01)
        self.fail('condition timed out')

    def result(self, future, timeout=8):
        self.wait(future.done, timeout)
        return future.result()

    def send(self, target=0.1, kind=1, timeout=0.0):
        goal = ExecuteMotion.Goal()
        goal.motion_type, goal.target, goal.timeout_sec = kind, float(target), float(timeout)
        return self.result(self.client.send_goal_async(goal))

    def expect_zero(self):
        self.wait(lambda: self.plant.commands and self.plant.commands[-1] == (0, 0, 0))

    def test_successive_forward_reverse_and_rotation(self):
        for kind, target in [(1, 0.12), (1, -0.12), (2, 0.35), (2, -0.35)]:
            handle = self.send(target, kind)
            self.assertTrue(handle.accepted)
            result = self.result(handle.get_result_async()).result
            self.assertEqual(result.code, result.SUCCEEDED, result.message)
            self.expect_zero()

    def test_busy_rejected_cancel_then_new_goal(self):
        handle = self.send(1)
        self.assertTrue(handle.accepted)
        self.assertFalse(self.send(1).accepted)
        updated = self.controller.set_parameters([Parameter('max_linear_speed', value=0.1)])
        self.assertFalse(updated[0].successful)
        self.assertTrue(self.result(handle.cancel_goal_async()).goals_canceling)
        result = self.result(handle.get_result_async()).result
        self.assertEqual(result.code, result.CANCELED)
        self.expect_zero()
        handle = self.send(0)
        self.assertEqual(self.result(handle.get_result_async()).result.code, 0)

    def test_pose_loss_and_stop_service(self):
        handle = self.send(1)
        self.wait(lambda: any(abs(v[0]) > 0 for v in self.plant.commands))
        self.plant.publish_pose = False
        result = self.result(handle.get_result_async()).result
        self.assertEqual(result.code, result.POSE_INVALID)
        self.expect_zero()
        self.plant.publish_pose = True
        self.wait(lambda: self.controller.core.ready(time.monotonic()))
        handle = self.send(1)
        service = self.plant.create_client(Trigger, '/motion_test/stop')
        self.assertTrue(service.wait_for_service(timeout_sec=3))
        self.assertTrue(self.result(service.call_async(Trigger.Request())).success)
        self.assertEqual(self.result(handle.get_result_async()).result.code, result.STOPPED)
        self.expect_zero()

    def test_frame_and_source_timestamp_checks(self):
        for fault in ('frame', 'stamp'):
            handle = self.send(1)
            self.assertTrue(handle.accepted)
            if fault == 'frame':
                self.plant.frame = 'wrong_frame'
            else:
                self.plant.stamp_offset = -2.0
            result = self.result(handle.get_result_async()).result
            self.assertEqual(result.code, result.POSE_INVALID)
            self.expect_zero()
            self.plant.frame, self.plant.stamp_offset = 'camera_init', 0.0
            self.wait(lambda: self.controller.core.ready(time.monotonic()))

    def test_timeout_no_progress_and_invalid_goal(self):
        self.assertFalse(self.send(float('nan')).accepted)
        self.plant.freeze_motion = True
        handle = self.send(1, timeout=0.2)
        self.assertEqual(self.result(handle.get_result_async()).result.code, 3)
        handle = self.send(1)
        self.assertEqual(self.result(handle.get_result_async()).result.code, 4)
        self.expect_zero()

    def test_parameter_validation_and_output_gate(self):
        bad = self.controller.set_parameters([Parameter('max_linear_speed', value=-1.0)])
        self.assertFalse(bad[0].successful)
        bad = self.controller.set_parameters([Parameter('control_rate_hz', value=10.0)])
        self.assertFalse(bad[0].successful)
        ok = self.controller.set_parameters([Parameter('control_enabled', value=False)])
        self.assertTrue(ok[0].successful)
        time.sleep(0.15)  # Drain the disable-time zero command.
        count = len(self.plant.commands)
        self.assertFalse(self.send(0.1).accepted)
        time.sleep(0.15)
        self.assertEqual(count, len(self.plant.commands))
        ok = self.controller.set_parameters([Parameter('control_enabled', value=True),
                                             Parameter('extrinsics_calibrated', value=False)])
        self.assertTrue(all(r.successful for r in ok))
        self.assertFalse(self.send(0.1).accepted)


if __name__ == '__main__':
    unittest.main()
