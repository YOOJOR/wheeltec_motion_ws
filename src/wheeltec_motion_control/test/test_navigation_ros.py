"""Isolated ROS wire-contract tests: only /motion_test/* and synthetic poses."""
import time
import threading

from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_srvs.srv import SetBool, Trigger
from wheeltec_motion_interfaces.msg import MotionStatus
from wheeltec_motion_control.navigation_client import NavigationClient, decision_velocity
from wheeltec_motion_control.node import MotionNode
from test_ros_integration import RosHarness


class NavigationRosTests(RosHarness):
    def setUp(self):
        super().setUp()
        self.nav_pub = self.plant.create_publisher(TwistStamped, '/motion_test/navigation/cmd_vel', 1)
        self.mode_client = self.plant.create_client(SetBool, '/motion_test/navigation/set_enabled')
        self.stop_client = self.plant.create_client(Trigger, '/motion_test/stop')
        self.statuses = []
        self.status_sub = self.plant.create_subscription(
            MotionStatus, '/motion_test/status', self.statuses.append,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.assertTrue(self.mode_client.wait_for_service(timeout_sec=3))
        self.assertTrue(self.stop_client.wait_for_service(timeout_sec=3))
        self.wait(lambda: self.nav_pub.get_subscription_count() == 1)
        self.wait(lambda: self.controller.core.valid_pose_samples >= 2)

    def mode(self, enabled=True):
        req = SetBool.Request()
        req.data = enabled
        return self.result(self.mode_client.call_async(req))

    def message(self, vx=.1, wz=0., offset=0., frame='wheeltec_base', stamp=None):
        msg = TwistStamped()
        ns = self.plant.get_clock().now().nanoseconds+int(offset*1e9) if stamp is None else stamp
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
        msg.header.frame_id = frame
        msg.twist.linear.x, msg.twist.angular.z = float(vx), float(wz)
        return msg

    def request(self, vx=.1, wz=0., **kwargs):
        msg = self.message(vx, wz, **kwargs)
        self.nav_pub.publish(msg)
        return msg

    def moving(self):
        self.assertTrue(self.mode().success)
        self.request()
        self.wait(lambda: self.plant.commands[-1][0] > 0, .3)

    def idle(self, reason=None):
        self.wait(lambda: not self.controller.navigation.enabled)
        self.expect_zero()
        self.wait(lambda: self.statuses and self.statuses[-1].mode == MotionStatus.IDLE)
        if reason:
            self.assertIn(reason, self.statuses[-1].reason)
        self.assertEqual(self.controller.navigation.target.vx, 0.)

    def test_continuous_same_speed_and_independent_twenty_hz_output(self):
        self.assertTrue(self.mode().success)
        self.expect_zero()
        self.request(.15, .35)
        self.wait(lambda: self.plant.commands[-1][0] > 0, .3)
        begin = len(self.plant.commands)
        for _ in range(12):
            self.request(.15, .35)
            time.sleep(.06)
        outputs = self.plant.commands[begin:]
        self.assertGreaterEqual(len(outputs), 10)
        self.assertTrue(all(x > 0 and z > 0 and y == 0 for x, y, z in outputs))
        self.assertTrue(all(x <= .15 and z <= .35 for x, _, z in outputs))
        self.assertTrue(self.controller.navigation.enabled)
        self.wait(lambda: self.statuses[-1].mode == MotionStatus.NAVIGATION)
        self.assertEqual(self.controller.count_publishers('/motion_test/velocity'), 1)

    def test_moving_update_latest_wins_duplicates_do_not_renew(self):
        self.moving()
        latest = self.request(-.15, -.35)
        ns = latest.header.stamp.sec*10**9+latest.header.stamp.nanosec
        self.wait(lambda: self.controller.navigation.source_stamp_ns == ns)
        received = self.controller.navigation.received
        self.nav_pub.publish(latest)
        self.request(.15, .35, stamp=ns-1)
        time.sleep(.04)
        self.assertEqual(self.controller.navigation.received, received)
        self.assertEqual(self.controller.navigation.target.vx, -.15)
        self.wait(lambda: self.plant.commands[-1][0] < 0, .25)
        self.idle('timeout')

    def test_stop_service_disarms_and_clears_cached_packets(self):
        self.moving()
        old = self.message()
        self.assertTrue(self.result(self.stop_client.call_async(Trigger.Request())).success)
        self.idle('stop service')
        self.nav_pub.publish(old)
        self.request()
        time.sleep(.1)
        self.assertEqual(self.plant.commands[-1], (0, 0, 0))
        self.assertTrue(self.mode().success)
        self.nav_pub.publish(old)
        time.sleep(.05)
        self.assertIsNone(self.controller.navigation.received)
        self.expect_zero()
        self.request()
        self.wait(lambda: self.plant.commands[-1][0] > 0, .3)

    def test_exit_and_idempotent_enable(self):
        self.moving()
        nav = self.controller.navigation
        session, received = nav.session, nav.received
        self.assertTrue(self.mode().success)
        self.assertEqual(nav.session, session)
        self.assertEqual(nav.received, received)
        self.assertTrue(self.mode(False).success)
        self.idle('explicitly disabled')
        self.assertTrue(self.mode().success)
        self.assertEqual(nav.session, session+1)
        time.sleep(.1)
        self.expect_zero()
        self.assertIsNone(nav.received)

    def test_command_timeout_does_not_resume_on_fresh_packets(self):
        self.moving()
        self.idle('timeout')
        self.request()
        time.sleep(.1)
        self.expect_zero()
        self.assertFalse(self.controller.navigation.enabled)

    def test_pose_loss_recovery_and_jump_do_not_replay(self):
        self.moving()
        self.plant.publish_pose = False
        for _ in range(6):
            self.request()
            time.sleep(.06)
        self.idle('pose')
        self.plant.publish_pose = True
        self.wait(lambda: self.controller.core.valid_pose_samples >= 2)
        self.request()
        time.sleep(.05)
        self.expect_zero()
        self.moving()
        self.plant.x += 1.0
        self.idle('pose jump')
        self.wait(lambda: self.controller.core.valid_pose_samples >= 2)
        self.request()
        time.sleep(.05)
        self.expect_zero()

    def test_navigation_and_action_are_mutually_exclusive(self):
        handle = self.send(1.)
        self.assertTrue(handle.accepted)
        self.assertFalse(self.mode().success)
        self.assertFalse(self.mode(False).success)
        self.assertTrue(self.result(self.stop_client.call_async(Trigger.Request())).success)
        self.assertEqual(self.result(handle.get_result_async()).result.code, 5)
        self.wait(lambda: not self.controller.reserved)
        self.assertTrue(self.mode().success)
        self.assertFalse(self.send().accepted)
        self.assertTrue(self.mode(False).success)
        handle = self.send(0.)
        self.assertTrue(handle.accepted)
        self.assertEqual(self.result(handle.get_result_async()).result.code, 0)

    def test_new_invalid_requests_fail_stop_on_wire(self):
        cases = ({'vx': float('nan')}, {'wz': float('inf')}, {'frame': ''},
                 {'frame': 'base_link'}, {'vx': .151}, {'wz': -.351}, {'offset': .2})
        for case in cases:
            with self.subTest(case=case):
                self.moving()
                self.request(**case)
                self.idle()
        for member in ('linear.y', 'linear.z', 'angular.x', 'angular.y'):
            with self.subTest(member=member):
                self.moving()
                msg = self.message()
                group, attr = member.split('.')
                setattr(getattr(msg.twist, group), attr, .001)
                self.nav_pub.publish(msg)
                self.idle('unsupported')

    def test_stale_source_and_source_expiry_use_original_stamp(self):
        self.assertTrue(self.mode().success)
        cutoff = self.controller.navigation.enabled_stamp_ns
        time.sleep(.35)  # WAITING is allowed; no request is cached yet.
        self.request(stamp=cutoff+1)
        self.idle('source timestamp is stale')
        self.assertTrue(self.mode().success)
        cutoff = self.controller.navigation.enabled_stamp_ns
        time.sleep(.21)
        self.request(stamp=cutoff+1)
        self.wait(lambda: self.plant.commands[-1][0] > 0, .08)
        self.idle('source timeout')

    def test_control_timer_gap_fails_stop_before_late_request(self):
        self.moving()
        self.controller.timer.cancel()
        time.sleep(.85)
        self.request()
        self.idle('scheduling gap')
        self.controller.timer.reset()
        self.request()
        time.sleep(.05)
        self.expect_zero()

    def test_zero_velocity_immediately_stops_without_settling_or_exit(self):
        self.moving()
        self.request(0., 0.)
        self.expect_zero()
        self.assertTrue(self.controller.navigation.enabled)
        self.assertEqual(self.controller.navigation.phase, 'ZERO')
        self.request()
        self.wait(lambda: self.plant.commands[-1][0] > 0, .3)

    def test_parameters_are_locked_during_navigation(self):
        self.moving()
        results = self.controller.set_parameters([Parameter('navigation_command_timeout', value=2.)])
        self.assertFalse(results[0].successful)
        self.assertTrue(self.mode(False).success)
        results = self.controller.set_parameters([Parameter('navigation_command_timeout', value=.2)])
        self.assertTrue(results[0].successful)
        self.assertFalse(self.controller.navigation.enabled)

    def test_node_restart_does_not_replay_old_or_new_unsolicited_packets(self):
        self.moving()
        old = self.message()
        self.controller.close()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        self.executor.remove_node(self.controller)
        self.executor.remove_node(self.plant)
        self.controller.destroy_node()
        self.controller = MotionNode()
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.controller)
        self.executor.add_node(self.plant)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.wait(lambda: self.controller.core.valid_pose_samples >= 2)
        self.wait(lambda: self.nav_pub.get_subscription_count() == 1)
        self.nav_pub.publish(old)
        self.request()
        time.sleep(.1)
        self.idle('navigation not enabled')
        self.assertTrue(self.mode().success)
        self.nav_pub.publish(old)
        time.sleep(.05)
        self.expect_zero()
        self.assertIsNone(self.controller.navigation.received)
        self.request()
        self.wait(lambda: self.plant.commands[-1][0] > 0, .3)

    def test_adapter_fresh_decisions_stop_and_explicit_recovery(self):
        adapter = NavigationClient(prefix='/motion_test')
        try:
            adapter.enable()
            self.wait(lambda: adapter.publisher.get_subscription_count() == 1)
            stamp = adapter.get_clock().now().nanoseconds
            adapter.update('forward', stamp)
            self.wait(lambda: self.plant.commands[-1][0] > 0, .3)
            with self.assertRaises(ValueError):
                adapter.update('left', stamp)
            adapter.update('left', adapter.get_clock().now().nanoseconds)
            self.wait(lambda: self.plant.commands[-1][2] > 0, .3)
            adapter.stop()
            self.idle('stop service')
            with self.assertRaises(RuntimeError):
                adapter.update('forward', adapter.get_clock().now().nanoseconds)
            adapter.enable()
            self.expect_zero()
            adapter.update('right', adapter.get_clock().now().nanoseconds)
            self.wait(lambda: self.plant.commands[-1][2] < 0, .3)
            adapter.disable()
            self.idle('explicitly disabled')
        finally:
            adapter.destroy_node()

    def test_adapter_enable_rejection_does_not_stop_existing_action(self):
        handle = self.send(1.)
        self.assertTrue(handle.accepted)
        adapter = NavigationClient(prefix='/motion_test')
        try:
            with self.assertRaises(RuntimeError):
                adapter.enable()
            self.assertTrue(self.controller.reserved)
            self.assertIsNone(self.controller.core.outcome)
            self.assertFalse(self.controller.navigation.enabled)
            self.assertTrue(self.result(self.stop_client.call_async(Trigger.Request())).success)
            self.assertEqual(self.result(handle.get_result_async()).result.code, 5)
        finally:
            adapter.destroy_node()

    def test_adapter_status_loss_and_stale_decision_do_not_publish(self):
        adapter = NavigationClient(prefix='/motion_test')
        try:
            adapter.enable()
            with self.assertRaises(ValueError):
                adapter.update('forward', adapter.get_clock().now().nanoseconds-int(1e9))
            self.expect_zero()
            adapter.destroy_subscription(adapter.subscription)
            # Server remains WAITING; no cached movement is available.
            time.sleep(.65)
            with self.assertRaises(RuntimeError):
                adapter.update('forward', adapter.get_clock().now().nanoseconds)
            self.expect_zero()
            self.assertEqual(decision_velocity('forward'), (.12, 0.))
            self.assertEqual(decision_velocity('left'), (0., .30))
            self.assertEqual(decision_velocity('right'), (0., -.30))
            with self.assertRaises(ValueError):
                decision_velocity('forward', .3, .5)
            adapter.stop()
        finally:
            adapter.destroy_node()
