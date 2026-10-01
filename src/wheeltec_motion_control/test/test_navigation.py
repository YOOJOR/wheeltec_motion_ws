"""Deterministic navigation watchdog, latest-request and output tests."""
from dataclasses import replace
import math
import unittest

from wheeltec_motion_control.core import Config, Command, Controller, Pose
from wheeltec_motion_control.navigation import NavigationController


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.config = replace(Config(), linear_accel=0.3, angular_accel=1.0)
        self.nav = NavigationController(self.config, 'wheeltec_base')
        self.now = 0.0
        self.epoch = 10**12
        self.nav.enable(self.now, self.epoch)

    def stamp(self, seconds=None):
        return self.epoch+int((self.now if seconds is None else seconds)*1e9)

    def request(self, vx=0.1, wz=0.0, **kw):
        return self.nav.accept(kw.get('components', (vx, 0., 0., 0., 0., wz)),
                               kw.get('stamp', self.stamp()), kw.get('frame', 'wheeltec_base'),
                               self.now, kw.get('ros_now', self.stamp()))

    def tick(self, dt=0.05, pose=True, ros_now=None):
        self.now += dt
        return self.nav.step(self.now, self.stamp() if ros_now is None else ros_now, pose)

    def start_moving(self):
        self.tick()
        self.assertTrue(self.request())
        self.assertGreater(self.tick().vx, 0)

    def test_enable_waits_for_new_request_and_is_idempotent(self):
        self.assertEqual(self.tick(), Command())
        self.assertFalse(self.request(stamp=self.epoch))
        self.start_moving()
        original = (self.nav.received, self.nav.enabled_stamp_ns, self.nav.session, self.nav.command)
        self.nav.enable(self.now, self.stamp())
        self.assertEqual(original, (self.nav.received, self.nav.enabled_stamp_ns, self.nav.session, self.nav.command))

    def test_continuous_identical_fresh_requests_never_settle(self):
        outputs = []
        for _ in range(30):
            self.tick()
            self.assertTrue(self.request(0.15, 0.35))
            outputs.append(self.tick())
        self.assertTrue(all(v.vx > 0 and v.wz > 0 for v in outputs))
        self.assertAlmostEqual(outputs[-1].vx, .15)
        self.assertAlmostEqual(outputs[-1].wz, .35)
        self.assertEqual(self.nav.phase, 'RUNNING')

    def test_request_change_and_reversal_while_moving(self):
        self.start_moving()
        self.assertTrue(self.request(-.15, -.35))
        before = self.nav.command
        after = self.tick()
        self.assertLess(after.vx, before.vx)
        self.assertLess(after.wz, before.wz)
        self.assertTrue(self.nav.enabled)

    def test_older_duplicate_or_malformed_old_packet_cannot_replace_or_refresh(self):
        self.start_moving()
        stamp, received = self.nav.source_stamp_ns, self.nav.received
        for old in (stamp, stamp-1, self.epoch):
            self.assertFalse(self.request(-.15, stamp=old, frame='bad'))
            self.assertEqual(self.nav.target.vx, .1)
            self.assertEqual(self.nav.received, received)
        for _ in range(7):
            self.tick()
        self.assertFalse(self.nav.enabled)
        self.assertEqual(self.nav.command, Command())

    def test_receive_timeout_requires_explicit_enable_and_new_stamp(self):
        self.start_moving()
        old_stamp = self.nav.source_stamp_ns
        for _ in range(7):
            output = self.tick()
        self.assertEqual(output, Command())
        self.assertFalse(self.request())
        self.nav.enable(self.now, self.stamp())
        self.assertFalse(self.request(stamp=old_stamp))
        self.assertEqual(self.tick(), Command())
        self.assertTrue(self.request())
        self.assertGreater(self.tick().vx, 0)

    def test_late_callback_cannot_hide_expired_watchdog(self):
        self.start_moving()
        self.now += .31
        self.assertFalse(self.request())
        self.assertFalse(self.nav.enabled)

    def test_source_age_does_not_restart_at_reception(self):
        for _ in range(4):
            self.tick()
        self.assertTrue(self.request(stamp=self.epoch+1))
        self.tick()
        self.tick()
        self.assertEqual(self.tick(), Command())
        self.assertIn('source timeout', self.nav.reason)

    def test_new_stale_or_future_source_fails_stop(self):
        for offset in (-.31, .101):
            with self.subTest(offset=offset):
                self.setUp()
                self.tick()
                self.assertFalse(self.request(ros_now=self.stamp()-int(offset*1e9)))
                self.assertFalse(self.nav.enabled)
                self.assertEqual(self.nav.command, Command())

    def test_source_clock_jump_fails_stop(self):
        self.start_moving()
        self.assertEqual(self.tick(ros_now=self.epoch-int(1e9)), Command())
        self.assertFalse(self.nav.enabled)

    def test_pose_loss_and_control_gap_are_immediate_stops(self):
        for kwargs in ({'pose': False}, {'dt': .301}, {'dt': -.01}):
            with self.subTest(kwargs=kwargs):
                self.setUp()
                self.start_moving()
                self.assertEqual(self.tick(**kwargs), Command())
                self.assertFalse(self.nav.enabled)
                self.assertEqual(self.nav.target, Command())
                self.assertEqual(self.tick(), Command())

    def test_stop_and_zero_request_bypass_acceleration(self):
        self.start_moving()
        self.assertTrue(self.request(0., 0.))
        self.assertEqual(self.nav.command, Command())
        self.assertEqual(self.tick(), Command())
        self.assertTrue(self.nav.enabled)
        self.request()
        self.tick()
        self.assertEqual(self.nav.disable('stop requested'), Command())
        self.assertFalse(self.request())

    def test_all_nonfinite_and_unsupported_components_fail_stop(self):
        cases = []
        for index in range(6):
            for value in (math.nan, math.inf, -math.inf):
                vector = [.1, 0., 0., 0., 0., .1]
                vector[index] = value
                cases.append(vector)
        for index in (1, 2, 3, 4):
            vector = [.1, 0., 0., 0., 0., .1]
            vector[index] = 1e-12
            cases.append(vector)
        for vector in cases:
            with self.subTest(vector=vector):
                self.setUp()
                self.start_moving()
                self.assertFalse(self.request(components=vector))
                self.assertFalse(self.nav.enabled)
                self.assertEqual(self.nav.command, Command())

    def test_wrong_or_empty_frame_fails_stop(self):
        for frame in ('', 'base_link', 'camera_init', 'body'):
            self.setUp()
            self.start_moving()
            self.assertFalse(self.request(frame=frame))
            self.assertFalse(self.nav.enabled)

    def test_speed_bounds_rejected_and_acceleration_limits_hold(self):
        for vx, wz in ((.150001, 0), (-.150001, 0), (0, .350001), (0, -.350001)):
            self.setUp()
            self.start_moving()
            self.assertFalse(self.request(vx, wz))
            self.assertEqual(self.nav.command, Command())
        self.setUp()
        previous = Command()
        for _ in range(40):
            self.tick()
            previous = self.nav.command
            self.assertTrue(self.request(.15, -.35))
            output = self.tick()
            self.assertLessEqual(abs(output.vx-previous.vx), .3*.05+1e-12)
            self.assertLessEqual(abs(output.wz-previous.wz), 1.*.05+1e-12)
            self.assertLessEqual(abs(output.vx), .15)
            self.assertLessEqual(abs(output.wz), .35)
            self.assertEqual(output.vy, 0)

    def test_pose_fault_latch_survives_recovery_sample(self):
        core = Controller(self.config)
        core.observe(Pose(0, 0, 0), 1., 1.)
        core.observe(Pose(0, 0, 0), 1.1, 1.1)
        sequence = core.fault_sequence
        core.observe(Pose(1, 0, 0), 1.2, 1.2)
        self.assertGreater(core.fault_sequence, sequence)
        self.assertEqual(core.valid_pose_samples, 1)
        core.observe(Pose(1, 0, 0), 1.3, 1.3)
        self.assertEqual(core.valid_pose_samples, 2)

    def test_independent_positive_navigation_configuration(self):
        for name in ('navigation_command_timeout', 'navigation_future_stamp_tolerance'):
            for value in (0., -1., math.nan, math.inf):
                with self.assertRaises(ValueError):
                    replace(self.config, **{name: value}).validate()
        replace(self.config, navigation_command_timeout=.2, pose_timeout=.9).validate()
