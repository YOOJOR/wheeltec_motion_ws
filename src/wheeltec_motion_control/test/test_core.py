"""Deterministic plant simulation and fault injection, independent of ROS."""
import math
import unittest
from dataclasses import replace

from wheeltec_motion_control.core import (
    Config, Controller, Goal, Pose, Command, MOVE_LINEAR, ROTATE,
    SUCCEEDED, POSE_INVALID, TIMEOUT, NO_PROGRESS, CANCELED, INTERNAL_ERROR,
    chassis_pose, quaternion_from_rpy, wrap,
)


def simulate(kind, target, yaw=0.0, config=None, disturbance=0.0):
    c = Controller(config or Config())
    x, y, angle, now = 0.0, 0.0, yaw, 10.0
    c.observe(Pose(x, y, wrap(angle)), now, now)
    c.start(Goal(kind, target), now)
    commands = []
    for _ in range(3600):
        now += 0.05
        cmd = c.step(now)
        commands.append(cmd)
        x += (math.cos(angle)*cmd.vx-math.sin(angle)*cmd.vy)*0.05
        y += (math.sin(angle)*cmd.vx+math.cos(angle)*cmd.vy)*0.05
        if abs(cmd.vx) > 0.01:
            y += disturbance*0.05
        angle += cmd.wz*0.05
        c.observe(Pose(x, y, wrap(angle)), now, now)
        if c.outcome is not None:
            break
    return c, Pose(x, y, angle), commands


class GeometryTests(unittest.TestCase):
    def test_translation_is_rotated_in_full_3d(self):
        # 90-degree body pitch rotates a +Z lever arm into world +X.
        p = chassis_pose((1, 2, 3), quaternion_from_rpy(0, math.pi/2, 0),
                         (0, 0, 1), (0, -math.pi/2, 0))
        self.assertAlmostEqual(p.x, 2)
        self.assertAlmostEqual(p.y, 2)
        self.assertAlmostEqual(p.yaw, 0)

    def test_mount_rotation_and_lever_arm(self):
        p = chassis_pose((0, 0, 0), quaternion_from_rpy(0, 0, math.pi/2),
                         (-0.2, 0, 0), (0, 0, math.pi/2))
        self.assertAlmostEqual(p.x, 0)
        self.assertAlmostEqual(p.y, -0.2)
        self.assertAlmostEqual(abs(p.yaw), math.pi)

    def test_invalid_quaternion(self):
        for q in [(0, 0, 0, 0), (0, 0, 0, float('nan')), (0, 0, 0, 2)]:
            with self.assertRaises(ValueError):
                chassis_pose((0, 0, 0), q, (0, 0, 0), (0, 0, 0))


class ControlTests(unittest.TestCase):
    def test_forward_reverse_and_rotated_heading(self):
        for distance in (1.0, -1.0):
            for heading in (0.0, 1.2, -2.5):
                with self.subTest(distance=distance, heading=heading):
                    c, p, _ = simulate(MOVE_LINEAR, distance, heading)
                    self.assertEqual(c.outcome.code, SUCCEEDED)
                    along = p.x*math.cos(heading)+p.y*math.sin(heading)
                    self.assertLessEqual(abs(along-distance), c.config.distance_tolerance)

    def test_rotation_direction_and_wrap(self):
        for target in (math.pi/2, -math.pi/2, 3*math.pi/2, -2*math.pi):
            with self.subTest(target=target):
                c, p, _ = simulate(ROTATE, target, yaw=3.0)
                self.assertEqual(c.outcome.code, SUCCEEDED)
                self.assertLessEqual(abs((p.yaw-3.0)-target), c.config.angle_tolerance)

    def test_optional_mecanum_cross_track_correction(self):
        c, p, commands = simulate(MOVE_LINEAR, 1.0,
                                  config=replace(Config(), lateral_correction=True), disturbance=0.02)
        self.assertEqual(c.outcome.code, SUCCEEDED)
        self.assertLess(abs(p.y), c.config.cross_track_tolerance)
        self.assertTrue(any(cmd.vy < 0 for cmd in commands))

    def test_speed_and_acceleration_limits(self):
        c, _, commands = simulate(MOVE_LINEAR, 1.0)
        previous = Command()
        for cmd in commands:
            self.assertLessEqual(abs(cmd.vx), c.config.max_linear_speed+1e-9)
            if cmd != Command():  # Stop requests intentionally bypass ramping.
                self.assertLessEqual(abs(cmd.vx-previous.vx), c.config.linear_accel*0.05+1e-9)
            previous = cmd

    def prepared(self, config=None, goal=None):
        c = Controller(config or Config())
        c.observe(Pose(0, 0, 0), 10, 10)
        c.start(goal or Goal(MOVE_LINEAR, 1), 10)
        return c

    def test_stale_pose_stops_and_requires_new_goal(self):
        c = self.prepared()
        c.step(10.1)
        self.assertEqual(c.step(10.6), Command())
        self.assertEqual(c.outcome.code, POSE_INVALID)
        c.observe(Pose(0, 0, 0), 10.7, 10.7)
        self.assertEqual(c.step(10.7), Command())
        self.assertEqual(c.outcome.code, POSE_INVALID)

    def test_duplicate_timestamp_fails(self):
        c = self.prepared()
        self.assertFalse(c.observe(Pose(0, 0, 0), 10, 10.1))
        self.assertEqual(c.outcome.code, POSE_INVALID)

    def test_jump_fails(self):
        c = self.prepared()
        c.observe(Pose(1, 0, 0), 10.1, 10.1)
        self.assertEqual(c.outcome.code, POSE_INVALID)
        self.assertEqual(c.step(10.1), Command())

    def test_no_progress(self):
        c = self.prepared(replace(Config(), no_progress_timeout=0.3))
        for i in range(1, 10):
            now = 10+i*0.05
            c.observe(Pose(0, 0, 0), now, now)
            c.step(now)
        self.assertEqual(c.outcome.code, NO_PROGRESS)

    def test_action_timeout(self):
        c = self.prepared(goal=Goal(MOVE_LINEAR, 1, timeout=0.2))
        for i in range(1, 8):
            now = 10+i*0.05
            c.observe(Pose(i*0.001, 0, 0), now, now)
            c.step(now)
        self.assertEqual(c.outcome.code, TIMEOUT)

    def test_zero_goal_does_not_finish_on_repeated_timer_samples(self):
        c = self.prepared(replace(Config(), pose_timeout=2), Goal(MOVE_LINEAR, 0))
        c.observe(Pose(0, 0, 0), 10.1, 10.1)
        for i in range(1, 15):
            c.step(10.1+i*0.05)
        self.assertIsNone(c.outcome)
        c.observe(Pose(0, 0, 0), 10.9, 10.9)
        c.step(10.9)
        self.assertEqual(c.outcome.code, SUCCEEDED)

    def test_overshoot_corrects_direction(self):
        c = self.prepared()
        c.observe(Pose(0.4, 0, 0), 10.1, 10.1)
        c.observe(Pose(0.8, 0, 0), 10.2, 10.2)
        c.observe(Pose(1.1, 0, 0), 10.3, 10.3)
        self.assertLess(c.step(10.3).vx, 0)

    def test_cancel_and_timer_gap(self):
        c = self.prepared()
        c.finish(CANCELED, 'cancel')
        self.assertEqual(c.step(10.1), Command())
        c = self.prepared()
        c.step(10.4)
        self.assertEqual(c.outcome.code, INTERNAL_ERROR)

    def test_invalid_configuration_and_goal(self):
        for config in (replace(Config(), max_linear_speed=0),
                       replace(Config(), angle_tolerance=float('nan')),
                       replace(Config(), max_yaw_jump=math.pi)):
            with self.assertRaises(ValueError):
                Controller(config)
        for goal in (Goal(99, 1), Goal(MOVE_LINEAR, float('inf')),
                     Goal(ROTATE, 10), Goal(MOVE_LINEAR, 1, max_speed=5),
                     Goal(MOVE_LINEAR, 1, timeout=-1)):
            with self.assertRaises(ValueError):
                goal.validate(Config())


if __name__ == '__main__':
    unittest.main()
