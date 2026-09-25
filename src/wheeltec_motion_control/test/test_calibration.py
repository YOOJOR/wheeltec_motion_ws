"""Known lever arm, drift, observability and actual Humble rosbag IO tests."""
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from wheeltec_motion_control.calibration import Options, analyze, rotation, main, read_bag
from wheeltec_motion_control.core import quaternion_from_rpy, quaternion_product


def synthetic(tilt=0.0, drift=0.0, different=False):
    rows = []
    mount = quaternion_from_rpy(tilt, 0, 0)
    for segment, angles in enumerate([np.linspace(0, 2*math.pi, 361), np.linspace(2*math.pi, 0, 361)]):
        for k, yaw in enumerate(angles):
            q = quaternion_product(quaternion_from_rpy(0, 0, yaw), mount)
            lever = np.array([-0.2+(0.08 if different and segment else 0), 0.03, -0.4])
            centre = np.array([1+drift*(segment+k/360), 2, 0.0])
            p = centre-rotation(q) @ lever
            rows.append([100+len(rows)*0.1, *p, *q])
    return np.array(rows)


class CalibrationTests(unittest.TestCase):
    def test_known_offset_and_independent_directions(self):
        report = analyze(synthetic(), Options(body_z=-0.4))
        self.assertTrue(report['quality_passed'])
        np.testing.assert_allclose(report['translation_conditioned_on_body_z'], [-0.2, 0.03, -0.4], atol=1e-8)
        self.assertEqual(len(report['segments']), 2)
        self.assertLess(report['left_right_difference_m'], 1e-8)

    def test_unknown_height_never_estimated(self):
        report = analyze(synthetic(), Options())
        self.assertTrue(report['quality_passed'])
        self.assertFalse(report['body_z_measured'])
        self.assertEqual(report['translation_conditioned_on_body_z'][2], 0)

    def test_tilt_requires_measured_height(self):
        self.assertFalse(analyze(synthetic(tilt=0.2), Options())['quality_passed'])
        report = analyze(synthetic(tilt=0.2), Options(body_z=-0.4))
        self.assertTrue(report['quality_passed'])
        np.testing.assert_allclose(report['translation_conditioned_on_body_z'], [-0.2, 0.03, -0.4], atol=1e-8)

    def test_drift_and_direction_disagreement(self):
        self.assertFalse(analyze(synthetic(drift=0.4), Options())['quality_passed'])
        self.assertFalse(analyze(synthetic(different=True), Options())['quality_passed'])
        self.assertFalse(analyze(synthetic()[:361], Options())['quality_passed'])

    def test_invalid_or_unobservable_data(self):
        for kind in ('timestamp', 'nan', 'quaternion', 'jump', 'stationary', 'short'):
            rows = synthetic()
            if kind == 'timestamp': rows[10, 0] = rows[9, 0]
            if kind == 'nan': rows[10, 1] = np.nan
            if kind == 'quaternion': rows[10, 4:] = 0
            if kind == 'jump': rows[10, 1] += 5
            if kind == 'stationary': rows[:, 1:] = rows[0, 1:]
            if kind == 'short': rows = rows[:10]
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                analyze(rows, Options())

    def test_noise_tolerance(self):
        rows = synthetic()
        rows[:, 1:4] += np.random.default_rng(4).normal(0, 0.003, (len(rows), 3))
        report = analyze(rows, Options(body_z=-0.4))
        self.assertTrue(report['quality_passed'])
        np.testing.assert_allclose(report['translation_conditioned_on_body_z'][:2], [-0.2, 0.03], atol=0.002)

    def test_rosbag_cli_report_and_plot(self):
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from nav_msgs.msg import Odometry
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bag = root/'poses_bag'
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            writer.create_topic(rosbag2_py.TopicMetadata(name='/Odometry', type='nav_msgs/msg/Odometry', serialization_format='cdr'))
            for row in synthetic():
                msg = Odometry()
                ns = round(row[0]*1e9)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
                msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = row[1:4]
                q = msg.pose.pose.orientation
                q.x, q.y, q.z, q.w = row[4:8]
                writer.write('/Odometry', serialize_message(msg), ns)
            del writer
            output = root/'result'
            self.assertEqual(main(['--bag', str(bag), '--output', str(output), '--body-z', '-0.4']), 0)
            self.assertTrue((output/'trajectory.png').stat().st_size > 1000)
            self.assertTrue((output/'translation_suggestion.yaml').exists())
            report = json.loads((output/'report.json').read_text())
            self.assertTrue(report['quality_passed'])
            # Never overwrite an existing output directory.
            self.assertEqual(main(['--bag', str(bag), '--output', str(output)]), 1)
            unknown = root/'unknown_height'
            self.assertEqual(main(['--csv', str(output/'poses.csv'), '--output', str(unknown), '--no-plot']), 0)
            self.assertFalse((unknown/'translation_suggestion.yaml').exists())
            with self.assertRaises(ValueError):
                read_bag(bag, '/Odometry', 'wrong', 'body', 'sqlite3')
            with self.assertRaises(ValueError):
                read_bag(bag, '/missing', 'camera_init', 'body', 'sqlite3')


if __name__ == '__main__':
    unittest.main()
