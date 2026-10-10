import numpy as np
import pytest
import rclpy
from nav_msgs.msg import Odometry
from wheeltec_rgbd.posed import PoseWindow, PosedObservation, PosedRGBDReader, rigid, transform
from wheeltec_rgbd.client import NoFreshFrame


def test_transform_chain_optical_axes_and_translation():
    # Optical z forward -> base x; optical x right -> base -y; optical y down -> base -z.
    base_camera = np.array([[0,0,1,.2],[-1,0,0,0],[0,-1,0,.3],[0,0,0,1.]])
    world_body = transform([1,2,0], [0,0,2**-.5,2**-.5])
    body_base = transform([-.1,0,-.2], [0,0,0,1])
    wb = rigid(world_body @ body_base)
    obs = PosedObservation(None, 1, wb, rigid(wb @ base_camera))
    np.testing.assert_allclose(obs.camera_points_to_world([[0,0,1]]), [[1,3.1,.1]])
    assert not obs.world_from_camera.flags.writeable


@pytest.mark.parametrize('bad', [np.zeros((4,4)), np.diag([-1,1,1,1]),
                                  np.full((4,4),np.nan), np.eye(3)])
def test_invalid_extrinsics(bad):
    with pytest.raises(ValueError): rigid(bad)


def test_matching_bounded_and_no_extrapolation():
    w = PoseWindow()
    for i in range(20):
        stamp = 1_000_000_000+i*100_000_000
        w.add(stamp, np.eye(4), stamp, i*.1)
    assert len(w.samples) == 16
    assert w.match(stamp-30_000_000, stamp, 1.9)[0] == stamp
    with pytest.raises(NoFreshFrame, match='NO_TIME_MATCHED'): w.match(stamp+60_000_000, stamp, 1.9)


@pytest.mark.parametrize('kind', ['duplicate','backward','jump','rotation','gap','future','stale'])
def test_fault_latches_and_explicit_reset(kind):
    w = PoseWindow()
    w.add(1_000_000_000, np.eye(4), 1_000_000_000, 0.)
    stamp, now, mono, matrix = 1_100_000_000, 1_100_000_000, .1, np.eye(4)
    if kind == 'duplicate': stamp = 1_000_000_000
    if kind == 'backward': stamp = 900_000_000
    if kind == 'jump': matrix[0,3] = .6
    if kind == 'rotation': matrix = transform([0,0,0],[0,0,np.sin(.5),np.cos(.5)])
    if kind == 'gap': stamp = now = 1_600_000_000; mono = .6
    if kind == 'future': stamp = 1_300_000_000
    if kind == 'stale': now = 2_000_000_000
    with pytest.raises(NoFreshFrame): w.add(stamp, matrix, now, mono)
    with pytest.raises(NoFreshFrame): w.add(2_100_000_000, np.eye(4), 2_100_000_000, 2.)
    assert not w.samples
    w.reset()
    w.add(2_100_000_000, np.eye(4), 2_100_000_000, 2.)
    assert w.match(2_100_000_000, 2_100_000_000, 2.)[0] == 2_100_000_000


def test_monotonic_timeout_even_when_ros_clock_frozen():
    w = PoseWindow()
    w.add(1_000_000_000, np.eye(4), 1_000_000_000, 0.)
    with pytest.raises(NoFreshFrame, match='POSE_STALE'): w.match(1_000_000_000, 1_000_000_000, .6)


def test_unconfirmed_camera_extrinsics_refused_before_node_creation():
    with pytest.raises(ValueError, match='NOT_CONFIRMED'):
        PosedRGBDReader(body_from_base=np.eye(4), base_from_camera=np.eye(4))


def test_ros_odometry_subscription_frames_and_reset():
    rclpy.init()
    reader = PosedRGBDReader(body_from_base=np.eye(4), base_from_camera=np.eye(4),
                            extrinsics_confirmed=True, odom_topic='/posed_test/odom')
    pub = reader.create_publisher(Odometry, '/posed_test/odom', 1)
    try:
        import time
        deadline = time.monotonic()+3
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(reader, timeout_sec=.01)
        assert pub.get_subscription_count() > 0
        msg = Odometry()
        msg.header.stamp = reader.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'camera_init', 'body'
        msg.pose.pose.orientation.w = 1.
        pub.publish(msg)
        while not reader.poses.samples and time.monotonic() < deadline:
            rclpy.spin_once(reader, timeout_sec=.01)
        assert len(reader.poses.samples) == 1
        msg.child_frame_id = 'base_link'
        pub.publish(msg)
        while not reader.poses.reason and time.monotonic() < deadline:
            rclpy.spin_once(reader, timeout_sec=.01)
        assert reader.poses.reason == 'POSE_FRAME_MISMATCH'
        reader.reset_localization()
        assert reader.latest is None and not reader.poses.samples and reader.poses.reason is None
        assert not reader.get_publishers_info_by_topic('/cmd_vel')
    finally:
        reader.destroy_node()
        rclpy.shutdown()


def observation_at(stamp):
    from types import SimpleNamespace
    return SimpleNamespace(stamp_ns=stamp, color_stamp_ns=stamp, depth_stamp_ns=stamp)


def test_phase_offset_stream_keeps_progress_without_relaxing_pose_tolerance():
    from wheeltec_rgbd.core import Settings
    w, settings = PoseWindow(), Settings()
    last, matched = 0, []
    # At each 10 Hz read, the newest image is 70 ms ahead of the last pose.
    # The previous read's image gets a pose in the following cycle.
    for i in range(20):
        stamp = 1_000_000_000+i*100_000_000
        now, mono = stamp+80_000_000, i*.1+.08
        w.add(stamp, np.eye(4), now, mono)
        newest = observation_at(stamp+70_000_000)
        try:
            obs, pose_stamp, _ = w.match_observation(newest, mono, 0, last, now, mono, settings)
        except NoFreshFrame as exc:
            assert str(exc) == 'NO_TIME_MATCHED_POSE'
        else:
            assert obs.stamp_ns > last
            assert abs(obs.depth_stamp_ns-pose_stamp) <= 50_000_000
            assert now-obs.stamp_ns <= 200_000_000
            last = obs.stamp_ns
            matched.append(i)
    assert matched == list(range(1,20,2))
    assert w.tolerance == .05 and w.timeout == .5


@pytest.mark.parametrize('invalidator', ['source_age','receipt_age','epoch','reset','consumed'])
def test_pending_image_cannot_survive_expiry_invalidation_reset_or_replay(invalidator):
    from wheeltec_rgbd.core import Settings
    w, settings = PoseWindow(), Settings()
    old = observation_at(1_070_000_000)
    w.add(1_000_000_000,np.eye(4),1_080_000_000,.08)
    with pytest.raises(NoFreshFrame):
        w.match_observation(old,.08,0,0,1_080_000_000,.08,settings)
    assert w.pending is not None
    epoch,last,mono,now=0,0,.18,1_180_000_000
    if invalidator=='source_age':now=1_600_000_000
    if invalidator=='receipt_age':mono=.7
    if invalidator=='epoch':epoch=1
    if invalidator=='reset':w.reset()
    if invalidator=='consumed':last=old.stamp_ns
    # Fresh pose that matches the old pending image; newest image has no pose.
    # Inject directly to isolate pending freshness from PoseWindow.add checks.
    w.samples.clear();w.samples.append((1_100_000_000,np.eye(4),mono))
    if invalidator=='source_age':w.samples.append((1_500_000_000,np.eye(4),mono))
    newest=observation_at(now-10_000_000)
    with pytest.raises(NoFreshFrame,match='NO_TIME_MATCHED_POSE'):
        w.match_observation(newest,mono,epoch,last,now,mono,settings)
    assert w.pending[0] is newest


def test_newest_match_wins_over_pending_and_localization_fault_discards_pending():
    from wheeltec_rgbd.core import Settings
    w,settings=PoseWindow(),Settings()
    w.add(1_000_000_000,np.eye(4),1_080_000_000,.08)
    with pytest.raises(NoFreshFrame):
        w.match_observation(observation_at(1_070_000_000),.08,0,0,1_080_000_000,.08,settings)
    w.add(1_100_000_000,np.eye(4),1_120_000_000,.12)
    newest=observation_at(1_120_000_000)
    obs,_,_=w.match_observation(newest,.12,0,0,1_120_000_000,.12,settings)
    assert obs is newest and w.pending is None
    with pytest.raises(NoFreshFrame):
        w.match_observation(observation_at(1_170_000_000),.18,0,0,1_180_000_000,.18,settings)
    with pytest.raises(NoFreshFrame):w.fail('POSE_DISCONTINUITY')
    assert w.pending is None
    with pytest.raises(NoFreshFrame,match='POSE_DISCONTINUITY'):
        w.match_observation(observation_at(1_200_000_000),.2,0,0,1_200_000_000,.2,settings)


def test_unmatchable_image_in_pose_gap_is_replaced_without_waiting_for_expiry():
    from wheeltec_rgbd.core import Settings
    w,settings=PoseWindow(),Settings()
    w.add(1_000_000_000,np.eye(4),1_080_000_000,.08)
    with pytest.raises(NoFreshFrame):
        w.match_observation(observation_at(1_070_000_000),.08,0,0,1_080_000_000,.08,settings)
    # The pending image lies 70/60 ms from both sides of a 130 ms pose gap.
    # It is fresh but can never match within the unchanged 50 ms tolerance.
    w.add(1_130_000_000,np.eye(4),1_210_000_000,.21)
    replacement=observation_at(1_200_000_000)
    with pytest.raises(NoFreshFrame):
        w.match_observation(replacement,.21,0,0,1_210_000_000,.21,settings)
    assert w.pending[0] is replacement
    w.add(1_230_000_000,np.eye(4),1_310_000_000,.31)
    obs,stamp,_=w.match_observation(observation_at(1_300_000_000),.31,0,0,
                                   1_310_000_000,.31,settings)
    assert obs is replacement and stamp==1_230_000_000
    assert w.pending is None
