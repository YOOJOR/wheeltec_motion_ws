"""Synthetic publishers in the validation domain; no driver, USB or motion node."""
import threading
import time
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from diagnostic_msgs.msg import DiagnosticArray
from wheeltec_rgbd.node import RGBDNode, latest_qos
from wheeltec_rgbd.client import RGBDReader, NoFreshFrame
from test_core import make_pair


class Harness:
    def __init__(self):
        rclpy.init()
        self.adapter = RGBDNode(parameter_overrides=[
            Parameter('color_topic', value='/rgbd_test/color'),
            Parameter('depth_topic', value='/rgbd_test/depth'),
            Parameter('color_info_topic', value='/rgbd_test/color_info'),
            Parameter('depth_info_topic', value='/rgbd_test/depth_info'),
            Parameter('output_topic', value='/rgbd_test/output'),
            Parameter('status_topic', value='/rgbd_test/status'),
        ])
        self.reader = RGBDReader(topic='/rgbd_test/output', status_topic='/rgbd_test/status')
        self.source = Node('rgbd_test_source')
        self.pubs = {k:self.source.create_publisher(CameraInfo if k.endswith('info') else Image,
                     '/rgbd_test/'+k, latest_qos(4)) for k in self.adapter.buffer.kinds}
        self.statuses = []
        self.status_sub = self.source.create_subscription(DiagnosticArray, '/rgbd_test/status',
                                      lambda m:self.statuses.append(m.status[0].message), 10)
        self.executor = MultiThreadedExecutor(num_threads=2)
        for node in (self.adapter, self.reader, self.source): self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.wait(lambda: all(p.get_subscription_count() for p in self.pubs.values()))
        self.wait(lambda: self.adapter.publisher.get_subscription_count() > 0)

    def wait(self, predicate, timeout=3):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if predicate(): return
            time.sleep(.01)
        raise AssertionError('condition timed out')

    def send(self, fault=None):
        pair = make_pair(stamp=self.source.get_clock().now().nanoseconds)
        if fault == 'frame': pair[1].header.frame_id = 'unregistered'
        for k,msg in zip(self.pubs,pair): self.pubs[k].publish(msg)
        return pair

    def close(self):
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        for node in (self.adapter,self.reader,self.source): node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def h():
    h = Harness()
    try: yield h
    finally: h.close()


def test_atomic_fresh_output_legacy_units_and_no_new_frame(h):
    h.send()
    h.wait(lambda:h.reader.latest is not None)
    obs = h.reader.read_latest()
    assert obs.depth_m[0,2] == 1. and tuple(obs.rgb[0,0]) == (0,1,2)
    assert obs.as_legacy_conav_observation()['depth'][0,2] == 1000.
    assert obs.cam_K.fx == 100.
    with pytest.raises(NoFreshFrame,match='NO_NEW'): h.reader.read_latest(obs.stamp_ns)
    h.send()
    h.wait(lambda:h.reader.latest is not None and h.reader.latest.stamp_ns > obs.stamp_ns)
    assert h.reader.read_latest(obs.stamp_ns).sequence == 2
    assert not h.source.get_publishers_info_by_topic('/cmd_vel')


def test_expiry_no_republication_and_fresh_recovery(h):
    old_pair = h.send()
    h.wait(lambda:h.reader.latest is not None)
    old = h.reader.read_latest()
    h.wait(lambda:'STALE' in h.statuses)
    with pytest.raises(NoFreshFrame): h.reader.read_latest()
    assert h.adapter.accepted == 1
    # Replaying the old source cannot refresh its lifetime.
    for k,msg in zip(h.pubs,old_pair): h.pubs[k].publish(msg)
    h.wait(lambda:h.adapter.rejected > 0)
    with pytest.raises(NoFreshFrame): h.reader.read_latest()
    h.send()
    h.wait(lambda:h.reader.latest is not None)
    assert h.reader.read_latest(old.stamp_ns).sequence == 2


def test_invalid_frame_clears_output_and_requires_new_complete_pair(h):
    h.send()
    h.wait(lambda:h.reader.latest is not None)
    h.send('frame')
    h.wait(lambda:h.adapter.rejected > 0 and h.reader.latest is None)
    with pytest.raises(NoFreshFrame): h.reader.read_latest()
    h.send()
    h.wait(lambda:h.reader.latest is not None)
    assert h.reader.read_latest().sequence == 2


def test_consumer_monotonic_expiry_when_ros_clock_is_frozen(h, monkeypatch):
    from types import SimpleNamespace
    h.send()
    h.wait(lambda:h.reader.latest is not None)
    frozen_ns = h.reader.latest.stamp_ns
    monkeypatch.setattr(h.reader, 'get_clock', lambda: SimpleNamespace(
        now=lambda:SimpleNamespace(nanoseconds=frozen_ns)))
    # Stop consumer callbacks, so neither status nor source-time expiry can invalidate it.
    h.executor.remove_node(h.reader)
    time.sleep(.55)
    with pytest.raises(NoFreshFrame,match='STALE'):
        h.reader.read_latest()


def test_vga_bundle_can_be_consumed_before_expiry(h):
    from array import array
    import numpy as np
    def publish_vga():
        pair = make_pair(stamp=h.source.get_clock().now().nanoseconds)
        for image in pair[:2]: image.width, image.height = 640, 480
        for info in pair[2:]: info.width, info.height = 640, 480
        pair[0].step = 640*3
        pair[0].data = array('B', np.full((480,640,3), 42, np.uint8).tobytes())
        pair[1].step = 640*2
        pair[1].data = array('B', np.full((480,640), 1000, dtype='<u2').tobytes())
        for k,msg in zip(h.pubs,pair): h.pubs[k].publish(msg)
    # Allow best-effort discovery/transport losses without accepting stale output.
    for _ in range(5):
        publish_vga()
        time.sleep(.1)
        try:
            obs = h.reader.read_latest()
            assert obs.rgb.shape == (480,640,3) and np.all(obs.rgb == 42)
            assert np.all(obs.depth_m == 1.)
            return
        except NoFreshFrame:
            pass
    raise AssertionError('No fresh VGA observation')


def test_processing_stall_cannot_publish_expired_observation(h, monkeypatch):
    import wheeltec_rgbd.node as adapter_module
    original = adapter_module.rectify
    def delayed(*args):
        time.sleep(.55)
        return original(*args)
    monkeypatch.setattr(adapter_module,'rectify',delayed)
    h.send()
    h.wait(lambda:h.adapter.rejected > 0)
    assert h.adapter.accepted == 0 and h.reader.latest is None
