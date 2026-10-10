"""Validate and publish atomic observations; never publishes chassis commands or TF."""
import copy
from array import array
import signal
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Image, CameraInfo
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from wheeltec_rgbd_interfaces.msg import RGBDFrame
from .calibration import load_camera_info, overlay_calibration
from .core import Settings, PairBuffer, FrameError, normalize, rectify, stamp_ns, prepare_rectification


def latest_qos(depth=1):
    return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT)


class RGBDNode(Node):
    def __init__(self, **kwargs):
        # Lazy OpenCV initialization can exceed the frame deadline on Orin.
        # Complete it before subscriptions admit the first source image.
        prepare_rectification()
        super().__init__('rgbd_adapter', **kwargs)
        defaults = {
            'color_topic': '/wheeltec_camera/raw/color/image_raw',
            'depth_topic': '/wheeltec_camera/raw/depth/image_raw',
            'color_info_topic': '/wheeltec_camera/raw/color/camera_info',
            'depth_info_topic': '/wheeltec_camera/raw/depth/camera_info',
            'output_topic': '/wheeltec_camera/rgbd', 'status_topic': '/wheeltec_camera/status',
            'calibration_source': 'driver_camera_info',
            'hardware_id': 'astra_s', 'timestamp_origin': 'driver_host_callback',
            'color_calibration_url': '', 'depth_calibration_url': '',
        }
        defaults.update(vars(Settings()))
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        cfg = {key: self.get_parameter(key).value for key in vars(Settings())}
        self.settings = Settings(**cfg)
        self.buffer = PairBuffer(self.settings)
        self.color_template = load_camera_info(self.get_parameter('color_calibration_url').value)
        self.depth_template = load_camera_info(self.get_parameter('depth_calibration_url').value)
        self.publisher = self.create_publisher(RGBDFrame, self.get_parameter('output_topic').value, latest_qos())
        self.status_pub = self.create_publisher(DiagnosticArray, self.get_parameter('status_topic').value, 1)
        self.sequence = self.accepted = self.rejected = 0
        self.input_counts = {kind: 0 for kind in PairBuffer.kinds}
        self.processing_max_sec = 0.0
        self.rejection_reasons = {}
        self.last_rx = None
        self.last_source_ns = 0
        self.valid_fraction = 0.0
        self.reason = 'WAITING'
        self.subscriptions_rgbd = []
        for kind in PairBuffer.kinds:
            msg_type = CameraInfo if kind.endswith('info') else Image
            self.subscriptions_rgbd.append(self.create_subscription(
                msg_type, self.get_parameter(kind+'_topic').value,
                lambda msg, k=kind: self.receive(k, msg), latest_qos(4)))
        # Health checks continue even if ROS simulated time stalls.
        self.health_timer = self.create_timer(0.1, self.health,
                                              clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive(self, kind, msg):
        self.input_counts[kind] += 1
        try:
            pair = self.buffer.add(kind, msg, self.get_clock().now().nanoseconds)
            if pair is None:
                return
            now_ns = self.get_clock().now().nanoseconds
            processing_start = time.monotonic()
            pair = overlay_calibration(pair, self.color_template, self.depth_template)
            rgb, metres, fraction, delta = normalize(pair, self.settings, now_ns)
            frame = RGBDFrame()
            color, depth, color_info, depth_info = pair
            rgb, metres, color_info, depth_info = rectify(rgb, metres, color_info, depth_info)
            fraction = float(np.isfinite(metres).mean())
            if fraction == 0:
                raise FrameError('NO_USABLE_RECTIFIED_DEPTH')
            frame.header = copy.deepcopy(max((color, depth), key=stamp_ns).header)
            frame.color.header = copy.deepcopy(color.header)
            frame.color.height, frame.color.width = rgb.shape[:2]
            frame.color.encoding, frame.color.step = 'rgb8', rgb.shape[1]*3
            frame.color.data = array('B', rgb.tobytes())
            frame.depth.header = copy.deepcopy(depth.header)
            frame.depth.height, frame.depth.width = metres.shape
            frame.depth.encoding, frame.depth.step = '32FC1', metres.shape[1]*4
            frame.depth.data = array('B', metres.astype('<f4', copy=False).tobytes())
            frame.color_info, frame.depth_info = color_info, depth_info
            self.sequence += 1
            frame.sequence = self.sequence
            frame.calibration_source = self.get_parameter('calibration_source').value
            frame.rectified = True
            frame.valid_depth_fraction, frame.pairing_delta_sec = fraction, delta
            publish_time = self.get_clock().now().nanoseconds
            for source in pair:
                self.settings.check_stamp(source, publish_time)
            self.publisher.publish(frame)
            self.accepted += 1
            self.last_rx = time.monotonic()
            self.last_source_ns = min(stamp_ns(color), stamp_ns(depth))
            self.valid_fraction = fraction
            self.reason = 'READY'
            self.processing_max_sec = max(self.processing_max_sec, time.monotonic()-processing_start)
        except (FrameError, ValueError, TypeError, BufferError, OverflowError) as exc:
            self.rejected += 1
            self.reason = str(exc) if isinstance(exc, FrameError) else 'INVALID_DATA'
            self.rejection_reasons[self.reason] = self.rejection_reasons.get(self.reason, 0)+1
            self.last_rx = None
            self.buffer.clear()
            self.health()

    def health(self):
        if self.last_rx is not None:
            source_age = (self.get_clock().now().nanoseconds-self.last_source_ns)/1e9
            if (time.monotonic()-self.last_rx > self.settings.frame_timeout
                    or source_age > self.settings.frame_timeout
                    or source_age < -self.settings.future_tolerance):
                self.reason = 'STALE'
                self.last_rx = None
                self.buffer.clear()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='wheeltec_camera/rgbd', hardware_id=self.get_parameter('hardware_id').value)
        status.level = (DiagnosticStatus.OK if self.reason == 'READY' else
                        DiagnosticStatus.WARN if self.reason == 'WAITING' else DiagnosticStatus.ERROR)
        status.message = self.reason
        status.values = [KeyValue(key=k, value=str(v)) for k, v in {
            'accepted': self.accepted, 'rejected': self.rejected,
            'rejection_reasons': self.rejection_reasons,
            'valid_depth_fraction': self.valid_fraction,
            'depth_unit': 'metres', 'timestamp_origin': self.get_parameter('timestamp_origin').value,
            'chassis_extrinsics': 'not_configured',
            'calibration_source': self.get_parameter('calibration_source').value,
            'color_calibration_url': self.get_parameter('color_calibration_url').value,
            'depth_calibration_url': self.get_parameter('depth_calibration_url').value,
            'input_counts': self.input_counts,
            'processing_max_sec': self.processing_max_sec,
        }.items()]
        msg.status = [status]
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RGBDNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # launch may forward a second SIGINT while the ROS context is closing.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
