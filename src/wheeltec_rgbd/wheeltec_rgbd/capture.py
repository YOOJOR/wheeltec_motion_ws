"""Camera-only fresh-frame smoke check and optional snapshot; no control dependencies."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.utilities import remove_ros_args
from .client import RGBDReader, NoFreshFrame


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--topic', default='/wheeltec_camera/rgbd')
    parser.add_argument('--status-topic', default='/wheeltec_camera/status')
    argv = remove_ros_args(args=args)[1:] if args is not None else remove_ros_args()[1:]
    opts = parser.parse_args(argv)
    if not 0 < opts.duration <= 120:
        parser.error('--duration must be in (0,120]')
    rclpy.init(args=args)
    reader = RGBDReader(topic=opts.topic, status_topic=opts.status_topic)
    start = time.monotonic()
    stamps, deltas, fractions, source_ages = [], [], [], []
    last_stamp, obs = 0, None
    try:
        while rclpy.ok() and time.monotonic()-start < opts.duration:
            rclpy.spin_once(reader, timeout_sec=0.02)
            try:
                current = reader.read_latest(after_stamp_ns=last_stamp)
            except NoFreshFrame:
                continue
            obs = current
            last_stamp = obs.stamp_ns
            stamps.append(time.monotonic())
            deltas.append(obs.pairing_delta_sec)
            fractions.append(obs.valid_depth_fraction)
            source_ages.append((reader.get_clock().now().nanoseconds-obs.depth_stamp_ns)/1e9)
        # A snapshot must still be fresh at the end, not simply the last ever received frame.
        try:
            reader.read_latest()
            fresh = obs is not None
        except NoFreshFrame:
            fresh = False
        result = {'fresh_at_end': fresh, 'frames': len(stamps),
                  'observed_hz': (len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0.0,
                  'pairing_delta_max_sec': max(deltas, default=None),
                  'source_age_max_sec': max(source_ages, default=None),
                  'valid_fraction_mean': float(np.mean(fractions)) if fractions else None,
                  'cmd_vel_publishers': len(reader.get_publishers_info_by_topic('/cmd_vel')),
                  'reason': reader.reason,
                  'adapter_reason': reader.last_status.message if reader.last_status else None,
                  'adapter_values': {v.key: v.value for v in reader.last_status.values}
                                    if reader.last_status else {},
                  'timestamp_origin': 'driver_host_callback',
                  'chassis_extrinsics': 'not_configured'}
        if fresh:
            result.update(color_shape=list(obs.rgb.shape), depth_shape=list(obs.depth_m.shape),
                          calibration_source=obs.calibration_source,
                          frame=obs.color_info.header.frame_id,
                          color_K=list(obs.color_info.k), depth_K=list(obs.depth_info.k),
                          color_D=list(obs.color_info.d), depth_D=list(obs.depth_info.d),
                          depth_min_m=float(np.nanmin(obs.depth_m)),
                          depth_max_m=float(np.nanmax(obs.depth_m)))
        if opts.output:
            opts.output.mkdir(parents=True, exist_ok=True)
            (opts.output/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
            if fresh:
                np.savez_compressed(opts.output/'rgbd.npz', rgb=obs.rgb, depth_m=obs.depth_m,
                                    color_K=np.array(obs.color_info.k).reshape(3,3),
                                    depth_K=np.array(obs.depth_info.k).reshape(3,3),
                                    color_stamp_ns=obs.color_stamp_ns, depth_stamp_ns=obs.depth_stamp_ns)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if not fresh or len(stamps) < 2:
            raise SystemExit(1)
    finally:
        reader.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
