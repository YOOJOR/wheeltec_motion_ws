#!/usr/bin/env python3
"""Temporary live diagnostics: subscriptions only, no image conversion or motion requests."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import shutil
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from wheeltec_motion_interfaces.msg import MotionStatus


def stamp_ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=120.)
    parser.add_argument('--label', default='manual')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 600:
        parser.error('duration must be in (0, 600] seconds')
    output = Path('/sim/runtime/diagnostics') / time.strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('conavbot_timing_recorder')
    start = time.monotonic()
    stream = (output/'events.jsonl').open('w', buffering=65536)
    stats = defaultdict(lambda: {'count': 0, 'max_gap_s': 0., 'gaps_over_300ms': 0})
    last = {}
    clock = {'stamp_ns': None, 'received': None, 'count': 0}
    previous_tick = [start, None]

    def write(kind, **values):
        stream.write(json.dumps({'kind': kind, 't': time.monotonic()-start,
                                 'wall_ns': time.time_ns(), **values})+'\n')

    def receive(topic, msg, simulated):
        now = time.monotonic()
        source = stamp_ns(msg.header.stamp)
        gap = None if topic not in last else now-last[topic]
        last[topic] = now
        item = stats[topic]
        item['count'] += 1
        if gap is not None:
            item['max_gap_s'] = max(item['max_gap_s'], gap)
            item['gaps_over_300ms'] += int(gap > .3)
        # Raw Gazebo stamps and wall-epoch D455 stamps use different clocks.
        reference = clock['stamp_ns'] if simulated else node.get_clock().now().nanoseconds
        write('message', topic=topic, stamp_ns=source, gap_s=gap,
              clock_domain='simulation' if simulated else 'ros_wall',
              age_s=None if reference is None else (reference-source)/1e9)

    prefix = '/camera/d455/raw/d455/'
    topics = [(Image, '/sim/rgbd/image', True),
              (Image, '/sim/rgbd/depth_image', True),
              (CameraInfo, '/sim/rgbd/camera_info', True),
              (Odometry, '/sim/odometry', True), (Odometry, '/Odometry', False)]
    topics += [(kind, prefix+name, False) for kind, name in (
        (Image, 'color/image_raw'), (Image, 'aligned_depth_to_color/image_raw'),
        (CameraInfo, 'color/camera_info'), (CameraInfo, 'aligned_depth_to_color/camera_info'))]
    for kind, topic, simulated in topics:
        node.create_subscription(kind, topic,
            lambda msg, t=topic, s=simulated: receive(t, msg, s), qos_profile_sensor_data)

    def on_clock(msg):
        clock.update(stamp_ns=stamp_ns(msg.clock), received=time.monotonic(), count=clock['count']+1)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(String, '/conavbot/decision',
                             lambda msg: write('decision', data=json.loads(msg.data)), qos_profile_sensor_data)
    node.create_subscription(MotionStatus, '/wheeltec_motion/status',
                             lambda msg: write('motion', phase=msg.phase, reason=msg.reason), qos_profile_sensor_data)

    def tick():
        now = time.monotonic()
        before, sim_before = previous_tick
        sim = clock['stamp_ns']
        rate = None if sim is None or sim_before is None else (sim-sim_before)/1e9/(now-before)
        write('health', simulated_realtime_ratio=rate, timer_interval_s=now-before,
              clock_silence_s=None if clock['received'] is None else now-clock['received'],
              clock_messages=clock['count'],
              topic_silence_s={t: None if t not in last else now-last[t] for _, t, _ in topics},
              host_load=Path('/proc/loadavg').read_text().strip())
        previous_tick[:] = [now, sim]
        stream.flush()

    node.create_timer(1., tick)
    write('start', label=args.label, duration=args.duration,
          note='Best-effort observer; reception gaps may include recorder scheduling or DDS loss.')
    print(f'Recording to {output}; Ctrl+C finishes early. No motion commands.', flush=True)
    try:
        while rclpy.ok() and time.monotonic()-start < args.duration:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        tick()
        stream.close()
        (output/'summary.json').write_text(json.dumps(dict(stats), indent=2)+'\n')
        runtime = Path('/sim/runtime')
        # Capture the logs that were current at recording end, without changing the session.
        reports = list((runtime/'agent/runs').glob('*/report.json'))
        latest = max(reports, key=lambda p:p.stat().st_mtime) if reports else None
        if latest is not None:
            shutil.copy2(latest, output/'agent_report.json')
        for relative in ('session.log', 'task.log', 'stack.log', 'generated/motion.yaml'):
            source = runtime/relative
            if source.is_file():
                shutil.copy2(source, output/relative.replace('/', '_'))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'Saved {output}', flush=True)


if __name__ == '__main__':
    main()
