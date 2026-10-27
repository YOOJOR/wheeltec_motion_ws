#!/usr/bin/env python3
"""Read-only 2 Hz terminal view of the simulator's aggregated status."""
import json
import math
import shutil
import sys
import time
import unicodedata

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def number(value, unit='', scale=1.0, precision=1):
    return '--' if value is None else f'{value * scale:.{precision}f}{unit}'


def fit_terminal(line, width):
    used = 0
    result = []
    for character in line:
        used += 2 if unicodedata.east_asian_width(character) in ('W', 'F') else 1
        if used > width:
            break
        result.append(character)
    return ''.join(result)


def goal_text(goal):
    if goal is None:
        return '--'
    values = (goal['x'], goal['y']) if isinstance(goal, dict) else goal[:2]
    return '(' + ', '.join(f'{value:.2f}' for value in values) + ') m'


class StatusWatch(Node):
    def __init__(self):
        super().__init__('conavbot_sim_status_watch')
        self.status = None
        self.received_at = None
        self.run_id = None
        self.event_seq = -1
        self.events = []
        self.tty = sys.stdout.isatty()
        self.view_lines = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/sim/exploration_status', self.receive, qos)
        self.create_timer(0.5, self.render)

    def receive(self, message):
        self.status = json.loads(message.data)
        self.received_at = time.monotonic()
        run_id = self.status['run_id']
        if run_id != self.run_id:
            self.run_id = run_id
            self.event_seq = -1
        seq = self.status['event_seq']
        if seq > self.event_seq:
            self.event_seq = seq
            if self.status['event']:
                self.events.append(f"事件 [{run_id[:8] if run_id else '--'} #{seq}] {self.status['event']}")

    def render(self):
        if self.tty and self.view_lines:
            sys.stdout.write(f'\033[{self.view_lines}A\r\033[J')
        for event in self.events:
            print(event)
        self.events.clear()
        if self.status is None:
            lines = ['等待状态：/sim/exploration_status（Ctrl+C 退出查看）']
        else:
            s = self.status
            disconnected = time.monotonic() - self.received_at > 3.0
            stale = disconnected or s['stale']
            motion = s['motion']
            velocity_stale = stale or motion['stale']
            state = '状态失联' if disconnected else ('状态过期' if stale else (s['phase_zh'] or '--'))
            kind = {'frontier': '前沿目标', 'random': '随机回退（继续探索）', 'none': '无目标'}.get(s['goal_kind'], '--')
            lines = [
                f"{state} | 模式 {s['mode'] or '--'} | 运行 {s['run_id'][:8] if s['run_id'] else '--'}",
                f"原因 {s['reason'] or '--'}",
                f"已用 {number(s['elapsed_sec'], ' s')} | 已处理 {s['processed'] if s['processed'] is not None else '--'} | 前沿 {s['frontier_count'] if s['frontier_count'] is not None else '--'} | {kind}",
                f"当前目标 {goal_text(s['goal_world'])} | 距离 {number(s['goal_distance_m'], ' m')}",
                f"输入年龄 {number(s['input_age_sec'], ' ms', 1000)} | Agent {number(s['agent_sec'], ' ms', 1000)} | 源年龄 {number(s['source_age_sec'], ' ms', 1000)}",
                f"等待观测 {number(s['observation_wait_sec'], ' ms', 1000)} | 扫描 {number(s['scan_progress_rad'], '°', 180 / math.pi)} / {number(s['scan_target_rad'], '°', 180 / math.pi)}",
                f"运动 {motion['phase'] or '--'} | 原因 {motion['reason'] or '--'} | 输出指令 " + ('--（状态过期）' if velocity_stale else f"{number(motion['output_linear_x'], ' m/s', precision=3)} / {number(motion['output_angular_z'], ' rad/s', precision=3)}"),
            ]
        if self.tty:
            width = max(1, shutil.get_terminal_size().columns - 1)
            print('\n'.join(fit_terminal(line, width) for line in lines), flush=True)
            self.view_lines = len(lines)
        else:
            print(' | '.join(lines), flush=True)


def main():
    rclpy.init()
    node = StatusWatch()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
