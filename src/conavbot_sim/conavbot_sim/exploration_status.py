"""Read-only session, Agent and motion status aggregation for simulation."""
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from wheeltec_motion_interfaces.msg import MotionStatus


TERMINAL = {'STOPPED', 'EXITED', 'ERROR'}
WAITING = {'WAITING', 'NO_NEW_OBSERVATION', 'NEW_OBSERVATION_REQUIRED',
           'NO_TIME_MATCHED_POSE', 'WAITING_FOR_POSE'}
ZH = {'WAITING': '等待观测', 'INITIAL_SCAN': '启动扫描', 'EXPLORING': '前沿探索',
      'RANDOM_FALLBACK': '随机目标回退', 'NO_PATH': '无可行路径', 'STOPPED': '已停止',
      'EXITED': '任务退出', 'ERROR': '任务错误', 'STALE': '状态失联', 'IDLE': '空闲',
      'GOAL_NAVIGATION': '目标导航', 'HALTED': '已暂停',
      'GOAL_REACHED': '已到达当前目标', 'NO_DESCENT': '距离场无下降方向',
      'REPEATED_POINT': '路径点重复', 'PATH_BLOCKED': '路径被障碍阻断',
      'GOAL_UNREACHABLE': '目标与当前区域不连通'}


class ExplorationStatus(Node):
    def __init__(self):
        super().__init__('sim_exploration_status')
        self.declare_parameter('status_directory', '/sim/runtime/status')
        self.directory = Path(self.get_parameter('status_directory').value)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        volatile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.output = self.create_publisher(String, '/sim/exploration_status', retained)
        self.markers = self.create_publisher(MarkerArray, '/sim/exploration_markers', retained)
        self.session = None
        self.session_at = self.decision_at = self.motion_at = self.pose_at = None
        self.decision = {}
        self.active = {}
        self.motion = self.pose = None
        self.event_seq = 0
        self.event = ''
        self.event_key = None
        self.terminal_elapsed = None
        self.waiting_since = None
        self.create_subscription(String, '/sim/session_status', self.on_session, retained)
        self.create_subscription(String, '/conavbot/decision', self.on_decision, volatile)
        self.create_subscription(MotionStatus, '/wheeltec_motion/status', self.on_motion, volatile)
        self.create_subscription(Odometry, '/sim/ground_truth', self.on_pose, volatile)
        self.create_timer(.5, self.publish)

    def decode(self, msg):
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Invalid status JSON: {exc}')
            return None
        if not isinstance(value, dict):
            self.get_logger().error('Status JSON must be an object')
            return None
        return value

    def on_session(self, msg):
        value = self.decode(msg)
        if value is None or not value.get('run_id'):
            return
        if self.session is None or value['run_id'] != self.session['run_id']:
            self.decision, self.active = {}, {}
            self.decision_at = None
            self.terminal_elapsed = None
            self.waiting_since = None
            self.event_seq, self.event, self.event_key = 0, '', None
        elif self.session['state'] in TERMINAL:
            return  # A late heartbeat cannot resurrect a finished run.
        self.session, self.session_at = value, time.monotonic()
        report = value.get('report')
        if value['state'] in TERMINAL and isinstance(report, dict):
            self.decision = report
        if value['state'] in TERMINAL:
            self.terminal_elapsed = self.decision.get('elapsed_sec')
            if self.terminal_elapsed is None and value.get('started_wall_ns'):
                self.terminal_elapsed = max(0., (time.time_ns()-value['started_wall_ns'])*1e-9)

    def on_decision(self, msg):
        value = self.decode(msg)
        if (value is None or self.session is None or
                value.get('run_id') != self.session['run_id'] or
                self.session['state'] in TERMINAL):
            return
        self.decision, self.decision_at = value, time.monotonic()
        if value.get('reason') not in WAITING and value.get('action') != 'waiting':
            self.active = value
            self.waiting_since = None
        elif self.waiting_since is None:
            self.waiting_since = self.decision_at

    def on_motion(self, msg):
        self.motion, self.motion_at = msg, time.monotonic()

    def on_pose(self, msg):
        self.pose, self.pose_at = msg.pose.pose, time.monotonic()

    @staticmethod
    def age(now, received):
        return None if received is None else max(0., now-received)

    def publish(self):
        now = time.monotonic()
        session = self.session or {}
        state = session.get('state', 'STARTING')
        decision = self.decision
        terminal = state in TERMINAL or decision.get('reason') == 'EXITED'
        waiting = decision.get('reason') in WAITING or decision.get('action') == 'waiting'
        data = self.active if waiting else decision
        ages = {name: self.age(now, received) for name, received in
                [('session', self.session_at), ('agent', self.decision_at), ('motion', self.motion_at)]}
        warmup = (state == 'STARTING' or
                  (session.get('mode') == 'explore' and self.decision_at is None)) and not terminal
        missing = []
        if session and not terminal:
            missing = [name for name, age in ages.items()
                       if (name == 'session' or not warmup)
                       and (name != 'agent' or session.get('mode') == 'explore')
                       and (age is None or age > 3.)]
        phase = data.get('phase', 'WAITING')
        reason = data.get('reason', session.get('reason', 'WAITING'))
        goal = data.get('goal_world')
        kind = data.get('goal_kind', 'none')
        if kind == 'random' and phase == 'EXPLORING':
            phase = 'RANDOM_FALLBACK'
        if (reason == 'NO_PATH' or (data.get('action') == 'path'
                and len(data.get('path', [])) < 2 and phase != 'INITIAL_SCAN')):
            reason = data.get('path_end_reason', 'NO_PATH')
            phase = reason if reason in {'GOAL_REACHED', 'NO_DESCENT', 'REPEATED_POINT', 'PATH_BLOCKED', 'GOAL_UNREACHABLE'} else 'NO_PATH'
        if session.get('mode') == 'goal' and not data and state == 'RUNNING':
            phase, reason = 'GOAL_NAVIGATION', session.get('reason', 'RUNNING')
        if terminal:
            phase = state if state in TERMINAL else 'EXITED'
            reason = session.get('reason') if state in TERMINAL else decision.get('exit_reason')
            if reason == 'PROCESS_EXITED':
                reason = decision.get('exit_reason') or reason
            reason = reason or phase
            goal, kind = None, 'none'
        elif missing:
            phase, reason = 'STALE', 'STALE_' + '_'.join(missing).upper()
        elif waiting and not data:
            phase, reason = 'WAITING', decision.get('reason', 'WAITING')
        motion_stale = ages['motion'] is None or ages['motion'] > 3.
        motion = self.motion
        elapsed = self.terminal_elapsed if terminal else decision.get('elapsed_sec')
        if terminal and elapsed is None:
            elapsed = decision.get('elapsed_sec')
        if elapsed is None and session.get('started_wall_ns'):
            elapsed = max(0., (time.time_ns()-session['started_wall_ns'])*1e-9)
        distance = None
        pose_age = self.age(now, self.pose_at)
        if goal is not None and self.pose is not None and pose_age <= 3. and not missing:
            distance = math.hypot(goal[0]-self.pose.position.x, goal[1]-self.pose.position.y)
        out = dict(run_id=session.get('run_id'), mode=session.get('mode', 'idle'),
                   phase=phase, phase_zh=ZH.get(phase, phase), reason=reason,
                   elapsed_sec=elapsed, processed=decision.get('processed'),
                   frontier_count=data.get('frontier_count'), goal_world=goal, goal_kind=kind,
                   goal_distance_m=distance, goal_change_reason=data.get('goal_change_reason'),
                   goal_selection_reason=data.get('goal_selection_reason'),
                   goal_attempts=data.get('goal_attempts', []), input_age_sec=data.get('input_age_sec'),
                   agent_sec=data.get('agent_sec'), source_age_sec=data.get('source_age_sec'),
                   observation_wait_sec=(now-self.waiting_since if waiting and self.waiting_since is not None
                                         else decision.get('observation_wait_sec')),
                   scan_progress_rad=data.get('scan_progress_rad'), scan_target_rad=data.get('scan_target_rad'),
                   waiting=waiting and not terminal, waiting_reason=decision.get('reason') if waiting else None,
                   session_age_sec=ages['session'], decision_age_sec=ages['agent'],
                   motion=dict(phase=motion.phase if motion else None, reason=motion.reason if motion else None,
                               output_linear_x=motion.output_linear_x if motion and not motion_stale else None,
                               output_angular_z=motion.output_angular_z if motion and not motion_stale else None,
                               stale=motion_stale), stale=bool(missing))
        # Fixed map targets can differ by floating-point transform roundoff.
        key = (phase, reason, tuple(round(v, 4) for v in goal) if goal is not None else None)
        if key != self.event_key:
            self.event_key = key
            self.event_seq += 1
            self.event = f"{out['phase_zh']}：{reason}"
            if goal is not None:
                self.event += f'；目标 ({goal[0]:.2f}, {goal[1]:.2f}) m'
            if session:
                target = self.directory / session['run_id']
                target.mkdir(parents=True, exist_ok=True)
                with (target/'events.jsonl').open('a') as stream:
                    stream.write(json.dumps(dict(event_seq=self.event_seq, event=self.event,
                                                 wall_ns=time.time_ns(), **out), ensure_ascii=False)+'\n')
        out.update(event_seq=self.event_seq, event=self.event)
        self.output.publish(String(data=json.dumps(out, ensure_ascii=False, allow_nan=False)))
        self.publish_markers(out, [] if terminal or missing else data.get('frontier_points_world', []))

    def publish_markers(self, status, frontiers):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        for index, kind in enumerate((Marker.POINTS, Marker.SPHERE, Marker.TEXT_VIEW_FACING)):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = 'camera_init', stamp
            marker.ns, marker.id, marker.type = 'exploration_status', index, kind
            marker.pose.orientation.w = 1.
            marker.color.a = 1.
            marker.action = Marker.ADD
            marker.lifetime.sec = 1
            marker.lifetime.nanosec = 500_000_000
            if index == 0:
                marker.scale.x = marker.scale.y = .12
                marker.color.g = marker.color.b = 1.
                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in frontiers]
                if not frontiers:
                    marker.action = Marker.DELETE
            elif index == 1:
                goal = status['goal_world']
                marker.scale.x = marker.scale.y = marker.scale.z = .24
                marker.color.r, marker.color.g = 1., .45
                if goal is None or status['stale']:
                    marker.action = Marker.DELETE
                else:
                    marker.pose.position = Point(x=float(goal[0]), y=float(goal[1]), z=float(goal[2]))
            else:
                marker.scale.z = .16
                marker.color.r = marker.color.g = marker.color.b = 1.
                if self.pose is not None:
                    marker.pose.position.x, marker.pose.position.y = self.pose.position.x, self.pose.position.y
                marker.pose.position.z = 1.1
                marker.text = f"{status['phase']} | {str(status['reason'])[:80]}"
                if status['frontier_count'] is not None:
                    marker.text += f" | frontiers {status['frontier_count']}"
                if status['phase'] == 'INITIAL_SCAN' and status['scan_progress_rad'] is not None:
                    progress = math.degrees(status['scan_progress_rad'])
                    target = status['scan_target_rad']
                    marker.text += f'\nScan {progress:.0f}'
                    if target is not None:
                        marker.text += f'/{math.degrees(target):.0f}'
                    marker.text += ' deg'
                if status['waiting']:
                    marker.text += '\nWaiting for observation'
            array.markers.append(marker)
        self.markers.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationStatus()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
