"""One owner for the GUI stack and mutually exclusive goal / Agent processes."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

ROOT = Path('/sim/runtime')
SOCKET = ROOT / 'session.sock'
GENERATED = ROOT / 'generated'


class Session(Node):
    def __init__(self):
        super().__init__('conavbot_sim_session')
        self.stop_client = self.create_client(Trigger, '/wheeltec_motion/stop')
        self.export_client = self.create_client(Trigger, '/conavbot/export_stage')
        self.estop_pub = self.create_publisher(Bool, '/sim/emergency_stop', 1)
        self.processes = {}
        self.logs = []
        self.navigation = yaml.safe_load((GENERATED / 'navigation.yaml').read_text())
        self.goal_args = ['python3', '-m', 'conavbot_sim.goal_navigation', '--ros-args',
                          '-p', 'map_file:=' + str(GENERATED / 'map.yaml'),
                          '-p', 'forward_speed:=' + str(self.navigation['forward_speed']),
                          '-p', 'turn_speed:=' + str(self.navigation['turn_speed'])]
        self.mode = 'idle'
        self.emergency = False
        self.running = True
        self.lifecycle = dict(run_id='', mode='idle', state='STOPPED',
                              started_wall_ns=0, reason='WAITING_FOR_TASK',
                              exit_code=None, report=None)
        self.status_pub = self.create_publisher(String, '/sim/session_status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.5, self.publish_status)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.lifecycle)))

    def start_task(self, mode, command):
        self.lifecycle = dict(run_id=uuid.uuid4().hex, mode=mode, state='STARTING',
                              started_wall_ns=time.time_ns(), reason='TASK_STARTED',
                              exit_code=None, report=None)
        self.mode = mode
        self.publish_status()
        self.spawn('task', command)
        self.lifecycle['state'] = 'RUNNING'
        self.publish_status()

    def finish_task(self, state, reason, exit_code=None):
        report = None
        path = ROOT / 'agent/runs' / self.lifecycle['run_id'] / 'report.json'
        if self.lifecycle['mode'] in ('explore', 'preview') and path.exists():
            # Only the final report from this process belongs to this task.
            candidate = json.loads(path.read_text())
            if candidate.get('run_id') == self.lifecycle['run_id'] and candidate.get('reason') == 'EXITED':
                report = candidate
        self.lifecycle.update(state=state, reason=reason, exit_code=exit_code, report=report)
        self.publish_status()

    def spawn(self, name, command):
        log = (ROOT / (name + '.log')).open('a', buffering=1)
        self.logs.append(log)
        env = dict(os.environ)
        if name == 'task':
            env['CONAVBOT_RUN_ID'] = self.lifecycle['run_id']
        self.processes[name] = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                                start_new_session=True, env=env)

    def terminate(self, name):
        process = self.processes.pop(name, None)
        if process is None or process.poll() is not None:
            return
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=20 if name == 'task' else 8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def stop_motion(self):
        if not self.stop_client.wait_for_service(timeout_sec=2):
            return False
        future = self.stop_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        return future.done() and future.result() is not None and future.result().success

    def latch(self, enabled):
        self.emergency = enabled
        # Bridge discovery may take a moment at startup; explicit reset occurs
        # only on user commands after the stack has started.
        self.estop_pub.publish(Bool(data=enabled))
        rclpy.spin_once(self, timeout_sec=.05)

    def stop_task(self, reason='USER_STOP'):
        # Stop publisher first; no producer may recreate navigation after stop.
        self.terminate('task')
        stopped = self.stop_motion()
        if not stopped:
            self.latch(True)
        self.mode = 'idle'
        self.finish_task('STOPPED' if stopped else 'ERROR',
                         reason if stopped else 'MOTION_STOP_UNAVAILABLE')
        return stopped

    def dispatch(self, command):
        if command == 'status':
            export_file = ROOT / 'agent/runs' / self.lifecycle['run_id'] / 'stages/export_status.json'
            export_status = json.loads(export_file.read_text()) if export_file.is_file() else None
            return {'mode': self.mode, 'emergency': self.emergency,
                    'processes': {k: 'running' if p.poll() is None else f'exited {p.returncode}'
                                  for k, p in self.processes.items()},
                    'logs': str(ROOT), 'export': export_status, 'run_id': self.lifecycle['run_id'],
                    'results': str(ROOT / 'agent/runs' / self.lifecycle['run_id'])}
        if command == 'export':
            task = self.processes.get('task')
            if self.mode not in ('explore', 'preview') or task is None or task.poll() is not None:
                return {'error': 'No active Agent; start explore or readonly preview first.'}
            if not self.export_client.wait_for_service(timeout_sec=2):
                return {'error': 'Agent export service not ready.'}
            future = self.export_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=3)
            if not future.done() or future.result() is None:
                return {'error': 'Export request timed out; inspect export_status.json.'}
            response = future.result()
            return {'queued' if response.success else 'error': response.message}
        if command == 'stop':
            return {'stopped': self.stop_task(), 'mode': self.mode}
        if command == 'estop':
            self.latch(True)
            self.stop_task('EMERGENCY_STOP')
            return {'emergency': True, 'message': 'Queue cleared; physical braking remains active.'}
        if command == 'shutdown':
            self.latch(True)
            self.stop_task('SESSION_SHUTDOWN')
            self.running = False
            return {'shutdown': True}
        if command not in ('goal', 'explore', 'preview'):
            return {'error': 'Unknown session command: ' + command}
        if not self.stop_task('MODE_SWITCH'):
            return {'error': 'Motion stop service unavailable; inspect motion/stack logs before starting.'}
        self.latch(False)
        if command == 'goal':
            args = self.goal_args
        else:
            args = ['python3', '-m', 'conavbot.runtime', '--config', str(GENERATED / 'agent.yaml'),
                    '--output', str(ROOT / 'agent'), '--duration', '0' if command == 'explore' else '20',
                    '--export-interval', str(self.navigation['export_interval']),
                    '--forward-speed', str(self.navigation['forward_speed']),
                    '--turn-speed', str(self.navigation['turn_speed']), '--map-size-cm', '2400', '--input-stride', '4']
        if command == 'explore':args.append('--motion')
        self.start_task(command, args)
        return {'mode': command, 'message': 'Waiting for RViz goal' if command == 'goal' else 'Agent started (motion)' if command == 'explore' else 'Readonly observation capture (20 s)'}


def serve():
    ROOT.mkdir(parents=True, exist_ok=True)
    if SOCKET.exists():
        raise RuntimeError('Session socket already exists; use reset or down before starting again')
    rclpy.init()
    node = Session()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(SOCKET))
    server.listen(2)
    server.settimeout(.2)
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'running', False))
    signal.signal(signal.SIGINT, lambda *_: setattr(node, 'running', False))
    node.spawn('stack', ['ros2', 'launch', 'conavbot_sim', 'simulation.launch.py'])
    # Goal node remains passive until the user clicks; no service is enabled here.
    node.start_task('goal', node.goal_args)
    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0)
            if node.processes['stack'].poll() is not None:
                node.get_logger().error('Simulation stack exited; stopping session')
                break
            task = node.processes.get('task')
            if task is not None and task.poll() is not None:
                node.get_logger().info(f'{node.mode} process exited with code {task.returncode}; see task.log')
                node.processes.pop('task')
                node.stop_motion()
                node.finish_task('EXITED' if task.returncode == 0 else 'ERROR',
                                 'PROCESS_EXITED', task.returncode)
                node.mode = 'idle'
            try:
                connection, _ = server.accept()
            except socket.timeout:
                continue
            with connection:
                connection.settimeout(5)
                command = connection.recv(256).decode().strip()
                response = node.dispatch(command)
                connection.sendall(json.dumps(response).encode())
    finally:
        node.latch(True)
        node.stop_task('SESSION_SHUTDOWN')
        node.terminate('stack')
        server.close()
        SOCKET.unlink(missing_ok=True)
        for log in node.logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['serve', 'status', 'goal', 'explore', 'preview', 'export', 'stop', 'estop', 'shutdown'])
    args = parser.parse_args()
    if args.command == 'serve':
        serve()
        return
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(35)
        client.connect(str(SOCKET))
        client.sendall(args.command.encode())
        result = json.loads(client.recv(65536))
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if 'error' in result:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
