"""Foreground ROS stack supervisor. Never sends a motion goal or resumes one."""
import argparse
import fcntl
import json
import logging
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

ORDER = ('base', 'livox', 'fastlio', 'motion')
DEPENDENCIES = {'base': (), 'livox': ('base',),
                'fastlio': ('base', 'livox'), 'motion': ('base', 'livox', 'fastlio')}
RESTART_SETS = {'base': ('motion', 'base'),
                'livox': ('motion', 'fastlio', 'livox'),
                'fastlio': ('motion', 'fastlio'), 'motion': ('motion',)}
MARKERS = ('/turn_on_wheeltec_robot/wheeltec_robot_node',
           '/livox_ros_driver2/livox_ros_driver2_node',
           '/fast_lio/fastlio_mapping', '/wheeltec_motion_control/motion_controller')


def load_config(path):
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict):
        raise ValueError('stack config must be a mapping')
    ws = data['workspaces']
    for key in ORDER:
        ws[key] = str(Path(os.path.expanduser(ws[key])).resolve())
    variables = {key+'_workspace': value for key, value in ws.items()}
    data['motion_params'] = str(Path(os.path.expanduser(data['motion_params'].format(**variables))).resolve())
    variables['motion_params'] = data['motion_params']
    data['log_root'] = str(Path(os.path.expanduser(data['log_root'].format(**variables))).resolve())
    data['ros_setup'] = str(Path(os.path.expanduser(data['ros_setup'])).resolve())
    for key in ('poll_sec', 'startup_timeout_sec', 'health_timeout_sec',
                'unhealthy_grace_sec', 'restart_delay_sec', 'restart_max_delay_sec',
                'restart_reset_sec', 'shutdown_grace_sec', 'shutdown_term_sec'):
        value = data[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(key+' must be finite and positive')
    if data['restart_max_delay_sec'] < data['restart_delay_sec']:
        raise ValueError('restart_max_delay_sec must be >= restart_delay_sec')
    if type(data['odom_ready_samples']) is not int or data['odom_ready_samples'] < 2:
        raise ValueError('odom_ready_samples must be an integer >= 2')
    for key in ORDER:
        cmd = data['commands'][key]
        if not isinstance(cmd, list) or not cmd or not all(isinstance(v, str) and v for v in cmd):
            raise ValueError(key+' command must be a nonempty argv list')
        data['commands'][key] = [v.format(**variables) for v in cmd]
    return data


def environment_setups(config):
    """Source each workspace once, even when multiple components share it."""
    paths = [config['ros_setup']]+[
        str(Path(config['workspaces'][name])/'install/local_setup.bash') for name in ORDER]
    return list(dict.fromkeys(paths))


def existing_nodes():
    matches = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = path.read_bytes().split(b'\0')
            if any(any(marker in arg.decode(errors='replace') for marker in MARKERS) for arg in args):
                matches.append((int(path.parent.name), b' '.join(args).decode(errors='replace').strip()))
        except (OSError, ValueError):
            pass
    return matches


class Processes:
    def __init__(self, config, log_dir):
        self.config, self.log_dir = config, Path(log_dir)
        self.children = {}

    def start(self, name):
        output = (self.log_dir/(name+'.log')).open('ab', buffering=0)
        try:
            self.children[name] = subprocess.Popen(
                self.config['commands'][name], stdin=subprocess.DEVNULL,
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        finally:
            output.close()
        logging.info('START %s pid=%s argv=%s', name, self.children[name].pid,
                     self.config['commands'][name])

    def exited(self, name):
        return self.children[name].poll() is not None

    def stop(self, name):
        proc = self.children.pop(name, None)
        if proc is None:
            return
        # Signal our own launch process group, including nodes if launch exited.
        for sig, grace in ((signal.SIGINT, self.config['shutdown_grace_sec']),
                           (signal.SIGTERM, self.config['shutdown_term_sec']),
                           (signal.SIGKILL, 1.0)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic()+grace
            while time.monotonic() < deadline:
                proc.poll()
                try:
                    os.killpg(proc.pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                continue
            break
        proc.wait(timeout=2)
        logging.info('STOP %s exit=%s', name, proc.returncode)


class Supervisor:
    def __init__(self, config, processes, monitor, clock=time.monotonic):
        self.config, self.processes, self.monitor, self.clock = config, processes, monitor, clock
        self.started, self.ready, self.bad_since = {}, set(), {}
        self.retry_at = {name: 0.0 for name in ORDER}
        self.failures = {name: 0 for name in ORDER}
        self.healthy_since = {}
        self.was_ready = False

    def stop(self, name):
        self.processes.stop(name)
        self.started.pop(name, None)
        self.ready.discard(name)
        self.bad_since.pop(name, None)
        self.healthy_since.pop(name, None)
        self.monitor.reset(name)
        self.was_ready = False

    def recover(self, name, reason):
        logging.warning('RECOVER %s: %s; old actions will not resume', name, reason)
        self.failures[name] += 1
        delay = min(self.config['restart_max_delay_sec'],
                    self.config['restart_delay_sec']*2**min(self.failures[name]-1, 10))
        for affected in RESTART_SETS[name]:
            self.stop(affected)
        for affected in RESTART_SETS[name]:
            self.retry_at[affected] = self.clock()+delay

    def tick(self):
        now = self.clock()
        for name in ORDER:
            if name not in self.started:
                continue
            if self.processes.exited(name):
                self.recover(name, 'process exited')
                return
            healthy, reason = self.monitor.healthy(name)
            if healthy:
                self.bad_since.pop(name, None)
                if name not in self.ready:
                    self.ready.add(name)
                    self.healthy_since[name] = now
                    logging.info('READY %s', name)
                if now-self.healthy_since[name] >= self.config['restart_reset_sec']:
                    self.failures[name] = 0
            elif name in self.ready:
                self.ready.discard(name)
                self.healthy_since.pop(name, None)
                self.bad_since[name] = now
                logging.warning('UNHEALTHY %s: %s', name, reason)
                # Withdraw motion interface immediately; underlying streams get
                # a grace period to avoid restarting on one transient bad sample.
                if name != 'motion':
                    self.stop('motion')
            elif name in self.bad_since:
                if now-self.bad_since[name] >= self.config['unhealthy_grace_sec']:
                    self.recover(name, reason)
                    return
            elif now-self.started[name] >= self.config['startup_timeout_sec']:
                self.recover(name, 'startup readiness timeout: '+reason)
                return
        for name in ORDER:
            if name not in self.started and now >= self.retry_at[name] and all(
                    dependency in self.ready for dependency in DEPENDENCIES[name]):
                self.monitor.reset(name)
                self.processes.start(name)
                self.started[name] = self.clock()
                # Let ROS discover and receive data before starting next stage.
                break
        if all(name in self.ready for name in ORDER) and not self.was_ready:
            logging.info('STACK READY: accepting NEW motion goals; no goal was sent')
            self.was_ready = True

    def shutdown(self):
        for name in reversed(ORDER):
            self.stop(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(Path(__file__).with_name('config.yaml')))
    parser.add_argument('--check', action='store_true', help='validate environment/config only; start no processes')
    parser.add_argument('--print-setups', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    config = load_config(args.config)
    setups = environment_setups(config)
    for path in setups+[config['motion_params']]:
        if not Path(path).is_file():
            raise ValueError('missing file: '+path)
    if args.print_setups:
        sys.stdout.buffer.write(('\0'.join(setups)+'\0').encode())
        return
    from ament_index_python.packages import get_package_prefix
    packages = ('turn_on_wheeltec_robot', 'livox_ros_driver2', 'fast_lio', 'wheeltec_motion_control')
    for name, package in zip(ORDER, packages):
        prefix = Path(get_package_prefix(package)).resolve()
        expected = Path(config['workspaces'][name])/'install'
        if not prefix.is_relative_to(expected.resolve()):
            raise ValueError(f'{package} resolved to wrong workspace: {prefix}')
    params = yaml.safe_load(Path(config['motion_params']).read_text())['wheeltec_motion_controller']['ros__parameters']
    if params.get('control_enabled') is not True or params.get('extrinsics_calibrated') is not True:
        raise ValueError('motion config must explicitly enable control and confirmed extrinsics')
    existing = existing_nodes()
    if args.check:
        print('CHECK PASSED: paths, packages and enabled motion configuration are valid.')
        print('Existing managed nodes (must stop them before normal startup):', existing)
        print('Commands:', json.dumps(config['commands'], ensure_ascii=False))
        return
    if existing:
        raise ValueError('existing manual/managed nodes found; stop their launch terminals first: '+str(existing))
    lock_path = Path(config['log_root'])/'supervisor.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('another supervisor is already running') from exc
        lock.seek(0)
        lock.truncate()
        lock.write(str(os.getpid()))
        lock.flush()
        log_dir = lock_path.parent/(time.strftime('%Y%m%d-%H%M%S')+'-'+str(os.getpid()))
        log_dir.mkdir()
        os.environ['ROS_LOG_DIR'] = str(log_dir/'ros')
        logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s',
                            handlers=[logging.StreamHandler(), logging.FileHandler(log_dir/'supervisor.log')])
        (log_dir/'stack-config.yaml').write_text(yaml.safe_dump(config))
        (log_dir/'motion-config.yaml').write_text(Path(config['motion_params']).read_text())
        logging.info('Log directory: %s', log_dir)
        from health import Health
        monitor = Health(config, params)
        runner = Supervisor(config, Processes(config, log_dir), monitor)
        stopping = False

        def stop_requested(signum, frame):
            nonlocal stopping
            stopping = True

        signal.signal(signal.SIGINT, stop_requested)
        signal.signal(signal.SIGTERM, stop_requested)
        try:
            next_tick = time.monotonic()
            while not stopping:
                monitor.spin(max(0.0, next_tick-time.monotonic()))
                if not stopping and time.monotonic() >= next_tick:
                    runner.tick()
                    next_tick = time.monotonic()+config['poll_sec']
        finally:
            logging.info('Shutting down; stopping controller before hardware interfaces')
            runner.shutdown()
            monitor.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('robot stack failed: '+str(exc), file=sys.stderr)
        raise SystemExit(1)
