"""Component processes, startup dependencies and recovery for the tabs manager."""
import logging
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading

import yaml

ORDER = ('base', 'livox', 'fastlio', 'motion')
DEPENDENCIES = {'base': (), 'livox': ('base',),
                'fastlio': ('base', 'livox'), 'motion': ('base', 'livox', 'fastlio'), 'camera': (), 'rviz': ('fastlio',)}
RESTART_SETS = {'base': ('motion', 'base'),
                'livox': ('motion', 'fastlio', 'livox'),
                'fastlio': ('motion', 'fastlio'), 'motion': ('motion',), 'camera': ('camera',), 'rviz': ('rviz',)}
MARKERS = ('/turn_on_wheeltec_robot/wheeltec_robot_node',
           '/livox_ros_driver2/livox_ros_driver2_node',
           '/fast_lio/fastlio_mapping', '/wheeltec_motion_control/motion_controller',
           '/realsense2_camera/realsense2_camera_node')


def load_config(path):
    data = yaml.safe_load(Path(path).read_text())
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
        except (FileNotFoundError, ProcessLookupError):
            pass
    return matches


class Processes:
    def __init__(self, config, log_dir):
        self.config, self.log_dir = config, Path(log_dir)
        self.children = {}

    def start(self, name):
        proc = subprocess.Popen(self.config['commands'][name], stdin=subprocess.DEVNULL,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                start_new_session=True, bufsize=0)
        self.children[name] = proc
        output = (self.log_dir/(name+'.log')).open('ab', buffering=0)

        def copy_output():
            try:
                while True:
                    chunk = os.read(proc.stdout.fileno(), 4096)
                    if not chunk:
                        break
                    output.write(chunk)
                    try:
                        sys.stdout.buffer.write(chunk)
                        sys.stdout.buffer.flush()
                    except (BrokenPipeError, OSError):
                        # Closing the tab still runs process-group cleanup.
                        pass
            finally:
                output.close()
                proc.stdout.close()
        threading.Thread(target=copy_output, daemon=True).start()
        logging.info('START %s pid=%s argv=%s', name, proc.pid, self.config['commands'][name])

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
    def __init__(self, config, processes, monitor):
        self.config, self.processes, self.monitor = config, processes, monitor
        self.order = config['order']
        self.started, self.ready, self.bad_since = {}, set(), {}
        self.retry_at = {name: 0.0 for name in self.order}
        self.failures = {name: 0 for name in self.order}
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
            self.retry_at[affected] = time.monotonic()+delay

    def tick(self):
        now = time.monotonic()
        for name in self.order:
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
                if name in ('base', 'livox', 'fastlio'):
                    self.stop('motion')
            elif name in self.bad_since:
                if now-self.bad_since[name] >= self.config['unhealthy_grace_sec']:
                    self.recover(name, reason)
                    return
            elif now-self.started[name] >= self.config['startup_timeout_sec']:
                self.recover(name, 'startup readiness timeout: '+reason)
                return
        for name in self.order:
            if name not in self.started and now >= self.retry_at[name] and all(
                    dependency in self.ready for dependency in DEPENDENCIES[name]):
                self.monitor.reset(name)
                self.processes.start(name)
                self.started[name] = time.monotonic()
                # Let ROS discover and receive data before starting next stage.
                break
        if all(name in self.ready for name in self.order) and not self.was_ready:
            logging.info('STACK READY: accepting NEW motion goals; no goal was sent')
            self.was_ready = True

    def shutdown(self):
        self.stop('motion')
        for name in reversed(self.order):
            if name != 'motion':
                self.stop(name)
