"""Four GNOME Terminal tabs, with the existing dependency-aware recovery policy."""
import argparse
import fcntl
import json
import logging
import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import threading
import time

import yaml
from supervisor import ORDER, Processes, Supervisor, existing_nodes, load_config

TITLES = {'base': '底盘通信', 'livox': 'Livox 雷达', 'fastlio': 'FAST-LIO 定位', 'motion': '自动控制接口'}
SCRIPT = str(Path(__file__).resolve())


def write_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name+f'.tmp-{os.getpid()}-{threading.get_ident()}')
    tmp.write_text(json.dumps(value, ensure_ascii=False))
    tmp.replace(path)


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return {}


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except (OSError, TypeError):
        return False


def process_start(pid):
    try:
        return Path('/proc', str(pid), 'stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


def stop_orphan(state, config):
    pid, started = state.get('launch_pid'), state.get('launch_start')
    if not pid or not started or process_start(pid) != started:
        return
    # Only the recorded, still-identical launch group created by this worker.
    if os.getpgid(pid) != pid:
        raise RuntimeError('recorded launch is no longer its own process group')
    for sig, grace in ((signal.SIGINT, config['shutdown_grace_sec']),
                       (signal.SIGTERM, config['shutdown_term_sec']), (signal.SIGKILL, 1)):
        try:
            os.killpg(pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic()+grace
        while time.monotonic() < deadline:
            if process_start(pid) != started:
                return
            time.sleep(0.1)


def terminal_command(session, config_path):
    argv = ['gnome-terminal', '--window']
    for index, name in enumerate(ORDER):
        if index:
            argv.append('--tab')
        command = [sys.executable, SCRIPT, '--worker', name, '--session', str(session), '--config', str(config_path)]
        # GNOME's per-tab command option; '--' only applies to the final tab.
        argv += ['--title='+TITLES[name], '--command='+shlex.join(command)]
    return argv


class VisibleProcesses(Processes):
    """Each ROS launch lives under its terminal worker; tee output to its log."""
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


def worker(name, session, config):
    state_path, command_path = session/(name+'.state.json'), session/(name+'.command.json')
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s',
                        handlers=[logging.StreamHandler(), logging.FileHandler(session/(name+'-tab.log'))])
    os.environ['ROS_LOG_DIR'] = str(session/'ros'/name)
    processes = VisibleProcesses(config, session)
    stopping = False
    generation = 0
    phase = 'idle'
    status_message = None

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, request_stop)
    logging.info('%s：等待启动条件。Ctrl+C / 关闭任一标签页将停止整套服务。', TITLES[name])
    try:
        while not stopping:
            manager = read_json(session/'manager.json')
            if not alive(manager.get('pid')) or time.time()-manager.get('heartbeat', 0) > 15:
                logging.error('总控已退出或失联；退出本标签页的 ROS 进程。')
                break
            command = read_json(command_path)
            requested = command.get('generation', 0)
            if command.get('op') == 'quit':
                break
            if command.get('op') == 'stop' and (phase != 'idle' or generation != requested):
                processes.stop(name)
                generation, phase = requested, 'idle'
            elif command.get('op') == 'run' and requested != generation:
                processes.stop(name)
                generation = requested
                logging.info('启动第 %s 次；指令：%s', generation, config['commands'][name])
                processes.start(name)
                phase = 'running'
            if phase == 'running' and processes.exited(name):
                phase = 'exited'
                logging.warning('启动进程已退出，等待总控安排本标签页重启。')
            notice = read_json(session/(name+'.notice.json')).get('message')
            if notice and notice != status_message:
                logging.info('%s', notice)
                status_message = notice
            write_json(state_path, {'pid': os.getpid(), 'generation': generation,
                                    'phase': phase, 'heartbeat': time.time(),
                                    'launch_pid': processes.children[name].pid if name in processes.children else None,
                                    'launch_start': process_start(processes.children[name].pid) if name in processes.children else None})
            time.sleep(0.1)
    finally:
        # Publish intentional stop before waiting for this launch's descendants.
        write_json(session/'stop-request.json', {'component': name, 'pid': os.getpid()})
        processes.stop(name)
        write_json(state_path, {'pid': os.getpid(), 'generation': generation, 'phase': 'closed'})
        logging.info('本标签页已停止，不再自动重启。')


class TerminalProcesses:
    """Supervisor process adapter: commands are executed in persistent tabs."""
    def __init__(self, config, session):
        self.config, self.session = config, Path(session)
        self.children, self.generations = {}, {name: 0 for name in ORDER}

    def command(self, name, op):
        write_json(self.session/(name+'.command.json'),
                   {'op': op, 'generation': self.generations[name]})

    def start(self, name):
        self.generations[name] += 1
        self.children[name] = True
        self.command(name, 'run')
        logging.info('START %s in terminal tab; generation=%s', name, self.generations[name])

    def exited(self, name):
        state = read_json(self.session/(name+'.state.json'))
        return (state.get('phase') == 'closed' or not alive(state.get('pid')) or
                (state.get('generation') == self.generations[name] and state.get('phase') == 'exited'))

    def stop(self, name):
        if name not in self.children:
            return
        self.command(name, 'stop')
        deadline = time.monotonic()+self.config['shutdown_grace_sec']+self.config['shutdown_term_sec']+4
        while time.monotonic() < deadline:
            state = read_json(self.session/(name+'.state.json'))
            if (state.get('generation') == self.generations[name] and state.get('phase') in ('idle', 'closed')):
                self.children.pop(name, None)
                logging.info('STOP %s in terminal tab', name)
                return
            if not alive(state.get('pid')):
                stop_orphan(state, self.config)
                self.children.pop(name, None)
                logging.warning('STOP orphaned %s launch after terminal worker disappeared', name)
                return
            time.sleep(0.1)
        raise RuntimeError(name+' terminal worker did not acknowledge stop')


def validate(config):
    from ament_index_python.packages import get_package_prefix
    for name, package in zip(ORDER, ('turn_on_wheeltec_robot', 'livox_ros_driver2', 'fast_lio', 'wheeltec_motion_control')):
        expected = (Path(config['workspaces'][name])/'install').resolve()
        if not Path(get_package_prefix(package)).resolve().is_relative_to(expected):
            raise ValueError(package+' resolved outside expected workspace')
    params = yaml.safe_load(Path(config['motion_params']).read_text())['wheeltec_motion_controller']['ros__parameters']
    if params.get('control_enabled') is not True or params.get('extrinsics_calibrated') is not True:
        raise ValueError('motion config must enable control and confirmed extrinsics')
    return params


def manager(session, config, config_path):
    root = Path(config['log_root'])
    lock_path = root/'supervisor.lock'  # Same lock as original script: never run both.
    with lock_path.open('a+') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError('已有总控在运行，请先停止旧版脚本，再试标签页版本。') from exc
        if existing_nodes():
            raise ValueError('已有手动 ROS 节点，请先退出它们再启动标签页版本。')
        params = validate(config)
        lock.seek(0)
        lock.truncate()
        lock.write(str(os.getpid()))
        lock.flush()
        logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(message)s',
                            handlers=[logging.FileHandler(session/'supervisor.log'), logging.StreamHandler()])
        (session/'stack-config.yaml').write_text(yaml.safe_dump(config))
        (session/'motion-config.yaml').write_text(Path(config['motion_params']).read_text())
        write_json(root/'tabs-active.json', {'session': str(session), 'pid': os.getpid()})
        stopping = False
        heartbeat_done = threading.Event()
        def heartbeat():
            while not heartbeat_done.is_set():
                write_json(session/'manager.json', {'pid': os.getpid(), 'heartbeat': time.time()})
                heartbeat_done.wait(0.5)
        thread = threading.Thread(target=heartbeat, daemon=True)
        thread.start()
        def request_stop(signum, frame):
            nonlocal stopping
            stopping = True
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        processes = TerminalProcesses(config, session)
        monitor = None
        runner = None
        try:
            result = subprocess.run(terminal_command(session, config_path), capture_output=True, text=True, timeout=20)
            if result.returncode:
                raise RuntimeError('无法打开 GNOME Terminal：'+result.stderr.strip())
            deadline = time.monotonic()+30
            while not stopping and time.monotonic() < deadline:
                if (session/'stop-request.json').exists():
                    stopping = True
                    break
                if all(alive(read_json(session/(name+'.state.json')).get('pid')) for name in ORDER):
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError('标签页启动超时；检查图形会话与 bootstrap.log')
            if stopping:
                return
            from health import Health
            monitor = Health(config, params)
            runner = Supervisor(config, processes, monitor)
            write_json(session/'launch.json', {'ok': True, 'pid': os.getpid()})
            logging.info('Four tabs opened. Ctrl+C in any tab stops the whole stack. Logs: %s', session)
            next_tick = time.monotonic()
            while not stopping:
                if (session/'stop-request.json').exists() or any(
                        not alive(read_json(session/(name+'.state.json')).get('pid')) for name in ORDER):
                    break
                monitor.spin(max(0, next_tick-time.monotonic()))
                if time.monotonic() >= next_tick:
                    runner.tick()
                    for name in ORDER:
                        message = ('就绪，可以接收新的控制指令。' if name == 'motion' else '就绪。') if name in runner.ready else '等待启动条件或自动恢复；查看本标签页输出。'
                        write_json(session/(name+'.notice.json'), {'message': message})
                    next_tick = time.monotonic()+config['poll_sec']
        finally:
            logging.info('Stopping all tabs; old goals are never resumed.')
            try:
                if runner:
                    runner.shutdown()
            finally:
                for name in reversed(ORDER):
                    processes.command(name, 'quit')
                if monitor:
                    monitor.close()
                heartbeat_done.set()
                thread.join(timeout=2)
                write_json(session/'stopped.json', {'pid': os.getpid()})


def active_manager(config):
    entry = read_json(Path(config['log_root'])/'tabs-active.json')
    pid = entry.get('pid')
    try:
        argv = Path('/proc', str(pid), 'cmdline').read_bytes().split(b'\0')
        expected = SCRIPT.encode()
        if expected not in argv or b'--manager' not in argv or entry.get('session', '').encode() not in argv:
            return {}
    except OSError:
        return {}
    return entry


def main():
    parser = argparse.ArgumentParser(description='新增的四标签页版本；保留原 start_robot.sh。')
    parser.add_argument('--config', default=str(Path(__file__).with_name('config.yaml')))
    parser.add_argument('--check', action='store_true', help='只检查环境，不启动')
    parser.add_argument('--status', action='store_true', help='查看标签页版本是否运行与日志位置')
    parser.add_argument('--stop', action='store_true', help='停止标签页版本及四个 ROS 组件')
    parser.add_argument('--print-setups', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--manager', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--worker', choices=ORDER, help=argparse.SUPPRESS)
    parser.add_argument('--session', help=argparse.SUPPRESS)
    args = parser.parse_args()
    config_path = str(Path(args.config).resolve())
    config = load_config(config_path)
    if args.stop or args.status:
        entry = active_manager(config)
        if not entry:
            print('标签页版本未运行。')
            return
        if args.stop:
            os.kill(entry['pid'], signal.SIGINT)
            deadline = time.monotonic()+60
            while alive(entry['pid']) and not Path(entry['session'], 'stopped.json').exists():
                if time.monotonic() > deadline:
                    raise RuntimeError('停止仍在进行，请查看日志：'+entry['session'])
                time.sleep(0.1)
            print('已停止标签页版本。')
        else:
            print('总控 PID:', entry['pid'], '\n日志:', entry['session'])
            for name in ORDER:
                print(TITLES[name]+':', read_json(Path(entry['session'])/(name+'.state.json')).get('phase', 'waiting'),
                      read_json(Path(entry['session'])/(name+'.notice.json')).get('message', ''))
        return
    setups = [config['ros_setup']]+[str(Path(config['workspaces'][name])/'install/local_setup.bash') for name in ORDER]
    for path in setups+[config['motion_params']]:
        if not Path(path).is_file():
            raise ValueError('missing file: '+path)
    if args.print_setups:
        sys.stdout.buffer.write(('\0'.join(setups)+'\0').encode())
        return
    if args.worker:
        worker(args.worker, Path(args.session), config)
        return
    if args.manager:
        manager(Path(args.session), config, config_path)
        return
    validate(config)
    if args.check:
        print('CHECK PASSED. 当前已有节点：', existing_nodes())
        return
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        raise ValueError('请在小车图形桌面的终端运行；普通 SSH 会话不能直接打开标签页。')
    if active_manager(config):
        print('标签页版本已运行；用 --status 查看，用 --stop 停止。')
        return
    root = Path(config['log_root'])
    root.mkdir(parents=True, exist_ok=True)
    session = root/(time.strftime('%Y%m%d-%H%M%S')+'-tabs-'+str(os.getpid()))
    session.mkdir()
    with (session/'bootstrap.log').open('ab', buffering=0) as output:
        proc = subprocess.Popen([sys.executable, SCRIPT, '--manager', '--session', str(session), '--config', config_path],
                                start_new_session=True, stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+55
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError((session/'bootstrap.log').read_text().strip())
            if read_json(session/'launch.json').get('ok'):
                print('已打开四个标签页；等待“自动控制接口”标签显示就绪。\n日志：'+str(session))
                return
            time.sleep(0.1)
        raise RuntimeError('打开标签页超时；日志：'+str(session))
    except BaseException:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
        raise


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('robot tabs failed: '+str(exc), file=sys.stderr)
        raise SystemExit(1)
