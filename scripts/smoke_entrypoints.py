"""Exercise installed launch and CLI entrypoints with motion output disabled."""
import os
from pathlib import Path
import signal
import subprocess
import time


logs = Path('logs')
logs.mkdir(exist_ok=True)
launch_log = logs/'launch-smoke.log'
env = dict(os.environ, ROS_DOMAIN_ID='174', ROS_LOCALHOST_ONLY='1')
with launch_log.open('w') as output:
    process = subprocess.Popen(
        ['ros2', 'launch', 'wheeltec_motion_control', 'motion_control.launch.py'],
        stdout=output, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        deadline = time.monotonic()+10
        while 'Motion server ready' not in launch_log.read_text():
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('installed launch failed to become ready')
            time.sleep(0.1)
        for arguments, expected in [(['move', '0.1'], 1), (['stop'], 0)]:
            command = ['ros2', 'run', 'wheeltec_motion_control', 'motion_client', *arguments]
            result = subprocess.run(command, env=env, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
            with (logs/'client-smoke.log').open('a') as stream:
                stream.write(f'COMMAND: {command}\nEXIT: {result.returncode}\n{result.stdout}\n')
            if result.returncode != expected:
                raise RuntimeError(f'CLI exit {result.returncode}, expected {expected}')
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            raise RuntimeError('launch did not shut down cleanly')
if process.returncode != 0:
    raise RuntimeError(f'launch exit: {process.returncode}')
if 'Traceback' in launch_log.read_text():
    raise RuntimeError('launch logged a traceback')
print('Installed launch, disabled-goal rejection, stop CLI and Ctrl-C shutdown passed.')
