"""Exercise terminal IPC using real child processes, without ROS or hardware."""
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from tabs import ORDER, TerminalProcesses, terminal_command, write_json, read_json, process_start


class TabsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.config = {'commands': {name: [sys.executable, '-u', '-c', 'import time; print("test-output", flush=True); time.sleep(120)'] for name in ORDER},
                       'shutdown_grace_sec': 0.5, 'shutdown_term_sec': 0.5}
        self.workers = []
        write_json(self.root/'manager.json', {'pid': os.getpid(), 'heartbeat': time.time()})
        self.adapter = TerminalProcesses(self.config, self.root)

    def wait_for(self, test):
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            if test():
                return
            time.sleep(0.03)
        self.fail('condition did not become true')

    def spawn(self, name):
        code = 'from tabs import worker; import json,sys; from pathlib import Path; worker(sys.argv[1], Path(sys.argv[2]), json.loads(sys.argv[3]))'
        proc = subprocess.Popen([sys.executable, '-c', code, name, str(self.root), json.dumps(self.config)],
                                cwd=Path(__file__).parent, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.workers.append(proc)
        self.wait_for(lambda: read_json(self.root/(name+'.state.json')).get('phase') == 'idle')
        return proc

    def tearDown(self):
        for name in ORDER:
            self.adapter.command(name, 'quit')
        for proc in self.workers:
            try:
                proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        self.tmp.cleanup()

    def test_four_per_tab_commands_are_quoted(self):
        command = terminal_command('/tmp/space and 中文', '/tmp/a b.yaml')
        self.assertEqual(command.count('--tab'), 3)
        argv = [shlex.split(value.split('=', 1)[1]) for value in command if value.startswith('--command=')]
        self.assertEqual([value[value.index('--worker')+1] for value in argv], list(ORDER))
        self.assertTrue(all(value[value.index('--session')+1] == '/tmp/space and 中文' for value in argv))

    def test_run_stop_and_restart_in_same_worker(self):
        worker = self.spawn('base')
        self.adapter.start('base')
        self.wait_for(lambda: read_json(self.root/'base.state.json').get('phase') == 'running')
        state = read_json(self.root/'base.state.json')
        launch = state['launch_pid']
        self.wait_for(lambda: (self.root/'base.log').exists() and b'test-output' in (self.root/'base.log').read_bytes())
        self.adapter.stop('base')
        self.assertIsNone(process_start(launch))
        self.assertIsNone(worker.poll())
        self.adapter.start('base')
        self.wait_for(lambda: read_json(self.root/'base.state.json').get('generation') == 2)
        self.assertEqual(read_json(self.root/'base.state.json')['pid'], worker.pid)
        self.adapter.stop('base')

    def test_exit_detected_even_while_tab_worker_lives(self):
        worker = self.spawn('motion')
        self.adapter.start('motion')
        self.wait_for(lambda: read_json(self.root/'motion.state.json').get('phase') == 'running')
        os.kill(read_json(self.root/'motion.state.json')['launch_pid'], signal.SIGTERM)
        self.wait_for(lambda: self.adapter.exited('motion'))
        self.assertIsNone(worker.poll())
        self.adapter.stop('motion')

    def test_ctrl_c_requests_whole_stack_stop(self):
        worker = self.spawn('base')
        self.adapter.start('base')
        self.wait_for(lambda: read_json(self.root/'base.state.json').get('phase') == 'running')
        launch = read_json(self.root/'base.state.json')['launch_pid']
        worker.send_signal(signal.SIGINT)
        worker.wait(timeout=4)
        self.assertEqual(read_json(self.root/'stop-request.json')['component'], 'base')
        self.assertIsNone(process_start(launch))

    def test_all_tabs_can_quit_together(self):
        workers = [self.spawn(name) for name in ORDER]
        for name in ORDER:
            self.adapter.start(name)
        self.wait_for(lambda: all(read_json(self.root/(name+'.state.json')).get('phase') == 'running' for name in ORDER))
        launches = [read_json(self.root/(name+'.state.json'))['launch_pid'] for name in ORDER]
        for name in ORDER:
            self.adapter.command(name, 'quit')
        for worker in workers:
            self.assertEqual(worker.wait(timeout=4), 0)
        self.assertTrue(all(process_start(pid) is None for pid in launches))
        self.assertIn(read_json(self.root/'stop-request.json')['component'], ORDER)

    def test_manager_loss_stops_child(self):
        worker = self.spawn('base')
        self.adapter.start('base')
        self.wait_for(lambda: read_json(self.root/'base.state.json').get('phase') == 'running')
        launch = read_json(self.root/'base.state.json')['launch_pid']
        write_json(self.root/'manager.json', {'pid': 99999999, 'heartbeat': time.time()})
        worker.wait(timeout=4)
        self.assertIsNone(process_start(launch))


if __name__ == '__main__':
    unittest.main()
