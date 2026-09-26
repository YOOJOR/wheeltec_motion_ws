"""No ROS/hardware needed: exercise failure handling and actual process cleanup."""
import importlib.util
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
import unittest

spec = importlib.util.spec_from_file_location('supervisor', Path(__file__).with_name('supervisor.py'))
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class FakeHealth:
    def __init__(self):
        self.status = {name: True for name in m.ORDER}
    def reset(self, name):
        pass
    def healthy(self, name):
        return self.status[name], 'test unhealthy'


class FakeProcesses:
    def __init__(self):
        self.events, self.dead = [], set()
    def start(self, name):
        self.events.append(('start', name)); self.dead.discard(name)
    def stop(self, name):
        self.events.append(('stop', name))
    def exited(self, name):
        return name in self.dead


class SupervisorTests(unittest.TestCase):
    def setUp(self):
        self.config = m.load_config(Path(__file__).with_name('config.yaml'))
        self.clock = 100.0
        self.health, self.proc = FakeHealth(), FakeProcesses()
        self.runner = m.Supervisor(self.config, self.proc, self.health, lambda: self.clock)
    def ready_stack(self):
        for _ in range(5): self.runner.tick()
        self.assertEqual(self.runner.ready, set(m.ORDER))
    def test_startup_is_ordered_and_requires_data(self):
        self.health.status['base'] = False
        self.runner.tick(); self.runner.tick()
        self.assertEqual(self.proc.events, [('start', 'base')])
        self.health.status['base'] = True
        for _ in range(4): self.runner.tick()
        self.assertEqual([e[1] for e in self.proc.events if e[0]=='start'], list(m.ORDER))
    def test_lidar_exit_stops_motion_before_localization_and_restarts_order(self):
        self.ready_stack(); self.proc.events.clear()
        self.proc.dead.add('livox'); self.runner.tick()
        self.assertEqual(self.proc.events, [('stop','motion'), ('stop','fastlio'), ('stop','livox')])
        self.assertIn('base', self.runner.ready)
        self.runner.tick(); self.assertNotIn(('start','livox'),self.proc.events)
        self.clock += 4
        for _ in range(4): self.runner.tick()
        self.assertEqual([e[1] for e in self.proc.events if e[0]=='start'], ['livox','fastlio','motion'])
    def test_stale_pose_withdraws_interface_immediately_then_restarts_fastlio(self):
        self.ready_stack(); self.proc.events.clear()
        self.health.status['fastlio'] = False; self.runner.tick()
        self.assertEqual(self.proc.events, [('stop','motion')])
        self.clock += self.config['unhealthy_grace_sec']+0.1; self.runner.tick()
        self.assertIn(('stop','fastlio'), self.proc.events)
        self.assertNotIn(('stop','base'),self.proc.events)
    def test_motion_exit_only_restarts_motion(self):
        self.ready_stack(); self.proc.events.clear()
        self.proc.dead.add('motion'); self.runner.tick()
        self.assertEqual(self.proc.events, [('stop','motion')])
    def test_communication_exit_preserves_map_and_withdraws_interface(self):
        self.ready_stack(); self.proc.events.clear()
        self.proc.dead.add('base'); self.runner.tick()
        self.assertEqual(self.proc.events, [('stop','motion'), ('stop','base')])
    def test_shutdown_order(self):
        self.ready_stack(); self.proc.events.clear(); self.runner.shutdown()
        self.assertEqual(self.proc.events, [('stop',v) for v in reversed(m.ORDER)])
    def test_startup_timeout_and_backoff(self):
        self.health.status['base'] = False
        self.runner.tick(); self.clock += 46; self.runner.tick()
        first_delay = self.runner.retry_at['base']-self.clock
        self.clock += first_delay+0.1; self.runner.tick()
        self.clock += 46; self.runner.tick()
        self.assertGreater(self.runner.retry_at['base']-self.clock, first_delay)
    def test_real_process_group_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            pidfile = Path(directory)/'child.pid'
            code = "import subprocess,time; from pathlib import Path; p=subprocess.Popen(['sleep','60']); Path(%r).write_text(str(p.pid)); time.sleep(60)" % str(pidfile)
            cfg = dict(self.config, commands={'base':[sys.executable,'-c',code]},
                       shutdown_grace_sec=0.2, shutdown_term_sec=0.2)
            proc = m.Processes(cfg,directory)
            try:
                proc.start('base')
                deadline=time.monotonic()+2
                while not pidfile.exists() and time.monotonic()<deadline: time.sleep(0.02)
                self.assertTrue(pidfile.exists())
                child=int(pidfile.read_text()); proc.stop('base')
                path=Path(f'/proc/{child}/stat')
                # A killed orphan may briefly be a zombie awaiting PID 1 reaping.
                self.assertTrue(not path.exists() or path.read_text().split()[2]=='Z')
            finally:
                proc.stop('base')


if __name__ == '__main__':
    unittest.main()
