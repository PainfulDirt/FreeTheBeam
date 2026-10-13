"""Tests for driver/examples/dump_profile.py against the daemon + fake module."""

import json
import os
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER = os.path.join(HERE, '..')
EXAMPLE = os.path.join(DRIVER, 'examples', 'dump_profile.py')
sys.path.insert(0, HERE)
sys.path.insert(0, DRIVER)
sys.path.insert(0, os.path.join(DRIVER, 'examples'))
import test_lidar_direct as tld  # noqa: E402
import test_lidar_serial as t  # noqa: E402
import dump_profile as dp  # noqa: E402

ld = tld.ld


def start_daemon(points):
    master, slave = os.openpty()
    tty.setraw(master)
    fake = t.FakeModule(master, points)
    fake.start()
    path = os.path.join(tempfile.mkdtemp(), 'uds')
    srv = ld.Server(path, os.ttyname(slave))
    threading.Thread(target=srv.serve, daemon=True).start()
    for _ in range(50):
        if os.path.exists(path):
            break
        time.sleep(0.05)
    return path, fake


def run(*args):
    return subprocess.run([sys.executable, EXAMPLE] + list(args),
                          capture_output=True, text=True, timeout=30)


class DumpProfileTests(unittest.TestCase):
    def setUp(self):
        self.points = [[x * 20 - 14400, -1000, 25500 + x % 7] for x in range(1500)]
        self.sock, self.fake = start_daemon(self.points)

    def test_fetch_profile_returns_points(self):
        self.assertEqual(dp.fetch_profile(self.sock), self.points)

    def test_cli_summary(self):
        p = run('--socket', self.sock)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertIn('1500 points', p.stdout)

    def test_out_file_written(self):
        out = os.path.join(tempfile.mkdtemp(), 'p.json')
        self.assertEqual(run('--socket', self.sock, '--out', out).returncode, 0)
        self.assertEqual(json.load(open(out)), self.points)

    def test_no_daemon_message_and_exit_1(self):
        p = run('--socket', os.path.join(tempfile.mkdtemp(), 'nothing'))
        self.assertEqual(p.returncode, 1)
        self.assertIn('is lidar-direct.py running?', p.stderr)

    def test_code_4_reported(self):
        # empty frame: the daemon answers a non-zero code; the tool names it
        sock, _ = start_daemon([])
        p = run('--socket', sock)
        self.assertEqual(p.returncode, 1)
        self.assertNotIn('Traceback', p.stderr)
        self.assertIn('code', p.stderr)


ROOT = os.path.abspath(os.path.join(DRIVER, '..'))


class PackagingTests(unittest.TestCase):
    def test_install_sh_requires_address(self):
        p = subprocess.run(['sh', os.path.join(ROOT, 'helper', 'install.sh')],
                           capture_output=True, text=True, timeout=30)
        self.assertEqual(p.returncode, 1)
        self.assertIn('usage', p.stderr)

    def test_shell_scripts_parse(self):
        for f in ('helper/install.sh', 'k1max/lidar-start.sh'):
            r = subprocess.run(['sh', '-n', os.path.join(ROOT, f)], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)

    def test_python_files_compile(self):
        for f in ('klipper/lidar.py', 'helper/lidar_service.py', 'helper/lidar-bridge.py'):
            src = open(os.path.join(ROOT, f)).read()
            compile(src, f, 'exec')

    def test_service_requires_printer(self):
        p = subprocess.run([sys.executable, os.path.join(ROOT, 'helper', 'lidar_service.py')],
                           capture_output=True, text=True, timeout=30,
                           env=dict(os.environ, PYTHONPATH=os.path.join(ROOT, 'klipper')))
        self.assertNotEqual(p.returncode, 0)
        self.assertIn('--printer', p.stderr)


if __name__ == '__main__':
    unittest.main(verbosity=2)
