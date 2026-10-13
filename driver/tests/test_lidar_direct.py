"""Offline test of driver/lidar-direct.py: the daemon against the fake module
(test_lidar_serial.FakeModule) on a pty, a client like lidar.py on its socket."""

import contextlib
import importlib.util
import io
import json
import os
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tty
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
CUSTOM = os.path.join(HERE, '..')
sys.path.insert(0, CUSTOM)
sys.path.insert(0, HERE)
import test_lidar_serial as t  # noqa: E402

spec = importlib.util.spec_from_file_location('lidar_direct', os.path.join(CUSTOM, 'lidar-direct.py'))
ld = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ld)
ld.LOG = os.path.join(tempfile.gettempdir(), 'lidar-direct-test.log')


class DirectTests(unittest.TestCase):
    def setUp(self):
        master, slave = os.openpty()
        tty.setraw(master)
        self.points = [[x * 20 - 14400, -1000, 25500 + x % 7] for x in range(1500)]
        self.fake = t.FakeModule(master, self.points)
        self.fake.start()
        self.sock = os.path.join(tempfile.mkdtemp(), 'uds')
        self.srv = ld.Server(self.sock, os.ttyname(slave))
        threading.Thread(target=self.srv.serve, daemon=True).start()
        for _ in range(50):
            if os.path.exists(self.sock):
                break
            time.sleep(0.05)
        self.c = socket.socket(socket.AF_UNIX)
        self.c.connect(self.sock)
        self.c.settimeout(5)

    def q(self, **msg):
        self.c.sendall(json.dumps(msg).encode() + b'\x03')
        buf = b''
        while not buf.endswith(b'\x03'):
            buf += self.c.recv(1 << 20)
        return json.loads(buf[:-1])

    def test_like_lidar_py(self):
        self.assertEqual(self.q(control='get_laser_exposure'),
                         {'control': 'get_laser_exposure', 'result': {'code': 0, 'exposure': 1000}})
        self.assertEqual(self.q(control='get_point_cloud')['result']['code'], 4)   # stream off
        for c in ('set_laser_open', 'open_flow'):
            self.assertEqual(self.q(control=c)['result'], {'code': 0})
        self.assertEqual(self.q(control='set_point_cloud_level', level=0)['result'], {'code': 0})
        r = self.q(control='get_point_cloud')
        self.assertEqual(r['result']['point_cloud'], self.points)
        self.q(control='set_point_cloud_level', level=1)
        self.assertEqual(len(self.q(control='get_point_cloud')['result']['point_cloud']), 750)
        self.assertEqual(self.q(control='get_point_cloud_level')['result'], {'code': 0, 'level': 1})
        for c in ('close_flow', 'set_laser_close'):
            self.assertEqual(self.q(control=c)['result'], {'code': 0})

    def test_prefetch_stream(self):
        self.q(control='set_laser_open'); self.q(control='open_flow')
        ts = []
        for i in range(6):
            r = self.q(control='get_point_cloud', prefetch=1)['result']
            self.assertEqual(r['code'], 0)
            self.assertEqual(r['point_cloud'], self.points)
            ts.append(r['t'])
            time.sleep(0.05)            # a slow client doesn't stop the stream
        self.assertEqual(ts, sorted(ts))
        self.assertTrue(self.srv.stream.running)
        self.assertEqual(self.q(control='get_laser_exposure')['result']['exposure'], 1000)
        self.assertFalse(self.srv.stream.running)   # stopped for the other command
        r = self.q(control='get_point_cloud')['result']
        self.assertEqual(len(r['point_cloud']), 1500)
        self.assertIn('t', r)

    def test_refused_and_bad(self):
        for c in ('set_laser_cali_step', 'create_ir_image', 'get_flow_detection', 'nonsense'):
            self.assertEqual(self.q(control=c, step=1)['result']['code'], 103)
        self.assertEqual(self.q(control='set_point_cloud_level', level=7)['result']['code'], 103)
        self.c.sendall(b'not json\x03')
        buf = b''
        while not buf.endswith(b'\x03'):
            buf += self.c.recv(4096)
        self.assertEqual(json.loads(buf[:-1])['result']['code'], 103)

    def test_two_clients_and_split_requests(self):
        c2 = socket.socket(socket.AF_UNIX)
        c2.connect(self.sock)
        c2.settimeout(5)
        msg = json.dumps({'control': 'get_laser_gain'}).encode() + b'\x03'
        c2.sendall(msg[:5]); time.sleep(0.1); c2.sendall(msg[5:])
        buf = b''
        while not buf.endswith(b'\x03'):
            buf += c2.recv(4096)
        self.assertEqual(json.loads(buf[:-1])['result'], {'code': 0, 'gain': 1000})
        self.assertEqual(self.q(control='get_laser_exposure')['result']['exposure'], 1000)

    def test_module_lost_auth_reinit(self):
        self.q(control='set_laser_open'); self.q(control='open_flow')
        self.fake.authorized = False          # e.g. the module was power-cycled
        self.fake.stream = False
        r = self.q(control='get_point_cloud')   # times out once, re-inits, retries
        self.assertEqual(r['result']['code'], 0)
        self.assertEqual(len(r['result']['point_cloud']), 1500)


class CliTests(unittest.TestCase):
    def test_parse_args_defaults(self):
        a = ld.parse_args([])
        self.assertEqual(a.socket, '/tmp/ai_server_uds')
        self.assertEqual(a.port, ld.ls.PORT)

    def test_parse_args_overrides(self):
        a = ld.parse_args(['--socket', '/x', '--port', '/dev/ttyUSB1'])
        self.assertEqual((a.socket, a.port), ('/x', '/dev/ttyUSB1'))

    def test_help_exits_zero(self):
        with self.assertRaises(SystemExit) as cm, contextlib.redirect_stdout(io.StringIO()):
            ld.parse_args(['--help'])
        self.assertEqual(cm.exception.code, 0)

    def test_missing_port_clear_notice(self):
        p = subprocess.Popen([sys.executable, os.path.join(CUSTOM, 'lidar-direct.py'),
                              '--port', '/nonexistent', '--socket',
                              os.path.join(tempfile.mkdtemp(), 'uds')],
                             stderr=subprocess.PIPE, text=True)
        try:
            line = p.stderr.readline()      # printed at startup, before the retry loop
        finally:
            p.kill()
            p.wait()
        self.assertIn('/nonexistent', line)
        self.assertIn('waiting', line)
        self.assertNotIn('Traceback', line)

    def test_stale_socket_removed(self):
        path = os.path.join(tempfile.mkdtemp(), 'uds')
        old = socket.socket(socket.AF_UNIX)
        old.bind(path)
        old.close()                       # leaves the socket file behind
        master, slave = os.openpty()
        tty.setraw(master)
        t.FakeModule(master, [[0, 0, 25000]]).start()
        threading.Thread(target=ld.Server(path, os.ttyname(slave)).serve, daemon=True).start()
        c = socket.socket(socket.AF_UNIX)
        for _ in range(100):
            try:
                c.connect(path)
                break
            except OSError:
                time.sleep(0.05)
        else:
            self.fail('could not connect to the new daemon')


if __name__ == '__main__':
    unittest.main(verbosity=2)
