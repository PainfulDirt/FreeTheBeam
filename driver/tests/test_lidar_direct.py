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

    def test_stream_mode(self):
        self.assertEqual(self.fake.mode, 0)      # default: stock mode 0
        self.q(control='set_laser_open'); self.q(control='open_flow')
        self.assertEqual(self.q(control='set_stream_mode', mode=1)['result'], {'code': 0})
        self.assertEqual(self.fake.mode, 1)
        self.assertTrue(self.fake.stream)        # the stream was on: on again
        self.assertEqual(self.q(control='get_stream_mode')['result'], {'code': 0, 'mode': 1})
        self.assertEqual(len(self.q(control='get_point_cloud', prefetch=1)['result']['point_cloud']), 1500)
        self.assertEqual(self.q(control='set_stream_mode', mode=2)['result'], {'code': 103})
        # module restart: mode back to 0, auth lost -> re-init sets mode 1 again
        self.fake.authorized, self.fake.stream, self.fake.mode = False, False, 0
        r = self.q(control='get_point_cloud')
        self.assertEqual(r['result']['code'], 0)
        self.assertEqual(self.fake.mode, 1)
        self.assertEqual(self.q(control='set_stream_mode', mode=0)['result'], {'code': 0})
        self.assertEqual(self.fake.mode, 0)

    def test_module_info_selftest_ambient(self):
        self.assertEqual(self.q(control='get_sensor_temp')['result'], {'code': 0, 'temp': 43.5})
        r = self.q(control='get_module_info')['result']
        self.assertEqual((r['code'], r['version'], r['temp'], r['stream_mode']), (0, '1.0.13', 43.5, 0))
        self.assertEqual(r['cal']['plane'], [0.0, 0.5, 1.0, 1.5])
        self.assertEqual(len(r['cal']['intrinsics']), 9)
        self.q(control='set_laser_open'); self.q(control='open_flow')
        self.assertEqual(self.q(control='self_test')['result'],
                         {'code': 0, 'self_test': 0, 'laser_ok': True, 'light_ok': True})
        self.fake.selftest = 1          # laser weak: a value, not an error
        self.assertEqual(self.q(control='self_test')['result']['laser_ok'], False)
        self.assertFalse(self.srv.flow)
        r = self.q(control='ambient_points')['result']
        self.assertEqual(r, {'code': 0, 'points': [1500, 1500, 1500]})   # the fake ignores the laser
        self.assertFalse(self.srv.flow)  # state as before

    def test_timings(self):
        self.assertEqual(self.q(control='get_timings')['result'], {'code': 103})   # old firmware
        self.assertIsNone(self.q(control='get_module_info')['result']['timings'])
        self.fake.timings = True
        self.assertEqual(self.q(control='get_timings')['result'],
                         {'code': 0, 'temp': 43.5, 'fps': 10, 'alg_ms': 88, 'alg_op_ms': 80, 'wait_ms': 92})
        self.assertEqual(self.q(control='get_sensor_temp')['result'], {'code': 0, 'temp': 43.5})

    def test_camera(self):
        import urllib.request
        import urllib.error
        web = socket.socket()
        web.bind(('127.0.0.1', 0))
        port = web.getsockname()[1]
        web.close()
        sock = self.sock + 'cam'
        srv = ld.Server(sock, self.srv.dev.port, camera_port=port)
        self.srv.dev.close()
        threading.Thread(target=srv.serve, daemon=True).start()
        for _ in range(50):
            if os.path.exists(sock):
                break
            time.sleep(0.05)
        c = socket.socket(socket.AF_UNIX)
        c.connect(sock)
        def q(**msg):
            c.sendall(json.dumps(msg).encode() + b'\x03')
            buf = b''
            while not buf.endswith(b'\x03'):
                buf += c.recv(1 << 20)
            return json.loads(buf[:-1])
        q(control='set_laser_open'); q(control='open_flow')
        def get(path):
            try:
                r = urllib.request.urlopen('http://127.0.0.1:%d%s' % (port, path), timeout=10)
                return r.read(), r.headers.get('X-LiDAR-Camera'), r.headers.get('Content-Type')
            except urllib.error.HTTPError as e:
                return e.read(), e.headers.get('X-LiDAR-Camera'), e.code
        switch = [False]
        srv.camera_enabled = lambda: switch[0]
        off = get('/snapshot')                                    # switch off
        self.assertEqual((off[0], off[1][:3], off[2]), (srv.off_jpeg, 'off', 'image/jpeg'))
        switch[0] = True
        srv.last_use = 0.0                     # pretend the LiDAR has been idle
        jpg, state, ctype = get('/snapshot')
        self.assertEqual((state, ctype, jpg[:2], len(jpg)), ('live', 'image/jpeg', b'\xff\xd8', 4504))
        self.assertTrue(self.fake.stream)      # stream (and laser) restored
        self.assertTrue(srv.flow and srv.laser)
        self.assertEqual(self.fake.led, 1)     # light on and stays on
        self.assertEqual(get('/snapshot')[1], 'cached')          # < 0.3 s old
        time.sleep(0.35)
        self.assertEqual(get('/snapshot')[1], 'live')
        self.assertEqual((self.fake.led, self.fake.captures), (1, 2))
        q(control='get_laser_exposure')                          # LiDAR in use
        self.assertEqual(self.fake.led, 0)     # light off for the LiDAR
        time.sleep(0.35)
        jpg2, state, _ = get('/snapshot')
        self.assertEqual(state, 'cached (LiDAR busy)')
        self.assertEqual(self.fake.captures, 2)
        switch[0] = False
        srv.last_use = 0.0
        srv.light(True)
        self.assertEqual(get('/snapshot')[1][:3], 'off')
        self.assertEqual(self.fake.led, 0)     # switch off -> light off
        self.assertIn(b'<img', get('/')[0])
        c.close()

    def test_ir_photo(self):
        import base64
        self.q(control='set_laser_open'); self.q(control='open_flow')
        r = self.q(control='ir_photo', light=1, laser=0, exposure=3000, res=3)['result']
        self.assertEqual(r['code'], 0, r)
        jpg = base64.b64decode(r['jpeg'])
        self.assertEqual((jpg[:2], len(jpg), r['w'], r['h'], r['exposure']),
                         (b'\xff\xd8', 4504, 1600, 1200, 3000))
        self.assertEqual(r['intr'], [8.0, 8.5, 9.0, 9.5])
        self.assertEqual(self.fake.exp_log[-2:], [3000, 1000])
        self.assertEqual(self.fake.led, 0)
        self.assertTrue(self.fake.stream)      # stream restored

    def test_ir_photo_restores_on_error(self):
        self.fake.fail_capture = True
        r = self.q(control='ir_photo', light=1, laser=0, exposure=4000, res=3)['result']
        self.assertNotEqual(r['code'], 0)
        self.assertEqual(self.fake.exp_log[-1], 1000)
        self.assertEqual(self.fake.led, 0)

    def test_ir_photo_laser(self):
        r = self.q(control='ir_photo', light=0, laser=1, exposure=1000, res=3)['result']
        self.assertEqual(r['code'], 0, r)
        seen = self.fake.seen
        i56 = len(seen) - 1 - seen[::-1].index(0x56)
        i13 = max(i for i, c in enumerate(seen[:i56]) if c == 0x13)
        i02 = max(i for i, c in enumerate(seen[:i56]) if c == 0x02)
        self.assertTrue(i02 < i13 < i56)
        self.assertEqual(self.fake.laser, 0)   # laser off again afterwards

    def test_webcam_follows_switch(self):
        import http.server
        posts = []
        class H(http.server.BaseHTTPRequestHandler):
            def do_POST(self):
                posts.append(json.loads(self.rfile.read(int(self.headers['Content-Length']))))
                self.send_response(200); self.end_headers(); self.wfile.write(b'{}')
            def log_message(self, *a):
                pass
        httpd = http.server.HTTPServer(('127.0.0.1', 0), H)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        self.srv.set_moonraker('http://127.0.0.1:%d' % httpd.server_address[1], 'none', 'LiDAR')
        switch = [True]
        self.srv.camera_enabled = lambda: switch[0]
        self.srv.camera_tick(); self.srv.camera_tick()
        switch[0] = False
        self.srv.camera_tick()
        httpd.shutdown()
        self.assertEqual(posts, [{'name': 'LiDAR', 'enabled': True},
                                 {'name': 'LiDAR', 'enabled': False}])   # only on changes

    def test_pipeline(self):
        d = self.srv.dev
        self.srv.ensure_ready()
        d.request(t.ls.STREAM_OFF)
        jpg = d.capture_jpeg(0)
        self.assertEqual((len(jpg), jpg[:2], jpg[-2:]), (4504, b'\xff\xd8', b'\xff\xd9'))
        self.assertEqual(self.fake.seen.count(0x57), 4)   # 3 chunks + free

    def test_refused(self):
        for cmd in (0x58, 0xAB, 0x12, 0x1B):
            with self.assertRaises(t.ls.LidarError):
                self.srv.dev.request(cmd, b'\x0a')

    def test_stream_mode_at_start(self):
        srv = ld.Server(self.sock + '2', self.srv.dev.port, stream_mode=1)
        self.srv.dev.close()
        self.assertTrue(srv.ensure_ready())
        self.assertEqual(self.fake.mode, 1)
        srv.dev.close()

    def test_camera_switch_from_moonraker(self):
        import http.server
        value = [1]
        queries = []
        class H(http.server.BaseHTTPRequestHandler):
            def do_GET(self):
                queries.append(self.path)
                body = json.dumps({'result': {'status': {
                    'output_pin My_Cam': {'value': value[0]}}}}).encode()
                self.send_response(200); self.end_headers(); self.wfile.write(body)
            def log_message(self, *a):
                pass
        httpd = http.server.HTTPServer(('127.0.0.1', 0), H)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        url = 'http://127.0.0.1:%d' % httpd.server_address[1]
        self.srv.set_moonraker(url, 'output_pin My_Cam')
        self.assertTrue(self.srv.camera_enabled())
        self.assertEqual(queries, ['/printer/objects/query?output_pin%20My_Cam'])
        value[0] = 0
        self.srv.cam_switch_time = -1e9           # past the 3 s cache
        self.assertFalse(self.srv.camera_enabled())
        self.srv.set_moonraker(url, 'output_pin Missing')
        self.srv.cam_switch_time = -1e9
        self.assertFalse(self.srv.camera_enabled())   # unreadable: off
        httpd.shutdown()
        self.srv.set_moonraker(url, 'none')
        self.assertTrue(self.srv.camera_enabled())    # no switch: always on
        self.srv.camera_tick()                         # no webcam: no request
        self.assertIsNone(self.srv.cam_published)

    def test_jpeg_size(self):
        with open(os.path.join(CUSTOM, 'lidar-camera-off.jpg'), 'rb') as f:
            self.assertEqual(ld.jpeg_size(f.read()), (512, 288))
        self.assertIsNone(ld.jpeg_size(b'\xff\xd8' + b'\x01' * 50))


class CliTests(unittest.TestCase):
    def test_parse_args_defaults(self):
        a = ld.parse_args([])
        self.assertEqual(a.socket, '/tmp/ai_server_uds')
        self.assertEqual(a.port, ld.ls.PORT)

    def test_parse_args_overrides(self):
        a = ld.parse_args(['--socket', '/x', '--port', '/dev/ttyUSB1'])
        self.assertEqual((a.socket, a.port), ('/x', '/dev/ttyUSB1'))

    def test_parse_args_camera(self):
        a = ld.parse_args([])
        self.assertEqual((a.stream_mode, a.camera_port, a.camera_switch, a.webcam),
                         (0, None, 'output_pin LiDAR_Camera', None))
        a = ld.parse_args(['--stream-mode', '1', '--camera-port', '8081',
                           '--camera-switch', 'none', '--webcam', 'LiDAR',
                           '--moonraker', 'http://h:7125'])
        self.assertEqual((a.stream_mode, a.camera_port, a.camera_switch, a.webcam,
                          a.moonraker), (1, 8081, 'none', 'LiDAR', 'http://h:7125'))
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            ld.parse_args(['--stream-mode', '2'])

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
