"""Regression: lidar-direct.py driver robustness against the fake module of
test_lidar_serial.py (pty, no hardware). K1 bug hunt 2026-11-16:

- serial requests are never made from two threads at once;
- #5: termios.error while opening the port (USB re-enumeration) does not
  kill the daemon: ensure_ready() returns False, serve() lives;
- #14: an idle camera HTTP connection does not stall LiDAR requests or the
  prefetch frame stream; the fill light is off again for LiDAR requests
  after a failed camera stream restore; re-init after a module restart
  restores point-cloud level and exposure; the prefetch path recovers from
  one missed frame reply;
- a late reply to the camera's 0x57 "free" request does not break later
  captures.
"""
import importlib.util
import json
import os
import shutil
import socket
import sys
import tempfile
import threading
import time
import tty
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVER = os.path.abspath(os.path.join(HERE, '..'))
sys.path[:0] = [DRIVER, HERE]
import test_lidar_serial as t  # noqa: E402

ls = t.ls
assert os.path.dirname(os.path.abspath(ls.__file__)) == DRIVER, \
    "lidar_serial must come from driver/, got %s" % (ls.__file__,)

spec = importlib.util.spec_from_file_location(
    'lidar_direct', os.path.join(DRIVER, 'lidar-direct.py'))
ld = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ld)

_saved = []


def setUpModule():
    # a private temp dir (removed afterwards); short base so the unix
    # socket paths below it stay under the 107-byte AF_UNIX limit
    base = tempfile.gettempdir()
    if len(base) > 50 and os.path.isdir('/tmp'):
        base = '/tmp'
    d = tempfile.mkdtemp(prefix='ftb-regress-', dir=base)
    _saved.append((tempfile.tempdir, ld.LOG, d))
    tempfile.tempdir = d
    ld.LOG = os.path.join(d, 'lidar-direct.log')


def tearDownModule():
    old, log, d = _saved.pop()
    tempfile.tempdir = old
    ld.LOG = log
    shutil.rmtree(d, ignore_errors=True)


POINTS = [[x * 20 - 14400, -1000, 25500 + x % 7] for x in range(1500)]


class SlowFake(t.FakeModule):
    """Fake module that answers after `delay` s; `drop` = set of cmds whose
    next reply is swallowed once (a missed reply)."""
    delay = 0.0

    def __init__(self, fd, points):
        t.FakeModule.__init__(self, fd, points)
        self.drop = set()

    def handle(self, cmd, p):
        if cmd in self.drop:
            self.drop.discard(cmd)
            return
        if self.delay:
            time.sleep(self.delay)
        t.FakeModule.handle(self, cmd, p)


def make_fake(delay=0.0):
    master, slave = os.openpty()
    tty.setraw(master)
    fake = SlowFake(master, POINTS)
    fake.delay = delay
    fake.start()
    return fake, os.ttyname(slave)


def start_server(port, **kw):
    sock = os.path.join(tempfile.mkdtemp(), 'uds')
    srv = ld.Server(sock, port, **kw)
    th = threading.Thread(target=srv.serve, daemon=True)
    th.start()
    for _ in range(100):
        if os.path.exists(sock):
            break
        time.sleep(0.02)
    return srv, sock, th


def client(sock):
    c = socket.socket(socket.AF_UNIX)
    c.connect(sock)
    c.settimeout(15)
    return c


def q(c, **msg):
    c.sendall(json.dumps(msg).encode() + b'\x03')
    buf = b''
    while not buf.endswith(b'\x03'):
        buf += c.recv(1 << 20)
    return json.loads(buf[:-1])


class SerialRequestsSerialised(unittest.TestCase):
    """serve()'s own startup ensure_ready() and a second thread using
    srv.dev never talk to the serial port at the same time."""

    def test_two_threads_inside_request(self):
        fake, port = make_fake(delay=0.02)       # widen the window like a loaded CI box
        inside, overlap = [0], []
        lock = threading.Lock()
        orig = ls.LidarSerial.request

        def spy(self, *a, **k):
            with lock:
                inside[0] += 1
                if inside[0] > 1:
                    overlap.append(threading.current_thread().name)
            try:
                return orig(self, *a, **k)
            finally:
                with lock:
                    inside[0] -= 1
        ls.LidarSerial.request = spy
        try:
            srv, sock, _ = start_server(port)
            d = srv.dev
            err = None
            try:                                   # exactly test_pipeline's body
                srv.ensure_ready()
                d.request(ls.STREAM_OFF)
                d.capture_jpeg(0)
            except Exception as e:                 # noqa: BLE001
                err = e
        finally:
            ls.LidarSerial.request = orig
        self.assertEqual(overlap, [], 'serial requests from two threads at once '
                         '(serve startup init + test thread); test error: %r' % (err,))

    def test_control_after_serve_idle_passes(self):
        """Control: the same steps after serve's startup init (one socket
        round trip) work every time."""
        fake, port = make_fake(delay=0.02)
        srv, sock, _ = start_server(port)
        q(client(sock), control='get_laser_gain')  # serialised after serve's init
        for _ in range(5):
            srv.dev.request(ls.STREAM_OFF)
            self.assertEqual(len(srv.dev.capture_jpeg(0)), 4504)


class TermiosErrorHandled(unittest.TestCase):
    """#5: termios.error is not an OSError; open() failing in tcgetattr (EIO
    while the USB device re-enumerates; ENOTTY here) must stay inside
    ensure_ready() (False + retry)."""

    def test_ensure_ready_returns_false(self):
        srv = ld.Server(os.path.join(tempfile.mkdtemp(), 'uds'), '/dev/null')
        try:
            ok = srv.ensure_ready()
        except Exception as e:                     # noqa: BLE001
            self.fail('ensure_ready raised %s: %s (expected False + retry)' % (type(e).__name__, e))
        self.assertFalse(ok)

    def test_serve_survives(self):
        srv, sock, th = start_server('/dev/null')
        time.sleep(0.5)
        self.assertTrue(th.is_alive(), 'serve() thread died: daemon exits, socket left behind')


class IdleHttpDoesNotStall(unittest.TestCase):
    """#14: camera HTTP is served off the main loop: a TCP connection that
    sends nothing must not delay LiDAR requests. Was: camera_http() inline
    in the single serve loop with a 2 s recv
    timeout: a TCP connection that sends nothing (browser preconnect, port
    scan, half-open Wi-Fi client) blocks every LiDAR request."""

    def test_lidar_request_latency(self):
        fake, port = make_fake()
        web = socket.socket()
        web.bind(('127.0.0.1', 0))
        cport = web.getsockname()[1]
        web.close()
        srv, sock, _ = start_server(port, camera_port=cport)
        srv.camera_enabled = lambda: False
        c = client(sock)
        q(c, control='get_laser_gain')             # warm
        idle = socket.create_connection(('127.0.0.1', cport))   # sends nothing
        time.sleep(0.05)
        t0 = time.monotonic()
        q(c, control='get_laser_gain')
        dt = time.monotonic() - t0
        idle.close()
        self.assertLess(dt, 0.5, 'LiDAR request took %.2f s behind an idle HTTP connection' % dt)

    def test_prefetch_stream_survives_idle_connection(self):
        fake, port = make_fake()
        web = socket.socket()
        web.bind(('127.0.0.1', 0))
        cport = web.getsockname()[1]
        web.close()
        srv, sock, _ = start_server(port, camera_port=cport)
        srv.camera_enabled = lambda: False
        c = client(sock)
        q(c, control='set_laser_open'); q(c, control='open_flow')
        starts = []
        orig_start = srv.stream.start
        srv.stream.start = lambda: (starts.append(1), orig_start())
        q(c, control='get_point_cloud', prefetch=1)
        idles = [socket.create_connection(('127.0.0.1', cport)) for _ in range(2)]
        time.sleep(0.05)
        t0 = time.monotonic()
        q(c, control='get_point_cloud', prefetch=1)   # waits behind them
        dt = time.monotonic() - t0
        for s in idles:
            s.close()
        # 1 start = the frame stream of this scan kept running; 2 = its 2 s
        # watchdog fired while the loop was blocked and it was restarted
        self.assertEqual(len(starts), 1, 'stream restarted (%d starts), frame took %.2f s'
                         % (len(starts), dt))


class FillLightOffAfterReinit(unittest.TestCase):
    """#14: after a failed camera stream restore the next LiDAR request has
    the fill light off. Was: camera_jpeg turns the light on; if restoring
    the stream times out
    it closes the port with the light still on; the next LiDAR request's
    re-init sets self.led = False without switching the module's LED off, so
    reply() never turns it off and the LiDAR measures with the light on."""

    def test_led_off_for_lidar_request(self):
        fake, port = make_fake()
        srv = ld.Server(os.path.join(tempfile.mkdtemp(), 'uds'), port)
        self.assertTrue(srv.ensure_ready())
        srv.dev.set_laser(True); srv.dev.open_flow()
        srv.flow = srv.laser = True
        srv.camera_enabled = lambda: True
        time.sleep(3.1)                            # last init attempt > 3 s ago, as in service
        srv.last_use = 0.0
        fake.drop.add(0x01)                        # one missed reply: stream-on restore
        jpg, state = srv.camera_jpeg(0, True)
        self.assertEqual(fake.led, 1)
        self.assertFalse(srv.dev.ready)            # closed by the failed restore
        r = json.loads(srv.reply({'control': 'get_laser_exposure'}))
        self.assertEqual(r['result']['code'], 0)
        self.assertEqual(fake.led, 0, 'fill light still ON in the module after a LiDAR request '
                         '(srv.led=%r)' % (srv.led,))


class ReinitRestoresLevelExposure(unittest.TestCase):
    """#14: re-init restores level and exposure too. Was: call() re-inits
    after a module restart and restores laser + stream (and
    stream mode) but not the RAM-only point-cloud level / exposure: the retried
    frame silently comes back at level 0 / exposure 1000."""

    def test_level_after_module_restart(self):
        fake, port = make_fake()
        srv, sock, _ = start_server(port)
        c = client(sock)
        q(c, control='set_laser_open'); q(c, control='open_flow')
        q(c, control='set_point_cloud_level', level=1)
        q(c, control='set_laser_exposure', exposure=4000)
        self.assertEqual(len(q(c, control='get_point_cloud')['result']['point_cloud']), 750)
        # module power-cycled (laser_power.sh restart / USB drop): RAM state lost
        fake.authorized, fake.stream, fake.level, fake.exp = False, False, 0, 1000
        r = q(c, control='get_point_cloud')['result']
        self.assertEqual(r['code'], 0)
        self.assertEqual((len(r['point_cloud']), fake.exp), (750, 4000),
                         'frame returned with code 0 at level %d / exposure %d' % (fake.level, fake.exp))


class PrefetchRetries(unittest.TestCase):
    """#14: one missed 0x04 reply is retried on both paths: plain
    get_point_cloud (call()) and the prefetch path used by dwell scans
    (was: code 102)."""

    def test_plain_path_recovers(self):
        fake, port = make_fake()
        srv, sock, _ = start_server(port)
        c = client(sock)
        q(c, control='set_laser_open'); q(c, control='open_flow')
        fake.drop.add(0x04)
        self.assertEqual(q(c, control='get_point_cloud')['result']['code'], 0)

    def test_prefetch_path_recovers(self):
        fake, port = make_fake()
        srv, sock, _ = start_server(port)
        c = client(sock)
        q(c, control='set_laser_open'); q(c, control='open_flow')
        self.assertEqual(q(c, control='get_point_cloud', prefetch=1)['result']['code'], 0)
        while srv.stream.frames:                   # drain, then lose one reply
            q(c, control='get_point_cloud', prefetch=1)
        time.sleep(1.0)
        q(c, control='get_point_cloud', prefetch=1)  # keep the stream alive past next_init
        time.sleep(1.0)
        q(c, control='get_point_cloud', prefetch=1)
        time.sleep(1.2)
        fake.drop.add(0x04)
        codes = [q(c, control='get_point_cloud', prefetch=1)['result']['code'] for _ in range(6)]
        self.assertNotIn(102, codes, 'prefetch codes after one missed frame: %s' % codes)


class StaleIrReplyDropped(unittest.TestCase):
    """Stale replies are dropped. Was: request() never dropped a
    stale reply with the same cmd; if capture_jpeg's
    final 0x57 'free' request times out (its reply comes late), every later
    camera_jpeg capture is shifted by one 0x57 reply and fails 'incomplete'."""

    def test_camera_recovers(self):
        fake, port = make_fake()
        srv = ld.Server(os.path.join(tempfile.mkdtemp(), 'uds'), port)
        self.assertTrue(srv.ensure_ready())
        srv.camera_enabled = lambda: True
        orig = fake.handle

        def late_free(cmd, p, once=[True]):
            if cmd == 0x57 and p[:1] == b'\x02' and once[0]:
                once[0] = False
                time.sleep(3.3)                    # > the 3 s timeout of the free request
            orig(cmd, p)
        fake.handle = late_free
        srv.last_use = 0.0
        srv.camera_jpeg(0, False)                  # first capture ok, its free times out
        time.sleep(0.5)
        states = []
        for _ in range(3):
            srv.cam_time = 0.0
            states.append(srv.camera_jpeg(0, False)[1])
        self.assertEqual(states, ['live'] * 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
