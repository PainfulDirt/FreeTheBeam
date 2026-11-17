"""lidar_service.py on the printer itself (local mode): the driver over
its unix socket, and --profile printer (scan cap, no raw files,
data-folder trim, numpy/PIL work refused)."""
import json
import os
import shutil
import socketserver
import sys
import tempfile
import threading
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "..",
                                                     "klipper")]
import lidar_service as svc_mod  # noqa: E402

ETX = b"\x03"


class _DriverHandler(socketserver.StreamRequestHandler):
    """Fake lidar-direct.py: every ETX-terminated JSON request gets
    {"control": <same>, "result": {"code": 0}} + ETX."""

    def handle(self):
        buf = b""
        while True:
            d = self.request.recv(4096)
            if not d:
                return
            buf += d
            while ETX in buf:
                one, buf = buf.split(ETX, 1)
                msg = json.loads(one.decode())
                self.request.sendall(json.dumps(
                    {"control": msg.get("control"),
                     "result": {"code": 0}}).encode() + ETX)


class FakeDriver:
    def __init__(self, path):
        self.path = path
        self.srv = None

    def start(self):
        if os.path.exists(self.path):
            os.remove(self.path)
        self.srv = socketserver.ThreadingUnixStreamServer(self.path,
                                                          _DriverHandler)
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def stop(self):
        self.srv.shutdown()
        self.srv.server_close()
        # the server keeps the open connections: close them like a dying
        # driver would
        os.remove(self.path)


class UnixMiddlewareTests(unittest.TestCase):
    def setUp(self):
        self.d = tempfile.mkdtemp()
        self.path = os.path.join(self.d, "ai_server_uds")
        self.drv = FakeDriver(self.path)
        self.drv.start()
        self.addCleanup(shutil.rmtree, self.d, True)

    def test_unix_middleware_roundtrip(self):
        mw = svc_mod.Middleware("unix:" + self.path)
        r = mw.call({"control": "get_sensor_temp"}, timeout=5.0)
        self.assertEqual(r["result"]["code"], 0)
        self.assertEqual(r["control"], "get_sensor_temp")
        mw.close()
        self.drv.stop()

    def test_unix_middleware_reconnects(self):
        mw = svc_mod.Middleware("unix:" + self.path)
        self.assertEqual(mw.call({"control": "a"}, timeout=5.0)
                         ["result"]["code"], 0)
        # the driver restarts (lidar-start.sh restart): old connection dead
        self.drv.stop()
        try:
            mw.sock.shutdown(2)       # what the dying driver does to it
        except OSError:
            pass
        self.drv.start()
        self.assertEqual(mw.call({"control": "b"}, timeout=5.0)
                         ["result"]["code"], 0)
        mw.close()
        self.drv.stop()

    def test_unix_driver_missing_gives_error_then_reconnects(self):
        mw = svc_mod.Middleware("unix:" + self.path)
        self.drv.stop()
        with self.assertRaises(RuntimeError):
            mw.call({"control": "a"}, timeout=2.0)
        self.drv.start()
        self.assertEqual(mw.call({"control": "b"}, timeout=5.0)
                         ["result"]["code"], 0)
        mw.close()
        self.drv.stop()

    def test_service_middleware_request_driver_down(self):
        s = svc_mod.Service(svc_mod.Middleware("unix:" + self.path),
                            os.path.join(self.d, "data"))
        self.drv.stop()
        r = svc_mod.Handler.one(s, json.dumps(
            {"cmd": "middleware", "msg": {"control": "x"},
             "timeout": 3}).encode() + b"\n")
        self.assertFalse(r["ok"])
        self.drv.start()
        r = s.handle({"cmd": "middleware", "msg": {"control": "x"}})
        self.assertTrue(r["ok"])
        s.mw.close()
        self.drv.stop()

    def test_plain_socket_path(self):
        # --socket PATH (no "unix:" prefix) works the same
        mw = svc_mod.Middleware(self.path)
        self.assertEqual(mw.call({"control": "a"}, timeout=5.0)
                         ["result"]["code"], 0)
        mw.close()
        self.drv.stop()

    def test_profile_arg(self):
        ap = svc_mod.build_parser()
        a = ap.parse_args(["--socket", "/tmp/ai_server_uds", "--profile",
                           "printer"])
        self.assertEqual(a.profile, "printer")
        self.assertEqual(ap.parse_args(["--printer", "h:7130"]).profile,
                         "helper")


class FakeMW:
    def __init__(self):
        self.calls = []

    def call(self, msg, timeout=15.0):
        self.calls.append(msg)
        return {"control": msg["control"], "result": {"code": 0}}


BEGIN = {"x": 100.0, "z": 3.0, "x_dir": -1.0, "x_offset": 0.0,
         "y_offset": 0.0, "grid": [0.0, 1.0, 0.5]}
PC = [[(i - 50) * 100, 0, 25600 + i] for i in range(100)]


def scan(s, name):
    s.handle(dict(BEGIN, cmd="begin", name=name))
    for row in range(3):
        s.handle({"cmd": "frame", "name": name, "row": row,
                  "raw": {"result": {"point_cloud": PC}}})
    return s.handle({"cmd": "end", "name": name})


class ProfileTests(unittest.TestCase):
    def make(self, **kw):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, True)
        return svc_mod.Service(FakeMW(), d, **kw), d

    def test_printer_keeps_six_scans(self):
        s, d = self.make(profile="printer")
        for i in range(8):
            self.assertTrue(scan(s, "s%d" % i)["ok"])
        self.assertEqual(len(s.scans), 6)
        self.assertIn("s7", s.scans)

    def test_printer_no_raw_files(self):
        s, d = self.make(profile="printer")
        r = scan(s, "a")
        self.assertTrue(r["ok"])
        self.assertEqual([n for n in os.listdir(d)
                          if n.endswith(".raw.jsonl")], [])
        self.assertEqual(len(s.scans["a"]["rows"]), 3)
        self.assertTrue(os.path.exists(os.path.join(d, "a.json")))

    def test_helper_profile_unchanged(self):
        s, d = self.make()
        self.assertEqual(s.KEEP_SCANS, 16)
        self.assertFalse(s.trim_bytes)
        for i in range(18):
            scan(s, "s%d" % i)
        self.assertEqual(len(s.scans), 16)
        self.assertTrue(os.path.exists(os.path.join(d, "s17.raw.jsonl")))

    def test_printer_numpy_refused(self):
        s, d = self.make(profile="printer")
        saved = dict((m, sys.modules.get(m)) for m in (
            "numpy", "lidar_camera", "lidar_bands"))
        sys.modules["numpy"] = None          # import numpy -> ImportError
        for m in ("lidar_camera", "lidar_bands"):
            sys.modules.pop(m, None)
        try:
            for req in ({"cmd": "photo", "job": "j", "name": "t", "x": 150,
                         "y": 150},
                        {"cmd": "pf_analyze", "job": "j", "test": {}}):
                r = svc_mod.Handler.one(s, json.dumps(req).encode() + b"\n")
                self.assertFalse(r["ok"], r)
                self.assertIn("needs the helper", r["error"])
                self.assertIn(req["cmd"], r["error"])
            # no picture taken for nothing
            self.assertNotIn("ir_photo", [m["control"] for m in s.mw.calls])
        finally:
            for m, v in saved.items():
                if v is None:
                    sys.modules.pop(m, None)
                else:
                    sys.modules[m] = v

    def test_printer_trims_after_save(self):
        s, d = self.make(profile="printer")
        s.trim_bytes = 1                     # everything old goes
        old = os.path.join(d, "old.json")
        with open(old, "w") as f:
            f.write("x" * 1000)
        os.utime(old, (1, 1))
        scan(s, "a")
        self.assertFalse(os.path.exists(old))
        self.assertTrue(os.path.exists(os.path.join(d, "a.json")))


class TrimTests(unittest.TestCase):
    MB = 1024 * 1024

    def test_trim_dir(self):
        top = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, top, True)
        d = os.path.join(top, "data")
        os.makedirs(os.path.join(d, "meshes"))
        outside = os.path.join(top, "outside.json")
        with open(outside, "wb") as f:
            f.write(b"x" * self.MB)
        os.utime(outside, (1, 1))
        now = __import__("time").time()
        names = ["h1.json", "meshes/h2.html", "h3.jsonl", "h4.jpg",
                 "h5.json"]
        for i, n in enumerate(names):
            p = os.path.join(d, n)
            with open(p, "wb") as f:
                f.write(b"x" * self.MB)
            t = now - (5 - i) * 3600          # h1 oldest (5 h)
            os.utime(p, (t, t))
        young = os.path.join(d, "young.json")
        with open(young, "wb") as f:
            f.write(b"x" * self.MB)
        os.utime(young, (now - 60, now - 60))
        other = os.path.join(d, "keep.cfg")       # not ours to delete
        with open(other, "wb") as f:
            f.write(b"x" * self.MB)
        os.utime(other, (1, 1))
        link = os.path.join(d, "link.json")       # never followed
        os.symlink(outside, link)
        os.utime(link, (1, 1), follow_symlinks=False)

        removed = svc_mod.trim_dir(d, int(2.5 * self.MB))
        left = sorted(os.path.relpath(os.path.join(r, f), d)
                      for r, _, fs in os.walk(d) for f in fs)
        # 7 MB counted (6 data + keep.cfg), oldest data files go until
        # <= 2.5 MB: h1..h4 and h5 (keep.cfg can't go), young stays
        self.assertEqual(left, ["keep.cfg", "link.json", "young.json"])
        self.assertEqual(removed, 5 * self.MB)
        self.assertTrue(os.path.exists(outside))

    def test_trim_dir_under_limit_noop(self):
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, True)
        p = os.path.join(d, "a.json")
        with open(p, "w") as f:
            f.write("x")
        os.utime(p, (1, 1))
        self.assertEqual(svc_mod.trim_dir(d, 10), 0)
        self.assertTrue(os.path.exists(p))


if __name__ == "__main__":
    unittest.main()
