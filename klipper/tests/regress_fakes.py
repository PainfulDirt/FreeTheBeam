"""Fake Klipper pieces for the regression tests ported from the K1 Max bug
hunt (2026-11-16): a reactor with a clock that advances on pause(), a
driver socket that answers like lidar-direct.py, a G-code runner that logs
(and can fail), and make_lidar() for a Lidar built without __init__."""
import json
import os
import shutil
import tempfile

import fake_klipper  # noqa: F401  (puts klipper/ on sys.path)
from fake_klipper import Err, Obj  # noqa: F401
import lidar  # noqa: E402

ETX = b"\x03"
CmdError = Err


class Reactor:
    def __init__(self):
        self.t = 1000.0
        self.pauses = 0

    def monotonic(self):
        return self.t

    def pause(self, until):
        self.pauses += 1
        self.t = max(self.t, until)
        return self.t


class FakeDriverSock:
    """Behaves like lidar-direct.py: one reply per request, in order.
    one_at_a_time: each recv returns one queued reply (replies arrive
    separately); else recv returns everything queued (glued)."""

    def __init__(self, one_at_a_time=True):
        self.q = []
        self.sent = []
        self.one = one_at_a_time

    def sendall(self, data):
        for m in data.split(ETX)[:-1]:
            req = json.loads(m)
            self.sent.append(req)
            c = req["control"]
            if c == "get_point_cloud":
                res = {"code": 0, "t": 1000.0 + len(self.sent) * 0.2,
                       "point_cloud": [[0, i, 3000] for i in range(5)]}
            elif c == "get_sensor_temp":
                res = {"code": 0, "temp": 41.5}
            else:
                res = {"code": 0}
            self.q.append(json.dumps({"control": c, "result": res}).encode()
                          + ETX)

    def recv(self, n):
        if not self.q:
            raise BlockingIOError()
        if self.one:
            return self.q.pop(0)
        out = b"".join(self.q)
        self.q = []
        return out

    def close(self):
        pass


class Gcode:
    def __init__(self, fail_on=None):
        self.scripts = []
        self.fail_on = fail_on

    def run_script_from_command(self, s):
        self.scripts.append(s)
        if self.fail_on and s.startswith(self.fail_on):
            raise CmdError("Move out of range")


class Printer:
    command_error = CmdError

    def __init__(self, objs):
        self.objs = objs

    def lookup_object(self, name, default=None):
        return self.objs.get(name, default)


class Gcmd(fake_klipper.Gcmd):
    """fake_klipper.Gcmd that also keeps the messages as .out."""

    @property
    def out(self):
        return self.info


def make_lidar(objs=None, gcode=None, service=None):
    L = lidar.Lidar.__new__(lidar.Lidar)
    L.reactor = Reactor()
    L.printer = Printer(objs or {})
    L.gcode = gcode or Gcode()
    L.sock = None
    L.sock_path = "/nonexistent"
    L.drv_buf = b""
    L.service = service
    L.svc_sock = None
    L.svc_buf = b""
    L.svc_down = None
    L.x_offset, L.y_offset, L.x_per_z, L.y_per_z = -36.0, -21.2, 0.0, 0.0
    L.offset_z = 3.0
    L.offset_source = "config"
    L.x_dir = -1.0
    L.speed = 50.0
    L.scan_level = 0
    L.scan_z = 3.0
    L.captures = 1
    L.settle = 0.15
    L.mode = "dwell"
    L.frame_period, L.frame_latency, L.frame_spread = 0.235, 0.194, 0.05
    L.dwell_margin, L.dwell_settle = 0.015, 0.05
    L.moving_lag = 0.0
    L.scans = {}
    L.mesh_info = {}
    L.last_result = {}
    L.fl = None
    L.pf = None
    L.pa_test = None
    return L


_saved = []


def isolate_tmp():
    """tempfile.mkdtemp()/gettempdir() into a private directory until
    restore_tmp() (the harnesses don't clean up after themselves)."""
    base = tempfile.gettempdir()
    if len(base) > 50 and os.path.isdir("/tmp"):
        base = "/tmp"     # AF_UNIX paths are limited to 107 bytes
    d = tempfile.mkdtemp(prefix="ftb-regress-", dir=base)
    _saved.append((tempfile.tempdir, d))
    tempfile.tempdir = d
    return d


def restore_tmp():
    old, d = _saved.pop()
    tempfile.tempdir = old
    shutil.rmtree(d, ignore_errors=True)
