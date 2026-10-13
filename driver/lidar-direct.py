#!/usr/bin/env python3
# Open replacement for Creality's cx_ai_middleware: serves the same JSON API
# on /tmp/ai_server_uds (request = JSON + 0x03, reply the same), but drives
# the LiDAR directly over its serial port with lidar_serial.py. lidar.py,
# lidar-bridge.py and the helper service don't notice the difference.
# Started by lidar-start.sh when variables.cfg has lidar_driver = 'direct'.
#
# Streaming: {"control": "get_point_cloud", "prefetch": 1} starts a thread
# that keeps asking the module for frames back-to-back (its frame clock never
# slips, whatever the client's reaction time) and queues them; each reply
# carries "t", the CLOCK_MONOTONIC_RAW time the frame arrived from the module
# (Klipper's reactor clock), so dwell scans can time frames exactly. Any
# other request, or no client asking for 2s, stops the stream first.
#
# Only the commands lidar.py and the helper use (plus a few harmless ones);
# Creality's algorithms (flow / first floor / offset detection) and the
# dangerous commands (sticker calibration, IR image) answer code 103.
#
#   lidar-direct.py [--socket PATH] [--port DEV]
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import array
import collections
import errno
import json
import os
import select
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lidar_serial as ls  # noqa: E402

ETX = b"\x03"
LOG = "/tmp/lidar-direct.log"
BAD_PARAM = 103


def log(msg):
    try:
        if os.path.exists(LOG) and os.path.getsize(LOG) > 200000:
            os.rename(LOG, LOG + ".old")
        with open(LOG, "a") as f:
            f.write("%s %s\n" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg))
    except OSError:
        pass


def clock():
    """Klipper's reactor clock (chelper get_monotonic)."""
    return time.clock_gettime(time.CLOCK_MONOTONIC_RAW)


def cloud_json(data, t=None):
    """Reply for get_point_cloud, formatted directly from the frame bytes:
    3.5x faster than json.dumps on the K1 (7 ms per 1500 points)."""
    n = len(data) // 6
    a = array.array("h")
    a.frombytes(data[:n * 6])
    if sys.byteorder != "little":
        a.byteswap()
    pc = ("[%d,%d,%d]," * n % tuple(a))[:-1] if n else ""
    ts = ' "t": %.6f,' % (t,) if t is not None else ""
    return ('{"control": "get_point_cloud", "result": {"code": 0,' + ts
            + ' "point_cloud": [' + pc + ']}}')


class Stream:
    """Frames requested back-to-back in a thread, queued with their time."""

    def __init__(self, server):
        self.server = server
        self.cond = threading.Condition()
        self.frames = collections.deque(maxlen=4)
        self.error = None
        self.running = False
        self.thread = None
        self.last_take = 0.0

    def start(self):
        if self.running:
            return
        self.frames.clear()
        self.error = None
        self.running = True
        self.last_take = time.monotonic()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        dev = self.server.dev
        while self.running:
            if time.monotonic() - self.last_take > 2.0:
                break   # nobody is taking frames any more
            try:
                data = dev.request(ls.GET_POINT_CLOUD, b"", 2.0)
                t = clock()
            except (OSError, ls.LidarError) as e:
                with self.cond:
                    self.error = e
                    self.cond.notify_all()
                break
            with self.cond:
                self.frames.append((t, data))
                self.cond.notify_all()
        with self.cond:
            self.running = False
            self.cond.notify_all()

    def take(self, timeout=2.0):
        end = time.monotonic() + timeout
        with self.cond:
            self.last_take = time.monotonic()
            while not self.frames:
                if self.error is not None:
                    e, self.error = self.error, None
                    raise e
                left = end - time.monotonic()
                if left <= 0 or not self.running:
                    raise ls.LidarError("no frame from the stream")
                self.cond.wait(left)
            self.last_take = time.monotonic()
            return self.frames.popleft()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(3.0)
            self.thread = None


class Server:
    def __init__(self, sock_path, port):
        self.sock_path = sock_path
        self.dev = ls.LidarSerial(port=port, timeout=2.0)
        self.next_init = 0.0
        self.flow = False
        self.laser = False
        self.stream = Stream(self)

    # -- module ------------------------------------------------------------
    def ensure_ready(self):
        if self.dev.ready:
            return True
        now = time.monotonic()
        if now < self.next_init:
            return False
        self.next_init = now + 3.0
        try:
            self.dev.close()
            self.dev.open()
            info = self.dev.init()
            self.flow = self.laser = False
            log("module ready: %s" % (info,))
            return True
        except (OSError, ls.LidarError) as e:
            log("init failed: %s" % (e,))
            self.dev.close()
            return False

    def call(self, fn, *args):
        """Run a driver call; after a port error or timeout re-initialise the
        module (restoring the stream if it was on) and try once more."""
        if not self.ensure_ready():
            raise ls.LidarError("module not ready", ls.NOT_READY)
        try:
            return fn(*args)
        except OSError as e:
            err = "port error: %s" % (e,)
        except ls.LidarError as e:
            if e.code != ls.FAILED:
                raise           # the module answered with a status
            err = str(e)
        log("%s, re-initialising" % (err,))
        self.dev.close()
        self.next_init = 0.0
        flow, laser = self.flow, self.laser
        if not self.ensure_ready():
            raise ls.LidarError("module not ready", ls.NOT_READY)
        try:
            if laser:
                self.dev.set_laser(True)
            if flow:
                self.dev.open_flow()
                self.flow = True
            return fn(*args)
        except OSError as e:
            self.dev.close()
            raise ls.LidarError("port error: %s" % (e,))

    # -- requests ----------------------------------------------------------
    def handle(self, req):
        c = req.get("control")
        d = self.dev
        simple = {
            "open_flow": d.open_flow, "close_flow": d.close_flow,
            "set_laser_open": lambda: d.set_laser(True),
            "set_laser_close": lambda: d.set_laser(False),
            "set_fill_light_open": lambda: d.set_fill_light(True),
            "set_fill_light_close": lambda: d.set_fill_light(False),
        }
        if c == "get_point_cloud" and req.get("prefetch"):
            if not self.stream.running:
                self.stream.stop()
                if not self.ensure_ready():
                    raise ls.LidarError("module not ready", ls.NOT_READY)
                self.stream.start()
            try:
                t, data = self.stream.take()
            except OSError as e:
                self.stream.stop()
                self.dev.close()
                raise ls.LidarError("port error: %s" % (e,))
            except ls.LidarError as e:
                self.stream.stop()
                if e.code == ls.FAILED:
                    self.dev.close()   # re-initialise on the next request
                raise
            return cloud_json(data, t)
        self.stream.stop()   # everything else talks to the module itself
        if c == "get_point_cloud":
            data = self.call(d.request, ls.GET_POINT_CLOUD, b"", 2.0)
            return cloud_json(data, clock())
        if c in simple:
            self.call(simple[c])
            if c in ("open_flow", "close_flow"):
                self.flow = c == "open_flow"
            elif c in ("set_laser_open", "set_laser_close"):
                self.laser = c == "set_laser_open"
            return {}
        if c == "set_point_cloud_level":
            lv = req.get("level")
            if lv not in (0, 1, 2):
                return BAD_PARAM
            self.call(d.set_point_cloud_level, lv)
            return {}
        if c == "get_point_cloud_level":
            return {"level": self.call(d.get_point_cloud_level)}
        if c in ("get_laser_exposure", "get_laser_gain"):
            k = c[10:]
            fn = d.get_exposure if k == "exposure" else d.get_gain
            return {k: self.call(fn)}
        if c in ("set_laser_exposure", "set_laser_gain"):
            k = c[10:]
            v = req.get(k)
            if not isinstance(v, int) or not 0 < v < 1 << 24:
                return BAD_PARAM
            self.call(d.set_exposure if k == "exposure" else d.set_gain, v)
            return {}
        if c == "set_laser_roi_area":
            try:
                self.call(d.set_roi, int(req["x_min"]), int(req["x_max"]))
            except (KeyError, ValueError, TypeError):
                return BAD_PARAM
            return {}
        if c == "module_restart":
            d.close()
            self.flow = self.laser = False
            self.next_init = 0.0
            return {} if self.ensure_ready() else ls.NOT_READY
        log("unsupported request: %s" % (c,))
        return BAD_PARAM

    def reply(self, req):
        c = req.get("control") if isinstance(req, dict) else None
        try:
            r = self.handle(req) if c else BAD_PARAM
        except ls.LidarError as e:
            log("%s: %s" % (c, e))
            r = e.code
        except Exception as e:   # never let one request kill the server
            log("%s: unexpected %r" % (c, e))
            r = ls.FAILED
        if isinstance(r, str):
            return r
        res = {"code": r} if isinstance(r, int) else dict(r, code=0)
        return json.dumps({"control": c, "result": res})

    # -- socket ------------------------------------------------------------
    def serve(self):
        # bind under a temporary name and rename: the path only appears
        # once the socket accepts connections
        tmp = self.sock_path + ".new"
        try:
            os.unlink(tmp)
        except OSError as e:
            if e.errno != errno.ENOENT:
                raise
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(tmp)
        srv.listen(8)
        os.rename(tmp, self.sock_path)
        clients = {}   # socket -> receive buffer
        log("listening on %s" % (self.sock_path,))
        self.ensure_ready()
        while True:
            r, _, _ = select.select([srv] + list(clients), [], [], 5.0)
            if not r and not self.dev.ready:
                self.ensure_ready()   # keep retrying while idle
            for s in r:
                if s is srv:
                    c, _ = srv.accept()
                    clients[c] = b""
                    continue
                try:
                    data = s.recv(65536)
                except OSError:
                    data = b""
                if not data:
                    clients.pop(s, None)
                    s.close()
                    continue
                buf = clients[s] + data
                while ETX in buf:
                    msg, buf = buf.split(ETX, 1)
                    try:
                        req = json.loads(msg.decode(errors="replace"))
                    except ValueError:
                        req = None
                    out = self.reply(req).encode() + ETX
                    try:
                        s.setblocking(True)
                        s.sendall(out)
                    except OSError:
                        buf = b""
                        break
                if s in clients:
                    clients[s] = buf[-65536:]


def parse_args(argv):
    ap = argparse.ArgumentParser(
        description="Serve the Creality LiDAR JSON API on a unix socket, "
                    "driving the module directly over its serial port.")
    ap.add_argument("--socket", default="/tmp/ai_server_uds",
                    help="unix socket to serve (default: %(default)s)")
    ap.add_argument("--port", default=ls.PORT,
                    help="LiDAR serial device (default: %(default)s)")
    return ap.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])
    if not os.path.exists(args.port):
        # the module may enumerate after we start (boot order): keep retrying
        print("lidar-direct: %s not found, waiting for the LiDAR to show up"
              % args.port, file=sys.stderr, flush=True)
    Server(args.socket, args.port).serve()


if __name__ == "__main__":
    main()
