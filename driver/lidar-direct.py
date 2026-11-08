#!/usr/bin/env python3
# Open replacement for Creality's cx_ai_middleware: serves the same JSON API
# on /tmp/ai_server_uds (request = JSON + 0x03, reply the same), but drives
# the LiDAR directly over its serial port with lidar_serial.py. lidar.py,
# lidar-bridge.py and the helper service don't notice the difference.
# Started by lidar-start.sh unless variables.cfg has lidar_driver =
# 'middleware'.
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
# dangerous commands (sticker calibration, IR image by YMODEM) answer code
# 103.
#
# Stream mode 1 (--stream-mode 1, or {"control": "set_stream_mode", "mode":
# 1}): the module captures continuously, ~10.7 instead of ~5 frames/s, each
# frame ~1 frame older (LIDAR_TIMING per mode). Applied after every init.
#
# IR camera (--camera-port N): http://printer:N/ shows it, /snapshot returns
# one JPEG (?res=0..3, default 0 = 800x300, 512x288 16:9 on patched
# firmware >= 1.0.17; light=0 without the fill light). Only while the camera switch is on
# (--camera-switch, a Klipper object with a "value" read from Moonraker,
# default "output_pin LiDAR_Camera"; "none" = always on). A picture is only
# taken when no client has used the LiDAR for 10 s; otherwise the last one
# comes back (header X-LiDAR-Camera says which). The fill light stays on
# while the camera is watched, off for LiDAR requests. --webcam NAME: that
# Moonraker webcam (add it first, URL http://printer:N/snapshot) is
# enabled/disabled with the switch.
#
# {"control": "ir_photo", "res": 3, "exposure": N, "light": 1, "laser": 0}
# (the helper's camera calibrations): one JPEG, base64, with the camera
# intrinsics; only type 2 (JPEG) is ever asked from the module.
#
#   lidar-direct.py [--socket PATH] [--port DEV] [--stream-mode 0|1]
#                   [--camera-port N] [--camera-switch OBJECT|none]
#                   [--moonraker URL] [--webcam NAME] [--version]
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import argparse
import array
import base64
import collections
import errno
import json
import os
import select
import socket
import sys
import threading
import time
import urllib.parse
import urllib.request
import zlib

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


def jpeg_size(data):
    """(width, height) from a JPEG's SOF header, None if not found (res 0
    differs between firmware builds)."""
    i = 2
    while i + 9 <= len(data) and data[i] == 0xFF:
        marker = data[i + 1]
        seg = data[i + 2] << 8 | data[i + 3]
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            return (data[i + 7] << 8 | data[i + 8], data[i + 5] << 8 | data[i + 6])
        i += 2 + seg
    return None


class Server:
    def __init__(self, sock_path, port, stream_mode=0, camera_port=None):
        self.sock_path = sock_path
        # IR camera over HTTP (camera_port): a picture is only taken while the
        # LiDAR is idle (no client request for CAMERA_IDLE s, no stream);
        # otherwise the last one is served
        self.camera_port = camera_port
        self.cam_jpeg, self.cam_time, self.last_use = None, 0.0, 0.0
        # fill light: on while somebody watches the camera (no flashing per
        # picture), off for every LiDAR request and after CAMERA_LIGHT_IDLE s
        self.led, self.cam_seen = False, 0.0
        self.cam_switch, self.cam_switch_time = False, -1e9
        # the webcam entry (--webcam) follows the switch (None: not synced yet)
        self.cam_published = None
        try:
            with open(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                   "lidar-camera-off.jpg"), "rb") as f:
                self.off_jpeg = f.read()
        except OSError:
            self.off_jpeg = None
        self.dev = ls.LidarSerial(port=port, timeout=2.0)
        self.next_init = 0.0
        self.flow = False
        self.laser = False
        # module stream mode (lidar_serial.set_stream_mode); RAM only in the
        # module, so it's set again after every init
        self.stream_mode = stream_mode
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
            self.flow = self.laser = self.led = False
            self.apply_stream_mode()
            extra = ""
            try:
                cal = self.dev.request(ls.GET_CAL_PARAM)[:100]
                extra = ", %.1f C, calibration %08x" % (
                    self.dev.get_sensor_temp(), zlib.crc32(cal))
            except ls.LidarError as e:
                extra = ", no temperature/calibration (%s)" % (e,)
            log("module ready: %s, stream mode %d%s" % (
                info, self.stream_mode, extra))
            return True
        except (OSError, ls.LidarError) as e:
            log("init failed: %s" % (e,))
            self.dev.close()
            return False

    def stream_off(self):
        """Stream off, also when it already is (the module answers that with
        a status)."""
        try:
            self.dev.close_flow()
        except ls.LidarError as e:
            if e.code == ls.FAILED:
                raise
        self.flow = False

    def apply_stream_mode(self):
        """The module only takes a new stream mode with its stream off."""
        self.stream_off()
        self.dev.set_stream_mode(self.stream_mode)

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
        if c == "set_stream_mode":
            mode = req.get("mode")
            if mode not in (0, 1):
                return BAD_PARAM
            flow, self.stream_mode = self.flow, mode
            self.call(self.apply_stream_mode)
            if flow:
                self.call(d.open_flow)
                self.flow = True
            log("stream mode %d" % (mode,))
            return {}
        if c == "get_stream_mode":
            return {"mode": self.call(d.get_stream_mode)}
        if c == "get_sensor_temp":
            return {"temp": round(self.call(d.get_sensor_temp), 2)}
        if c == "get_timings":     # firmware >= 1.0.16, else code 103
            t = self.call(d.get_timings)
            return t if t else BAD_PARAM
        if c == "get_module_info":
            cal = self.call(d.get_cal_param)
            return dict(d.info, temp=round(self.call(d.get_sensor_temp), 2),
                        timings=self.call(d.get_timings),
                        stream_mode=self.stream_mode,
                        level=self.call(d.get_point_cloud_level),
                        cal={k: [round(v, 6) for v in vs]
                             for k, vs in cal.items()})
        if c == "self_test":
            # needs the stream off; leaves laser and fill light off
            self.call(self.stream_off)
            self.laser = False
            code = self.call(d.self_test)
            return {"self_test": code, "laser_ok": not code & 1,
                    "light_ok": not code & 2}
        if c == "ambient_points":
            # points found with the laser off = room light read as a line
            flow, laser = self.flow, self.laser
            if not flow:
                self.call(d.open_flow)      # (switches the laser on)
            self.call(d.set_laser, False)
            time.sleep(0.3)
            counts = [len(self.call(d.request, ls.GET_POINT_CLOUD, b"", 2.0))
                      // 6 for _ in range(3)]
            if laser:
                self.call(d.set_laser, True)
            if not flow:
                self.call(self.stream_off)
            self.flow, self.laser = flow, laser
            return {"points": counts}
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
        if c == "ir_photo":
            return self.ir_photo(req)
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

    # -- IR camera (HTTP) ----------------------------------------------------
    CAMERA_IDLE = 10.0       # s without a LiDAR request before the camera may
    CAMERA_MIN_AGE = 0.3     # take a picture; pictures younger than this are reused
    CAMERA_LIGHT_IDLE = 30.0  # s without a picture request: fill light off
    MOONRAKER = "http://127.0.0.1:7125"
    CAMERA_SWITCH = "output_pin LiDAR_Camera"   # None: always on
    WEBCAM_NAME = None       # Moonraker webcam that follows the switch

    def set_moonraker(self, url, switch, webcam=None):
        """Where the camera switch is read and which Moonraker webcam (if
        any) is enabled/disabled with it; switch 'none' = camera always on."""
        self.MOONRAKER = url.rstrip("/")
        self.CAMERA_SWITCH = None if switch in (None, "", "none") else switch
        self.WEBCAM_NAME = webcam or None

    def camera_enabled(self):
        """The camera switch (a Klipper object with a "value", e.g. an
        output_pin), read from Moonraker at most every 3 s; off when it
        can't be read."""
        if self.CAMERA_SWITCH is None:
            return True
        now = time.monotonic()
        if now - self.cam_switch_time > 3.0:
            self.cam_switch_time = now
            url = "%s/printer/objects/query?%s" % (
                self.MOONRAKER, urllib.parse.quote(self.CAMERA_SWITCH))
            try:
                with urllib.request.urlopen(url, timeout=1.0) as r:
                    st = json.loads(r.read().decode())["result"]["status"]
                self.cam_switch = st[self.CAMERA_SWITCH]["value"] > 0
            except Exception:
                self.cam_switch = False
        return self.cam_switch

    def ir_photo(self, req):
        """One JPEG (res 3 = 1600x1200) for the calibrations (helper service): fill light
        and/or laser as asked, at the given exposure. Afterwards the light
        is off and exposure, laser and stream are as before, also when the
        capture fails. Only type 2 (JPEG) is ever sent."""
        res, exp = req.get("res", 3), req.get("exposure", 1000)
        if res not in (0, 1, 2, 3) or not isinstance(exp, int) \
                or not 0 < exp < 1 << 24:
            return BAD_PARAM
        d = self.dev
        self.stream.stop()
        if getattr(self, "intr", None) is None:
            self.intr = [round(v, 3) for v in
                         self.call(d.get_cal_param)["intrinsics"][:4]]
        flow, laser = self.flow, self.laser
        prev = self.call(d.get_exposure)
        t0 = time.monotonic()
        try:
            self.call(d.set_exposure, exp)
            self.light(bool(req.get("light", 1)))
            jpg = self.call(d.capture_jpeg, res, bool(req.get("laser", 0)))
        finally:
            # the capture switched the stream (and the laser) off
            self.flow = self.laser = False
            self.light(False)
            for fn, args in ((d.set_laser, (False,)), (d.set_exposure, (prev,))):
                try:
                    self.call(fn, *args)
                except ls.LidarError as e:
                    log("ir_photo: restoring failed: %s" % (e,))
            try:
                if laser:
                    self.call(d.set_laser, True)
                    self.laser = True
                if flow:
                    self.call(d.open_flow)
                    self.flow = True
            except ls.LidarError as e:
                log("ir_photo: restoring the stream failed: %s" % (e,))
        w, h = jpeg_size(jpg) or {0: (800, 300), 1: (800, 600),
                                  2: (1600, 600), 3: (1600, 1200)}[res]
        return {"jpeg": base64.b64encode(jpg).decode(), "w": w, "h": h,
                "exposure": exp, "intr": self.intr,
                "secs": round(time.monotonic() - t0, 2)}

    def light(self, on):
        if self.led == on:
            return
        try:
            self.dev.set_fill_light(on)
            self.led = on
        except (ls.LidarError, OSError) as e:
            log("fill light %s: %s" % ("on" if on else "off", e))

    def camera_tick(self):
        """Fill light off once nobody watches or the switch went off; the
        Moonraker webcam entry (--webcam) is enabled/disabled with the
        switch."""
        enabled = self.camera_enabled()
        if self.led and (time.monotonic() - self.cam_seen > self.CAMERA_LIGHT_IDLE
                         or not enabled):
            self.light(False)
        if self.WEBCAM_NAME and enabled != self.cam_published:
            self.cam_published = enabled
            body = json.dumps({"name": self.WEBCAM_NAME, "enabled": enabled}).encode()
            try:
                req = urllib.request.Request(
                    self.MOONRAKER + "/server/webcams/item", data=body, headers={
                    "Content-Type": "application/json"})
                urllib.request.urlopen(req, timeout=2.0).read()
                log("camera %s (webcam %s)" % (
                    "on" if enabled else "off", self.WEBCAM_NAME))
            except Exception as e:
                log("camera: updating the webcam failed: %s" % (e,))

    def camera_jpeg(self, res, light):
        """(jpeg or None, 'live' / 'cached' / reason)."""
        now = time.monotonic()
        if not self.camera_enabled():
            self.light(False)
            return self.off_jpeg, "off (camera switch %s)" % (self.CAMERA_SWITCH,)
        self.cam_seen = now
        if self.cam_jpeg and now - self.cam_time < self.CAMERA_MIN_AGE:
            return self.cam_jpeg, "cached"
        if now - self.last_use < self.CAMERA_IDLE or self.stream.running:
            return self.cam_jpeg, "cached (LiDAR busy)"
        if not self.ensure_ready():
            return self.cam_jpeg, "cached (module not ready)"
        d = self.dev
        flow, laser = self.flow, self.laser
        state = "live"
        try:
            self.light(light)
            self.cam_jpeg, self.cam_time = d.capture_jpeg(res), time.monotonic()
        except ls.LidarError as e:
            state = "cached (%s)" % (e,)
            log("camera: %s" % (e,))
        except OSError as e:
            log("camera: port error %s" % (e,))
            self.dev.close()
            return self.cam_jpeg, "cached (port error)"
        # the capture switched the stream (and with it the laser) off
        self.flow = self.laser = False
        try:
            if laser:
                d.set_laser(True)
                self.laser = True
            if flow:
                d.open_flow()
                self.flow = True
        except (ls.LidarError, OSError) as e:
            log("camera: restoring the stream failed: %s" % (e,))
            self.dev.close()
        return self.cam_jpeg, state

    PAGE = (b"<!doctype html><title>LiDAR camera</title><body style='margin:0;"
            b"background:#000'><img id=i style='width:100%'><script>"
            b"function n(){var i=document.getElementById('i');"
            b"i.onload=i.onerror=function(){setTimeout(n,200)};"
            b"i.src='snapshot?'+Date.now()}n()</script>")

    def camera_http(self, conn):
        try:
            conn.settimeout(2.0)
            req = b""
            while b"\r\n\r\n" not in req and len(req) < 4096:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                req += chunk
            line = req.split(b"\r\n", 1)[0].decode(errors="replace").split()
            path, _, query = (line[1] if len(line) > 1 else "/").partition("?")
            args = dict(kv.partition("=")[::2] for kv in query.split("&") if kv)
            if path.rstrip("/") in ("", "/index.html"):
                body, ctype, status, extra = self.PAGE, "text/html", "200 OK", ""
            elif path in ("/snapshot", "/stream"):
                try:
                    res = int(args.get("res", 0))
                except ValueError:
                    res = 0
                jpg, state = self.camera_jpeg(
                    res if res in (0, 1, 2, 3) else 0, args.get("light", "1") != "0")
                extra = "X-LiDAR-Camera: %s\r\n" % (state,)
                if jpg:
                    body, ctype, status = jpg, "image/jpeg", "200 OK"
                else:
                    body, ctype, status = (state.encode(), "text/plain",
                                           "503 Service Unavailable")
            else:
                body, ctype, status, extra = b"not found", "text/plain", "404 Not Found", ""
            conn.sendall(("HTTP/1.0 %s\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
                          "Cache-Control: no-store\r\nAccess-Control-Allow-Origin: *\r\n"
                          "%s\r\n" % (status, ctype, len(body), extra)).encode() + body)
        except OSError:
            pass
        finally:
            conn.close()

    def reply(self, req):
        self.last_use = time.monotonic()
        if self.led and self.dev.ready:
            self.light(False)          # never measure with the fill light on
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
        cam = []
        if self.camera_port:
            web = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            web.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            web.bind(("", self.camera_port))
            web.listen(4)
            cam = [web]
            log("IR camera on http port %d" % (self.camera_port,))
        self.ensure_ready()
        while True:
            r, _, _ = select.select([srv] + cam + list(clients), [], [], 5.0)
            if not r and not self.dev.ready:
                self.ensure_ready()   # keep retrying while idle
            if cam:
                self.camera_tick()
            for s in r:
                if s is srv:
                    c, _ = srv.accept()
                    clients[c] = b""
                    continue
                if cam and s is cam[0]:
                    c, _ = s.accept()
                    self.camera_http(c)
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
    ap.add_argument("--stream-mode", type=int, choices=(0, 1), default=0,
                    help="module stream mode: 1 = continuous capture, "
                         "~10.7 instead of ~5 frames/s (default: 0)")
    ap.add_argument("--camera-port", type=int, default=None,
                    help="serve the IR camera over HTTP on this port "
                         "(default: off)")
    ap.add_argument("--camera-switch", default="output_pin LiDAR_Camera",
                    help="Klipper object whose value switches the camera "
                         "on, read from Moonraker; 'none' = always on "
                         "(default: %(default)s)")
    ap.add_argument("--moonraker", default="http://127.0.0.1:7125",
                    help="Moonraker URL for the camera switch and webcam "
                         "(default: %(default)s)")
    ap.add_argument("--webcam", default=None,
                    help="name of a Moonraker webcam that is enabled and "
                         "disabled with the camera switch (default: none)")
    ap.add_argument("--version", action="version",
                    version="FreeTheBeam lidar-direct " + ls.__version__)
    return ap.parse_args(argv)


def main():
    args = parse_args(sys.argv[1:])
    if not os.path.exists(args.port):
        # the module may enumerate after we start (boot order): keep retrying
        print("lidar-direct: %s not found, waiting for the LiDAR to show up"
              % args.port, file=sys.stderr, flush=True)
    srv = Server(args.socket, args.port, args.stream_mode, args.camera_port)
    srv.set_moonraker(args.moonraker, args.camera_switch, args.webcam)
    srv.serve()


if __name__ == "__main__":
    main()
