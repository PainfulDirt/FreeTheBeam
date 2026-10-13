# K1 Max LiDAR (Creality "AI LiDAR") for Kalico
#
# The LiDAR is a line-laser height sensor on the toolhead: one capture is a
# ~29mm wide height profile (~1500 points, in um) across X. Creality's
# cx_ai_middleware (started by lidar-start.sh) drives it and answers JSON
# requests on a unix socket; this plugin moves the toolhead and collects the
# profiles, so scans can run inside a print.
#
# The K1 has ~200MB of RAM and a slow CPU, and a heavy job starves Klipper
# (MCU "Rescheduled timer in the past"). So everything is reduced while
# scanning: each stop becomes one averaged profile of 580 bins (0.05mm), and
# the analysis is plain Python done a few rows at a time.
#
#   [lidar]
#   x_offset: -36       # laser line centre relative to the nozzle,
#   y_offset: -21.2     #   with the nozzle offset_z above the surface
#   #x_per_z: 0         # how far the spot moves per mm of nozzle height
#   #y_per_z: 0         #   (the laser hits at an angle)
#   #offset_z: 3
#   x_dir: -1           # profile x runs towards printer -X
#   LIDAR_OFFSET_CALIBRATE saves its result as lidar_x_offset, lidar_y_offset,
#   lidar_x_per_z, lidar_y_per_z (variables.cfg); they override these
#   #service: HELPER_IP:7131   # optional: lidar_service.py on a helper
#                              # computer does the captures and the analysis
#
# Commands:
#   LIDAR_STATUS
#   LIDAR_SCAN NAME= X= Y0= Y1= [STEP=0.4] [Z=3] [MODE=dwell|stop]
#       [CAPTURES=3] [LEVEL=scan_level]
#       X/Y are where the laser looks (not the nozzle); one profile per Y.
#       dwell (default with a service): rows planned just ahead and locked
#       to the LiDAR's frames, one frame per row taken standing still (one
#       frame period per row); stop: stop at every row and average CAPTURES
#       frames (slower, ~1.3s per row)
#   LIDAR_TIMING [Z=3] [DZ=1.5] [SPEED=5] [CYCLES=15] [LEVELS=0,1] [APPLY=1]
#       measures the frame period and the time window a frame covers before
#       it arrives, for the current driver (slow Z moves over a flat plate
#       at the current X/Y), and saves them for dwell scans (lidar_timing).
#       Run again after LIDAR_DRIVER
#   LIDAR_PA_ANALYZE BASE= SCAN= [APPLY=1]
#       compares the scan of the printed PA test with the bare-plate scan
#   LIDAR_PA_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [PA_START=0] [PA_STEP=0.01]
#       [LINES=9] [X=130] [Y=100]
#       writes the PA test as a print and starts it: START_PRINT (with the
#       bare-plate scan after the nozzle touch, before the purge), the test
#       lines, cool down, scan, analysis, END_PRINT
#   LIDAR_OFFSET_CALIBRATE EXTRUDER_TEMP= BED_TEMP= [X=150] [Y=150]
#       [Z1=3] [Z2=5] [APPLY=1]
#       prints a small cross (nozzle at X/Y = its centre), scans it at Z1
#       and Z2 and works out where the laser looks: X/Y offset and how far
#       it moves per mm of height. Run directly, nothing in the job list
#   LIDAR_PA_CREALITY EXTRUDER_TEMP= BED_TEMP= [X=170] [Y=60]
#       [PA_START=0.02] [PA_STEP=0.004]
#       Creality's own PA test pattern (frame, flow boxes, four zig-zags,
#       20 PA values), run directly: no file in gcodes, nothing in the job
#       list. X/Y = front-left corner of the 27 x 139mm frame. Not scanned
#       or analysed yet: read it by eye (sharpest corners)
#
# Copy or symlink into klippy/plugins (see docs/install-k1max.md).
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import array
import json
import math
import os
import socket

ETX = b"\x03"
NAN = float("nan")

# profile bins: 0.05mm from -14.5 to +14.5mm of the laser line
BIN = 0.05
BIN_MIN = -14.5
NBINS = 580


def bin_center(b):
    return BIN_MIN + (b + 0.5) * BIN


def reduce_caps(caps):
    """Average the captures of one stop into NBINS bins (mm, NaN = empty)."""
    s = [0.0] * NBINS
    n = [0] * NBINS
    for c in caps:
        for p in c:
            b = int((p[0] * 0.001 - BIN_MIN) / BIN)
            if 0 <= b < NBINS:
                s[b] += p[2]
                n[b] += 1
    return array.array("f", [s[i] * 0.001 / n[i] if n[i] else NAN
                             for i in range(NBINS)])


def median(v):
    v = sorted(x for x in v if x == x)
    if not v:
        return NAN
    m = len(v) // 2
    return v[m] if len(v) % 2 else 0.5 * (v[m - 1] + v[m])


def linfit(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0, (ys[0] if ys else 0.0)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx == 0:
        return 0.0, my
    k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return k, my - k * mx


def line_areas(base, scan, x, x_dir, line_x, half=1.2):
    """Plastic cross-section (mm2) at each line X in one row.

    base/scan: binned distances of the same row without and with the print.
    """
    X = [x + x_dir * bin_center(b) for b in range(NBINS)]
    h = [b - s for b, s in zip(base, scan)]   # plastic is closer
    # drift between the two scans (tilt + offset) from the bare parts
    fx, fy = [], []
    for b in range(NBINS):
        if h[b] == h[b] and abs(h[b]) < 0.08 and all(
                abs(X[b] - lx) > 0.9 for lx in line_x):
            fx.append(X[b])
            fy.append(h[b])
    if len(fx) > 50:
        k, c = linfit(fx, fy)
        h = [v - (k * xx + c) for v, xx in zip(h, X)]
    out = []
    for lx in line_x:
        a, cnt = 0.0, 0
        for b in range(NBINS):
            if abs(X[b] - lx) < half:
                cnt += 1
                if h[b] == h[b]:
                    a += min(max(h[b], -0.05), 0.8) * BIN
        out.append(abs(a) if cnt else NAN)
    return out


W_ACC, W_DEC, EDGE = 8.0, 3.0, 2.0


def pa_fit(ys, A, test):
    """PA from the per-row line areas A[row][line] of the PA test.

    With PA too low a line thins after speeding up and bulges after slowing
    down; too high, the other way round. Per line
      score = extra area after decelerations - missing area after
              accelerations (in mm of line, against its steady sections)
    and a straight line through score vs PA crosses zero at the right PA.
    """
    n = len(test["pa"])
    step = median([abs(b - a) for a, b in zip(ys, ys[1:])])
    bounds, y = [], test["y0"]
    for seglen, v in test["segments"]:
        bounds.append((y, y + seglen, v))
        y += seglen
    nominal = test["height"] * test["width"]

    def med(i, lo, hi):
        lo, hi = min(lo, hi), max(lo, hi)
        v = [A[r][i] for r in range(len(ys)) if lo <= ys[r] <= hi]
        return median(v) if len(v) >= 3 else NAN

    def excess(i, at, d, w, ref):
        lo, hi = sorted((at, at + d * w))
        return sum(A[r][i] - ref for r in range(len(ys))
                   if lo <= ys[r] <= hi and A[r][i] == A[r][i]) * step / ref

    report, scores, steady = [], [], []
    for i in range(n):
        d = test["dirs"][i]
        acc, dec = [], []
        for j in range(len(bounds) - 1):
            a0, a1, va = bounds[j]
            b0, b1, vb = bounds[j + 1]
            # +Y: from segment j into j+1 at a1; -Y: from j+1 into j
            cur, nxt = (va, vb) if d > 0 else (vb, va)
            w = W_ACC if nxt > cur else W_DEC
            if d > 0:
                ref = med(i, b0 + w, b1 - EDGE)
            else:
                ref = med(i, a0 + EDGE, a1 - w)
            if not ref > 0:
                continue
            (acc if nxt > cur else dec).append(excess(i, a1, d, w, ref))
        fast = median([med(i, b[0] + W_ACC, b[1] - W_ACC)
                       for b in bounds if b[2] == test["fast"]])
        steady.append(fast)
        ma = sum(acc) / len(acc) if acc else NAN
        md = sum(dec) / len(dec) if dec else NAN
        scores.append(md - ma)
        report.append(
            "  PA %.3f: line %.3f mm2 (%.0f%%), after accel %+.2f mm,"
            " after decel %+.2f mm, score %+.2f" % (
                test["pa"][i], fast, 100 * fast / nominal, ma, md, md - ma))
    res = {"ok": False, "pa_tested": test["pa"],
           "scores": [round(s, 4) if s == s else None for s in scores],
           "line_area": [round(s, 4) if s == s else None for s in steady]}
    good = [i for i in range(n)
            if scores[i] == scores[i] and steady[i] > 0.3 * nominal]
    if len(good) < 4:
        res["reason"] = "only %d of %d lines were found" % (len(good), n)
    else:
        px = [test["pa"][i] for i in good]
        sy = [scores[i] for i in good]
        k, c = linfit(px, sy)
        my = sum(sy) / len(sy)
        ss = sum((v - my) ** 2 for v in sy)
        r2 = (1 - sum((v - (k * p + c)) ** 2 for p, v in zip(px, sy)) / ss
              if ss > 0 else 0.0)
        res.update({"slope": round(k, 3), "r2": round(r2, 3)})
        if k >= 0:
            res["reason"] = "score doesn't fall with PA (r2 %.2f)" % (r2,)
        else:
            best = -c / k
            res["pa_fit"] = round(best, 4)
            lo, hi = min(test["pa"]), max(test["pa"])
            if best < lo - 0.5 * (hi - lo) / (n - 1) or best > hi:
                res["reason"] = "PA %.3f is outside the tested range" % (best,)
            elif r2 < 0.5:
                res["reason"] = "measurement too noisy (r2 %.2f)" % (r2,)
            else:
                res["ok"] = True
                res["pa"] = round(max(best, 0.0), 4)
    report.append("LiDAR PA fit: %s" % (
        "PA %.4f (r2 %.2f)" % (res["pa"], res["r2"]) if res["ok"]
        else res.get("reason")))
    res["report"] = report
    return res


def plane_fit(pts):
    """Least squares d = a + b*x + c*y through (x, y, d) points."""
    n = sx = sy = sd = sxx = sxy = syy = sxd = syd = 0.0
    for x, y, d in pts:
        n += 1
        sx += x
        sy += y
        sd += d
        sxx += x * x
        sxy += x * y
        syy += y * y
        sxd += x * d
        syd += y * d
    m = [[n, sx, sy], [sx, sxx, sxy], [sy, sxy, syy]]
    r = [sd, sxd, syd]

    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    dm = det(m)
    if abs(dm) < 1e-12:
        return None
    out = []
    for c in range(3):
        mc = [row[:] for row in m]
        for i in range(3):
            mc[i][c] = r[i]
        out.append(det(mc) / dm)
    return out


def cross_fit(scan, test, tick=None):
    """Find the printed cross in a scan.

    test: {"cx", "cy", "arm", "height"}, the cross as commanded (nozzle
    coordinates). Returns where the scan sees the arm along Y (its X) and
    the arm along X (its Y), in the scan's own coordinates, and the line
    height: {"ok", "x", "y", "h", ...}.
    """
    ys, rows = scan["ys"], scan["rows"]
    X = [scan["x"] + scan["x_dir"] * bin_center(b) for b in range(NBINS)]
    res = {"ok": False}
    # bare plate: plane fit, leaving out what is closer (plastic)
    pts = [(X[b], y, row[b]) for y, row in zip(ys, rows)
           for b in range(0, NBINS, 4) if row[b] == row[b]]
    if len(pts) < 1000:
        res["reason"] = "scan is empty (%d points)" % (len(pts),)
        return res
    coef = None
    for it in range(4):
        use = pts if coef is None else [
            p for p in pts
            if coef[0] + coef[1] * p[0] + coef[2] * p[1] - p[2] < 0.08]
        coef = plane_fit(use)
        if coef is None:
            res["reason"] = "no plate found"
            return res
        if tick:
            tick()
    thr = 0.5 * test["height"]
    # plastic weight per point: its height where it stands out, else 0
    P = []
    for y, row in zip(ys, rows):
        base = coef[0] + coef[2] * y
        P.append([max(0.0, base + coef[1] * X[b] - row[b])
                  if row[b] == row[b] and
                  base + coef[1] * X[b] - row[b] > thr else 0.0
                  for b in range(NBINS)])
        if tick:
            tick()

    def peak(w, k):
        sm = [sum(w[max(0, i - k):i + k + 1]) for i in range(len(w))]
        return max(range(len(sm)), key=lambda i: sm[i])

    # first guess: the arm along Y is the column with the most plastic,
    # the arm along X the row with the most
    xp = X[peak([sum(r[b] for r in P) for b in range(NBINS)], 5)]
    yp = ys[peak([sum(r) for r in P], 2)]
    win, gap = 1.5, 3.0
    # arm along Y: plastic centroid in X, row by row (away from the other arm)
    xs, rows_y = [], []
    for r, y in enumerate(ys):
        if abs(y - yp) < gap:
            continue
        w = [(P[r][b], X[b]) for b in range(NBINS)
             if abs(X[b] - xp) < win and P[r][b] > 0]
        if len(w) >= 3:
            xs.append(sum(a * x for a, x in w) / sum(a for a, x in w))
            rows_y.append(y)
    # arm along X: plastic centroid in Y, column by column
    yc, cols_x = [], []
    xm = median(xs) if xs else xp
    for b in range(NBINS):
        if abs(X[b] - xm) < gap:
            continue
        w = [(P[r][b], ys[r]) for r in range(len(ys))
             if abs(ys[r] - yp) < win and P[r][b] > 0]
        if len(w) >= 2:
            yc.append(sum(a * y for a, y in w) / sum(a for a, y in w))
            cols_x.append(X[b])
    tops = [max(P[r][b] for b in range(NBINS) if abs(X[b] - xm) < win)
            for r, y in enumerate(ys) if y in rows_y]
    res.update({"rows": len(xs), "cols": len(yc),
                "plane": [round(c, 5) for c in coef]})
    if len(xs) < 20 or len(yc) < 50:
        res["reason"] = ("cross not found (arm along Y in %d rows, arm along"
                         " X in %d columns)" % (len(xs), len(yc)))
        return res
    res.update({"ok": True, "x": round(xm, 4), "y": round(median(yc), 4),
                "h": round(median(tops), 4),
                "x_spread": round(median([abs(v - xm) for v in xs]), 4),
                "y_len": round(max(rows_y) - min(rows_y), 2),
                "x_len": round(max(cols_x) - min(cols_x), 2)})
    return res


def offset_solve(fits, test, ref_z):
    """Laser offset from cross fits at two nozzle heights.

    fits: [(z, x_offset used, y_offset used, cross_fit result)]. Where the
    scan sees the arm along Y (X) and along X (Y), against where it was
    printed, is the error of the offset used. The laser sees the top of the
    lines, (z - line height) below the nozzle, so the offsets belong to that
    height; a line through both gives the offset at ref_z and per mm.
    """
    report, pts = [], []
    for z, xo, yo, f in fits:
        if not f.get("ok"):
            return {"ok": False, "reason": "Z%.1f: %s" % (z, f.get("reason")),
                    "report": ["LiDAR offset: Z%.1f: %s" % (
                        z, f.get("reason"))], "fits": [x[3] for x in fits]}
        xt = xo + test["cx"] - f["x"]
        yt = yo + test["cy"] - f["y"]
        d = z - f["h"]
        pts.append((d, xt, yt))
        report.append(
            "  Z%.1f: cross seen at X%.2f Y%.2f (printed X%.2f Y%.2f), lines"
            " %.2fmm high, arms %.1f / %.1fmm -> offset X%.3f Y%.3f" % (
                z, f["x"], f["y"], test["cx"], test["cy"], f["h"],
                f["x_len"], f["y_len"], xt, yt))
    (d1, x1, y1), (d2, x2, y2) = pts
    kx, ky = (x2 - x1) / (d2 - d1), (y2 - y1) / (d2 - d1)
    res = {"ok": True, "fits": [x[3] for x in fits],
           "x_offset": round(x1 + kx * (ref_z - d1), 3),
           "y_offset": round(y1 + ky * (ref_z - d1), 3),
           "x_per_z": round(kx, 4), "y_per_z": round(ky, 4)}
    report.append(
        "LiDAR offset at Z%.1f: X%.3f Y%.3f, per mm of height X%+.3f"
        " Y%+.3f" % (ref_z, res["x_offset"], res["y_offset"], kx, ky))
    if abs(kx) > 1.0 or abs(ky) > 3.0:
        res.update(ok=False, reason="implausible change with height"
                   " (X%+.2f Y%+.2f per mm)" % (kx, ky))
    res["report"] = report
    return res


class Lidar:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.sock_path = config.get("socket", "/tmp/ai_server_uds")
        self.x_offset = config.getfloat("x_offset", -36.0)
        self.y_offset = config.getfloat("y_offset", -21.2)
        self.x_per_z = config.getfloat("x_per_z", 0.0)
        self.y_per_z = config.getfloat("y_per_z", 0.0)
        self.offset_z = config.getfloat("offset_z", 3.0)
        self.offset_source = "config"
        self.printer.register_event_handler("klippy:connect",
                                            self._load_offsets)
        self.x_dir = config.getfloat("x_dir", -1.0)
        self.scan_z = config.getfloat("scan_z", 3.0)
        self.captures = config.getint("captures", 3, minval=1)
        self.settle = config.getfloat("settle_time", 0.15, minval=0.0)
        self.speed = config.getfloat("speed", 50.0, above=0.0)
        # dwell scans (needs the service): frames requested back-to-back
        # come every frame_period. A frame is not a snapshot: it covers
        # about 0.1s, ending ~0.15s before it arrives (direct driver, level
        # 0, measured 2026-10-12). frame_latency +- frame_spread is that
        # window. Defaults: cx_ai_middleware, level 0 (2026-10-07: a 0.33mm
        # lag at 1.7mm/s); LIDAR_TIMING measures it per driver and level and
        # saves it (lidar_timing), which wins
        self.frame_period = config.getfloat("frame_period", 0.235,
                                            above=0.05)
        self.frame_latency = config.getfloat("frame_latency", 0.194,
                                             minval=0.0)
        self.frame_spread = config.getfloat("frame_spread", 0.05,
                                            minval=0.0)
        # standing still: settle time after a move, and extra time before
        # and after a frame's window (prediction error)
        self.dwell_settle = config.getfloat("dwell_settle", 0.05, minval=0.0)
        self.dwell_margin = config.getfloat("dwell_margin", 0.015, minval=0.0)
        # point cloud level for dwell scans: 0 all points (~0.02mm apart),
        # 1 every 2nd (~0.04mm, frames ~15% faster)
        self.scan_level = config.getint("scan_level", 0, minval=0, maxval=2)
        self.printer.load_object(config, "motion_report")
        self.mode = config.getchoice("mode", {"dwell": "dwell",
                                              "stop": "stop"}, "dwell")
        # K1 Max (Simple AF) defaults; change them on other machines
        self.data_dir = config.get(
            "data_dir", "/usr/data/printer_data/lidar")
        self.gcode_dir = config.get(
            "gcode_dir", "/usr/data/printer_data/gcodes")
        self.sock = None
        # optional helper computer running lidar_service.py ("host:port"):
        # the LiDAR captures and the analysis happen there, the K1 only
        # moves the toolhead and forwards bytes (lidar-bridge.py)
        self.service = config.get("service", None)
        self.svc_sock = None
        self.svc_buf = b""
        # name -> {"x", "ys", "rows"}; only the last few scans are kept
        self.scans = {}
        self.pa_test = None
        self.last_result = {}
        for cmd in ("LIDAR_STATUS", "LIDAR_SCAN", "LIDAR_PA_ANALYZE",
                    "LIDAR_PA_CALIBRATE", "LIDAR_PA_CREALITY",
                    "LIDAR_OFFSET_CALIBRATE", "LIDAR_TIMING"):
            self.gcode.register_command(
                cmd, getattr(self, "cmd_" + cmd),
                desc=getattr(self, "desc_" + cmd))

    def get_status(self, eventtime):
        return {"last_result": self.last_result}

    OFFSET_VARS = (("x_offset", "lidar_x_offset"),
                   ("y_offset", "lidar_y_offset"),
                   ("x_per_z", "lidar_x_per_z"),
                   ("y_per_z", "lidar_y_per_z"))

    def _load_offsets(self):
        # the last LIDAR_OFFSET_CALIBRATE result wins over [lidar]
        sv = self.printer.lookup_object("save_variables", None)
        saved = getattr(sv, "allVariables", None) or {}
        if all(v in saved for a, v in self.OFFSET_VARS):
            for attr, var in self.OFFSET_VARS:
                setattr(self, attr, float(saved[var]))
            self.offset_source = "saved by LIDAR_OFFSET_CALIBRATE"

    def _xo(self, z):
        """Laser line centre relative to the nozzle at nozzle height z."""
        return self.x_offset + self.x_per_z * (z - self.offset_z)

    def _yo(self, z):
        return self.y_offset + self.y_per_z * (z - self.offset_z)

    def _yield(self):
        self.reactor.pause(self.reactor.monotonic() + 0.001)

    # ---------------------------------------------------- middleware socket
    def _close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def _call(self, msg, timeout=10.0):
        raw, t = self._call_raw(msg, timeout)
        try:
            return json.loads(raw.decode(errors="replace"))
        except ValueError as e:
            raise self.printer.command_error("LiDAR: bad reply (%s)" % (e,))

    def _send(self, msg):
        """Send a request without waiting for its reply (OSError)."""
        # one persistent connection: cx_ai_middleware leaks a descriptor per
        # connection and stops accepting after a few thousand
        if self.sock is None:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect(self.sock_path)
            s.setblocking(False)
            self.sock = s
        self.sock.sendall(json.dumps(msg).encode() + ETX)

    def _recv(self, timeout=10.0):
        """The reply bytes (without ETX) and the reactor time they arrived."""
        buf = b""
        end = self.reactor.monotonic() + timeout
        while not buf.endswith(ETX):
            try:
                d = self.sock.recv(1 << 20)
                if not d:
                    raise OSError("connection closed")
                buf += d
                continue
            except (BlockingIOError, InterruptedError):
                pass
            now = self.reactor.monotonic()
            if now > end:
                raise OSError("timeout")
            # wait without blocking the reactor
            self.reactor.pause(now + 0.002)
        return buf[:-1], self.reactor.monotonic()

    def _driver_error(self, err):
        self._close()
        return self.printer.command_error(
            "LiDAR: no answer from the driver (%s, %s). Is it running?"
            " (k1max/lidar-start.sh)" % (self._driver(), err))

    def _call_raw(self, msg, timeout=10.0):
        """Reply bytes (without ETX) and the reactor time they arrived."""
        err = None
        for attempt in range(2):
            try:
                self._send(msg)
                return self._recv(timeout)
            except OSError as e:
                err = e
                self._close()
        raise self._driver_error(err)

    def _frame_msg(self):
        """get_point_cloud for timed frames: with the direct driver the
        frames are streamed back-to-back by the driver and time-stamped."""
        if self._driver() == "direct":
            return {"control": "get_point_cloud", "prefetch": 1}
        return {"control": "get_point_cloud"}

    def _frame_time(self, raw, t):
        """When a frame arrived from the LiDAR: the direct driver's own time
        stamp (the reactor's clock), else when its reply arrived here."""
        i = raw.find(b'"t": ', 0, 120)
        if i >= 0:
            try:
                return float(raw[i + 5:raw.index(b",", i)])
            except ValueError:
                pass
        return t

    def _driver(self):
        """Which driver serves the socket (variables.cfg lidar_driver)."""
        sv = self.printer.lookup_object("save_variables", None)
        saved = getattr(sv, "allVariables", None) or {}
        return str(saved.get("lidar_driver", "middleware"))

    def _timing(self, level):
        """(frame period, window start, window end, source) for the driver
        and level: a frame covers arrival - end .. arrival - start."""
        sv = self.printer.lookup_object("save_variables", None)
        saved = getattr(sv, "allVariables", None) or {}
        t = (saved.get("lidar_timing") or {}).get(self._driver(), {})
        t = t.get(str(level))
        if t and len(t) >= 4:
            return float(t[0]), float(t[2]), float(t[3]), "LIDAR_TIMING"
        return (self.frame_period, self.frame_latency - self.frame_spread,
                self.frame_latency + self.frame_spread, "config")

    def _laser(self, on):
        if on:
            self._call({"control": "set_laser_open"})
            self._call({"control": "open_flow"})
            self._call({"control": "set_point_cloud_level", "level": 0})
        else:
            for c in ("close_flow", "set_laser_close"):
                try:
                    self._call({"control": c})
                except self.printer.command_error:
                    pass

    # ------------------------------------------------------- helper service
    def _svc_close(self):
        if self.svc_sock is not None:
            try:
                self.svc_sock.close()
            except OSError:
                pass
        self.svc_sock = None
        self.svc_buf = b""

    def _svc_send(self, data, timeout=10.0):
        # non-blocking send of a request that gets no reply (frames)
        end = self.reactor.monotonic() + timeout
        view = memoryview(data)
        try:
            while len(view):
                try:
                    n = self.svc_sock.send(view)
                    view = view[n:]
                except (BlockingIOError, InterruptedError):
                    now = self.reactor.monotonic()
                    if now > end:
                        raise OSError("timeout")
                    self.reactor.pause(now + 0.002)
        except OSError as e:
            self._svc_close()
            raise self.printer.command_error(
                "LiDAR: lost the lidar service (%s)" % (e,))

    def _svc(self, req, timeout=30.0):
        """One request to lidar_service.py, without blocking the reactor."""
        host, port = self.service.rsplit(":", 1)
        try:
            if self.svc_sock is None:
                s = socket.create_connection((host, int(port)), 3.0)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.setblocking(False)
                self.svc_sock = s
            self.svc_sock.sendall(json.dumps(req).encode() + b"\n")
            end = self.reactor.monotonic() + timeout
            while b"\n" not in self.svc_buf:
                try:
                    d = self.svc_sock.recv(65536)
                    if not d:
                        raise OSError("connection closed")
                    self.svc_buf += d
                    continue
                except (BlockingIOError, InterruptedError):
                    pass
                now = self.reactor.monotonic()
                if now > end:
                    raise OSError("timeout")
                self.reactor.pause(now + 0.005)
            line, self.svc_buf = self.svc_buf.split(b"\n", 1)
            rep = json.loads(line.decode(errors="replace"))
        except (OSError, ValueError) as e:
            self._svc_close()
            raise self.printer.command_error(
                "LiDAR: no answer from the lidar service at %s (%s)" % (
                    self.service, e))
        if not rep.get("ok"):
            raise self.printer.command_error(
                "LiDAR service: %s" % (rep.get("error"),))
        return rep

    # ------------------------------------------------------------- scanning
    desc_LIDAR_STATUS = "Check that the LiDAR answers"

    def cmd_LIDAR_STATUS(self, gcmd):
        msg = {"control": "get_laser_exposure"}
        if self.service:
            r = self._svc({"cmd": "middleware", "msg": msg})["reply"]
        else:
            r = self._call(msg)
        gcmd.respond_info(
            "LiDAR ok: exposure %s, offset X%.2f Y%.2f at Z%.1f, per mm Z"
            " X%+.3f Y%+.3f (%s), x_dir %+d, %s, scans: %s" % (
                r.get("result", {}).get("exposure"), self.x_offset,
                self.y_offset, self.offset_z, self.x_per_z, self.y_per_z,
                self.offset_source, self.x_dir,
                "work done by the service at " + self.service
                if self.service else "work done on this printer",
                ", ".join(sorted(self.scans)) or "none"))

    def _scan(self, name, x, y0, y1, step, z, captures):
        toolhead = self.printer.lookup_object("toolhead")
        n = int(round(abs(y1 - y0) / step))
        d = step if y1 >= y0 else -step
        ys = [round(y0 + i * d, 4) for i in range(n + 1)]
        xo, yo = self._xo(z), self._yo(z)
        nozzle_x = x - xo
        rows = []
        start = self.reactor.monotonic()
        self.gcode.run_script_from_command(
            "SAVE_GCODE_STATE NAME=_lidar_scan\nG90\nG0 Z%.3f F600" % (z,))
        if self.service:
            self._svc({"cmd": "begin", "name": name, "x": x, "z": z,
                       "x_dir": self.x_dir, "x_offset": xo,
                       "y_offset": yo, "captures": captures})
        else:
            self._laser(True)
        try:
            for y in ys:
                self.gcode.run_script_from_command(
                    "G0 X%.3f Y%.3f F%.0f" % (
                        nozzle_x, y - yo, self.speed * 60.0))
                toolhead.wait_moves()
                self.reactor.pause(self.reactor.monotonic() + self.settle)
                if self.service:
                    # the service captures and keeps the data
                    self._svc({"cmd": "capture", "name": name, "y": y})
                    continue
                caps = []
                for i in range(captures):
                    r = self._call({"control": "get_point_cloud"})
                    caps.append(r.get("result", {}).get("point_cloud", []))
                rows.append(reduce_caps(caps))
                del caps
        finally:
            if self.service:
                try:
                    self._svc({"cmd": "end", "name": name})
                except self.printer.command_error:
                    pass
            else:
                self._laser(False)
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=_lidar_scan")
        scan = {"x": x, "z": z, "x_dir": self.x_dir, "ys": ys, "rows": rows}
        # keep memory bounded: base + scan of one test, plus one spare
        while len(self.scans) >= 3:
            self.scans.pop(next(iter(self.scans)))
        self.scans[name] = scan
        if not self.service:
            self._save(name, scan)
        return len(ys), self.reactor.monotonic() - start

    def _scan_dwell(self, name, x, y0, y1, step, z, level=None, debug=None):
        """Stop at every row, with the rows locked to the LiDAR's frames.

        Frames are requested back-to-back: the next request goes out the
        moment a reply arrives, so they keep coming every `period`, and each
        one covers the time from `hi` to `lo` before it arrives
        (LIDAR_TIMING). From the last arrival the next arrivals are
        predicted, and the rows are planned just ahead of the toolhead so it
        stands still at row i for the whole window of one of them (+-
        dwell_margin), moving to the next row in between: one frame per row
        when move + settle + window fit into a frame period (level 0 on the
        direct driver: ~0.2s per row). A frame is only used if its whole
        window is inside its row's still time, so a wrong prediction costs a
        row (filled in from the neighbours), never a wrong measurement.
        """
        toolhead = self.printer.lookup_object("toolhead")
        mcu = self.printer.lookup_object("mcu")
        level = self.scan_level if level is None else level
        period, lo, hi, src = self._timing(level)
        margin, settle = self.dwell_margin, self.dwell_settle
        n = int(round(abs(y1 - y0) / step))
        d = step if y1 >= y0 else -step
        ys = [round(y0 + i * d, 4) for i in range(n + 1)]
        xo, yo = self._xo(z), self._yo(z)
        nozzle_x = x - xo
        start = self.reactor.monotonic()
        self.gcode.run_script_from_command(
            "SAVE_GCODE_STATE NAME=_lidar_scan\nG90\nG0 Z%.3f F600\n"
            "G0 X%.3f Y%.3f F%.0f" % (z, nozzle_x, ys[0] - yo,
                                      self.speed * 60.0))
        toolhead.wait_moves()
        self._svc({"cmd": "begin", "name": name, "x": x, "z": z,
                   "x_dir": self.x_dir, "x_offset": xo,
                   "y_offset": yo, "captures": 1, "level": level,
                   "grid": [y0, y1, step]})
        got = [0] * len(ys)
        windows = []     # per row: (still from, still until), print time
        # predicted next arrival, last used arrival, last move duration
        clock = {"next": None, "last": -1e9, "move": 0.02}
        head = '{"cmd": "frame", "name": %s, "row": %%d, "raw": ' % (
            json.dumps(name),)
        in_flight = False

        def plan_ahead(ahead=1.0):
            # planning costs the K1 noticeable time, so plan as we go
            now = mcu.estimated_print_time(self.reactor.monotonic())
            while len(windows) < len(ys) and (
                    not windows or windows[-1][1] - now < ahead):
                i = len(windows)
                if i:
                    self.gcode.run_script_from_command(
                        "G0 Y%.3f F%.0f" % (ys[i] - yo, self.speed * 60.0))
                t_arrive = toolhead.get_last_move_time()
                if i:
                    clock["move"] = t_arrive - windows[-1][1]
                still = t_arrive + settle
                # the first predicted frame after the last row's whose whole
                # window we can stand still for
                a = clock["next"]
                while a - hi - margin < still or a < clock["last"] + period / 2:
                    a += period
                clock["last"] = a
                # stay as long as the next row can still catch the next
                # frame: frames that come late (the module's clock slips by
                # a sensor tick now and then) are still inside the window
                tail = max(margin, period - (hi - lo) - margin - settle
                           - clock["move"] - 0.005)
                self.gcode.run_script_from_command(
                    "G4 P%.0f" % ((a - lo + tail - t_arrive) * 1000.0,))
                windows.append((still, toolhead.get_last_move_time()))

        try:
            frame_msg = self._frame_msg()
            try:
                self._send(frame_msg)
                in_flight = True
                i = 0
                while True:
                    raw, t = self._recv(5.0)
                    in_flight = False
                    arrival = mcu.estimated_print_time(
                        self._frame_time(raw, t))
                    done = (len(windows) == len(ys) and
                            arrival - hi > windows[-1][1])
                    if not done:
                        # keep the frame clock running: ask for the next
                        # frame before anything else
                        self._send(frame_msg)
                        in_flight = True
                    w0, w1 = arrival - hi, arrival - lo   # what it covers
                    clock["next"] = arrival + period
                    plan_ahead()
                    if done:
                        break
                    while i < len(windows) - 1 and w1 > windows[i][1]:
                        i += 1
                    ok = (windows[i][0] <= w0 and w1 <= windows[i][1]
                          and not got[i])
                    if debug is not None:
                        debug.append((round(arrival, 4), round(w0, 4),
                                      round(w1, 4), i, windows[i], ok,
                                      got[i]))
                    if not ok:
                        continue   # partly taken while moving, or row done
                    got[i] = 1
                    # forwarded as is: the K1 doesn't decode the point cloud
                    self._svc_send((head % (i,)).encode()
                                   + raw.replace(b"\n", b" ") + b"}\n")
            except OSError as e:
                in_flight = False
                raise self._driver_error(e)
            toolhead.wait_moves()
        finally:
            if in_flight:
                # don't leave a reply in the socket for the next request
                try:
                    self._recv(5.0)
                except OSError:
                    self._close()
            try:
                self._svc({"cmd": "end", "name": name}, timeout=60.0)
            except self.printer.command_error:
                pass
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=_lidar_scan")
        while len(self.scans) >= 3:
            self.scans.pop(next(iter(self.scans)))
        self.scans[name] = {"x": x, "z": z, "x_dir": self.x_dir, "ys": ys,
                            "rows": []}
        return len(ys), self.reactor.monotonic() - start, len(ys) - sum(got)

    def _save(self, name, scan):
        # compact copy for looking at on a PC: distances in um, -1 = empty
        try:
            if not os.path.isdir(self.data_dir):
                os.makedirs(self.data_dir)
            with open(os.path.join(self.data_dir, name + ".json"), "w") as f:
                f.write(json.dumps({k: scan[k] for k in ("x", "z", "x_dir",
                                                         "ys")})[:-1])
                f.write(', "bin": %s, "bin_min": %s, "rows": [' % (
                    BIN, BIN_MIN))
                for i, row in enumerate(scan["rows"]):
                    f.write(("," if i else "") + "[" + ",".join(
                        str(int(v * 1000)) if v == v else "-1"
                        for v in row) + "]")
                    if i % 20 == 19:
                        self._yield()
                f.write("]}\n")
        except OSError:
            pass

    desc_LIDAR_SCAN = "Scan a strip with the LiDAR (X/Y where the laser looks)"

    def cmd_LIDAR_SCAN(self, gcmd):
        name = gcmd.get("NAME")
        args = (name, gcmd.get_float("X"), gcmd.get_float("Y0"),
                gcmd.get_float("Y1"), gcmd.get_float("STEP", 0.4, above=0.05),
                gcmd.get_float("Z", self.scan_z))
        mode = gcmd.get("MODE", self.mode).lower()
        if mode == "dwell" and self.service:
            dbg = [] if gcmd.get_int("DEBUG", 0) else None
            rows, secs, missed = self._scan_dwell(*args, level=gcmd.get_int(
                "LEVEL", self.scan_level, minval=0, maxval=2), debug=dbg)
            if dbg is not None:
                with open("/tmp/lidar_scan_debug.txt", "w") as f:
                    for row in dbg:
                        f.write("%r\n" % (row,))
            gcmd.respond_info("LiDAR scan %s: %d rows in %.0fs (%d filled in"
                              " from neighbours)" % (name, rows, secs, missed))
            return
        rows, secs = self._scan(
            *args, gcmd.get_int("CAPTURES", self.captures, minval=1))
        gcmd.respond_info("LiDAR scan %s: %d rows in %.0fs" % (
            name, rows, secs))

    # --------------------------------------------------------------- timing
    def _median_dist(self, raw):
        """Median distance (um) of a get_point_cloud reply, None if empty."""
        try:
            pc = json.loads(raw.decode(errors="replace"))["result"][
                "point_cloud"]
        except (ValueError, KeyError, TypeError):
            return None
        return median([p[2] for p in pc]) if pc else None

    def _last_time_at(self, trapq, z, before, span=0.7, dt=0.002):
        """Latest print time before `before` at which the toolhead was at
        height z (linear interpolation), None if not within span."""
        t = before
        z1 = trapq.get_trapq_position(t)[0]
        if z1 is None:
            return None
        while t > before - span:
            p, v = trapq.get_trapq_position(t - dt)
            if p is None:
                return None
            if (p[2] - z) * (z1[2] - z) <= 0 and p[2] != z1[2]:
                return t - dt + dt * (z - p[2]) / (z1[2] - p[2])
            z1 = p
            t -= dt
        return None

    def _timing_run(self, toolhead, mcu, trapq, z0, dz, speed, cycles,
                    debug=None):
        rest = [self._median_dist(
            self._call_raw({"control": "get_point_cloud"})[0])
            for i in range(4)]
        rest = [r for r in rest if r is not None]
        if len(rest) < 3:
            raise self.printer.command_error(
                "LIDAR_TIMING: no points at Z%.2f: is a flat plate under"
                " the laser?" % (z0,))
        d0 = median(rest)
        f = speed * 60.0
        # up, pause, down, pause, `cycles` times, in 0.05mm steps fed while
        # the frames come in (Klipper blocks a command that queues more than
        # ~2s of moves). The pauses are longer than any latency, so the last
        # time the nozzle was at a height stays unambiguous. Fast ramps: the
        # laser hits at ~45 deg, so a Z move also slides the line over the
        # plate's texture, which reads as height noise; at 5mm/s 25um of
        # texture is 5ms of timing error (50ms at 0.5mm/s)
        n = max(1, int(round(dz / 0.05)))
        up = ["G1 Z%.4f F%.0f" % (z0 + dz * k / n, f) for k in range(1, n + 1)]
        down = ["G1 Z%.4f F%.0f" % (z0 + dz * k / n, f)
                for k in range(n - 1, -1, -1)]
        todo = (up + ["G4 P600"] + down + ["G4 P600"]) * cycles

        end = []   # print time the last move ends, once it is queued

        def feed(ahead=0.8):
            now = mcu.estimated_print_time(self.reactor.monotonic())
            while todo and toolhead.get_last_move_time() - now < ahead:
                self.gcode.run_script_from_command(todo.pop(0))
                if not todo:
                    end.append(toolhead.get_last_move_time())

        frames = []
        frame_msg = self._frame_msg()
        try:
            self._send(frame_msg)
            while True:
                raw, t = self._recv(5.0)
                pt = mcu.estimated_print_time(self._frame_time(raw, t))
                # (not get_last_move_time() here: on an idle toolhead it
                # keeps moving ahead, and this would never end)
                last = bool(end) and pt > end[0] + 0.3
                if not last:
                    self._send(frame_msg)
                frames.append((pt, raw))
                if last:
                    break
                feed()
        except OSError as e:
            raise self._driver_error(e)
        toolhead.wait_moves()
        periods = [b[0] - a[0] for a, b in zip(frames, frames[1:])]
        lats = []
        for pt, raw in frames:
            dist = self._median_dist(raw)
            self._yield()
            c = z = None
            if dist is not None:
                z = z0 + (dist - d0) / 1000.0
                if z0 + 0.15 * dz < z < z0 + 0.85 * dz:
                    c = self._last_time_at(trapq, z, pt)
                    if c is not None:
                        lats.append(pt - c)
            if debug is not None:
                debug.append((pt, dist, z, c,
                              trapq.get_trapq_position(pt)[0]))
        if len(lats) < 4:
            raise self.printer.command_error(
                "LIDAR_TIMING: only %d frames on the ramps" % (len(lats),))
        return median(periods), lats

    desc_LIDAR_TIMING = ("Measure the LiDAR frame period and latency of the"
                         " current driver (slow Z moves over a flat plate)")

    def cmd_LIDAR_TIMING(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        mcu = self.printer.lookup_object("mcu")
        trapq = self.printer.lookup_object("motion_report").trapqs[
            "toolhead"]
        z0 = gcmd.get_float("Z", self.scan_z)
        dz = gcmd.get_float("DZ", 1.5, above=0.3, maxval=2.0)
        speed = gcmd.get_float("SPEED", 5.0, above=0.2, maxval=10.0)
        cycles = gcmd.get_int("CYCLES", 15, minval=1, maxval=50)
        levels = [int(v) for v in gcmd.get("LEVELS", "0,1").split(",")]
        if any(v not in (0, 1, 2) for v in levels):
            raise gcmd.error("LIDAR_TIMING: LEVELS are 0, 1 or 2")
        driver = self._driver()
        self.gcode.run_script_from_command(
            "SAVE_GCODE_STATE NAME=_lidar_timing\nG90\nG0 Z%.3f F600"
            % (z0,))
        toolhead.wait_moves()
        res = {}
        try:
            self._laser(True)
            for level in levels:
                self._call({"control": "set_point_cloud_level",
                            "level": level})
                dbg = [] if gcmd.get_int("DEBUG", 0) else None
                p, lats = self._timing_run(toolhead, mcu, trapq, z0,
                                           dz, speed, cycles, dbg)
                if dbg is not None:
                    with open("/tmp/lidar_timing_%d.txt" % (level,), "w") as f:
                        for row in dbg:
                            f.write("%r\n" % (row,))
                # the frame's window: earliest .. latest measured capture
                # time before the arrival (a frame is not a snapshot)
                res[str(level)] = [round(p, 4), round(median(lats), 4),
                                   round(min(lats), 4), round(max(lats), 4)]
                gcmd.respond_info(
                    "LiDAR timing (%s, level %d): frame every %.1f ms,"
                    " covering %.0f..%.0f ms before it arrives (median %.0f,"
                    " %d frames)" % (driver, level, p * 1000.0,
                                     max(lats) * 1000.0, min(lats) * 1000.0,
                                     median(lats) * 1000.0, len(lats)))
        finally:
            try:
                self._call({"control": "set_point_cloud_level", "level": 0})
            except self.printer.command_error:
                pass
            self._laser(False)
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE NAME=_lidar_timing")
        if gcmd.get_int("APPLY", 1):
            sv = self.printer.lookup_object("save_variables", None)
            saved = getattr(sv, "allVariables", None) or {}
            timing = dict(saved.get("lidar_timing") or {})
            timing[driver] = dict(timing.get(driver, {}), **res)
            # no spaces: one G-code parameter
            val = repr(timing).replace(" ", "")
            self.gcode.run_script_from_command(
                'SAVE_VARIABLE VARIABLE=lidar_timing VALUE="%s"' % (val,))
            gcmd.respond_info("LiDAR timing saved for %s: %s" % (
                driver, res))

    # ------------------------------------------------------------- analysis
    desc_LIDAR_PA_ANALYZE = "Find pressure advance from a LiDAR PA test scan"

    def cmd_LIDAR_PA_ANALYZE(self, gcmd):
        test = self.pa_test
        if test is None:
            tf = os.path.join(self.data_dir, "pa_test.json")
            if not os.path.exists(tf):
                raise gcmd.error("LiDAR: no PA test layout found")
            with open(tf) as f:
                test = json.load(f)
        if self.service:
            res = self._svc({"cmd": "pa_analyze", "base": gcmd.get("BASE"),
                             "scan": gcmd.get("SCAN"), "test": test},
                            timeout=120.0)["result"]
        else:
            res = self._pa_local(gcmd, test)
        self.last_result = res
        for line in res["report"]:
            gcmd.respond_info(line)
        if not res["ok"]:
            raise gcmd.error("LiDAR PA: %s" % (res.get("reason"),))
        pa = res["pa"]
        if gcmd.get_int("APPLY", 1):
            # kept for the loaded filament: restored at boot, cleared by
            # your UNLOAD_FILAMENT macro
            self.gcode.run_script_from_command(
                "SET_PRESSURE_ADVANCE ADVANCE=%.4f\n"
                "SAVE_VARIABLE VARIABLE=lidar_pa VALUE=%.4f" % (pa, pa))
        gcmd.respond_info(
            "LiDAR pressure advance: %.4f (kept until UNLOAD_FILAMENT)"
            % (pa,))

    def _pa_local(self, gcmd, test):
        base = self.scans.get(gcmd.get("BASE"))
        scan = self.scans.get(gcmd.get("SCAN"))
        if base is None or scan is None:
            raise gcmd.error("LiDAR: scan %s/%s not in memory" % (
                gcmd.get("BASE"), gcmd.get("SCAN")))
        if base["ys"] != scan["ys"] or base["x"] != scan["x"]:
            raise gcmd.error("LiDAR: base and scan don't cover the same rows")
        line_x = [test["x0"] + i * test["pitch"] for i in range(
            len(test["pa"]))]
        A = []
        for r in range(len(scan["ys"])):
            A.append(line_areas(base["rows"][r], scan["rows"][r], scan["x"],
                                scan["x_dir"], line_x))
            self._yield()   # one row at a time: never block the reactor long
        res = pa_fit(scan["ys"], A, test)
        try:
            with open(os.path.join(self.data_dir, "pa_result.json"), "w") as f:
                json.dump(dict(res, areas=A, ys=scan["ys"]), f)
        except OSError:
            pass
        return res

    # ------------------------------------------------------- PA test print
    desc_LIDAR_PA_CALIBRATE = "Print and measure a pressure advance test"

    def cmd_LIDAR_PA_CALIBRATE(self, gcmd):
        temp = gcmd.get_float("EXTRUDER_TEMP", above=150.0)
        bed = gcmd.get_float("BED_TEMP", minval=0.0)
        chamber = gcmd.get_float("CHAMBER_TEMP", 0.0, minval=0.0)
        pa0 = gcmd.get_float("PA_START", 0.0, minval=0.0)
        pas = gcmd.get_float("PA_STEP", 0.01, above=0.0)
        lines = gcmd.get_int("LINES", 9, minval=4, maxval=9)
        x0 = gcmd.get_float("X", 130.0)
        y0 = gcmd.get_float("Y", 100.0)
        fast = gcmd.get_float("FAST", 150.0, above=20.0)
        slow = gcmd.get_float("SLOW", 20.0, above=1.0)
        height = gcmd.get_float("HEIGHT", 0.2, above=0.05)
        width = gcmd.get_float("WIDTH", 0.45, above=0.1)
        accel = gcmd.get_float("ACCEL", 5000.0, above=100.0)
        fan = gcmd.get_int("FAN", 64, minval=0, maxval=255)
        pitch = 3.0
        # slow / fast segments along Y; slow ones are long enough to settle
        segs = [(15.0, slow), (20.0, fast), (15.0, slow), (20.0, fast),
                (15.0, slow), (20.0, fast), (15.0, slow)]
        y1 = y0 + sum(s[0] for s in segs)
        x1 = x0 + (lines - 1) * pitch
        cx = (x0 + x1) / 2.0
        toolhead = self.printer.lookup_object("toolhead")
        fil_area = getattr(toolhead.get_extruder(), "filament_area", 2.405)
        epm = height * width / fil_area
        test = {"x0": x0, "pitch": pitch, "y0": y0, "y1": y1,
                "pa": [round(pa0 + i * pas, 4) for i in range(lines)],
                "dirs": [1 if i % 2 == 0 else -1 for i in range(lines)],
                "segments": segs, "fast": fast, "slow": slow,
                "height": height, "width": width}
        if not os.path.isdir(self.data_dir):
            os.makedirs(self.data_dir)
        with open(os.path.join(self.data_dir, "pa_test.json"), "w") as f:
            json.dump(test, f)
        self.pa_test = test
        scan = "X=%.2f Y0=%.2f Y1=%.2f" % (cx, y0 - 3, y1 + 3)

        g = ["; LiDAR pressure advance test, written by lidar.py",
             "; PA %s" % (" ".join("%.3f" % p for p in test["pa"]),),
             "SET_PRINT_STATS_INFO TOTAL_LAYER=1",
             "EXCLUDE_OBJECT_DEFINE NAME=lidar_pa_test CENTER=%.1f,%.1f"
             " POLYGON=[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]]" % (
                 cx, (y0 + y1) / 2, x0 - 2, y0 - 2, x1 + 2, y0 - 2,
                 x1 + 2, y1 + 2, x0 - 2, y1 + 2),
             # the bare plate is scanned inside START_PRINT, after the nozzle
             # touch and before heating up and purging (_LIDAR_PRINT_SCAN)
             "SET_GCODE_VARIABLE MACRO=_LIDAR_PRINT_SCAN VARIABLE=x VALUE=%.2f"
             % (cx,),
             "SET_GCODE_VARIABLE MACRO=_LIDAR_PRINT_SCAN VARIABLE=y0 VALUE=%.2f"
             % (y0 - 3,),
             "SET_GCODE_VARIABLE MACRO=_LIDAR_PRINT_SCAN VARIABLE=y1 VALUE=%.2f"
             % (y1 + 3,),
             "START_PRINT EXTRUDER_TEMP=%.0f BED_TEMP=%.0f CHAMBER_TEMP=%.0f"
             % (temp, bed, chamber),
             "SET_PRINT_STATS_INFO CURRENT_LAYER=1",
             "M83", "G90",
             "SET_VELOCITY_LIMIT ACCEL=%.0f" % (accel,),
             "M106 S%d" % (fan,),
             "EXCLUDE_OBJECT_START NAME=lidar_pa_test",
             "G0 Z2 F600",
             "G0 X%.3f Y%.3f F9000" % (x0, y0),
             "G0 Z%.3f F600" % (height,)]
        # serpentine: each line starts where the last one ended, so there
        # are no travels to string across the lines
        for i, pa in enumerate(test["pa"]):
            x = x0 + i * pitch
            d = test["dirs"][i]
            g.append("SET_PRESSURE_ADVANCE ADVANCE=%.4f" % (pa,))
            y = y0 if d > 0 else y1
            for seglen, v in (segs if d > 0 else list(reversed(segs))):
                y += d * seglen
                g.append("G1 X%.3f Y%.3f E%.5f F%.0f" % (
                    x, y, seglen * epm, v * 60.0))
            if i + 1 < lines:
                g.append("G1 X%.3f E%.5f F%.0f" % (
                    x + pitch, pitch * epm, slow * 60.0))
        g += ["G1 E-0.8 F2400",
              "G0 Z3 F600",
              "EXCLUDE_OBJECT_END NAME=lidar_pa_test",
              "M106 S0",
              "SET_VELOCITY_LIMIT ACCEL=%.0f" % (toolhead.max_accel,),
              # cool down over the purge line, in front of the test, so
              # nothing drips onto the lines while they are scanned
              "G0 X%.3f Y%.3f F9000" % (cx, y0 - 12),
              "M104 S150",
              "M106 S255",
              "TEMPERATURE_WAIT SENSOR=extruder MAXIMUM=155",
              "M106 S0",
              "LIDAR_SCAN NAME=pa_scan " + scan,
              "LIDAR_PA_ANALYZE BASE=pa_base SCAN=pa_scan",
              "END_PRINT"]
        fn = "lidar_pa_test.gcode"
        with open(os.path.join(self.gcode_dir, fn), "w") as f:
            f.write("\n".join(g) + "\n")
        gcmd.respond_info(
            "LiDAR PA test: %d lines, PA %.3f-%.3f, X%.0f-%.0f Y%.0f-%.0f;"
            " starting %s" % (lines, test["pa"][0], test["pa"][-1], x0 - 2,
                              x1 + 2, y0 - 2, y1 + 2, fn))
        self.gcode.run_script_from_command(
            "SDCARD_PRINT_FILE FILENAME=%s" % (fn,))

    # ------------------------------------------- Creality's PA test pattern
    desc_LIDAR_PA_CREALITY = "Print Creality's pressure advance test pattern"

    def cmd_LIDAR_PA_CREALITY(self, gcmd):
        eventtime = self.reactor.monotonic()
        state = self.printer.lookup_object("print_stats").get_status(
            eventtime)["state"]
        if state in ("printing", "paused"):
            raise gcmd.error("LiDAR: not while a print is running")
        temp = gcmd.get_float("EXTRUDER_TEMP", above=150.0)
        bed = gcmd.get_float("BED_TEMP", minval=0.0)
        chamber = gcmd.get_float("CHAMBER_TEMP", 0.0, minval=0.0)
        ox = gcmd.get_float("X", 170.0, minval=0.0, maxval=266.0)
        oy = gcmd.get_float("Y", 60.0, minval=0.0, maxval=140.0)
        pa0 = gcmd.get_float("PA_START", 0.02, minval=0.0)
        pas = gcmd.get_float("PA_STEP", 0.004, above=0.0)
        g, grid = creality_pa_gcode(ox, oy, pa0, pas)
        toolhead = self.printer.lookup_object("toolhead")
        gcmd.respond_info(
            "Creality PA test at X%.0f-%.0f Y%.0f-%.0f, PA %.3f-%.3f" % (
                ox, ox + 27, oy, oy + 139, grid[0][0], grid[-1][-1]))
        # the object makes START_PRINT's adaptive mesh cover the frame
        self.gcode.run_script_from_command(
            "EXCLUDE_OBJECT_DEFINE RESET=1\n"
            "EXCLUDE_OBJECT_DEFINE NAME=creality_pa_test CENTER=%.1f,%.1f"
            " POLYGON=[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]]\n"
            "START_PRINT EXTRUDER_TEMP=%.0f BED_TEMP=%.0f CHAMBER_TEMP=%.0f" % (
                ox + 13.5, oy + 69.5, ox, oy, ox + 27, oy, ox + 27, oy + 139,
                ox, oy + 139, temp, bed, chamber))
        pa_was = toolhead.get_extruder().get_status(
            self.reactor.monotonic()).get("pressure_advance", 0.0)
        try:
            self.gcode.run_script_from_command("\n".join(
                ["M83", "G90", "SET_VELOCITY_LIMIT ACCEL=5000", "M106 S0"]
                + g + ["SET_PRESSURE_ADVANCE ADVANCE=%.4f" % (pa_was,)]))
        except Exception:
            self.gcode.run_script_from_command(
                "M221 S100\nSET_PRESSURE_ADVANCE ADVANCE=%.4f\n"
                "TURN_OFF_HEATERS\nEXCLUDE_OBJECT_DEFINE RESET=1" % (pa_was,))
            raise
        self.gcode.run_script_from_command(
            "END_PRINT\nEXCLUDE_OBJECT_DEFINE RESET=1")
        gcmd.respond_info(
            "Creality PA test done. PA per V (rows front to back, zig-zags"
            " left to right):\n" + "\n".join(
                "  " + "  ".join("%.3f" % v for v in row) for row in grid))

    # --------------------------------------------- LiDAR offset self-check
    desc_LIDAR_OFFSET_CALIBRATE = ("Print a small cross and measure where"
                                   " the LiDAR looks")

    def _scan_any(self, name, x, y0, y1, step, z):
        if self.mode == "dwell" and self.service:
            return self._scan_dwell(name, x, y0, y1, step, z)[0]
        return self._scan(name, x, y0, y1, step, z, self.captures)[0]

    def _cross_fit(self, name, test):
        if self.service:
            return self._svc({"cmd": "offset_analyze", "scan": name,
                              "test": test}, timeout=120.0)["result"]
        scan = self.scans.get(name)
        if scan is None:
            raise self.printer.command_error(
                "LiDAR: scan %s not in memory" % (name,))
        return cross_fit(scan, test, self._yield)

    def cmd_LIDAR_OFFSET_CALIBRATE(self, gcmd):
        state = self.printer.lookup_object("print_stats").get_status(
            self.reactor.monotonic())["state"]
        if state in ("printing", "paused"):
            raise gcmd.error("LiDAR: not while a print is running")
        temp = gcmd.get_float("EXTRUDER_TEMP", above=150.0)
        bed = gcmd.get_float("BED_TEMP", minval=0.0)
        chamber = gcmd.get_float("CHAMBER_TEMP", 0.0, minval=0.0)
        cx = gcmd.get_float("X", 150.0)
        cy = gcmd.get_float("Y", 150.0)
        z1 = gcmd.get_float("Z1", 3.0, minval=2.0, maxval=5.0)
        z2 = gcmd.get_float("Z2", 5.0, minval=2.0, maxval=6.0)
        apply = gcmd.get_int("APPLY", 1)
        if z2 - z1 < 1.0:
            raise gcmd.error("LiDAR: Z2 must be at least 1mm above Z1")
        arm, lh, width, half = 10.0, 0.2, 0.6, 15.0
        test = {"cx": cx, "cy": cy, "arm": arm, "height": 2 * lh}
        toolhead = self.printer.lookup_object("toolhead")
        st = toolhead.get_status(self.reactor.monotonic())
        lo, hi = st["axis_minimum"], st["axis_maximum"]
        for z in (z1, z2):
            nx = cx - self._xo(z)
            if not (lo[0] <= nx <= hi[0] and lo[1] <= cy - half - self._yo(z)
                    and cy + half - self._yo(z) <= hi[1]):
                raise gcmd.error("LiDAR: the scans of a cross at X%.0f Y%.0f"
                                 " would leave the bed" % (cx, cy))
        fil_area = getattr(toolhead.get_extruder(), "filament_area", 2.405)
        epm = lh * width / fil_area
        g = ["M83", "G90", "M106 S0"]
        for layer in (1, 2):
            z = layer * lh
            if layer == 2:
                g.append("M106 S128")
            # arm along X, then the arm along Y in two halves (no crossing
            # over the first arm with the nozzle down)
            for (xa, ya), (xb, yb) in (
                    ((cx - arm, cy), (cx + arm, cy)),
                    ((cx, cy - arm), (cx, cy - 2.0)),
                    ((cx, cy + 2.0), (cx, cy + arm))):
                ln = math.hypot(xb - xa, yb - ya)
                g += ["G0 Z%.2f F600" % (z + 1.0,),
                      "G0 X%.3f Y%.3f F9000" % (xa, ya),
                      "G0 Z%.2f F600" % (z,), "G1 E0.8 F2400",
                      "G1 X%.3f Y%.3f E%.5f F1200" % (xb, yb, ln * epm),
                      "G1 E-0.8 F2400"]
        g += ["G0 Z%.2f F600" % (z2,), "M106 S0",
              # cool down away from the scanned area, so nothing drips on it
              "G0 X%.3f Y%.3f F9000" % (cx, cy - half - 12.0),
              "M104 S150", "M106 S255",
              "TEMPERATURE_WAIT SENSOR=extruder MAXIMUM=155", "M106 S0"]
        gcmd.respond_info(
            "LiDAR offset check: cross at X%.0f Y%.0f, scans at Z%.1f and"
            " Z%.1f" % (cx, cy, z1, z2))
        # the object keeps the purge line out of the scanned area and makes
        # the adaptive mesh cover it
        self.gcode.run_script_from_command(
            "EXCLUDE_OBJECT_DEFINE RESET=1\n"
            "EXCLUDE_OBJECT_DEFINE NAME=lidar_offset_cross CENTER=%.1f,%.1f"
            " POLYGON=[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]]\n"
            "START_PRINT EXTRUDER_TEMP=%.0f BED_TEMP=%.0f CHAMBER_TEMP=%.0f" % (
                cx, cy, cx - half, cy - half - 2, cx + half, cy - half - 2,
                cx + half, cy + half + 2, cx - half, cy + half + 2,
                temp, bed, chamber))
        fits = []
        try:
            self.gcode.run_script_from_command("\n".join(g))
            for i, z in enumerate((z1, z2)):
                name = "offset_z%d" % (i + 1,)
                xo, yo = self._xo(z), self._yo(z)
                self._scan_any(name, cx, cy - half, cy + half, 0.2, z)
                fits.append((z, xo, yo, self._cross_fit(name, test)))
        except Exception:
            self.gcode.run_script_from_command(
                "TURN_OFF_HEATERS\nEXCLUDE_OBJECT_DEFINE RESET=1")
            raise
        self.gcode.run_script_from_command(
            "END_PRINT\nEXCLUDE_OBJECT_DEFINE RESET=1")
        res = offset_solve(fits, test, self.offset_z)
        res["old"] = [round(self.x_offset, 3), round(self.y_offset, 3),
                      round(self.x_per_z, 4), round(self.y_per_z, 4)]
        self.last_result = res
        try:
            with open(os.path.join(self.data_dir, "offset_result.json"),
                      "w") as f:
                json.dump(res, f)
        except OSError:
            pass
        for line in res["report"]:
            gcmd.respond_info(line)
        if not res["ok"]:
            raise gcmd.error("LiDAR offset: %s" % (res["reason"],))
        new = (res["x_offset"], res["y_offset"], res["x_per_z"],
               res["y_per_z"])
        if abs(new[0] - self.x_offset) > 5 or abs(new[1] - self.y_offset) > 5:
            raise gcmd.error("LiDAR offset: more than 5mm from the current"
                             " one, not applied (check the cross)")
        if not apply:
            return
        for (attr, var), v in zip(self.OFFSET_VARS, new):
            setattr(self, attr, v)
            self.gcode.run_script_from_command(
                "SAVE_VARIABLE VARIABLE=%s VALUE=%.4f" % (var, v))
        self.offset_source = "saved by LIDAR_OFFSET_CALIBRATE"
        gcmd.respond_info("LiDAR offset saved: X%.2f Y%.2f at Z%.1f, per mm"
                          " Z X%+.3f Y%+.3f" % (new[0], new[1],
                                                self.offset_z, new[2], new[3]))


def creality_pa_gcode(ox, oy, pa0, pas):
    """Creality's PA test pattern, from Auto_pressure_advance_testpadvance.gcode
    (/etc/sysConfig/defData, firmware 1.3.5) which their LiDAR app prints and
    scans. Same moves, but absolute from the frame corner (ox, oy) instead of
    the file's G92 shifts, a Z hop between the boxes and no M205.

    Frame 27 x 139mm; five flow boxes at 140% flow; four zig-zags 2mm apart
    at 180mm/s, each V at its own PA: zig-zag k (left to right), V i (front
    to back) runs at pa0 + pas * (k + 4 * i). Returns (gcode lines, PA grid).
    """
    g = []

    def mv(x, y, e=None, f=1200.0, bx=0.0, by=0.0):
        cmd = "G1 X%.3f Y%.3f" % (ox + bx + x, oy + by + y)
        if e is not None:
            cmd += " E%.6f" % (e,)
        g.append(cmd + " F%.0f" % (f,))

    # frame
    g += ["G0 Z3 F600"]
    mv(0, 139, f=12000)
    g += ["G1 Z0.2 F600", "G1 E0.8 F2400"]
    for x, y, e in ((0, 0, 6.95), (27, 0, 1.35), (27, 0.4, 0.02),
                    (0.4, 0.4, 1.33), (0.4, 4.4, 0.2), (27, 4.4, 1.33),
                    (27, 139, 6.73), (0.4, 139, 1.33), (0.4, 10.4, 6.43),
                    (13.5, 10.4, 0.66), (13.5, 6.4, 0.2), (27, 6.4, 0.675)):
        mv(x, y, e)
    g += ["G1 E-0.8 F2400", "G0 Z3 F600"]
    # flow boxes (Creality's file moves its origin by X1 Y7 here)
    g += ["M221 S140", "SET_PRESSURE_ADVANCE ADVANCE=0.04"]
    # (start x, start y, opposite x, opposite y, E along x, E along y)
    boxes = [(10, 130, 25, 125, 0.75, 0.25)] + [
        (1, y, 12, y + 10, 0.5, 0.5) for y in (112.5, 87.5, 62.5, 37.5, 12.5)]
    for xa, ya, xb, yb, ex, ey in boxes:
        mv(xa, ya, f=18000, bx=1, by=7)
        g += ["G1 Z0.2 F600", "G1 E0.8 F2400"]
        mv(xb, ya, ex, bx=1, by=7)
        mv(xb, yb, ey, bx=1, by=7)
        mv(xa, yb, ex, bx=1, by=7)
        mv(xa, ya, ey, bx=1, by=7)
        g += ["G1 E-0.8 F2400", "G0 Z3 F600"]
    # zig-zags
    g += ["M221 S110"]
    grid = [[round(pa0 + pas * (k + 4 * i), 4) for k in range(4)]
            for i in range(5)]
    for k in range(4):
        bx = 1 + 2 * k
        g += ["SET_PRESSURE_ADVANCE ADVANCE=0.04"]
        mv(15, -7, f=18000, bx=bx, by=7)
        g += ["G1 E0.8 F2400", "G1 Z0.2 F600"]
        mv(15, 5, 0.6, bx=bx, by=7)
        for i in range(5):
            g.append("SET_PRESSURE_ADVANCE ADVANCE=%.4f" % (grid[i][k],))
            mv(2.5, 17.5 + 25 * i, 0.883883, 10800, bx=bx, by=7)
            mv(15.0, 30.0 + 25 * i, 0.883883, 10800, bx=bx, by=7)
        g.append("SET_PRESSURE_ADVANCE ADVANCE=0")
        mv(18, 130, 0.23, bx=bx, by=7)
        g += ["G1 E-0.8 F2400", "G0 Z3 F600"]
        mv(27, 130, f=18000, bx=bx, by=7)
        mv(27, -7, f=18000, bx=bx, by=7)
    g.append("M221 S100")
    return g, grid


def load_config(config):
    return Lidar(config)
