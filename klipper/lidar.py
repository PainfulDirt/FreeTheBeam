# K1 Max LiDAR (Creality "AI LiDAR") for Kalico
#
# The LiDAR is a line-laser height sensor on the toolhead: one capture is a
# ~29mm wide height profile (~1500 points, in um) across X. lidar-direct.py
# (the open driver, started by lidar-start.sh; it replaced Creality's
# cx_ai_middleware, same API) drives it and answers JSON requests on a unix
# socket; this plugin moves the toolhead and collects the profiles, so scans
# can run inside a print.
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
#   #mesh_min: 5, 5            # narrow where meshes and calibrations look
#   #mesh_max: 270, 265        #   (default: what the laser reaches)
#   #twist_y: 145              # LIDAR_TWIST_CALIBRATE lines (default: from
#   #ytwist_x: 100, 200        #   the laser area)
#   #touch: auto               # load_cell, probe, gcode or none (auto:
#                              #   load_cell, else probe, else none)
#   #touch_gcode: NOZZLE_TOUCH # touch: gcode, contact Z from the status
#   #touch_result: nozzle_touch:last_touch_z   #   value object:key
#   #touch_temp: 150           # nozzle temperature for touches
#   #touch_wipe_gcode:         # run before a series of touches
#
# Commands:
#   LIDAR_STATUS
#   LIDAR_SETUP_CHECK  what this printer's setup can do: driver, offsets,
#       timing, service, laser area, touch method; lists what is missing
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
#   LIDAR_GRID_CALIBRATE [COUNT=7,5] [X0= X1= Y0= Y1=] (default: the laser
#       area, _laser_area; K1 Max X10-269 Y6-264)
#       [PASSES=2] [TOLERANCE=0.010] [MAX_TAPS=7] [MAX_SPREAD=0.04] [APPLY=1]
#       the LiDAR against nozzle taps (3 agreeing) on a grid, the laser on
#       the same spot before moving on; X curve -> lidar_twist, Y curve ->
#       lidar_ytwist, the rest reported and kept as data (lidar_grid). After
#       Axis Twist Mesh (github.com/sterling5241/axis-mesh, with permission)
#   LIDAR_TWIST_CALIBRATE [AXIS=X] [Y=twist_y] [X0= X1=] [POINTS=7]
#       [PASSES=2] [SAMPLES=3] [MAX_SPREAD=0.04] [APPLY=1]
#       AXIS=Y [X=ytwist_x] [POINTS=9] [Y0= Y1=]: the same along Y,
#       saved as lidar_ytwist, a straight line (LiDAR meshes were tilted
#       ~8-15 um/100mm along Y against the nozzle and the probe; each
#       spot's texture scatters +-20 um, so only the slope is kept, with
#       its uncertainty); LIDAR_BED_MESH takes off both
#       the gantry tips the toolhead forward/back by an amount that changes
#       along X, which the LiDAR (21mm in front of the nozzle) reads as
#       height (~0.2mm across the bed, 2026-10-29). At spots along X the
#       nozzle touches the bed (load cells), then the laser looks at the same
#       spot: LiDAR - nozzle is saved as lidar_twist for LIDAR_BED_MESH.
#       Also logs the LiDAR distance with the nozzle on the bed
#       (lidar_zero_log, data only). Wrapped by LIDAR_TWIST_CALIBRATION
#       (lidar.example.cfg: nozzle temperature, wipe, bed heater off)
#   LIDAR_BED_MESH [LEVEL=1-5 | MODE=quick|fast|normal|fine|ultimate]
#       [MOTION=continuous|dwell] [X0= X1= Y0= Y1=] [ADAPTIVE=0]
#       [COUNT_X= COUNT_Y=] [PROFILE=lidar] [TWIST=1] [HALF_X=3]
#       1-3: frames along columns at the grid X, moving at constant speed
#       (SPEED=150/120/60, frames kept only at full speed or standing still,
#       placed with the trapq over their exposure window; BOTH_WAYS=1 scans
#       every column there and back and reports the timing shift as
#       moving_lag) or standing at the points (MOTION=dwell, [FRAMES=]
#       1mm apart, SPEED=300/200/200); 4 fine: strips side by side using
#       the whole laser line ([STEP=2] [STRIPS=] [HALF_Y=2]), 30x30 and the
#       full height map on the helper; 5 ultimate: fine + a grid of nozzle
#       touches ([TOUCHES=5] [SAMPLES=3] [WIPE=1] [BED_OFF=1] [MAX_ORDER=3])
#       whose difference to the LiDAR becomes a smooth correction (order
#       picked by leave-one-out, which is also the reported accuracy).
#       ADAPTIVE=1: only the printed objects (+5mm). The mesh is made
#       relative at zero_reference_position, loaded and saved as PROFILE for
#       the session (SAVE_CONFIG keeps it); every mesh is also kept on the
#       helper (meshes/, heightmap-*.json)
#   MESH_COMPARE A=<profile> [B=<profile>|active] [STEP=10]
#       A - B over their overlap: offset, plane, rms/max after each, worst
#       points, what made each profile, whether axis twist compensation is on
#   MESH_BENCHMARK [METHODS=probe,1,2,3,4,5,repeat2] [TRUTH=6] [X0= X1= Y0=
#       Y1=] every method on one area (also cont<speed>, dwell<level>,
#       repeat<level>), then a TRUTHxTRUTH nozzle-touch grid judges them;
#       JSON + HTML report on the helper (benchmarks/). Wrapped by
#       MESH_BENCHMARK_RUN (lidar.example.cfg: soak first)
#   FIRST_LAYER_SCAN PHASE=before|after|cancel  first-layer check: before
#       scans the bare plate under the file's objects (exclude_object) +
#       [MARGIN=8], strips 26mm apart, rows [STEP=2] (coarser until one scan
#       fits [MAX_TIME=120]s), [HEIGHT=] sliced first layer, FILAMENT_TYPE=,
#       FILAMENT_NAME=; after (layer 2, or FINAL=1 from END_PRINT for
#       one-layer prints) retracts [RETRACT=1], lets the nozzle cool by
#       [COOL=30], scans the same strips and rows, has the helper compute the
#       thickness map (fl_analyze), heats up and puts the print back. Bed
#       mesh off during both scans. First_Layer_Learn: half the error per
#       print (max 0.03mm per print, 0.15mm total) becomes the filament's Z
#       offset (fl_zoffset), set by _FIRST_LAYER_OFFSET at the next start
#
# Copy or symlink into klippy/plugins (see docs/install-k1max.md).
# Part of FreeTheBeam, https://github.com/PainfulDirt/FreeTheBeam
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import array
import json
import math
import os
import socket
import sys
import time

# FreeTheBeam release (also in driver/lidar_serial.py and CHANGELOG.md)
VERSION = "0.2.0"

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


def strip_column(rows, half_x=3.0):
    """Heights along the middle of a strip scan: per row the median of the
    bins within half_x mm of the profile centre, as -distance (mm, NaN =
    nothing seen). This close to the centre the profile's own tilt (~0.04mm
    per mm of profile, the LiDAR's mounting) doesn't matter."""
    lo = max(0, int(round((-half_x - BIN_MIN) / BIN)))
    hi = min(NBINS, int(round((half_x - BIN_MIN) / BIN)))
    return [-median(r[lo:hi]) for r in rows]


def twist_at(twist, x):
    """LiDAR twist correction (mm) at laser X: linear between the
    LIDAR_TWIST_CALIBRATE spots, the end values outside them."""
    if not twist or "xs" not in twist:
        return 0.0
    return _table_at(twist["xs"], twist["comp"], x)


def line_slope(spots, comp, k):
    """Slope of comp along coordinate k of the spots [(x, y)], one slope
    for all the lines (spots sharing the other coordinate) with an offset of
    their own: along Y the lines at different X differ by the X twist.
    Returns (slope, its standard error, scatter)."""
    lines = {}
    for sp, c in zip(spots, comp):
        lines.setdefault(sp[1 - k], []).append((sp[k], c))
    pts = []
    for v in lines.values():
        ma = sum(a for a, c in v) / len(v)
        mc = sum(c for a, c in v) / len(v)
        pts += [(a - ma, c - mc) for a, c in v]
    sxx = sum(a * a for a, c in pts)
    if sxx <= 0:
        return 0.0, 0.0, 0.0
    slope = sum(a * c for a, c in pts) / sxx
    dof = max(1, len(pts) - 1 - len(lines))
    sd = math.sqrt(sum((c - slope * a) ** 2 for a, c in pts) / dof)
    return slope, sd / math.sqrt(sxx), sd


# --- Grid calibration against the nozzle (LIDAR_GRID_CALIBRATE). The method
# and this analysis follow Axis Twist Mesh by sterling5241
# (github.com/sterling5241/axis-mesh), used with the author's permission:
# agreeing nozzle taps, the sensor on the same spot before moving on, the
# difference split into an X curve, a Y curve and the rest.
MIN_AGREEING_TAPS = 3


def drift_fit_spots(meas):
    """meas: {spot: [(time, value), ...]} over passes in alternating order.
    Fits a value per spot + one linear drift in time (the sensor against
    the nozzle warms up on a hot bed: ~6 um/min seen on 2026-11-01).
    Returns ({spot: value at the mean time}, drift per s, spread), spread =
    the largest disagreement between the passes once the drift is out."""
    num = den = 0.0
    for v in meas.values():
        mt = sum(t for t, d in v) / len(v)
        md = sum(d for t, d in v) / len(v)
        num += sum((t - mt) * (d - md) for t, d in v)
        den += sum((t - mt) ** 2 for t, d in v)
    k = num / den if den > 0 else 0.0
    tm = sum(t for v in meas.values() for t, d in v) / sum(
        len(v) for v in meas.values())
    vals, spread = {}, 0.0
    for sp, v in meas.items():
        r = [d - k * (t - tm) for t, d in v]
        vals[sp] = sum(r) / len(r)
        spread = max(spread, max(r) - min(r))
    return vals, k, spread


def agreeing_value(taps, k, tolerance):
    """Mean of the k readings closest together if they lie within
    tolerance of each other, else None (tap again)."""
    if len(taps) < k:
        return None
    ordered = sorted(taps)
    best = None
    for i in range(len(ordered) - k + 1):
        group = ordered[i:i + k]
        spread = group[-1] - group[0]
        if spread <= tolerance and (best is None or spread < best[0]):
            best = (spread, group)
    return sum(best[1]) / k if best is not None else None


def grid_analyze(xs, ys, d, tolerance=0.010):
    """d[(i, j)] = sensor - nozzle (mm) at (xs[i], ys[j]), points may be
    missing. The mean is the constant offset; the rest is fitted as an X
    curve + a Y curve (backfitting: on a full grid the first pass is the
    row/column-mean least-squares fit), and what is left after both is the
    interaction. Also flags suspect points and edge rows/columns."""
    nx, ny = len(xs), len(ys)
    present = sorted(d, key=lambda k: (k[1], k[0]))
    mean = sum(d[k] for k in present) / len(present)
    res = [[d[(i, j)] - mean if (i, j) in d else None for i in range(nx)]
           for j in range(ny)]
    cols = dict((i, [j for j in range(ny) if res[j][i] is not None])
                for i in range(nx))
    rows = dict((j, [i for i in range(nx) if res[j][i] is not None])
                for j in range(ny))
    if any(not v for v in cols.values()) or any(not v for v in rows.values()):
        raise ValueError("a whole row or column of the grid is missing")
    full = len(present) == nx * ny
    x_eff, y_eff = [0.0] * nx, [0.0] * ny
    for _ in range(1 if full else 50):
        x_eff = [sum(res[j][i] - y_eff[j] for j in cols[i]) / len(cols[i])
                 for i in range(nx)]
        m = sum(len(cols[i]) * x_eff[i] for i in range(nx)) / len(present)
        x_eff = [v - m for v in x_eff]
        y_eff = [sum(res[j][i] - x_eff[i] for i in rows[j]) / len(rows[j])
                 for j in range(ny)]
    inter = [[None if res[j][i] is None else res[j][i] - x_eff[i] - y_eff[j]
              for i in range(nx)] for j in range(ny)]

    def rms(vals):
        vals = list(vals)
        return math.sqrt(sum(v * v for v in vals) / len(vals))
    iv = [inter[j][i] for i, j in present]
    return {"count": len(present), "mean_offset": mean,
            "rms_residual": rms(res[j][i] for i, j in present),
            "x_effect": x_eff, "y_effect": y_eff,
            "x_effect_range": max(x_eff) - min(x_eff),
            "y_effect_range": max(y_eff) - min(y_eff),
            "interaction": inter, "interaction_rms": rms(iv),
            "interaction_max": max(abs(v) for v in iv),
            "suspect_points": _grid_suspects(inter, xs, ys, tolerance),
            "edge_warnings": _grid_edges("X", xs, x_eff)
            + _grid_edges("Y", ys, y_eff)}


def _grid_suspects(inter, xs, ys, tolerance):
    """Points of the interaction far off the median of their neighbours
    (robust MAD threshold): likely one bad reading."""
    nx, ny = len(xs), len(ys)
    if nx < 3 or ny < 3:
        return []
    dev = {}
    for j in range(ny):
        for i in range(nx):
            if inter[j][i] is None:
                continue
            nb = [inter[jj][ii] for jj in range(max(0, j - 1), min(ny, j + 2))
                  for ii in range(max(0, i - 1), min(nx, i + 2))
                  if (ii, jj) != (i, j) and inter[jj][ii] is not None]
            if len(nb) >= 3:
                dev[(i, j)] = inter[j][i] - median(nb)
    if len(dev) < 5:
        return []
    sigma = 1.4826 * median([abs(v) for v in dev.values()])
    limit = max(4.0 * sigma, 2.0 * tolerance)
    return [(xs[i], ys[j], round(v, 4)) for (i, j), v in sorted(
        dev.items(), key=lambda kv: kv[0][::-1]) if abs(v) > limit]


def _grid_edges(axis, pos, eff):
    """An outer row/column whose effect breaks from the trend of the two
    next to it far more than the interior curve does."""
    n = len(eff)
    if n < 5:
        return []
    second = [eff[k - 1] - 2 * eff[k] + eff[k + 1] for k in range(1, n - 1)]
    limit = max(3.0 * median([abs(v) for v in second[1:-1]]), 0.02)
    out = []
    for edge, a, b in ((0, 1, 2), (n - 1, n - 2, n - 3)):
        jump = eff[edge] - (2 * eff[a] - eff[b])
        if abs(jump) > limit:
            out.append((axis, pos[edge], round(jump, 4)))
    return out


def ytwist_at(twist, y):
    """The same along laser Y (LIDAR_TWIST_CALIBRATE AXIS=Y, carried in
    the twist dict as ys/ycomp)."""
    if not twist or "ys" not in twist:
        return 0.0
    return _table_at(twist["ys"], twist["ycomp"], y)


CAL_HOT = 45.0    # C: calibrations at or above it form the hot group
CAL_RUNS = 5      # runs kept per bed temperature


def cal_runs(entry):
    """The runs of one lidar_cal_by_temp entry (an entry from before
    2026-11-02 is one run)."""
    if "runs" in entry:
        return list(entry["runs"])
    return [dict((k, entry[k]) for k in ("twist", "ytwist", "time")
                 if k in entry)]


def average_tables(tabs, key):
    """Weighted mean of tables [(table, weight)] at the union of their
    points (key "xs" or "ys", values "comp")."""
    pts = sorted(set(p for t, w in tabs for p in t[key]))
    tw = sum(w for t, w in tabs)
    return {key: pts, "comp": [round(sum(
        w * _table_at(t[key], t["comp"], p) for t, w in tabs) / tw, 5)
        for p in pts]}


def tables_for_temp(store, temp):
    """The LiDAR tables for bed temperature temp from lidar_cal_by_temp
    ({"70": {"runs": [{"twist": ..., "ytwist": ...}, ...]}, ...}): the
    weighted mean of every run of its group (hot: bed >= CAL_HOT, else
    cold; per table, the other group's if its own has none). Between 60 and 100C the
    tables agreed within the scatter between runs (2026-11-01), so more
    runs beat picking the nearest temperature. Returns (twist, ytwist,
    note)."""
    runs = [(float(t), r) for t, e in (store or {}).items()
            for r in cal_runs(e)]
    hot = temp >= CAL_HOT
    out, notes = [], []
    for name, key in (("twist", "xs"), ("ytwist", "ys")):
        have = [x for x in runs if x[1].get(name)]
        group = [x for x in have if (x[0] >= CAL_HOT) == hot] or have
        tabs = [(r[name], float(r[name].get("w", 1))) for t, r in group]
        if not tabs:
            out.append(None)
            continue
        out.append(average_tables(tabs, key))
        temps = sorted(set(t for t, r in group))
        notes.append("%s %d run%s (%sC)" % (
            name, len(tabs), "" if len(tabs) == 1 else "s",
            "/".join("%.0f" % t for t in temps)))
    return out[0], out[1], ", ".join(notes)


def _table_at(xs, comp, x):
    if x <= xs[0]:
        return comp[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return comp[i - 1] + t * (comp[i] - comp[i - 1])
    return comp[-1]


def _fill_column(col):
    """Fill NaN entries of a mesh column from its neighbours (in place);
    returns how many, None if fewer than two values are there."""
    have = [j for j, v in enumerate(col) if v == v]
    if len(have) < 2:
        return None
    n = 0
    for j, v in enumerate(col):
        if v != v:
            lo = max([k for k in have if k < j], default=None)
            hi = min([k for k in have if k > j], default=None)
            if lo is None or hi is None:
                col[j] = col[hi if lo is None else lo]
            else:
                col[j] = col[lo] + (col[hi] - col[lo]) * (j - lo) / (hi - lo)
            n += 1
    return n


def _mesh_result(res, cols, xs, ys):
    # rows = Y, relative to the median: the heights are -distance, only
    # differences mean anything (Klipper's zero reference does the rest)
    flat = sorted(v for c in cols for v in c)
    mid = median(flat)
    res.update({"ok": True, "range": round(flat[-1] - flat[0], 4),
                "matrix": [[round(cols[i][j] - mid, 5)
                            for i in range(len(xs))] for j in range(len(ys))]})
    return res


def column_value(fr, y, half_y=1.5):
    """Height at y from a column's frames fr = sorted [(frame y, height)]:
    the median of the frames within half_y, else linear between the
    nearest frame on each side, else the nearest one (NaN if none)."""
    near = [h for fy, h in fr if abs(fy - y) <= half_y + 1e-6]
    if near:
        return median(near)
    lo = [f for f in fr if f[0] < y]
    hi = [f for f in fr if f[0] > y]
    if lo and hi:
        (y0, h0), (y1, h1) = lo[-1], hi[0]
        return h0 + (h1 - h0) * (y - y0) / (y1 - y0)
    if lo or hi:
        return (lo[-1] if lo else hi[0])[1]
    return NAN


def mesh_points(path, rows, xs, ys, half_x=3.0, twist=None, half_y=1.5):
    """Bed mesh from frames along columns (LIDAR_BED_MESH levels 1-3):
    path[k] = (x, y) where frame rows[k] was taken (standing still, or the
    middle of what it saw while moving along Y). Line centre +-half_x; per
    grid point (xs[i], ys[j]) the frames of column xs[i] within half_y, or
    the nearest frames around it; minus the twist correction at xs[i]."""
    res = {"ok": False, "xs": list(xs), "ys": list(ys)}
    hs = strip_column(rows, half_x)
    by_col = {}
    for (px, py), h in zip(path, hs):
        if h == h:
            by_col.setdefault(round(px, 2), []).append((py, h))
    cols, spread = [], []
    for x in xs:
        fr = sorted(by_col.get(round(x, 2), []))
        if len(fr) < 2:
            res["reason"] = "the LiDAR saw nothing along X%.1f" % (x,)
            return res
        col = []
        for y in ys:
            near = [h for fy, h in fr if abs(fy - y) <= half_y + 1e-6]
            if len(near) > 1:
                spread.append(max(near) - min(near))
            col.append(column_value(fr, y, half_y))
        tw = twist_at(twist, x)
        cols.append([v - tw - ytwist_at(twist, y) for v, y in zip(col, ys)])
    res.update({"filled": 0, "frames": len(path),
                "spread": round(median(spread), 4) if spread else None,
                "twist": [round(twist_at(twist, x), 4) for x in xs]})
    return _mesh_result(res, cols, xs, ys)


def direction_lag(path, rows, moves, half_x=3.0):
    """Moving scans: how far the frames' Y is off along the move (the frame
    timing, LIDAR_TIMING, measured standing still). Columns scanned both
    ways disagree by twice that: find the shift d (mm, + = the frame looked
    further along its move than assumed) that makes the two directions
    agree. moves[k] = (span, dir) per frame. Returns {"shift", "rms",
    "rms0", "n"} or None."""
    hs = strip_column(rows, half_x)
    cols = {}
    for (x, y), h, (span, d) in zip(path, hs, moves):
        if h == h and d:
            cols.setdefault(round(x, 2), {1: [], -1: []})[d].append((y, h))
    pairs = [(sorted(c[1]), sorted(c[-1])) for c in cols.values()
             if len(c[1]) > 3 and len(c[-1]) > 3]
    if not pairs:
        return None

    def mis(dd):
        e = []
        for up, dn in pairs:
            dn2 = [(y - dd, h) for y, h in dn]
            for y, h in up:
                yy = y + dd
                if dn2[0][0] <= yy <= dn2[-1][0]:
                    e.append(h - column_value(dn2, yy, 0.0))
        if len(e) < 4:
            return None
        mu = sum(e) / len(e)
        return math.sqrt(sum((v - mu) ** 2 for v in e) / len(e))
    best = None
    for i in range(-40, 41):
        r = mis(i * 0.25)
        if r is not None and (best is None or r < best[1]):
            best = (i * 0.25, r)
    if best is None:
        return None
    r0 = mis(0.0)
    return {"shift": best[0], "rms": round(best[1], 5),
            "rms0": None if r0 is None else round(r0, 5), "n": len(pairs)}


# --- surfaces: least squares polynomials in X/Y, order picked by
# leave-one-out (the nozzle-touch correction of the ultimate mesh)
def solve(a, b):
    """Solve a x = b (small dense system, Gauss-Jordan with pivoting)."""
    n = len(b)
    m = [list(row) + [v] for row, v in zip(a, b)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        if abs(m[p][c]) < 1e-12:
            raise ValueError("singular")
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                m[r] = [x - f * y for x, y in zip(m[r], m[c])]
    return [m[i][n] / m[i][i] for i in range(n)]


def _poly_row(x, y, order):
    # scaled to about +-1 over the bed, for a well-conditioned fit
    u, v = (x - 150.0) / 150.0, (y - 150.0) / 150.0
    return [u ** i * v ** (d - i) for d in range(order + 1)
            for i in range(d + 1)]


def poly_fit(pts, order):
    """pts [(x, y, value)] -> coefficients of a polynomial of `order`."""
    rows = [_poly_row(x, y, order) for x, y, v in pts]
    n = len(rows[0])
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(n)]
           for i in range(n)]
    atb = [sum(r[i] * p[2] for r, p in zip(rows, pts)) for i in range(n)]
    return solve(ata, atb)


def poly_eval(coef, order, x, y):
    return sum(c * t for c, t in zip(coef, _poly_row(x, y, order)))


def ref_height(xs, ys, matrix, ref, radius=60.0):
    """Mesh height at the zero reference from a smooth fit over the points
    within `radius` (quadratic, plane with few points), None if too few.
    The mesh is pinned to this height (True Zero touches there): taken from
    the few nearest points, their texture noise would shift the whole mesh
    (~10-15 um between runs measured on 2026-10-29)."""
    pts = [(x, y, matrix[j][i]) for j, y in enumerate(ys)
           for i, x in enumerate(xs)
           if matrix[j][i] == matrix[j][i]
           and (x - ref[0]) ** 2 + (y - ref[1]) ** 2 <= radius ** 2]
    for order, need in ((2, 15), (1, 6)):
        if len(pts) >= need:
            try:
                return poly_eval(poly_fit(pts, order), order, ref[0], ref[1])
            except ValueError:
                pass
    return None


def fit_correction(pts, max_order=3):
    """Smooth correction surface through pts [(x, y, nozzle - LiDAR)].

    For each order up to max_order (with at least 2.5 points per
    coefficient) the leave-one-out residuals: every point predicted by the
    fit without it, i.e. how wrong the corrected mesh is where nothing was
    touched. The order with the lowest LOO rms wins (a higher one only if
    it is 10% better). Returns a dict with order, coef, loo (rms per
    order), residuals (LOO, at the chosen order)."""
    out = {"loo": {}, "fit_rms": {}}
    best = None
    for order in range(max_order + 1):
        nterm = (order + 1) * (order + 2) // 2
        if len(pts) < 2.5 * nterm:
            break
        try:
            coef = poly_fit(pts, order)
            loo = []
            for k in range(len(pts)):
                c = poly_fit(pts[:k] + pts[k + 1:], order)
                loo.append(pts[k][2] - poly_eval(c, order, pts[k][0],
                                                 pts[k][1]))
        except ValueError:
            continue
        rms = math.sqrt(sum(v * v for v in loo) / len(loo))
        fit = [p[2] - poly_eval(coef, order, p[0], p[1]) for p in pts]
        out["loo"][order] = round(rms, 5)
        out["fit_rms"][order] = round(math.sqrt(
            sum(v * v for v in fit) / len(fit)), 5)
        if best is None or rms < 0.9 * best[0]:
            best = (rms, order, coef, loo)
    if best is None:
        raise ValueError("not enough points for a correction")
    out.update({"order": best[1], "coef": best[2], "loo_rms": best[0],
                "residuals": [round(v, 5) for v in best[3]]})
    return out


def drift_fit(refs):
    """refs [(time, z)] of repeated touches on one spot: (z0, slope per
    second) of a straight line, to take a slow drift (bed or hotend
    settling) out of the touches between them."""
    if len(refs) < 2:
        return (refs[0][1] if refs else 0.0), 0.0
    k, c = linfit([t for t, z in refs], [z for t, z in refs])
    return c, k


def mesh_stats(pts):
    """pts [(x, y, a - b)] on a common grid: offset, plane and what is left
    (MESH_COMPARE, the benchmark)."""
    d = [p[2] for p in pts]
    mu = sum(d) / len(d)
    d0 = [v - mu for v in d]
    coef = poly_fit(pts, 1)
    d1 = [p[2] - poly_eval(coef, 1, p[0], p[1]) for p in pts]

    def rms(v):
        return math.sqrt(sum(a * a for a in v) / len(v))
    worst = sorted(range(len(pts)), key=lambda i: -abs(d0[i]))[:3]
    return {"n": len(pts), "offset": round(mu, 4),
            # order 1 terms are 1, v, u, per 150mm (_poly_row) -> per 100mm
            "plane_x": round(coef[2] / 1.5, 4),
            "plane_y": round(coef[1] / 1.5, 4),
            "rms": round(rms(d0), 4), "max": round(max(map(abs, d0)), 4),
            "rms_plane": round(rms(d1), 4),
            "max_plane": round(max(map(abs, d1)), 4),
            "worst": [[pts[i][0], pts[i][1], round(d0[i], 4)]
                      for i in worst]}


# fine meshes keep a strip as groups of GROUP bins (0.5mm of profile)
GROUP = 10
NGROUPS = NBINS // GROUP


def group_row(row, spike=0.1, reach=4):
    """A row of binned distances as NGROUPS medians (NaN if less than half
    of a group was seen). A group further than `spike` mm from the median
    of the `reach` groups on each side is dropped, and so is one with fewer
    than 3 seen groups among them: the line throws a few spurious readings
    (0.4 % of a fine scan on 2026-10-29, -1 to -4.5 mm, mostly at the
    line's +X end and at the front of the bed)."""
    out = []
    for g in range(NGROUPS):
        v = [d for d in row[g * GROUP:(g + 1) * GROUP] if d == d]
        out.append(median(v) if 2 * len(v) >= GROUP else NAN)
    clean = []
    for g, d in enumerate(out):
        nb = [e for e in out[max(0, g - reach):g] + out[g + 1:g + 1 + reach]
              if e == e]
        # a lone reading among no-data can't be judged and is the usual
        # spurious one (the line's +X end at the front-right: -1..-4 mm
        # readings between empty groups on 2026-10-29), so it goes too
        clean.append(NAN if d != d or len(nb) < 3 or
                     abs(d - median(nb)) > spike else d)
    return clean


def mesh_fine(strips, xs, ys, x_dir=-1.0, half_x=3.0, half_y=2.0,
              twist=None, cell=0.5, touches=None, max_order=3):
    """Dense bed mesh (LIDAR_BED_MESH MODE=fine) from strips that use the
    whole width of the laser line.

    strips: [(laser x, row ys, rows of NGROUPS distances)]. The profile has
    its own tilt (the LiDAR's mounting, the same in every strip): the
    median over all rows of each group against the line centre, minus what
    the bed's own slope across X adds to that. Every strip also gets the
    twist correction at its X. The corrected heights go into a map of
    `cell` mm columns x the scanned rows; each mesh point (xs[i], ys[j]) is
    the median of the map within +-half_x / +-half_y. Returns (result,
    height map).

    touches [(x, y, nozzle z, time, is_ref)] (MODE=ultimate): nozzle
    touches with the load cells. The repeated touches on one spot (is_ref)
    give a straight-line drift in time that comes off all of them; then
    nozzle - LiDAR at the touched spots becomes a smooth correction surface
    (fit_correction) added to the mesh: the mesh then follows the nozzle,
    not the LiDAR. The map keeps the LiDAR heights, with the correction
    alongside."""
    res = {"ok": False, "xs": list(xs), "ys": list(ys)}
    gx = [BIN_MIN + (g + 0.5) * GROUP * BIN for g in range(NGROUPS)]
    use = [g for g in range(NGROUPS) if abs(gx[g]) <= 14.0]
    mid = [g for g in range(NGROUPS) if abs(gx[g]) <= 1.0]
    hs = []      # per strip: rows of heights (-distance) and centre values
    for x, sy, rows in strips:
        rr = [[-d for d in r] for r in rows]
        hs.append((rr, [median([r[g] for g in mid]) for r in rr]))
    # profile tilt against the line centre
    rel = [[] for g in range(NGROUPS)]
    for rr, cen in hs:
        for r, c in zip(rr, cen):
            if c == c:
                for g in use:
                    if r[g] == r[g]:
                        rel[g].append(r[g] - c)
    tilt = [median(v) for v in rel]
    # the bed's slope across X shows up in it too: take it out
    sx = [x for x, sy, rows in strips]
    sc = [median(cen) - twist_at(twist, x)
          for x, (rr, cen) in zip(sx, hs)]
    slope = linfit(sx, sc)[0] if len(sx) > 1 else 0.0
    tilt = [t - slope * x_dir * gx[g] for g, t in enumerate(tilt)]
    # height map: cell columns (cell c from x0 + c * cell) x row ys (all
    # strips scan the same ys)
    wx = [x + x_dir * gx[g] for x in sx for g in use]
    x0 = math.floor(min(wx) / cell) * cell
    ncell = int(math.floor((max(wx) - x0) / cell)) + 1
    ykeys = sorted(set(round(y, 3) for x, sy, rows in strips for y in sy))
    yi = dict((y, i) for i, y in enumerate(ykeys))
    acc = [[[0.0, 0] for c in range(ncell)] for y in ykeys]
    edges = []   # per strip: (first, last) cell
    for (x, sy, rows), (rr, cen) in zip(strips, hs):
        tw = twist_at(twist, x)
        cells = [int(math.floor((x + x_dir * gx[g] - x0) / cell))
                 for g in use]
        edges.append((min(cells), max(cells)))
        for y, r in zip(sy, rr):
            row = acc[yi[round(y, 3)]]
            ytw = ytwist_at(twist, y)
            for g, c in zip(use, cells):
                if r[g] == r[g]:
                    a = row[c]
                    a[0] += r[g] - tilt[g] - tw - ytw
                    a[1] += 1
    hmap = [[a[0] / a[1] if a[1] else NAN for a in row] for row in acc]
    # seams: where two strips meet, the step between their last cells
    # (per +-2 rows), a check of the tilt and twist corrections: median step
    # and a robust spread (1.4826 x the median deviation, = the rms for
    # clean data), so a stray reading at a line's end can't dominate it
    seams = []
    order = sorted(range(len(sx)), key=lambda i: sx[i])
    for i, j in zip(order, order[1:]):
        ca, cb = sorted((edges[i][0], edges[i][1], edges[j][0],
                         edges[j][1]))[1:3]
        d = []
        for k in range(2, len(hmap) - 2, 5):
            va = median([hmap[m][ca] for m in range(k - 2, k + 3)])
            vb = median([hmap[m][cb] for m in range(k - 2, k + 3)])
            if va == va and vb == vb:
                d.append(va - vb)
        if d:
            m = median(d)
            seams.append([round(x0 + (ca + cb + 1) * cell / 2, 2),
                          round(m * 1000.0),
                          round(1.4826 * median([abs(v - m) for v in d])
                                * 1000.0)])
    # mesh points
    cols, filled = [], 0
    for x in xs:
        # cells whose centre is within half_x
        c0 = max(0, int(math.ceil((x - half_x - x0) / cell - 0.5 - 1e-6)))
        c1 = min(ncell - 1,
                 int(math.floor((x + half_x - x0) / cell - 0.5 + 1e-6)))
        col = []
        for y in ys:
            v = [hmap[yi[k]][c] for k in ykeys if abs(k - y) <= half_y + 1e-6
                 for c in range(c0, c1 + 1)]
            col.append(median(v))
        n = _fill_column(col)
        if n is None:
            res["reason"] = "nothing seen along X%.1f" % (x,)
            return res, None
        filled += n
        cols.append(col)
    corr = None
    if touches:
        def at(x, y, hw=1.5):
            c0 = max(0, int(math.ceil((x - hw - x0) / cell - 0.5 - 1e-6)))
            c1 = min(ncell - 1,
                     int(math.floor((x + hw - x0) / cell - 0.5 + 1e-6)))
            return median([hmap[yi[k]][c] for k in ykeys
                           if abs(k - y) <= hw + 1e-6
                           for c in range(c0, c1 + 1)])
        refs = [(t[3], t[2]) for t in touches if t[4]]
        z0, k = drift_fit(refs)
        t0 = refs[0][0] if refs else 0.0
        pts = []
        for x, y, z, t, is_ref in touches:
            h = at(x, y)
            if not is_ref and h == h:
                pts.append((x, y, z - k * (t - t0) - h))
        try:
            corr = fit_correction(pts, max_order)
        except ValueError as e:
            res["reason"] = "touch correction: %s" % (e,)
            return res, None
        for i, x in enumerate(xs):
            cols[i] = [v + poly_eval(corr["coef"], corr["order"], x, y)
                       for v, y in zip(cols[i], ys)]
        corr.update({"drift_per_min": round(k * 60.0, 5),
                     "touches": [[p[0], p[1], round(p[2], 5), r]
                                 for p, r in zip(pts, corr["residuals"])],
                     "refs": [[round(t - t0, 1), round(z, 5)]
                              for t, z in refs],
                     "poly": "sum coef * u^i * v^(d-i), d = 0..order,"
                             " i = 0..d, u = (x-150)/150, v = (y-150)/150"})
        res["correction"] = corr
    k = linfit(gx[use[0]:use[-1] + 1], tilt[use[0]:use[-1] + 1])[0]
    res.update({"filled": filled, "tilt": round(k, 5),
                "bed_slope": round(slope, 6), "seams": seams,
                "twist": [round(twist_at(twist, x), 4) for x in sx]})
    _mesh_result(res, cols, xs, ys)
    m = median([v for row in hmap for v in row])
    out = {"x0": x0 + cell / 2, "dx": cell, "ys": ykeys, "unit": "um",
           "strips": sx, "tilt_um": [round(t * 1000.0) if t == t else None
                                     for t in tilt],
           "h": [[int(round((v - m) * 1000.0)) if v == v else None
                  for v in row] for row in hmap]}
    if corr is not None:
        # nozzle-referenced height = h + correction (um), the first-layer
        # check can use either
        out["correction"] = dict((k, corr[k]) for k in (
            "order", "coef", "poly", "loo_rms", "touches", "drift_per_min"))
    return res, out


# --- first-layer check: bare plate before the print, the first layer after
# it, the same strips and rows both times (FIRST_LAYER_SCAN)
def fl_plan(objects, margin, lo, hi, step, row_s=0.25, strip_s=2.5,
            max_time=120.0, width=28.0, spacing=26.0):
    """Strips and rows over the printed objects + margin.

    objects: [[(x, y), ...] polygon per object]; lo/hi: the laser's reach
    (x0, y0) / (x1, y1). Strips are `spacing` apart (the line covers
    `width`, +-14 mm used), rows `step` apart; the step grows (1, 1.5, 2,
    3 mm) until one scan fits max_time (s, at row_s per row + strip_s per
    strip). Returns dict(strips, y0, y1, step, est) or None (nothing in
    reach)."""
    xs = [p[0] for o in objects for p in o]
    ys = [p[1] for o in objects for p in o]
    if not xs:
        return None
    ax0, ax1 = max(min(xs) - margin, lo[0] - width / 2), \
        min(max(xs) + margin, hi[0] + width / 2)
    ay0, ay1 = max(min(ys) - margin, lo[1]), min(max(ys) + margin, hi[1])
    if ax1 <= ax0 or ay1 <= ay0:
        return None
    n = max(1, int(math.ceil((ax1 - ax0 - width) / spacing)) + 1)
    # strips centred on the area, inside the reach
    span = (n - 1) * spacing
    c = (ax0 + ax1) / 2.0
    strips = [min(max(c - span / 2.0 + i * spacing, lo[0]), hi[0])
              for i in range(n)]
    steps = [s for s in (1.0, 1.5, 2.0, 3.0) if s >= step] or [step]
    for s in steps:
        rows = int(math.ceil((ay1 - ay0) / s)) + 1
        est = n * (rows * row_s + strip_s)
        if est <= max_time:
            break
    return {"strips": [round(x, 2) for x in strips], "y0": round(ay0, 2),
            "y1": round(ay0 + (rows - 1) * s, 2), "step": s,
            "est": round(est, 1)}


def _in_poly(x, y, poly):
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and \
                x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def _poly_dist(x, y, poly):
    """Distance from (x, y) to the polygon's outline."""
    best = 1e9
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i - 1], poly[i]
        dx, dy = x2 - x1, y2 - y1
        L = dx * dx + dy * dy
        t = 0.0 if L == 0 else max(0.0, min(1.0, ((x - x1) * dx
                                                  + (y - y1) * dy) / L))
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return best


def fl_analyze(plan, before, after, cell=0.5, erode=2.0, bare_gap=3.0,
               coarse=10.0):
    """Thickness of the first layer from the two scans.

    plan: strips, ys, x_dir, z_before / z_after (toolhead Z of each scan:
    commanded + gcode offset, bed mesh off), height (sliced first layer),
    objects [{name, polygon}]. before/after: {strip x: rows of NGROUPS
    distances (mm)}, the same rows. Surface height = z - distance, so the
    thickness per cell = (z_a - d_a) - (z_b - d_b). The plate itself moves
    a little between the scans (glass settling, minutes apart): a plane
    through the bare-plate cells (further than bare_gap from every object)
    comes off. Printed cells: inside an object, at least `erode` mm from
    its outline (the laser sheet blends ~1.8 mm along Y at edges) and
    clearly above the plate (> 40 % of the height). Returns (result,
    map)."""
    h0 = plan["height"]
    res = {"ok": False, "height": h0}
    gx = [BIN_MIN + (g + 0.5) * GROUP * BIN for g in range(NGROUPS)]
    use = [g for g in range(NGROUPS) if abs(gx[g]) <= 14.0]
    x_dir = plan.get("x_dir", -1.0)
    ys = plan["ys"]
    dz = plan["z_after"] - plan["z_before"]
    wx = [x + x_dir * gx[g] for x in plan["strips"] for g in use]
    x0 = math.floor(min(wx) / cell) * cell
    ncell = int(math.floor((max(wx) - x0) / cell)) + 1
    acc = [[[0.0, 0, 0.0] for c in range(ncell)] for y in ys]
    for x in plan["strips"]:
        rb, ra = before.get(x), after.get(x)
        if rb is None or ra is None:
            continue
        cells = [int(math.floor((x + x_dir * gx[g] - x0) / cell))
                 for g in use]
        for j, (b, a) in enumerate(zip(rb, ra)):
            row = acc[j]
            for g, c in zip(use, cells):
                if b[g] == b[g] and a[g] == a[g]:
                    row[c][0] += dz - (a[g] - b[g])
                    row[c][1] += 1
                    row[c][2] += gx[g]
    t = [[a[0] / a[1] if a[1] else NAN for a in row] for row in acc]
    # position along the laser line (mm) each cell was seen at
    lp = [[a[2] / a[1] if a[1] else 0.0 for a in row] for row in acc]
    cx = [x0 + (c + 0.5) * cell for c in range(ncell)]
    polys = [o["polygon"] for o in plan["objects"]]
    # where each cell is: (object index or -1, distance to that outline);
    # bare = outside every object by more than bare_gap
    where = []
    for y in ys:
        wrow = []
        for x in cx:
            k, d = -1, 1e9
            for i, p in enumerate(polys):
                di = _poly_dist(x, y, p)
                if _in_poly(x, y, p):
                    k, d = i, di
                    break
                d = min(d, di)
            wrow.append((k, d))
        where.append(wrow)
    bare = [(x, y, t[j][c]) for j, y in enumerate(ys)
            for c, x in enumerate(cx)
            if t[j][c] == t[j][c] and where[j][c][0] < 0
            and where[j][c][1] > bare_gap]
    if len(bare) < 30:
        res["reason"] = "too little bare plate around the print (%d cells)"\
            % (len(bare),)
        return res, None
    # robust plane: twice, dropping what is far off (strings, blobs)
    pl = plane_fit(bare)
    for it in range(2):
        r = [p[2] - (pl[0] + pl[1] * p[0] + pl[2] * p[1]) for p in bare]
        mad = median([abs(v) for v in r]) or 1e-3
        bare = [p for p, v in zip(bare, r) if abs(v) <= 4.0 * mad + 0.005]
        pl = plane_fit(bare)
    r = [p[2] - (pl[0] + pl[1] * p[0] + pl[2] * p[1]) for p in bare]
    bare_rms = math.sqrt(sum(v * v for v in r) / len(r))
    tc = [[v - (pl[0] + pl[1] * x + pl[2] * y) if v == v else NAN
           for x, v in zip(cx, row)] for y, row in zip(ys, t)]
    # printed cells, then their neighbours along X and Y must be printed too
    pr = [[where[j][c][0] >= 0 and where[j][c][1] >= erode
           and tc[j][c] == tc[j][c] and tc[j][c] > 0.4 * h0
           for c in range(ncell)] for j in range(len(ys))]
    printed = []
    for j in range(1, len(ys) - 1):
        for c in range(1, ncell - 1):
            if pr[j][c] and pr[j - 1][c] and pr[j + 1][c] and \
                    pr[j][c - 1] and pr[j][c + 1]:
                printed.append((cx[c], ys[j], tc[j][c], where[j][c][0],
                                lp[j][c]))
    if len(printed) < 50:
        res["reason"] = "too few solid first-layer cells (%d): narrow" \
            " lines can't be measured" % (len(printed),)
        return res, None
    # The line reads plastic with a slope along the line that it doesn't
    # have on PEI (PLA: -0.9 um/mm against PEI, measured on 2026-10-29 by
    # overlapping strips; ~30 um across a strip, bands every strip). The
    # strips sit at different X, so it can be fitted together with the
    # layer's own tilt and taken off; over less than ~1.5 strips the two
    # can't be told apart and the plan's value (the filament's last fit)
    # is used instead.
    ramp, fitted = plan.get("line_ramp") or 0.0, False
    if max(p[0] for p in printed) - min(p[0] for p in printed) > 40.0:
        rows4 = [(1.0, p[0] - 150.0, p[1] - 150.0, p[4]) for p in printed]
        ata = [[sum(r[i] * r[k] for r in rows4) for k in range(4)]
               for i in range(4)]
        atb = [sum(r[i] * p[2] for r, p in zip(rows4, printed))
               for i in range(4)]
        try:
            ramp, fitted = solve(ata, atb)[3], True
        except ValueError:
            pass
    if ramp:
        printed = [(p[0], p[1], p[2] - ramp * p[4], p[3], p[4])
                   for p in printed]
        tc = [[v - ramp * lp[j][c] if v == v else NAN
               for c, v in enumerate(row)] for j, row in enumerate(tc)]
    v = sorted(p[2] for p in printed)

    def pct(q):
        return v[min(len(v) - 1, int(q * len(v)))]
    med = median(v)
    objs = []
    for i, o in enumerate(plan["objects"]):
        w = [p[2] for p in printed if p[3] == i]
        if w:
            objs.append({"name": o.get("name", str(i)), "cells": len(w),
                         "median": round(median(w), 4)})
    tilt = plane_fit([(p[0], p[1], p[2]) for p in printed])
    # coarse residual map (thickness - median) for the learned mesh
    # correction: thicker = nozzle too high there
    cm = {}
    for x, y, d, k, q in printed:
        key = (int(math.floor(x / coarse)), int(math.floor(y / coarse)))
        cm.setdefault(key, []).append(d - med)
    coarse_map = [[(k[0] + 0.5) * coarse, (k[1] + 0.5) * coarse,
                   round(median(w), 4), len(w)]
                  for k, w in sorted(cm.items()) if len(w) >= 20]
    res.update({
        "ok": True, "median": round(med, 4), "p05": round(pct(0.05), 4),
        "p95": round(pct(0.95), 4), "error": round(med - h0, 4),
        "cells": len(printed), "area_mm2": round(len(printed) * cell
                                                 * plan.get("step", 2.0), 1),
        "objects": objs,
        "tilt_um_per_100mm": [round(tilt[1] * 1e5, 1),
                              round(tilt[2] * 1e5, 1)] if tilt else None,
        # slope along the laser line taken off (um/mm), fitted or not
        "line_ramp": round(ramp * 1000.0, 2), "line_ramp_fitted": fitted,
        # plate drift between the scans: at the middle of the area (mm) and
        # its slope (um/100mm in X, Y)
        "drift": {"plane": [round(pl[0] + pl[1] * (cx[0] + cx[-1]) / 2
                                  + pl[2] * (ys[0] + ys[-1]) / 2, 4),
                            round(pl[1] * 1e5, 2), round(pl[2] * 1e5, 2)],
                  "bare_cells": len(bare), "bare_rms": round(bare_rms, 4)},
        "coarse": coarse_map})
    out = {"x0": cx[0], "dx": cell, "ys": ys, "unit": "um",
           "t": [[int(round(d * 1000.0)) if d == d else None for d in row]
                 for row in tc]}
    return res, out


# --- XY skew from a printed square, without calipers (SKEW_SCAN,
# SKEW_CALIBRATE). A square printed and scanned through the same skewed
# axes scans square. Turning the part ~90 degrees on the plate breaks that:
# any linear error of the measurement (gantry skew, the laser line's angle
# and scale) shifts the angle measured at the front-left corner by the same
# D in both scans, while the physical corner sitting there changes from A to
# its neighbour, 180 - A. So A = (alpha_0 - alpha_90 + 180) / 2.
def skew_plan(box, margin, lo, hi, step, width=28.0, spacing=26.0):
    """Strips and rows over the part's box (x0, y0, x1, y1) + margin."""
    ax0, ay0 = max(box[0] - margin, lo[0] - width / 2), max(box[1] - margin,
                                                         lo[1])
    ax1, ay1 = min(box[2] + margin, hi[0] + width / 2), min(box[3] + margin,
                                                         hi[1])
    if ax1 <= ax0 or ay1 <= ay0:
        return None
    n = max(1, int(math.ceil((ax1 - ax0 - width) / spacing)) + 1)
    c, span = (ax0 + ax1) / 2.0, (n - 1) * spacing
    strips = [round(min(max(c - span / 2.0 + i * spacing, lo[0]), hi[0]), 2)
              for i in range(n)]
    rows = int(math.ceil((ay1 - ay0) / step)) + 1
    return {"strips": strips, "y0": round(ay0, 2),
            "y1": round(ay0 + (rows - 1) * step, 2), "step": step,
            "est": round(n * (rows * 0.23 + 2.5), 1)}


def _levels(vals):
    """Plate and top height of a scan with one raised part: two clusters."""
    v = sorted(vals)
    if len(v) < 100:
        return None
    lo, hi = v[len(v) // 10], v[len(v) * 9 // 10]
    for i in range(10):
        thr = (lo + hi) / 2.0
        a = [x for x in v if x < thr]
        b = [x for x in v if x >= thr]
        if not a or not b:
            return None
        lo, hi = a[len(a) // 2], b[len(b) // 2]
    return lo, hi


def _crossings(pts, thr, max_gap, step=None, near=(0.3, 1.0), ramp=None):
    """pts [(t, h)] sorted by t: where h crosses thr (linear across gaps up
    to max_gap), as [(t, +1 rising / -1 falling)]. With `step`, a crossing
    counts only if the mean heights `near` (from, to) on both sides differ
    by 0.6 step the same way: texture on the plate doesn't. With `ramp`
    (mm), the crossing comes from a line through all points within +-ramp
    that lie on the slope (15-85 % of the step), when there are 3: an edge
    that drifts by less than a row along its length then moves smoothly
    instead of sticking to one pair of rows."""
    out = []
    for i, ((t0, h0), (t1, h1)) in enumerate(zip(pts, pts[1:])):
        if t1 - t0 > max_gap or (h0 - thr) * (h1 - thr) > 0 or h0 == h1:
            continue
        t = t0 + (thr - h0) / (h1 - h0) * (t1 - t0)
        s = 1 if h1 > h0 else -1
        if step is not None:
            a = [h for tt, h in pts[max(0, i - 80):i + 1]
                 if near[0] <= t - tt <= near[1]]
            b = [h for tt, h in pts[i + 1:i + 82]
                 if near[0] <= tt - t <= near[1]]
            if not a or not b or s * (sum(b) / len(b) - sum(a) / len(a)) \
                    < 0.6 * step:
                continue
            if ramp:
                lo, hi = thr - 0.35 * step, thr + 0.35 * step
                sl = [(tt, h) for tt, h in pts[max(0, i - 40):i + 42]
                      if abs(tt - t) <= ramp and lo < h < hi]
                if len(sl) >= 3 and sl[-1][0] - sl[0][0] >= 0.6:
                    k, c = linfit([q[0] for q in sl], [q[1] for q in sl])
                    if k * s > 0:
                        tf = (thr - c) / k
                        if abs(tf - t) <= ramp:
                            t = tf
        out.append((t, s))
    return out


def _robust_lsq(rows, vals, keep_sigma=3.5, rounds=4):
    """Least squares with outliers (> keep_sigma x 1.48 MAD) dropped.
    Returns (coef, residual rms, n used, covariance diagonal)."""
    if not rows:
        return None
    use = list(range(len(rows)))
    for r in range(rounds):
        n = len(rows[0])
        if len(use) <= n + 2:
            return None
        ata = [[sum(rows[k][i] * rows[k][j] for k in use) for j in range(n)]
               for i in range(n)]
        atb = [sum(rows[k][i] * vals[k] for k in use) for i in range(n)]
        try:
            coef = solve(ata, atb)
        except ValueError:
            return None
        res = [vals[k] - sum(c * x for c, x in zip(coef, rows[k]))
               for k in range(len(rows))]
        mad = median([abs(res[k]) for k in use]) * 1.4826
        new = [k for k in range(len(rows))
               if abs(res[k]) <= max(keep_sigma * mad, 1e-4)]
        if new == use:
            break
        use = new
    rms = math.sqrt(sum(res[k] ** 2 for k in use) / max(1, len(use) - n))
    # covariance diagonal: rms^2 * inv(A^T A), one column at a time
    cov = []
    for i in range(n):
        e = [1.0 if j == i else 0.0 for j in range(n)]
        cov.append(solve(ata, e)[i] * rms * rms)
    return coef, rms, len(use), cov


def _rm_line(ts, vs, n_max=150):
    """Repeated-median line v = a + b t (robust to ~50 % outliers)."""
    k = max(1, len(ts) // n_max)
    ts, vs = ts[::k], vs[::k]
    sl = []
    for i in range(len(ts)):
        s = [(vs[j] - vs[i]) / (ts[j] - ts[i]) for j in range(len(ts))
             if abs(ts[j] - ts[i]) > 1.0]
        if s:
            sl.append(median(s))
    if not sl:
        return None
    b = median(sl)
    return median([v - b * t for t, v in zip(ts, vs)]), b


def profile_tilt(strips, base=40):
    """The laser line's own tilt (mm of height per mm of profile x, the
    LiDAR's mounting: ~-0.04, 1.2mm across the line): per row the median
    slope over bin pairs `base` bins apart (pairs across a step are few),
    then the median over all rows."""
    sl = []
    for sx, ys, rows in strips:
        for r in rows:
            v = [(-r[b + base] + r[b]) / (base * BIN)
                 for b in range(0, NBINS - base, 4)
                 if r[b] == r[b] and r[b + base] == r[b + base]]
            if len(v) > 20:
                sl.append(median(v))
    return median(sl) if sl else 0.0


def _skew_diamond(ys_pts, corner, min_pts=20):
    """The part lying at ~45 degrees: all 4 sides are crossed along the
    laser line (sharp, every row), none along the rows. ys_pts: profile
    crossings {+1 rising (left boundary), -1 falling (right boundary)}.
    Returns the result or None if the part isn't lying diagonally."""
    sides = {}
    for s, (lo_key, hi_key) in ((1, ("fl", "bl")), (-1, ("fr", "br"))):
        pts = sorted(ys_pts[s], key=lambda p: p[1])
        if len(pts) < 4 * min_pts:
            return None
        h = len(pts) // 2
        a = _rm_line([p[1] for p in pts[:h]], [p[0] for p in pts[:h]])
        b = _rm_line([p[1] for p in pts[h:]], [p[0] for p in pts[h:]])
        if a is None or b is None or abs(a[1]) < 0.4 or abs(b[1]) < 0.4 \
                or a[1] * b[1] > 0:
            return None
        sides[lo_key], sides[hi_key] = a, b

    def meet(l1, l2):
        # x = a1 + b1 y = a2 + b2 y
        y = (l2[0] - l1[0]) / (l1[1] - l2[1])
        return (l1[0] + l1[1] * y, y)

    fits = {}
    for it in range(2):
        cs = {"left": meet(sides["fl"], sides["bl"]),
              "right": meet(sides["fr"], sides["br"]),
              "front": meet(sides["fl"], sides["fr"]),
              "back": meet(sides["bl"], sides["br"])}
        for key, s in (("fl", 1), ("bl", 1), ("fr", -1), ("br", -1)):
            ln = sides[key]
            p = [(x, y) for x, y in ys_pts[s]
                 if abs(x - (ln[0] + ln[1] * y)) <= 1.0
                 and all(math.hypot(x - cx, y - cy) > corner + 1.0
                         for cx, cy in cs.values())]
            f = _robust_lsq([[1.0, y] for x, y in p], [x for x, y in p])
            if f is None or f[2] < min_pts:
                return None
            fits[key] = f
            sides[key] = (f[0][0], f[0][1])
    deg = math.degrees

    def theta(key):
        b = fits[key][0][1]
        return deg(math.atan2(1.0, b)), deg(math.sqrt(max(fits[key][3][1],
                                                           0.0))) / (1 + b * b)
    t = dict((k, theta(k)) for k in fits)
    # interior angle at the front corner: from the front-right side to the
    # front-left one; opposite sides averaged
    tp = (t["fl"][0] + t["br"][0]) / 2.0
    tq = (t["fr"][0] + t["bl"][0]) / 2.0
    alpha = tp - tq
    sigma = 0.5 * math.sqrt(sum(v[1] ** 2 for v in t.values()))

    def dist(l1, l2):
        return abs(l1[0] - l2[0]) / math.sqrt(1.0 + ((l1[1] + l2[1]) / 2.0)
                                              ** 2)
    cs = {"left": meet(sides["fl"], sides["bl"]),
          "right": meet(sides["fr"], sides["br"]),
          "front": meet(sides["fl"], sides["fr"]),
          "back": meet(sides["bl"], sides["br"])}
    return {"ok": True, "mode": "diamond", "alpha": round(alpha, 5),
            "sigma": round(sigma, 5), "phi": round(tq, 4),
            "sides_deg": dict((k, round(v[0], 4)) for k, v in t.items()),
            "parallel_deg": [round(t["fl"][0] - t["br"][0], 4),
                             round(t["fr"][0] - t["bl"][0], 4)],
            "size": [round(dist(sides["fr"], sides["bl"]), 3),
                     round(dist(sides["fl"], sides["br"]), 3)],
            "corners": dict((k, [round(v[0], 3), round(v[1], 3)])
                            for k, v in cs.items()),
            "points": dict((k, f[2]) for k, f in fits.items()),
            "rms": dict((k, round(f[1], 4)) for k, f in fits.items()),
            "in_strip_slope_deg": 0.0}


def skew_measure(strips, x_dir=-1.0, corner=3.0, max_gap=3.0, band=2.5):
    """Edges of one raised rectangle in a dwell scan -> its corner angles.

    strips: [(laser x, ys, rows of NBINS distances)], all at one nozzle Z
    (heights = -distance). Sides along Y come from the crossings along the
    profile (sharp, 0.05mm bins); sides along X from the crossings along
    the rows of 0.5mm column groups (the laser sheet blurs them over ~1.8mm
    and shadows one side: a constant shift, harmless for their direction).
    The line's mounting angle tilts X-sides inside each strip (a sawtooth
    across strips): fitted as a common in-strip slope. Angles in degrees;
    alpha = front-left corner (front side +X, left side +Y), the mean of it
    and the back-right one."""
    # heights with the line's tilt taken out
    k = profile_tilt(strips)
    strips = [(sx, ys, [[d + k * bin_center(b) if d == d else d
                         for b, d in enumerate(r)] for r in rows])
              for sx, ys, rows in strips]
    vals = [-d for x, ys, rows in strips for r in rows for d in r if d == d]
    lv = _levels(vals)
    if lv is None or lv[1] - lv[0] < 0.12:
        return {"ok": False, "reason": "no raised part found (plate and top"
                " less than 0.12mm apart)"}
    plate, top = lv
    thr = (plate + top) / 2.0
    ys_pts = {1: [], -1: []}     # sides along Y: (x, y) rising/falling in X
    xs_pts = {1: [], -1: []}     # sides along X: (x, y, strip x)
    for sx, ys, rows in strips:
        for y, r in zip(ys, rows):
            pts = [(sx + x_dir * bin_center(b), -d) for b, d in enumerate(r)
                   if d == d and abs(bin_center(b)) < 13.5]
            pts.sort()
            # running median over 7 bins: a spurious reading on the top or
            # the plate would make a false pair of crossings; a real step
            # stays where it is
            hs = [p[1] for p in pts]
            pts = [(pts[i][0], median(hs[max(0, i - 3):i + 4]))
                   for i in range(len(pts))]
            # (diagonal sides facing -Y have the laser's shadow next to the
            # step: look past it)
            for t, s in _crossings(pts, thr, 2.5, top - plate, (0.3, 2.5),
                                   ramp=2.0):
                ys_pts[s].append((t, y))
        grows = [group_row(r, spike=1e9) for r in rows]
        for g in range(NGROUPS):
            gx = sx + x_dir * (BIN_MIN + (g + 0.5) * GROUP * BIN)
            if abs(BIN_MIN + (g + 0.5) * GROUP * BIN) > 13.5:
                continue
            col = [(y, -gr[g]) for y, gr in zip(ys, grows) if gr[g] == gr[g]]
            for t, s in _crossings(col, thr, max_gap, top - plate,
                                   (1.0, 3.5), ramp=2.0):
                xs_pts[s].append((gx, t, sx))
    dm = _skew_diamond(ys_pts, corner)
    if dm is not None:
        dm.update({"plate": round(plate, 4), "top": round(top, 4),
                   "profile_tilt": round(k, 5)})
        return dm
    if min(len(v) for v in list(ys_pts.values()) + list(xs_pts.values())) \
            < 20:
        return {"ok": False, "reason": "not all 4 sides seen (%d/%d/%d/%d"
                " points); is the whole part inside the scan?" % (
                    len(xs_pts[1]), len(ys_pts[-1]), len(xs_pts[-1]),
                    len(ys_pts[1]))}

    # first guess of every side, robust: the rows on the X-sides sit in the
    # laser's blur and cross the threshold anywhere along them
    rough = {}
    for key, pts, tv in (("left", ys_pts[1], lambda p: (p[1], p[0])),
                         ("right", ys_pts[-1], lambda p: (p[1], p[0])),
                         ("front", xs_pts[1], lambda p: (p[0], p[1])),
                         ("back", xs_pts[-1], lambda p: (p[0], p[1]))):
        tvs = [tv(p) for p in pts]
        # around the densest 2mm band of positions (a part turned by up to
        # ~5 degrees still fits in +-6mm over its length)
        cnt = {}
        for t, v in tvs:
            cnt[int(v // 2.0)] = cnt.get(int(v // 2.0), 0) + 1
        if cnt:
            m = (max(cnt, key=lambda c: (cnt[c], -c)) + 0.5) * 2.0
            tvs = [(t, v) for t, v in tvs if abs(v - m) <= 6.0]
        ln = _rm_line([t for t, v in tvs], [v for t, v in tvs])
        if ln is None:
            return {"ok": False, "reason": "no %s side found" % (key,)}
        rough[key] = ln

    def near(ln, t, v, d):
        return abs(v - (ln[0] + ln[1] * t)) <= d

    def fit(corners):
        # left/right: x = a + b y; front/back: y = a + b x + g (x - strip x)
        out = {}
        for key, s in (("left", 1), ("right", -1)):
            p = [(x, y) for x, y in ys_pts[s] if _clear(x, y, corners)
                 and near(rough[key], y, x, 1.0)
                 and not near(rough["front"], x, y, band)
                 and not near(rough["back"], x, y, band)]
            f = _robust_lsq([[1.0, y] for x, y in p], [x for x, y in p])
            if f is None:
                return None
            out[key] = (f[0], f[1], f[2], f[3][1])
        p = [(x, y, sx, s) for s in (1, -1) for x, y, sx in xs_pts[s]
             if _clear(x, y, corners)
             and near(rough["front" if s == 1 else "back"], x, y, 1.0)
             and not near(rough["left"], y, x, band)
             and not near(rough["right"], y, x, band)]
        # (the line's mounting angle tilts these sides inside each strip,
        # a sawtooth across strips; fitting it as a common slope traded off
        # against the sides' own slopes once the part is turned a little,
        # and it is the same in both scans of a part lying in one place, so
        # it cancels in SKEW_CALIBRATE)
        for key, s in (("front", 1), ("back", -1)):
            q = [(x, y) for x, y, sx, ss in p if ss == s]
            f = _robust_lsq([[1.0, x] for x, y in q], [y for x, y in q])
            if f is None:
                return None
            out[key] = (f[0], f[1], f[2], f[3][1])
        # what is left of the in-strip slope, for the report
        res = [y - (out["front" if ss == 1 else "back"][0][0]
                    + out["front" if ss == 1 else "back"][0][1] * x)
               for x, y, sx, ss in p]
        dx = [x - sx for x, y, sx, ss in p]
        g = linfit(dx, res)
        out["in_strip_slope"] = g[0] if g else 0.0
        return out

    def corners_of(fl):
        def meet(v, h):
            # x = a + b y and y = c + d x
            a, b = v[0]
            c, d = h[0]
            y = (c + d * a) / (1.0 - d * b)
            return (a + b * y, y)
        return [meet(fl["left"], fl["front"]), meet(fl["right"], fl["front"]),
                meet(fl["right"], fl["back"]), meet(fl["left"], fl["back"])]

    def _clear(x, y, corners):
        return corners is None or all(
            math.hypot(x - cx, y - cy) > corner for cx, cy in corners)

    fl = fit(None)
    if fl is None:
        return {"ok": False, "reason": "edge fit failed"}
    cs = corners_of(fl)
    fl = fit(cs)
    if fl is None:
        return {"ok": False, "reason": "edge fit failed"}
    cs = corners_of(fl)
    deg = math.degrees
    tF = deg(math.atan(fl["front"][0][1]))
    tB = deg(math.atan(fl["back"][0][1]))
    tL = 90.0 - deg(math.atan(fl["left"][0][1]))
    tR = 90.0 - deg(math.atan(fl["right"][0][1]))
    # slope sigma (rad) -> direction sigma (deg)
    sd = dict((k, deg(math.sqrt(max(fl[k][3], 0.0))))
              for k in ("front", "back", "left", "right"))
    a_fl, a_br = tL - tF, tR - tB
    alpha = (a_fl + a_br) / 2.0
    sigma = 0.5 * math.sqrt(sum(v * v for v in sd.values()))
    mid_y = (cs[0][1] + cs[3][1] + cs[1][1] + cs[2][1]) / 4.0
    mid_x = (cs[0][0] + cs[1][0] + cs[2][0] + cs[3][0]) / 4.0
    lx = (fl["right"][0][0] + fl["right"][0][1] * mid_y) - \
        (fl["left"][0][0] + fl["left"][0][1] * mid_y)
    ly = (fl["back"][0][0] + fl["back"][0][1] * mid_x) - \
        (fl["front"][0][0] + fl["front"][0][1] * mid_x)
    return {"ok": True, "mode": "axis", "alpha": round(alpha, 5),
            "sigma": round(sigma, 5),
            "corners_deg": [round(a_fl, 4), round(180.0 - (tR - tF), 4),
                            round(a_br, 4), round(180.0 - (tL - tB), 4)],
            "phi": round((tF + tB) / 2.0, 4),
            "sides_deg": {"front": round(tF, 4), "back": round(tB, 4),
                          "left": round(tL, 4), "right": round(tR, 4)},
            "parallel_deg": [round(tF - tB, 4), round(tL - tR, 4)],
            "size": [round(lx, 3), round(ly, 3)],
            "corners": [[round(x, 3), round(y, 3)] for x, y in cs],
            "points": dict((k, fl[k][2]) for k in
                           ("front", "back", "left", "right")),
            "rms": dict((k, round(fl[k][1], 4)) for k in
                        ("front", "back", "left", "right")),
            "in_strip_slope_deg": round(deg(math.atan(fl["in_strip_slope"])),
                                        4),
            "plate": round(plate, 4), "top": round(top, 4),
            "profile_tilt": round(k, 5)}


def skew_solve(scans, printed_skew=0.0, length=100.0):
    """scans: [(rot_deg, skew_measure result)], the part turned by about
    rot (0, 90, 180, 270) between scans. Returns the true front-left angle
    A of the part as printed, the printer's XY skew (deg, = 90 - A plus
    printed_skew: the skew correction that was active when the part was
    printed) and Klipper's SET_SKEW lengths for a `length` square."""
    eq = []
    rot0, phi0 = scans[0][0], scans[0][1]["phi"]
    diamond = all(m.get("mode") == "diamond" for r, m in scans)
    for rot, m in scans:
        # the corner sitting at the tracked place changes from A to its
        # neighbour with every quarter turn (counted from the first scan)
        odd = int(round((rot - rot0) / 90.0)) % 2
        # alpha = (A or 180 - A) + D0 + D1 * (phi - phi_0)
        eq.append(([(-1.0 if odd else 1.0), 1.0, m["phi"] - phi0],
                   m["alpha"] - (180.0 if odd else 0.0), m["sigma"], odd))
    if len(set(e[3] for e in eq)) < 2:
        return {"ok": False, "reason": "needs scans with the part turned by"
                " about 90 degrees between them (ROT=0 and ROT=90)"}
    phis = [m["phi"] for r, m in scans]
    use_d1 = len(eq) >= 3 and max(phis) - min(phis) > 3.0
    nc = 3 if use_d1 else 2
    w = [1.0 / max(e[2], 0.003) ** 2 for e in eq]
    ata = [[sum(wi * e[0][i] * e[0][j] for wi, e in zip(w, eq))
            for j in range(nc)] for i in range(nc)]
    atb = [sum(wi * e[0][i] * e[1] for wi, e in zip(w, eq))
           for i in range(nc)]
    coef = solve(ata, atb)
    A = coef[0]
    inv0 = solve(ata, [1.0] + [0.0] * (nc - 1))[0]
    sig = math.sqrt(inv0)
    res = [e[1] - sum(c * x for c, x in zip(coef, e[0][:nc])) for e in eq]
    # a scale difference e between the measured X and Y also shifts alpha,
    # by e * sin(2 phi): it cancels only if both scans have the part at the
    # same phi. X positions come mostly from the strip spacing (the
    # gantry), the line's +-1 % scale only acts within +-14 mm of each
    # strip, so |e| <= ~0.4 %. (The sides' length ratio is reported, but
    # the constant edge shifts of the laser blur dominate it.)
    ev = [m["size"] for r, m in scans]
    aniso = (sum(math.log(s[1]) for s in ev) - sum(math.log(s[0])
                                                    for s in ev)) / len(ev)
    d_phi = max(phis) - min(phis)
    if diamond:
        # at 45 degrees the first-order term is the measurement's shear
        # (gantry skew + the line's mounting angle, <= ~0.01 rad) times the
        # turn's mismatch; no sides along the rows
        aniso_err = 0.01 * math.sin(math.radians(2.0 * d_phi)) / 2.0
        extra = 0.0
    else:
        aniso_err = 0.004 * math.sin(math.radians(2.0 * d_phi)) / 2.0
        # sides along the rows: what the simulation (test_skew.py) shows on
        # top of the fit, ~0.01 deg per degree the turn is off
        extra = 0.01 * d_phi
    sig = math.sqrt(sig * sig + math.degrees(aniso_err) ** 2 + extra ** 2)
    if nc == 2 and len(eq) > 2:
        chi = math.sqrt(sum(r * r for r in res) / (len(eq) - 2))
        sig = max(sig, chi / math.sqrt(len(eq)))
    skew = 90.0 - A + printed_skew
    t = math.radians(90.0 - skew)
    ac = length * math.sqrt(2.0 * (1.0 + math.cos(t)))
    bd = length * math.sqrt(2.0 * (1.0 - math.cos(t)))
    return {"ok": True, "angle": round(A, 4), "skew_deg": round(skew, 4),
            "sigma_deg": round(sig, 4),
            "mm_per_100": round(100.0 * math.tan(math.radians(skew)), 3),
            "factor": math.tan(math.radians(skew)),
            "set_skew": [round(ac, 4), round(bd, 4), round(length, 4)],
            "residuals": [round(r, 4) for r in res],
            "measurement_d": round(coef[1], 4), "aniso": round(aniso, 5),
            "turn_spread_deg": round(d_phi, 3),
            "mode": "diamond" if diamond else "axis"}


def _default_dirs(config):
    """(data_dir, gcode_dir): next to Klipper's virtual_sdcard path
    (printer_data/gcodes -> printer_data/lidar), else the K1 Max ones."""
    if config.has_section("virtual_sdcard"):
        path = config.getsection("virtual_sdcard").get("path", None)
        if path:
            gcodes = os.path.normpath(os.path.expanduser(path))
            return os.path.join(os.path.dirname(gcodes), "lidar"), gcodes
    return "/usr/data/printer_data/lidar", "/usr/data/printer_data/gcodes"


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
        # moving scans: added to the frame window (s); a frame that looked
        # later than LIDAR_TIMING says shows as a Y shift between the two
        # directions (LIDAR_BED_MESH ... BOTH_WAYS=1 measures it)
        self.moving_lag = config.getfloat("moving_lag", 0.0)
        # where meshes and calibrations look (laser coordinates): the
        # travel shifted by the laser offset (_laser_area), narrowed by
        # these if set (K1 Max: 5,5 / 270,265, its keep-out zones end Y)
        self.mesh_min = config.getfloatlist("mesh_min", None, count=2)
        self.mesh_max = config.getfloatlist("mesh_max", None, count=2)
        self.mesh_max_count = config.getint("mesh_max_count", 30, minval=4)
        # calibration lines: LIDAR_TWIST_CALIBRATE AXIS=X at laser Y
        # twist_y, AXIS=Y at laser X ytwist_x (defaults from the area)
        self.twist_y = config.getfloat("twist_y", None)
        self.ytwist_x = config.getfloatlist("ytwist_x", None)
        # nozzle touches (calibrations, level 5, benchmarks), see
        # _touch_mode; touch: auto picks load_cell, else probe, else none
        touch = config.getchoice("touch", dict((m, m) for m in (
            "auto",) + self.TOUCH_MODES), "auto")
        self.touch = None if touch == "auto" else touch
        self.touch_gcode = config.get("touch_gcode", "NOZZLE_TOUCH")
        self.touch_result = config.get("touch_result", None)
        self.touch_temp = config.getfloat("touch_temp", 150.0, minval=0.0)
        self.touch_wipe_gcode = config.get("touch_wipe_gcode", None)
        # what made each mesh profile this session (MESH_COMPARE)
        self.mesh_info = {}
        # point cloud level for dwell scans: 0 all points (~0.02mm apart),
        # 1 every 2nd (~0.04mm, frames ~15% faster)
        self.scan_level = config.getint("scan_level", 0, minval=0, maxval=2)
        self.printer.load_object(config, "motion_report")
        self.mode = config.getchoice("mode", {"dwell": "dwell",
                                              "stop": "stop"}, "dwell")
        data_dir, gcode_dir = _default_dirs(config)
        self.data_dir = config.get("data_dir", data_dir)
        self.gcode_dir = config.get("gcode_dir", gcode_dir)
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
        # first-layer check of the running print (FIRST_LAYER_SCAN)
        self.fl = None
        for cmd in ("LIDAR_STATUS", "LIDAR_SCAN", "LIDAR_PA_ANALYZE",
                    "LIDAR_PA_CALIBRATE", "LIDAR_PA_CREALITY",
                    "LIDAR_OFFSET_CALIBRATE", "LIDAR_TIMING",
                    "LIDAR_BED_MESH", "LIDAR_TWIST_CALIBRATE",
                    "LIDAR_GRID_CALIBRATE", "MESH_COMPARE", "MESH_BENCHMARK",
                    "FIRST_LAYER_SCAN", "SKEW_SCAN", "SKEW_CALIBRATE",
                    "LIDAR_SETUP_CHECK"):
            self.gcode.register_command(
                cmd, getattr(self, "cmd_" + cmd),
                desc=getattr(self, "desc_" + cmd))

    def get_status(self, eventtime):
        return {"last_result": self.last_result,
                "first_layer_armed": bool(self.fl and self.fl.get("armed"))}

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
        """Which driver serves the socket (variables.cfg lidar_driver):
        lidar-direct.py since the middleware was retired (2026-11-01)."""
        sv = self.printer.lookup_object("save_variables", None)
        saved = getattr(sv, "allVariables", None) or {}
        return str(saved.get("lidar_driver", "direct"))

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
            "LiDAR ok (FreeTheBeam %s): exposure %s, offset X%.2f Y%.2f"
            " at Z%.1f, per mm Z"
            " X%+.3f Y%+.3f (%s), x_dir %+d, %s, scans: %s" % (
                VERSION, r.get("result", {}).get("exposure"), self.x_offset,
                self.y_offset, self.offset_z, self.x_per_z, self.y_per_z,
                self.offset_source, self.x_dir,
                "work done by the service at " + self.service
                if self.service else "work done on this printer",
                ", ".join(sorted(self.scans)) or "none"))

    offset_source = "config"

    def setup_report(self):
        """[(item, ok, text)]: what this printer's setup can do."""
        err = (self.printer.command_error, OSError)
        out = []
        msg = {"control": "get_laser_exposure"}
        try:
            r = (self._svc({"cmd": "middleware", "msg": msg})["reply"]
                 if self.service else self._call(msg))
            out.append(("driver", True, "answers (exposure %s)" % (
                r.get("result", {}).get("exposure"),)))
        except err as e:
            out.append(("driver", False, "no answer (%s)" % (e,)))
        measured = self.offset_source != "config"
        out.append(("offsets", measured, "X%.2f Y%.2f, %s" % (
            self.x_offset, self.y_offset, self.offset_source if measured
            else "from [lidar], not measured: LIDAR_OFFSET_CALIBRATION")))
        timing = (self._saved().get("lidar_timing") or {}).get(
            self._driver())
        out.append(("timing", bool(timing), "saved for the %s driver"
                    % (self._driver(),) if timing else
                    "not measured: LIDAR_TIMING (dwell scans need it)"))
        if not self.service:
            out.append(("service", False, "not set: meshes, the first-layer"
                        " check and dwell scans need it"))
        else:
            try:
                self._svc({"cmd": "ping"})
                out.append(("service", True, "answers at " + self.service))
            except err as e:
                out.append(("service", False, "%s: no answer (%s)"
                            % (self.service, e)))
        x0, x1, y0, y1 = self._laser_area(self.scan_z)
        if x1 - x0 < 1.0 or y1 - y0 < 1.0:
            out.append(("area", False, "the LiDAR reaches no part of the bed"
                        " (X%.1f-%.1f Y%.1f-%.1f): check x_offset/y_offset,"
                        " mesh_min/mesh_max" % (x0, x1, y0, y1)))
        else:
            out.append(("area", True, "levels 1-3 X%.1f-%.1f Y%.1f-%.1f,"
                        " levels 4-5 to X%.1f (at Z%.1f)" % (
                            x0, x1, y0, y1, x1 + 14.0, self.scan_z)))
        mode = self._touch_mode()

        class _G:
            error = self.printer.command_error
        try:
            self._touch_ready(_G())
            out.append(("touch", True, mode + (
                ": reference is the probe, less accurate than a nozzle"
                " touch" if mode == "probe" else "")))
        except self.printer.command_error as e:
            out.append(("touch", False, "%s: %s (LIDAR_TWIST_CALIBRATE,"
                        " LIDAR_GRID_CALIBRATE, level 5 and MESH_BENCHMARK"
                        " need it)" % (mode, e)))
        ko = self.printer.lookup_object("keep_out", None)
        sk = self.printer.lookup_object("skew_correction", None)
        out.append(("limits", True, "skew correction %s, keep-out zones %s"
                    % ("on" if sk is not None else "off",
                       "on" if ko is not None else "off")))
        return out

    desc_LIDAR_SETUP_CHECK = "What this printer's LiDAR setup can do"

    def cmd_LIDAR_SETUP_CHECK(self, gcmd):
        rep = self.setup_report()
        lines = ["LiDAR setup (FreeTheBeam %s):" % (VERSION,)]
        lines += ["  [%s] %s: %s" % ("ok" if ok else "--", item, text)
                  for item, ok, text in rep]
        missing = [item for item, ok, text in rep if not ok]
        lines.append("missing: " + ", ".join(missing) if missing
                     else "all set")
        gcmd.respond_info("\n".join(lines))

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

    def _scan_dwell(self, name, x, y0, y1, step, z, level=None, debug=None,
                    path=None, speed=None):
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

        path: [(x, y)] laser positions instead of the strip at x from y0 to
        y1, one frame per point, moving in X and Y between them (sparse bed
        meshes); a missed point stays empty. speed: mm/s between rows.
        """
        toolhead = self.printer.lookup_object("toolhead")
        mcu = self.printer.lookup_object("mcu")
        level = self.scan_level if level is None else level
        period, lo, hi, src = self._timing(level)
        margin, settle = self.dwell_margin, self.dwell_settle
        if path is None:
            n = int(round(abs(y1 - y0) / step))
            d = step if y1 >= y0 else -step
            pts = [(x, round(y0 + i * d, 4)) for i in range(n + 1)]
        else:
            pts = [(round(px, 4), round(py, 4)) for px, py in path]
        ys = [p[1] for p in pts]
        xo, yo = self._xo(z), self._yo(z)
        f = (speed or self.speed) * 60.0
        start = self.reactor.monotonic()
        self.gcode.run_script_from_command(
            "SAVE_GCODE_STATE NAME=_lidar_scan\nG90\nG0 Z%.3f F600\n"
            "G0 X%.3f Y%.3f F%.0f" % (z, pts[0][0] - xo, ys[0] - yo,
                                      self.speed * 60.0))
        toolhead.wait_moves()
        req = {"cmd": "begin", "name": name, "x": x, "z": z,
               "x_dir": self.x_dir, "x_offset": xo, "y_offset": yo,
               "captures": 1, "level": level}
        if path is None:
            req["grid"] = [y0, y1, step]
        else:
            req["path"] = pts
        self._svc(req)
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
                        "G0 X%.3f Y%.3f F%.0f" % (pts[i][0] - xo,
                                                  ys[i] - yo, f))
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
                            "rows": [], "path": path and pts}
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

    # ------------------------------------------------------------- bed mesh
    # [lidar] options; class defaults too, for objects built without
    # __init__ (tests)
    mesh_min = mesh_max = twist_y = ytwist_x = None
    mesh_max_count = 30   # 30x30: ~0.5s to build in Klipper on the K1

    def _laser_area(self, z):
        """Where the laser can look with the nozzle at height z (laser X/Y
        = mesh coordinates): the part of the travel the nozzle also covers
        shifted by the laser offset, 0.5mm inside, never below 0, narrowed
        by mesh_min/mesh_max. (x0, x1, y0, y1); empty if x1 <= x0 or
        y1 <= y0. Skew and keep-out zones are checked per move
        (_reach_x, _check_reach)."""
        st = self.printer.lookup_object("toolhead").get_status(
            self.reactor.monotonic())
        lo, hi = st["axis_minimum"], st["axis_maximum"]
        off = (self._xo(z), self._yo(z))
        area = []
        for i in (0, 1):
            a = max(max(lo[i], lo[i] + off[i]) + 0.5, 0.0)
            b = min(hi[i], hi[i] + off[i]) - 0.5
            if self.mesh_min is not None:
                a = max(a, self.mesh_min[i])
            if self.mesh_max is not None:
                b = min(b, self.mesh_max[i])
            area += [round(a, 3), round(b, 3)]
        return tuple(area)

    def _area_or_error(self, gcmd, z):
        x0, x1, y0, y1 = area = self._laser_area(z)
        if x1 - x0 < 1.0 or y1 - y0 < 1.0:
            raise gcmd.error(
                "LiDAR: the LiDAR reaches no part of the bed (laser area"
                " X%.1f-%.1f Y%.1f-%.1f at Z%.1f): check x_offset/y_offset"
                " and mesh_min/mesh_max in [lidar]" % (x0, x1, y0, y1, z))
        return area

    def _twist_spots(self, gcmd, axis, z):
        """LIDAR_TWIST_CALIBRATE's spots (laser = nozzle touch coordinates):
        AXIS=X a line along X at twist_y (default: the area's Y centre),
        AXIS=Y lines along Y at ytwist_x (default: 1/3 and 2/3 of the
        area's X)."""
        ax0, ax1, ay0, ay1 = self._area_or_error(gcmd, z)
        if axis == "X":
            n = gcmd.get_int("POINTS", 7, minval=3, maxval=15)
            y = gcmd.get_float("Y", self.twist_y if self.twist_y is not None
                               else round((ay0 + ay1) / 2.0, 1))
            x0 = gcmd.get_float("X0", ax0)
            x1 = gcmd.get_float("X1", min(ax1, self._reach_x(
                y - 1.0, y + 1.0, z, 0.3)[1]))
            return [(x, y) for x in self._grid(x0, x1, n)]
        # the Y error is small (~10 um/100mm) next to the texture under
        # each spot (+-20 um between a nozzle touch and the laser's view of
        # the same spot, the same in every pass): only many spots help, so
        # two lines and a straight-line fit
        lx = self.ytwist_x or [round(ax0 + (ax1 - ax0) * f, 1)
                               for f in (1.0 / 3.0, 2.0 / 3.0)]
        if gcmd.get("X", None) is not None:
            try:
                lx = [float(v) for v in gcmd.get("X").split(",")]
            except ValueError:
                raise gcmd.error("LiDAR: X is one value or a list (X=100,200)")
        n = gcmd.get_int("POINTS", 9, minval=3, maxval=15)
        y0 = gcmd.get_float("Y0", ay0 + 1.0)
        y1 = gcmd.get_float("Y1", ay1 - 1.0)
        spots = []
        for i, x in enumerate(lx):
            ys = self._grid(y0, y1, n)
            spots += [(x, y) for y in (ys if i % 2 == 0 else ys[::-1])]
        return spots

    def _grid_axes(self, gcmd, z, nx, ny):
        """LIDAR_GRID_CALIBRATE's columns and rows: the area, 5mm in from
        its left edge and 1mm in along Y."""
        ax0, ax1, ay0, ay1 = self._area_or_error(gcmd, z)
        y0 = gcmd.get_float("Y0", ay0 + 1.0)
        y1 = gcmd.get_float("Y1", ay1 - 1.0)
        x0 = gcmd.get_float("X0", ax0 + 5.0)
        x1 = gcmd.get_float("X1", min(ax1, self._reach_x(
            y0 - 1.0, y1 + 1.0, z, 0.3)[1]))
        return self._grid(x0, x1, nx), self._grid(y0, y1, ny)

    # ------------------------------------------------------- nozzle touches
    # load_cell: Kalico's [load_cell_probe] (the K1's load cells); probe:
    # any [probe], moved so the probe is over the spot (the reference is
    # then the probe, not the nozzle: less accurate); gcode: touch_gcode
    # with the nozzle over the spot, contact Z read from the status
    # "object:key" in touch_result; none: no touches
    TOUCH_MODES = ("load_cell", "probe", "gcode", "none")
    touch = touch_result = touch_wipe_gcode = None
    touch_gcode = "NOZZLE_TOUCH"
    touch_temp = 150.0

    def _touch_mode(self):
        if self.touch:
            return self.touch
        if self.printer.lookup_object("load_cell_probe", None) is not None:
            return "load_cell"
        if self.printer.lookup_object("probe", None) is not None:
            return "probe"
        return "none"

    def _touch_error(self, msg):
        return self.printer.command_error("LiDAR: " + msg)

    def _touch_ready(self, gcmd):
        """Refuse before anything moves if nozzle touches can't work."""
        mode = self._touch_mode()
        if mode == "none":
            raise gcmd.error(
                "LiDAR: this needs a touch method ([lidar] touch:):"
                " load_cell, probe or gcode")
        if mode == "load_cell" and self.printer.lookup_object(
                "load_cell_probe", None) is None:
            raise gcmd.error("LiDAR: touch: load_cell but no"
                             " [load_cell_probe]")
        if mode == "probe" and self.printer.lookup_object(
                "probe", None) is None:
            raise gcmd.error("LiDAR: touch: probe but no [probe]")
        if mode == "gcode":
            self._touch_result_object()

    def _touch_result_object(self):
        """(object, key) named by touch_result ("object:key")."""
        name, _, key = (self.touch_result or "").rpartition(":")
        obj = self.printer.lookup_object(name, None) if name else None
        if obj is None or not key:
            raise self._touch_error(
                "touch: gcode needs touch_result: <object>:<key> naming a"
                " status value that holds the contact Z (got %r)"
                % (self.touch_result,))
        return obj, key

    def _touch_xy(self, x, y):
        """Toolhead X/Y that puts the touching part over spot (x, y)."""
        if self._touch_mode() == "probe":
            off = self.printer.lookup_object("probe").get_offsets()
            return x - off[0], y - off[1]
        return x, y

    def _touch(self, samples):
        """Touch at the current position; the contact Z (median of
        samples)."""
        mode = self._touch_mode()
        run = self.gcode.run_script_from_command
        if mode == "load_cell":
            run("LOAD_CELL_PROBE SAMPLES=%d SAMPLES_RESULT=median" % (
                samples,))
            return self.printer.lookup_object(
                "load_cell_probe").get_printer_probe().last_z_result
        if mode == "probe":
            run("PROBE SAMPLES=%d SAMPLES_RESULT=median" % (samples,))
            return self.printer.lookup_object("probe").get_status(
                self.reactor.monotonic())["last_z_result"]
        if mode == "gcode":
            obj, key = self._touch_result_object()
            zs = []
            for i in range(samples):
                run(self.touch_gcode)
                v = obj.get_status(self.reactor.monotonic()).get(key)
                if v is None:
                    raise self._touch_error(
                        "touch_result %r: no value after %s"
                        % (self.touch_result, self.touch_gcode))
                zs.append(float(v))
            return median(zs)
        raise self._touch_error("this needs a touch method ([lidar] touch:):"
                                " load_cell, probe or gcode")

    def _touch_heat(self, wait, wipe=True):
        """Nozzle to touch_temp (M109 with wait), then touch_wipe_gcode."""
        run = self.gcode.run_script_from_command
        run("%s S%.0f" % ("M109" if wait else "M104", self.touch_temp))
        if wait and wipe and self.touch_wipe_gcode:
            run(self.touch_wipe_gcode)

    def _grid(self, a, b, n):
        return [round(a + (b - a) * i / (n - 1), 3) for i in range(n)]

    def _skewed(self, p):
        """Where the skew correction (SKEW_PROFILE / SET_SKEW) really sends
        the nozzle for G-code position p [x, y, z]: Klipper checks the
        travel limits after it (at the back -0.2 deg adds ~1mm in X)."""
        sk = self.printer.lookup_object("skew_correction", None)
        if sk is None:
            return list(p)
        return sk.calc_skew(list(p) + [0.0])[:3]

    def _unskewed(self, p):
        """G-code position of toolhead (trapq) position p: the trapq has
        the skew-corrected one, the scans are planned in G-code."""
        sk = self.printer.lookup_object("skew_correction", None)
        if sk is None:
            return list(p)
        return sk.calc_unskew(list(p) + [0.0])[:3]

    def _reach_x(self, y0, y1, z, margin=0.5):
        """Laser X range the nozzle reaches at height z over laser Y y0-y1,
        skew correction included (it shifts X along Y, so the far end of
        the range decides)."""
        st = self.printer.lookup_object("toolhead").get_status(
            self.reactor.monotonic())
        lo, hi = st["axis_minimum"][0], st["axis_maximum"][0]
        xo, yo = self._xo(z), self._yo(z)
        dx = [self._skewed([0.0, y - yo, z])[0] for y in (y0, y1)]
        return lo + xo - min(dx) + margin, hi + xo - max(dx) - margin

    def _check_reach(self, gcmd, segs, z=None):
        """segs: (x0, y0, x1, y1) lines the toolhead will follow, nozzle
        coordinates, or laser ones with the nozzle at z: refuse before
        anything moves if the nozzle would leave its travel or cross a
        keep-out zone."""
        toolhead = self.printer.lookup_object("toolhead")
        st = toolhead.get_status(self.reactor.monotonic())
        if "xyz" not in st["homed_axes"]:
            raise gcmd.error("LiDAR: home first (G28)")
        lo, hi = st["axis_minimum"], st["axis_maximum"]
        ko = self.printer.lookup_object("keep_out", None)
        xo, yo = (0.0, 0.0) if z is None else (self._xo(z), self._yo(z))
        nz = 0.0 if z is None else z
        for x0, y0, x1, y1 in segs:
            a, b = [x0 - xo, y0 - yo, 0.0], [x1 - xo, y1 - yo, 0.0]
            for p in (a, b):
                q = self._skewed([p[0], p[1], nz])
                if not (lo[0] <= q[0] <= hi[0] and lo[1] <= q[1] <= hi[1]):
                    raise gcmd.error(
                        "LiDAR: the nozzle would go to X%.1f Y%.1f%s, outside"
                        " its travel" % (p[0], p[1], "" if abs(
                            q[0] - p[0]) + abs(q[1] - p[1]) < 0.001 else
                            " (X%.1f Y%.1f with the skew correction)" % (
                                q[0], q[1])))
            if ko is not None and ko._blocked(a, b):
                raise gcmd.error(
                    "LiDAR: the nozzle would cross a keep-out zone (X%.1f"
                    " Y%.1f to X%.1f Y%.1f)" % (a[0], a[1], b[0], b[1]))

    def _saved(self):
        sv = self.printer.lookup_object("save_variables", None)
        return getattr(sv, "allVariables", None) or {}

    def _bed_temp_for_tables(self):
        """The bed temperature the tables are chosen for: its target, or
        the bed's own temperature when it isn't heated."""
        bed = self.printer.lookup_object("heater_bed", None)
        if bed is None:
            return 25.0
        st = bed.get_status(self.reactor.monotonic())
        return st.get("target") or st.get("temperature", 25.0)

    def _lidar_tables(self):
        """(X twist, Y table, note) for the current bed temperature: from
        the calibrations per temperature (lidar_cal_by_temp) when there are
        any, else the last calibration (lidar_twist, lidar_ytwist)."""
        sv = self._saved()
        store = sv.get("lidar_cal_by_temp")
        if store:
            t = self._bed_temp_for_tables()
            tw, yt, note = tables_for_temp(store, t)
            tw = tw or sv.get("lidar_twist")
            yt = yt or sv.get("lidar_ytwist")
            return tw, yt, "bed %.0fC: %s" % (t, note)
        return sv.get("lidar_twist"), sv.get("lidar_ytwist"), ""

    def _store_by_temp(self, tables):
        """Keep tables {"twist": ..., "ytwist": ...} (either, with their
        weight "w") as one more run under the bed temperature they were
        measured at (lidar_cal_by_temp; 5C steps, the last CAL_RUNS runs);
        meshes average the runs (tables_for_temp)."""
        t = int(5 * round(self._bed_temp_for_tables() / 5.0))
        store = dict(self._saved().get("lidar_cal_by_temp") or {})
        runs = cal_runs(store.get(str(t)) or {"runs": []})
        run = dict(tables, time=time.strftime("%Y-%m-%dT%H:%M"))
        runs = (runs + [run])[-CAL_RUNS:]
        store[str(t)] = {"runs": runs}
        self.gcode.run_script_from_command(
            'SAVE_VARIABLE VARIABLE=lidar_cal_by_temp VALUE="%s"' % (
                repr(store).replace(" ", ""),))
        return t

    def _temps(self):
        now = self.reactor.monotonic()
        toolhead = self.printer.lookup_object("toolhead")
        bed = self.printer.lookup_object("heater_bed", None)
        return (round(toolhead.get_extruder().get_heater().get_status(
            now)["temperature"], 1),
            round(bed.get_status(now)["temperature"], 1) if bed else None)

    # the five levels: (name, grid, continuous speed mm/s, dwell grid,
    # frames per point, dwell speed); 4 and 5 scan whole strips. Times and
    # accuracy: k1-lidar/bedmesh/study_levels.py
    LEVELS = {
        1: ("quick", 7, 150.0, 7, 1, 300.0),
        2: ("fast", 12, 120.0, 12, 1, 200.0),
        3: ("normal", 20, 60.0, 15, 2, 200.0),
        4: ("fine", 30, None, None, None, None),
        5: ("ultimate", 30, None, None, None, None),
    }
    LEVEL_NAMES = {"quick": 1, "fast": 2, "normal": 3, "fine": 4,
                   "ultimate": 5}

    desc_LIDAR_BED_MESH = ("Bed mesh with the LiDAR, LEVEL=1-5 (quick, fast,"
                           " normal, fine, ultimate)")

    def cmd_LIDAR_BED_MESH(self, gcmd):
        """FALLBACK=1 (START_PRINT): the probe's mesh instead of an error
        when the LiDAR can't do it (service down, objects out of its reach,
        a failed scan), so the print goes on."""
        if not gcmd.get_int("FALLBACK", 0):
            self._bed_mesh_cmd(gcmd)
            return
        try:
            self._bed_mesh_cmd(gcmd)
        except self.printer.command_error as e:
            adaptive = gcmd.get_int("ADAPTIVE", 0)
            gcmd.respond_info("%s: probe mesh instead (ADAPTIVE=%d)" % (
                e, adaptive))
            self.gcode.run_script_from_command(
                "BED_MESH_CALIBRATE ADAPTIVE=%d" % (adaptive,))

    def _bed_mesh_cmd(self, gcmd):
        if not self.service:
            raise gcmd.error("LiDAR: LIDAR_BED_MESH needs the lidar service")
        mode = gcmd.get("MODE", None)
        if mode is not None:
            if mode.lower() not in self.LEVEL_NAMES:
                raise gcmd.error("LiDAR: MODE is one of %s" % (
                    ", ".join(sorted(self.LEVEL_NAMES)),))
            level = self.LEVEL_NAMES[mode.lower()]
        else:
            level = gcmd.get_int("LEVEL", 2, minval=1, maxval=5)
        o = {"level": level, "profile": gcmd.get("PROFILE", "lidar"),
             "motion": gcmd.get("MOTION", "continuous").lower(),
             "z": gcmd.get_float("Z", self.scan_z),
             "half_x": gcmd.get_float("HALF_X", 3.0, above=0.2, maxval=10.0),
             "twist": gcmd.get_int("TWIST", 1),
             "adaptive": gcmd.get_int("ADAPTIVE", 0),
             "fallback": gcmd.get_int("FALLBACK", 0)}
        for k in ("X0", "X1", "Y0", "Y1", "SPEED", "STEP", "HALF_Y",
                  "STRIP_X0", "STRIP_X1"):
            v = gcmd.get_float(k, None)
            if v is not None:
                o[k.lower()] = v
        for k in ("COUNT_X", "COUNT_Y", "FRAMES", "STRIPS", "TOUCHES",
                  "WIPE", "BED_OFF", "BOTH_WAYS", "SAMPLES", "MAX_ORDER"):
            v = gcmd.get_int(k, None)
            if v is not None:
                o[k.lower()] = v
        if o["motion"] not in ("continuous", "dwell"):
            raise gcmd.error("LiDAR: MOTION is continuous or dwell")
        self._mesh(gcmd, o)

    def _area(self, gcmd, o, fine):
        """Mesh area (laser = mesh coordinates): given, the printed objects
        (ADAPTIVE=1, like the probe's adaptive mesh) or all the LiDAR
        reaches."""
        ax0, ax1, y0, y1 = self._area_or_error(gcmd, o["z"])
        # strips (fine) look 14mm either side of their centre line
        x0, x1 = (ax0 - 1.0, ax1 + 14.0) if fine else (ax0, ax1)
        # strips (fine) reach 14mm past their centre line
        rx0, rx1 = self._reach_x(y0, y1, o["z"], 0.3)
        w = 14.0 if fine else 0.0
        x0, x1 = max(x0, rx0 - w), min(x1, rx1 + w)
        if o.get("adaptive"):
            eo = self.printer.lookup_object("exclude_object", None)
            objs = eo.get_status(self.reactor.monotonic()).get(
                "objects", []) if eo is not None else []
            pts = [p for ob in objs for p in (ob.get("polygon") or [])]
            if pts:
                m = 5.0
                bx0, bx1 = min(p[0] for p in pts), max(p[0] for p in pts)
                by0, by1 = min(p[1] for p in pts), max(p[1] for p in pts)
                if bx0 < x0 - 1.0 or bx1 > x1 + 1.0 or by0 < y0 - 1.0 \
                        or by1 > y1 + 1.0:
                    msg = ("the objects (X%.0f-%.0f Y%.0f-%.0f) reach past"
                           " what the LiDAR sees (X%.0f-%.0f Y%.0f-%.0f)" % (
                               bx0, bx1, by0, by1, x0, x1, y0, y1))
                    if o.get("fallback"):
                        raise gcmd.error("LiDAR mesh: " + msg)
                    gcmd.respond_info("LiDAR mesh: %s, the mesh keeps its"
                                      " edge values there" % (msg,))
                # the LiDAR has no absolute height: the mesh is made 0 at
                # zero_reference_position (where Z is homed and touched),
                # so that spot has to be in it
                ref = self.printer.lookup_object("bed_mesh").bmc.zero_ref_pos
                if ref is not None:
                    bx0, bx1 = min(bx0, ref[0]), max(bx1, ref[0])
                    by0, by1 = min(by0, ref[1]), max(by1, ref[1])
                x0 = max(x0, bx0 - m)
                x1 = min(x1, bx1 + m)
                y0 = max(y0, by0 - m)
                y1 = min(y1, by1 + m)
            else:
                gcmd.respond_info("LiDAR mesh: no objects defined, whole bed")
        x0, x1 = o.get("x0", x0), o.get("x1", x1)
        y0, y1 = o.get("y0", y0), o.get("y1", y1)
        if x1 - x0 < 20.0 or y1 - y0 < 20.0:
            # bicubic needs 4 points a side; keep at least 20mm
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            x0, x1 = max(cx - 10.0, ax0 - 1.0 if fine else ax0), \
                max(cx + 10.0, x0 + 20.0)
            y0, y1 = cy - 10.0, cy + 10.0
        return x0, x1, y0, y1

    def _counts(self, o, n, x0, x1, y0, y1, full_x, full_y):
        # the level's spacing over the whole bed, fewer points on a smaller
        # area (4 to mesh_max_count)
        def cnt(size, full):
            return max(4, min(n, int(round(size / (full / (n - 1)))) + 1))
        nx = o.get("count_x", cnt(x1 - x0, full_x))
        ny = o.get("count_y", cnt(y1 - y0, full_y))
        m = self.mesh_max_count
        return min(max(nx, 4), m), min(max(ny, 4), m)

    def _mesh(self, gcmd, o):
        """One LiDAR mesh with options o (LIDAR_BED_MESH, MESH_BENCHMARK).
        Returns the service's result plus "seconds"."""
        level = o["level"]
        name, n, cspeed, dn, dframes, dspeed = self.LEVELS[level]
        fine = level >= 4
        z = o["z"]
        x0, x1, y0, y1 = self._area(gcmd, o, fine)
        dwell = o["motion"] == "dwell"
        if not fine and dwell:
            n = dn
        ax0, ax1, ay0, ay1 = self._laser_area(z)
        nx, ny = self._counts(o, n, x0, x1, y0, y1,
                              ax1 - ax0 + (15.0 if fine else 0.0), ay1 - ay0)
        xs, ys = self._grid(x0, x1, nx), self._grid(y0, y1, ny)
        twist, yt, tnote = self._lidar_tables() if o["twist"] else (
            None, None, "")
        if tnote:
            gcmd.respond_info("LiDAR mesh: tables for %s" % (tnote,))
        if yt:
            twist = dict(twist or {}, ys=yt["ys"], ycomp=yt["comp"])
        if twist is None and o["twist"]:
            gcmd.respond_info(
                "LiDAR mesh: no LiDAR twist table yet (LIDAR_MESH_"
                "CALIBRATION), the mesh keeps the X-beam twist (~0.2mm across X)"
                + ("; the touches correct it" if level == 5 else ""))
        elif yt is None and o["twist"]:
            gcmd.respond_info(
                "LiDAR mesh: no Y table yet (LIDAR_MESH_CALIBRATION), the mesh"
                " can be tilted along Y by ~10-20 um/100mm"
                + ("; the touches correct it" if level == 5 else ""))
        bm = self.printer.lookup_object("bed_mesh")
        start = self.reactor.monotonic()
        # the scan moves must not follow the old mesh
        bm.set_mesh(None)
        how = name
        if fine:
            res = self._mesh_fine(gcmd, o, xs, ys, z, twist, level == 5)
            how += " (strips, STEP %.1f)" % (o.get("step", 2.0),)
        elif dwell:
            # one scan along a path through the grid points (serpentine
            # by columns), FRAMES frames 1mm apart in Y standing at each
            frames = o.get("frames", dframes)
            speed = o.get("speed", dspeed)
            self._check_reach(gcmd, [(x, y0, x, y1) for x in xs] + [
                (a, y, b, y) for a, b in zip(xs, xs[1:]) for y in (y0, y1)],
                z)
            path = []
            for i, x in enumerate(xs):
                for y in (ys if i % 2 == 0 else ys[::-1]):
                    path += [(x, y + k - (frames - 1) / 2.0)
                             for k in range(frames)]
            rows, secs, missed = self._scan_dwell(
                "mesh_fast", xs[0], y0, y1, 1.0, z, path=path, speed=speed)
            res = self._svc({"cmd": "mesh_fast", "scan": "mesh_fast",
                             "xs": xs, "ys": ys, "half_x": o["half_x"],
                             "half_y": o.get("half_y", 1.5),
                             "twist": twist}, timeout=60.0)["result"]
            res["missed"] = missed
            how += " (dwell, %d frame%s at %.0fmm/s)" % (
                frames, "s" if frames > 1 else "", speed)
        else:
            speed = o.get("speed", cspeed)
            self._check_reach(gcmd, [(x, y0, x, y1) for x in xs] + [
                (a, y, b, y) for a, b in zip(xs, xs[1:]) for y in (y0, y1)],
                z)
            kept, dropped = self._scan_moving(
                "mesh_fast", xs, y0, y1, z, speed, o.get("both_ways", 0))
            res = self._svc({"cmd": "mesh_fast", "scan": "mesh_fast",
                             "xs": xs, "ys": ys, "half_x": o["half_x"],
                             "half_y": o.get("half_y", 1.5),
                             "twist": twist}, timeout=60.0)["result"]
            res.update({"kept": kept, "dropped": dropped})
            how += " (continuous at %.0fmm/s, %d frames, %d dropped)" % (
                speed, kept, dropped)
            lag = res.get("lag")
            if lag:
                gcmd.respond_info(
                    "LiDAR mesh: the two directions agree best shifted by"
                    " %+.2fmm (%.4fs at %.0fmm/s; rms %.0f -> %.0f um):"
                    " moving_lag %+.4f" % (
                        lag["shift"], lag["shift"] / speed, speed,
                        (lag["rms0"] or 0) * 1000, lag["rms"] * 1000,
                        self.moving_lag + lag["shift"] / speed))
        if not res.get("ok"):
            raise gcmd.error("LiDAR mesh: %s" % (res.get("reason"),))
        self._load_mesh(gcmd, bm, o["profile"], xs, ys, res["matrix"])
        secs = self.reactor.monotonic() - start
        res["seconds"] = round(secs, 1)
        self.last_result = res
        info = {"method": "lidar level %d %s" % (level, how),
                "twist": bool(twist), "seconds": round(secs, 1),
                "time": time.strftime("%Y-%m-%dT%H:%M:%S")}
        extra = ""
        if fine:
            extra = (", profile tilt %.4f mm/mm, seams %s (X, step um,"
                     " spread um), height map %s" % (res["tilt"], res["seams"],
                                              res.get("map_file")))
        elif res.get("spread") is not None:
            extra = ", frame spread %.0f um" % (res["spread"] * 1000.0,)
        c = res.get("correction")
        if c:
            info["loo_um"] = round(c["loo_rms"] * 1000.0, 1)
            extra += (", nozzle correction order %d, LOO rms %.1f um at %d"
                      " touches (orders: %s), drift %+.1f um/min" % (
                          c["order"], c["loo_rms"] * 1000.0,
                          len(c["touches"]), ", ".join(
                              "%s: %.1f" % (k, v * 1000.0)
                              for k, v in sorted(c["loo"].items())),
                          c["drift_per_min"] * 1000.0))
        self.mesh_info[o["profile"]] = info
        gcmd.respond_info(
            "LiDAR mesh level %d %s: %dx%d points X%.0f-%.0f Y%.0f-%.0f,"
            " range %.3fmm, %.0fs%s%s" % (
                level, how, nx, ny, x0, x1, y0, y1, res["range"], secs,
                "" if twist else ", NO twist table", extra))
        return res

    def _mesh_fine(self, gcmd, o, xs, ys, z, twist, touch):
        # strips that use the whole laser line (+-14mm), side by side,
        # covering the mesh area
        x0, x1, y0, y1 = xs[0], xs[-1], ys[0], ys[-1]
        rx1 = min(self._laser_area(z)[1], self._reach_x(y0, y1, z, 0.3)[1])
        sx0 = o.get("strip_x0", max(x0 + 13.5, min(18.0, rx1)))
        sx1 = o.get("strip_x1", min(max(x1 - 13.5, sx0), rx1))
        ns = o.get("strips", max(1, int(math.ceil((sx1 - sx0) / 28.0)) + 1))
        step = o.get("step", 2.0)
        half_y = o.get("half_y", 2.0)
        sxs = self._grid(sx0, sx1, ns) if ns > 1 else [(sx0 + sx1) / 2]
        if ns > 1 and sxs[1] - sxs[0] > 28.5:
            raise gcmd.error("LiDAR: strips more than 28.5mm apart leave"
                             " gaps (the line is ~28mm wide)")
        self._check_reach(gcmd, [(x, y0, x, y1) for x in sxs], z)
        if touch:
            self._touch_ready(gcmd)
            # the nozzle heats for the touches while the LiDAR scans
            self._touch_heat(wait=False)
        for i, x in enumerate(sxs):
            name = "mesh_s%02d" % (i,)
            a, b = (y0, y1) if i % 2 == 0 else (y1, y0)
            rows, secs, missed = self._scan_dwell(name, x, a, b, step, z)
            self._svc({"cmd": "mesh_strip", "mesh": "fine",
                       "scan": name, "first": i == 0}, timeout=60.0)
            gcmd.respond_info("LiDAR mesh: strip %d/%d at X%.1f, %.0fs"
                              % (i + 1, ns, x, secs))
        touches = None
        if touch:
            n = o.get("touches", 5)
            m = 5.0
            tx = self._grid(x0 + m, x1 - m, n)
            ty = self._grid(y0 + m, y1 - m, n)
            run = self.gcode.run_script_from_command
            self._touch_heat(wait=True, wipe=o.get("wipe", 1))
            spots = [(x, y) for i, x in enumerate(tx)
                     for y in (ty if i % 2 == 0 else ty[::-1])]
            touches = self.touch_grid(gcmd, spots, o.get("samples", 3),
                                      o.get("bed_off", 1))
        ext, bed = self._temps()
        sv = self._saved()
        ps = self.printer.lookup_object("print_stats", None)
        meta = {"time": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "extruder": ext, "bed": bed, "z": z, "step": step,
                "twist": twist, "level": o["level"],
                "mesh_xs": xs, "mesh_ys": ys,
                "filament": sv.get("filament_type"),
                "file": ps.get_status(self.reactor.monotonic()).get(
                    "filename") if ps is not None else None}
        return self._svc({"cmd": "mesh_fine", "mesh": "fine", "xs": xs,
                          "ys": ys, "x_dir": self.x_dir,
                          "half_x": o["half_x"], "half_y": half_y,
                          "twist": twist, "meta": meta, "touches": touches,
                          "max_order": o.get("max_order", 3)},
                         timeout=180.0)["result"]

    def touch_grid(self, gcmd, spots, samples=3, bed_off=1, ref=None,
                   ref_every=6):
        """Nozzle touches (_touch) at spots [(x, y)], nozzle coordinates,
        bed mesh off. Every ref_every spots, and at the start and the end,
        the reference spot (zero_reference_position) once more: the drift
        in time between them comes off later (drift_fit). bed_off: the bed
        heater off just for each touch (its switching disturbs the load
        cells; a few seconds don't cool the bed). Returns [(x, y, z, time,
        is_ref)]."""
        self._touch_ready(gcmd)
        bm = self.printer.lookup_object("bed_mesh")
        if ref is None:
            ref = bm.bmc.zero_ref_pos
        if ref is None:
            ax0, ax1, ay0, ay1 = self._laser_area(self.scan_z)
            ref = (round((ax0 + ax1) / 2.0, 3), round((ay0 + ay1) / 2.0, 3))
        ref = list(ref)
        self._check_reach(gcmd, [self._touch_xy(x, y) * 2
                                 for x, y in spots + [ref]])
        heater = self.printer.lookup_object("heater_bed", None)
        target = heater.get_status(self.reactor.monotonic())["target"] \
            if heater is not None else 0.0
        off = on = ""
        if bed_off and target > 0:
            off = "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=0\nG4 P300\n"
            on = "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=%.1f\n" % (
                target,)
        run = self.gcode.run_script_from_command
        old = bm.get_mesh()
        bm.set_mesh(None)
        out = []
        order = [tuple(ref)]
        for i, p in enumerate(spots):
            if i and i % ref_every == 0:
                order.append(tuple(ref))
            order.append(p)
        order.append(tuple(ref))
        try:
            run("G90\nG0 Z5 F900")
            # the first nozzle tap after a wipe reads off: throw it away
            run("G0 X%.2f Y%.2f F9000\nG0 Z2 F900\n%s" % (
                self._touch_xy(*ref) + (off,)))
            self._touch(1)
            run("%sG0 Z5 F900" % (on,))
            for k, (x, y) in enumerate(order):
                run("G0 Z5 F900\nG0 X%.2f Y%.2f F9000\nG0 Z2 F900\n%s" % (
                    self._touch_xy(x, y) + (off,)))
                nz = self._touch(samples)
                run("%sG0 Z5 F900" % (on,))
                out.append((x, y, nz, self.reactor.monotonic(),
                            (x, y) == tuple(ref)))
                gcmd.respond_info("  touch %d/%d X%.1f Y%.1f: z %.4f" % (
                    k + 1, len(order), x, y, nz))
        finally:
            if on:
                run(on)
            bm.set_mesh(old)
        return out

    def _scan_moving(self, name, xs, y0, y1, z, speed, both_ways=0):
        """Columns at laser X xs from y0 to y1 (serpentine; both_ways: each
        column there and back), at constant speed, frames streamed. Each
        frame covers a window before it arrives (LIDAR_TIMING, + moving_lag)
        and is kept if the toolhead stood still or moved along Y at the full
        speed for all of it, at the middle of where it looked (trapq); the
        others (speeding up, slowing down, X steps) are dropped. The ends of
        every column stand still for one whole frame. Returns (kept,
        dropped).

        The trapq only answers from its history, and a move gets there once
        it is finalized, which for a long cruise is near its end: asked
        earlier, it answers with the previous move clamped at its end (the
        end of the speed-up, 0.4 mm into the column, at full speed). Frames
        therefore wait until the history covers their window (2026-10-29:
        without this the first half of every column was put at its start,
        stripes alternating by column)."""
        toolhead = self.printer.lookup_object("toolhead")
        mcu = self.printer.lookup_object("mcu")
        trapq = self.printer.lookup_object("motion_report").trapqs[
            "toolhead"]
        period, lo, hi, src = self._timing(self.scan_level)
        lag = self.moving_lag
        xo, yo = self._xo(z), self._yo(z)
        hold = period + (hi - lo) + 2 * self.dwell_margin + \
            self.dwell_settle + 0.05
        f, travel = speed * 60.0, max(speed, 150.0) * 60.0
        todo = []
        for i, x in enumerate(xs):
            a, b = (y0, y1) if i % 2 == 0 else (y1, y0)
            if i:
                todo.append("G0 X%.3f Y%.3f F%.0f" % (x - xo, a - yo,
                                                      travel))
            legs = [b, a] if both_ways else [b]
            todo.append("G4 P%.0f" % (hold * 1000.0,))
            for e in legs:
                todo += ["G0 Y%.3f F%.0f" % (e - yo, f),
                         "G4 P%.0f" % (hold * 1000.0,)]
        self.gcode.run_script_from_command(
            "SAVE_GCODE_STATE NAME=_lidar_scan\nG90\nG0 Z%.3f F600\n"
            "G0 X%.3f Y%.3f F%.0f" % (z, xs[0] - xo, y0 - yo, travel))
        toolhead.wait_moves()
        self._svc({"cmd": "begin", "name": name, "x": xs[0], "z": z,
                   "x_dir": self.x_dir, "x_offset": xo, "y_offset": yo,
                   "captures": 1, "level": self.scan_level, "path": []})
        head = ('{"cmd": "frame", "name": %s, "x": %%.3f, "y": %%.3f,'
                ' "span": %%.3f, "dir": %%d, "t": %%.4f, "raw": '
                % (json.dumps(name),))
        end = []
        move_end = [0.0]   # print time the last queued G0 ends
        pending = []       # (arrival, raw): waiting for the trapq history
        count = [0, 0]     # kept, dropped

        def feed(ahead=1.0):
            now = mcu.estimated_print_time(self.reactor.monotonic())
            while todo and toolhead.get_last_move_time() - now < ahead:
                cmd = todo.pop(0)
                self.gcode.run_script_from_command(cmd)
                if cmd.startswith("G0"):
                    move_end[0] = toolhead.get_last_move_time()
                if not todo:
                    end.append(toolhead.get_last_move_time())

        def place(arrival, raw):
            w0, w1 = arrival - hi + lag, arrival - lo + lag
            ps = [trapq.get_trapq_position(w) for w in
                  (w0, (w0 + w1) / 2, w1)]
            ok = all(p is not None for p, v in ps)
            still = mv = False
            if ok:
                ps = [(self._unskewed(p), v) for p, v in ps]
                p0, p1 = ps[0][0], ps[2][0]
                vs = [v for p, v in ps]
                still = max(vs) < 1e-6 and abs(p0[1] - p1[1]) < 1e-4
                # moving: full speed over the whole window, and it really
                # moved that far (a clamped answer has the speed, not the
                # distance)
                run = speed * (w1 - w0)
                mv = (min(vs) > 0.97 * speed and max(vs) < 1.03 * speed
                      and abs(p0[0] - p1[0]) < 1e-3
                      and abs(abs(p1[1] - p0[1]) - run) < 0.1 * run + 0.05)
                lx = (p0[0] + p1[0]) / 2 + xo
                col = min(xs, key=lambda c: abs(c - lx))
                ok = (still or mv) and abs(col - lx) < 0.05
            if ok:
                ly = (p0[1] + p1[1]) / 2 + yo
                d = 0 if still else (1 if p1[1] > p0[1] else -1)
                self._svc_send((head % (
                    col, ly, abs(p1[1] - p0[1]), d, arrival)).encode()
                    + raw.replace(b"\n", b" ") + b"}\n")
                count[0] += 1
            else:
                count[1] += 1

        def flush(final=False):
            # place the frames whose window the history covers by now
            horizon = self._trapq_horizon(trapq)
            while pending and (final or (
                    horizon is not None and
                    pending[0][0] - lo + lag <= horizon)):
                place(*pending.pop(0))

        frame_msg = self._frame_msg()
        in_flight = False
        try:
            try:
                self._send(frame_msg)
                in_flight = True
                feed()
                while True:
                    raw, t = self._recv(5.0)
                    in_flight = False
                    arrival = mcu.estimated_print_time(
                        self._frame_time(raw, t))
                    last = bool(end) and arrival - hi > end[0]
                    if not last:
                        self._send(frame_msg)
                        in_flight = True
                    feed()
                    pending.append((arrival, raw))
                    flush()
                    if last:
                        break
                toolhead.wait_moves()
                # the last move reaches the history shortly after it ended;
                # after it the toolhead stands still, so what is left can go
                stop = self.reactor.monotonic() + 3.0
                while pending and self.reactor.monotonic() < stop:
                    h = self._trapq_horizon(trapq)
                    if h is not None and h >= move_end[0] - 1e-6:
                        break
                    self.reactor.pause(self.reactor.monotonic() + 0.05)
                flush(final=True)
            except OSError as e:
                in_flight = False
                raise self._driver_error(e)
        finally:
            if in_flight:
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
        return count[0], count[1]

    def _trapq_horizon(self, trapq):
        """Print time up to which the trapq's history is complete: the end
        of its newest finalized move (None if empty)."""
        f = getattr(trapq, "history_end", None)
        if f is not None:       # the offline simulator
            return f()
        from klippy import chelper
        ffi_main, ffi_lib = chelper.get_ffi()
        data = ffi_main.new("struct pull_move[1]")
        if not ffi_lib.trapq_extract_old(trapq.trapq, data, 1, 0.0, 1e30):
            return None
        return data[0].print_time + data[0].move_t

    # ------------------------------------------------------- compare/bench
    def _zmesh(self, bm, name):
        """A profile (or "active") as a Kalico ZMesh."""
        if name == "active":
            m = bm.get_mesh()
            if m is None:
                raise self.printer.command_error("no active mesh")
            return m
        prof = bm.pmgr.get_profiles().get(name)
        if prof is None:
            raise self.printer.command_error("no mesh profile %s" % (name,))
        mod = sys.modules[type(bm).__module__]
        z = mod.ZMesh(prof["mesh_params"], name)
        z.build_mesh([list(r) for r in prof["points"]])
        return z

    def compare(self, bm, a, b, step=10.0):
        """mesh_stats of profile a - profile b over their overlap, on a
        step mm grid (~0.12ms per point on the K1)."""
        za, zb = self._zmesh(bm, a), self._zmesh(bm, b)
        pa, pb = za.get_mesh_params(), zb.get_mesh_params()
        x0, x1 = max(pa["min_x"], pb["min_x"]), min(pa["max_x"], pb["max_x"])
        y0, y1 = max(pa["min_y"], pb["min_y"]), min(pa["max_y"], pb["max_y"])
        if x1 - x0 < step or y1 - y0 < step:
            raise self.printer.command_error(
                "meshes %s and %s don't overlap" % (a, b))
        nx = int((x1 - x0) / step) + 1
        ny = int((y1 - y0) / step) + 1
        pts = []
        for j in range(ny):
            y = y0 + (y1 - y0) * j / max(ny - 1, 1)
            for i in range(nx):
                x = x0 + (x1 - x0) * i / max(nx - 1, 1)
                pts.append((x, y, za.calc_z(x, y) - zb.calc_z(x, y)))
            self._yield()
        st = mesh_stats(pts)
        st["area"] = [round(x0, 1), round(x1, 1), round(y0, 1), round(y1, 1)]
        return st

    def _info(self, name):
        i = self.mesh_info.get(name)
        if i:
            return "%s: %s, %s" % (name, i["method"], i["time"])
        return "%s: not made this session (method unknown)" % (name,)

    desc_MESH_COMPARE = "Compare two bed mesh profiles (A - B)"

    def cmd_MESH_COMPARE(self, gcmd):
        bm = self.printer.lookup_object("bed_mesh")
        a, b = gcmd.get("A"), gcmd.get("B", "active")
        try:
            st = self.compare(bm, a, b, gcmd.get_float("STEP", 10.0,
                                                       minval=2.0))
        except self.printer.command_error as e:
            raise gcmd.error("MESH_COMPARE: %s" % (e,))
        atc = self.printer.lookup_object("axis_twist_compensation", None)
        tw = bool(atc is not None and atc.z_compensations)
        gcmd.respond_info(
            "MESH_COMPARE %s - %s over X%.0f-%.0f Y%.0f-%.0f (%d points):\n"
            "  offset %+.1f um, plane X %+.1f Y %+.1f um/100mm\n"
            "  after the offset: rms %.1f um, max %.1f um\n"
            "  after the plane:  rms %.1f um, max %.1f um\n"
            "  worst (X, Y, um): %s\n  %s\n  %s\n"
            "  axis twist compensation (probe meshes) is %s now" % (
                a, b, st["area"][0], st["area"][1], st["area"][2],
                st["area"][3], st["n"], st["offset"] * 1000,
                st["plane_x"] * 1000, st["plane_y"] * 1000, st["rms"] * 1000,
                st["max"] * 1000, st["rms_plane"] * 1000,
                st["max_plane"] * 1000, ", ".join(
                    "(%.0f, %.0f, %+.0f)" % (x, y, v * 1000)
                    for x, y, v in st["worst"]),
                self._info(a), self._info(b), "ON" if tw else "OFF"))
        self.last_result = dict(st, a=a, b=b)

    desc_MESH_BENCHMARK = ("Run several bed mesh methods on one area, judge"
                           " them against a nozzle-touch grid")

    def cmd_MESH_BENCHMARK(self, gcmd):
        """METHODS (comma list): probe, 1-5 (LiDAR levels), cont<speed>
        (level 2 grid moving at that speed), dwell<level>, repeat<level>.
        Then a TRUTH x TRUTH grid of nozzle touches (held out: level 5's own
        touches are not scored) judges every mesh; the run goes to the helper
        as JSON + HTML (benchmarks/)."""
        if not self.service:
            raise gcmd.error("MESH_BENCHMARK needs the lidar service")
        methods = [m.strip().lower() for m in gcmd.get(
            "METHODS", "probe,1,2,3,4,5,repeat2").split(",") if m.strip()]
        ax0, ax1, ay0, ay1 = self._area_or_error(gcmd, self.scan_z)
        area = {"x0": gcmd.get_float("X0", ax0),
                "x1": gcmd.get_float("X1", ax1),
                "y0": gcmd.get_float("Y0", ay0),
                "y1": gcmd.get_float("Y1", ay1)}
        nt = gcmd.get_int("TRUTH", 6, minval=0, maxval=10)
        bm = self.printer.lookup_object("bed_mesh")
        atc = self.printer.lookup_object("axis_twist_compensation", None)
        run = self.gcode.run_script_from_command
        stamp = time.strftime("%Y%m%d-%H%M%S")
        done = []      # (name, profile, seconds, note, own touches)
        for m in methods:
            t0 = self.reactor.monotonic()
            note, own = "", []
            if m == "probe":
                prof = "bench_probe"
                run("BED_MESH_CALIBRATE PROFILE=%s MESH_MIN=%.1f,%.1f"
                    " MESH_MAX=%.1f,%.1f PROBE_COUNT=12,12" % (
                        prof, area["x0"], area["y0"], area["x1"],
                        area["y1"]))
                tw = bool(atc is not None and atc.z_compensations)
                note = "axis twist compensation %s" % (
                    "ON" if tw else "OFF (carries the X-beam twist)")
                self.mesh_info[prof] = {
                    "method": "probe 12x12, " + note,
                    "time": time.strftime("%Y-%m-%dT%H:%M:%S")}
            else:
                o = dict(area, z=self.scan_z, half_x=3.0, twist=1,
                         adaptive=0, motion="continuous")
                if m.startswith("cont"):
                    o.update(level=2, speed=float(m[4:]))
                    prof = "bench_" + m
                elif m.startswith("dwell"):
                    o.update(level=int(m[5:]), motion="dwell")
                    prof = "bench_" + m
                elif m.startswith("repeat"):
                    o.update(level=int(m[6:]))
                    prof = "bench_" + m
                elif m in ("1", "2", "3", "4", "5"):
                    o.update(level=int(m))
                    prof = "bench_L" + m
                else:
                    raise gcmd.error("MESH_BENCHMARK: unknown method %s" % (
                        m,))
                o["profile"] = prof
                res = self._mesh(gcmd, o)
                if o["level"] == 5 and res.get("correction"):
                    own = [(t[0], t[1]) for t in res["correction"]["touches"]]
                    note = "LOO %.1f um" % (
                        res["correction"]["loo_rms"] * 1000,)
            done.append((m, prof, self.reactor.monotonic() - t0, note, own))
        truth = []
        if nt:
            # the judge: its own grid, inset differently from level 5's
            tx = self._grid(area["x0"] + 12.0, area["x1"] - 12.0, nt)
            ty = self._grid(area["y0"] + 12.0, area["y1"] - 12.0, nt)
            spots = [(x, y) for i, x in enumerate(tx)
                     for y in (ty if i % 2 == 0 else ty[::-1])]
            tch = self.touch_grid(gcmd, spots)
            refs = [(t[3], t[2]) for t in tch if t[4]]
            z0, k = drift_fit(refs)
            t0 = refs[0][0] if refs else 0.0
            truth = [(x, y, z - k * (t - t0)) for x, y, z, t, r in tch
                     if not r]
        out = []
        first = {}
        for m, prof, secs, note, own in done:
            zm = self._zmesh(bm, prof)
            p = zm.get_mesh_params()
            e = {"name": m, "profile": prof, "time": round(secs, 1),
                 "note": note, "xs": self._grid(p["min_x"], p["max_x"],
                                                p["x_count"]),
                 "ys": self._grid(p["min_y"], p["max_y"], p["y_count"]),
                 "matrix": zm.get_probed_matrix()}
            held = [(x, y, z) for x, y, z in truth
                    if all(abs(x - a) > 3 or abs(y - b) > 3 for a, b in own)
                    and p["min_x"] <= x <= p["max_x"]
                    and p["min_y"] <= y <= p["max_y"]]
            if len(held) >= 4:
                d = [(x, y, z - zm.calc_z(x, y)) for x, y, z in held]
                e["vs_nozzle"] = mesh_stats(d)
                mu = e["vs_nozzle"]["offset"]
                e["touch_res"] = [(x, y, round(v - mu, 5)) for x, y, v in d]
            if m != "probe" and any(x[0] == "probe" for x in done):
                e["vs_probe"] = self.compare(bm, prof, "bench_probe")
            lv = m[6:] if m.startswith("repeat") else None
            if lv and lv in first:
                e["repeat"] = self.compare(bm, prof, first[lv])
            if m in ("1", "2", "3", "4", "5"):
                first[m] = prof
            out.append(e)
            gcmd.respond_info(
                "MESH_BENCHMARK %-8s %5.0fs  vs nozzle %s  vs probe %s%s" % (
                    m, secs, "rms %.1f max %.1f um" % (
                        e["vs_nozzle"]["rms"] * 1000,
                        e["vs_nozzle"]["max"] * 1000)
                    if "vs_nozzle" in e else "-",
                    "rms %.1f um" % (e["vs_probe"]["rms"] * 1000,)
                    if "vs_probe" in e else "-",
                    "  repeat rms %.1f um" % (e["repeat"]["rms"] * 1000,)
                    if "repeat" in e else ""))
        ext, bed = self._temps()
        ref = "5" if "5" in methods else ("probe" if "probe" in methods
                                          else None)
        rep = self._svc({"cmd": "benchmark_save", "run": {
            "stamp": stamp, "area": area, "extruder": ext, "bed": bed,
            "time": time.strftime("%Y-%m-%dT%H:%M:%S"), "ref": ref,
            "truth": [list(t) for t in truth], "methods": out,
            "summary": "bed %s C, extruder %s C, %d nozzle touches" % (
                bed, ext, len(truth))}}, timeout=60.0)
        gcmd.respond_info("MESH_BENCHMARK saved: %s" % (rep["html"],))

    def _load_mesh(self, gcmd, bm, profile, xs, ys, matrix):
        """Make the matrix Klipper's active mesh and save it as a profile
        for this session (SAVE_CONFIG keeps it), like BED_MESH_CALIBRATE."""
        mod = sys.modules[type(bm).__module__]
        cfg = bm.bmc.orig_config
        algo = cfg["algo"]
        if algo == "lagrange" and max(len(xs), len(ys)) > 6:
            algo = "bicubic"
        params = {"min_x": xs[0], "max_x": xs[-1], "min_y": ys[0],
                  "max_y": ys[-1], "x_count": len(xs), "y_count": len(ys),
                  "mesh_x_pps": cfg["mesh_x_pps"],
                  "mesh_y_pps": cfg["mesh_y_pps"],
                  "algo": algo if max(cfg["mesh_x_pps"],
                                      cfg["mesh_y_pps"]) else "direct",
                  "tension": cfg["tension"]}
        z_mesh = mod.ZMesh(params, profile)
        # relative to where Z is homed, like the probe's mesh (the LiDAR
        # has no absolute height)
        ref = bm.bmc.zero_ref_pos
        inside = ref is not None and xs[0] <= ref[0] <= xs[-1] and \
            ys[0] <= ref[1] <= ys[-1]
        z0 = ref_height(xs, ys, matrix, ref) if inside else None
        if z0 is not None:
            matrix = [[v - z0 for v in r] for r in matrix]
        try:
            z_mesh.build_mesh([list(r) for r in matrix])
        except mod.BedMeshError as e:
            raise gcmd.error(str(e))
        if inside and z0 is None:
            z_mesh.set_zero_reference(*ref)
        elif not inside:
            gcmd.respond_info("LiDAR mesh: zero_reference_position not"
                              " inside the mesh, mesh relative to its median")
        bm.set_mesh(z_mesh)
        bm.save_profile(profile)

    desc_LIDAR_TWIST_CALIBRATE = ("Measure the LiDAR's X-beam twist (AXIS=X)"
                                  " or its error along Y (AXIS=Y) against"
                                  " nozzle touches (load cells)")

    def cmd_LIDAR_TWIST_CALIBRATE(self, gcmd):
        """At spots along X (one Y): the nozzle touches the bed with the load
        cells, then the laser looks at the same spot. The gantry tips the
        toolhead forward/back by an amount that changes along X; the LiDAR
        (21mm in front of the nozzle) reads that as height. LiDAR - nozzle
        relative to its mean is the correction LIDAR_BED_MESH takes off.
        AXIS=Y: the same at spots along Y (one X), saved as lidar_ytwist:
        LiDAR meshes were tilted along Y by 10-20 um/100mm against the
        nozzle and the probe (2026-10-29, 2026-11-01)."""
        if not self.service:
            raise gcmd.error("LiDAR: LIDAR_TWIST_CALIBRATE needs the lidar"
                             " service")
        state = self.printer.lookup_object("print_stats").get_status(
            self.reactor.monotonic())["state"]
        if state in ("printing", "paused"):
            raise gcmd.error("LiDAR: not while a print is running")
        self._touch_ready(gcmd)
        z = gcmd.get_float("Z", self.scan_z)
        axis = gcmd.get("AXIS", "X").upper()
        if axis not in ("X", "Y"):
            raise gcmd.error("LiDAR: AXIS is X or Y")
        spots = self._twist_spots(gcmd, axis, z)
        # the coordinate along the measured axis
        k = 0 if axis == "X" else 1
        passes = gcmd.get_int("PASSES", 2, minval=1, maxval=4)
        samples = gcmd.get_int("SAMPLES", 3, minval=1)
        max_spread = gcmd.get_float("MAX_SPREAD", 0.04, above=0.0)
        apply = gcmd.get_int("APPLY", 1)
        # the bed heater switching disturbs the load cells: off just for
        # each touch (like touch_grid), so a hot bed stays at temperature
        bed_off = gcmd.get_int("BED_OFF", 1)
        heater = self.printer.lookup_object("heater_bed", None)
        target = heater.get_status(self.reactor.monotonic()).get(
            "target", 0.0) if heater is not None else 0.0
        off = on = ""
        if bed_off and target > 0:
            off = "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=0\nG4 P300\n"
            on = "\nSET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=%.1f" % (
                target,)
        # the nozzle touches at (x, y), the laser looks at y-1..y+1
        self._check_reach(gcmd, [(x, y - 1.0, x, y + 1.0) for x, y in spots],
                          z)
        self._check_reach(gcmd, [self._touch_xy(x, y) * 2 for x, y in spots])
        bm = self.printer.lookup_object("bed_mesh")
        old_mesh = bm.get_mesh()
        gcode_move = self.printer.lookup_object("gcode_move")
        z_off = gcode_move.get_status(
            self.reactor.monotonic())["homing_origin"][2]
        run = self.gcode.run_script_from_command
        gcmd.respond_info("LiDAR twist (%s): %d spots X%.0f-%.0f Y%.0f-%.0f,"
                          " %d passes" % (
                              axis, len(spots), min(sp[0] for sp in spots),
                              max(sp[0] for sp in spots),
                              min(sp[1] for sp in spots),
                              max(sp[1] for sp in spots), passes))
        diffs = dict((sp, []) for sp in spots)
        dists = []
        bm.set_mesh(None)
        try:
            run("G90\nG0 Z5 F900")
            # the first nozzle tap after a wipe reads off: throw it away
            run("G0 X%.2f Y%.2f F9000\nG0 Z2 F900" % self._touch_xy(
                *spots[0]))
            self._touch(1)
            run("G0 Z5 F900")
            for p in range(passes):
                for x, y in (spots if p % 2 == 0 else spots[::-1]):
                    run("G0 Z5 F900\nG0 X%.2f Y%.2f F9000\nG0 Z2 F900\n%s"
                        % (self._touch_xy(x, y) + (off,)))
                    nz = self._touch(samples)
                    run("G0 Z5 F900%s" % (on,))
                    h = self.twist_reading(x, y, z)
                    diffs[(x, y)].append((self.reactor.monotonic(), h - nz))
                    # LiDAR distance with the nozzle tip on the surface
                    dists.append(-h - (z + z_off - nz))
                    gcmd.respond_info("  X%.1f Y%.1f: nozzle z %.4f, LiDAR"
                                      " %.4f" % (x, y, nz, h))
            run("G0 Z5 F900")
        finally:
            bm.set_mesh(old_mesh)
        vals, drift, spread = drift_fit_spots(diffs)
        means = [vals[sp] for sp in spots]
        avg = sum(means) / len(means)
        comp = [round(m - avg, 5) for m in means]
        along = [sp[k] for sp in spots]
        slope, err, sd = line_slope(spots, comp, k)
        c0 = sum(comp) / len(comp) - slope * sum(along) / len(along)
        gcmd.respond_info(
            "LiDAR twist (LiDAR - nozzle, um): %s; slope %+.1f +-%.1f"
            " um/100mm, scatter %.0f um; drift %+.1f um/min taken out, then"
            " pass spread %.3fmm" % (
                ", ".join("%s%.0f %+.0f" % (axis, a, c * 1000.0)
                          for a, c in zip(along, comp)),
                slope * 1e5, err * 1e5, sd * 1000.0, drift * 6e4, spread))
        if passes > 1 and spread > max_spread:
            raise gcmd.error("LiDAR twist: passes disagree by %.3fmm (more"
                             " than MAX_SPREAD), not saved" % (spread,))
        if axis == "X":
            self.twist_save(gcmd, along, comp, spots[0][1], z, spread, dists,
                            apply)
        else:
            # the straight line through all spots, at the ends of the range
            # (ytwist_at holds the end values beyond them)
            ye = [min(along), max(along)]
            line = [round(slope * a + c0, 5) for a in ye]
            self.twist_save(gcmd, ye, line, sorted(set(sp[0] for sp in spots)),
                            z, spread, dists, apply, axis,
                            extra={"slope": round(slope, 7),
                                   "slope_err": round(err, 7),
                                   "scatter": round(sd, 5)})

    desc_LIDAR_GRID_CALIBRATE = (
        "Calibrate the LiDAR against nozzle touches on a grid (COUNT=7,5):"
        " X curve + Y curve for LIDAR_BED_MESH, the rest reported")

    def cmd_LIDAR_GRID_CALIBRATE(self, gcmd):
        """At every grid point the nozzle taps the bed with the load cells
        until 3 readings agree, then the laser looks at the same spot,
        before moving on (passes in alternate order, so a drift in time
        cancels). LiDAR - nozzle splits into an X curve (saved as
        lidar_twist), a Y curve (lidar_ytwist) and the interaction left
        after both, which is only reported (and saved as data): each spot's
        texture scatters +-13-20 um between the nozzle and the laser's view
        of it, so a per-point map would mostly copy that. Method and
        analysis after Axis Twist Mesh (see grid_analyze)."""
        if not self.service:
            raise gcmd.error("LiDAR: LIDAR_GRID_CALIBRATE needs the lidar"
                             " service")
        state = self.printer.lookup_object("print_stats").get_status(
            self.reactor.monotonic())["state"]
        if state in ("printing", "paused"):
            raise gcmd.error("LiDAR: not while a print is running")
        self._touch_ready(gcmd)
        z = gcmd.get_float("Z", self.scan_z)
        try:
            cnt = [int(v) for v in gcmd.get("COUNT", "7,5").split(",")]
        except ValueError:
            raise gcmd.error("LiDAR: COUNT=nx,ny")
        nx, ny = (cnt * 2)[:2]
        if not (3 <= nx <= 15 and 3 <= ny <= 15):
            raise gcmd.error("LiDAR: COUNT 3-15 per axis")
        xs, ys = self._grid_axes(gcmd, z, nx, ny)
        x0, x1, y0, y1 = xs[0], xs[-1], ys[0], ys[-1]
        passes = gcmd.get_int("PASSES", 2, minval=1, maxval=4)
        tol = gcmd.get_float("TOLERANCE", 0.010, above=0.0)
        max_taps = gcmd.get_int("MAX_TAPS", 7, minval=MIN_AGREEING_TAPS)
        max_spread = gcmd.get_float("MAX_SPREAD", 0.04, above=0.0)
        apply = gcmd.get_int("APPLY", 1)
        # serpentine by rows
        order = [(i, j) for j in range(ny)
                 for i in (range(nx) if j % 2 == 0 else range(nx - 1, -1, -1))]
        pts = [(xs[i], ys[j]) for i, j in order]
        self._check_reach(gcmd, [(x, y - 1.0, x, y + 1.0) for x, y in pts], z)
        self._check_reach(gcmd, [self._touch_xy(x, y) * 2 for x, y in pts])
        heater = self.printer.lookup_object("heater_bed", None)
        target = heater.get_status(self.reactor.monotonic()).get(
            "target", 0.0) if heater is not None else 0.0
        off = on = ""
        if gcmd.get_int("BED_OFF", 1) and target > 0:
            off = "SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=0\nG4 P300\n"
            on = "\nSET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=%.1f" % (
                target,)
        bm = self.printer.lookup_object("bed_mesh")
        old_mesh = bm.get_mesh()
        gcode_move = self.printer.lookup_object("gcode_move")
        z_off = gcode_move.get_status(
            self.reactor.monotonic())["homing_origin"][2]
        run = self.gcode.run_script_from_command
        gcmd.respond_info(
            "LiDAR grid: %dx%d points X%.0f-%.0f Y%.0f-%.0f, %d passes"
            " (~%.0f min)" % (nx, ny, x0, x1, y0, y1, passes,
                              nx * ny * passes * 21.0 / 60.0))
        diffs = dict((k, []) for k in order)
        dists, retaps = [], 0
        bm.set_mesh(None)
        try:
            run("G90\nG0 Z5 F900")
            # the first nozzle tap after a wipe reads off: throw it away
            run("G0 X%.2f Y%.2f F9000\nG0 Z2 F900" % self._touch_xy(*pts[0]))
            self._touch(1)
            run("G0 Z5 F900")
            for p in range(passes):
                for i, j in (order if p % 2 == 0 else order[::-1]):
                    x, y = xs[i], ys[j]
                    run("G0 Z5 F900\nG0 X%.2f Y%.2f F9000\nG0 Z2 F900"
                        % self._touch_xy(x, y))
                    taps, nz = [], None
                    while nz is None:
                        if len(taps) >= max_taps:
                            raise gcmd.error(
                                "LiDAR grid: no %d of %d nozzle taps at"
                                " X%.1f Y%.1f agreed within %.3fmm (%s);"
                                " clean the nozzle or raise TOLERANCE" % (
                                    MIN_AGREEING_TAPS, max_taps, x, y, tol,
                                    ", ".join("%.4f" % t for t in taps)))
                        if taps:
                            run("G91\nG0 Z1 F600\nG90")
                        if off:
                            run(off)
                        taps.append(self._touch(1))
                        if on:
                            run(on)
                        nz = agreeing_value(taps, MIN_AGREEING_TAPS, tol)
                    retaps += len(taps) - MIN_AGREEING_TAPS
                    run("G0 Z5 F900")
                    h = self.twist_reading(x, y, z)
                    diffs[(i, j)].append((self.reactor.monotonic(), h - nz))
                    dists.append(-h - (z + z_off - nz))
            run("G0 Z5 F900")
        finally:
            bm.set_mesh(old_mesh)
        d, drift, spread = drift_fit_spots(diffs)
        try:
            rep = grid_analyze(xs, ys, d, tol)
        except ValueError as e:
            raise gcmd.error("LiDAR grid: %s" % (e,))

        def um(vals):
            return ", ".join("%+.0f" % (v * 1000.0) for v in vals)
        gcmd.respond_info(
            "LiDAR grid (LiDAR - nozzle): X curve %s (X %s) range %.0f um;"
            " Y curve %s (Y %s) range %.0f um; left after both: rms %.1f um,"
            " max %.0f um; %d extra taps; drift %+.1f um/min taken out, then"
            " pass spread %.3fmm" % (
                um(rep["x_effect"]), ",".join("%.0f" % v for v in xs),
                rep["x_effect_range"] * 1000.0, um(rep["y_effect"]),
                ",".join("%.0f" % v for v in ys),
                rep["y_effect_range"] * 1000.0,
                rep["interaction_rms"] * 1000.0,
                rep["interaction_max"] * 1000.0, retaps, drift * 6e4,
                spread))
        gcmd.respond_info("LiDAR grid, left after both curves (um, rows Y"
                          " front to back): %s" % ("; ".join(
                              "Y%.0f: %s" % (ys[j], um(rep["interaction"][j]))
                              for j in range(ny)),))
        for x, y, v in rep["suspect_points"]:
            gcmd.respond_info("LiDAR grid: suspect point X%.0f Y%.0f (%+.0f"
                              " um off its neighbours), re-run if it stays"
                              % (x, y, v * 1000.0))
        for axis, pos, jump in rep["edge_warnings"]:
            gcmd.respond_info("LiDAR grid: the %s curve bends at its edge"
                              " %s%.0f by %+.0f um more than its trend (a"
                              " real bend of the gantry, or a misreading"
                              " there: check)" % (axis, axis, pos,
                                                  jump * 1000.0))
        if passes > 1 and spread > max_spread:
            raise gcmd.error("LiDAR grid: passes disagree by %.3fmm (more"
                             " than MAX_SPREAD), not saved" % (spread,))
        if not apply:
            return
        stamp = time.strftime("%Y-%m-%dT%H:%M")
        ym = sum(ys) / len(ys)
        xm = sum(xs) / len(xs)
        tables = (
            ("lidar_twist", {"xs": xs, "comp": [round(v, 5) for v in
                                                rep["x_effect"]],
                             "y": round(ym, 1), "z": z,
                             "spread": round(spread, 4), "time": stamp,
                             "source": "grid %dx%d" % (nx, ny)}),
            ("lidar_ytwist", {"ys": ys, "comp": [round(v, 5) for v in
                                                 rep["y_effect"]],
                              "x": round(xm, 1), "z": z,
                              "spread": round(spread, 4), "time": stamp,
                              "source": "grid %dx%d" % (nx, ny)}),
            ("lidar_grid", {"xs": xs, "ys": ys, "time": stamp,
                            "rms": round(rep["interaction_rms"], 5),
                            "rest": [[None if v is None else round(v, 4)
                                      for v in row]
                                     for row in rep["interaction"]]}))
        ext, bed = self._temps()
        # weights in the average of runs: the rows (X curve) and columns
        # (Y curve) each point of a curve averages
        tables[0][1]["w"], tables[1][1]["w"] = ny, nx
        for var, val in tables:
            val["bed"] = bed
            run('SAVE_VARIABLE VARIABLE=%s VALUE="%s"' % (
                var, repr(val).replace(" ", "")))
        t = self._store_by_temp({"twist": tables[0][1],
                                 "ytwist": tables[1][1]})
        gcmd.respond_info("LiDAR grid: tables kept for bed %dC (with the"
                          " earlier runs in lidar_cal_by_temp, which"
                          " meshes average)" % (t,))
        dist = round(sum(dists) / len(dists), 4)
        gcmd.respond_info(
            "LiDAR grid saved: X curve (lidar_twist) and Y curve"
            " (lidar_ytwist) for LIDAR_BED_MESH, the rest as data"
            " (lidar_grid); LiDAR distance with the nozzle on the bed"
            " %.4fmm" % (dist,))

    # also used by AXIS_TWIST_TOUCH_CALIBRATE (twist_touch.py), which measures
    # the probe and the LiDAR against the same nozzle touches
    def twist_reach(self, x, y, z=None):
        """Whether the laser can look at (x, y) from height z."""
        z = self.scan_z if z is None else z

        class _G:
            error = self.printer.command_error
        try:
            self._check_reach(_G(), [(x, y - 1.0, x, y + 1.0)], z)
        except self.printer.command_error:
            return False
        return True

    def twist_reading(self, x, y, z=None):
        """Height (mm, toolhead frame) the LiDAR reads at laser (x, y) from
        nozzle height z: a few frames standing still around the spot. Bed
        mesh off, printer homed."""
        z = self.scan_z if z is None else z
        path = [(x, y - 1.0), (x, y), (x, y + 1.0)]
        self._scan_dwell("twist", x, y, y, 1.0, z, path=path)
        r = self._svc({"cmd": "heights", "scan": "twist"})
        h = median([v for v in r["h"] if v is not None])
        if h != h:
            raise self.printer.command_error(
                "LiDAR twist: nothing seen at X%.1f Y%.1f" % (x, y))
        return h

    def twist_save(self, gcmd, xs, comp, y, z, spread, dists, apply=1,
                   axis="X", extra=None):
        """Save a twist table (comp = LiDAR - nozzle relative to the mean,
        per laser X; axis Y: xs are laser Y, y the X they were taken at)
        and log the LiDAR distance with the nozzle on the bed (dists,
        mm)."""
        run = self.gcode.run_script_from_command
        ext, bed = self._temps()
        stamp = time.strftime("%Y-%m-%dT%H:%M")
        if axis == "X":
            var = "lidar_twist"
            twist = {"xs": xs, "comp": comp, "y": y, "z": z,
                     "spread": round(spread, 4), "time": stamp}
        else:
            var = "lidar_ytwist"
            twist = {"ys": xs, "comp": comp, "x": y, "z": z,
                     "spread": round(spread, 4), "time": stamp}
            twist.update(extra or {})
        # kept as data for the Z offset question (nothing uses it yet)
        dist = round(sum(dists) / len(dists), 4)
        log = list(self._saved().get("lidar_zero_log") or [])[-19:]
        log.append([stamp, dist, ext, bed])
        gcmd.respond_info("LiDAR distance with the nozzle on the bed: %.4fmm"
                          " (extruder %s, bed %s)" % (dist, ext, bed))
        if not apply:
            return
        twist["bed"] = bed
        # weight in the average of runs: the lines it came from
        twist["w"] = len(y) if isinstance(y, list) else 1
        for v, val in ((var, twist), ("lidar_zero_log", log)):
            # no spaces: one G-code parameter
            run('SAVE_VARIABLE VARIABLE=%s VALUE="%s"' % (
                v, repr(val).replace(" ", "")))
        t = self._store_by_temp({"twist" if axis == "X" else "ytwist":
                                 twist})
        gcmd.respond_info("LiDAR twist saved (%s, and for bed %dC), used by"
                          " LIDAR_BED_MESH" % (var, t))

    # ---------------------------------------------------- first-layer check
    desc_FIRST_LAYER_SCAN = ("First-layer check: PHASE=before (bare plate"
                             " under the objects), PHASE=after (the first"
                             " layer, same rows), PHASE=cancel")

    def cmd_FIRST_LAYER_SCAN(self, gcmd):
        phase = gcmd.get("PHASE").lower()
        if phase == "cancel":
            self.fl = None
            return
        if phase == "before":
            return self._fl_before(gcmd)
        if phase == "after":
            return self._fl_after(gcmd)
        raise gcmd.error("FIRST_LAYER_SCAN: PHASE=before, after or cancel")

    def _fl_z_offset(self):
        gm = self.printer.lookup_object("gcode_move")
        return gm.get_status(self.reactor.monotonic())["homing_origin"][2]

    def _fl_scans(self, gcmd, phase):
        """The planned strips, bed mesh off (the scans are in the toolhead
        frame), every strip the same rows both times."""
        fl = self.fl
        bm = self.printer.lookup_object("bed_mesh", None)
        mesh = bm.get_mesh() if bm is not None else None
        if bm is not None:
            bm.set_mesh(None)
        try:
            for i, x in enumerate(fl["strips"]):
                name = "fl_%s%02d" % (phase[0], i)
                self._scan_dwell(name, x, fl["y0"], fl["y1"], fl["step"],
                                 fl["z"])
                self._svc({"cmd": "fl_strip", "fl": fl["id"],
                           "phase": phase, "scan": name}, timeout=60.0)
        finally:
            if bm is not None:
                bm.set_mesh(mesh)
        fl["z_" + phase] = round(fl["z"] + self._fl_z_offset(), 5)

    def _fl_before(self, gcmd):
        self.fl = None
        if not self.service:
            raise gcmd.error("FIRST_LAYER_SCAN: needs the lidar service")
        eo = self.printer.lookup_object("exclude_object", None)
        objs = eo.get_status(self.reactor.monotonic()).get("objects", []) \
            if eo is not None else []
        objs = [{"name": o.get("name"), "polygon": [
            (float(p[0]), float(p[1])) for p in o.get("polygon") or []]}
            for o in objs]
        objs = [o for o in objs if len(o["polygon"]) >= 3]
        if not objs:
            gcmd.respond_info("First-layer check: no objects in the file"
                              " (label objects in the slicer), skipped")
            return
        height = gcmd.get_float("HEIGHT", 0.2, above=0.04, maxval=1.0)
        z = self.scan_z + height
        toolhead = self.printer.lookup_object("toolhead")
        st = toolhead.get_status(self.reactor.monotonic())
        if "xyz" not in st["homed_axes"]:
            raise gcmd.error("FIRST_LAYER_SCAN: home first")
        xo, yo = self._xo(z), self._yo(z)
        ly0 = st["axis_minimum"][1] + yo + 0.5
        ay1 = self._laser_area(z)[3]
        rx0, rx1 = self._reach_x(ly0, ay1, z)
        lo, hi = (rx0, ly0), (rx1, ay1)
        plan = fl_plan([o["polygon"] for o in objs],
                       gcmd.get_float("MARGIN", 8.0, minval=4.0),
                       lo, hi, gcmd.get_float("STEP", 2.0, minval=0.5),
                       max_time=gcmd.get_float("MAX_TIME", 120.0,
                                               above=10.0))
        if plan is None:
            gcmd.respond_info("First-layer check: the objects are out of the"
                              " LiDAR's reach, skipped")
            return
        try:
            self._check_reach(gcmd, [(x, plan["y0"], x, plan["y1"])
                                     for x in plan["strips"]], z)
        except self.printer.command_error as e:
            gcmd.respond_info("First-layer check: skipped (%s)" % (e,))
            return
        ps = self.printer.lookup_object("print_stats", None)
        self.fl = dict(plan, z=z, height=height, objects=objs,
                       id=time.strftime("%Y%m%d-%H%M%S"),
                       filament=gcmd.get("FILAMENT_TYPE", "") or "",
                       filament_name=gcmd.get("FILAMENT_NAME", "") or "",
                       file=ps.get_status(self.reactor.monotonic()).get(
                           "filename") if ps is not None else "",
                       armed=False)
        self._svc({"cmd": "fl_begin", "fl": self.fl["id"]})
        t0 = self.reactor.monotonic()
        self._fl_scans(gcmd, "before")
        self.fl["armed"] = True
        self.fl["temps_before"] = self._temps()
        gcmd.respond_info(
            "First-layer check: bare plate scanned, %d strips X%.0f-%.0f,"
            " Y%.0f-%.0f every %.1fmm, %.0fs (the first layer is scanned"
            " the same way when layer 2 starts)" % (
                len(plan["strips"]), plan["strips"][0], plan["strips"][-1],
                plan["y0"], plan["y1"], plan["step"],
                self.reactor.monotonic() - t0))

    FL_PRIME_RETRACT = 0.5   # mm, after the prime line (like the line purge)

    def _fl_prime(self, fl, e):
        """Prime line after the first-layer scan: e mm of filament along a
        25 mm line 8 mm in front of the objects (between them and the line
        purge, 15 mm in front), 0.5 mm high like the purge, then a short
        retract and a sideways flick for the string. Called lifted, hot.
        False (nothing done) if there is no room in front."""
        pts = [p for o in fl.get("objects") or [] for p in o["polygon"]]
        if not pts:
            return False
        y = min(p[1] for p in pts) - 8.0
        cx = (min(p[0] for p in pts) + max(p[0] for p in pts)) / 2.0
        a, b = cx - 12.5, cx + 12.5
        if y < 5.0:
            return False

        class _G:
            error = self.printer.command_error
        try:
            self._check_reach(_G(), [(a, y, b + 5.0, y)])
        except self.printer.command_error:
            return False
        gm = self.printer.lookup_object("gcode_move")
        lift = gm.get_status(self.reactor.monotonic())["gcode_position"][2]
        run = self.gcode.run_script_from_command
        run("G90\nG0 X%.3f Y%.3f F%.0f\nG0 Z0.5 F600" % (
            a, y, self.speed * 60.0))
        # 10 mm/s: ~0.4 mm2 of line at 0.5 mm, ~4 mm3/s
        run("M83\nG1 X%.3f E%.3f F600" % (b, e))
        run("G1 E-%.2f F2100\nG0 X%.3f F6000\nG90\nG0 Z%.3f F600" % (
            self.FL_PRIME_RETRACT, b + 5.0, max(lift, 1.0)))
        return True

    def _fl_after(self, gcmd):
        fl = self.fl
        if fl is None or not fl.get("armed"):
            return
        final = gcmd.get_int("FINAL", 0)
        ps = self.printer.lookup_object("print_stats", None)
        state = ps.get_status(self.reactor.monotonic())["state"] \
            if ps is not None else "printing"
        if state != "printing" and not final:
            gcmd.respond_info("First-layer check: print is %s, skipped"
                              % (state,))
            self.fl = None
            return
        fl["armed"] = False
        retract = gcmd.get_float("RETRACT", 1.0, minval=0.0, maxval=3.0)
        cool = gcmd.get_float("COOL", 30.0, minval=0.0, maxval=80.0)
        # mm of filament on top of the retract to refill the nozzle after
        # the pause (it oozes while it waits), as a line off the part
        prime = gcmd.get_float("PRIME", 3.0, minval=0.0, maxval=15.0)
        run = self.gcode.run_script_from_command
        heater = self.printer.lookup_object("toolhead").get_extruder() \
            .get_heater()
        target = heater.get_status(self.reactor.monotonic())["target"]
        gm = self.printer.lookup_object("gcode_move")
        pos = gm.get_status(self.reactor.monotonic())["gcode_position"]
        t0 = self.reactor.monotonic()
        res = None
        run("SAVE_GCODE_STATE NAME=_fl_after")
        try:
            # no ooze onto the layer while the LiDAR looks at it: retract,
            # and let the nozzle cool a little (heated back before printing)
            if retract:
                run("M83\nG1 E-%.2f F1800" % (retract,))
            if cool and target > 0:
                run("M104 S%.0f" % (max(0.0, target - cool),))
            run("G90\nG0 Z%.3f F600" % (max(fl["z"], pos[2] + 1.0),))
            self._fl_scans(gcmd, "after")
            fl["temps_after"] = self._temps()
            res = self._fl_analyze(gcmd)
        except self.printer.command_error as e:
            # the print goes on; only the check is lost
            gcmd.respond_info("First-layer check failed: %s" % (e,))
        finally:
            if not final:
                # back over where the print stopped, still lifted, hot
                # again, then down and unretracted
                if cool and target > 0:
                    run("M109 S%.0f" % (target,))
                back = retract
                if prime and self._fl_prime(fl, retract + prime):
                    # the prime line refilled it and left this much
                    # retracted for the travel back
                    back = self.FL_PRIME_RETRACT
                run("G90\nG0 X%.3f Y%.3f F%.0f" % (pos[0], pos[1],
                                                     self.speed * 60.0))
                run("RESTORE_GCODE_STATE NAME=_fl_after MOVE=1"
                    " MOVE_SPEED=10")
                if back:
                    run("SAVE_GCODE_STATE NAME=_fl_u\nM83\nG1 E%.2f F1800\n"
                        "RESTORE_GCODE_STATE NAME=_fl_u" % (back,))
            else:
                run("RESTORE_GCODE_STATE NAME=_fl_after")
            self.fl = None
        gcmd.respond_info("First-layer check: print paused %.0fs for the"
                          " scan" % (self.reactor.monotonic() - t0,))
        return res

    def _fl_key(self, fl):
        name = fl.get("filament_name") or fl.get("filament") or "unknown"
        return name.replace(" ", "_").replace('"', "").replace("'", "")

    def _fl_analyze(self, gcmd):
        fl = self.fl
        meta = {"time": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "file": fl["file"], "filament": fl["filament"],
                "filament_name": fl["filament_name"],
                "temps_before": fl.get("temps_before"),
                "temps_after": fl.get("temps_after"),
                "z_offset": round(self._fl_z_offset(), 5)}
        plan = {"strips": fl["strips"], "y0": fl["y0"], "y1": fl["y1"],
                "step": fl["step"], "x_dir": self.x_dir,
                "z_before": fl["z_before"], "z_after": fl["z_after"],
                "height": fl["height"], "objects": fl["objects"]}
        try:
            res = self._svc({"cmd": "fl_analyze", "fl": fl["id"],
                             "plan": plan, "meta": meta},
                            timeout=60.0)["result"]
        except self.printer.command_error as e:
            gcmd.respond_info("First-layer check: analysis failed (%s)"
                              % (e,))
            return None
        if not res.get("ok"):
            gcmd.respond_info("First-layer check: no result (%s)"
                              % (res.get("reason"),))
            return res
        h = fl["height"]
        gcmd.respond_info(
            "First-layer check: %.3fmm measured for %.3fmm sliced (%+.3f),"
            " p05-p95 %.3f-%.3f over %.0f mm2; tilt X %+.0f Y %+.0f"
            " um/100mm; plate drift between the scans %+.0f um" % (
                res["median"], h, res["error"], res["p05"], res["p95"],
                res["area_mm2"], res["tilt_um_per_100mm"][0],
                res["tilt_um_per_100mm"][1], res["drift"]["plane"][0] * 1000))
        for o in res["objects"][:6]:
            gcmd.respond_info("  %s: %.3fmm (%d cells)" % (
                o["name"], o["median"], o["cells"]))
        self._fl_feedback(gcmd, res, meta["z_offset"])
        if res.get("html"):
            gcmd.respond_info("  report: %s on the helper" % (res["html"],))
        return res

    # learning: half the error per print, at most FL_STEP per print and
    # FL_MAX in total (mm), only from a clean measurement
    FL_RATE, FL_STEP, FL_MAX = 0.5, 0.03, 0.15

    def _fl_feedback(self, gcmd, res, applied):
        e = res["error"]
        good = (res["cells"] >= 100 and res["p95"] - res["p05"] <= 0.15
                and abs(e) <= 0.15 and res["drift"]["bare_rms"] <= 0.03)
        gcmd.respond_info(
            "  Z offset now %+.3f; for %.3fmm it would be %+.3f (%s)" % (
                applied, res["height"], applied - e,
                "measurement clean" if good else "measurement too noisy or"
                " too far off to learn from"))
        sw = self.printer.lookup_object("output_pin First_Layer_Learn", None)
        on = sw is not None and sw.get_status(
            self.reactor.monotonic()).get("value", 0) > 0
        if not (on and good):
            return
        key = self._fl_key(self.fl)
        step = max(-self.FL_STEP, min(self.FL_STEP, -self.FL_RATE * e))
        new = max(-self.FL_MAX, min(self.FL_MAX, applied + step))
        saved = dict(self._saved().get("fl_zoffset") or {})
        saved[key] = round(new, 4)
        self.gcode.run_script_from_command(
            'SAVE_VARIABLE VARIABLE=fl_zoffset VALUE="%s"' % (
                repr(saved).replace(" ", ""),))
        gcmd.respond_info("  learned for %s: Z offset %+.3f from the next"
                          " print (First_Layer_Learn)" % (key, new))

    # ----------------------------------------------------------------- skew
    desc_SKEW_SCAN = ("Scan a printed square for SKEW_CALIBRATE (NAME=, ROT="
                      "how far it is turned from as printed, e.g. 45 then"
                      " 135)")

    def cmd_SKEW_SCAN(self, gcmd):
        if not self.service:
            raise gcmd.error("SKEW_SCAN: needs the lidar service")
        name = gcmd.get("NAME")
        rot = gcmd.get_float("ROT", 0.0)
        toolhead = self.printer.lookup_object("toolhead")
        st = toolhead.get_status(self.reactor.monotonic())
        if "xyz" not in st["homed_axes"]:
            raise gcmd.error("SKEW_SCAN: home first (G28)")
        ps = self.printer.lookup_object("print_stats").get_status(
            self.reactor.monotonic())["state"]
        if ps in ("printing", "paused"):
            raise gcmd.error("SKEW_SCAN: not while a print is running")
        box = [gcmd.get_float(k, None) for k in ("X0", "Y0", "X1", "Y1")]
        if None in box:
            # the last print's objects
            eo = self.printer.lookup_object("exclude_object", None)
            objs = eo.get_status(self.reactor.monotonic()).get(
                "objects", []) if eo is not None else []
            pts = [p for o in objs for p in o.get("polygon") or []]
            if not pts:
                raise gcmd.error("SKEW_SCAN: give the part's area as X0= Y0="
                                 " X1= Y1= (no objects from the last print)")
            box = [min(p[0] for p in pts), min(p[1] for p in pts),
                   max(p[0] for p in pts), max(p[1] for p in pts)]
        # the part may lie turned: scan the square around its circle
        cx, cy = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
        r = math.hypot(box[2] - box[0], box[3] - box[1]) / 2.0
        box = [cx - r, cy - r, cx + r, cy + r]
        # the part's top (~1mm) stays in the LiDAR's 3-5mm range, and so
        # does the plate; Z homed on top of the part only adds its height
        height = gcmd.get_float("HEIGHT", 1.0, minval=0.2, maxval=3.0)
        z = gcmd.get_float("Z", self.scan_z + height, minval=self.scan_z)
        xo, yo = self._xo(z), self._yo(z)
        ly0 = st["axis_minimum"][1] + yo + 0.5
        ay1 = self._laser_area(z)[3]
        rx0, rx1 = self._reach_x(ly0, ay1, z)
        lo, hi = (rx0, ly0), (rx1, ay1)
        plan = skew_plan(box, gcmd.get_float("MARGIN", 10.0, minval=4.0),
                         lo, hi, gcmd.get_float("STEP", 1.0, minval=0.2,
                                                maxval=2.0))
        if plan is None:
            raise gcmd.error("SKEW_SCAN: the part is out of the LiDAR's"
                             " reach")
        self._check_reach(gcmd, [(x, plan["y0"], x, plan["y1"])
                                 for x in plan["strips"]], z)
        gcmd.respond_info(
            "Skew scan %s: %d strips X%.0f-%.0f, Y%.0f-%.0f every %.1fmm"
            " (about %.0fs)" % (name, len(plan["strips"]),
                                plan["strips"][0] - 14,
                                plan["strips"][-1] + 14, plan["y0"],
                                plan["y1"], plan["step"], plan["est"]))
        bm = self.printer.lookup_object("bed_mesh", None)
        mesh = bm.get_mesh() if bm is not None else None
        if bm is not None:
            bm.set_mesh(None)
        try:
            for i, x in enumerate(plan["strips"]):
                sname = "skew_%02d" % (i,)
                self._scan_dwell(sname, x, plan["y0"], plan["y1"],
                                 plan["step"], z)
                self._svc({"cmd": "skew_strip", "sk": name, "scan": sname,
                           "first": i == 0}, timeout=60.0)
        finally:
            if bm is not None:
                bm.set_mesh(mesh)
        res = self._svc({"cmd": "skew_analyze", "sk": name, "rot": rot,
                         "x_dir": self.x_dir,
                         "meta": {"time": time.strftime("%Y-%m-%dT%H:%M"),
                                  "box": box, "plan": plan, "z": z}},
                        timeout=120.0)
        if not res.get("ok"):
            raise gcmd.error("SKEW_SCAN: %s" % (res.get("error"),))
        m = res["result"]
        self.last_result = m
        if not m["ok"]:
            raise gcmd.error("SKEW_SCAN: %s" % (m["reason"],))
        gcmd.respond_info(
            "Skew scan %s (ROT=%.0f, %s): %s corner %.3f +-%.3f deg, part"
            " %.2f x %.2f mm, sides at %+.2f deg, parallel within %.3f/%.3f"
            " deg, edge rms %s um, %s points" % (
                name, rot, m["mode"],
                "front" if m["mode"] == "diamond" else "front-left",
                m["alpha"], m["sigma"], m["size"][0], m["size"][1], m["phi"],
                m["parallel_deg"][0], m["parallel_deg"][1],
                "/".join("%.0f" % (v * 1000.0) for v in m["rms"].values()),
                "/".join(str(v) for v in m["points"].values())))
        if m["mode"] != "diamond":
            gcmd.respond_info(
                "Skew scan: sides along the rows are only placed to about a"
                " row; for a precise result lay the part at about 45 degrees")
        gcmd.respond_info(
            "Skew scan: next, turn the part a quarter turn where it lies"
            " (don't flip it over) and run SKEW_SCAN NAME=<other> ROT=%.0f,"
            " then SKEW_CALIBRATE NAMES=%s,<other>" % (rot + 90.0, name))

    desc_SKEW_CALIBRATE = ("XY skew from SKEW_SCANs of one part turned"
                           " between them (NAMES=a,b; APPLY=1 sets it)")

    def cmd_SKEW_CALIBRATE(self, gcmd):
        if not self.service:
            raise gcmd.error("SKEW_CALIBRATE: needs the lidar service")
        names = [n.strip() for n in gcmd.get("NAMES").split(",") if n.strip()]
        res = self._svc({"cmd": "skew_solve", "names": names,
                         "printed_skew": gcmd.get_float("PRINTED_SKEW", 0.0),
                         "length": gcmd.get_float("LENGTH", 100.0,
                                                  above=10.0)})
        if not res.get("ok"):
            raise gcmd.error("SKEW_CALIBRATE: %s" % (res.get("error"),))
        r = res["result"]
        self.last_result = r
        if not r["ok"]:
            raise gcmd.error("SKEW_CALIBRATE: %s" % (r["reason"],))
        ac, bd, ad = r["set_skew"]
        gcmd.respond_info(
            "Skew: the part's corner is %.3f deg as printed, so XY skew"
            " %+.3f +-%.3f deg = %+.3f mm per 100 mm (measuring error of the"
            " scans %+.3f deg, residuals %s). Klipper: SET_SKEW XY=%.4f,%.4f,"
            "%.4f" % (r["angle"], r["skew_deg"], r["sigma_deg"],
                      r["mm_per_100"], r["measurement_d"], r["residuals"],
                      ac, bd, ad))
        if not gcmd.get_int("APPLY", 0):
            return
        if self.printer.lookup_object("skew_correction", None) is None:
            raise gcmd.error("SKEW_CALIBRATE: no [skew_correction] section")
        if abs(r["skew_deg"]) > 1.0 or r["sigma_deg"] > 0.1:
            raise gcmd.error("SKEW_CALIBRATE: %+.3f +-%.3f deg looks wrong,"
                             " not applied" % (r["skew_deg"],
                                               r["sigma_deg"]))
        self.gcode.run_script_from_command(
            "SET_SKEW XY=%.4f,%.4f,%.4f\nSKEW_PROFILE SAVE=lidar" % (
                ac, bd, ad))
        gcmd.respond_info("Skew: in use now as profile 'lidar'; SAVE_CONFIG"
                          " keeps it (it is loaded at startup)")

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
            nxs = [self._skewed([cx - self._xo(z), y - self._yo(z), z])[0]
                   for y in (cy - half, cy + half)]
            if not (lo[0] <= min(nxs) and max(nxs) <= hi[0]
                    and lo[1] <= cy - half - self._yo(z)
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
