"""IR-camera analysis for the LiDAR calibrations (helper only: numpy + PIL).

The module's sensor takes fill-light pictures (0x56 type 2, 1600x1200 JPEG):
image column -> printer -X, image row -> printer +Y, ~19.4 um per pixel at
Z3, the picture's centre at the laser position. Printed lines show up with
sharp edges (white matte PLA on black PEI: bright lines; on a bright plate
they can be darker), so their WIDTH can be measured to a few um, which the
LiDAR can't do on thin translucent lines (the laser light spreads inside
them). Background and measurements: docs/camera.md.

This file may be distributed under the terms of the GNU GPLv3 license.
"""
import io
import math

import numpy as np

MM_PX = 0.0194          # mm per pixel at Z3 (the layouts give the real scale)
CENTRE_COL = 800.0      # image column of the photo position (approx.)
MIN_CONTRAST = 10.0     # grey levels between a line and the plate beside it


def load_gray(data):
    """JPEG bytes (or raw 8-bit 1600x1200 / 1600x600) -> float image."""
    if len(data) in (1600 * 1200, 1600 * 600):
        return np.frombuffer(data, np.uint8).reshape(-1, 1600).astype(float)
    from PIL import Image
    return np.asarray(Image.open(io.BytesIO(data)).convert("L"), dtype=float)


def _profile(img):
    """Column profile over the middle of the picture, plate level removed,
    sign chosen so lines are positive (bright or dark lines)."""
    h = img.shape[0]
    prof = np.median(img[h // 12:h - h // 12], axis=0)
    bg = np.percentile(prof, 20)
    p = prof - bg
    if -p.min() > p.max():          # dark lines on a bright plate
        p = -p
    return p


def _peaks(p, rel, merge=20):
    lim = max(6.0, rel * p.max())
    c = [i for i in range(merge, len(p) - merge)
         if p[i] >= lim and p[i] == p[i - merge:i + merge + 1].max()]
    groups = []
    for i in c:
        if groups and i - groups[-1][-1] < merge:
            groups[-1].append(i)
        else:
            groups.append([i])
    return np.array([float(np.mean(g)) for g in groups])


def find_lines(img, xs_mm, photo_x, mm_per_px=MM_PX, rel=0.25):
    """Image column of each line at printer X xs_mm (None when not found).
    The picture's offset is found by matching the expected pattern."""
    cand = _peaks(_profile(img), rel)
    if not len(cand):
        return [None] * len(xs_mm)
    xs = np.asarray(xs_mm, float)
    rel_c = (photo_x - xs) / mm_per_px
    best = None
    for c0 in np.arange(CENTRE_COL - 250, CENTRE_COL + 250, 1.0):
        d = np.abs(cand[None, :] - (c0 + rel_c)[:, None]).min(axis=1)
        s = np.minimum(d, 40.0).sum()
        if best is None or s < best[0]:
            best = (s, c0)
    out = []
    for ec in best[1] + rel_c:
        k = int(np.argmin(np.abs(cand - ec)))
        out.append(float(cand[k]) if abs(cand[k] - ec) <= 25 else None)
    return out


def scale_from_cols(cols, xs_mm):
    """mm per pixel from the found columns of lines at known X."""
    pts = [(c, x) for c, x in zip(cols, xs_mm) if c is not None]
    if len(pts) < 2:
        return MM_PX
    c, x = np.array(pts).T
    k = np.polyfit(x, c, 1)[0]      # px per mm (negative: column -> -X)
    return 1.0 / abs(k)


def line_widths(img, col, half=25, side=(18, 32)):
    """Per image row (row, width px, centre px) of the line near column col,
    following a tilted line. Edges at half way between the plate beside the
    line and the line's plateau, linear sub-pixel. Rows where the line is
    weaker than MIN_CONTRAST are left out."""
    sign = 1.0 if _profile(img).max() >= 0 else -1.0
    h, w = img.shape
    c = int(round(col))
    out = []
    box = np.ones(13)
    track = c
    for r in range(h):
        if track - half - 6 < 0 or track + max(half, side[1]) + 6 >= w:
            break
        row = sign * img[r]
        # re-centre on the brightest 13 px span near the tracked centre
        # (lines tilt; the column from find_lines is their middle); the
        # track itself only moves with a measured row
        seg = np.convolve(row[track - half // 2 - 6:track + half // 2 + 7],
                          box, "valid")
        c = track - half // 2 + int(np.argmax(seg))
        bg = np.median(np.r_[row[c - side[1]:c - side[0]],
                             row[c + side[0]:c + side[1]]])
        # plateau: median of the line's middle, not its brightest pixels
        # (the fill light leaves a specular stripe on the round top)
        top = np.median(row[c - 6:c + 7])
        if top - bg < MIN_CONTRAST:
            continue
        lvl = 0.5 * (bg + top)
        i = c
        while i > c - half and row[i] > lvl:
            i -= 1
        j = c
        while j < c + half and row[j] > lvl:
            j += 1
        if (i <= c - half or j >= c + half or row[i + 1] == row[i]
                or row[j - 1] == row[j]):
            continue
        left = i + (lvl - row[i]) / (row[i + 1] - row[i])
        right = j - 1 + (row[j - 1] - lvl) / (row[j - 1] - row[j])
        centre = 0.5 * (left + right)
        out.append((r, right - left, centre))
        track = int(round(centre))
    return np.array(out).reshape(-1, 3)


def central_rows(w, img_h, keep=0.75):
    """The rows of line_widths() output inside the central `keep` of the
    picture (the corners are dark from the fill light: noisier edges)."""
    lo = img_h * (1.0 - keep) / 2.0
    m = (w[:, 0] >= lo) & (w[:, 0] < img_h - lo)
    return w[m]


# ------------------------------------------------------------ PA from widths
W_ACC, W_DEC = 8.0, 3.0         # mm after a speed-up / slowdown


def windows_for_pitch(pitch_px):
    """line_widths() search half width and plate window for lines pitch_px
    apart: as wide as the neighbours allow (bulges at low PA are wide)."""
    half = int(min(60, max(20, 0.40 * pitch_px)))
    side = (int(min(45, max(16, 0.30 * pitch_px))),
            int(min(62, max(28, 0.45 * pitch_px))))
    return half, side


def _line_rows(photos, xs):
    """Per line: list of (Y array, width mm array, rows-in-photo Y array) per
    photo where the line was found."""
    per = [[] for _ in xs]
    for ph in photos:
        img = ph["img"]
        h = img.shape[0]
        cols = find_lines(img, xs, ph["x"])
        k = scale_from_cols(cols, xs)
        half, side = windows_for_pitch(abs(xs[1] - xs[0]) / k)
        all_y = ph["y"] + (np.arange(h) - h / 2.0) * k
        for i, c in enumerate(cols):
            if c is None:
                continue
            w = line_widths(img, c, half=half, side=side)
            if len(w) < 0.3 * h:
                continue
            per[i].append((ph["y"] + (w[:, 0] - h / 2.0) * k, w[:, 1] * k,
                           all_y))
    return per


def pa_from_widths(photos, pa):
    """PA from the width of the PA test's lines in photos centred on its
    speed changes. photos: [{"img", "x", "y"}] (laser position); pa: the
    layout (x0, pitch, y0, pa, dirs, segments, fast, slow). Per line the
    extra width after slowing down minus after speeding up (mm of line,
    relative to the steady width it settles to); a straight line through
    score vs PA crosses zero at the right PA."""
    n = len(pa["pa"])
    xs = [pa["x0"] + i * pa["pitch"] for i in range(n)]
    bounds, y = [], pa["y0"]
    for seglen, v in pa["segments"]:
        bounds.append((y, y + seglen, v))
        y += seglen
    per = _line_rows(photos, xs)
    report, scores = [], []

    def steady(rows, lo, hi):
        vals = [w[(Y >= lo) & (Y <= hi)] for Y, w, _ in rows]
        vals = np.concatenate(vals) if vals else np.array([])
        return float(np.median(vals)) if len(vals) >= 30 else float("nan")

    for i in range(n):
        rows, d = per[i], pa["dirs"][i]
        if not rows:
            scores.append(None)
            report.append("  PA %.3f: line not found" % (pa["pa"][i],))
            continue
        acc, dec = [], []
        for j in range(len(bounds) - 1):
            a0, a1, va = bounds[j]
            b0, b1, vb = bounds[j + 1]
            cur, nxt = (va, vb) if d > 0 else (vb, va)
            win = W_ACC if nxt > cur else W_DEC
            dst = (b0 + W_ACC + 0.5, b1 - 2.0) if d > 0 else \
                (a0 + 2.0, a1 - W_ACC - 0.5)
            lo, hi = sorted((a1, a1 + d * win))
            for Y, w, all_y in rows:
                inwin = (all_y >= lo) & (all_y <= hi)
                if inwin.sum() == 0 or \
                        all_y[inwin].max() - all_y[inwin].min() < 0.7 * win:
                    continue
                ref = steady([(Y, w, all_y)], *dst)
                if not ref > 0:
                    ref = np.nanmedian(
                        [steady(rows, b[0] + W_ACC + 0.5, b[1] - 2.0)
                         for b in bounds if b[2] == nxt]
                        + [steady(rows, b[0] + 2.0, b[1] - W_ACC - 0.5)
                           for b in bounds if b[2] == nxt])
                if not ref > 0:
                    continue
                # rows where the line isn't found (dark picture corners,
                # thin fast sections) are left out, not counted as 0
                m = (Y >= lo) & (Y <= hi)
                if m.sum() < 0.5 * inwin.sum():
                    continue
                ex = (w[m].mean() - ref) / ref * win
                (acc if nxt > cur else dec).append(ex)
        ma = float(np.mean(acc)) if acc else float("nan")
        md = float(np.mean(dec)) if dec else float("nan")
        s = md - ma
        scores.append(s if math.isfinite(s) else None)
        report.append("  PA %.3f: after speed-up %+.2f mm (%d), after slowdown"
                      " %+.2f mm (%d), score %+.2f" % (
                          pa["pa"][i], ma, len(acc), md, len(dec), s))
    found = sum(1 for r in per if r)
    good = [(pa["pa"][i], s) for i, s in enumerate(scores) if s is not None]
    res = {"ok": False, "pa": None, "r2": None, "slope": None,
           "scores": [None if s is None else round(s, 4) for s in scores],
           "lines_found": found, "reason": "", "report": report}
    if len(good) < 7:
        res["reason"] = "only %d of %d lines measured" % (len(good), n)
    else:
        px, sy = np.array(good).T
        k, c = np.polyfit(px, sy, 1)
        ss = ((sy - sy.mean()) ** 2).sum()
        r2 = 1.0 - ((sy - (k * px + c)) ** 2).sum() / ss if ss > 0 else 0.0
        res.update(slope=round(float(k), 3), r2=round(float(r2), 3))
        lo, hi = min(pa["pa"]), max(pa["pa"])
        step = (hi - lo) / max(1, n - 1)
        best = -c / k if k else float("nan")
        if k >= 0:
            res["reason"] = "score doesn't fall with PA (slope %+.1f)" % (k,)
        elif not lo - 0.5 * step <= best <= hi:
            res["reason"] = "PA %.3f is outside the tested range" % (best,)
        elif r2 < 0.6:
            res["reason"] = "measurement too noisy (r2 %.2f)" % (r2,)
        else:
            res.update(ok=True, pa=round(max(float(best), 0.0), 4))
    report.append("camera PA: %s" % ("%.4f (r2 %.2f)" % (res["pa"], res["r2"])
                                     if res["ok"] else res["reason"]))
    return res


# ---------------------------------------------------- flow from widths + gap
SKIP_START = 8.0        # mm after a flow line's start (prime blob)
ROUND = 1.0 - math.pi / 4.0     # area lost by round sides: ROUND * h^2


def table_at(t, key, v):
    """LiDAR correction table value (lidar_twist "xs" / lidar_ytwist "ys"),
    linear between the spots, the end values outside; 0 without a table."""
    if not t or key not in t:
        return 0.0
    return float(np.interp(v, t[key], t["comp"]))


def strip_plate(rows, y_eval, half=0.6):
    """Plate height (mm, -distance) at the strip centre for each Y: rows =
    scan Y -> points (N, 3) in um. Only the profile centre is used: off
    centre the profile's own tilt (~0.04 mm/mm, module mounting) is in it."""
    ys = np.array(sorted(rows))
    out = {}
    for y in y_eval:
        p = rows[float(ys[np.argmin(np.abs(ys - y))])]
        m = np.abs(p[:, 0] / 1000.0) < half
        if m.sum() >= 10:
            out[float(y)] = -float(np.median(p[m, 2])) / 1000.0
    return out


def gap_pairs(rows, strip_x, ys, mesh_at, twist=None, ytwist=None):
    """[(mesh z, plate z)] along a bare strip centred at laser X strip_x,
    the plate with the same corrections as the LiDAR mesh."""
    tx = table_at(twist, "xs", strip_x)
    return [(mesh_at(strip_x, y), p - tx - table_at(ytwist, "ys", y))
            for y, p in sorted(strip_plate(rows, ys).items())]


def group_gaps(samples):
    """Gap difference per triplet group (mm): median(mesh - plate) minus the
    mean over the groups (the absolute gap comes from the nozzle touch)."""
    med = dict((g, float(np.median([m - p for m, p in s])))
               for g, s in samples.items() if s)
    if not med:
        return {}
    mean = sum(med.values()) / len(med)
    return dict((g, v - mean) for g, v in med.items())


def flow_widths(photos, flow):
    """Width of every flow-test line from the photos over the triplets.
    flow: layout (y0, y1, groups [{k, lines [{x, flow}]}]). Per line
    {"x", "flow", "group", "width" mm, "noise" mm (row to row), "rows"}."""
    lines = [(ln["x"], ln["flow"], g["k"]) for g in flow["groups"]
             for ln in g["lines"]]
    xs = [l[0] for l in lines]
    pitch = min(abs(a - b) for a, b in zip(sorted(xs), sorted(xs)[1:]))
    got = [[] for _ in lines]
    for ph in photos:
        img = ph["img"]
        h = img.shape[0]
        cols = find_lines(img, xs, ph["x"])
        k = scale_from_cols(cols, xs)
        half, side = windows_for_pitch(pitch / k)
        for i, c in enumerate(cols):
            if c is None:
                continue
            w = central_rows(line_widths(img, c, half=half, side=side), h)
            y = ph["y"] + (w[:, 0] - h / 2.0) * k
            keep = (y >= flow["y0"] + SKIP_START) & (y <= flow["y1"] - 1.0)
            got[i].append(w[keep, 1] * k)
    out = []
    for (x, f, g), parts in zip(lines, got):
        parts = [p for p in parts if len(p)]
        if not parts:
            out.append({"x": x, "flow": f, "group": g, "width": None,
                        "noise": None, "rows": 0})
            continue
        noise = float(np.median(np.concatenate(
            [np.abs(np.diff(p)) for p in parts if len(p) > 1]))) / 0.954
        allw = np.concatenate(parts)
        out.append({"x": x, "flow": f, "group": g,
                    "width": float(np.median(allw)), "noise": noise,
                    "rows": int(len(allw))})
    return out


def flow_fit(lines, gaps, h0=0.2, w_nom=0.45, h_nom=0.2):
    """Filament flow ratio from the line widths: the line's cross-section is
    W*h - ROUND*h^2 at gap h = h0 + its group's gap difference; commanded was
    f * w_nom * h_nom. Checks that the width follows the commanded flow."""
    keep, report = [], []
    for ln in lines:
        if ln["width"] is None or ln["rows"] < 200 or ln["noise"] > 0.020:
            report.append("  X%.1f flow %.2f: dropped (%d rows, noise %s)" % (
                ln["x"], ln["flow"], ln["rows"], "-" if ln["noise"] is None
                else "%.0f um" % (ln["noise"] * 1000)))
            continue
        h = h0 + gaps.get(ln["group"], 0.0)
        nom = ln["flow"] * w_nom * h_nom
        F = (ln["width"] * h - ROUND * h * h) / nom
        dfdh = (ln["width"] - 2.0 * ROUND * h) / nom
        keep.append(dict(ln, h=h, F=F, dfdh=dfdh))
        report.append("  X%.1f flow %.2f: width %.3f mm, gap %.3f -> F %.3f" % (
            ln["x"], ln["flow"], ln["width"], h, F))
    res = {"ok": False, "f": None, "dfdh": None, "sd": None, "by_flow": {},
           "steps_um": {}, "kept": len(keep), "reason": "", "report": report}
    if len(keep) < 0.75 * len(lines) or len(keep) < 3:
        res["reason"] = "only %d of %d flow lines measured" % (len(keep),
                                                               len(lines))
        report.append("camera flow: " + res["reason"])
        return res
    Fs = np.array([k["F"] for k in keep])
    f, sd = float(Fs.mean()), float(Fs.std())
    res.update(f=round(f, 4), sd=round(sd, 4),
               dfdh=round(float(np.mean([k["dfdh"] for k in keep])), 3))
    for fl in sorted(set(k["flow"] for k in keep)):
        res["by_flow"][fl] = round(float(np.mean(
            [k["F"] for k in keep if k["flow"] == fl])), 4)
    for g in sorted(set(k["group"] for k in keep)):
        pts = [(k["flow"], k["width"]) for k in keep if k["group"] == g]
        if len(set(p[0] for p in pts)) >= 2:
            a = np.array(pts)
            res["steps_um"][g] = round(float(np.polyfit(a[:, 0], a[:, 1], 1)[0])
                                       * 0.1 * 1000, 1)
    bad = [g for g, s in res["steps_um"].items() if not 25 <= s <= 70]
    off = [fl for fl, v in res["by_flow"].items() if abs(v - f) > 0.03]
    if bad:
        res["reason"] = "the width doesn't follow the flow (groups %s: %s um" \
            " per +10 %%)" % (bad, [res["steps_um"][g] for g in bad])
    elif sd > 0.05:
        res["reason"] = "lines disagree (sd %.3f)" % (sd,)
    elif off:
        res["reason"] = "commanded flows %s give different results" % (off,)
    elif not 0.80 <= f <= 1.20:
        res["reason"] = "flow %.3f is implausible" % (f,)
    else:
        res["ok"] = True
    report.append("camera flow: %s" % ("%.3f (sd %.3f, %d lines)" % (
        f, sd, len(keep)) if res["ok"] else res["reason"]))
    return res


def line_contrast(img, cols):
    """Median line-to-plate contrast (grey levels) at the found columns."""
    p = _profile(img)
    v = [p[int(round(c))] for c in cols if c is not None]
    return float(np.median(v)) if v else 0.0


# ------------------------------------------------ offset cross from photos
def _robust_line(t, v, passes=3, tol=2.0):
    """v = a + b t, refitted without points more than tol off."""
    keep = np.ones(len(t), bool)
    a = b = 0.0
    for _ in range(passes):
        if keep.sum() < 10:
            break
        b, a = np.polyfit(t[keep], v[keep], 1)
        keep = np.abs(v - (a + b * t)) < tol
    return a, b, keep


def _arm(img, search=45):
    """The strongest straight line running along the rows of img: centre
    column = a + b*row, and the first/last row where it is seen. No row to
    row tracking (real arms have blobs, gaps and strings next to them):
    every row is searched near the arm's column, the brightest 13 px span
    with real contrast (its median: thin strings don't count) gives the
    centre, a robust line fit keeps the arm."""
    # an arm covers less than half of the rows: a high percentile finds it
    p = np.percentile(img, 80, axis=0)
    col = int(np.argmax(p - np.percentile(p, 20)))
    w = img.shape[1]
    lo, hi = max(0, col - search), min(w, col + search + 1)
    if hi - lo < 30:
        return None
    box = np.ones(13) / 13.0
    pts = []
    for r in range(img.shape[0]):
        row = img[r]
        seg = row[lo:hi]
        sm = np.convolve(seg, box, "valid")
        c = lo + 6 + int(np.argmax(sm))
        top = np.median(row[c - 6:c + 7])
        side = np.r_[row[max(0, c - search - 30):max(0, c - search)],
                     row[c + search:c + search + 30]]
        bg = np.median(side) if len(side) else 0.0
        if top - bg < MIN_CONTRAST:
            continue
        lvl = 0.5 * (top + bg)
        a, b = max(0, c - 25), min(w, c + 26)
        wv = np.clip(row[a:b] - lvl, 0, None)
        if wv.sum() <= 0:
            continue
        pts.append((r, (np.arange(a, b) * wv).sum() / wv.sum()))
    if len(pts) < 50:
        return None
    t, v = np.array(pts).T
    a_, b_, keep = _robust_line(t, v, tol=4.0)
    r = t[keep]
    return {"a": a_, "b": b_, "r0": float(r.min()), "r1": float(r.max()),
            "n": int(keep.sum())}


def cross_from_photos(light_img, laser_img, arm_mm=7.0):
    """The 1-layer offset cross in a fill-light picture and the laser line in
    a laser picture taken at the same pose. Returns the cross centre (px),
    the scale along columns / rows (mm per px, from the arm lengths: the
    camera sees the plate at an angle, so they differ) and the laser line
    (row = laser_fit[0] + laser_fit[1] * column)."""
    ya = _arm(light_img)              # arm along printer Y: along the rows
    xa = _arm(light_img.T)            # arm along printer X: along the columns
    if ya is None or xa is None or ya["n"] < 150 or xa["n"] < 150:
        return {"ok": False, "reason": "cross not found in the picture"}
    # col = ya.a + ya.b row ; row = xa.a + xa.b col
    cy = (xa["a"] + xa["b"] * ya["a"]) / (1.0 - xa["b"] * ya["b"])
    cx = ya["a"] + ya["b"] * cy
    sy = 2.0 * arm_mm / (ya["r1"] - ya["r0"] + 1.0)
    sx = 2.0 * arm_mm / (xa["r1"] - xa["r0"] + 1.0)
    if not (0.012 < sx < 0.035 and 0.012 < sy < 0.035):
        return {"ok": False, "reason": "implausible scale (%.4f / %.4f mm"
                " per px)" % (sx, sy)}
    # laser line: brightest rows per column, away from the X arm (it glows)
    f = laser_img.astype(float)
    skip = arm_mm / sx + 40.0
    cols = [c for c in range(5, f.shape[1] - 5, 2) if abs(c - cx) > skip]
    pts = []
    for c in cols:
        colv = f[:, c]
        r = int(np.argmax(colv))
        if r < 5 or r > len(colv) - 6:
            continue
        bgv = np.median(colv)
        wv = np.clip(colv[r - 4:r + 5] - bgv, 0, None)
        if wv.sum() <= 0 or colv[r] - bgv < 3 * MIN_CONTRAST:
            continue
        pts.append((c, (np.arange(r - 4, r + 5) * wv).sum() / wv.sum()))
    if len(pts) < 50:
        return {"ok": False, "reason": "laser line not found"}
    t, v = np.array(pts).T
    la, lb, keep = _robust_line(t, v, tol=1.5)
    return {"ok": True, "cx_px": float(cx), "cy_px": float(cy),
            "sx": float(sx), "sy": float(sy), "laser_fit": [float(la), float(lb)],
            "laser_row_px": float(la + lb * cx), "reason": ""}


def laser_row_at(cross, col):
    a, b = cross["laser_fit"]
    return a + b * col


def offset_from_crosses(per_z, test, intr_cx, ref_z):
    """LiDAR offsets from cross pictures at two nozzle heights, in
    LIDAR_OFFSET_CALIBRATE's convention: where the profile's x = 0 (image
    column intr_cx) on the laser line lies, relative to the nozzle, for the
    distance d from the nozzle to the surface the laser hits. The laser
    line is found on the bare plate beside the cross (not on top of the
    arms, as the LiDAR fit is), so d = z here; a line through both heights
    gives the offset at ref_z and per mm.
    per_z: [{"z", "nozzle": (x, y), "cross": cross_from_photos()}]."""
    pts, report = [], []
    for p in per_z:
        c = p["cross"]
        if not c.get("ok"):
            return {"ok": False, "reason": "Z%.1f: %s" % (p["z"], c.get("reason")),
                    "report": report}
        lx = test["cx"] - (intr_cx - c["cx_px"]) * c["sx"]
        ly = test["cy"] + (laser_row_at(c, intr_cx) - c["cy_px"]) * c["sy"]
        xt, yt = lx - p["nozzle"][0], ly - p["nozzle"][1]
        d = p["z"]                  # laser on the plate, not on the arms
        pts.append((d, xt, yt))
        report.append("  Z%.1f: cross at col %.1f row %.1f, laser row %.1f,"
                      " %.4f / %.4f mm/px -> offset X%.3f Y%.3f" % (
                          p["z"], c["cx_px"], c["cy_px"],
                          laser_row_at(c, intr_cx), c["sx"], c["sy"], xt, yt))
    (d1, x1, y1), (d2, x2, y2) = pts[:2]
    kx, ky = (x2 - x1) / (d2 - d1), (y2 - y1) / (d2 - d1)
    res = {"ok": True, "x_offset": round(x1 + kx * (ref_z - d1), 3),
           "y_offset": round(y1 + ky * (ref_z - d1), 3),
           "x_per_z": round(kx, 4), "y_per_z": round(ky, 4), "reason": "",
           "report": report}
    report.append("camera offset at Z%.1f: X%.3f Y%.3f, per mm X%+.3f Y%+.3f"
                  % (ref_z, res["x_offset"], res["y_offset"], kx, ky))
    if abs(kx) > 1.0 or abs(ky) > 3.0:
        res.update(ok=False, reason="implausible change with height")
    return res
