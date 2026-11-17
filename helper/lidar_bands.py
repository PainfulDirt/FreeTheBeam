"""Flow from 1-layer flatness bands (runs on the helper; numpy).

The calibration print lays 6 bands of 14 x 10 mm at 92..112 % flow (lines
along Y at the slicer's spacing). Too little flow leaves gaps and a ridge
pattern at the line spacing; the right flow is the first band that is
closed and smooth.
- camera (IR fill light, measured on a K1 Max with black PEI):
  per band photo the dark gaps (% of pixels < 0.35 median), the ridge pattern
  (strongest spatial frequency of the column means, % of the median) and the
  gloss. Rule from the 2026-11-08 run (104 % judged best by eye): first
  band with gaps <= 0.05 % and pattern <= 5.5 %, at exposure 1000.
- LiDAR: per band a bare strip before the print and the same strip after;
  thickness = after - before in 0.05 mm bins across the lines, roughness =
  its row-wise detrended spread. Provisional rule (no real band scans yet):
  the first band within 25 % of the way from the smoothest to the roughest
  band. Thickness is only reported: thin white lines on black PEI read low.
Both refuse to save when the lowest band already passes or none does.

This file may be distributed under the terms of the GNU GPLv3 license.
"""
import numpy as np

BAND_GAPS_MAX = 0.05        # % dark pixels (camera)
BAND_PATTERN_MAX = 5.5      # % ridge amplitude of the median (camera)
CAMERA_EXPOSURE = 1000      # the camera thresholds were measured at this
LIDAR_KNEE = 0.25           # fraction smoothest -> roughest (LiDAR)
LIDAR_MIN_RANGE_UM = 3.0    # less spread than this: no signal (LiDAR)
LIDAR_GAPS_MAX = 1.0        # % bins near the bare plate (LiDAR)
BIN_MM = 0.05


def band_metrics(img):
    """Camera metrics of one band photo taken over the band's centre: the
    central 600 x 360 px (~11.6 x 7 mm at Z3)."""
    h, w = img.shape
    c = img[h // 2 - 180:h // 2 + 180, w // 2 - 300:w // 2 + 300]
    med = float(np.median(c))
    prof = c.mean(axis=0)
    f = np.abs(np.fft.rfft(prof - prof.mean()))
    k = int(np.argmax(f[10:])) + 10
    return {"median": med,
            "gaps": 100.0 * float((c < 0.35 * med).mean()),
            "pattern": 100.0 * float(f[k]) / (len(prof) / 2.0) / max(med, 1.0),
            "period_px": len(prof) / float(k),
            "gloss": 100.0 * float((c > 2.0 * med).mean())}


def _pick(bands, passes, what, line):
    report = [line(b) for b in bands]
    ok = [b for b in bands if passes(b)]
    res = {"ok": False, "bands": bands, "report": report}
    if not ok:
        res["reason"] = "no band passes (%s): flow above the tested range?" % (
            what,)
    elif ok[0] is bands[0]:
        res["reason"] = ("the lowest band (%.0f %%) already passes (%s):"
                         " test lower flows" % (bands[0]["flow"] * 100, what))
    else:
        fl = ok[0]["flow"]
        res.update(ok=True, flow=fl, f=round(1.0 / fl, 4))
    report.append("  -> %s" % ("flow %.0f %% (f %.4f)" % (
        res["flow"] * 100, res["f"]) if res["ok"] else res["reason"]))
    return res


def pick_camera(bands):
    """bands: [{flow, gaps, pattern, ...}] in rising flow."""
    return _pick(
        bands, lambda b: b["gaps"] <= BAND_GAPS_MAX
        and b["pattern"] <= BAND_PATTERN_MAX,
        "gaps <= %.2f %%, pattern <= %.1f %%" % (BAND_GAPS_MAX,
                                                 BAND_PATTERN_MAX),
        lambda b: "  camera %3.0f %%: gaps %.2f %%, pattern %.2f %%, gloss"
        " %.2f %%" % (b["flow"] * 100, b["gaps"], b["pattern"],
                      b.get("gloss", 0.0)))


def _bins(p, x_strip, x_dir, x0, x1):
    """Median height (mm) per BIN_MM bin over printer X x0..x1 of one row
    (points (N, 3) um; the LiDAR reads distance: height = -z)."""
    x = x_strip + x_dir * p[:, 0] / 1000.0
    m = (x >= x0) & (x < x1)
    idx = ((x[m] - x0) / BIN_MM).astype(int)
    z = -p[m, 2] / 1000.0
    n = int(round((x1 - x0) / BIN_MM))
    out = np.full(n, np.nan)
    for i in np.unique(idx):
        if 0 <= i < n:
            out[i] = np.median(z[idx == i])
    return out


def lidar_band_metrics(before, after, x_strip, x_dir, band, margin=1.5):
    """One band from its bare strip and the same strip after printing:
    rows {y: points (N, 3) um}, the laser's X and profile direction."""
    x0, x1 = band["x0"] + margin, band["x1"] - margin
    ys = sorted(y for y in set(before) & set(after)
                if band["y0"] + margin <= y <= band["y1"] - margin)
    diffs, sds = [], []
    for y in ys:
        d = _bins(after[y], x_strip, x_dir, x0, x1) - \
            _bins(before[y], x_strip, x_dir, x0, x1)
        ok = ~np.isnan(d)
        if ok.sum() < 20:
            continue
        t = np.arange(len(d))[ok]
        v = d[ok]
        res = v - np.polyval(np.polyfit(t, v, 1), t)
        diffs.append(v)
        sds.append(float(np.std(res)))
    if not diffs:
        return {"thickness": None, "roughness": None, "gaps": None, "rows": 0}
    allv = np.concatenate(diffs)
    th = float(np.median(allv))
    return {"thickness": round(th, 4),
            "roughness": round(1000.0 * float(np.median(sds)), 2),
            "gaps": round(100.0 * float((allv < 0.35 * th).mean()), 2),
            "rows": len(diffs)}


def pick_lidar(bands):
    """bands: [{flow, roughness um, gaps %, thickness mm}] in rising flow."""
    r = [b["roughness"] for b in bands if b.get("roughness") is not None]
    line = lambda b: "  LiDAR  %3.0f %%: %s" % (
        b["flow"] * 100, "no scan data" if b.get("roughness") is None else
        "roughness %.1f um, gaps %.2f %%, thickness %.3f mm" % (
            b["roughness"], b["gaps"], b["thickness"]))
    if len(r) < len(bands):
        return {"ok": False, "bands": bands, "reason": "bands without scan"
                " data", "report": [line(b) for b in bands]}
    lo, hi = min(r), max(r)
    if hi - lo < LIDAR_MIN_RANGE_UM:
        return {"ok": False, "bands": bands, "reason": "roughness differs by"
                " only %.1f um between the bands: no signal" % (hi - lo,),
                "report": [line(b) for b in bands]}
    thr = lo + LIDAR_KNEE * (hi - lo)
    res = _pick(bands, lambda b: b["roughness"] <= thr
                and b["gaps"] <= LIDAR_GAPS_MAX,
                "roughness <= %.1f um, gaps <= %.1f %%" % (thr, LIDAR_GAPS_MAX),
                line)
    res["threshold_um"] = round(thr, 2)
    return res
