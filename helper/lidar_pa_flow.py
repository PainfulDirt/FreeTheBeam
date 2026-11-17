"""G-code planners for the LiDAR camera calibrations (runs on the helper;
pure Python). The Klipper plugin (lidar.py) passes the parameters, checks the returned paths
against its travel / keep-out / skew (it has the Klipper objects), writes
the G-code and runs it.

plan_pa_flow: LIDAR_PA_FLOW_CALIBRATE, one print for pressure advance (photo
widths at the speed changes of 9 PA lines) and/or flow (6 one-layer bands
at 92..112 %, judged by the camera, the LiDAR or both: lidar_bands.py).
plan_offset_cross: LIDAR_OFFSET_CALIBRATE's 1-layer cross.
Background: docs/camera.md

This file may be distributed under the terms of the GNU GPLv3 license.
"""
import math

H_NOM, W_NOM = 0.2, 0.45
PA_PITCH, PA_LINES = 2.5, 9
PA_SEGS = [(15.0, 20.0), (20.0, 150.0), (15.0, 20.0), (20.0, 150.0),
           (15.0, 20.0), (20.0, 150.0), (15.0, 20.0)]
BAND_FLOWS = [0.92, 0.96, 1.00, 1.04, 1.08, 1.12]
BAND_W, BAND_H, BAND_GAP = 14.0, 10.0, 4.0
BAND_SP = W_NOM - H_NOM * (1 - math.pi / 4)     # the slicer's line spacing
BAND_EXPOSURE = 1000          # lidar_bands.CAMERA_EXPOSURE (no numpy here)
PA_AFTER_BANDS = 25.0         # PA lines start this far behind Y
PRIME_LEN = 50.0


def _q(v):
    return '"%s"' % (str(v).replace('"', "'"),)


def _segments(g):
    """Consecutive XY positions of the G0/G1 moves as (x0, y0, x1, y1)."""
    segs, pos = [], None
    for l in g:
        if not l.startswith(("G0 ", "G1 ")):
            continue
        p = dict((t[0], float(t[1:])) for t in l.split()[1:] if t[0] in "XY")
        if not p:
            continue
        nxt = (p.get("X", pos[0] if pos else 0.0),
               p.get("Y", pos[1] if pos else 0.0))
        if pos is not None and nxt != pos:
            segs.append((pos[0], pos[1], nxt[0], nxt[1]))
        pos = nxt
    return segs


def _band_gcode(b, speed, fil_area):
    """One 1-layer band: lines along Y at the slicer's spacing, E scaled by
    the band's flow (M221 stays 100)."""
    epm = BAND_SP * H_NOM / fil_area * b["flow"]
    g = ["G0 Z1 F600"]
    for k in range(int(BAND_W / BAND_SP)):
        cx = b["x0"] + BAND_SP * (k + 0.5)
        ya, yb = (b["y0"], b["y1"]) if k % 2 == 0 else (b["y1"], b["y0"])
        if k == 0:
            g += ["G0 X%.3f Y%.3f F9000" % (cx, ya), "G0 Z%.3f F600" % H_NOM,
                  "G1 E0.8 F2400"]
        else:
            g.append("G1 X%.3f Y%.3f E%.5f F%.0f" % (cx, ya, BAND_SP * epm,
                                                      speed * 60))
        g.append("G1 X%.3f Y%.3f E%.5f F%.0f" % (cx, yb, BAND_H * epm,
                                                  speed * 60))
    return g + ["G1 E-0.8 F2400"]


def plan_pa_flow(p):
    """tests: ["pa"], ["flow"] or both; pa_method / flow_method: "ir"
    (camera photos), "lidar" (bare + printed LiDAR strips) or "both"
    (benchmark)."""
    X, Y = float(p.get("x", 100.0)), float(p.get("y", 35.0))
    temp, bed = float(p["extruder_temp"]), float(p["bed_temp"])
    tests = [t for t in ("pa", "flow") if t in p.get("tests", ["pa", "flow"])]
    method = p.get("flow_method", "ir")
    pa_method = p.get("pa_method", "ir")
    flows = [float(f) for f in p.get("flows") or BAND_FLOWS]
    speed = float(p.get("speed", 40.0))
    pa_keep = float(p.get("pa_keep", 0.0))
    fil = float(p["fil_area"])
    epm = H_NOM * W_NOM / fil
    test = {"version": 2, "tests": tests, "photos": [], "strips": [],
            "filament_name": p.get("filament_name", ""),
            "filament_type": p.get("filament_type", ""),
            "extruder_temp": temp, "bed_temp": bed, "pa_keep": pa_keep}
    x1o, y1o = X + 24.0, Y + PRIME_LEN + 2.0
    if "flow" in tests:
        bands = [{"name": "B%d" % (i + 1), "flow": f,
                  "x0": X + i * (BAND_W + BAND_GAP),
                  "x1": X + i * (BAND_W + BAND_GAP) + BAND_W,
                  "y0": Y, "y1": Y + BAND_H} for i, f in enumerate(flows)]
        test["flow"] = {"bands": bands, "method": method, "height": H_NOM,
                        "spacing": round(BAND_SP, 4), "speed": speed,
                        "exposure": BAND_EXPOSURE}
        mids = [((b["x0"] + b["x1"]) / 2, Y + BAND_H / 2) for b in bands]
        if method in ("ir", "both"):
            test["photos"] += [{"name": b["name"], "x": mx, "y": my,
                                "exposure": BAND_EXPOSURE}
                               for b, (mx, my) in zip(bands, mids)]
        if method in ("lidar", "both"):
            test["strips"] = [{"name": "pf_b%d" % (i + 1), "x": mx,
                               "y0": Y - 2.0, "y1": Y + BAND_H + 2.0}
                              for i, (mx, my) in enumerate(mids)]
        x1o = max(x1o, bands[-1]["x1"] + 2.0)
        y1o = max(y1o, Y + BAND_H + 2.0)
    y0p = Y + PA_AFTER_BANDS if "flow" in tests else Y
    y1p = y0p + sum(s[0] for s in PA_SEGS)
    pas = [round(float(p.get("pa_start", 0.0)) + i * float(p.get("pa_step", 0.01)), 4)
           for i in range(PA_LINES)]
    dirs = [1 if i % 2 == 0 else -1 for i in range(PA_LINES)]
    if "pa" in tests:
        changes, yy = [], y0p
        for seglen, _ in PA_SEGS[:-1]:
            yy += seglen
            changes.append(yy)
        test["pa"] = {"x0": X, "pitch": PA_PITCH, "y0": y0p, "y1": y1p,
                      "pa": pas, "dirs": dirs, "segments": PA_SEGS,
                      "fast": 150.0, "slow": 20.0, "height": H_NOM,
                      "width": W_NOM, "method": pa_method}
        test["pa_strip"] = {"name": "pf_pa", "x": X + 10.0, "y0": y0p - 3.0,
                            "y1": y1p + 3.0}
        if pa_method in ("ir", "both"):
            test["photos"] += [{"name": "T%d" % (i + 1), "x": X + 10.0,
                                "y": c} for i, c in enumerate(changes)]
        test["plate_photo"] = {"x": X + 10.0, "y": y0p + 60.0}
        y1o = max(y1o, y1p + 2.0)
    else:
        test["plate_photo"] = {"x": X + 7.0, "y": Y + BAND_H / 2}
    x0o, y0o = X - 14.0, Y - 2.0
    test["area"] = [x0o, y0o, x1o, y1o]
    what = " and ".join({"pa": "PA (%s)" % (pa_method,),
                         "flow": "flow (%s)" % (method,)}[t]
                        for t in tests)
    g = ["; LiDAR %s calibration (helper/lidar_pa_flow.py)" % (what,),
         "SET_PRINT_STATS_INFO TOTAL_LAYER=1",
         "EXCLUDE_OBJECT_DEFINE NAME=lidar_pa_flow CENTER=%.1f,%.1f"
         " POLYGON=[[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f],[%.1f,%.1f]]" % (
             (x0o + x1o) / 2, (y0o + y1o) / 2, x0o, y0o, x1o, y0o, x1o, y1o,
             x0o, y1o),
         "START_PRINT EXTRUDER_TEMP=%.0f BED_TEMP=%.0f CHAMBER_TEMP=0"
         " FILAMENT_NAME=%s FILAMENT_TYPE=%s" % (
             temp, bed, _q(test["filament_name"]), _q(test["filament_type"])),
         "SET_PRINT_STATS_INFO CURRENT_LAYER=1",
         # (the bare step ran inside START_PRINT, before its purge:
         # _LIDAR_PRINT_SCAN in lidar.example.cfg)
         "M221 S100", "M83", "G90", "SET_VELOCITY_LIMIT ACCEL=5000",
         "M109 S%.0f" % temp, "M106 S64",
         "EXCLUDE_OBJECT_START NAME=lidar_pa_flow",
         "SET_PRESSURE_ADVANCE ADVANCE=%.4f" % pa_keep,
         # prime line, outside the photos
         "G0 Z1 F600", "G0 X%.3f Y%.3f F9000" % (X - 12.0, Y),
         "G0 Z%.3f F600" % H_NOM, "G1 E1.6 F2400",
         "G1 X%.3f Y%.3f E%.5f F1200" % (X - 12.0, Y + PRIME_LEN,
                                         PRIME_LEN * epm * 1.2),
         "G1 E-0.8 F2400"]
    if "flow" in tests:
        g.append("; bands %s" % "/".join("%.0f" % (f * 100) for f in flows))
        for b in test["flow"]["bands"]:
            g += _band_gcode(b, speed, fil)
    if "pa" in tests:
        g += ["; PA %s" % " ".join("%.3f" % v for v in pas),
              "G0 Z1 F600", "G0 X%.3f Y%.3f F9000" % (X, y0p),
              "G0 Z%.3f F600" % H_NOM, "G1 E0.8 F2400"]
        for i, pa in enumerate(pas):
            x = X + i * PA_PITCH
            y = y0p if dirs[i] > 0 else y1p
            g.append("SET_PRESSURE_ADVANCE ADVANCE=%.4f" % pa)
            for seglen, v in (PA_SEGS if dirs[i] > 0 else list(reversed(PA_SEGS))):
                y += dirs[i] * seglen
                g.append("G1 X%.3f Y%.3f E%.5f F%.0f" % (x, y, seglen * epm, v * 60))
            if i + 1 < PA_LINES:
                g.append("G1 X%.3f E%.5f F1200" % (x + PA_PITCH, PA_PITCH * epm))
        g.append("G1 E-0.8 F2400")
    g += ["G0 Z3 F600",
          "EXCLUDE_OBJECT_END NAME=lidar_pa_flow", "M106 S0",
          "SET_PRESSURE_ADVANCE ADVANCE=%.4f" % pa_keep,
          "SET_VELOCITY_LIMIT ACCEL=%.0f" % float(p.get("max_accel", 5000.0)),
          # cool down in front of everything, nothing drips on the lines
          "G0 X%.3f Y%.3f F9000" % (X + 10.0, Y - 15.0),
          "M104 S150", "M106 S255",
          "TEMPERATURE_WAIT SENSOR=extruder MAXIMUM=155", "M106 S0",
          "LIDAR_PA_FLOW_STEP STEP=PHOTOS",
          "END_PRINT",
          "LIDAR_PA_FLOW_STEP STEP=ANALYZE"]
    lsegs = [(s["x"], s["y0"], s["x"], s["y1"]) for s in test["strips"]]
    if "pa_strip" in test:
        s = test["pa_strip"]
        lsegs.append((s["x"], s["y0"], s["x"], s["y1"]))
    lsegs += [(ph["x"], ph["y"], ph["x"], ph["y"])
              for ph in test["photos"] + [test["plate_photo"]]]
    if "flow" in tests:     # the camera/strip positions of every band
        lsegs += [(mx, my, mx, my) for mx, my in mids]
    return {"gcode": g, "test": test, "segments": _segments(g),
            "laser_segments": lsegs}


def plan_offset_cross(p):
    cx, cy = float(p["cx"]), float(p["cy"])
    z1, z2 = float(p.get("z1", 3.0)), float(p.get("z2", 5.0))
    arm, width, h = 7.0, 0.6, 0.2
    epm = h * width / float(p["fil_area"])
    g = ["M83", "G90", "M106 S0"]
    # the X arm, then the Y arm in two halves (the nozzle never crosses the
    # X arm low)
    for (xa, ya), (xb, yb) in (((cx - arm, cy), (cx + arm, cy)),
                               ((cx, cy - arm), (cx, cy - 2.0)),
                               ((cx, cy + 2.0), (cx, cy + arm))):
        ln = ((xb - xa) ** 2 + (yb - ya) ** 2) ** 0.5
        g += ["G0 Z%.2f F600" % (h + 1.0), "G0 X%.3f Y%.3f F9000" % (xa, ya),
              "G0 Z%.2f F600" % h, "G1 E0.8 F2400",
              "G1 X%.3f Y%.3f E%.5f F1200" % (xb, yb, ln * epm),
              "G1 E-0.8 F2400"]
    g += ["G0 Z%.2f F600" % z2, "M106 S0",
          "G0 X%.3f Y%.3f F9000" % (cx, cy - arm - 20.0),
          "M104 S150", "M106 S255",
          "TEMPERATURE_WAIT SENSOR=extruder MAXIMUM=155", "M106 S0"]
    return {"gcode": g,
            "test": {"cx": cx, "cy": cy, "arm": arm, "width": width,
                     "height": h},
            "segments": _segments(g),
            "photos": [{"z": z, "x": cx, "y": cy} for z in (z1, z2)]}
