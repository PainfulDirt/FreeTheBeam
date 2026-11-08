"""G-code planners for the LiDAR camera calibrations (runs on the helper;
pure Python). The Klipper plugin (lidar.py) passes the parameters, checks the returned paths
against its travel / keep-out / skew (it has the Klipper objects), writes
the G-code and runs it.

plan_pa_flow: LIDAR_PA_FLOW_CALIBRATE, one print for pressure advance (photo
widths at the speed changes of 9 PA lines) and flow (photo widths of 4
triplets of lines at 90/100/110 %, the gap from bare LiDAR strips).
plan_offset_cross: LIDAR_OFFSET_CALIBRATE's 1-layer cross.
Background: docs/camera.md

This file may be distributed under the terms of the GNU GPLv3 license.
"""
H_NOM, W_NOM = 0.2, 0.45
PA_PITCH, PA_LINES = 2.5, 9
PA_SEGS = [(15.0, 20.0), (20.0, 150.0), (15.0, 20.0), (20.0, 150.0),
           (15.0, 20.0), (20.0, 150.0), (15.0, 20.0)]
TRI_PITCH, GROUP_PITCH, GROUPS, TRI_LEN = 1.5, 6.0, 4, 50.0


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


def plan_pa_flow(p):
    X, Y = float(p.get("x", 100.0)), float(p.get("y", 35.0))
    temp, bed = float(p["extruder_temp"]), float(p["bed_temp"])
    flows = [float(f) for f in p.get("flows", [0.9, 1.0, 1.1])]
    speed = float(p.get("speed", 40.0))
    pa_keep = float(p.get("pa_keep", 0.0))
    epm = H_NOM * W_NOM / float(p["fil_area"])
    pas = [round(float(p.get("pa_start", 0.0)) + i * float(p.get("pa_step", 0.01)), 4)
           for i in range(PA_LINES)]
    groups = []
    for k in range(GROUPS):
        order = flows[-k % len(flows):] + flows[:-k % len(flows)] if k else flows
        gx = X + GROUP_PITCH * k
        groups.append({"k": k, "center": round(gx + TRI_PITCH, 3),
                       "lines": [{"x": round(gx + TRI_PITCH * j, 3), "flow": f}
                                 for j, f in enumerate(order)]})
    y0p = Y + 65.0
    y1p = y0p + sum(s[0] for s in PA_SEGS)
    dirs = [1 if i % 2 == 0 else -1 for i in range(PA_LINES)]
    changes, yy = [], y0p
    for seglen, _ in PA_SEGS[:-1]:
        yy += seglen
        changes.append(yy)
    test = {
        "version": 1,
        "pa": {"x0": X, "pitch": PA_PITCH, "y0": y0p, "y1": y1p, "pa": pas,
               "dirs": dirs, "segments": PA_SEGS, "fast": 150.0, "slow": 20.0,
               "height": H_NOM, "width": W_NOM},
        "flow": {"y0": Y, "y1": Y + TRI_LEN, "speed": speed, "height": H_NOM,
                 "width": W_NOM, "groups": groups},
        "strips": [{"name": "pf_g%d" % g["k"], "x": g["center"], "y0": Y - 2.0,
                    "y1": Y + TRI_LEN + 2.0} for g in groups],
        "pa_strip": {"name": "pf_pa", "x": X + 10.0, "y0": y0p - 3.0,
                     "y1": y1p + 3.0},
        "photos": [{"name": "F%d" % (i + 1), "x": X + 10.5, "y": Y + dy}
                   for i, dy in enumerate((12.0, 25.0, 38.0))]
        + [{"name": "T%d" % (i + 1), "x": X + 10.0, "y": c}
           for i, c in enumerate(changes)],
        "plate_photo": {"x": X + 10.0, "y": Y + 125.0},
        "filament_name": p.get("filament_name", ""),
        "filament_type": p.get("filament_type", ""),
        "extruder_temp": temp, "bed_temp": bed, "pa_keep": pa_keep,
    }
    x0o, y0o, x1o, y1o = X - 14.0, Y - 2.0, X + 24.0, y1p + 2.0
    g = ["; LiDAR PA + flow calibration (helper/lidar_pa_flow.py)",
         "; PA %s, flows %s" % (" ".join("%.3f" % v for v in pas),
                                "/".join("%.0f" % (f * 100) for f in flows)),
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
         "G1 X%.3f Y%.3f E%.5f F1200" % (X - 12.0, Y + TRI_LEN,
                                         TRI_LEN * epm * 1.2),
         "G1 E-0.8 F2400"]
    for grp in groups:
        for ln in grp["lines"]:
            g += ["G0 Z1 F600", "G0 X%.3f Y%.3f F9000" % (ln["x"], Y),
                  "G0 Z%.3f F600" % H_NOM, "G1 E0.8 F2400",
                  "G1 X%.3f Y%.3f E%.5f F%.0f" % (
                      ln["x"], Y + TRI_LEN, TRI_LEN * epm * ln["flow"],
                      speed * 60),
                  "G1 E-0.8 F2400"]
    g += ["G0 Z1 F600", "G0 X%.3f Y%.3f F9000" % (X, y0p),
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
    g += ["G1 E-0.8 F2400", "G0 Z3 F600",
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
    lsegs = [(s["x"], s["y0"], s["x"], s["y1"])
             for s in test["strips"] + [test["pa_strip"]]]
    lsegs += [(ph["x"], ph["y"], ph["x"], ph["y"])
              for ph in test["photos"] + [test["plate_photo"]]]
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
