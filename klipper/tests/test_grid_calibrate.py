"""LIDAR_GRID_CALIBRATE (klipper/lidar.py) against a synthetic printer: the
bed has a shape, the nozzle reads it with tap noise and the odd bad tap, the
LiDAR reads it plus a known X curve and Y curve, texture noise per spot and a
drift in time. The command must give back both curves, reject bad taps and
switch the bed heater off only around the touches."""
import ast
import math
import os
import random
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import lidar  # noqa: E402


class Err(Exception):
    pass


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Gcmd:
    def __init__(self, **p):
        self.p = dict((k.upper(), str(v)) for k, v in p.items())

    def get(self, k, default=None):
        return self.p.get(k, default)

    def get_float(self, k, default=None, **kw):
        return float(self.p[k]) if k in self.p else default

    def get_int(self, k, default=None, **kw):
        return int(self.p[k]) if k in self.p else default

    def respond_info(self, m):
        print("  //", m)

    def error(self, m):
        return Err(m)


def bed(x, y):
    return 0.08 * math.sin(x / 60.0) + 0.05 * math.cos(y / 45.0) \
        + 0.0003 * x


def ex(x):      # X twist the LiDAR sees (mm)
    return 0.1 - 0.2 * (x / 270.0) + 0.03 * math.sin(x / 40.0)


def ey(y):      # its error along Y
    return 0.00022 * y


def main():
    rnd = random.Random(5)
    t = [0.0]
    pos = [0.0, 0.0]
    saved = {}
    log = []
    tex = {}

    def run(script):
        for line in script.split("\n"):
            w = line.split()
            if not w:
                continue
            log.append(line)
            if w[0] == "G0":
                d = dict((a[0], a[1:]) for a in w[1:])
                if "X" in d:
                    pos[0] = float(d["X"])
                if "Y" in d:
                    pos[1] = float(d["Y"])
                t[0] += 1.0
            elif w[0] == "LOAD_CELL_PROBE":
                t[0] += 3.0
                z = bed(*pos) + rnd.gauss(0, 0.002)
                if rnd.random() < 0.12:
                    z += 0.04      # a bad tap
                lc.last_z_result = z
            elif w[0] == "SAVE_VARIABLE":
                d = dict(a.split("=", 1) for a in w[1:])
                saved[d["VARIABLE"]] = ast.literal_eval(d["VALUE"].strip('"'))

    def twist_reading(x, y, z=None):
        t[0] += 2.0
        # the same spot's texture is the same every pass
        k = (round(x, 1), round(y, 1))
        tex.setdefault(k, rnd.gauss(0, 0.012))
        return bed(x, y) + ex(x) + ey(y) + tex[k] + 0.0001 * t[0] - 25.6

    lc = Obj(last_z_result=None)
    mesh = [None]
    objs = {
        "toolhead": Obj(get_status=lambda e: {
            "homed_axes": "xyz", "axis_minimum": [-2.0, -2.0, -10.0, 0.0],
            "axis_maximum": [306.5, 306.0, 302.0, 0.0]},
            get_extruder=lambda: Obj(get_heater=lambda: Obj(
                get_status=lambda e: {"temperature": 150.0}))),
        "save_variables": Obj(allVariables=saved),
        "bed_mesh": Obj(get_mesh=lambda: mesh[0],
                        set_mesh=lambda m: mesh.__setitem__(0, m)),
        "print_stats": Obj(get_status=lambda e: {"state": "standby"}),
        "gcode_move": Obj(get_status=lambda e: {"homing_origin": [0, 0, 0]}),
        "load_cell_probe": Obj(get_printer_probe=lambda: lc),
        "heater_bed": Obj(get_status=lambda e: {"temperature": 70.0,
                                                 "target": 70.0}),
    }
    L = object.__new__(lidar.Lidar)
    L.printer = Obj(lookup_object=lambda n, d=None: objs.get(n, d),
                    command_error=Err)
    L.reactor = Obj(monotonic=lambda: t[0], pause=lambda w: w)
    L.gcode = Obj(run_script_from_command=run)
    L.service, L.scan_z = "pi:7131", 3.0
    L.x_offset, L.y_offset, L.offset_z = -36.013, -21.336, 3.0
    L.x_per_z, L.y_per_z = 0.0354, -0.986
    L.twist_reading = twist_reading

    print("LIDAR_GRID_CALIBRATE COUNT=7,5:")
    L.cmd_LIDAR_GRID_CALIBRATE(Gcmd(count="7,5"))
    ok = True
    tw, yt = saved.get("lidar_twist"), saved.get("lidar_ytwist")
    ok &= tw is not None and yt is not None and "lidar_grid" in saved
    xs, ys = tw["xs"], yt["ys"]
    exm = sum(ex(x) for x in xs) / len(xs)
    eym = sum(ey(y) for y in ys) / len(ys)
    errx = max(abs(c - (ex(x) - exm)) for x, c in zip(xs, tw["comp"]))
    erry = max(abs(c - (ey(y) - eym)) for y, c in zip(ys, yt["comp"]))
    print("X curve error max %.1f um, Y curve error max %.1f um"
          " (texture 12 um per spot)" % (errx * 1e3, erry * 1e3))
    # texture averages over 5 rows / 7 columns: ~5-9 um
    ok &= errx < 0.012 and erry < 0.010
    offs = sum(1 for line in log if "TARGET=0" in line)
    ons = sum(1 for line in log if "TARGET=70.0" in line)
    taps = sum(1 for line in log if line.startswith("LOAD_CELL_PROBE"))
    print("bed heater off %d / back on %d times, %d taps for 70 points"
          % (offs, ons, taps))
    ok &= offs == ons == taps - 1      # all but the throw-away first tap
    ok &= taps > 1 + 70 * 3           # bad taps were re-tapped
    st = saved.get("lidar_cal_by_temp") or {}
    print("kept per temperature: %s" % (sorted(st),))
    runs = st.get("70", {}).get("runs") or [{}]
    ok &= list(st) == ["70"] and len(runs) == 1 \
        and runs[0]["twist"]["xs"] == xs and runs[0]["ytwist"]["ys"] == ys \
        and runs[0]["twist"]["w"] == 5 and runs[0]["ytwist"]["w"] == 7
    print("rest after both curves rms %.1f um" % (
        saved["lidar_grid"]["rms"] * 1e3))
    print("OK" if ok else "FAILED")
    sys.exit(0 if ok else 1)


class TestGridCalibrate(unittest.TestCase):
    def test_recovers_both_curves(self):
        with self.assertRaises(SystemExit) as cm:
            main()
        self.assertEqual(cm.exception.code, 0)


if __name__ == "__main__":
    main()
