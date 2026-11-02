"""Run klipper/nozzle_wipe.py against a fake Klipper: print the G-code, check
that every move stays on the strip and the mesh comes back."""
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import nozzle_wipe as nw  # noqa: E402


class Err(Exception):
    pass


class Cfg:
    def __init__(s, p, d):
        s.p, s.d = p, d

    def get_printer(s):
        return s.p

    def getfloat(s, k, default=None, **kw):
        return float(s.d.get(k, default))

    def getint(s, k, default=None, **kw):
        return int(s.d.get(k, default))

    def getboolean(s, k, default=None):
        return s.d.get(k, default)

    def getchoice(s, k, choices, default=None):
        return choices[s.d.get(k, default)]

    def error(s, m):
        return Err(m)


class Obj:
    pass


class Live:
    """last_z_result read from the fake printer when asked, like Klipper's."""

    def __init__(s, p, attr):
        s._p, s._a = p, attr

    @property
    def last_z_result(s):
        return getattr(s._p, s._a)


class P:
    command_error = Err

    def __init__(s):
        s.pos = [153.0, 153.0, 5.0]
        s.temp = s.target = 150.0
        s.mesh = "MESH"
        s.lz = s.pz = 0.0
        s.track, s.log = [], []

    def surf(s, x, y):
        return 0.08 - 0.0001 * (x - 40)

    def get_reactor(s):
        r = Obj()
        r.monotonic = lambda: 0.0
        return r

    def lookup_object(s, n, default=None):
        o = Obj()
        if n == "gcode":
            o.register_command = lambda *a, **k: None
            o.run_script_from_command = s.run
        elif n == "extruder":
            h = Obj()
            h.get_temp = lambda t: (s.temp, s.target)
            h.target_temp = s.target
            o.get_heater = lambda: h
        elif n == "bed_mesh":
            o.get_mesh = lambda: s.mesh
            o.set_mesh = lambda m: setattr(s, "mesh", m)
        elif n == "load_cell_probe":
            o.get_printer_probe = lambda: Live(s, "lz")
        elif n == "probe":
            o = Live(s, "pz")
            o.get_offsets = lambda: (0.0, 20.3, 0.831)
        elif n == "toolhead":
            o.get_position = lambda: list(s.pos)

            def mm(p, sp):
                for i, v in enumerate(p):
                    if v is not None:
                        s.pos[i] = v
            o.manual_move = mm
        elif n == "gcode_move":
            o.reset_last_position = lambda: None
        return o

    def run(s, script):
        for line in script.split("\n"):
            s.log.append(line)
            if line.startswith(("G0", "G1")):
                for ax, i in (("X", 0), ("Y", 1), ("Z", 2)):
                    m = re.search(r"\b" + ax + r"(-?[\d.]+)", line)
                    if m:
                        s.pos[i] = float(m.group(1))
                s.track.append((line, tuple(s.pos), s.mesh))
            if line.startswith("LOAD_CELL_PROBE"):
                s.lz = s.surf(s.pos[0], s.pos[1])
            if line.startswith("PROBE"):
                # pin 20.3mm behind the nozzle
                s.pz = s.surf(s.pos[0], s.pos[1] + 20.3) + 0.831
            m = re.match(r"M10[49] S(\d+)", line)
            if m:
                s.temp = s.target = float(m.group(1))


class Gcmd:
    def __init__(s, d):
        s.d = d

    def get_float(s, k, default=None, **kw):
        return float(s.d.get(k, default))

    def get(s, k, default=None):
        return s.d.get(k, default)

    def respond_info(s, m):
        print("  //", m)

    def error(s, m):
        return Err(m)


CFG = dict(x_min=40, x_max=100, y_min=297.4, y_max=298.6, wipe_len=25,
           peel_len=25, touch_temp=150, cool_temp=160)


def check(p, label):
    low = [t for t in p.track if t[1][2] < 2.0]
    xs = [t[1][0] for t in low]
    ys = [t[1][1] for t in low]
    ok = (min(xs) >= 40 and max(xs) <= 100 and min(ys) >= 297.4 - 20.3
          and max(ys) <= 298.6 and p.mesh == "MESH"
          and all(t[2] is None for t in p.track))
    print("%s: low moves X %.1f-%.1f Y %.2f-%.2f, mesh back %s -> %s" % (
        label, min(xs), max(xs), min(ys), max(ys), p.mesh, "OK" if ok
        else "FAIL"))
    return ok


def main():
    good = True
    for sensor in ("nozzle", "probe"):
        for blob in (1.0, 0.0):
            p = P()
            w = nw.NozzleWipe(Cfg(p, dict(CFG, blob=blob)))
            print("== SENSOR=%s blob=%s" % (sensor, blob))
            w.cmd_NOZZLE_WIPE(Gcmd({"TEMP": 215, "SENSOR": sensor}))
            good &= check(p, "  check")
            if sensor == "nozzle" and blob:
                print("\n".join("    " + l for l in p.log))
    p = P()
    w = nw.NozzleWipe(Cfg(p, CFG))
    w.cmd_NOZZLE_WIPE(Gcmd({"TEMP": 215}))
    p.pos, p.log, p.track = [150.0, 170.0, 0.5], [], []
    w.cmd_NOZZLE_SCRUB(Gcmd({}))
    back = p.pos == [150.0, 170.0, 0.5]
    touches = sum("PROBE" in l for l in p.log)
    print("== NOZZLE_SCRUB: back at start %s, touches %d -> %s" % (
        back, touches, "OK" if back and not touches else "FAIL"))
    good &= back and not touches
    print("ALL OK" if good else "FAILED")
    sys.exit(0 if good else 1)


class TestNozzleWipe(unittest.TestCase):
    def test_moves_stay_on_strip(self):
        with self.assertRaises(SystemExit) as cm:
            main()
        self.assertEqual(cm.exception.code, 0)


if __name__ == "__main__":
    main()
