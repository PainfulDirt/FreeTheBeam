"""helper/lidar_pa_flow.py: G-code planners for LIDAR_PA_FLOW_CALIBRATE and
the 1-layer offset cross (pure Python, runs on the helper)."""
import math
import os
import re
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import lidar_pa_flow as pf  # noqa: E402

FIL = math.pi * 1.75 ** 2 / 4
P = {"extruder_temp": 210, "bed_temp": 60, "filament_name": "PLA Matte @My Printer",
     "filament_type": "PLA", "x": 100.0, "y": 35.0, "pa_start": 0.0,
     "pa_step": 0.01, "flows": [0.9, 1.0, 1.1], "speed": 40.0,
     "pa_keep": 0.0475, "fil_area": FIL, "max_accel": 20000.0}


def moves(g):
    """(x, y, z, e, line) of every G0/G1 with an absolute position walk."""
    x = y = z = 0.0
    out = []
    for l in g:
        if not l.startswith(("G0", "G1")):
            continue
        p = dict((k, float(v)) for k, v in re.findall(r"([XYZEF])(-?[\d.]+)", l))
        x, y, z = p.get("X", x), p.get("Y", y), p.get("Z", z)
        out.append((x, y, z, p.get("E", 0.0), l))
    return out


def first(g, prefix):
    return next(i for i, l in enumerate(g) if l.startswith(prefix))


class PaFlowPlanTests(unittest.TestCase):
    def setUp(self):
        self.plan = pf.plan_pa_flow(dict(P))
        self.g = self.plan["gcode"]

    def test_order_and_steps(self):
        g = self.g
        i_start = first(g, "START_PRINT")
        # the bare step runs inside START_PRINT (_LIDAR_PRINT_SCAN hook),
        # before START_PRINT heats up and purges
        self.assertNotIn("LIDAR_PA_FLOW_STEP STEP=BARE", g)
        i_bare = i_start + 1
        i_photos = g.index("LIDAR_PA_FLOW_STEP STEP=PHOTOS")
        i_end = g.index("END_PRINT")
        i_an = g.index("LIDAR_PA_FLOW_STEP STEP=ANALYZE")
        i_tri = next(i for i, l in enumerate(g) if l.startswith("G1 X100.000 Y85.000"))
        i_pa = first(g, "SET_PRESSURE_ADVANCE ADVANCE=0.0000")
        self.assertTrue(first(g, "EXCLUDE_OBJECT_DEFINE") < i_start < i_bare
                        < i_tri < i_pa < i_photos < i_end < i_an)
        self.assertIn("TEMPERATURE_WAIT SENSOR=extruder MAXIMUM=155", g[i_pa:i_photos])
        self.assertEqual(g[i_start + 1], "SET_PRINT_STATS_INFO CURRENT_LAYER=1")
        self.assertIn("M221 S100", g[i_start:i_tri])

    def test_filament_quoting(self):
        p = dict(P, filament_name='PLA "Matte" @K1 Max')
        g = pf.plan_pa_flow(p)["gcode"]
        sp = g[first(g, "START_PRINT")]
        self.assertIn("FILAMENT_NAME=\"PLA 'Matte' @K1 Max\"", sp)
        self.assertIn('FILAMENT_TYPE="PLA"', sp)

    def test_triplet_extrusion(self):
        epm = 0.2 * 0.45 / FIL
        lines = [m for m in moves(self.g) if m[3] > 0 and m[2] == 0.2
                 and abs(m[1] - 85.0) < 1e-6 and 99 < m[0] < 122]
        self.assertEqual(len(lines), 12)
        xs = sorted(m[0] for m in lines)
        self.assertEqual([round(v, 3) for v in xs],
                         [100.0, 101.5, 103.0, 106.0, 107.5, 109.0,
                          112.0, 113.5, 115.0, 118.0, 119.5, 121.0])
        flows = dict((g["center"], [l["flow"] for l in g["lines"]])
                     for g in self.plan["test"]["flow"]["groups"])
        self.assertEqual(sorted(flows), [101.5, 107.5, 113.5, 119.5])
        want = dict((ln["x"], ln["flow"]) for g in self.plan["test"]["flow"]["groups"]
                    for ln in g["lines"])
        for x, y, z, e, l in lines:
            self.assertAlmostEqual(e / 50.0, epm * want[round(x, 3)], places=5)
        self.assertNotEqual(flows[101.5], flows[107.5])   # order rotated

    def test_pa_layout(self):
        pas = [float(l.split("=")[1]) for l in self.g
               if l.startswith("SET_PRESSURE_ADVANCE ADVANCE=")]
        self.assertEqual([round(v, 4) for v in pas[1:10]],
                         [round(0.01 * i, 4) for i in range(9)])
        pa = self.plan["test"]["pa"]
        self.assertEqual((pa["x0"], pa["pitch"], pa["y0"], pa["y1"]),
                         (100.0, 2.5, 100.0, 220.0))
        prints = [m for m in moves(self.g) if m[3] > 0 and m[1] >= 100.0]
        self.assertAlmostEqual(min(m[0] for m in prints), 100.0)
        self.assertAlmostEqual(max(m[0] for m in prints), 120.0)

    def test_photos_and_strips(self):
        t = self.plan["test"]
        f = [(p["x"], p["y"]) for p in t["photos"] if p["name"].startswith("F")]
        tr = [(p["x"], p["y"]) for p in t["photos"] if p["name"].startswith("T")]
        self.assertEqual(f, [(110.5, 47.0), (110.5, 60.0), (110.5, 73.0)])
        self.assertEqual(tr, [(110.0, y) for y in (115.0, 135.0, 150.0, 170.0, 185.0, 205.0)])
        self.assertEqual([(s["name"], s["x"], s["y0"], s["y1"]) for s in t["strips"]],
                         [("pf_g%d" % k, x, 33.0, 87.0)
                          for k, x in enumerate((101.5, 107.5, 113.5, 119.5))])
        self.assertEqual((t["plate_photo"]["x"], t["plate_photo"]["y"]), (110.0, 160.0))
        self.assertTrue(self.plan["segments"])
        self.assertTrue(self.plan["laser_segments"])


class CrossPlanTests(unittest.TestCase):
    def test_cross_one_layer(self):
        plan = pf.plan_offset_cross({"cx": 150.0, "cy": 150.0, "fil_area": FIL})
        prints = [m for m in moves(plan["gcode"]) if m[3] > 0 and "E0.8" not in m[4]]
        self.assertTrue(prints)
        self.assertTrue(all(abs(m[2] - 0.2) < 1e-9 for m in prints))
        xs = [m[0] for m in moves(plan["gcode"]) if m[2] <= 0.2 + 1e-9]
        ys = [m[1] for m in moves(plan["gcode"]) if m[2] <= 0.2 + 1e-9]
        self.assertAlmostEqual(min(xs), 143.0)
        self.assertAlmostEqual(max(xs), 157.0)
        self.assertAlmostEqual(min(ys), 143.0)
        self.assertAlmostEqual(max(ys), 157.0)
        epm = 0.2 * 0.6 / FIL
        x = y = None
        for mx, my, mz, e, l in moves(plan["gcode"]):
            if e > 0 and "E0.8" not in l and x is not None:
                self.assertAlmostEqual(e / math.hypot(mx - x, my - y), epm, places=5)
            x, y = mx, my
        self.assertEqual(plan["test"], {"cx": 150.0, "cy": 150.0, "arm": 7.0,
                                        "width": 0.6, "height": 0.2})
        self.assertEqual([p["z"] for p in plan["photos"]], [3.0, 5.0])


if __name__ == "__main__":
    unittest.main()
