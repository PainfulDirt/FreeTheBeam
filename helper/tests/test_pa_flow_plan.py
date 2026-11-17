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


BANDS = [0.92, 0.96, 1.0, 1.04, 1.08, 1.12]


def plan(tests, method="ir", **kw):
    return pf.plan_pa_flow(dict(P, tests=tests, flow_method=method,
                                flows=BANDS, **kw))


class PaFlowPlanTests(unittest.TestCase):
    """The combined print: bands at Y..Y+10, PA lines from Y+25."""
    def setUp(self):
        self.plan = plan(["pa", "flow"])
        self.g = self.plan["gcode"]

    def test_order_and_steps(self):
        g = self.g
        i_start = first(g, "START_PRINT")
        # the bare step runs inside START_PRINT (_LIDAR_PRINT_SCAN hook),
        # before START_PRINT heats up and purges
        self.assertNotIn("LIDAR_PA_FLOW_STEP STEP=BARE", g)
        i_photos = g.index("LIDAR_PA_FLOW_STEP STEP=PHOTOS")
        i_end = g.index("END_PRINT")
        i_an = g.index("LIDAR_PA_FLOW_STEP STEP=ANALYZE")
        i_band = first(g, "G1 X100.204 Y45.000")
        i_pa = first(g, "SET_PRESSURE_ADVANCE ADVANCE=0.0000")
        self.assertTrue(first(g, "EXCLUDE_OBJECT_DEFINE") < i_start < i_band
                        < i_pa < i_photos < i_end < i_an)
        self.assertIn("TEMPERATURE_WAIT SENSOR=extruder MAXIMUM=155", g[i_pa:i_photos])
        self.assertEqual(g[i_start + 1], "SET_PRINT_STATS_INFO CURRENT_LAYER=1")
        self.assertIn("M221 S100", g[i_start:i_band])

    def test_filament_quoting(self):
        g = plan(["pa"], filament_name='PLA "Matte" @My Printer')["gcode"]
        sp = g[first(g, "START_PRINT")]
        self.assertIn("FILAMENT_NAME=\"PLA 'Matte' @My Printer\"", sp)
        self.assertIn('FILAMENT_TYPE="PLA"', sp)
        self.assertIn("EXTRUDER_TEMP=210 BED_TEMP=60", sp)

    def test_band_extrusion(self):
        sp = 0.45 - 0.2 * (1 - math.pi / 4)
        epm = sp * 0.2 / FIL
        bands = self.plan["test"]["flow"]["bands"]
        self.assertEqual([b["flow"] for b in bands], BANDS)
        self.assertEqual([(b["x0"], b["x1"], b["y0"], b["y1"]) for b in bands],
                         [(100.0 + 18 * i, 114.0 + 18 * i, 35.0, 45.0)
                          for i in range(6)])
        for b in bands:
            lines = [m for m in moves(self.g) if m[3] > 0 and m[2] == 0.2
                     and b["x0"] < m[0] < b["x1"] and m[1] in (35.0, 45.0)
                     and abs(m[3] - 10.0 * epm * b["flow"]) < 1e-5]
            self.assertEqual(len(lines), int(14.0 / sp), b)

    def test_pa_layout(self):
        pas = [float(l.split("=")[1]) for l in self.g
               if l.startswith("SET_PRESSURE_ADVANCE ADVANCE=")]
        self.assertEqual([round(v, 4) for v in pas[1:10]],
                         [round(0.01 * i, 4) for i in range(9)])
        pa = self.plan["test"]["pa"]
        self.assertEqual((pa["x0"], pa["pitch"], pa["y0"], pa["y1"]),
                         (100.0, 2.5, 60.0, 180.0))
        i_pa = first(self.g, "SET_PRESSURE_ADVANCE ADVANCE=0.0000")
        prints = [m for m in moves(self.g[i_pa:]) if m[3] > 0]
        self.assertGreaterEqual(min(m[1] for m in prints), 60.0)
        self.assertAlmostEqual(min(m[0] for m in prints), 100.0)
        self.assertAlmostEqual(max(m[0] for m in prints), 120.0)

    def test_photos(self):
        t = self.plan["test"]
        b = [(p["name"], p["x"], p["y"], p.get("exposure"))
             for p in t["photos"] if p["name"].startswith("B")]
        tr = [(p["x"], p["y"]) for p in t["photos"] if p["name"].startswith("T")]
        self.assertEqual(b, [("B%d" % (i + 1), 107.0 + 18 * i, 40.0, 1000)
                             for i in range(6)])
        self.assertEqual(tr, [(110.0, y) for y in (75.0, 95.0, 110.0, 130.0, 145.0, 165.0)])
        self.assertEqual(t["strips"], [])          # camera method: no scans
        self.assertEqual(t["tests"], ["pa", "flow"])
        self.assertEqual(t["area"], [86.0, 33.0, 206.0, 182.0])
        self.assertTrue(self.plan["segments"])
        self.assertTrue(self.plan["laser_segments"])


class SplitPlanTests(unittest.TestCase):
    def test_pa_only(self):
        p = plan(["pa"])
        t = p["test"]
        self.assertNotIn("flow", t)
        self.assertEqual((t["pa"]["y0"], t["pa"]["y1"]), (35.0, 155.0))
        self.assertFalse([ph for ph in t["photos"] if ph["name"].startswith("B")])
        self.assertEqual(len(t["photos"]), 6)
        self.assertEqual(t["area"], [86.0, 33.0, 124.0, 157.0])
        self.assertIn("pa_strip", t)

    def test_flow_only_camera(self):
        p = plan(["flow"])
        t = p["test"]
        self.assertNotIn("pa", t)
        self.assertNotIn("pa_strip", t)
        self.assertFalse(any(l.startswith("SET_PRESSURE_ADVANCE ADVANCE=0.0000")
                             for l in p["gcode"]))
        self.assertEqual([ph["name"] for ph in t["photos"]],
                         ["B%d" % i for i in range(1, 7)])
        self.assertEqual(t["area"], [86.0, 33.0, 206.0, 87.0])
        self.assertEqual(t["flow"]["method"], "ir")

    def test_flow_lidar_strips(self):
        t = plan(["flow"], "lidar")["test"]
        self.assertEqual(t["photos"], [])
        self.assertEqual([(s["name"], s["x"], s["y0"], s["y1"]) for s in t["strips"]],
                         [("pf_b%d" % (i + 1), 107.0 + 18 * i, 33.0, 47.0)
                          for i in range(6)])
        self.assertEqual(t["flow"]["method"], "lidar")

    def test_flow_benchmark_both(self):
        t = plan(["flow"], "both")["test"]
        self.assertEqual(len(t["photos"]), 6)
        self.assertEqual(len(t["strips"]), 6)

    def test_pa_lidar_no_photos(self):
        t = pf.plan_pa_flow(dict(P, tests=["pa"], pa_method="lidar"))["test"]
        self.assertEqual(t["pa"]["method"], "lidar")
        self.assertEqual(t["photos"], [])
        self.assertEqual((t["pa_strip"]["y0"], t["pa_strip"]["y1"]), (32.0, 158.0))

    def test_pa_both_keeps_photos(self):
        t = pf.plan_pa_flow(dict(P, tests=["pa", "flow"], pa_method="both",
                                 flow_method="both"))["test"]
        self.assertEqual(t["pa"]["method"], "both")
        self.assertEqual(len([p for p in t["photos"] if p["name"][0] == "T"]), 6)
        self.assertEqual(t["pa"]["method"], "both")
        self.assertEqual(plan(["pa"])["test"]["pa"]["method"], "ir")   # default

    def test_bands_all_inside_laser_segments(self):
        p = plan(["flow"], "both")
        pts = set((s[0], s[1]) for s in p["laser_segments"])
        for b in p["test"]["flow"]["bands"]:
            self.assertIn(((b["x0"] + b["x1"]) / 2, 40.0), pts)


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
