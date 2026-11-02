"""klipper/lidar.py on printers other than the K1 Max: the laser area from
the printer's travel, calibration defaults from it, the touch methods and
LIDAR_SETUP_CHECK. The K1 Max values must stay what v0.2.0 hard-coded."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_klipper import Err, Gcmd, K1, SMALL, make_lidar  # noqa: E402


class TestGeometry(unittest.TestCase):
    def test_k1_area_unchanged(self):
        L, log, objs = make_lidar(**K1)
        self.assertEqual(L._laser_area(3.0), (5.0, 270.0, 5.0, 265.0))

    def test_small_bed_area(self):
        L, log, objs = make_lidar(**SMALL)
        self.assertEqual(L._laser_area(3.0), (0.5, 193.5, 0.5, 203.3))

    def test_positive_offset_area(self):
        L, log, objs = make_lidar(axis_min=(0.0, 0.0), axis_max=(230.0, 225.0),
                                  offset=(30.0, 20.0))
        self.assertEqual(L._laser_area(3.0), (30.5, 229.5, 20.5, 224.5))

    def test_options_only_narrow(self):
        L, log, objs = make_lidar(mesh_min=(-50.0, -50.0),
                                  mesh_max=(400.0, 400.0), **SMALL)
        self.assertEqual(L._laser_area(3.0), (0.5, 193.5, 0.5, 203.3))

    def test_empty_area_refused(self):
        L, log, objs = make_lidar(axis_min=(0.0, 0.0), axis_max=(30.0, 30.0),
                                  offset=(-36.0, 0.0))
        with self.assertRaises(Err) as cm:
            L._area_or_error(Gcmd(), 3.0)
        self.assertIn("reaches no part of the bed", str(cm.exception))

    def test_k1_calibration_defaults(self):
        L, log, objs = make_lidar(twist_y=145.0, ytwist_x=[100.0, 200.0],
                                  load_cell=True, **K1)
        g = Gcmd()
        self.assertEqual(L._twist_spots(g, "X", 3.0),
                         [(x, 145.0) for x in L._grid(5.0, 270.0, 7)])
        spots = L._twist_spots(g, "Y", 3.0)
        ys = L._grid(6.0, 264.0, 9)
        self.assertEqual(spots, [(100.0, y) for y in ys]
                         + [(200.0, y) for y in ys[::-1]])
        self.assertEqual(L._grid_axes(g, 3.0, 7, 5),
                         (L._grid(10.0, 270.0, 7), L._grid(6.0, 264.0, 5)))

    def test_small_bed_calibration_defaults(self):
        L, log, objs = make_lidar(**SMALL)
        g = Gcmd()
        spots = L._twist_spots(g, "X", 3.0)
        self.assertEqual({y for x, y in spots}, {101.9})
        self.assertEqual(spots[0][0], 0.5)
        xs = sorted({x for x, y in L._twist_spots(g, "Y", 3.0)})
        self.assertEqual(xs, [64.8, 129.2])


class TestTouch(unittest.TestCase):
    def test_auto_mode(self):
        self.assertEqual(make_lidar(load_cell=True, **SMALL)[0]._touch_mode(),
                         "load_cell")
        self.assertEqual(make_lidar(probe_offsets=(0.0, 20.3),
                                    **SMALL)[0]._touch_mode(), "probe")
        self.assertEqual(make_lidar(**SMALL)[0]._touch_mode(), "none")
        self.assertEqual(make_lidar(load_cell=True, touch="probe",
                                    probe_offsets=(0.0, 0.0),
                                    **SMALL)[0]._touch_mode(), "probe")

    def test_none_refuses_before_moving(self):
        L, log, objs = make_lidar(**SMALL)
        for cmd in (L.cmd_LIDAR_TWIST_CALIBRATE, L.cmd_LIDAR_GRID_CALIBRATE):
            with self.assertRaises(Err) as cm:
                cmd(Gcmd())
            self.assertIn("needs a touch method ([lidar] touch:)",
                          str(cm.exception))
        self.assertEqual(log, [])

    def test_probe_touch_moves_probe_over_spot(self):
        L, log, objs = make_lidar(probe_offsets=(0.0, 20.3), **SMALL)
        out = L.touch_grid(Gcmd(), [(100.0, 100.0)], 1, 0, ref=(110.0, 110.0))
        self.assertIn("G0 X100.00 Y79.70 F9000", log)
        self.assertIn("PROBE SAMPLES=1 SAMPLES_RESULT=median", log)
        self.assertNotIn("LOAD_CELL_PROBE SAMPLES=1", log)
        self.assertEqual([p[:3] for p in out if not p[4]],
                         [(100.0, 100.0, 0.0)])

    def test_probe_touch_out_of_travel_refused(self):
        L, log, objs = make_lidar(probe_offsets=(0.0, 20.3), **SMALL)
        with self.assertRaises(Err) as cm:
            L.touch_grid(Gcmd(), [(100.0, 5.0)], 1, 0, ref=(110.0, 110.0))
        self.assertIn("outside its travel", str(cm.exception))
        self.assertEqual(log, [])

    def _nozzle_touch(self, values):
        vals = list(values)

        def on_gcode(line, objs):
            if line.startswith("NOZZLE_TOUCH"):
                objs["nozzle_touch"].status["last_touch_z"] = vals.pop(0)
        return on_gcode

    def test_gcode_touch_reads_result(self):
        L, log, objs = make_lidar(
            touch="gcode", touch_result="nozzle_touch:last_touch_z",
            on_gcode=self._nozzle_touch([0.010, 0.012, 0.011]), **SMALL)
        nt = objs["nozzle_touch"] = type("NT", (), {})()
        nt.status = {}
        nt.get_status = lambda e: dict(nt.status)
        self.assertAlmostEqual(L._touch(3), 0.011)
        self.assertEqual([x for x in log if x == "NOZZLE_TOUCH"],
                         ["NOZZLE_TOUCH"] * 3)

    def test_gcode_touch_missing_result(self):
        for result in ("nozzle_touch:nope", "nothing:last_touch_z", None):
            L, log, objs = make_lidar(
                touch="gcode", touch_result=result,
                on_gcode=self._nozzle_touch([0.01]), **SMALL)
            nt = objs["nozzle_touch"] = type("NT", (), {})()
            nt.status = {}
            nt.get_status = lambda e: dict(nt.status)
            with self.assertRaises(Err) as cm:
                L._touch_ready(Gcmd())
                L._touch(1)
            self.assertIn("touch_result", str(cm.exception))

    def test_zero_ref_fallback_centre(self):
        L, log, objs = make_lidar(load_cell=True, **SMALL)
        L.touch_grid(Gcmd(), [(100.0, 100.0)], 1, 0)
        self.assertEqual([x for x in log if x.startswith("G0 X")][0],
                         "G0 X97.00 Y101.90 F9000")

    def test_touch_temp_and_wipe(self):
        L, log, objs = make_lidar(load_cell=True, touch_temp=170.0,
                                  touch_wipe_gcode="MY_WIPE", **SMALL)
        L._touch_heat(wait=False)
        L._touch_heat(wait=True)
        self.assertEqual(log, ["M104 S170", "M109 S170", "MY_WIPE"])
        self.assertNotIn("gcode_macro _LOAD_CELL_TOUCH_Z", objs["_looked_up"])


class FakeConfig:
    def __init__(self, sections):
        self.sections = sections

    def has_section(self, name):
        return name in self.sections

    def getsection(self, name):
        opts = self.sections[name]
        return type("S", (), {"get": lambda _s, k, d=None: opts.get(k, d)})()


class TestSetupCheck(unittest.TestCase):
    def report(self, L):
        return dict((item, (ok, text)) for item, ok, text in L.setup_report())

    def test_report_small_bed_probe(self):
        L, log, objs = make_lidar(probe_offsets=(0.0, 20.3), **SMALL)
        L.service = None
        L._call = lambda msg, timeout=10.0: {"result": {"exposure": 1000}}
        r = self.report(L)
        self.assertEqual([i for i, ok, t in L.setup_report()][:4],
                         ["driver", "offsets", "timing", "service"])
        self.assertTrue(r["driver"][0])
        self.assertFalse(r["timing"][0])
        self.assertFalse(r["service"][0])
        self.assertIn("need it", r["service"][1])
        self.assertTrue(r["area"][0])
        self.assertIn("X0.5-193.5 Y0.5-203.3", r["area"][1])
        self.assertTrue(r["touch"][0])
        self.assertIn("less accurate", r["touch"][1])

    def test_report_driver_down(self):
        L, log, objs = make_lidar(load_cell=True, **SMALL)

        def down(req, timeout=30.0):
            raise Err("no answer")
        L._svc = down
        r = self.report(L)
        self.assertFalse(r["driver"][0])
        self.assertFalse(r["service"][0])

    def test_report_empty_area(self):
        L, log, objs = make_lidar(axis_min=(0.0, 0.0), axis_max=(30.0, 30.0),
                                  offset=(-36.0, 0.0))
        L._svc = lambda req, timeout=30.0: {"ok": True, "reply": {}}
        r = self.report(L)
        self.assertFalse(r["area"][0])
        self.assertFalse(r["touch"][0])

    def test_default_dirs(self):
        import lidar
        home = os.path.expanduser("~/printer_data")
        cfg = FakeConfig({"virtual_sdcard": {"path": "~/printer_data/gcodes"}})
        self.assertEqual(lidar._default_dirs(cfg),
                         (os.path.join(home, "lidar"),
                          os.path.join(home, "gcodes")))
        self.assertEqual(lidar._default_dirs(FakeConfig({})),
                         ("/usr/data/printer_data/lidar",
                          "/usr/data/printer_data/gcodes"))


class TestMainline(unittest.TestCase):
    def test_trapq_horizon_with_top_level_chelper(self):
        """Mainline Klipper has chelper as a top-level module (klippy/ is on
        sys.path), Kalico as klippy.chelper: both must work."""
        import types
        L, log, objs = make_lidar(**SMALL)

        class Move:
            print_time, move_t = 10.0, 0.5

        class Lib:
            def trapq_extract_old(self, tq, data, n, t0, t1):
                data[0] = Move()
                return 1

        class Ffi:
            def new(self, what):
                return [None]
        fake = types.ModuleType("chelper")
        fake.get_ffi = lambda: (Ffi(), Lib())
        saved = dict((k, sys.modules.get(k)) for k in ("chelper", "klippy"))
        sys.modules["chelper"] = fake
        sys.modules.pop("klippy", None)
        try:
            self.assertEqual(L._trapq_horizon(types.SimpleNamespace(
                trapq=None)), 10.5)
        finally:
            for k, v in saved.items():
                if v is None:
                    sys.modules.pop(k, None)
                else:
                    sys.modules[k] = v


class TestReviewFixes(unittest.TestCase):
    """Findings of the 0.3.0 review."""
    def fits(self, L, pts, z=None):
        L._check_reach(Gcmd(), [tuple(p) * 2 if len(p) == 2 else p
                                for p in pts], z)

    def test_benchmark_area_reachable_by_probe(self):
        L, log, objs = make_lidar(probe_offsets=(0.0, 20.3), load_cell=True,
                                  **K1)
        a = L._bench_area(Gcmd(), ["probe", "1"])
        self.assertGreaterEqual(a["y0"], 18.3 + 0.5)
        self.assertEqual((a["x0"], a["x1"], a["y1"]), (5.0, 270.0, 265.0))
        # without the probe method the whole laser area
        self.assertEqual(L._bench_area(Gcmd(), ["1", "2"])["y0"], 5.0)

    def test_benchmark_checks_touch_first(self):
        L, log, objs = make_lidar(**SMALL)
        with self.assertRaises(Err) as cm:
            L.cmd_MESH_BENCHMARK(Gcmd(methods="1,2"))
        self.assertIn("needs a touch method ([lidar] touch:)",
                      str(cm.exception))
        self.assertEqual(log, [])

    def test_probe_calibration_defaults_reachable(self):
        L, log, objs = make_lidar(probe_offsets=(0.0, 20.3), **SMALL)
        g = Gcmd()
        for axis in ("X", "Y"):
            spots = L._twist_spots(g, axis, 3.0)
            self.fits(L, [L._touch_xy(x, y) for x, y in spots])
        xs, ys = L._grid_axes(g, 3.0, 7, 5)
        self.fits(L, [L._touch_xy(x, y) for x in xs for y in ys])

    def test_probe_touch_approach_above_trigger(self):
        L, log, objs = make_lidar(probe_offsets=(0.0, 20.3, 2.561), **SMALL)
        L.touch_grid(Gcmd(), [(100.0, 100.0)], 1, 0, ref=(110.0, 110.0))
        self.assertIn("G0 Z4.56 F900", log)
        self.assertNotIn("G0 Z2 F900", log)
        L, log, objs = make_lidar(load_cell=True, **SMALL)
        L.touch_grid(Gcmd(), [(100.0, 100.0)], 1, 0, ref=(110.0, 110.0))
        self.assertIn("G0 Z2.00 F900", log)

    def test_setup_reports_zero_ref(self):
        L, log, objs = make_lidar(load_cell=True, **SMALL)
        L._svc = lambda req, timeout=30.0: {"ok": True, "reply": {}}
        r = dict((i, ok) for i, ok, t in L.setup_report())
        self.assertFalse(r["zero_ref"])
        L, log, objs = make_lidar(load_cell=True, zero_ref=(100.0, 100.0),
                                  **SMALL)
        L._svc = lambda req, timeout=30.0: {"ok": True, "reply": {}}
        r = dict((i, ok) for i, ok, t in L.setup_report())
        self.assertTrue(r["zero_ref"])

    def test_fine_x_follows_the_laser_side(self):
        L, log, objs = make_lidar(**K1)
        self.assertEqual(L._fine_x(3.0), (4.0, 284.0))
        L, log, objs = make_lidar(axis_min=(0.0, 0.0),
                                  axis_max=(230.0, 225.0), offset=(30.0, 20.0))
        self.assertEqual(L._fine_x(3.0), (16.5, 230.0))

    def test_print_tests_k1_defaults_kept(self):
        L, log, objs = make_lidar(**K1)
        g = Gcmd()
        self.assertEqual(L._pa_place(g, 9), (130.0, 100.0))
        self.assertEqual(L._creality_place(g), (170.0, 60.0))
        self.assertEqual(L._cross_place(g, 3.0, 5.0), (150.0, 150.0))

    def test_print_tests_small_bed_placed_inside(self):
        L, log, objs = make_lidar(**SMALL)
        g = Gcmd()
        x0, y0 = L._pa_place(g, 9)
        self.assertNotEqual((x0, y0), (130.0, 100.0))
        ox, oy = L._creality_place(g)
        self.fits(L, [(ox, oy), (ox + 27.0, oy + 139.0)])
        L, log, objs = make_lidar(axis_min=(0.0, 0.0),
                                  axis_max=(160.0, 160.0),
                                  offset=(-36.0, -21.2))
        self.assertEqual(L._cross_place(g, 3.0, 5.0), (62.0, 69.4))

    def test_print_tests_explicit_out_of_reach_refused(self):
        L, log, objs = make_lidar(**SMALL)
        with self.assertRaises(Err):
            L._pa_place(Gcmd(x=200, y=150), 9)
        with self.assertRaises(Err):
            L._creality_place(Gcmd(x=210, y=150))


if __name__ == "__main__":
    unittest.main()
