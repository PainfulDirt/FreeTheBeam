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


if __name__ == "__main__":
    unittest.main()
