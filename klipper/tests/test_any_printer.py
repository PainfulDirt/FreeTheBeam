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


if __name__ == "__main__":
    unittest.main()
