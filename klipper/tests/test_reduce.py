"""lidar.reduce_caps: median per 0.05 mm bin (a stray point from ambient
light or gloss must not pull a bin)."""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import lidar  # noqa: E402


class ReduceTests(unittest.TestCase):
    def test_reduce_caps_median(self):
        b = 300                                   # a bin near the centre
        x = int((lidar.bin_center(b)) * 1000)
        good = [[x, 0, 25600], [x + 5, 0, 25602]]
        caps = [good, good, [[x + 10, 0, 27600]]]  # one stray, 2 mm off
        v = lidar.reduce_caps(caps)[b]
        self.assertAlmostEqual(v, 25.601, delta=0.0015)

    def test_reduce_caps_empty_bin_nan(self):
        v = lidar.reduce_caps([[[0, 0, 25000]]])
        self.assertTrue(v[0] != v[0])


if __name__ == "__main__":
    unittest.main()
