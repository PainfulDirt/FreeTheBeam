"""helper/lidar_bands.py: flow from 1-layer flatness bands, measured by the
camera (real photos of 2026-11-08 on a K1 Max, fixtures/camera/bands) and by the LiDAR
(synthetic before/after scans). Skipped without numpy/PIL."""
import math
import os
import sys
import unittest

try:
    import numpy as np
    import PIL  # noqa: F401
except ImportError:  # pragma: no cover (CI installs them)
    np = None

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, "fixtures", "camera", "bands")
sys.path.insert(0, os.path.join(HERE, ".."))
if np is not None:
    import lidar_bands as lb  # noqa: E402
    import lidar_camera as cam  # noqa: E402

FLOWS = (92, 96, 100, 104, 108, 112)
SP = 0.45 - 0.2 * (1 - math.pi / 4)      # line spacing of the bands (mm)


def photo(f, e=1000):
    with open(os.path.join(FIX, "band_%d_e%d.jpg" % (f, e)), "rb") as fh:
        return cam.load_gray(fh.read())


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class CameraBandTests(unittest.TestCase):
    def test_metrics_match_the_spike(self):
        m92, m104 = lb.band_metrics(photo(92)), lb.band_metrics(photo(104))
        self.assertAlmostEqual(m92["gaps"], 0.33, delta=0.01)
        self.assertAlmostEqual(m92["pattern"], 10.0, delta=0.1)
        self.assertAlmostEqual(m104["gaps"], 0.04, delta=0.01)
        self.assertAlmostEqual(m104["pattern"], 5.03, delta=0.1)

    def test_real_photos_pick_104(self):
        # at the exposure the rule was set at (at 2000 the 104 band's gaps
        # read 0.05x %: the rule sits on its edge there and picks 108)
        bands = [dict(lb.band_metrics(photo(f, lb.CAMERA_EXPOSURE)),
                      flow=f / 100.0) for f in FLOWS]
        r = lb.pick_camera(bands)
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertAlmostEqual(r["flow"], 1.04)
        self.assertAlmostEqual(r["f"], 1 / 1.04, places=4)
        self.assertEqual(len(r["report"]), len(FLOWS) + 1)

    def test_lowest_band_passing_not_saved(self):
        bands = [{"flow": f / 100.0, "gaps": 0.0, "pattern": 3.0} for f in FLOWS]
        r = lb.pick_camera(bands)
        self.assertFalse(r["ok"])
        self.assertIn("lowest", r["reason"])

    def test_no_band_passing_not_saved(self):
        bands = [{"flow": f / 100.0, "gaps": 1.0, "pattern": 9.0} for f in FLOWS]
        r = lb.pick_camera(bands)
        self.assertFalse(r["ok"])
        self.assertIn("no band", r["reason"])


def synth_rows(ys, x_strip, x_dir, band, h, ridge, seed, bare=False):
    """{y: points (N, 3) um} of one dwell strip: a tilted plate, noise, and
    (unless bare) a band of thickness h mm with ridges of amplitude
    ridge mm at the line spacing (the LiDAR reads distance: z = -height)."""
    rng = np.random.default_rng(seed)
    px = np.arange(-14000, 14000, 19.0)
    rows = {}
    for y in ys:
        x = x_strip + x_dir * px / 1000.0
        z = 0.04 * px / 1000.0 + 0.002 * (y - ys[0])
        if not bare:
            inb = (x >= band["x0"]) & (x <= band["x1"]) & \
                (band["y0"] <= y <= band["y1"])
            z = z + inb * (h + ridge * np.cos(2 * math.pi * (x - band["x0"]) / SP))
        z = z + rng.normal(0, 0.003, len(px))
        rows[y] = np.stack([px, np.zeros_like(px), -z * 1000.0], axis=1)
    return rows


@unittest.skipIf(np is None, "needs numpy and PIL (helper only)")
class LidarBandTests(unittest.TestCase):
    band = {"x0": 100.0, "x1": 114.0, "y0": 35.0, "y1": 45.0}
    ys = [round(33.0 + 0.4 * i, 3) for i in range(36)]

    def metrics(self, h, ridge, seed=1):
        before = synth_rows(self.ys, 107.0, -1.0, self.band, 0, 0, seed, bare=True)
        after = synth_rows(self.ys, 107.0, -1.0, self.band, h, ridge, seed + 100)
        return lb.lidar_band_metrics(before, after, 107.0, -1.0, self.band)

    def test_thickness_and_roughness(self):
        m = self.metrics(0.15, 0.010)
        self.assertAlmostEqual(m["thickness"], 0.15, delta=0.005)
        # cos of amplitude a: sd a / sqrt(2), plus the noise of two scans
        want = math.hypot(10.0 / math.sqrt(2), 3.0 * math.sqrt(2))
        self.assertAlmostEqual(m["roughness"], want, delta=1.5)
        self.assertLess(m["gaps"], 0.5)
        self.assertGreater(m["rows"], 15)

    def test_smoother_band_reads_smoother(self):
        self.assertLess(self.metrics(0.15, 0.002)["roughness"],
                        self.metrics(0.15, 0.012)["roughness"] - 3.0)

    def test_pick_relative_knee(self):
        rough = (20.0, 15.0, 10.0, 7.0, 6.0, 5.5)
        bands = [{"flow": f / 100.0, "roughness": r, "gaps": 0.0,
                  "thickness": 0.15} for f, r in zip(FLOWS, rough)]
        r = lb.pick_lidar(bands)
        self.assertTrue(r["ok"], r.get("reason"))
        self.assertAlmostEqual(r["flow"], 1.04)

    def test_pick_flat_refused(self):
        bands = [{"flow": f / 100.0, "roughness": 6.0 + 0.2 * i, "gaps": 0.0,
                  "thickness": 0.15} for i, f in enumerate(FLOWS)]
        r = lb.pick_lidar(bands)
        self.assertFalse(r["ok"])

    def test_pick_band_without_data_refused(self):
        bands = [{"flow": f / 100.0, "roughness": None, "gaps": None,
                  "thickness": None} for f in FLOWS]
        self.assertFalse(lb.pick_lidar(bands)["ok"])


if __name__ == "__main__":
    unittest.main()
