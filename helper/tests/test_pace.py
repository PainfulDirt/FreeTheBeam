"""Pacing of the heavy analysis (0.5.1): lidar._TICK, called once per
row/strip in mesh_fine, fl_analyze and group_row; the service sets it to a
Pacer (work ~20 ms, then rest ~20 ms) only with --profile printer, where the
analysis shares the printer's CPU with Klipper (local-mode measurements on
a K1 Max: 38 s at 50-55 % for the level 4/5 fit, 24 s for the first-layer
check; CPU down to 4-9 % idle). Results must not change."""
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, ".."), os.path.join(HERE, "..", "..",
                                                     "klipper")]
import lidar  # noqa: E402
import lidar_service  # noqa: E402


def fl_scans(objects, strips, ys, layer=0.2, z=3.2, x_dir=-1.0):
    """Synthetic before/after first-layer scans: a flat plate at height 0
    with `layer` mm of plastic inside each polygon; distances = z -
    surface, NGROUPS groups per row."""
    gx = [lidar.BIN_MIN + (g + 0.5) * lidar.GROUP * lidar.BIN
          for g in range(lidar.NGROUPS)]
    before, after = {}, {}
    for sx in strips:
        rb, ra = [], []
        for y in ys:
            b, a = [], []
            for g in range(lidar.NGROUPS):
                x = sx + x_dir * gx[g]
                h = layer if any(lidar._in_poly(x, y, p) for p in objects) \
                    else 0.0
                b.append(z)
                a.append(z - h)
            rb.append(b)
            ra.append(a)
        before[sx], after[sx] = rb, ra
    plan = {"strips": strips, "ys": ys, "x_dir": x_dir, "z_before": z,
            "z_after": z, "height": 0.2, "step": ys[1] - ys[0],
            "objects": [{"name": "o%d" % i, "polygon": p}
                        for i, p in enumerate(objects)]}
    return plan, before, after


def fine_strips():
    """Flat bed with a gentle bow, 6 strips of rows, full laser width."""
    ys = [25.0 + 2.0 * j for j in range(60)]
    gx = [lidar.BIN_MIN + (g + 0.5) * lidar.GROUP * lidar.BIN
          for g in range(lidar.NGROUPS)]
    strips = []
    for x in (40.0, 68.0, 96.0, 124.0, 152.0, 180.0):
        rows = [[3.0 - 0.0002 * (x - gx[g] - 110.0) - 0.0001 * (y - 85.0)
                 for g in range(lidar.NGROUPS)] for y in ys]
        strips.append((x, ys, rows))
    return strips


class Counter:
    def __init__(self):
        self.n = 0

    def __call__(self):
        self.n += 1


class TickHook(unittest.TestCase):
    def tearDown(self):
        lidar._TICK = None

    def test_default_off(self):
        self.assertIsNone(lidar._TICK)
        lidar._tick()                       # no-op

    def test_fl_analyze_ticks_same_result(self):
        objs = [[[100, 100], [140, 100], [140, 140], [100, 140]]]
        ys = [90.0 + 0.5 * j for j in range(110)]
        plan, b, a = fl_scans(objs, [110.0, 130.0], ys)
        ref, _ = lidar.fl_analyze(plan, b, a)
        c = Counter()
        lidar._TICK = c
        res, _ = lidar.fl_analyze(plan, b, a)
        self.assertGreater(c.n, 100)
        self.assertEqual(res.get("median"), ref.get("median"))

    def test_mesh_fine_ticks_same_result(self):
        xs = [40.0 + 28.0 * i for i in range(6)]
        ys = [30.0 + 20.0 * j for j in range(6)]
        ref = lidar.mesh_fine(fine_strips(), xs, ys)[0]
        c = Counter()
        lidar._TICK = c
        res = lidar.mesh_fine(fine_strips(), xs, ys)[0]
        self.assertGreater(c.n, 100)
        self.assertTrue(ref.get("matrix"))
        self.assertEqual(res["matrix"], ref["matrix"])

    def test_group_row_ticks(self):
        c = Counter()
        lidar._TICK = c
        lidar.group_row([3.0] * lidar.NBINS)
        self.assertEqual(c.n, 1)


class PacerTests(unittest.TestCase):
    def test_rests_after_work(self):
        clock = [0.0]
        slept = []
        p = lidar_service.Pacer(work=0.02, rest=0.02,
                                clock=lambda: clock[0],
                                sleep=lambda s: slept.append(s))
        p()                                  # just started: no rest
        clock[0] = 0.01
        p()
        self.assertEqual(slept, [])
        clock[0] = 0.025
        p()                                  # worked 25 ms: rest
        self.assertEqual(slept, [0.02])
        clock[0] = 0.03
        p()
        self.assertEqual(slept, [0.02])

    def test_profile_sets_the_hook(self):
        d = tempfile.mkdtemp()
        try:
            lidar_service.Service(None, d, profile="printer")
            self.assertIsInstance(lidar._TICK, lidar_service.Pacer)
            lidar_service.Service(None, d)               # a helper
            self.assertIsNone(lidar._TICK)
        finally:
            lidar._TICK = None
            shutil.rmtree(d, ignore_errors=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
