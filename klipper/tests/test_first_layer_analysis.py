"""Regression (K1 bug hunt 2026-11-16 #16, #17, #20): first-layer planning
and analysis (fl_plan, skew_plan, fl_analyze; the helper runs the same
code), on synthetic scans.

#16: a 9 mm bar whose solid cells all fall in one scan row gives no tilt
    (None, handled by the consumers) or a ~0 tilt, never a crash or a
    made-up tilt.
#20: the rounded-up last row stays inside the laser's reach (fl_plan and
    skew_plan).
#17: a full plate of 25 round parts is analysed within 10 s (the printer
    waits 60 s; a small helper computer is ~5x slower).
"""
import math
import time
import unittest

import fake_klipper  # noqa: F401  (klipper/ on sys.path)
import lidar as L

LO, HI = (6.0, -15.0), (240.0, 265.0)      # laser reach (x0, y0) / (x1, y1)


def make(objects, strips, ys, layer=0.2, z=3.2, x_dir=-1.0):
    """Synthetic before/after scans: a flat plate at height 0 with `layer`
    mm of plastic inside each polygon; distances = z - surface, NGROUPS
    groups per row."""
    gx = [L.BIN_MIN + (g + 0.5) * L.GROUP * L.BIN for g in range(L.NGROUPS)]
    before, after = {}, {}
    for sx in strips:
        rb, ra = [], []
        for y in ys:
            b, a = [], []
            for g in range(L.NGROUPS):
                x = sx + x_dir * gx[g]
                h = layer if any(L._in_poly(x, y, p) for p in objects) \
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


def bar_case(off, w=9.0):
    y0 = 145.0 + off
    bar = [(120.0, y0), (180.0, y0), (180.0, y0 + w), (120.0, y0 + w),
           (120.0, y0)]
    strips = [115.5 + off, 141.5 + off, 167.5 + off, 193.5 + off]
    ys = [round(131.0 + off + 2.0 * k, 3) for k in range(19)]
    plan, before, after = make([bar], strips, ys)
    return L.fl_analyze(plan, before, after)[0]


class SingleRowTilt(unittest.TestCase):
    def test_exact_rows(self):
        res = bar_case(0.0)
        self.assertTrue(res["ok"])
        tl = res["tilt_um_per_100mm"]
        if tl is not None:          # one row -> None (consumers handle it)
            tl[0], tl[1]            # else it must be a usable pair

    def test_inexact_rows(self):
        res = bar_case(0.13)
        self.assertTrue(res["ok"])
        tl = res["tilt_um_per_100mm"]
        # the synthetic layer is perfectly flat: tilt ~0, or None
        if tl is not None:
            self.assertLess(abs(tl[0]) + abs(tl[1]), 5.0,
                            "made-up tilt %r for a flat layer" % (tl,))


class RowsInReach(unittest.TestCase):
    def test_fl_plan_rows_stay_in_reach(self):
        # a part near the back: Y 230..262, margin 8 -> area clamped to 265
        part = [(200.0, 230.0), (240.0, 230.0), (240.0, 262.0),
                (200.0, 262.0)]
        plan = L.fl_plan([part], 8.0, LO, HI, 2.0)
        self.assertLessEqual(plan["y1"], HI[1], plan)

    def test_skew_plan_rows_stay_in_reach(self):
        plan = L.skew_plan((200.0, 230.0, 240.0, 262.0), 10.0, LO, HI, 2.0)
        self.assertLessEqual(plan["y1"], HI[1], plan)


class AnalyzeSpeed(unittest.TestCase):
    def test_full_plate_25_round_parts(self):
        objs = []
        for i in range(5):
            for j in range(5):
                cx, cy = 40 + 45 * i, 40 + 45 * j
                objs.append([(cx + 15 * math.cos(2 * math.pi * k / 32),
                              cy + 15 * math.sin(2 * math.pi * k / 32))
                             for k in range(32)])
        plan = L.fl_plan(objs, 8.0, (6.0, -15.0), (263.0, 265.0), 2.0)
        n = int(round((plan["y1"] - plan["y0"]) / plan["step"])) + 1
        ys = [round(plan["y0"] + plan["step"] * k, 3) for k in range(n)]
        p, b, a = make(objs, plan["strips"], ys)
        t = time.perf_counter()
        res, _ = L.fl_analyze(p, b, a)
        dt = time.perf_counter() - t
        self.assertTrue(res.get("ok"), res)
        self.assertLess(dt, 10.0, "fl_analyze %.1f s for %d strips x %d"
                        " rows" % (dt, len(plan["strips"]), n))


if __name__ == "__main__":
    unittest.main(verbosity=2)
